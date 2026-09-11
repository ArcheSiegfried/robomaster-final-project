"""Task takeover coordinator: the only place that decides who drives the robot.

`main.py` produces one `FramePacket` per cycle and hands it to `step()`. The
coordinator decides whether the base line follower or one external task owns
the single motion outlet, and it is the only code that calls `MotionOutput`.

Ownership rules (frozen for the whole project):

* `MotionOutput` is the only motion outlet. Owner is `"line"` or `"external"`.
* At most one motion task owns `"external"` at any moment. Tasks are asked in
  `task_registry` order and the first one returning `RUNNING` wins.
* A takeover is only possible while the base line is armed: the follower must
  be `TRACKING`, `COASTING` or `LINE_LOST`. It can never happen from `STOPPED`
  (the operator has not started the line, or a fault was just reset) or from
  `VIDEO_LOST`. A module must never start the robot by itself.
* A task that returned `RUNNING` must keep returning `RUNNING` until it returns
  `COMPLETED` or `FAILED`. `NOT_TRIGGERED` while owning is treated as failure,
  because otherwise a task could silently keep control without commanding.
* Returning control always means: hard stop, `claim("line")`, clear line
  history, then wait for a fresh valid frame before resuming. If no fresh valid
  line arrives within `release_resume_timeout`, the robot stays stopped and a
  human must press SPACE.
* Human override always wins: `human_stop()` / `human_reset()` end any takeover
  immediately and require an explicit human resume.
* Every task `MotionCommand` is clamped into the `TaskConfig` envelope, and
  `nan` / `inf` become zero, so a module bug cannot command a runaway speed.

This module never touches a camera, the SDK or the network.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from config import RuntimeConfig
from models import (
    FramePacket,
    MotionCommand,
    RuntimeDecision,
    STOP_COMMAND,
    TaskStatus,
    TaskUpdate,
)
from runtime import COASTING, LINE_LOST, TRACKING, LineFollower

LINE_FOLLOWING = "LINE_FOLLOWING"
TASK_ACTIVE = "TASK_ACTIVE"
RELEASING = "RELEASING"

OWNER_LINE = "line"
OWNER_EXTERNAL = "external"

# The base line must be armed before any module may claim motion from it.
TAKEOVER_ALLOWED_STATES = (TRACKING, COASTING, LINE_LOST)


@dataclass(frozen=True)
class CoordinatorDecision:
    """What happened in one cycle, for the main loop, logging and tests."""

    state: str
    owner: str
    line: Optional[RuntimeDecision] = None
    task_name: Optional[str] = None
    task_update: Optional[TaskUpdate] = None
    command: MotionCommand = STOP_COMMAND
    force_stop: bool = False
    message: str = ""
    errors: Tuple[str, ...] = ()


def _finite(value) -> float:
    """Reject nan/inf/None from module code instead of forwarding it."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return number if math.isfinite(number) else 0.0


def _limit(value, low: float, high: float) -> float:
    return max(low, min(_finite(value), high))


def clamp_task_command(command: MotionCommand, settings) -> MotionCommand:
    """Bound a task request inside the TaskConfig safety envelope."""
    return MotionCommand(
        forward=_limit(command.forward, -settings.task_max_forward, settings.task_max_forward),
        lateral=_limit(command.lateral, -settings.task_max_lateral, settings.task_max_lateral),
        yaw=_limit(command.yaw, -settings.task_max_yaw, settings.task_max_yaw),
    )


class TaskCoordinator:
    def __init__(
        self,
        settings: RuntimeConfig,
        follower: LineFollower,
        output,
        motion_tasks: Sequence = (),
        observers: Sequence = (),
    ) -> None:
        self.settings = settings
        self.follower = follower
        self.output = output
        self.motion_tasks = tuple(motion_tasks)
        self.observers = tuple(observers)
        self.state = LINE_FOLLOWING
        self.last_line_decision: Optional[RuntimeDecision] = None
        self.active_task = None
        self.active_task_name: Optional[str] = None
        self._active_started: Optional[float] = None
        self._release_started: Optional[float] = None

    # -- introspection -------------------------------------------------
    @property
    def takeover_allowed(self) -> bool:
        return self.follower.state in TAKEOVER_ALLOWED_STATES

    @property
    def task_active(self) -> bool:
        return self.active_task is not None

    # -- helpers -------------------------------------------------------
    def _call(self, function, label: str, frame, now, errors):
        """Call one module hook, containing its time and its exceptions."""
        started = time.monotonic()
        try:
            result = function(frame, now)
        except Exception as error:  # a module bug must not break the loop
            errors.append(f"{label} raised an exception: {error}")
            return None
        elapsed = time.monotonic() - started
        if elapsed > self.settings.tasks.max_step_seconds:
            errors.append(
                f"{label} step was slow: {elapsed:.3f}s "
                f"(limit {self.settings.tasks.max_step_seconds:.3f}s)"
            )
        return result

    def _observe(self, frame, now, errors) -> None:
        for observer in self.observers:
            self._call(observer.observe, observer.name, frame, now, errors)

    def _find_takeover(self, frame, now, errors):
        for task in self.motion_tasks:
            update = self._call(task.step, task.name, frame, now, errors)
            if update is None:
                continue
            if update.status is TaskStatus.RUNNING:
                return task, update
        return None

    def _apply_task_motion(self, update: TaskUpdate, errors) -> MotionCommand:
        if update.motion is None:
            self.output.hard_stop()
            return STOP_COMMAND
        command = clamp_task_command(update.motion, self.settings.tasks)
        if (
            command.forward != update.motion.forward
            or command.lateral != update.motion.lateral
            or command.yaw != update.motion.yaw
        ):
            errors.append(
                f"task command clamped from "
                f"({update.motion.forward}, {update.motion.lateral}, {update.motion.yaw}) "
                f"to ({command.forward}, {command.lateral}, {command.yaw})"
            )
        self.output.send(OWNER_EXTERNAL, command)
        return command

    # -- per-cycle entry point -----------------------------------------
    def step(self, frame: FramePacket, now: float) -> CoordinatorDecision:
        errors = []
        self._observe(frame, now, errors)

        if self.active_task is not None:
            return self._step_active(frame, now, errors)
        if self.state == RELEASING:
            return self._step_releasing(frame, now, errors)
        return self._step_line(frame, now, errors)

    def _step_line(self, frame, now, errors) -> CoordinatorDecision:
        # Process the frame first so the base state machine stays current, but
        # hold its command until we know no task is taking over this cycle.
        decision = self.follower.process_frame(frame.image, frame.captured_at)
        self.last_line_decision = decision

        if self.takeover_allowed:
            takeover = self._find_takeover(frame, now, errors)
            if takeover is not None:
                task, update = takeover
                return self._begin_takeover(task, update, now, errors)

        if decision.force_stop:
            self.output.hard_stop()
        elif self.follower.motion_enabled:
            self.output.send(OWNER_LINE, decision.command)
        return CoordinatorDecision(
            state=self.state,
            owner=self.output.owner,
            line=decision,
            command=decision.command,
            force_stop=decision.force_stop,
            message=decision.message,
            errors=tuple(errors),
        )

    def _begin_takeover(self, task, update, now, errors) -> CoordinatorDecision:
        self.follower.pause(now)
        self.output.claim(OWNER_EXTERNAL)
        self.active_task = task
        self.active_task_name = task.name
        self._active_started = now
        self.state = TASK_ACTIVE
        command = self._apply_task_motion(update, errors)
        return CoordinatorDecision(
            state=TASK_ACTIVE,
            owner=self.output.owner,
            line=self.last_line_decision,
            task_name=task.name,
            task_update=update,
            command=command,
            message="task took over",
            errors=tuple(errors),
        )

    def _step_active(self, frame, now, errors) -> CoordinatorDecision:
        task = self.active_task
        if now - self._active_started > self.settings.tasks.max_task_seconds:
            errors.append(
                f"{task.name} exceeded max_task_seconds "
                f"({self.settings.tasks.max_task_seconds:.1f}s)"
            )
            return self._release(now, errors, "task timeout")

        update = self._call(task.step, task.name, frame, now, errors)
        if update is None:
            return self._release(now, errors, "task raised an exception")
        if update.status is TaskStatus.RUNNING:
            command = self._apply_task_motion(update, errors)
            return CoordinatorDecision(
                state=TASK_ACTIVE,
                owner=self.output.owner,
                line=self.last_line_decision,
                task_name=task.name,
                task_update=update,
                command=command,
                message=update.message,
                errors=tuple(errors),
            )
        if update.status is TaskStatus.NOT_TRIGGERED:
            errors.append(
                f"{task.name} returned NOT_TRIGGERED while owning motion; "
                f"a task must keep returning RUNNING until it completes or fails"
            )
            return self._release(now, errors, "task gave up while owning motion")
        return self._release(
            now, errors, f"task {update.status.value}", update=update
        )

    def _release(self, now, errors, message, update=None) -> CoordinatorDecision:
        name = self.active_task_name
        self.output.hard_stop()
        self.output.claim(OWNER_LINE)
        self.follower.reset_fault(now)
        self.active_task = None
        self.active_task_name = None
        self._active_started = None
        self._release_started = now
        self.state = RELEASING
        return CoordinatorDecision(
            state=RELEASING,
            owner=self.output.owner,
            line=self.last_line_decision,
            task_name=name,
            task_update=update,
            command=STOP_COMMAND,
            force_stop=True,
            message=message,
            errors=tuple(errors),
        )

    def _step_releasing(self, frame, now, errors) -> CoordinatorDecision:
        decision = self.follower.process_frame(frame.image, frame.captured_at)
        self.last_line_decision = decision

        if self.follower.resume(frame.captured_at):
            self._release_started = None
            self.state = LINE_FOLLOWING
            return CoordinatorDecision(
                state=LINE_FOLLOWING,
                owner=self.output.owner,
                line=decision,
                command=STOP_COMMAND,
                message="resumed on a fresh valid line",
                errors=tuple(errors),
            )

        if now - self._release_started > self.settings.tasks.release_resume_timeout:
            self._release_started = None
            self.state = LINE_FOLLOWING
            self.output.hard_stop()
            return CoordinatorDecision(
                state=LINE_FOLLOWING,
                owner=self.output.owner,
                line=decision,
                command=STOP_COMMAND,
                force_stop=True,
                message="auto-resume timed out; show a fresh line and press SPACE",
                errors=tuple(errors),
            )

        return CoordinatorDecision(
            state=RELEASING,
            owner=self.output.owner,
            line=decision,
            command=STOP_COMMAND,
            message="waiting for a fresh valid line before resuming",
            errors=tuple(errors),
        )

    # -- video gap -----------------------------------------------------
    def video_gap(self, frame_age: float, now: float) -> CoordinatorDecision:
        """No new camera frame. Never let a task keep driving through it."""
        errors = []
        message = ""
        if self.active_task is not None:
            message = f"{self.active_task_name} ended by video gap"
            self._end_takeover(now)
        decision = self.follower.process_video_gap(frame_age, now)
        self.last_line_decision = decision
        if decision.force_stop:
            self.output.hard_stop()
        return CoordinatorDecision(
            state=self.state,
            owner=self.output.owner,
            line=decision,
            command=STOP_COMMAND,
            force_stop=decision.force_stop,
            message=message or decision.message,
            errors=tuple(errors),
        )

    # -- human override -------------------------------------------------
    def _end_takeover(self, now: float) -> Optional[str]:
        if self.active_task is None:
            return None
        name = self.active_task_name
        self.output.hard_stop()
        self.output.claim(OWNER_LINE)
        self.follower.reset_fault(now)
        self.active_task = None
        self.active_task_name = None
        self._active_started = None
        self._release_started = None
        self.state = LINE_FOLLOWING
        return name

    def human_stop(self, now: float) -> str:
        """SPACE while a task runs, or the loop is unsafe: stop everything."""
        ended = self._end_takeover(now)
        self.follower.pause(now)
        self.output.hard_stop()
        self._release_started = None
        self.state = LINE_FOLLOWING
        if ended is not None:
            return f"Stopped; task {ended} ended. Show a fresh line, then press SPACE."
        return "Paused."

    def human_reset(self, now: float) -> str:
        ended = self._end_takeover(now)
        self.follower.reset_fault(now)
        self.output.hard_stop()
        self._release_started = None
        self.state = LINE_FOLLOWING
        if ended is not None:
            return f"Reset; task {ended} ended. Show a valid line, then press SPACE."
        return "Reset: stopped; show a valid line, then press SPACE."

    def human_resume(self, now: float) -> bool:
        if self.active_task is not None:
            return False
        if self.state != LINE_FOLLOWING:
            return False
        self._release_started = None
        return bool(self.follower.resume(now))

    def close(self) -> None:
        """Release observer resources (open files, queued rows).

        Called once from the main loop's cleanup path. Never raises: a recording
        module must not be able to break shutdown or the safety stop.
        """
        for observer in self.observers:
            closer = getattr(observer, "close", None)
            if callable(closer):
                try:
                    closer()
                except Exception:
                    pass
