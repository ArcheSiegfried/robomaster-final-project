"""Task takeover coordinator: the only place that decides who drives the robot.

`main.py` produces one `FramePacket` per cycle and hands it to `step()`. The
coordinator decides whether the base line follower or one external task owns
the single motion outlet, and it is the only code that calls `MotionOutput`.

Ownership rules (v0.3, 2026-09-18 — arbitration layer):

* `MotionOutput` is the only motion outlet. Owner is `"line"` or `"external"`.
* Every module only *requests* motion by returning `RUNNING`; `ControlArbiter`
  (`control_arbiter.py`) picks the single winner for the frame out of the
  collected `ControlRequest`s. Tasks are asked in **priority** order
  (`task_registry.TASK_PRIORITIES`, descending) and, when nobody owns motion,
  the first `RUNNING` one wins — which is the highest-priority claimant.
* Two owners are tracked separately and must not be conflated:
  `active_task` (the task holding the logical takeover) and `motion_owner`
  (who may command the chassis *this frame*). While a **safety-priority**
  module (priority >= `SAFETY_PRIORITY`, i.e. the red light) claims control,
  the task owner is frozen — it is not stepped and its motion is not sent —
  and it resumes from its own state once the light clears.
* A request carries a `ttl` (lease). A module renews it by returning `RUNNING`
  on a frame it is asked; a stale request is dropped and the frame falls back
  to whoever else is valid, or to a hard stop when nobody is.
* Non-safety preemption is **off by default** (`preempt_all=False`): a module
  can only take control from another task when the arbiter is configured to
  allow it. This keeps the current, field-confirmed takeover order intact.
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

from control_arbiter import (
    LINE_OWNER,
    LINE_PRIORITY,
    SAFETY_PRIORITY,
    ControlArbiter,
    ControlRequest,
)
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
from task_registry import priority_of

LINE_FOLLOWING = "LINE_FOLLOWING"
TASK_ACTIVE = "TASK_ACTIVE"
RELEASING = "RELEASING"

OWNER_LINE = "line"
OWNER_EXTERNAL = "external"

#: 红绿灯模块的注册名。v0.3 起"红灯否决"由通用机制实现（安全级模块每帧都被问，
#: 见 `_ask_list`），这个常量保留给测试与诊断（"红灯否决需要一个可问的模块"）。
LIGHT_TASK_NAME = "traffic_light"

# The base line must be armed before any module may claim motion from it.
TAKEOVER_ALLOWED_STATES = (TRACKING, COASTING, LINE_LOST)

#: 连续这么多帧单步耗时都超 `max_step_seconds`，就认定这个模块卡住了并强制释放。
#: 今天只记一条 error 不处理（模块卡死全靠 SDK 的 0.15s 命令超时兜底）。
SLOW_STEP_FRAMES = 5


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
    #: 控制权变更时的终端日志行（`[ARB] owner changed: ...`）；没换人就是 None。
    owner_change: Optional[str] = None
    #: 探测帧的"竞争实况"：每个被问到的模块各自想不想接管。
    claims: tuple = ()


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
        gimbal_output=None,
    ) -> None:
        self.settings = settings
        self.follower = follower
        self.output = output
        self.motion_tasks = tuple(motion_tasks)
        self.observers = tuple(observers)
        self.gimbal_output = gimbal_output
        self.state = LINE_FOLLOWING
        self.last_line_decision: Optional[RuntimeDecision] = None
        self.active_task = None
        self.active_task_name: Optional[str] = None
        self._active_started: Optional[float] = None
        self._release_started: Optional[float] = None
        # 红灯否决在 v0.3 里**不再是特判**：`_ask_list()` 让安全级（priority >=
        # SAFETY_PRIORITY）模块在"有任务接管"时也每帧被问，而 `_collect()` 一旦
        # 拿到安全级请求就短路，owner 那一帧根本不会被调用 —— 行为与老的红灯否决
        # 等价（实测：障碍接管期间红灯亮着，车仍以 forward=0.2 在走，就是没有它）。
        #: 被红灯暂停掉的累计时间：等红灯不该算进任务的 max_task_seconds。
        self._paused_seconds = 0.0
        #: 最近一次"竞争探测"的结果（只有探测帧非空）。给终端显示用：
        #: 那一刻**每个**模块各自想不想接管，以及最后判给了谁。
        self.last_claims: tuple = ()
        self._last_claim_probe_at: Optional[float] = None
        self._previous_frame_time: Optional[float] = None
        self._view_changed = False
        self._view_ready_at: Optional[float] = None
        self._view_restore_failed = False
        # -- 仲裁层（v0.3）---------------------------------------------
        # 仲裁器只决定"这一帧谁能下发运动指令"。任务的生命周期（active_task、
        # 20s 超时、释放握手）仍然由本协调器负责 —— 两者分开是刻意的：
        # task_owner 可以存在而 motion_owner 是别人（红灯把任务冻结住就是这种情形）。
        self.arbiter = ControlArbiter(
            safety_priority=SAFETY_PRIORITY,
            preempt_margin=int(getattr(settings.tasks, "preempt_margin", 0)),
            min_hold_seconds=float(getattr(settings.tasks, "min_hold_seconds", 0.2)),
            preempt_all=bool(getattr(settings.tasks, "preempt_all", False)),
        )
        #: 这一帧真正能下发运动指令的人："line" 或模块名。
        self.motion_owner = LINE_OWNER
        self._owner_priority = LINE_PRIORITY
        self._motion_owner_since: Optional[float] = None
        #: 本帧每个被问到的模块的 TaskUpdate（None = 抛异常/没被问）。
        self._last_step_updates = {}
        #: 每个模块"连续多少帧单步超预算"（超 SLOW_STEP_FRAMES 就强制释放）。
        self._slow_frames = {}
        # -- 任务时钟（v0.3）-------------------------------------------
        # 每个模块的 `now` **只累计它真正被调用的时段**。被跳过的帧（它不是 owner，
        # 也不是安全级）与被打断的帧（红灯把它冻结住）一律不计入 —— 否则模块会在
        # "它根本没在跑"的时候被自己的墙钟超时判死（实车可复现：红灯把正在转向的
        # 模块暂停 8 秒，它下一次被调用就 turn timeout）。
        self._task_clock = {}
        self._last_called = {}
        self._called_last_frame = set()
        self._called_this_frame = set()

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
            self._slow_frames[label] = self._slow_frames.get(label, 0) + 1
        else:
            self._slow_frames[label] = 0
        return result

    def _observe(self, frame, now, errors) -> None:
        for observer in self.observers:
            self._call(observer.observe, observer.name, frame, now, errors)

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

    def _apply_task_gimbal(self, update: TaskUpdate, errors) -> bool:
        if update.gimbal is None:
            return True
        if self.gimbal_output is None:
            errors.append("task requested gimbal motion but no gimbal outlet is configured")
            return False
        try:
            applied = self.gimbal_output.send(update.gimbal)
        except Exception as error:
            errors.append(f"gimbal request failed: {error}")
            return False
        self._view_changed = applied != self.gimbal_output.line_view
        self._view_restore_failed = False
        return True

    @staticmethod
    def _reset_task(task, errors) -> None:
        """Best-effort lifecycle reset for an ended stateful task.

        The hook is optional, so existing modules keep the frozen v0.2 step
        contract.  Human stop, video loss and coordinator timeouts must not
        leave a task in its old RUNNING state ready to retake control later.
        """
        reset = getattr(task, "reset", None)
        if not callable(reset):
            return
        try:
            reset()
        except Exception as error:
            errors.append(f"{task.name} reset failed: {error}")

    def _restore_line_view(self, now: float, errors, force: bool = False) -> bool:
        if not force and not self._view_changed and not self._view_restore_failed:
            self._view_ready_at = None
            return True
        if self.gimbal_output is None:
            errors.append("cannot restore line view: no gimbal outlet is configured")
            self._view_restore_failed = True
            self._view_ready_at = None
            return False
        try:
            self.gimbal_output.restore_line_view()
        except Exception as error:
            errors.append(f"cannot restore line view: {error}")
            self._view_restore_failed = True
            self._view_ready_at = None
            return False
        # 【v0.3 接口】出口可能只是把"回巡线视角"排进队列（上一个绝对动作还没跑完），
        # 这时**不能**当成已恢复：保持"意图"，由每帧的 _poll_gimbal 补发，等出口自己
        # 报告 at_line_view 之后才开始 settle 计时。老出口没有 at_line_view -> 视为已到位。
        if not self._gimbal_at_line_view():
            self._view_changed = True
            self._view_ready_at = None
            return True
        self._view_changed = False
        self._view_restore_failed = False
        self._view_ready_at = now + self.settings.gimbal_settle_seconds
        return True

    def _gimbal_at_line_view(self) -> bool:
        """出口是否真的停在巡线视角。老出口（无该属性）视为已到位，保持旧行为。"""
        value = getattr(self.gimbal_output, "at_line_view", None)
        if value is None:
            return True
        try:
            return bool(value)
        except Exception:
            return True

    def _poll_gimbal(self, errors) -> None:
        """每帧推进一次云台出口（收割在飞动作 + 补发队列里的最新目标）。

        【v0.3 接口】这一步必须**每帧、在任何状态分支之前**发生一次：出口在忙时只
        排队不发送，如果没有周期性的 poll，队列里的"回巡线视角"就永远补发不出去，
        车会一直停在 "waiting for gimbal to return to line view"（实车已复现）。
        老出口没有 poll -> 什么都不做，行为与今天一致。绝不抛异常。
        """
        poll = getattr(self.gimbal_output, "poll", None)
        if not callable(poll):
            return
        try:
            poll()
        except Exception as error:
            errors.append(f"gimbal poll failed: {error}")

    # -- 仲裁辅助 --------------------------------------------------------
    def _priority(self, task) -> int:
        """任务优先级：类属性优先，否则查注册表，再否则按巡线处理。"""
        value = getattr(task, "priority", None)
        if isinstance(value, bool) or not isinstance(value, int):
            return priority_of(task)
        return int(value)

    def _is_safety(self, task) -> bool:
        return self._priority(task) >= self.arbiter.safety_priority

    def _ranked_tasks(self):
        return sorted(
            self.motion_tasks,
            key=lambda task: (-self._priority(task), self.motion_tasks.index(task)),
        )

    def _ask_list(self, forced_full: bool) -> tuple:
        """这一帧有资格说话的模块。

        * **无人接管**（巡线在开车）：按优先级顺序全部问，遇到第一个 RUNNING 就停
          （`forced_full` 的探测帧问完所有人，只为终端显示竞争实况）。
        * **有任务在接管**：只问**安全级**模块 + 当前任务。安全级必须每帧被问，
          这是"红灯必须能停住正在开车的模块"的通用做法（替代原来的硬编码特判）。
        * 打开 `preempt_all` 时，再补上优先级严格更高的模块。
        """
        ranked = self._ranked_tasks()
        if self.active_task is None or forced_full:
            return tuple(ranked)
        asked = [task for task in ranked if self._is_safety(task)]
        if self.active_task not in asked:
            asked.append(self.active_task)
        if self.arbiter.preempt_all:
            for task in ranked:
                if self._priority(task) > self._owner_priority and task not in asked:
                    asked.append(task)
        return tuple(asked)

    def _collect(self, frame, now, errors, forced_full: bool = False):
        """按 ask 顺序调 step，收集 RUNNING 的模块 → [(request, task, update), ...]。"""
        collected = []
        for task in self._ask_list(forced_full):
            task_now = self._task_now(task, now)
            self._called_this_frame.add(task.name)
            update = self._call(task.step, task.name, frame, task_now, errors)
            self._last_step_updates[task.name] = update
            if update is None or update.status is not TaskStatus.RUNNING:
                continue
            collected.append(
                (
                    ControlRequest(
                        module=task.name,
                        priority=self._priority(task),
                        command=update.motion,
                        ttl=float(getattr(self.settings.tasks, "lease_seconds", 0.5)),
                        timestamp=now,
                        state=str(update.message or ""),
                        order=self.motion_tasks.index(task),
                    ),
                    task,
                    update,
                )
            )
            if forced_full:
                continue
            if self._is_safety(task):
                # 安全级赢家已定：后面的模块（含当前 owner）这一帧不再被调，
                # 这样"红灯期间任务真的被停住"而不是继续发速度。
                break
            if self.active_task is None:
                # 无人接管：按优先级顺序扫，第一个 RUNNING 就是最高优先级。
                break
        return collected

    @staticmethod
    def _pick(collected, request):
        for item in collected:
            if item[0] is request:
                return item[1], item[2]
        return None, None

    def _claim_rows(self, collected, asked) -> tuple:
        """终端"竞争实况"：本帧每个被问到的模块各自想不想接管。"""
        claimed = {item[1].name: item for item in collected}
        rows = []
        for task in asked:
            item = claimed.get(task.name)
            update = self._last_step_updates.get(task.name)
            if item is not None:
                status = "RUNNING"
            elif update is None:
                status = "ERROR"
            else:
                status = update.status.name
            rows.append(
                {
                    "name": task.name,
                    "priority": self._priority(task),
                    "status": status,
                    "message": "" if update is None else str(update.message or ""),
                }
            )
        return tuple(rows)

    def _note_owner(self, name: str, priority: int, now: float) -> None:
        self.motion_owner = name
        self._owner_priority = priority
        self._motion_owner_since = now

    def _task_now(self, task, real_now: float) -> float:
        """给模块的 `now`：只在"上一帧也被调用过"时累加真实时间差。

        这样被跳过/被冻结的时段不进它的时间线；连续被调用的正常情况与真实时钟一致。
        只可能 ≤ 真实 `now`（因为它只累加真实差值），不会倒退。
        """
        current = self._task_clock.get(task.name)
        if current is None:
            current = real_now
        elif task.name in self._called_last_frame:
            current = current + (real_now - self._last_called[task.name])
        self._task_clock[task.name] = current
        self._last_called[task.name] = real_now
        return current

    def _owner_change_text(self, result, previous: str, previous_priority: int):
        """把仲裁结果翻译成一行 `[ARB] ...`；没换人就返回 None。"""
        if result is None or result.request is None or result.owner == previous:
            return None
        return self.arbiter.change_text(previous, result, previous_priority)

    def _safety_winner(self, collected, task):
        for request, candidate, update in collected:
            if candidate is not task and self._is_safety(candidate):
                return candidate, update, request
        return None, None, None

    # -- per-cycle entry point -----------------------------------------
    def step(self, frame: FramePacket, now: float) -> CoordinatorDecision:
        errors = []
        delta = 0.0 if self._previous_frame_time is None else max(
            0.0, now - self._previous_frame_time)
        self._previous_frame_time = now
        # 任务时钟的分帧边界：本帧被调用过的模块，下一帧才允许继续累加时间。
        self._called_last_frame = self._called_this_frame
        self._called_this_frame = set()
        self._poll_gimbal(errors)
        self._observe(frame, now, errors)

        if self.active_task is not None:
            return self._step_owned(frame, now, errors, delta)
        if self.state == RELEASING:
            return self._step_releasing(frame, now, errors)
        probe = self._claim_probe_due(now)
        if probe:
            self._last_claim_probe_at = now
        return self._step_line(frame, now, errors, probe=probe)

    def _claim_probe_due(self, now: float) -> bool:
        """该不该做一次"竞争探测"（把所有模块都问一遍并记录各自的想法）。

        正常情况下协调器遇到第一个 RUNNING 就停，后面的模块**根本不会被问**，
        操作员看不到"还有谁想接管、为什么判给它"。这个探测帧就是为调优先级准备的：
        每 `claim_probe_seconds` 秒一次（默认 3 秒，设 0 关闭），**胜负规则不变**
        （仍是优先级最高的那个）；代价只是那一帧会让未接管的模块多推进一次
        内部计数，所以频率低、并且可以关。
        """
        interval = float(getattr(self.settings.tasks, "claim_probe_seconds", 0.0) or 0.0)
        if interval <= 0.0:
            return False
        if self._last_claim_probe_at is None:
            # 第一帧只记时、不探测：保持"正常帧只问到自己为止"的既有契约，
            # 第一次竞争实况在 interval 秒之后出现。
            self._last_claim_probe_at = now
            return False
        return now - self._last_claim_probe_at >= interval

    def _step_line(self, frame, now, errors, probe: bool = False) -> CoordinatorDecision:
        # Process the frame first so the base state machine stays current, but
        # hold its command until we know no task is taking over this cycle.
        decision = self.follower.process_frame(frame.image, frame.captured_at)
        self.last_line_decision = decision
        self.last_claims = ()
        self._last_step_updates = {}

        if self.takeover_allowed:
            asked = self._ask_list(probe)
            collected = self._collect(frame, now, errors, forced_full=probe)
            if probe:
                self.last_claims = self._claim_rows(collected, asked)
            result = self.arbiter.select(
                [item[0] for item in collected],
                now,
                current_owner=LINE_OWNER,
                current_owner_since=self._motion_owner_since,
            )
            if result.request is not None:
                task, update = self._pick(collected, result.request)
                if task is not None:
                    return self._begin_takeover(task, update, now, errors, result)

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

    def _begin_takeover(self, task, update, now, errors, result=None) -> CoordinatorDecision:
        previous = self.motion_owner
        previous_priority = self._owner_priority
        self.follower.pause(now)
        self.output.claim(OWNER_EXTERNAL)
        self.active_task = task
        self.active_task_name = task.name
        self._active_started = now
        self._paused_seconds = 0.0
        self.state = TASK_ACTIVE
        self._note_owner(task.name, self._priority(task), now)
        if not self._apply_task_gimbal(update, errors):
            return self._release(
                now, errors, "task gimbal request failed", reset_task=True
            )
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
            owner_change=self._owner_change_text(result, previous, previous_priority),
            claims=self.last_claims,
        )

    def _step_owned(self, frame, now, errors, delta: float = 0.0) -> CoordinatorDecision:
        """有任务在"逻辑上"持有任务：本帧唯一的运动指令由仲裁器决定。"""
        task = self.active_task
        # 有人在开车时不再报"竞争"（这一帧只有它被问，报出来会误导操作员）。
        self.last_claims = ()
        self._last_step_updates = {}
        elapsed = now - self._active_started - self._paused_seconds
        if elapsed > self.settings.tasks.max_task_seconds:
            errors.append(
                f"{task.name} exceeded max_task_seconds "
                f"({self.settings.tasks.max_task_seconds:.1f}s)"
            )
            return self._release(now, errors, "task timeout", reset_task=True)

        collected = self._collect(frame, now, errors, forced_full=False)
        owner_update = self._last_step_updates.get(task.name)

        # 情况 0：它连续多帧单步超预算 —— 判定卡住，主动夺权（不再只记一条 error）。
        if self._slow_frames.get(task.name, 0) >= SLOW_STEP_FRAMES:
            errors.append(
                f"{task.name} exceeded the per-step budget for "
                f"{SLOW_STEP_FRAMES} consecutive frames"
            )
            return self._release(
                now, errors, "task exceeded the per-step budget", reset_task=True
            )

        # 情况 1：当前任务这一帧没被调用（被安全级短路），或它抛异常了。
        if owner_update is None:
            rival, rival_update, _request = self._safety_winner(collected, task)
            if rival is None:
                return self._release(
                    now, errors, "task raised an exception", reset_task=True
                )
            return self._safety_veto(task, rival, rival_update, collected, now, delta, errors)

        # 情况 2：接管中却说自己"没触发" —— 违约，必须释放（契约不变）。
        if owner_update.status is TaskStatus.NOT_TRIGGERED:
            errors.append(
                f"{task.name} returned NOT_TRIGGERED while owning motion; "
                f"a task must keep returning RUNNING until it completes or fails"
            )
            return self._release(
                now, errors, "task gave up while owning motion", reset_task=True
            )

        # 情况 3：正常终态。
        if owner_update.status is not TaskStatus.RUNNING:
            return self._release(
                now, errors, f"task {owner_update.status.value}", update=owner_update
            )

        # 情况 4：它还在 RUNNING —— 但仲裁器可能判给别人（安全级或 preempt_all 抢占）。
        result = self.arbiter.select(
            [item[0] for item in collected],
            now,
            current_owner=self.motion_owner,
            current_owner_since=self._motion_owner_since,
        )
        winner, winner_update = self._pick(collected, result.request) if result.request else (None, None)
        if winner is not None and winner is not task:
            if self._is_safety(winner):
                return self._safety_veto(task, winner, winner_update, collected, now, delta, errors)
            return self._hand_off(task, winner, winner_update, now, errors, result)

        # 情况 5：当前任务继续开车（也可能刚从否决/抢占里恢复）。
        previous = self.motion_owner
        previous_priority = self._owner_priority
        if previous != task.name:
            self._note_owner(task.name, self._priority(task), now)
        if not self._apply_task_gimbal(owner_update, errors):
            return self._release(
                now, errors, "task gimbal request failed", reset_task=True
            )
        command = self._apply_task_motion(owner_update, errors)
        return CoordinatorDecision(
            state=TASK_ACTIVE,
            owner=self.output.owner,
            line=self.last_line_decision,
            task_name=task.name,
            task_update=owner_update,
            command=command,
            message=owner_update.message,
            errors=tuple(errors),
            owner_change=self._owner_change_text(result, previous, previous_priority),
        )

    def _safety_veto(self, task, rival, rival_update, collected, now, delta, errors):
        """安全级（红灯）接管：**冻结**当前任务，只下发安全级要求的指令。

        不做释放握手 —— 红灯不是故障，不需要人来按 SPACE；等灯的时间也不计入任务的
        超时预算。当前任务保持 active（状态不丢），灯一消失就继续从原状态跑。
        """
        previous = self.motion_owner
        previous_priority = self._owner_priority
        result = self.arbiter.select(
            [item[0] for item in collected],
            now,
            current_owner=previous,
            current_owner_since=self._motion_owner_since,
        )
        self._paused_seconds += delta
        self._note_owner(rival.name, self._priority(rival), now)
        command = self._apply_task_motion(rival_update, errors)
        return CoordinatorDecision(
            state=TASK_ACTIVE,
            owner=self.output.owner,
            line=self.last_line_decision,
            task_name=task.name,
            task_update=rival_update,
            command=command,
            message=f"red light veto: {task.name} paused",
            errors=tuple(errors),
            owner_change=self._owner_change_text(result, previous, previous_priority),
        )

    def _hand_off(self, task, winner, winner_update, now, errors, result):
        """非安全级抢占（只在 `preempt_all=True` 时发生）。

        被抢的任务**不 reset、不丢状态**，只是暂时不再被调用；新赢家立刻接管。
        释放握手不参与（那不是故障），motion_owner 直接换人。
        """
        previous = self.motion_owner
        previous_priority = self._owner_priority
        self.active_task = winner
        self.active_task_name = winner.name
        self._active_started = now
        self._paused_seconds = 0.0
        self._note_owner(winner.name, self._priority(winner), now)
        if not self._apply_task_gimbal(winner_update, errors):
            return self._release(
                now, errors, "task gimbal request failed", reset_task=True
            )
        command = self._apply_task_motion(winner_update, errors)
        return CoordinatorDecision(
            state=TASK_ACTIVE,
            owner=self.output.owner,
            line=self.last_line_decision,
            task_name=winner.name,
            task_update=winner_update,
            command=command,
            message="task took over from %s" % previous,
            errors=tuple(errors),
            owner_change=self._owner_change_text(result, previous, previous_priority),
        )

    def _release(
        self,
        now,
        errors,
        message,
        update=None,
        reset_task: bool = False,
    ) -> CoordinatorDecision:
        name = self.active_task_name
        task = self.active_task
        self.output.hard_stop()
        self.output.claim(OWNER_LINE)
        self.follower.reset_fault(now)
        # A normal terminal update may deliberately retain de-duplication or
        # cooldown state.  Forced endings cannot safely retain RUNNING state.
        if reset_task and task is not None:
            self._reset_task(task, errors)
        self.active_task = None
        self.active_task_name = None
        self._active_started = None
        self._paused_seconds = 0.0
        self._release_started = now
        self._note_owner(LINE_OWNER, LINE_PRIORITY, now)
        self._restore_line_view(now, errors)
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
        if self._view_restore_failed:
            self.output.hard_stop()
            if now - self._release_started > self.settings.tasks.release_resume_timeout:
                self._release_started = None
                self.state = LINE_FOLLOWING
                return CoordinatorDecision(
                    state=LINE_FOLLOWING,
                    owner=self.output.owner,
                    line=self.last_line_decision,
                    command=STOP_COMMAND,
                    force_stop=True,
                    message="line-view restore failed; reset and resume required",
                    errors=tuple(errors),
                )
            return CoordinatorDecision(
                state=RELEASING,
                owner=self.output.owner,
                line=self.last_line_decision,
                command=STOP_COMMAND,
                force_stop=True,
                message="waiting for line-view restore",
                errors=tuple(errors),
            )

        # 【v0.3 接口】释放期间只做两件事，**不要每帧重发 restore**（那会在
        # "已到位"和"重新发一次"之间无限循环，出口永远显示忙）：
        #   1. 恢复请求在 _end_task 里已经发出（出口忙时它只是排进了队列），
        #      由 step() 里的 _poll_gimbal 每帧把它补发出去；
        #   2. 这里每帧检查出口是否真的回到了巡线视角，没到就继续停车等，
        #      而且**有界**：超过 release_resume_timeout 转进既有的"需要人工复位"路径。
        if self._view_changed:
            self.output.hard_stop()
            if self._gimbal_at_line_view():
                self._view_changed = False
                self._view_restore_failed = False
                self._view_ready_at = now + self.settings.gimbal_settle_seconds
            elif self._release_started is not None and (
                now - self._release_started
                > self.settings.tasks.release_resume_timeout
            ):
                self._view_restore_failed = True
                errors.append("gimbal did not return to line view in time")
            if self._view_changed:
                return CoordinatorDecision(
                    state=RELEASING,
                    owner=self.output.owner,
                    line=self.last_line_decision,
                    command=STOP_COMMAND,
                    message="waiting for gimbal to return to line view",
                    errors=tuple(errors),
                )

        if self._view_ready_at is not None and now < self._view_ready_at:
            self.output.hard_stop()
            return CoordinatorDecision(
                state=RELEASING,
                owner=self.output.owner,
                line=self.last_line_decision,
                command=STOP_COMMAND,
                message="waiting for gimbal to return to line view",
                errors=tuple(errors),
            )
        self._view_ready_at = None
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
        """No new camera frame. Never let a task keep driving through it.

        **只有视频真的丢了才结束任务。** main 在"这一轮没等到新帧"时就会调到这里，
        而正常的一帧间隔也可能触发它（实测：等帧超时 12ms、相机 30fps 时，
        75% 的循环都会走到这里）。如果无条件结束任务，任何模块都活不过一帧 ——
        实车日志里"接管后 0.1 秒就被释放"就是这么来的。
        阈值与底座自身的判据共用 `video_gap_stop_seconds`：视频真的断了，
        任务照样被立刻结束并硬停车（安全性不变）。
        """
        errors = []
        message = ""
        video_really_lost = frame_age >= self.settings.video_gap_stop_seconds
        if self.active_task is not None and video_really_lost:
            message = f"{self.active_task_name} ended by video gap"
            self._end_takeover(now, errors)
        decision = self.follower.process_video_gap(frame_age, now)
        self.last_line_decision = decision
        if decision.force_stop:
            self.output.hard_stop()
        return CoordinatorDecision(
            state=self.state,
            owner=self.output.owner,
            line=decision,
            task_name=self.active_task_name,
            command=STOP_COMMAND,
            force_stop=decision.force_stop,
            message=message or decision.message,
            errors=tuple(errors),
        )

    # -- human override -------------------------------------------------
    def _end_takeover(self, now: float, errors=None) -> Optional[str]:
        if self.active_task is None:
            return None
        name = self.active_task_name
        task = self.active_task
        reset_errors = [] if errors is None else errors
        self.output.hard_stop()
        self.output.claim(OWNER_LINE)
        self.follower.reset_fault(now)
        self._reset_task(task, reset_errors)
        self.active_task = None
        self.active_task_name = None
        self._active_started = None
        self._paused_seconds = 0.0
        self._release_started = None
        self.state = LINE_FOLLOWING
        self._note_owner(LINE_OWNER, LINE_PRIORITY, now)
        self._restore_line_view(now, [] if errors is None else errors)
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
        restore_errors = []
        self._restore_line_view(now, restore_errors, force=True)
        self._release_started = None
        self.state = LINE_FOLLOWING
        if restore_errors:
            return "Reset: stopped; line-view restore failed. Check the gimbal."
        if ended is not None:
            return f"Reset; task {ended} ended. Show a valid line, then press SPACE."
        return "Reset: stopped; show a valid line, then press SPACE."

    def human_resume(self, now: float) -> bool:
        if self.active_task is not None:
            return False
        if self.state != LINE_FOLLOWING:
            return False
        if self._view_restore_failed:
            return False
        if self._view_ready_at is not None and now < self._view_ready_at:
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
