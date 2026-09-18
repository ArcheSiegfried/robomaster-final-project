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

#: 红绿灯模块的注册名。有任务在接管时，协调器仍然每帧单独问它一次（红灯否决）。
#: 2026-09-17：`traffic_light.py` 回来了（3 号重新提交），下面这套"红灯否决权"
#: **重新生效**：任何任务开车时遇到红灯都会被暂停（暂停的秒数不计入任务的超时预算），
#: 红灯消失后原任务继续。
#: 2026-09-18：它**不再排第 1 位**（现为第 5 位，考场顺序见 `task_registry.py`）。
#: 查找是按 **name** 而不是位次，所以否决权照样生效 —— 但也正因如此，
#: 灯模块一旦接管就会把当前任务按停。实测过的后果：第一岔路口"左红右绿"时，
#: 灯模块 2 帧即接管、而岔路模块要 3 帧才确认岔路，岔路模块被按成 599/600 帧零指令。
#: 现在靠 `traffic_light` 自己的 `fork_light_competition` 闸门避免（红绿同框时它不接管），
#: 这里的机制保持原样。见 PRIORITY_ORDER_AND_FORK_FIX.md。
#: 机制本身没变过，找不到名为 traffic_light 的模块时会自动失效（`light_task=None`）。
LIGHT_TASK_NAME = "traffic_light"

#: **预约接管**：这些模块即使排在很后面，也要有机会拿到运动权。
#:
#: 背景（2026-09-18 实车实测）：仲裁是"按顺序问、遇到第一个 RUNNING 就停"，
#: 所以排在前面的模块一强触发，后面的模块那一帧**根本不会被问**。
#: `number_marker` 排最后，那次 run 196 秒里被截断 9 次、**接管 0 次、
#: 一张标识照片都没存**（标识 5 分/个、满 25 分，是全场最大一块）。
#: 靠调注册表顺序救不了它：把它提前就会反过来把岔路/绕障挡住。
#:
#: 所以另开一条通道：这些模块可以实现一个**只读**的
#: ``wants_control(frame, now) -> bool``，协调器在"按顺序问"之前先问它们一遍。
#: 命名沿用骨架里已有的鸭子类型做法（`reset()` / `record_decision()` 都是这样），
#: 没有这个方法的模块行为**完全不变**。
#:
#: 何时生效：**只在"已经有别的模块拿着运动权"时**。没人开车时仍然按注册表
#: 顺序问，所以"岔路优先于标识"这类裁定保持不变 —— 饿死本来就只发生在
#: "前面的模块一直拿着运动权"的时候。
#:
#: 安全边界（都有测试）：
#:   * 只在基础巡线可接管时生效，且仍然受 TaskConfig 全部限幅与超时约束；
#:   * 红灯否决权优先于它 —— 红灯亮着时预约不生效；
#:   * 已有任务在开车时，预约只在该任务跑够 `RESERVATION_MIN_HOLD_SECONDS`
#:     之后才允许抢（防两个模块每帧互相抢）；
#:   * `wants_control()` 必须只读、必须快：它每帧都会被调用。
#: 目前只有 `number_marker` 用它。见 PRIORITY_ORDER_AND_FORK_FIX.md。
RESERVATION_HOOK = "wants_control"

#: 已接管的模块至少要跑这么久，才允许被"预约"抢走（秒）。
RESERVATION_MIN_HOLD_SECONDS = 1.0

#: **靠近补充**：接手中的任务可以额外请求一个受限前向量（m/s）。
#:
#: 背景（2026-09-18 实车）：`number_marker` 的 MotionCommand **只有 yaw、没有
#: forward** —— 它只原地转头瞄准，不会往前开。而赛题要求标识宽度 > 1/5 画面宽
#: 才计分，实测 SDK 报出来的只有 3.4%（22 像素）。车不靠近，标识永远不会变大，
#: 于是永远拿不到分（先是"不接管"的死锁，拆开接管门槛后变成"靠不近"的死锁）。
#:
#: 这里给集成层一个**不改模块语义**的补法：任务实现一个只读的
#: ``approach_forward_mps(now) -> float``，协调器在任务**自己已经给出运动**时
#: 把这个前向量**叠加**上去，再统一受 TaskConfig 限幅。
#:   * 任务没有这个方法 → 行为完全不变；
#:   * 返回 0 或负数 → 不叠加（模块自己就能喊停）；
#:   * 永远不覆盖任务给的 forward，只是加上去。
#: 目前只有 `number_marker` 用它。
APPROACH_HOOK = "approach_forward_mps"

#: 叠加量的上限（m/s）：再大就可能撞到标识或冲过岔路。默认很保守。
MAX_APPROACH_FORWARD = 0.12

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
        # 红灯否决（见 _light_veto）：即使有任务在接管，也要每帧问一次红绿灯。
        # 没有它，"一帧定生死、赢家通吃"就意味着任何模块先接管之后，红灯再亮也
        # 没人问（实测：障碍接管期间红灯亮着，车仍以 forward=0.2 在走）。
        self.light_task = next(
            (task for task in self.motion_tasks
             if getattr(task, "name", None) == LIGHT_TASK_NAME),
            None,
        )
        # 预约通道（见 RESERVATION_HOOK）：只有实现了 wants_control() 的模块参与。
        # 没有的话这个元组是空的，一切行为与以前完全一致。
        self.reservation_tasks = tuple(
            task for task in self.motion_tasks
            if callable(getattr(task, RESERVATION_HOOK, None))
        )
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

    def _find_reserved_takeover(self, frame, now, errors):
        """先问一遍"预约"模块：它们即使排在最后也要有机会拿到运动权。

        只调用**只读**的 `wants_control(frame, now)`，命中后再走正常的
        `step()` 路径（`_step_line` 会把它的 update 交给 `_begin_takeover`）。
        没有这个方法的模块完全不受影响；抛异常只记录、不打断主循环。
        """
        if not self.reservation_tasks:
            return None
        for task in self.reservation_tasks:
            wants = getattr(task, RESERVATION_HOOK, None)
            if not callable(wants):
                continue
            try:
                if not wants(frame, now):
                    continue
            except Exception as error:
                errors.append(f"{task.name} {RESERVATION_HOOK} raised: {error}")
                continue
            update = self._call(task.step, task.name, frame, now, errors)
            if update is not None and update.status is TaskStatus.RUNNING:
                errors.append(f"{task.name} took over via reservation")
                return (task, update)
        return None

    def _find_takeover(self, frame, now, errors, probe_all: bool = False):
        """按顺序问模块，返回第一个"想接管"的。

        `probe_all=True` 时**问完所有模块**（胜负规则不变，仍是第一个 RUNNING 赢），
        并把各自的想法记进 `self.last_claims` 供终端显示"竞争实况"。
        """
        winner = None
        claims = []
        for task in self.motion_tasks:
            update = self._call(task.step, task.name, frame, now, errors)
            if update is not None and update.status is TaskStatus.RUNNING:
                if winner is None:
                    winner = (task, update)
                if not probe_all:
                    # 正常帧：遇到第一个 RUNNING 就停（保持既有开销与状态推进）。
                    self.last_claims = ()
                    return winner
            if probe_all:
                claims.append({
                    "name": task.name,
                    "status": "ERROR" if update is None else update.status.name,
                    "message": "" if update is None else str(update.message or ""),
                })
        self.last_claims = tuple(claims) if probe_all else ()
        return winner

    def _apply_task_motion(self, update: TaskUpdate, errors, task=None,
                           now: Optional[float] = None) -> MotionCommand:
        if update.motion is None:
            self.output.hard_stop()
            return STOP_COMMAND
        requested = self._with_approach_forward(update.motion, task, now, errors)
        command = clamp_task_command(requested, self.settings.tasks)
        if (
            command.forward != requested.forward
            or command.lateral != requested.lateral
            or command.yaw != requested.yaw
        ):
            errors.append(
                f"task command clamped from "
                f"({requested.forward}, {requested.lateral}, {requested.yaw}) "
                f"to ({command.forward}, {command.lateral}, {command.yaw})"
            )
        self.output.send(OWNER_EXTERNAL, command)
        return command

    def _with_approach_forward(self, motion: MotionCommand, task, now,
                               errors) -> MotionCommand:
        """给"只会转头、不会前进"的任务叠加一个受限前向量（见 APPROACH_HOOK）。

        绝不覆盖任务给的 forward，只做加法；没有钩子、钩子返回非正数、
        或钩子抛异常时都原样返回。这是集成层的补丁，模块本身语义不变。
        """
        if task is None:
            return motion
        hook = getattr(task, APPROACH_HOOK, None)
        if not callable(hook):
            return motion
        try:
            extra = float(hook(now))
        except Exception as error:
            errors.append(f"{getattr(task, 'name', '?')} {APPROACH_HOOK} raised: {error}")
            return motion
        if not math.isfinite(extra) or extra <= 0.0:
            return motion
        extra = min(extra, MAX_APPROACH_FORWARD)
        return MotionCommand(
            forward=motion.forward + extra,
            lateral=motion.lateral,
            yaw=motion.yaw,
        )

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

    # -- per-cycle entry point -----------------------------------------
    def step(self, frame: FramePacket, now: float) -> CoordinatorDecision:
        errors = []
        delta = 0.0 if self._previous_frame_time is None else max(
            0.0, now - self._previous_frame_time)
        self._previous_frame_time = now
        self._poll_gimbal(errors)
        self._observe(frame, now, errors)

        if self.active_task is not None:
            return self._step_active(frame, now, errors, delta)
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
        （仍是顺序里第一个 RUNNING）；代价只是那一帧会让未接管的模块多推进一次
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

        if self.takeover_allowed:
            # 注意：预约通道**不在这里**。没人开车时仍按注册表顺序问，
            # 这样"岔路优先于标识"的裁定保持不变；饿死只发生在
            # "已经有别的模块拿着运动权"的时候（见 _step_active）。
            takeover = self._find_takeover(frame, now, errors, probe_all=probe)
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
        # 必须归零：这是"当前任务等红灯被暂停"的累计秒数，要减在**当前任务**
        # 的 max_task_seconds 上。残留上一个任务的值会让新任务的超时预算被
        # 莫名扣掉（也是预约交接路径最容易漏掉的一处）。
        self._paused_seconds = 0.0
        self.state = TASK_ACTIVE
        if not self._apply_task_gimbal(update, errors):
            return self._release(
                now, errors, "task gimbal request failed", reset_task=True
            )
        command = self._apply_task_motion(update, errors, task=task, now=now)
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

    def _step_active(self, frame, now, errors, delta: float = 0.0) -> CoordinatorDecision:
        task = self.active_task
        # 有人在开车时不再报"竞争"（这一帧只有它被问，报出来会误导操作员）。
        self.last_claims = ()
        elapsed = now - self._active_started - self._paused_seconds
        if elapsed > self.settings.tasks.max_task_seconds:
            errors.append(
                f"{task.name} exceeded max_task_seconds "
                f"({self.settings.tasks.max_task_seconds:.1f}s)"
            )
            return self._release(now, errors, "task timeout", reset_task=True)

        veto = self._light_veto(frame, now, errors)
        if veto is not None:
            # 红灯：暂停当前任务（这一帧不调它的 step），只下发停车指令。
            # 不做释放握手 —— 红灯不是故障，不需要人来按 SPACE；
            # 等灯的时间也不计入任务的超时预算。
            self._paused_seconds += delta
            self.output.send(OWNER_EXTERNAL, STOP_COMMAND)
            return CoordinatorDecision(
                state=TASK_ACTIVE,
                owner=self.output.owner,
                line=self.last_line_decision,
                task_name=task.name,
                task_update=veto,
                command=STOP_COMMAND,
                message=f"red light veto: {task.name} paused",
                errors=tuple(errors),
            )

        # 预约接管：正在开车的模块跑够 min_hold 之后，让排在很后面的模块
        # （例如 number_marker）有机会接手。红灯否决在上面，优先级更高。
        if self._reservation_can_preempt(now, task):
            takeover = self._find_reserved_takeover(frame, now, errors)
            if takeover is not None and takeover[0] is not task:
                other, other_update = takeover
                return self._hand_over_to_reservation(
                    task, other, other_update, now, errors
                )

        update = self._call(task.step, task.name, frame, now, errors)
        if update is None:
            return self._release(
                now, errors, "task raised an exception", reset_task=True
            )
        if update.status is TaskStatus.RUNNING:
            if not self._apply_task_gimbal(update, errors):
                return self._release(
                    now, errors, "task gimbal request failed", reset_task=True
                )
            command = self._apply_task_motion(update, errors, task=task, now=now)
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
            return self._release(
                now,
                errors,
                "task gave up while owning motion",
                reset_task=True,
            )
        return self._release(
            now, errors, f"task {update.status.value}", update=update
        )

    def _light_veto(self, frame, now, errors):
        """红灯否决：有任务正在开车时，也每帧问一次红绿灯。

        正常情况下红绿灯模块排在最前面，自己就能接管；但它只能"从巡线手里"接管。
        一旦别的模块先拿走运动出口，协调器整帧只调那一个模块，红绿灯模块再也
        没机会说话 —— 于是红灯亮着车照样走。

        这里只做两件事：问一次、以及把结果交回给调用方去执行停车。
        **不结束任务、不做释放握手**：红灯结束（或它自己超时）之后原任务继续干。
        """
        light = self.light_task
        if light is None or light is self.active_task:
            return None
        update = self._call(light.step, light.name, frame, now, errors)
        if update is None or update.status is not TaskStatus.RUNNING:
            return None
        return update

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

    def _reservation_can_preempt(self, now: float, current) -> bool:
        """现在允许"预约"抢走运动权吗？

        要求：已经有预约模块存在、当前任务不是预约模块之一、而且它已经
        连续开了 `RESERVATION_MIN_HOLD_SECONDS` 以上 —— 否则两个模块会
        每帧互相抢。红灯否决在调用处先判，所以红灯期间不会走到这里。
        """
        if not self.reservation_tasks or current is None:
            return False
        if any(current is task for task in self.reservation_tasks):
            return False
        started = self._active_started
        if started is None:
            return False
        return (now - started - self._paused_seconds) >= RESERVATION_MIN_HOLD_SECONDS

    def _hand_over_to_reservation(self, current, other, other_update, now, errors):
        """把运动权从 `current` 交给预约模块 `other`（同一次调用内完成）。

        不能用 `_release()` + `_begin_takeover()`：`_release` 会把状态机推进
        RELEASING 并等"新帧确认路线"，那个握手在这里没有意义。这里做的是
        原地交接：硬停 → 换 owner → 清巡线故障 → 让新模块接管。
        """
        errors.append(f"{current.name} handed over to {other.name} (reservation)")
        self.output.hard_stop()
        self.output.claim(OWNER_EXTERNAL)
        self.follower.reset_fault(now)
        self.active_task = None
        self.active_task_name = None
        self._active_started = None
        self._paused_seconds = 0.0
        self._release_started = None
        self.state = LINE_FOLLOWING
        return self._begin_takeover(other, other_update, now, errors)

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
