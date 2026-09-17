"""闭环回归：`gimbal_output` v0.3 的"忙时排队"必须真的会被补发。

为什么单独有这一组测试（这是实车复现过的坑）：
  `coordinator.py` 的释放阶段**只在任务结束那一帧**调一次 `restore_line_view()`，
  之后 `_step_releasing` 既不 poll 也不再调它。所以如果出口在忙时只把"回巡线视角"
  排进队列（v0.3 的行为），而没人周期性 poll，队列里的目标**永远不会被补发** ——
  车就会一直停在 `waiting for gimbal to return to line view`。

这组测试钉住三件事：
  1. 协调器**每帧**推进一次出口（`poll()` 每 cycle 恰好一次）；
  2. 没真的回到巡线视角就不许恢复巡线（不许"假装修好了"）；
  3. 云台一直回不来时**有界**：转进既有的"需要人工复位"路径，不无限等待。

GimbalOutput 自身的语义（排队/最新优先/last_error…）由孙宇鹏的
`tests/test_gimbal_output.py` 负责；这里用的是本文件里的假出口，不依赖它的实现。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import CONFIG  # noqa: E402
from models import (  # noqa: E402
    GimbalCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)
from tests.task_harness import TaskHarness  # noqa: E402


LINE_VIEW = GimbalCommand(pitch=CONFIG.gimbal_pitch, yaw=CONFIG.gimbal_yaw)
SEARCH = GimbalCommand(pitch=CONFIG.gimbal_search_pitch, yaw=CONFIG.gimbal_yaw)


class ScriptedOutlet:
    """v0.3 表面的假出口：一次只跑一个绝对动作，忙时最新目标进队列。

    `poll()` 每被调用一次就推进一帧"动作进度"；`at_line_view` 只有**动作跑完且
    队列为空且当前目标就是巡线视角**时才为真 —— 与真实出口的语义一致。
    """

    def __init__(self, complete_after: int = 2) -> None:
        self.line_view = LINE_VIEW
        self.started = []                  # 真正发出去的绝对动作
        self.queued = []                   # 因为忙而排队的目标
        self.polls = 0
        self.complete_after = complete_after
        self._current = LINE_VIEW          # 上电即在巡线视角
        self._in_flight = None
        self._progress = 0
        self._pending = None

    # -- v0.3 表面 -----------------------------------------------------
    def send(self, command):
        self._queue(command)
        return command

    def restore_line_view(self):
        self._queue(LINE_VIEW)
        return LINE_VIEW

    def poll(self):
        self.polls += 1
        if self._in_flight is None:
            if self._pending is not None:
                command, self._pending = self._pending, None
                self._start(command)
            return
        self._progress += 1
        if self._progress >= self.complete_after:
            self._current = self._in_flight
            self._in_flight = None
            self._progress = 0

    @property
    def busy(self):
        return self._in_flight is not None

    @property
    def pending(self):
        return self._pending

    @property
    def at_line_view(self):
        return (
            self._in_flight is None
            and self._pending is None
            and self._current == LINE_VIEW
        )

    def stats(self):
        return {"started": len(self.started), "queued": len(self.queued),
                "polls": self.polls}

    # -- 内部 -----------------------------------------------------------
    def _queue(self, command):
        if self._in_flight is not None:
            self._pending = command        # latest wins，绝不发第二个 moveto
            self.queued.append(command)
            return
        self._start(command)

    def _start(self, command):
        self._in_flight = command
        self._progress = 0
        self.started.append(command)


class LegacyOutlet:
    """v0.2 风格出口：没有 poll / at_line_view，行为必须与改造前一致。"""

    def __init__(self):
        self.line_view = LINE_VIEW
        self.sent = []

    def send(self, command):
        self.sent.append(command)
        return command

    def restore_line_view(self):
        self.sent.append(LINE_VIEW)
        return LINE_VIEW


class AimingTask:
    """会请求云台、然后自己结束的任务（驱动"释放"这条路径）。"""

    name = "aiming_probe"

    def __init__(self, aim_frames: int = 2) -> None:
        self.aim_frames = aim_frames
        self.calls = 0

    def step(self, frame, now):
        self.calls += 1
        detection = VisualDetection.no_result(self.name)
        if self.calls <= self.aim_frames:
            return TaskUpdate(
                TaskStatus.RUNNING,
                detection=detection,
                gimbal=SEARCH,
                message="aiming",
            )
        return TaskUpdate(TaskStatus.COMPLETED, detection=detection, message="done")


def _run_until(harness, condition, now, limit=200, step=0.05):
    """喂巡线帧直到条件成立；返回 (最后一帧的决策, 用掉的帧数)。"""
    decision = None
    for index in range(limit):
        now += step
        decision = harness.feed_line(now)
        if condition(decision):
            return decision, index + 1, now
    return decision, limit, now


class ReleaseClosureTests(unittest.TestCase):
    def test_polling_happens_exactly_once_per_cycle(self):
        outlet = ScriptedOutlet()
        harness = TaskHarness(task=AimingTask())
        harness.coordinator.gimbal_output = outlet
        now = 1.0
        for _ in range(5):
            now += 0.05
            harness.feed_line(now)
        self.assertEqual(outlet.polls, 5, "出口必须每帧被推进一次（poll）")

    def test_queued_line_view_is_actually_sent_and_blocks_resume_until_it_is(self):
        # complete_after=3：瞄准动作要 3 次 poll 才结束，期间释放请求必然进队列
        outlet = ScriptedOutlet(complete_after=3)
        harness = TaskHarness(task=AimingTask(aim_frames=1))
        harness.coordinator.gimbal_output = outlet

        now = 1.0
        harness.start_line(now)
        # 让它接管、请求云台、然后自己 COMPLETED -> 进 RELEASING 并请求恢复视角
        saw_wait = False
        for _ in range(40):
            now += 0.05
            decision = harness.feed_line(now)
            if decision.message == "waiting for gimbal to return to line view":
                saw_wait = True
            if outlet.started and outlet.started[-1] == LINE_VIEW:
                break
        else:
            self.fail("队列里的巡线视角一直没被补发（闭环断了）")

        self.assertIn(LINE_VIEW, outlet.queued,
                      "忙的时候没有把恢复请求排进队列（就直接发第二个 moveto 了）")
        self.assertTrue(saw_wait, "没到位时不应直接去认线，应报 waiting for gimbal...")
        self.assertEqual(outlet.started[-1], LINE_VIEW, "回巡线视角的动作最终必须真的发出")

        # 到位之后才允许恢复巡线
        decision, _, _ = _run_until(
            harness, lambda d: d.state == "LINE_FOLLOWING", now, limit=200)
        self.assertEqual(decision.state, "LINE_FOLLOWING", "云台到位后应恢复巡线")

    def test_line_view_never_reached_is_bounded_and_asks_for_a_human(self):
        outlet = ScriptedOutlet(complete_after=10 ** 9)   # 永远跑不完
        harness = TaskHarness(task=AimingTask(aim_frames=1))
        harness.coordinator.gimbal_output = outlet

        now = 1.0
        harness.start_line(now)
        messages = set()
        for _ in range(200):
            now += 0.05
            decision = harness.feed_line(now)
            messages.add(decision.message)
            if "reset and resume required" in decision.message:
                break
        self.assertIn(
            "reset and resume required", " ".join(messages),
            "云台回不来时必须走到「需要人工复位」这条既有路径，而不是无限等待：%s" % messages,
        )
        self.assertFalse(outlet.at_line_view)

    def test_a_legacy_outlet_keeps_the_old_behaviour(self):
        outlet = LegacyOutlet()
        harness = TaskHarness(task=AimingTask(aim_frames=1))
        harness.coordinator.gimbal_output = outlet
        now = 1.0
        harness.start_line(now)
        for _ in range(40):
            now += 0.05
            harness.feed_line(now)
        self.assertIn(LINE_VIEW, outlet.sent, "老出口也必须收到恢复请求")
        self.assertTrue(
            all(not hasattr(outlet, name) for name in ("poll", "at_line_view")),
            "这条测试用的就是老出口",
        )


if __name__ == "__main__":
    unittest.main()
