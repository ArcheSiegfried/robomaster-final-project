"""靠近补充通道：让"只会转头、不会前进"的任务能靠近到可计分的大小。

为什么要有这个文件（2026-09-18 实车，captures/run_20260918_175238）：
赛题要求标识宽度 > 画面宽 1/5 才计分，而 SDK 报出来的标识只有 **3.4%**
（22 像素）。`number_marker` 的 MotionCommand **只有 yaw、没有 forward** ——
它只原地转头瞄准。于是有两个死锁：

  1. 接管门槛原本就是 0.20 → 永远不接管（已由 trigger_min_marker_width_ratio 解开）；
  2. 解开之后，模块接管了但**车不前进** → 标识永远停在 3.4% → 还是 0 分。

这个通道解第 2 个：任务实现**只读**的 ``approach_forward_mps(now)``，
协调器把它**叠加**在任务自己的指令上（不改模块语义），再统一限幅。

只用假任务、合成帧、假底盘：不连车、不连相机。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tests.task_harness import (  # noqa: E402
    CONFIG,
    FakeChassis,
    FakeGimbal,
    GimbalOutput,
    LineFollower,
    MotionOutput,
    line_frame,
)

from coordinator import (  # noqa: E402
    MAX_APPROACH_FORWARD,
    TaskCoordinator,
)
from models import (  # noqa: E402
    FramePacket,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
)


class _Task:
    """最小任务：接管后给出的运动完全由构造时指定。"""

    def __init__(self, name="task", motion=None, extra=None, explode=False):
        self.name = name
        self.motion = motion
        self.extra = extra
        self.explode = explode
        self.calls = 0

    def step(self, frame, now):
        self.calls += 1
        return TaskUpdate(TaskStatus.RUNNING, motion=self.motion, message="running")

    def approach_forward_mps(self, now):
        if self.explode:
            raise RuntimeError("synthetic: approach hook on fire")
        return self.extra


class ApproachHookTests(unittest.TestCase):
    def setUp(self):
        self.sequence = 0
        self.now = 100.0

    def build(self, task):
        self.chassis = FakeChassis()
        self.follower = LineFollower(CONFIG)
        self.output = MotionOutput(self.chassis, CONFIG)
        self.coordinator = TaskCoordinator(
            CONFIG,
            self.follower,
            self.output,
            motion_tasks=(task,),
            observers=(),
            gimbal_output=GimbalOutput(FakeGimbal(), CONFIG),
        )
        packet = FramePacket(line_frame(320), 1, self.now)
        self.follower.process_frame(packet.image, packet.captured_at)
        self.assertTrue(self.follower.resume(self.now), "巡线底座要能起来")
        return self.coordinator

    def feed(self, frames=1, step=1.0 / 30.0):
        decision = None
        for _ in range(frames):
            self.now += step
            self.sequence += 1
            decision = self.coordinator.step(
                FramePacket(line_frame(320), self.sequence, self.now), self.now
            )
        return decision

    # -- 核心：叠加生效 -------------------------------------------------
    def test_extra_forward_is_added_to_the_task_command(self):
        """任务只给 yaw（和 number_marker 一样），协调器补上前进量。"""
        task = _Task(motion=MotionCommand(yaw=12.0), extra=0.10)
        self.build(task)
        decision = self.feed()
        self.assertAlmostEqual(decision.command.yaw, 12.0)
        self.assertAlmostEqual(decision.command.forward, 0.10,
                               msg="协调器应当把靠近量叠加上去")

    def test_a_task_without_the_hook_is_untouched(self):
        class Plain(_Task):
            approach_forward_mps = None      # 显式没有这个钩子

        task = Plain(motion=MotionCommand(yaw=12.0))
        task.__dict__.pop("approach_forward_mps", None)
        self.build(task)
        decision = self.feed()
        self.assertAlmostEqual(decision.command.forward, 0.0,
                               msg="没有钩子的任务行为必须完全不变")

    def test_zero_and_negative_extras_are_ignored(self):
        for extra in (0.0, -0.5):
            with self.subTest(extra=extra):
                task = _Task(motion=MotionCommand(yaw=5.0), extra=extra)
                self.build(task)
                decision = self.feed()
                self.assertAlmostEqual(decision.command.forward, 0.0)

    def test_the_extra_is_capped(self):
        task = _Task(motion=MotionCommand(yaw=5.0), extra=99.0)
        self.build(task)
        decision = self.feed()
        self.assertLessEqual(decision.command.forward, MAX_APPROACH_FORWARD + 1e-9)
        self.assertLessEqual(decision.command.forward,
                             CONFIG.tasks.task_max_forward)

    def test_it_never_replaces_the_task_own_forward(self):
        """任务自己给了前进量时，靠近量是**加**上去而不是覆盖。"""
        task = _Task(motion=MotionCommand(forward=0.05, yaw=1.0), extra=0.05)
        self.build(task)
        decision = self.feed()
        self.assertAlmostEqual(decision.command.forward, 0.10)

    def test_a_broken_hook_does_not_break_the_loop(self):
        task = _Task(motion=MotionCommand(yaw=3.0), explode=True)
        self.build(task)
        decision = self.feed()
        self.assertAlmostEqual(decision.command.forward, 0.0)
        self.assertAlmostEqual(decision.command.yaw, 3.0)
        self.assertTrue(any("on fire" in error for error in decision.errors),
                        f"钩子异常要记进 errors：{decision.errors}")

    def test_no_motion_still_means_stop(self):
        """任务不给运动时仍然是停车，不能因为钩子就自己开起来。"""
        task = _Task(motion=None, extra=0.10)
        self.build(task)
        decision = self.feed()
        self.assertEqual(decision.command, MotionCommand())


class ApproachInertnessTests(unittest.TestCase):
    """**这是最重要的一组**：靠近通道绝不能影响正常巡线。

    number_marker 的 approach_forward_mps() 只有在"已锁定标识 + 正在瞄准 +
    宽度还没到计分门槛"时才返回正数；其余情况一律 0.0。车不看到标识时，
    协调器叠加 0 → 与改动前**完全一致**。用户明确要求"跑完全程为先"。
    """

    def test_real_marker_module_is_inert_without_a_marker(self):
        from number_marker import NumberMarkerConfig, NumberMarkerTask

        task = NumberMarkerTask(NumberMarkerConfig(team_number="10"))
        self.assertEqual(task.approach_forward_mps(1.0), 0.0,
                         "没有标识时不许有任何前进请求")

    def test_the_hook_returns_zero_in_every_non_aiming_state(self):
        from number_marker import MarkerState, NumberMarkerConfig, NumberMarkerTask

        task = NumberMarkerTask(NumberMarkerConfig(team_number="10"))
        for state in MarkerState:
            task.state = state
            if state is MarkerState.AIMING:
                continue          # AIMING 由模块自己的测试覆盖
            self.assertEqual(task.approach_forward_mps(1.0), 0.0,
                             f"{state.value} 状态下不该请求前进")

    def test_a_locked_but_already_large_marker_does_not_close_in(self):
        from number_marker import MarkerState, NumberMarkerConfig, NumberMarkerTask

        task = NumberMarkerTask(NumberMarkerConfig(team_number="10"))
        task._target_id = "1"
        task.state = MarkerState.AIMING
        task._locked_target_width_ratio = 0.25      # 已过计分门槛
        self.assertEqual(task.approach_forward_mps(1.0), 0.0)


if __name__ == "__main__":
    unittest.main()
