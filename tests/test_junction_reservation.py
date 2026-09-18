"""拥堵岔路不该被 obstacle 抢走：free_junction 通过预约通道接管。

实车证据（captures/run_20260918_191329，用户报告"第二个障碍物岔路…为啥是冲进去"）：

    [63.3s] >>> 模块开始运行：obstacle
    [63.3s]     优先级截断：排在它后面、这一帧没被问到的模块 → free_junction, ...
    得分截图: Team 10 detects obstacle » the left side

画面里是**一辆小车停在左侧分支**上（任务 5 的"拥堵岔路"）。`obstacle` 排在
注册表第 2 位、`free_junction` 第 3 位，而 `obstacle.py` **没有岔路概念**，
于是它把"分支上的车"当成路中间的障碍去**往旁边让** → 拐进了被堵的分支。

修法：`free_junction` 实现预约钩子 `wants_control()`，只在
"岔路 + 某条分支有车"时声明想要控制权，协调器让它从 obstacle 手里接管。

只用合成帧、假底盘：不连车、不连相机。
"""

import pathlib
import sys
import unittest

import numpy as np

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
from tests.test_free_junction import fork_frame  # noqa: E402

from coordinator import RESERVATION_HOOK, TaskCoordinator  # noqa: E402
from free_junction import FreeJunctionConfig, FreeJunctionTask  # noqa: E402
from models import FramePacket, MotionCommand, TaskStatus, TaskUpdate  # noqa: E402


class ObstacleAtForkTests(unittest.TestCase):
    """默认**关**（真实 run 验证后决定）；打开时机制必须成立。

    2026-09-18：用恢复回来的真实 run（`captures/run_20260918_191329`，35 张
    关键帧）逐帧验证后发现拥堵判据**没有特异性** —— 4 个被检出岔路的真实帧
    **全部**返回 `reading=blocked`（100% 误报），包括"相机低头看自己车体"
    和"地上一个数字标识盒"。放出去会让本模块到处抢岔路、选错边。
    所以 `enable_reservation_hook` 默认 False；这里的测试显式打开它，
    保证"机制本身是好的、以后标定好了能直接启用"。
    """

    #: 打开预约钩子的配置（本文件所有测试都用它）
    CONFIG = FreeJunctionConfig(enable_reservation_hook=True)

    def setUp(self):
        self.sequence = 0
        self.now = 100.0

    def test_the_hook_is_off_by_default(self):
        """默认必须是关的 —— 判据没标定好之前不许它抢岔路。"""
        self.assertFalse(FreeJunctionConfig().enable_reservation_hook)

    # -- 钩子本身：惰性 + 只在"岔路 + 有车"时出手 ----------------------
    def test_the_hook_is_inert_on_a_plain_line(self):
        """没有岔路 → 绝不干预（用户要求"跑完全程为先"）。"""
        task = FreeJunctionTask(self.CONFIG)
        plain = line_frame(320)
        self.assertFalse(task.wants_control(FramePacket(plain, 1, 1.0), 1.0))

    def test_the_hook_is_inert_on_a_fork_without_a_vehicle(self):
        """看到岔路但两条分支都没车 → 也不出手（那是任务 4 的场景）。"""
        task = FreeJunctionTask(self.CONFIG)
        empty_fork = fork_frame()
        self.assertFalse(task.wants_control(FramePacket(empty_fork, 1, 1.0), 1.0))

    def test_the_hook_fires_when_a_branch_is_blocked(self):
        """岔路 + 左侧分支停着车 = 任务 5 场景 → 要控制权。"""
        task = FreeJunctionTask(self.CONFIG)
        blocked = fork_frame(car_left=True)
        self.assertTrue(
            task.wants_control(FramePacket(blocked, 1, 1.0), 1.0),
            "拥堵岔路上必须声明想要控制权，否则会被 obstacle 抢走并冲进堵的那条",
        )

    def test_the_hook_never_raises(self):
        task = FreeJunctionTask(self.CONFIG)
        blank = np.full((360, 640, 3), 200, np.uint8)
        self.assertIsInstance(
            task.wants_control(FramePacket(blank, 1, 1.0), 1.0), bool)

    # -- 用户定的规则：岔路口遇到车 → free_junction；直路障碍 → obstacle ----
    def test_a_straight_road_obstacle_does_not_trigger_the_junction_module(self):
        """**直路上的障碍**不该惊动 free_junction —— 那是 obstacle 的活。

        用户的原话："就在岔路口发现车就 free_junction，直路避障不就行了吗"。
        这条钉住前半句的反面：**没有岔路几何时，free_junction 必须让开**，
        否则它会去跟 obstacle 抢直路上的活。
        """
        task = FreeJunctionTask(self.CONFIG)
        # 直路 + 正前方一块"障碍"（不构成 Y 形分叉）
        straight = line_frame(320)
        straight[120:200, 280:360] = 60        # 路中间一个暗色方块
        self.assertFalse(
            task.wants_control(FramePacket(straight, 1, 1.0), 1.0),
            "直路上的障碍不该让 free_junction 出手（那是 obstacle 的场景）",
        )

    def test_a_real_lane_obstacle_still_belongs_to_obstacle(self):
        """协调器层：直路上 obstacle 照常接管，free_junction 不抢。"""

        class LaneObstacle:
            name = "obstacle"

            def step(self, frame, now):
                return TaskUpdate(
                    TaskStatus.RUNNING,
                    motion=MotionCommand(forward=0.08, lateral=-0.08),
                    message="obstacle; stepping aside",
                )

        obstacle = LaneObstacle()
        junction = FreeJunctionTask(self.CONFIG)
        coordinator = self._build((obstacle, junction))
        self.now += 0.05
        self.sequence += 1
        # 全程喂普通直路帧（没有岔路）
        for _ in range(120):
            self.now += 0.05
            self.sequence += 1
            coordinator.step(
                FramePacket(line_frame(320), self.sequence, self.now), self.now)
        self.assertEqual(
            coordinator.active_task_name, "obstacle",
            "直路上必须让 obstacle 干活，free_junction 不许抢",
        )

    # -- 协调器层：预约真的能把控制权从 obstacle 手里接过来 --------------
    def _build(self, tasks):
        self.chassis = FakeChassis()
        self.follower = LineFollower(CONFIG)
        self.output = MotionOutput(self.chassis, CONFIG)
        self.coordinator = TaskCoordinator(
            CONFIG,
            self.follower,
            self.output,
            motion_tasks=tuple(tasks),
            observers=(),
            gimbal_output=GimbalOutput(FakeGimbal(), CONFIG),
        )
        packet = FramePacket(line_frame(320), 1, self.now)
        self.follower.process_frame(packet.image, packet.captured_at)
        self.assertTrue(self.follower.resume(self.now), "巡线底座要能起来")
        return self.coordinator

    def test_free_junction_is_registered_for_reservations(self):
        task = FreeJunctionTask(self.CONFIG)
        self.assertTrue(callable(getattr(task, RESERVATION_HOOK, None)),
                        "free_junction 必须实现预约钩子才能从 obstacle 手里接管")

    def test_the_reservation_task_takes_over_from_an_earlier_module(self):
        """obstacle 先接管，之后 free_junction 靠预约把它接过来。"""

        class GreedyObstacle:
            name = "obstacle"

            def __init__(self):
                self.calls = 0

            def step(self, frame, now):
                self.calls += 1
                return TaskUpdate(
                    TaskStatus.RUNNING,
                    motion=MotionCommand(forward=0.08, lateral=-0.08),
                    message="obstacle; stepping aside",
                )

        obstacle = GreedyObstacle()
        junction = FreeJunctionTask(self.CONFIG)
        coordinator = self._build((obstacle, junction))
        self.assertEqual(len(coordinator.reservation_tasks), 1)

        # 先让 obstacle 接管（合成帧是普通线路）
        self.now += 0.05
        self.sequence += 1
        coordinator.step(FramePacket(line_frame(320), self.sequence, self.now),
                         self.now)
        self.assertEqual(coordinator.active_task_name, "obstacle")

        # 之后持续喂"岔路 + 左侧有车"的帧：free_junction 应当接手
        blocked = fork_frame(car_left=True)
        deadline = self.now + 6.0
        while self.now < deadline:
            self.now += 0.05
            self.sequence += 1
            coordinator.step(FramePacket(blocked, self.sequence, self.now), self.now)
            if coordinator.active_task_name == "free_junction":
                break

        self.assertEqual(
            coordinator.active_task_name, "free_junction",
            "拥堵岔路上 free_junction 必须从 obstacle 手里接管，否则车会冲进被堵的分支",
        )


if __name__ == "__main__":
    unittest.main()
