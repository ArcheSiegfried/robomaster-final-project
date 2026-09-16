"""红灯否决权：有任务在接管时，红灯照样能打断它。

为什么要有这个：仲裁是"一帧定生死、赢家通吃"——第一个返回 RUNNING 的模块拿走唯一的
运动出口，之后**整帧只调它**，别的模块连 step() 都不再被调用。于是只要障碍/岔路/路线
先接管，红绿灯模块在红灯亮着的时候**一次都不会被问**（实测：障碍接管期间红灯亮着，
车仍然 forward=0.2 在走）。

这里锁死四条：
1. 有任务在接管时，红灯每帧仍被询问，并且**能立刻把车停住**；
2. 停住靠的是"暂停当前任务"，**不是**"结束任务 + 硬停 + 要求人工按 SPACE"；
3. 红灯期间**不算任务超时**，绿灯后原任务继续干（不能因为等了一次红灯就被判超时）；
4. 没有任务接管时，一切照旧（红灯自己接管、自己释放）。

只用假任务、假底盘、合成帧：不连车、不连相机。
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
    blank_frame,
    line_frame,
)

from coordinator import RELEASING, TASK_ACTIVE, TaskCoordinator  # noqa: E402
from models import FramePacket, MotionCommand, TaskStatus, TaskUpdate  # noqa: E402

TRAFFIC_LIGHT = "traffic_light"
OBSTACLE = "obstacle"


class RedLight:
    """假红绿灯：red 时要求停车（RUNNING），否则什么都不做。"""

    name = TRAFFIC_LIGHT

    def __init__(self, red=False):
        self.red = red
        self.calls = 0

    def step(self, frame, now):
        self.calls += 1
        if self.red:
            return TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(),
                              message="holding red")
        return TaskUpdate(TaskStatus.NOT_TRIGGERED, message="no red light")


class Dodger:
    """假障碍任务：一旦接管就一直在侧移。"""

    name = OBSTACLE

    def __init__(self):
        self.calls = 0

    def step(self, frame, now):
        self.calls += 1
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(forward=0.20, lateral=-0.20, yaw=0.0),
            message="obstacle confirmed; stepping aside",
        )


class RedLightVetoTests(unittest.TestCase):
    def setUp(self):
        self.chassis = FakeChassis()
        self.follower = LineFollower(CONFIG)
        self.output = MotionOutput(self.chassis, CONFIG)
        self.gimbal_output = GimbalOutput(FakeGimbal(), CONFIG)
        self.light = RedLight()
        self.dodger = Dodger()
        self.coordinator = TaskCoordinator(
            CONFIG,
            self.follower,
            self.output,
            motion_tasks=(self.light, self.dodger),
            observers=(),
            gimbal_output=self.gimbal_output,
        )
        self.sequence = 0
        self.now = 100.0
        packet = FramePacket(line_frame(320), 1, self.now)
        self.follower.process_frame(packet.image, packet.captured_at)
        self.assertTrue(self.follower.resume(self.now), "巡线底座要能起来")

    def feed(self, step=1.0 / 30.0, image=None):
        self.now += step
        self.sequence += 1
        packet = FramePacket(line_frame(320) if image is None else image,
                             self.sequence, self.now)
        return self.coordinator.step(packet, self.now)

    def take_over(self):
        """在红灯没亮的情况下让障碍任务接管。"""
        for _ in range(8):
            self.feed()
            if self.coordinator.active_task is not None:
                return
        self.fail("障碍任务没能接管，测试前提不成立")

    # -- 1. 红灯能打断正在接管的任务 ------------------------------------
    def test_red_light_stops_the_car_while_a_task_owns_motion(self):
        self.take_over()
        self.assertEqual(self.coordinator.active_task_name, OBSTACLE)
        calls = self.dodger.calls
        self.light.red = True

        decision = self.feed()

        self.assertEqual(decision.command, MotionCommand(), "红灯时必须是停车指令")
        self.assertEqual(decision.command.lateral, 0.0)
        self.assertLessEqual(decision.command.forward, 0.0)
        self.assertEqual(self.dodger.calls, calls,
                         "红灯期间不该继续调用任务的 step（要真的停住它）")
        self.assertIsNotNone(self.coordinator.active_task, "任务被结束就错了")

    # -- 2. 不触发"硬停 + 人工复位" ------------------------------------
    def test_veto_does_not_need_a_human_reset(self):
        self.take_over()
        self.light.red = True
        decision = self.feed()

        self.assertNotEqual(self.coordinator.state, RELEASING,
                            "红灯否决不该走释放流程")
        self.assertFalse(decision.force_stop, "否决不是故障停车，不需要人工复位")
        self.assertIsNotNone(self.coordinator.active_task_name)

    # -- 3. 绿灯后交回原任务 -------------------------------------------
    def test_green_light_hands_control_back_to_the_task(self):
        self.take_over()
        self.light.red = True
        self.feed()
        paused_calls = self.dodger.calls
        self.light.red = False

        decision = self.feed()

        self.assertGreater(self.dodger.calls, paused_calls, "绿灯后任务要继续被调用")
        self.assertEqual(self.coordinator.active_task_name, OBSTACLE)
        self.assertEqual(decision.command.lateral, -0.20, "侧移指令要恢复")

    # -- 4. 等红灯不算任务超时 -----------------------------------------
    def test_waiting_for_a_red_light_does_not_time_the_task_out(self):
        self.take_over()
        self.light.red = True
        for _ in range(30):
            self.feed(step=1.0)  # 30 秒，远超 max_task_seconds=20
        self.assertIsNotNone(self.coordinator.active_task,
                             "等红灯的时间不能把任务耗到超时")

        self.light.red = False
        decision = self.feed()
        self.assertEqual(self.coordinator.active_task_name, OBSTACLE)
        self.assertNotIn("max_task_seconds", " ".join(decision.errors))

    # -- 5/6. 没有任务接管时，一切照旧 ---------------------------------
    def test_light_still_takes_over_by_itself_when_nobody_owns_motion(self):
        self.light.red = True
        self.feed()
        self.assertEqual(self.coordinator.active_task_name, TRAFFIC_LIGHT)

    def test_light_is_stepped_normally_when_it_owns_motion(self):
        self.light.red = True
        self.feed()
        calls = self.light.calls
        self.feed()
        self.assertGreater(self.light.calls, calls)
        self.assertEqual(self.coordinator.active_task_name, TRAFFIC_LIGHT)

    # -- 7. 诊断要写清"暂停了谁" ---------------------------------------
    def test_veto_message_names_the_paused_task(self):
        self.take_over()
        self.light.red = True
        decision = self.feed()
        text = "%s %s" % (decision.message, " ".join(decision.errors))
        self.assertIn(OBSTACLE, text)
        self.assertIn("red", text.lower())


if __name__ == "__main__":
    unittest.main()
