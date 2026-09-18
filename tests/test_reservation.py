"""预约接管通道：排在最后、按顺序永远轮不到的模块也要有机会拿运动权。

为什么要有这个文件（2026-09-18 实车证据，captures/run_20260918_163210）：
仲裁是"按顺序问、遇到第一个 RUNNING 就停"，所以前面的模块一强触发，
后面的模块那一帧**根本不会被问**。那次 run 196 秒里：

    green_junction  接管 1 次
    obstacle        接管 0 次   （被截断 1 次）
    free_junction   接管 5 次
    route           接管 2 次
    traffic_light   接管 1 次
    number_marker   接管 0 次   （被截断 9 次）

结果一张标识照片都没存 —— 标识 5 分/个、满 25 分，是全场最大一块。
只调注册表顺序救不了：把 number_marker 提前就会反过来挡住岔路/绕障。

所以另开一条通道：模块可以实现**只读**的 wants_control(frame, now)，
协调器在"按顺序问"之前先问它们一遍。

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
    RESERVATION_MIN_HOLD_SECONDS,
    TaskCoordinator,
)
from models import (  # noqa: E402
    FramePacket,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
)


class GreedyTask:
    """排第一、每帧都想接管（模拟 free_junction 那种强触发模块）。"""

    def __init__(self, name="greedy"):
        self.name = name
        self.calls = 0
        self.owns = 0

    def step(self, frame, now):
        self.calls += 1
        self.owns += 1
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(forward=0.10, yaw=-5.0),
            message="greedy branch",
        )


class ReservedTask:
    """排最后、只有预约通道才可能被问到的模块（模拟 number_marker）。"""

    def __init__(self, name="reserved", wants=True):
        self.name = name
        self.wants = wants
        self.probes = 0
        self.calls = 0

    def wants_control(self, frame, now):
        self.probes += 1
        return self.wants

    def step(self, frame, now):
        self.calls += 1
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(forward=0.0, yaw=0.0),
            message="reserved took over",
        )


class PlainTask:
    """没有 wants_control 的普通模块：行为必须和以前完全一样。"""

    def __init__(self, name="plain"):
        self.name = name
        self.calls = 0

    def step(self, frame, now):
        self.calls += 1
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(forward=0.05),
            message="plain",
        )


class ReservationTests(unittest.TestCase):
    def setUp(self):
        self.sequence = 0
        self.now = 100.0

    def build(self, tasks, gimbal=True):
        chassis = FakeChassis()
        follower = LineFollower(CONFIG)
        output = MotionOutput(chassis, CONFIG)
        coordinator = TaskCoordinator(
            CONFIG,
            follower,
            output,
            motion_tasks=tuple(tasks),
            observers=(),
            gimbal_output=GimbalOutput(FakeGimbal(), CONFIG) if gimbal else None,
        )
        packet = FramePacket(line_frame(320), 1, self.now)
        follower.process_frame(packet.image, packet.captured_at)
        self.assertTrue(follower.resume(self.now), "巡线底座要能起来")
        return coordinator, follower, chassis

    def feed(self, coordinator, frames=1, step=1.0 / 30.0):
        decision = None
        for _ in range(frames):
            self.now += step
            self.sequence += 1
            decision = coordinator.step(
                FramePacket(line_frame(320), self.sequence, self.now), self.now
            )
        return decision

    # -- 1. 核心：排最后的模块也能拿到运动权 ---------------------------
    def test_a_reserved_last_module_can_take_over(self):
        greedy = GreedyTask()
        reserved = ReservedTask()
        coordinator, _, _ = self.build((greedy, reserved))

        # 第一帧：greedy 按顺序先接管
        self.feed(coordinator)
        self.assertEqual(coordinator.active_task_name, "greedy")

        # 跑够 min_hold 之后，预约模块把它抢走
        self.feed(coordinator, frames=int(RESERVATION_MIN_HOLD_SECONDS * 30) + 3)
        self.assertEqual(coordinator.active_task_name, "reserved",
                         "预约通道没生效：排在最后的模块永远拿不到运动权")
        self.assertGreater(reserved.probes, 0, "wants_control 应该被调用过")

    def test_without_the_hook_nothing_changes(self):
        """普通模块没有 wants_control，仲裁行为必须和以前一模一样。"""
        greedy = GreedyTask()
        plain = PlainTask()
        coordinator, _, _ = self.build((greedy, plain))
        self.feed(coordinator, frames=90)
        self.assertEqual(coordinator.active_task_name, "greedy")
        self.assertEqual(plain.calls, 0, "没有钩子的模块不该被提前问")
        self.assertEqual(len(coordinator.reservation_tasks), 0)

    def test_a_reserved_module_that_does_not_want_is_not_asked_to_step(self):
        greedy = GreedyTask()
        reserved = ReservedTask(wants=False)
        coordinator, _, _ = self.build((greedy, reserved))
        self.feed(coordinator, frames=90)
        self.assertEqual(coordinator.active_task_name, "greedy")
        self.assertEqual(reserved.calls, 0, "不想接管就不该调它的 step()")
        self.assertGreater(reserved.probes, 0, "但应该被问过")

    def test_ordering_still_wins_when_nobody_owns_motion(self):
        """没人开车时仍按注册表顺序裁定：预约模块**不许**抢在岔路前面。

        这条是设计约束：预约通道只解决"已经有模块拿着运动权"造成的饿死，
        不改变"岔路优先于标识"这类裁定。否则标识一出现就会挡住岔路/绕障。
        """
        reserved = ReservedTask(name="reserved")
        greedy = GreedyTask(name="greedy")
        # 顺序故意把 reserved 放前面，看它是不是靠注册表顺序赢 —— 是则正常。
        coordinator, _, _ = self.build((reserved, greedy))
        self.feed(coordinator, frames=3)
        self.assertEqual(coordinator.active_task_name, "reserved",
                         "没人开车时应当按注册表顺序，而不是被预约打乱")

    def test_reserved_module_does_not_preempt_a_fork_seen_first(self):
        """更接近实战：岔路模块排在前、标识模块排在后，岔路先接管就该它开。"""
        greedy = GreedyTask(name="green_junction")
        reserved = ReservedTask(name="number_marker")
        coordinator, _, _ = self.build((greedy, reserved))
        # 岔路先被问到并接管
        self.feed(coordinator)
        self.assertEqual(coordinator.active_task_name, "green_junction")
        # 接管那一帧还没到 min_hold，所以钩子这时**还不该**被问
        self.assertEqual(reserved.probes, 0,
                         "接管当帧不该去问预约钩子（min_hold 还没到）")

    # -- 2. 不抢自己、不每帧互抢 ---------------------------------------
    def test_the_active_reserved_task_keeps_control(self):
        reserved = ReservedTask(name="first_reserved")
        other = ReservedTask(name="second_reserved")
        coordinator, _, _ = self.build((reserved, other))
        self.feed(coordinator, frames=60)
        self.assertEqual(coordinator.active_task_name, "first_reserved",
                         "预约模块自己在开车时不该被别的预约抢走")

    def test_handover_only_after_min_hold(self):
        """刚接管的模块不会被立刻抢走（否则两模块每帧互抢）。"""
        greedy = GreedyTask()
        reserved = ReservedTask()
        coordinator, _, _ = self.build((greedy, reserved))
        self.feed(coordinator)                      # greedy 接管
        self.assertEqual(coordinator.active_task_name, "greedy")
        self.feed(coordinator, frames=5)            # 还没到 min_hold
        self.assertEqual(coordinator.active_task_name, "greedy",
                         f"没跑够 {RESERVATION_MIN_HOLD_SECONDS}s 不该被抢")

    # -- 3. 安全：红灯否决优先，限幅与超时照旧 -------------------------
    def test_a_red_light_still_beats_the_reservation(self):
        """红灯否决权必须优先于预约：红灯亮着时不许被预约抢去开车。"""
        class RedLight:
            name = "traffic_light"

            def __init__(self):
                self.red = False
                self.calls = 0

            def step(self, frame, now):
                self.calls += 1
                if self.red:
                    return TaskUpdate(TaskStatus.RUNNING,
                                      motion=MotionCommand(),
                                      message="holding red")
                return TaskUpdate(TaskStatus.NOT_TRIGGERED)

        greedy = GreedyTask()
        reserved = ReservedTask()
        light = RedLight()
        # 顺序：greedy 先接管；light 通过名字参与否决；reserved 靠预约
        coordinator, _, _ = self.build((greedy, reserved, light))
        self.feed(coordinator)
        self.assertEqual(coordinator.active_task_name, "greedy")

        light.red = True
        decision = self.feed(coordinator, frames=int(RESERVATION_MIN_HOLD_SECONDS * 30) + 5)
        self.assertEqual(decision.command, MotionCommand(), "红灯时必须停车")
        self.assertNotEqual(coordinator.active_task_name, "reserved",
                            "红灯期间预约模块不许抢走运动权")

    def test_reserved_command_is_still_clamped(self):
        """预约接管同样受 TaskConfig 限幅，不能绕过安全护栏。"""
        class OverspeedReserved(ReservedTask):
            def step(self, frame, now):
                self.calls += 1
                return TaskUpdate(
                    TaskStatus.RUNNING,
                    motion=MotionCommand(forward=5.0, lateral=5.0, yaw=900.0),
                    message="way too fast",
                )

        greedy = GreedyTask()
        reserved = OverspeedReserved()
        coordinator, _, chassis = self.build((greedy, reserved))
        self.feed(coordinator, frames=int(RESERVATION_MIN_HOLD_SECONDS * 30) + 5)
        self.assertEqual(coordinator.active_task_name, "reserved")
        last = chassis.last_speed()
        self.assertIsNotNone(last)
        self.assertLessEqual(abs(last.get("x", 0.0)), CONFIG.tasks.task_max_forward)
        self.assertLessEqual(abs(last.get("y", 0.0)), CONFIG.tasks.task_max_lateral)
        self.assertLessEqual(abs(last.get("z", 0.0)), CONFIG.tasks.task_max_yaw)

    def test_reservation_is_ignored_when_takeover_is_not_allowed(self):
        """巡线 STOPPED 时不许接管 —— 预约通道也不能自己把车启动起来。"""
        greedy = GreedyTask()
        reserved = ReservedTask()
        coordinator, follower, _ = self.build((greedy, reserved))
        follower.pause(self.now)                    # 人工暂停
        self.feed(coordinator, frames=60)
        self.assertIsNone(coordinator.active_task_name,
                          "STOPPED 状态下任何模块都不许接管")
        self.assertEqual(reserved.calls, 0)

    def test_a_broken_hook_does_not_break_the_loop(self):
        class Exploding(ReservedTask):
            def wants_control(self, frame, now):
                raise RuntimeError("synthetic: hook on fire")

        greedy = GreedyTask()
        exploding = Exploding()
        coordinator, _, _ = self.build((greedy, exploding))
        # 要跑过 min_hold 才会去问钩子，所以这里必须跑够时间。
        decision = self.feed(
            coordinator, frames=int(RESERVATION_MIN_HOLD_SECONDS * 30) + 5
        )
        self.assertEqual(coordinator.active_task_name, "greedy")
        self.assertTrue(any("hook on fire" in error for error in decision.errors),
                        f"钩子异常要记进 errors：{decision.errors}")


if __name__ == "__main__":
    unittest.main()
