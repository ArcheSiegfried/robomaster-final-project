"""终端反馈：模块开始/结束运行、周期性"现在在跑谁"、以及**多个模块竞争控制权时的实况**。

为什么要有这个：现在的仲裁是"按注册表顺序问，遇到第一个 RUNNING 就停"——
**排在后面的模块那一帧根本没被问**，所以操作员看不到"还有谁想接管、最后为什么判给它"。
调优先级时最需要的就是这条信息。

做法：协调器**每 `claim_probe_seconds` 秒做一次"竞争探测"**（那一帧把所有模块都问一遍，
只记录它们的想法，胜负规则不变=仍是顺序里第一个 RUNNING）；探测帧的频率很低（默认 3 秒），
所以对模块内部计数的影响很小，而且可以关（设 0）。

只用假任务 + 假底盘 + 合成帧：不连车、不连相机。
"""

import io
import pathlib
import sys
import types
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dataclasses import replace  # noqa: E402

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
    LINE_FOLLOWING,
    RELEASING,
    TASK_ACTIVE,
    TaskCoordinator,
)
from models import FramePacket, MotionCommand, TaskStatus, TaskUpdate  # noqa: E402


class ScriptedTask:
    """一直返回同一个 TaskUpdate 的假任务。"""

    def __init__(self, name, status, message="", motion=None):
        self.name = name
        self._template = (status, message, motion)
        self.calls = 0

    def step(self, frame, now):
        self.calls += 1
        status, message, motion = self._template
        return TaskUpdate(status, motion=motion, message=message)


class MutableTask:
    """状态可改的假任务（用来模拟"先安静、探测帧才想接管"）。"""

    def __init__(self, name, status=TaskStatus.NOT_TRIGGERED, message="", motion=None):
        self.name = name
        self.status = status
        self.message = message
        self.motion = MotionCommand() if motion is None else motion
        self.calls = 0

    def step(self, frame, now):
        self.calls += 1
        return TaskUpdate(self.status, motion=self.motion, message=self.message)


def coordinator_with(*tasks, claim_probe_seconds=3.0):
    config = replace(CONFIG, tasks=replace(CONFIG.tasks,
                                           claim_probe_seconds=claim_probe_seconds))
    chassis = FakeChassis()
    follower = LineFollower(config)
    output = MotionOutput(chassis, config)
    coordinator = TaskCoordinator(config, follower, output, motion_tasks=tasks,
                                  observers=(), gimbal_output=GimbalOutput(FakeGimbal(), config))
    packet = FramePacket(line_frame(320), 1, 100.0)
    follower.process_frame(packet.image, packet.captured_at)
    assert follower.resume(100.0)
    return coordinator, follower


def feed(coordinator, now):
    return coordinator.step(FramePacket(line_frame(320), int(now * 30), now), now)


class ClaimProbeTests(unittest.TestCase):
    def test_probe_frame_asks_every_module_and_records_the_competition(self):
        light = MutableTask("traffic_light")
        marker = MutableTask("number_marker")
        quiet = MutableTask("route")
        coordinator, _ = coordinator_with(light, marker, quiet)

        feed(coordinator, 100.0)                      # 第一帧只计时，谁都不接管
        self.assertIsNone(coordinator.active_task)

        light.status = TaskStatus.RUNNING
        light.message = "holding red"
        marker.status = TaskStatus.RUNNING
        marker.message = "target found"
        feed(coordinator, 103.5)                      # 探测帧

        claims = {c["name"]: c["status"] for c in coordinator.last_claims}
        self.assertEqual(len(claims), 3, "探测帧必须把所有模块都问一遍")
        self.assertEqual(claims["traffic_light"], "RUNNING")
        self.assertEqual(claims["number_marker"], "RUNNING", "被顺序挡住的模块也要被记录")
        self.assertEqual(claims["route"], "NOT_TRIGGERED")
        self.assertEqual(coordinator.active_task_name, "traffic_light",
                         "胜负规则不变：顺序里第一个 RUNNING 仍然是赢家")

    def test_normal_frames_do_not_ask_the_later_modules(self):
        light = ScriptedTask("traffic_light", TaskStatus.RUNNING, "holding red",
                             MotionCommand())
        marker = ScriptedTask("number_marker", TaskStatus.RUNNING, "target found",
                              MotionCommand())
        coordinator, _ = coordinator_with(light, marker, claim_probe_seconds=3.0)

        feed(coordinator, 100.0)                      # 第一帧只计时
        marker_calls = marker.calls

        feed(coordinator, 100.5)                      # 普通帧
        self.assertEqual(coordinator.last_claims, (),
                         "普通帧不该报告竞争（否则终端每帧刷屏）")
        self.assertEqual(marker.calls, marker_calls,
                         "普通帧不该多问后面的模块（保持既有开销与状态推进）")

    def test_probe_is_periodic(self):
        light = ScriptedTask("traffic_light", TaskStatus.NOT_TRIGGERED, "no red")
        marker = ScriptedTask("number_marker", TaskStatus.NOT_TRIGGERED, "no target")
        coordinator, _ = coordinator_with(light, marker, claim_probe_seconds=3.0)

        feed(coordinator, 100.0)                      # 第一帧只计时
        self.assertEqual(coordinator.last_claims, ())
        feed(coordinator, 101.0)
        self.assertEqual(coordinator.last_claims, ())
        feed(coordinator, 103.5)
        self.assertTrue(coordinator.last_claims, "超过间隔后应当再探一次")

    def test_probe_can_be_switched_off(self):
        light = ScriptedTask("traffic_light", TaskStatus.NOT_TRIGGERED, "no red")
        marker = ScriptedTask("number_marker", TaskStatus.NOT_TRIGGERED, "no target")
        coordinator, _ = coordinator_with(light, marker, claim_probe_seconds=0.0)

        feed(coordinator, 100.0)
        self.assertEqual(coordinator.last_claims, ())
        feed(coordinator, 110.0)
        self.assertEqual(coordinator.last_claims, (), "关掉开关后永远不探测")

    def test_competition_is_not_reported_while_a_task_owns_motion(self):
        light = ScriptedTask("traffic_light", TaskStatus.RUNNING, "holding red",
                             MotionCommand())
        coordinator, _ = coordinator_with(light, claim_probe_seconds=3.0)
        feed(coordinator, 100.0)
        self.assertEqual(coordinator.state, TASK_ACTIVE)
        feed(coordinator, 100.2)
        self.assertEqual(coordinator.last_claims, (),
                         "有任务在开车时不该再刷竞争信息")


class ConsoleCompetitionTests(unittest.TestCase):
    def _console(self):
        import main as main_module
        stream = io.StringIO()
        console = main_module.ConsoleStatus(
            stream=stream, heartbeat_interval=3.0, task_heartbeat_interval=3.0,
            task_order=("traffic_light", "number_marker", "route"),
        )
        return console, stream

    def _decision(self, state=LINE_FOLLOWING, name=None, message=""):
        return types.SimpleNamespace(
            state=state, task_name=name, message=message,
            command=MotionCommand(), errors=(), line=types.SimpleNamespace(state="TRACKING"),
            task_update=None,
        )

    def test_console_prints_who_competed_and_who_won(self):
        console, stream = self._console()
        claims = (
            {"name": "traffic_light", "status": "RUNNING", "message": "holding red"},
            {"name": "number_marker", "status": "RUNNING", "message": "target found"},
            {"name": "route", "status": "NOT_TRIGGERED", "message": "no gap"},
        )
        console.update(self._decision(TASK_ACTIVE, "traffic_light", "holding red"),
                       1.0, 0, claims)
        text = stream.getvalue()
        self.assertIn("竞争", text)
        for name in ("traffic_light", "number_marker", "route"):
            self.assertIn(name, text, "竞争的各方都要列出来")
        self.assertIn("判给", text)
        self.assertIn("traffic_light", text.split("判给", 1)[1][:40])

    def test_console_says_so_when_nobody_wants_control(self):
        console, stream = self._console()
        claims = ({"name": "traffic_light", "status": "NOT_TRIGGERED", "message": ""},)
        console.update(self._decision(), 1.0, 0, claims)
        self.assertIn("无人", stream.getvalue())

    def test_module_start_and_end_are_explicit(self):
        console, stream = self._console()
        console.update(self._decision(TASK_ACTIVE, "obstacle", "stepping aside"), 1.0)
        console.update(self._decision(RELEASING, "obstacle", "task completed"), 5.0)
        text = stream.getvalue()
        self.assertIn("模块开始运行", text)
        self.assertIn("模块结束运行", text)
        self.assertIn("obstacle", text)

    def test_heartbeat_states_the_current_owner(self):
        console, stream = self._console()
        console.update(self._decision(), 0.0)
        console.update(self._decision(), 3.2)
        text = stream.getvalue()
        self.assertIn("--", text)
        self.assertIn("巡线", text, "心跳里要写清现在是谁在开车（巡线也算）")


if __name__ == "__main__":
    unittest.main()
