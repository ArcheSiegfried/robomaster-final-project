"""仲裁层集成测试：谁能开车、谁能抢占、释放后谁能回来、同优先级不抖动。

用假模块（`StubTask` 按脚本逐帧返回状态）+ 真协调器 + 假底盘 + 合成帧。
不跑 main.py、不连硬件。

注意：这里**不用** `TaskHarness.start_line()` —— 它会自己喂一帧给协调器，会让
"脚本第 1 帧"错位。这里手动把巡线拉到可接管状态，第一帧完全由测试控制。
"""

import dataclasses
import pathlib
import sys
import time
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from models import STOP_COMMAND, MotionCommand, TaskStatus, TaskUpdate  # noqa: E402
from tests.task_harness import CONFIG, TaskHarness  # noqa: E402

RUN = (TaskStatus.RUNNING, MotionCommand(forward=0.2, lateral=-0.2))
STOP = (TaskStatus.RUNNING, STOP_COMMAND)
IDLE = (TaskStatus.NOT_TRIGGERED, None)
DONE = (TaskStatus.COMPLETED, MotionCommand())


class StubTask:
    """按脚本返回状态；脚本用完后一直重复最后一个元素。"""

    def __init__(self, name, script, priority):
        self.name = name
        self.priority = priority
        self.script = list(script)
        self.calls = 0
        self.resets = 0

    def reset(self):
        self.resets += 1

    def step(self, frame, now):
        index = min(self.calls, len(self.script) - 1)
        self.calls += 1
        status, motion = self.script[index]
        return TaskUpdate(status=status, motion=motion, message="%s#%d" % (self.name, self.calls))


def settings_with(**tasks_overrides):
    """注意：`TaskConfig` 里还没有 `preempt_all` 字段（config.py 属"基础底座"，本次不动），
    所以生产环境要打开抢占需要在 config.py 里加一行；测试直接拨仲裁器的开关。"""
    return dataclasses.replace(
        CONFIG, tasks=dataclasses.replace(CONFIG.tasks, **tasks_overrides)
    )


class ArbitrationCase(unittest.TestCase):
    def make_harness(self, tasks, config=CONFIG, now=1.0):
        """起巡线但**不喂帧**：让测试脚本的第 1 帧就是协调器看到的第 1 帧。"""
        harness = TaskHarness(tasks=tuple(tasks), config=config)
        packet = harness.frame(now, 320)
        harness.follower.process_frame(packet.image, packet.captured_at)
        self.assertTrue(harness.follower.resume(packet.captured_at), "巡线底座要能起来")
        harness.now = now
        return harness

    def feed(self, harness, step=1.0 / 30.0):
        harness.now += step
        return harness.feed_line(harness.now)


class ArbitrationTests(ArbitrationCase):
    def test_line_owns_motion_when_nobody_claims(self):
        obstacle = StubTask("obstacle", [IDLE], 70)
        harness = self.make_harness((obstacle,))
        decision = self.feed(harness)
        self.assertEqual(harness.owner, "line")
        self.assertEqual(harness.coordinator.motion_owner, "line")
        self.assertIsNone(decision.task_name)
        self.assertIsNone(decision.owner_change, "没换人就不该有日志")

    def test_highest_priority_claimant_wins_from_the_line(self):
        obstacle = StubTask("obstacle", [RUN], 70)
        route = StubTask("route", [RUN], 60)
        harness = self.make_harness((route, obstacle))
        decision = self.feed(harness)
        self.assertEqual(decision.task_name, "obstacle", "优先级高的先赢")
        self.assertEqual(harness.coordinator.motion_owner, "obstacle")
        self.assertIn("[ARB] owner changed: line -> obstacle", decision.owner_change)

    def test_safety_priority_freezes_the_owner_and_stops_the_car(self):
        obstacle = StubTask("obstacle", [RUN], 70)
        light = StubTask("traffic_light", [IDLE, STOP], 90)
        harness = self.make_harness((obstacle, light))
        self.feed(harness)
        self.assertEqual(harness.coordinator.active_task_name, "obstacle")
        calls = obstacle.calls

        decision = self.feed(harness)                 # 红灯亮

        self.assertEqual(decision.command, STOP_COMMAND, "红灯必须停车")
        self.assertEqual(obstacle.calls, calls, "红灯期间 owner 不许继续被 step")
        self.assertEqual(harness.coordinator.active_task_name, "obstacle", "任务不许被结束")
        self.assertEqual(harness.coordinator.motion_owner, "traffic_light")
        self.assertIn("obstacle", decision.message)
        self.assertIn("red", decision.message.lower())
        self.assertIn("[ARB] owner changed: obstacle -> traffic_light", decision.owner_change)

    def test_owner_resumes_when_the_safety_claim_clears(self):
        obstacle = StubTask("obstacle", [RUN], 70)
        light = StubTask("traffic_light", [IDLE, STOP, IDLE], 90)
        harness = self.make_harness((obstacle, light))
        self.feed(harness)
        self.feed(harness)                            # 红灯
        calls = obstacle.calls

        decision = self.feed(harness)                 # 绿灯

        self.assertGreater(obstacle.calls, calls, "绿灯后原任务继续被调用")
        self.assertEqual(decision.task_name, "obstacle")
        self.assertEqual(decision.command.lateral, -0.2, "侧移指令要恢复")
        self.assertEqual(harness.coordinator.motion_owner, "obstacle")
        self.assertIn("[ARB] owner changed: traffic_light -> obstacle", decision.owner_change)

    def test_non_safety_challenger_is_not_even_asked_while_another_task_drives(self):
        """默认模式（preempt_all=False）：优先级更高的非安全级模块不能中途抢占。"""
        obstacle = StubTask("obstacle", [RUN], 70)
        junction = StubTask("green_junction", [IDLE, RUN], 80)
        harness = self.make_harness((obstacle, junction))
        self.feed(harness)
        self.assertEqual(harness.coordinator.active_task_name, "obstacle")
        calls = junction.calls

        decision = self.feed(harness)

        self.assertEqual(junction.calls, calls, "连问都不该问它")
        self.assertEqual(decision.task_name, "obstacle")

    def test_preempt_all_lets_a_higher_priority_task_take_over(self):
        obstacle = StubTask("obstacle", [RUN], 70)
        junction = StubTask("green_junction", [IDLE, RUN], 80)
        harness = self.make_harness((obstacle, junction))
        harness.coordinator.arbiter.preempt_all = True
        self.feed(harness)
        self.assertEqual(harness.coordinator.active_task_name, "obstacle")

        decision = None
        for _ in range(8):                            # 越过 min_hold_seconds=0.2
            decision = self.feed(harness)

        self.assertEqual(decision.task_name, "green_junction")
        self.assertEqual(obstacle.resets, 0, "被抢占的模块不许被 reset")

    def test_owner_that_stops_requesting_loses_control(self):
        obstacle = StubTask("obstacle", [RUN, IDLE], 70)
        harness = self.make_harness((obstacle,))
        self.feed(harness)

        decision = self.feed(harness)

        self.assertTrue(decision.force_stop, "失去有效请求必须硬停并交回巡线")
        self.assertEqual(harness.coordinator.motion_owner, "line")
        self.assertIn("NOT_TRIGGERED", " ".join(decision.errors))

    def test_three_modules_do_not_alternate(self):
        light = StubTask("traffic_light", [STOP], 90)
        obstacle = StubTask("obstacle", [RUN], 70)
        route = StubTask("route", [RUN], 60)
        harness = self.make_harness((route, obstacle, light))
        winners = {self.feed(harness).task_name for _ in range(6)}
        self.assertEqual(winners, {"traffic_light"}, "一帧只有一个赢家，且不许交替")

    def test_completed_task_releases_and_the_next_task_resumes(self):
        obstacle = StubTask("obstacle", [RUN, DONE], 70)
        route = StubTask("route", [RUN], 60)
        harness = self.make_harness((obstacle, route))
        self.feed(harness)
        self.assertEqual(harness.coordinator.active_task_name, "obstacle")

        release = self.feed(harness)
        self.assertTrue(release.force_stop)
        self.assertEqual(release.task_name, "obstacle", "释放那一帧仍报是谁结束了")

        owner = None
        for _ in range(60):
            owner = self.feed(harness)
            if owner.task_name:
                break
        self.assertEqual(owner.task_name, "route", "高优先级结束后该轮到剩下的模块")

    def test_probe_frame_records_every_module_with_its_priority(self):
        obstacle = StubTask("obstacle", [IDLE], 70)
        route = StubTask("route", [IDLE], 60)
        harness = self.make_harness((route, obstacle))
        self.feed(harness)
        self.assertEqual(harness.coordinator.last_claims, ())
        harness.now = 4.5                             # 越过 claim_probe_seconds=3.0
        self.feed(harness, step=0.0)
        rows = {row["name"]: row for row in harness.coordinator.last_claims}
        self.assertEqual(set(rows), {"route", "obstacle"})
        self.assertEqual(rows["obstacle"]["priority"], 70)
        self.assertEqual(rows["route"]["priority"], 60)


class BudgetAndLeaseTests(ArbitrationCase):
    def test_lease_is_renewed_while_the_owner_keeps_running(self):
        obstacle = StubTask("obstacle", [RUN], 70)
        harness = self.make_harness((obstacle,))
        decision = None
        for _ in range(20):
            decision = self.feed(harness)
        self.assertEqual(decision.task_name, "obstacle", "每帧都续租，不该被租约踢掉")
        self.assertFalse(decision.force_stop)

    def test_repeated_slow_steps_force_a_release(self):
        class SlowTask(StubTask):
            def step(self, frame, now):
                time.sleep(0.05)  # 超过 max_step_seconds=0.02
                return super().step(frame, now)

        obstacle = SlowTask("obstacle", [RUN], 70)
        harness = self.make_harness((obstacle,))
        decisions = [self.feed(harness) for _ in range(7)]

        self.assertTrue(
            any(decision.force_stop for decision in decisions),
            "连续超单步预算必须强制释放（不能只记一条 error 就算了）",
        )
        self.assertTrue(
            any("per-step budget" in " ".join(decision.errors) for decision in decisions)
        )
        releases = [
            decision
            for decision in decisions
            if decision.force_stop and "per-step budget" in " ".join(decision.errors)
        ]
        self.assertEqual(len(releases), 1, "只该强制释放一次")
        self.assertEqual(obstacle.resets, 1, "强制释放必须 reset 掉卡住的任务")


class RecordingTask(StubTask):
    """额外记录它每次被调用时拿到的 `now`（任务时钟就是被这样观测的）。"""

    def __init__(self, name, script, priority):
        super().__init__(name, script, priority)
        self.seen = []

    def step(self, frame, now):
        self.seen.append(now)
        return super().step(frame, now)


class TaskClockTests(ArbitrationCase):
    def test_a_frozen_owner_does_not_age_while_the_red_light_holds(self):
        obstacle = RecordingTask("obstacle", [RUN], 70)
        light = StubTask("traffic_light", [IDLE, STOP], 90)
        harness = self.make_harness((obstacle, light))
        self.feed(harness)                       # 障碍接管，记下它的时钟
        first = obstacle.seen[-1]

        for _ in range(40):                      # 连续 2 秒红灯
            self.feed(harness, step=0.05)

        self.assertEqual(obstacle.calls, 1, "红灯期间 owner 不该被调用")
        self.assertGreater(harness.now - first, 1.9, "前提：真实时间确实过了约 2 秒")

        light.script = [IDLE]
        self.feed(harness)                       # 绿灯，原任务继续

        self.assertGreater(obstacle.calls, 1)
        self.assertLess(
            obstacle.seen[-1] - first,
            0.5,
            "被红灯冻结的 2 秒不许计进它的时钟（否则模块会被自己的墙钟超时判死）",
        )

    def test_a_skipped_module_does_not_age_on_the_wall_clock(self):
        obstacle = StubTask("obstacle", [IDLE] + [RUN] * 200, 70)
        route = RecordingTask("route", [IDLE], 60)
        harness = self.make_harness((route, obstacle))
        self.feed(harness)                       # 这一帧无人接管，route 被问一次
        first = route.seen[-1]
        self.assertEqual(route.calls, 1)

        for _ in range(40):                      # 障碍接管 2 秒，route 完全不被问
            self.feed(harness, step=0.05)

        self.assertEqual(route.calls, 1, "别人开车时它不该被问")
        obstacle.script = [DONE]                 # 障碍结束 -> 释放 -> route 重新被问
        for _ in range(60):
            self.feed(harness)
            if route.calls > 1:
                break

        self.assertGreater(route.calls, 1, "释放后它应该重新被问")
        self.assertLess(
            route.seen[-1] - first, 0.5, "被跳过的 2 秒不许计进它的时钟"
        )

    def test_task_clock_tracks_real_time_for_a_continuously_called_module(self):
        route = RecordingTask("route", [IDLE], 60)
        harness = self.make_harness((route,))
        for _ in range(20):                      # 一直没人接管 -> route 每帧都被问
            self.feed(harness)
        self.assertAlmostEqual(
            route.seen[-1] - route.seen[0], 19.0 / 30.0, places=3
        )


if __name__ == "__main__":
    unittest.main()
