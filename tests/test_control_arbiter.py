"""仲裁器纯逻辑单测：优先级、TTL 过期、抢占资格、最小持有、tie-break、日志文本。

不用协调器、不用底盘、不用图像 —— 这一层是纯函数式判定，最容易钉死语义。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control_arbiter import (  # noqa: E402
    LINE_PRIORITY,
    SAFETY_PRIORITY,
    ControlArbiter,
    ControlRequest,
)
from models import MotionCommand  # noqa: E402


def req(module, priority, now, ttl=0.5, order=0, state="", command=None):
    return ControlRequest(
        module=module,
        priority=priority,
        command=MotionCommand(forward=0.1) if command is None else command,
        ttl=ttl,
        timestamp=now,
        state=state,
        order=order,
    )


class PriorityTests(unittest.TestCase):
    def test_highest_priority_wins_and_is_reported_as_a_change(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select([req("route", 60, now), req("obstacle", 70, now)], now)
        self.assertEqual(result.owner, "obstacle")
        self.assertTrue(result.changed)
        self.assertEqual(result.request.module, "obstacle")

    def test_same_priority_tie_break_is_deterministic_by_registry_order(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select(
            [req("later", 60, now, order=5), req("earlier", 60, now, order=2)], now
        )
        self.assertEqual(result.owner, "earlier")

    def test_safety_priority_constant_matches_the_table_value(self):
        self.assertEqual(SAFETY_PRIORITY, 90)
        self.assertEqual(LINE_PRIORITY, 10)


class TtlTests(unittest.TestCase):
    def test_expired_request_is_dropped(self):
        now = 10.0
        arbiter = ControlArbiter()
        stale = req("obstacle", 70, now - 5.0, ttl=0.5)
        result = arbiter.select([stale, req("route", 60, now)], now)
        self.assertEqual(result.owner, "route")

    def test_ttl_none_never_expires(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select([req("line", LINE_PRIORITY, 0.0, ttl=None)], now)
        self.assertEqual(result.owner, "line")

    def test_no_valid_request_releases_control(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select(
            [], now, current_owner="obstacle", current_owner_since=now - 1.0
        )
        self.assertIsNone(result.request)
        self.assertEqual(result.owner, "line")
        self.assertTrue(result.changed)
        self.assertEqual(result.reason, "no_valid_request")

    def test_no_valid_request_is_not_a_change_when_line_already_owns(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select([], now)
        self.assertFalse(result.changed)


class PreemptionTests(unittest.TestCase):
    def test_non_safety_challenger_cannot_preempt_by_default(self):
        """默认模式：priority 更高但非安全级的模块**不能**中途抢走控制权（行为保持）。"""
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select(
            [req("obstacle", 70, now), req("green_junction", 80, now)], now,
            current_owner="obstacle", current_owner_since=now - 1.0,
        )
        self.assertEqual(result.owner, "obstacle")
        self.assertFalse(result.changed)
        self.assertEqual(result.reason, "owner_keeps_control")

    def test_safety_priority_preempts_anyone(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select(
            [req("obstacle", 70, now), req("traffic_light", 90, now, state="holding red")],
            now, current_owner="obstacle", current_owner_since=now - 1.0,
        )
        self.assertEqual(result.owner, "traffic_light")
        self.assertTrue(result.changed)

    def test_preempt_all_allows_a_higher_priority_challenger(self):
        now = 10.0
        arbiter = ControlArbiter(preempt_all=True)
        result = arbiter.select(
            [req("route", 60, now), req("obstacle", 70, now)], now,
            current_owner="route", current_owner_since=now - 1.0,
        )
        self.assertEqual(result.owner, "obstacle")

    def test_preempt_margin_requires_a_strictly_higher_priority(self):
        now = 10.0
        arbiter = ControlArbiter(preempt_all=True, preempt_margin=15)
        result = arbiter.select(
            [req("route", 60, now), req("obstacle", 70, now)], now,
            current_owner="route", current_owner_since=now - 1.0,
        )
        self.assertEqual(result.owner, "route", "差 10 分不够越过 margin=15")

    def test_equal_priority_never_steals_from_the_current_owner(self):
        now = 10.0
        arbiter = ControlArbiter(preempt_all=True)
        result = arbiter.select(
            [req("route", 60, now, order=0), req("free_junction", 60, now, order=1)], now,
            current_owner="free_junction", current_owner_since=now - 1.0,
        )
        self.assertEqual(result.owner, "free_junction")
        self.assertFalse(result.changed)


class HysteresisTests(unittest.TestCase):
    def test_min_hold_blocks_a_non_safety_challenger(self):
        now = 10.0
        arbiter = ControlArbiter(preempt_all=True, min_hold_seconds=0.5)
        result = arbiter.select(
            [req("obstacle", 70, now), req("green_junction", 80, now)], now,
            current_owner="obstacle", current_owner_since=now - 0.1,
        )
        self.assertEqual(result.owner, "obstacle")
        self.assertEqual(result.reason, "min_hold")

    def test_min_hold_expires_and_the_challenger_takes_over(self):
        now = 10.0
        arbiter = ControlArbiter(preempt_all=True, min_hold_seconds=0.5)
        result = arbiter.select(
            [req("obstacle", 70, now), req("green_junction", 80, now)], now,
            current_owner="obstacle", current_owner_since=now - 0.9,
        )
        self.assertEqual(result.owner, "green_junction")

    def test_safety_priority_ignores_min_hold(self):
        now = 10.0
        arbiter = ControlArbiter(min_hold_seconds=0.5)
        result = arbiter.select(
            [req("obstacle", 70, now), req("traffic_light", 90, now)], now,
            current_owner="obstacle", current_owner_since=now - 0.1,
        )
        self.assertEqual(result.owner, "traffic_light")

    def test_current_owner_keeps_control_while_it_keeps_requesting(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select(
            [req("obstacle", 70, now, state="stepping aside")], now,
            current_owner="obstacle", current_owner_since=now - 5.0,
        )
        self.assertEqual(result.owner, "obstacle")
        self.assertFalse(result.changed)
        self.assertEqual(result.reason, "stepping aside")


class ChangeTextTests(unittest.TestCase):
    def test_change_text_reports_previous_new_reason_and_priorities(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select(
            [req("obstacle", 70, now, state="dodging")], now,
            current_owner="line", current_owner_since=now - 1.0,
        )
        text = arbiter.change_text("line", result)
        self.assertIn("[ARB] owner changed: line -> obstacle", text)
        self.assertIn("reason=dodging", text)
        self.assertIn("priority=70 > 10", text)

    def test_change_text_is_none_when_nothing_changed(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select(
            [req("obstacle", 70, now)], now,
            current_owner="obstacle", current_owner_since=now - 1.0,
        )
        self.assertIsNone(arbiter.change_text("obstacle", result, 70))

    def test_change_text_uses_the_previous_owner_priority(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select(
            [req("traffic_light", 90, now, state="holding red")], now,
            current_owner="obstacle", current_owner_since=now - 1.0,
        )
        text = arbiter.change_text("obstacle", result, 70)
        self.assertIn("priority=90 > 70", text)


if __name__ == "__main__":
    unittest.main()
