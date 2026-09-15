"""Offline contract and behaviour tests for long-gap route recovery."""

import pathlib
import sys
import unittest

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from coordinator import LINE_FOLLOWING, RELEASING, TASK_ACTIVE  # noqa: E402
from models import TaskStatus  # noqa: E402
from route import (  # noqa: E402
    ALIGNING,
    BRIDGE_FORWARD_SPEED,
    BRIDGE_MIN_SECONDS,
    BRIDGE_SLOW_SPEED,
    END_APPROACH,
    END_APPROACH_SPEED,
    MONITORING,
    SEARCH_HARD_LIMIT_DEG,
    SEARCH_YAW_SPEED,
    TOTAL_RECOVERY_SECONDS,
    RouteTask,
)
from route_detector import RouteVision  # noqa: E402
from config import CONFIG  # noqa: E402
from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
)


def segment_frame(start, end, thickness=18, height=360, width=640):
    image = np.full((height, width, 3), 210, np.uint8)
    cv2.line(image, start, end, (255, 0, 0), thickness)
    return image


def far_fragment_frame(x=400):
    return segment_frame((x, 210), (x, 85))


def near_route_frame(x=320):
    return segment_frame((x, 355), (x, 120), thickness=24)


def near_old_line_frame(x=400):
    """Old route remains only in the bottom band; base detector is invalid."""
    return segment_frame((x, 355), (x, 300), thickness=24)


def angled_far_frame():
    return segment_frame((390, 220), (470, 85))


def blue_square_frame():
    image = np.full((360, 640, 3), 210, np.uint8)
    cv2.rectangle(image, (250, 100), (390, 240), (255, 0, 0), -1)
    return image


def old_and_new_frame():
    image = np.full((360, 640, 3), 210, np.uint8)
    cv2.line(image, (100, 355), (100, 285), (255, 0, 0), 18)
    cv2.line(image, (380, 210), (450, 90), (255, 0, 0), 18)
    return image


def start_and_trigger(x=400):
    task = RouteTask()
    harness = TaskHarness(task=task)
    harness.start_line(now=1.0, x=x)
    before_grace = harness.feed_blank(1.30)
    assert before_grace.state == LINE_FOLLOWING
    takeover = harness.feed_blank(1.36)
    assert takeover.state == TASK_ACTIVE
    return task, harness, takeover


def settle_into_bridge(harness):
    """Wait longer than the calculated 20deg/30deg/s gimbal movement."""
    decision = harness.feed_blank(2.16)
    assert decision.command.forward == BRIDGE_FORWARD_SPEED
    return decision


class RouteContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        assert_module_source_is_clean(self, "route.py")
        assert_module_source_is_clean(self, "route_detector.py")

    def test_does_not_take_over_on_a_clear_line_frame(self):
        assert_inert_through_harness(self, RouteTask())


class RouteVisionTests(unittest.TestCase):
    def setUp(self):
        self.vision = RouteVision(CONFIG.vision)

    def test_one_ended_far_and_near_segments_are_detected(self):
        far = self.vision.candidates(far_fragment_frame())
        near = self.vision.candidates(near_route_frame())
        self.assertEqual(len(far), 1)
        self.assertFalse(far[0].near)
        self.assertEqual(len(near), 1)
        self.assertTrue(near[0].near)

    def test_angle_is_reported_for_an_oblique_new_route(self):
        candidate = self.vision.candidates(angled_far_frame())[0]
        self.assertGreater(candidate.angle_deg, 20.0)
        self.assertLess(candidate.angle_deg, 50.0)

    def test_square_blue_object_is_not_a_route_fragment(self):
        self.assertEqual(self.vision.candidates(blue_square_frame()), [])

    def test_new_far_fragment_outranks_visible_old_near_line(self):
        candidates = self.vision.candidates(old_and_new_frame())
        self.assertGreaterEqual(len(candidates), 2)
        self.assertFalse(candidates[0].near)
        self.assertGreater(candidates[0].detection.center[0], 300)


class RouteRecoveryTests(unittest.TestCase):
    def test_waits_beyond_base_grace_then_raises_view_while_stopped(self):
        _, harness, takeover = start_and_trigger()
        self.assertEqual(takeover.command.forward, 0.0)
        self.assertEqual(takeover.command.yaw, 0.0)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -5.0)
        self.assertEqual(harness.owner, "external")

    def test_far_loss_with_bottom_line_crawls_to_physical_endpoint(self):
        task = RouteTask()
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0, x=400)

        pending = harness.feed_image(1.05, near_old_line_frame())
        takeover = harness.feed_image(1.15, near_old_line_frame())
        self.assertEqual(pending.state, LINE_FOLLOWING)
        self.assertEqual(takeover.state, TASK_ACTIVE)
        self.assertEqual(takeover.command.forward, END_APPROACH_SPEED)
        self.assertIsNone(harness.gimbal.last_move())

        first_blank = harness.feed_blank(1.20)
        confirmed = harness.feed_blank(1.36)
        self.assertEqual(first_blank.command.forward, 0.0)
        self.assertEqual(confirmed.command.forward, 0.0)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -5.0)

    def test_endpoint_after_old_two_point_five_second_budget_still_recovers(self):
        """Real feedback: far sampling can vanish >0.20 m before line end."""
        task = RouteTask()
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0, x=400)
        harness.feed_image(1.05, near_old_line_frame())
        harness.feed_image(1.15, near_old_line_frame())

        # This is already beyond the previous 2.5 s phase limit.  The task
        # must still follow the visible bottom line instead of failing.
        after_old_limit = harness.feed_image(3.80, near_old_line_frame())
        self.assertEqual(after_old_limit.state, TASK_ACTIVE)
        self.assertEqual(task.state, END_APPROACH)
        self.assertEqual(after_old_limit.command.forward, END_APPROACH_SPEED)

        harness.feed_blank(3.85)
        raising = harness.feed_blank(4.01)
        self.assertEqual(raising.state, TASK_ACTIVE)
        self.assertEqual(raising.command.forward, 0.0)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -5.0)

        bridge = harness.feed_blank(4.81)
        self.assertEqual(bridge.command.forward, BRIDGE_FORWARD_SPEED)
        harness.feed_blank(7.82)
        search = harness.feed_blank(7.87)
        self.assertEqual(search.command.forward, 0.0)
        self.assertEqual(search.command.yaw, SEARCH_YAW_SPEED)

    def test_blank_after_raise_crosses_a_bounded_distance(self):
        task, harness, _ = start_and_trigger()
        bridge = settle_into_bridge(harness)
        self.assertEqual(bridge.command.forward, BRIDGE_FORWARD_SPEED)
        self.assertEqual(bridge.command.yaw, 0.0)

        harness.feed_blank(2.36)
        harness.feed_blank(2.56)
        self.assertGreater(task.estimated_forward_progress, 0.0)
        ended = harness.feed_blank(5.20)
        self.assertEqual(ended.command.forward, 0.0)
        search = harness.feed_blank(5.25)
        self.assertEqual(search.command.forward, 0.0)
        self.assertEqual(search.command.yaw, SEARCH_YAW_SPEED)

    def test_search_is_bounded_and_reverses_across_saved_heading(self):
        task, harness, _ = start_and_trigger(x=400)
        settle_into_bridge(harness)
        harness.feed_blank(5.20)
        commands = []
        now = 5.25
        while now < 15.0 and harness.state == TASK_ACTIVE:
            decision = harness.feed_blank(now)
            commands.append(decision.command.yaw)
            self.assertLessEqual(
                abs(task.estimated_heading_offset), SEARCH_HARD_LIMIT_DEG
            )
            now += 0.20
        self.assertTrue(any(yaw > 0.0 for yaw in commands))
        self.assertTrue(any(yaw < 0.0 for yaw in commands))
        self.assertTrue(all(abs(yaw) <= SEARCH_YAW_SPEED for yaw in commands))

    def test_far_fragment_needs_confirmation_then_approaches_slowly(self):
        _, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        first = harness.feed_image(3.01, far_fragment_frame())
        second = harness.feed_image(3.06, far_fragment_frame())
        confirmed = harness.feed_image(3.11, far_fragment_frame())
        approach = harness.feed_image(3.16, far_fragment_frame())
        self.assertEqual(first.command.forward, BRIDGE_FORWARD_SPEED)
        self.assertEqual(second.command.forward, BRIDGE_FORWARD_SPEED)
        self.assertEqual(confirmed.command.forward, 0.0)
        self.assertEqual(approach.command.forward, BRIDGE_SLOW_SPEED)
        self.assertGreater(approach.command.yaw, 0.0)

    def test_candidate_cannot_interrupt_initial_old_route_clearance(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        decisions = [
            harness.feed_image(2.21 + index * 0.05, near_route_frame())
            for index in range(3)
        ]
        self.assertNotEqual(task.state, ALIGNING)
        self.assertTrue(
            any("initial old-line clearance" in item.message for item in decisions)
        )
        self.assertTrue(
            all(item.command.forward == BRIDGE_FORWARD_SPEED for item in decisions)
        )
        self.assertGreater(BRIDGE_MIN_SECONDS, 0.0)

    def test_oblique_route_uses_center_and_heading_to_turn_towards_it(self):
        _, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        for now in (3.01, 3.06, 3.11):
            harness.feed_image(now, angled_far_frame())
        approach = harness.feed_image(3.16, angled_far_frame())
        self.assertEqual(approach.command.forward, BRIDGE_SLOW_SPEED)
        self.assertGreater(approach.command.yaw, 0.0)
        self.assertLessEqual(abs(approach.command.yaw), 26.0)

    def test_blue_square_never_becomes_a_route_candidate(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        decisions = [
            harness.feed_image(2.21 + index * 0.05, blue_square_frame())
            for index in range(4)
        ]
        self.assertTrue(all(item.command.forward == BRIDGE_FORWARD_SPEED for item in decisions))
        self.assertNotIn(task.state, (ALIGNING, "approaching"))

    def test_new_route_requires_approach_alignment_and_fresh_handoff_frames(self):
        _, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        for now in (3.01, 3.06, 3.11):
            harness.feed_image(now, far_fragment_frame(x=320))
        harness.feed_image(3.16, near_route_frame())
        for now in (3.21, 3.26, 3.31, 3.36, 3.41):
            decision = harness.feed_image(now, near_route_frame())
            self.assertEqual(decision.state, TASK_ACTIVE)
        done = harness.feed_image(3.46, near_route_frame())
        self.assertEqual(done.state, RELEASING)
        self.assertEqual(done.task_update.status, TaskStatus.COMPLETED)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -25.0)

        waiting = harness.feed_line(3.70, x=320)
        self.assertEqual(waiting.state, RELEASING)
        resumed = harness.feed_line(3.92, x=320)
        self.assertEqual(resumed.state, LINE_FOLLOWING)
        self.assertTrue(harness.follower.motion_enabled)

    def test_total_timeout_fails_stops_and_restores_line_view(self):
        _, harness, _ = start_and_trigger()
        failed = harness.feed_blank(1.36 + TOTAL_RECOVERY_SECONDS + 0.01)
        self.assertEqual(failed.state, RELEASING)
        self.assertEqual(failed.task_update.status, TaskStatus.FAILED)
        self.assertTrue(failed.force_stop)
        self.assertEqual(failed.command.forward, 0.0)
        self.assertEqual(failed.command.yaw, 0.0)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -25.0)

    def test_human_stop_resets_route_and_needs_explicit_resume(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        harness.coordinator.human_stop(2.20)
        self.assertEqual(task.state, MONITORING)
        self.assertEqual(harness.owner, "line")
        self.assertFalse(harness.follower.motion_enabled)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -25.0)

        # The coordinator deliberately refuses resume while the gimbal is
        # still returning from the raised search view.
        harness.feed_line(2.70, x=320)
        self.assertTrue(harness.coordinator.human_resume(2.71))
        normal = harness.feed_line(2.75, x=320)
        self.assertEqual(normal.state, LINE_FOLLOWING)
        self.assertEqual(task.state, MONITORING)
        self.assertEqual(harness.owner, "line")

    def test_video_gap_resets_route_and_latches_stop(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        decision = harness.coordinator.video_gap(
            CONFIG.video_gap_stop_seconds, 2.30
        )
        self.assertEqual(task.state, MONITORING)
        self.assertTrue(decision.force_stop)
        self.assertFalse(harness.follower.motion_enabled)
        self.assertEqual(harness.owner, "line")


if __name__ == "__main__":
    unittest.main()
