"""Offline contract and behaviour tests for long-gap route recovery."""

import pathlib
import sys
import unittest
from math import cos, radians, sin

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from coordinator import LINE_FOLLOWING, RELEASING, TASK_ACTIVE  # noqa: E402
from models import FramePacket, TaskStatus  # noqa: E402
from route import (  # noqa: E402
    ALIGNING,
    BRIDGING,
    BRIDGE_FORWARD_SPEED,
    BRIDGE_MIN_SECONDS,
    END_APPROACH,
    END_APPROACH_SPEED,
    MONITORING,
    CENTERING,
    CORNERING,
    SEARCHING,
    SEARCH_HARD_LIMIT_DEG,
    SEARCH_CONFIRM_YAW_SPEED,
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


def extending_route_frame(bottom_y, x=320):
    return segment_frame((x, bottom_y), (x, 85), thickness=24)


def near_old_line_frame(x=400):
    """Old route remains only in the bottom band; base detector is invalid."""
    return segment_frame((x, 355), (x, 300), thickness=24)


def angled_far_frame():
    return segment_frame((390, 220), (470, 85))


def blue_square_frame():
    image = np.full((360, 640, 3), 210, np.uint8)
    cv2.rectangle(image, (250, 100), (390, 240), (255, 0, 0), -1)
    return image


def transverse_route_frame():
    return segment_frame((100, 190), (540, 190), thickness=18)


def directed_route_frame(angle_deg=60.0, x=260, y=260, length=180):
    angle = radians(angle_deg)
    end = (
        int(round(x + sin(angle) * length)),
        int(round(y - cos(angle) * length)),
    )
    return segment_frame((x, y), end, thickness=18)


def near_threshold_frame(bottom_y):
    return segment_frame((320, bottom_y), (320, 120), thickness=18)


def old_and_new_frame():
    image = np.full((360, 640, 3), 210, np.uint8)
    cv2.line(image, (100, 355), (100, 285), (255, 0, 0), 18)
    cv2.line(image, (380, 210), (450, 90), (255, 0, 0), 18)
    return image


def connected_right_angle_frame():
    image = np.full((360, 640, 3), 210, np.uint8)
    cv2.line(image, (320, 355), (320, 300), (255, 0, 0), 20)
    cv2.line(image, (320, 300), (0, 300), (255, 0, 0), 20)
    return image


def disconnected_right_angle_frame():
    image = np.full((360, 640, 3), 210, np.uint8)
    cv2.line(image, (320, 355), (320, 285), (255, 0, 0), 20)
    cv2.line(image, (390, 190), (620, 190), (255, 0, 0), 20)
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

    def test_transverse_route_is_detected_as_nearly_ninety_degrees(self):
        candidate = self.vision.candidates(transverse_route_frame())[0]
        self.assertGreater(abs(candidate.angle_deg), 85.0)

    def test_new_far_fragment_outranks_visible_old_near_line(self):
        candidates = self.vision.candidates(old_and_new_frame())
        self.assertGreaterEqual(len(candidates), 2)
        self.assertFalse(candidates[0].near)
        self.assertGreater(candidates[0].detection.center[0], 300)

    def test_connected_corner_exits_boundary_instead_of_looking_like_gap(self):
        path = self.vision.connected_path(connected_right_angle_frame(), 320)
        self.assertTrue(path.present)
        self.assertIsNotNone(path.endpoint)
        self.assertFalse(path.endpoint.internal)
        # 2026-09-17：作者把转向用的前瞻点改成 `CORNER_LOOKAHEAD_PIXELS = 65`
        # （"在可见最远端转向会让连通直角看起来像断口"），这个误差的**量级**
        # 因此从 -0.5 变成 -0.15。判定"直角 vs 断口"靠的是 endpoint.internal
        # （上面那条），这里改成断言**两者的分离**而不是一个写死的旧阈值：
        gap = self.vision.connected_path(disconnected_right_angle_frame(), 320)
        self.assertLess(path.error, -0.10, "连通直角的横向误差应该明显偏一侧")
        self.assertGreater(
            gap.error, path.error + 0.10,
            "真断口与连通直角的横向误差必须能分开（否则会把直角当断口）",
        )

    def test_real_gap_has_internal_old_and_new_endpoints(self):
        image = disconnected_right_angle_frame()
        path = self.vision.connected_path(image, 320)
        candidates = self.vision.candidates(image)
        self.assertTrue(path.endpoint.internal)
        perpendicular = max(candidates, key=lambda item: abs(item.angle_deg))
        self.assertTrue(perpendicular.entry_endpoint.internal)
        self.assertGreater(abs(perpendicular.angle_deg), 80.0)

    def test_horizontal_entry_is_the_end_nearest_vehicle_center_on_either_side(self):
        right = self.vision.candidates(segment_frame((390, 190), (620, 190)))[0]
        left = self.vision.candidates(segment_frame((20, 190), (250, 190)))[0]
        self.assertLess(right.entry_endpoint.point[0], 450)
        self.assertGreater(left.entry_endpoint.point[0], 200)


class RouteRecoveryTests(unittest.TestCase):
    def test_waits_beyond_base_grace_then_raises_view_while_stopped(self):
        _, harness, takeover = start_and_trigger()
        self.assertEqual(takeover.command.forward, 0.0)
        self.assertEqual(takeover.command.yaw, 0.0)
        self.assertEqual(
            harness.gimbal.last_move()["pitch"], CONFIG.gimbal_search_pitch
        )
        self.assertEqual(harness.owner, "external")

    def test_far_loss_with_bottom_line_crawls_to_physical_endpoint(self):
        task = RouteTask()
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0, x=400)

        pending = harness.feed_image(1.05, near_old_line_frame())
        takeover = harness.feed_image(1.25, near_old_line_frame())
        self.assertEqual(pending.state, LINE_FOLLOWING)
        self.assertEqual(takeover.state, TASK_ACTIVE)
        self.assertEqual(takeover.command.forward, END_APPROACH_SPEED)
        self.assertIsNone(harness.gimbal.last_move())

        first_blank = harness.feed_blank(1.30)
        confirmed = harness.feed_blank(1.46)
        self.assertEqual(first_blank.command.forward, 0.0)
        self.assertEqual(confirmed.command.forward, 0.0)
        self.assertEqual(
            harness.gimbal.last_move()["pitch"], CONFIG.gimbal_search_pitch
        )

    def test_endpoint_after_old_two_point_five_second_budget_still_recovers(self):
        """Real feedback: far sampling can vanish >0.20 m before line end."""
        task = RouteTask()
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0, x=400)
        harness.feed_image(1.05, near_old_line_frame())
        harness.feed_image(1.25, near_old_line_frame())

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
        self.assertEqual(
            harness.gimbal.last_move()["pitch"], CONFIG.gimbal_search_pitch
        )

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

    def test_perpendicular_fragment_confirms_then_aligns_before_translation(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        first = harness.feed_image(3.01, transverse_route_frame())
        second = harness.feed_image(3.06, transverse_route_frame())
        confirmed = harness.feed_image(3.11, transverse_route_frame())
        approach = harness.feed_image(3.16, transverse_route_frame())
        self.assertEqual(first.command.forward, BRIDGE_FORWARD_SPEED)
        self.assertEqual(second.command.forward, BRIDGE_FORWARD_SPEED)
        self.assertEqual(confirmed.command.forward, 0.0)
        self.assertEqual(approach.command.forward, 0.0)
        self.assertEqual(approach.command.lateral, 0.0)
        self.assertEqual(task.state, ALIGNING)

    def test_candidate_cannot_interrupt_initial_old_route_clearance(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        decisions = [
            harness.feed_image(2.21 + index * 0.05, transverse_route_frame())
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

    def test_oblique_right_angle_candidate_rotates_before_translation(self):
        _, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        for now in (3.01, 3.06, 3.11):
            harness.feed_image(now, directed_route_frame())
        approach = harness.feed_image(3.16, directed_route_frame())
        self.assertEqual(approach.command.forward, 0.0)
        self.assertGreater(approach.command.yaw, 0.0)
        self.assertLessEqual(abs(approach.command.yaw), SEARCH_CONFIRM_YAW_SPEED)

    def test_alignment_heading_cannot_cancel_against_lateral_offset(self):
        self.assertGreaterEqual(RouteTask._alignment_yaw(26.5), 8.0)
        self.assertLessEqual(RouteTask._alignment_yaw(-59.0), -8.0)
        self.assertAlmostEqual(RouteTask._angle_difference(89.0, -89.0), 2.0)

    def test_near_classification_has_hysteresis(self):
        task = RouteTask()
        first = near_threshold_frame(275)
        entered = near_threshold_frame(300)

        task._observe_candidate(FramePacket(first, 1, 1.0), 1.0)
        self.assertFalse(task.candidate_near)
        task._observe_candidate(FramePacket(entered, 2, 1.05), 1.05)
        self.assertTrue(task.candidate_near)
        task._observe_candidate(FramePacket(first, 3, 1.10), 1.10)
        self.assertTrue(task.candidate_near)

    def test_temporal_tracking_keeps_the_same_horizontal_endpoint(self):
        task = RouteTask()
        # 2026-09-17 更新夹具：作者的新线加了 `MIN_LOCK_BRANCH_PIXELS = 100`
        # （"太小/太远的碎片不许进时序跟踪"）。原来 100px 的水平段实测
        # branch_length 只有 92 -> 被那道闸（而不是被跟踪逻辑）拒掉，
        # 所以把线段**往左**加长（右端=进场端点不动），保住本测试的意图：
        # "同一段胶带平移时，跟踪到的是同一个进场端点"。
        first = segment_frame((150, 190), (350, 190))
        shifted = segment_frame((190, 190), (390, 190))

        task._observe_candidate(FramePacket(first, 1, 1.0), 1.0)
        first_endpoint = task._candidate.entry_endpoint
        task._observe_candidate(FramePacket(shifted, 2, 1.05), 1.05)
        second_endpoint = task._candidate.entry_endpoint

        self.assertGreater(first_endpoint.point[0], 320)
        self.assertGreater(second_endpoint.point[0], 350)
        self.assertLess(
            RouteTask._directed_angle_difference(
                first_endpoint.tangent_deg,
                second_endpoint.tangent_deg,
            ),
            10.0,
        )

    def test_perpendicular_candidate_is_valid_for_known_right_angle_gap(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        harness.feed_blank(5.20)
        self.assertEqual(task.state, SEARCHING)

        decisions = [
            harness.feed_image(now, transverse_route_frame())
            for now in (5.25, 5.30, 5.35, 5.40)
        ]
        self.assertEqual(task.state, ALIGNING)
        self.assertTrue(any(abs(item.command.yaw) > 0.0 for item in decisions))

    def test_alignment_survives_one_missing_candidate_frame(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        for now in (3.01, 3.06, 3.11):
            harness.feed_image(now, transverse_route_frame())
        self.assertEqual(task.state, ALIGNING)

        missing = harness.feed_blank(3.20)
        self.assertEqual(task.state, ALIGNING)
        self.assertEqual(missing.command.forward, 0.0)
        self.assertEqual(missing.command.yaw, 0.0)
        self.assertIn("briefly missing", missing.message)

    def test_approach_targets_route_entry_instead_of_contour_centroid(self):
        image = segment_frame((180, 355), (600, 80), thickness=20)
        candidate = RouteVision(CONFIG.vision).candidates(image)[0]
        self.assertGreater(candidate.detection.center[0], image.shape[1] // 2)
        packet = FramePacket(image, 1, 1.0)
        self.assertLess(RouteTask._candidate_target_error(candidate, packet), 0.0)

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
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        # This test isolates the post-selection handoff sequence.  The
        # old/new direction gate is covered separately below.
        task._old_tangent_world = None
        for now in (3.01, 3.06, 3.11):
            harness.feed_image(now, far_fragment_frame(x=320))
        harness.feed_image(3.16, extending_route_frame(260))
        harness.feed_image(3.21, extending_route_frame(310))
        for now in (3.26, 3.31, 3.36, 3.41, 3.46, 3.51):
            decision = harness.feed_image(now, near_route_frame())
            self.assertEqual(decision.state, TASK_ACTIVE)
        lowering = harness.feed_image(3.56, near_route_frame())
        self.assertEqual(lowering.state, TASK_ACTIVE)
        self.assertEqual(harness.gimbal.last_move()["pitch"], CONFIG.gimbal_pitch)

        # A raised-view route candidate is not enough to complete.  The task
        # waits for the camera to lower, then requires three fresh frames that
        # satisfy the normal line detector.
        for now in (3.70, 3.92, 4.10, 4.15, 4.20):
            waiting = harness.feed_image(now, near_route_frame())
            self.assertEqual(waiting.state, TASK_ACTIVE)
        done = harness.feed_image(4.25, near_route_frame())
        self.assertEqual(done.state, RELEASING)
        self.assertEqual(done.task_update.status, TaskStatus.COMPLETED)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -25.0)

        resumed = harness.feed_line(4.45, x=320)
        self.assertEqual(resumed.state, LINE_FOLLOWING)
        self.assertTrue(harness.follower.motion_enabled)

    def test_lowered_view_without_a_valid_base_line_fails_stopped(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        task._old_tangent_world = None
        for now in (3.01, 3.06, 3.11):
            harness.feed_image(now, far_fragment_frame(x=320))
        harness.feed_image(3.16, extending_route_frame(260))
        harness.feed_image(3.21, extending_route_frame(310))
        for now in (3.26, 3.31, 3.36, 3.41, 3.46, 3.51, 3.56):
            harness.feed_image(now, near_route_frame())

        failed = harness.feed_blank(5.57)
        self.assertEqual(failed.state, RELEASING)
        self.assertEqual(failed.task_update.status, TaskStatus.FAILED)
        self.assertEqual(failed.command.forward, 0.0)
        self.assertEqual(failed.command.yaw, 0.0)

    def test_connected_right_angle_is_followed_without_gap_search(self):
        task = RouteTask()
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0, x=320)
        first = harness.feed_image(1.05, connected_right_angle_frame())
        corner = harness.feed_image(1.25, connected_right_angle_frame())
        self.assertEqual(first.state, LINE_FOLLOWING)
        self.assertEqual(corner.state, TASK_ACTIVE)
        self.assertEqual(task.state, CORNERING)
        self.assertGreater(corner.command.forward, 0.0)
        self.assertLess(corner.command.yaw, 0.0)
        self.assertIsNone(harness.gimbal.last_move())

    def test_connected_corner_tolerates_a_short_path_dropout(self):
        task = RouteTask()
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0, x=320)
        harness.feed_image(1.05, connected_right_angle_frame())
        harness.feed_image(1.25, connected_right_angle_frame())

        missing = harness.feed_blank(1.50)
        recovered = harness.feed_image(1.60, connected_right_angle_frame())
        self.assertEqual(task.state, CORNERING)
        self.assertEqual(missing.command.forward, 0.0)
        self.assertGreater(recovered.command.forward, 0.0)
        self.assertIsNone(harness.gimbal.last_move())

    def test_saved_old_tangent_rejects_old_line_and_accepts_perpendicular(self):
        task = RouteTask()
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0, x=320)
        harness.feed_image(1.05, near_old_line_frame(x=320))
        harness.feed_image(1.15, near_old_line_frame(x=320))
        harness.feed_blank(1.20)
        harness.feed_blank(1.36)
        harness.feed_blank(2.16)

        old_only = [
            harness.feed_image(now, far_fragment_frame(x=320))
            for now in (3.01, 3.06, 3.11)
        ]
        self.assertEqual(task.state, BRIDGING)
        self.assertTrue(all(item.command.forward == BRIDGE_FORWARD_SPEED for item in old_only))

        perpendicular = [
            harness.feed_image(now, transverse_route_frame())
            for now in (3.16, 3.21, 3.26)
        ]
        self.assertEqual(task.state, ALIGNING)
        self.assertEqual(perpendicular[-1].command.forward, 0.0)

    def test_centering_uses_lateral_motion_not_forward_arc(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        for now in (3.01, 3.06, 3.11):
            harness.feed_image(now, transverse_route_frame())
        task.state = CENTERING
        task._heading_offset = 90.0
        task._candidate_center = None
        task._candidate_angle = None
        task._stable_frames = 0
        decision = harness.feed_image(3.16, far_fragment_frame(x=400))
        self.assertEqual(decision.command.forward, 0.0)
        self.assertNotEqual(decision.command.lateral, 0.0)

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
