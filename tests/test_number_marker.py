"""Offline checks for the Final number-marker task (WP2 / Issue #2)."""

import pathlib
import sys
import unittest
from dataclasses import replace

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from models import FramePacket, TaskStatus  # noqa: E402
from number_marker import (  # noqa: E402
    MEASURED_DISTANCE,
    NEAREST_PROXY,
    MarkerCandidate,
    MarkerState,
    NumberMarkerConfig,
    NumberMarkerTask,
    compute_aim_intent,
    evaluate_number_markers,
    is_marker_eligible,
    marker_candidates_from_normalized,
    render_evidence_image,
    select_target_marker,
)
from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
)

WIDTH = 640
HEIGHT = 360
BASE_CONFIG = NumberMarkerConfig(team_number="03")


def packet(sequence=1, now=1.0):
    return FramePacket(np.zeros((HEIGHT, WIDTH, 3), np.uint8), sequence, now)


def candidate(
    target_id="1",
    x=WIDTH / 2,
    y=HEIGHT / 2,
    width=160.0,
    height=100.0,
    now=1.0,
    sequence=None,
    distance=None,
):
    return MarkerCandidate(
        target_id=str(target_id),
        center=(float(x), float(y)),
        width=float(width),
        height=float(height),
        observed_at=now,
        source_sequence=sequence,
        estimated_distance_m=distance,
    )


def set_current(task, sequence, now, *items):
    task.update_candidates(
        replace(item, observed_at=now, source_sequence=sequence) for item in items
    )


def advance_to_evidence(task, marker_id="1", start=1.0):
    required = task.settings.aim_stable_frames
    current = candidate(marker_id, now=start)
    set_current(task, 1, start, current)
    first = task.step(packet(1, start), start)
    assert first.status is TaskStatus.RUNNING
    result = first
    for offset in range(1, required + 1):
        now = start + offset * 0.01
        set_current(task, offset + 1, now, current)
        result = task.step(packet(offset + 1, now), now)
    return result


class NumberMarkerContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        assert_module_source_is_clean(self, "number_marker.py")

    def test_does_not_take_over_on_a_plain_line_frame(self):
        assert_inert_through_harness(self, NumberMarkerTask())

    def test_no_marker_returns_no_action(self):
        update = NumberMarkerTask().step(packet(), 1.0)
        self.assertIs(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertFalse(update.detection.valid)


class FilteringAndSelectionTests(unittest.TestCase):
    def evaluate(self, *items, aimed=()):
        return evaluate_number_markers(items, WIDTH, HEIGHT, 1.0, 1, aimed, 0.15)

    def test_id_1_is_valid(self):
        self.assertEqual(self.evaluate(candidate("1"))[0].target_id, "1")

    def test_id_5_is_valid(self):
        self.assertEqual(self.evaluate(candidate("5"))[0].target_id, "5")

    def test_irrelevant_id_6_is_ignored(self):
        self.assertEqual(self.evaluate(candidate("6")), ())

    def test_irrelevant_marker_does_not_block_valid_marker(self):
        accepted = self.evaluate(candidate("9"), candidate("3"))
        self.assertEqual([item.target_id for item in accepted], ["3"])

    def test_already_aimed_id_is_ignored(self):
        self.assertEqual(self.evaluate(candidate("2"), aimed=("2",)), ())

    def test_duplicate_aimed_id_mixed_with_new_id_selects_new_id(self):
        accepted = self.evaluate(candidate("1", width=220), candidate("4", width=150), aimed=("1",))
        selected = select_target_marker(accepted)
        self.assertEqual(selected.candidate.target_id, "4")

    def test_multiple_markers_use_named_visual_nearest_proxy(self):
        selected = select_target_marker((candidate("1", width=140), candidate("2", width=210)))
        self.assertEqual(selected.candidate.target_id, "2")
        self.assertEqual(selected.strategy, NEAREST_PROXY)

    def test_complete_distance_set_uses_measured_nearest(self):
        selected = select_target_marker(
            (candidate("1", width=220, distance=2.0), candidate("2", width=150, distance=1.0))
        )
        self.assertEqual(selected.candidate.target_id, "2")
        self.assertEqual(selected.strategy, MEASURED_DISTANCE)

    def test_exactly_one_fifth_width_is_not_eligible(self):
        self.assertFalse(is_marker_eligible(candidate(width=WIDTH * 0.20), WIDTH))

    def test_more_than_one_fifth_width_is_eligible(self):
        self.assertTrue(is_marker_eligible(candidate(width=WIDTH * 0.20 + 0.01), WIDTH))

    def test_stale_observation_is_rejected(self):
        stale = candidate(now=0.80)
        accepted = evaluate_number_markers((stale,), WIDTH, HEIGHT, 1.0, 1, (), 0.15)
        self.assertEqual(accepted, ())

    def test_wrong_source_sequence_is_rejected(self):
        self.assertEqual(self.evaluate(candidate(sequence=2)), ())

    def test_observation_without_time_or_frame_identity_is_rejected(self):
        unknown_age = replace(candidate(), observed_at=None, source_sequence=None)
        self.assertEqual(self.evaluate(unknown_age), ())

    def test_nan_candidate_is_rejected(self):
        self.assertEqual(self.evaluate(candidate(x=float("nan"))), ())

    def test_normalized_callback_tuple_converts_to_full_frame_pixels(self):
        converted = marker_candidates_from_normalized(
            ((0.25, 0.50, 0.30, 0.20, "5"),), WIDTH, HEIGHT, 1.0, 7
        )[0]
        self.assertEqual(converted.center, (160.0, 180.0))
        self.assertEqual(converted.width, 192.0)
        self.assertEqual(converted.source_sequence, 7)


class AimingStateTests(unittest.TestCase):
    def begin(self, item, settings=BASE_CONFIG):
        task = NumberMarkerTask(settings)
        set_current(task, 1, 1.0, item)
        update = task.step(packet(1, 1.0), 1.0)
        self.assertIs(update.status, TaskStatus.RUNNING)
        self.assertIsNone(update.motion)
        self.assertIs(task.state, MarkerState.STOPPING)
        return task

    def test_small_target_reports_too_small_without_takeover(self):
        task = NumberMarkerTask(BASE_CONFIG)
        set_current(task, 1, 1.0, candidate(width=100))
        update = task.step(packet(1, 1.0), 1.0)
        self.assertIs(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertEqual(update.message, "TARGET_TOO_SMALL")

    def test_left_target_requests_configured_negative_yaw(self):
        item = candidate(x=180)
        task = self.begin(item)
        set_current(task, 2, 1.01, item)
        update = task.step(packet(2, 1.01), 1.01)
        self.assertLess(update.motion.yaw, 0.0)

    def test_right_target_requests_opposite_yaw(self):
        item = candidate(x=460)
        task = self.begin(item)
        set_current(task, 2, 1.01, item)
        update = task.step(packet(2, 1.01), 1.01)
        self.assertGreater(update.motion.yaw, 0.0)

    def test_yaw_direction_sign_is_configurable(self):
        settings = replace(BASE_CONFIG, yaw_direction_sign=-1.0)
        intent = compute_aim_intent(candidate(x=460), WIDTH, HEIGHT, settings)
        self.assertLess(intent.yaw_rate, 0.0)

    def test_above_target_has_negative_vertical_intent(self):
        intent = compute_aim_intent(candidate(y=80), WIDTH, HEIGHT, BASE_CONFIG)
        self.assertLess(intent.pitch_rate, 0.0)
        self.assertTrue(intent.gimbal_pitch_required)

    def test_below_target_has_positive_vertical_intent(self):
        intent = compute_aim_intent(candidate(y=280), WIDTH, HEIGHT, BASE_CONFIG)
        self.assertGreater(intent.pitch_rate, 0.0)
        self.assertTrue(intent.gimbal_pitch_required)

    def test_one_centered_frame_is_not_immediate_success(self):
        settings = replace(BASE_CONFIG, aim_stable_frames=3)
        task = self.begin(candidate(), settings)
        set_current(task, 2, 1.01, candidate())
        update = task.step(packet(2, 1.01), 1.01)
        self.assertIs(update.status, TaskStatus.RUNNING)
        self.assertIsNone(task.pending_evidence_request)

    def test_n_fresh_centered_frames_lock_aim(self):
        task = NumberMarkerTask(replace(BASE_CONFIG, aim_stable_frames=3))
        update = advance_to_evidence(task)
        self.assertIs(update.status, TaskStatus.RUNNING)
        self.assertIs(task.state, MarkerState.EVIDENCE_PENDING)
        self.assertIn("1", task.aimed_ids)
        self.assertNotIn("1", task.saved_ids)

    def test_repeated_same_frame_does_not_advance_stability(self):
        task = self.begin(candidate(), replace(BASE_CONFIG, aim_stable_frames=2))
        set_current(task, 2, 1.01, candidate())
        task.step(packet(2, 1.01), 1.01)
        task.step(packet(2, 1.01), 1.01)
        self.assertIsNone(task.pending_evidence_request)

    def test_target_lost_during_aiming_holds_zero(self):
        task = self.begin(candidate(x=180))
        task.update_candidates(())
        update = task.step(packet(2, 1.01), 1.01)
        self.assertIs(update.status, TaskStatus.RUNNING)
        self.assertEqual(update.motion.yaw, 0.0)
        self.assertEqual(update.message, "TARGET_LOST:HOLD")

    def test_target_loss_timeout_fails_stopped(self):
        settings = replace(BASE_CONFIG, target_lost_timeout=0.05)
        task = self.begin(candidate(x=180), settings)
        task.update_candidates(())
        task.step(packet(2, 1.01), 1.01)
        update = task.step(packet(3, 1.07), 1.07)
        self.assertIs(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion.yaw, 0.0)

    def test_stale_frame_during_aiming_fails_stopped(self):
        task = self.begin(candidate(x=180))
        update = task.step(packet(2, 1.0), 1.30)
        self.assertIs(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion.yaw, 0.0)
        self.assertIn("STALE_FRAME", update.message)

    def test_observation_provider_exception_does_not_reuse_old_command(self):
        def broken_provider(frame, now):
            raise RuntimeError("synthetic worker failure")

        task = NumberMarkerTask(BASE_CONFIG, observation_provider=broken_provider)
        update = task.step(packet(), 1.0)
        self.assertIs(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertIsNone(update.motion)
        self.assertIn("OBSERVATION_PROVIDER_ERROR", update.message)

    def test_provider_exception_while_aiming_fails_with_zero_command(self):
        calls = [0]

        def failing_after_target(frame, now):
            calls[0] += 1
            if calls[0] == 1:
                return (candidate(x=180, now=now),)
            raise RuntimeError("synthetic worker failure")

        task = NumberMarkerTask(BASE_CONFIG, observation_provider=failing_after_target)
        task.step(packet(1, 1.0), 1.0)
        update = task.step(packet(2, 1.01), 1.01)
        self.assertIs(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion.yaw, 0.0)
        self.assertIn("OBSERVATION_PROVIDER_ERROR", update.message)


class EvidenceTests(unittest.TestCase):
    def task_at_evidence(self, settings=None):
        task = NumberMarkerTask(settings or replace(BASE_CONFIG, aim_stable_frames=1))
        advance_to_evidence(task)
        request = task.take_evidence_request()
        self.assertIsNotNone(request)
        return task, request

    def test_request_contains_rectangle_id_team_and_center_text_anchor(self):
        task, request = self.task_at_evidence()
        self.assertEqual(request.marker_id, "1")
        self.assertEqual(request.annotation, "Team 03 detects a marker with ID of 1")
        self.assertEqual(request.text_anchor, (WIDTH // 2, HEIGHT // 2))
        self.assertIsNotNone(request.detection.box)
        self.assertEqual(request.image.shape, (HEIGHT, WIDTH, 3))

    def test_annotation_preserves_full_scene_and_does_not_mutate_source(self):
        task, request = self.task_at_evidence()
        before = request.image.copy()
        shown = render_evidence_image(request)
        self.assertEqual(shown.shape, before.shape)
        self.assertTrue(np.array_equal(request.image, before))
        self.assertFalse(np.array_equal(shown, before))

    def test_success_ack_separately_marks_saved_and_completes(self):
        task, request = self.task_at_evidence()
        self.assertTrue(task.acknowledge_evidence(request.request_id, True))
        update = task.step(packet(3, 1.03), 1.03)
        self.assertIs(update.status, TaskStatus.COMPLETED)
        self.assertIn("1", task.aimed_ids)
        self.assertIn("1", task.saved_ids)

    def test_write_failure_is_not_reported_as_saved_success(self):
        task, request = self.task_at_evidence()
        self.assertTrue(task.acknowledge_evidence(request.request_id, False))
        update = task.step(packet(3, 1.03), 1.03)
        self.assertIs(update.status, TaskStatus.FAILED)
        self.assertIn("1", task.aimed_ids)
        self.assertNotIn("1", task.saved_ids)

    def test_failed_write_can_be_configured_for_one_retry(self):
        settings = replace(BASE_CONFIG, aim_stable_frames=1, max_evidence_attempts=2)
        task, first = self.task_at_evidence(settings)
        self.assertTrue(task.acknowledge_evidence(first.request_id, False))
        retry = task.take_evidence_request()
        self.assertEqual(retry.attempt, 2)
        self.assertNotEqual(retry.request_id, first.request_id)
        self.assertTrue(task.acknowledge_evidence(retry.request_id, True))
        update = task.step(packet(3, 1.03), 1.03)
        self.assertIs(update.status, TaskStatus.COMPLETED)

    def test_unknown_evidence_ack_is_rejected(self):
        task, request = self.task_at_evidence()
        self.assertFalse(task.acknowledge_evidence("wrong-request", True))
        self.assertNotIn(request.marker_id, task.saved_ids)

    def test_aim_lock_without_team_number_fails_before_false_evidence(self):
        task = NumberMarkerTask(NumberMarkerConfig(aim_stable_frames=1))
        update = advance_to_evidence(task)
        self.assertIs(update.status, TaskStatus.FAILED)
        self.assertIsNone(task.pending_evidence_request)
        self.assertEqual(task.saved_ids, set())


class CoordinatorIntegrationTests(unittest.TestCase):
    def test_real_harness_stops_takes_over_and_releases_after_saved_evidence(self):
        task = NumberMarkerTask(replace(BASE_CONFIG, aim_stable_frames=1))
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)

        item = candidate(now=1.05)
        task.update_candidates((item,))
        takeover = harness.feed_line(1.05)
        self.assertEqual(takeover.owner, "external")
        self.assertEqual(takeover.command.yaw, 0.0)

        task.update_candidates((replace(item, observed_at=1.10),))
        pending = harness.feed_line(1.10)
        self.assertIs(pending.task_update.status, TaskStatus.RUNNING)
        request = task.take_evidence_request()
        self.assertTrue(task.acknowledge_evidence(request.request_id, True))

        released = harness.feed_line(1.15)
        self.assertEqual(released.owner, "line")
        self.assertEqual(released.state, "RELEASING")
        self.assertTrue(released.force_stop)


if __name__ == "__main__":
    unittest.main()
