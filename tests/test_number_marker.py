"""Offline checks for the Final number-marker task (WP2 / Issue #2)."""

import io
import json
import pathlib
import sys
import unittest
from contextlib import redirect_stderr
from dataclasses import replace
from unittest.mock import patch

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import CONFIG  # noqa: E402
from evidence import render_task_evidence  # noqa: E402
from models import FramePacket, GimbalCommand, TaskStatus  # noqa: E402
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
    select_target_marker,
)
from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
)
import number_marker  # noqa: E402

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
    def test_default_team_number_is_03(self):
        self.assertEqual(NumberMarkerConfig().team_number, "03")
        self.assertEqual(NumberMarkerTask().settings.team_number, "03")

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
    def begin(self, item, settings=BASE_CONFIG, runtime_settings=None):
        task = NumberMarkerTask(settings, runtime_settings=runtime_settings)
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

    def test_above_target_has_positive_vertical_intent_with_default_sign(self):
        intent = compute_aim_intent(candidate(y=80), WIDTH, HEIGHT, BASE_CONFIG)
        self.assertGreater(intent.pitch_rate, 0.0)
        self.assertTrue(intent.gimbal_pitch_required)

    def test_below_target_has_negative_vertical_intent_with_default_sign(self):
        intent = compute_aim_intent(candidate(y=280), WIDTH, HEIGHT, BASE_CONFIG)
        self.assertLess(intent.pitch_rate, 0.0)
        self.assertTrue(intent.gimbal_pitch_required)

    def test_centered_marker_keeps_entry_pitch_without_gimbal_command(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        task = self.begin(candidate(), runtime_settings=runtime)
        set_current(task, 2, 1.10, candidate())
        update = task.step(packet(2, 1.10), 1.10)
        self.assertIsNone(update.gimbal)
        self.assertEqual(task.target_pitch, -10.0)

    def test_above_marker_updates_absolute_pitch_in_default_direction(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        item = candidate(y=80)
        task = self.begin(item, runtime_settings=runtime)
        set_current(task, 2, 1.10, item)
        update = task.step(packet(2, 1.10), 1.10)
        self.assertIsInstance(update.gimbal, GimbalCommand)
        self.assertGreater(update.gimbal.pitch, -10.0)
        self.assertEqual(update.gimbal.yaw, float(runtime.gimbal_yaw))

    def test_below_marker_updates_absolute_pitch_in_opposite_direction(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        item = candidate(y=280)
        task = self.begin(item, runtime_settings=runtime)
        set_current(task, 2, 1.10, item)
        update = task.step(packet(2, 1.10), 1.10)
        self.assertIsInstance(update.gimbal, GimbalCommand)
        self.assertLess(update.gimbal.pitch, -10.0)

    def test_pitch_direction_sign_can_be_flipped_in_one_setting(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        item = candidate(y=80)
        normal = self.begin(item, runtime_settings=runtime)
        set_current(normal, 2, 1.10, item)
        normal_update = normal.step(packet(2, 1.10), 1.10)

        flipped_settings = replace(BASE_CONFIG, pitch_direction_sign=1.0)
        flipped = self.begin(item, flipped_settings, runtime)
        set_current(flipped, 2, 1.10, item)
        flipped_update = flipped.step(packet(2, 1.10), 1.10)
        self.assertGreater(normal_update.gimbal.pitch, -10.0)
        self.assertLess(flipped_update.gimbal.pitch, -10.0)

    def test_large_elapsed_time_is_limited_by_integration_dt(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        item = candidate(y=80)
        settings = replace(BASE_CONFIG, max_pitch_integration_dt=0.20)
        task = self.begin(item, settings, runtime)
        set_current(task, 2, 2.00, item)
        update = task.step(packet(2, 2.00), 2.00)
        intent = compute_aim_intent(item, WIDTH, HEIGHT, settings)
        self.assertAlmostEqual(update.gimbal.pitch, -10.0 + intent.pitch_rate * 0.20)

    def test_repeated_identical_timestamp_does_not_accumulate_pitch(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        item = candidate(y=80)
        task = self.begin(item, runtime_settings=runtime)
        set_current(task, 2, 1.10, item)
        first = task.step(packet(2, 1.10), 1.10)
        set_current(task, 3, 1.10, item)
        repeated = task.step(packet(3, 1.10), 1.10)
        self.assertIsNone(repeated.gimbal)
        self.assertEqual(task.target_pitch, first.gimbal.pitch)

    def test_pitch_is_clamped_to_runtime_upper_bound(self):
        runtime = replace(CONFIG, gimbal_pitch=9, gimbal_pitch_min=-25, gimbal_pitch_max=10)
        item = candidate(y=0)
        task = self.begin(item, runtime_settings=runtime)
        set_current(task, 2, 1.20, item)
        update = task.step(packet(2, 1.20), 1.20)
        self.assertEqual(update.gimbal.pitch, 10.0)

    def test_pitch_is_clamped_to_runtime_lower_bound(self):
        runtime = replace(CONFIG, gimbal_pitch=-24, gimbal_pitch_min=-25, gimbal_pitch_max=10)
        item = candidate(y=360)
        task = self.begin(item, runtime_settings=runtime)
        set_current(task, 2, 1.20, item)
        update = task.step(packet(2, 1.20), 1.20)
        self.assertEqual(update.gimbal.pitch, -25.0)

    def test_horizontal_and_vertical_corrections_are_emitted_together(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        item = candidate(x=460, y=80)
        task = self.begin(item, runtime_settings=runtime)
        set_current(task, 2, 1.10, item)
        update = task.step(packet(2, 1.10), 1.10)
        self.assertGreater(update.motion.yaw, 0.0)
        self.assertGreater(update.gimbal.pitch, -10.0)

    def test_centered_frames_lock_after_correction_without_pitch_drift(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        settings = replace(BASE_CONFIG, aim_stable_frames=2)
        item = candidate(y=80)
        task = self.begin(item, settings, runtime)
        set_current(task, 2, 1.10, item)
        adjusted = task.step(packet(2, 1.10), 1.10)
        held_pitch = adjusted.gimbal.pitch
        for sequence, now in ((3, 1.20), (4, 1.30)):
            centered = candidate(now=now, sequence=sequence)
            set_current(task, sequence, now, centered)
            update = task.step(packet(sequence, now), now)
            self.assertIsNone(update.gimbal)
            self.assertEqual(task.target_pitch, held_pitch)
        self.assertIs(task.state, MarkerState.EVIDENCE_PENDING)

    def test_brief_target_loss_emits_no_pitch_and_preserves_target(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        item = candidate(y=80)
        task = self.begin(item, runtime_settings=runtime)
        set_current(task, 2, 1.10, item)
        task.step(packet(2, 1.10), 1.10)
        held_pitch = task.target_pitch
        task.update_candidates(())
        update = task.step(packet(3, 1.20), 1.20)
        self.assertIsNone(update.gimbal)
        self.assertEqual(task.target_pitch, held_pitch)

    def test_stale_frame_emits_no_pitch_and_clears_failed_transient(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        item = candidate(y=80)
        task = self.begin(item, runtime_settings=runtime)
        set_current(task, 2, 1.10, item)
        task.step(packet(2, 1.10), 1.10)
        update = task.step(packet(3, 1.10), 1.40)
        self.assertIsNone(update.gimbal)
        self.assertIs(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion.yaw, 0.0)
        self.assertIsNone(task.target_pitch)
        self.assertIsNone(task.pending_evidence_request)
        set_current(task, 4, 1.41, candidate("2"))
        task.step(packet(4, 1.41), 1.41)
        self.assertEqual(task.target_id, "2")

    def test_task_timeout_clears_queued_evidence_and_stops(self):
        settings = replace(BASE_CONFIG, aim_stable_frames=1, max_task_seconds=0.05)
        task = NumberMarkerTask(settings)
        advance_to_evidence(task)
        stale_request = task.pending_evidence_request
        self.assertIsNotNone(stale_request)
        failed = task.step(packet(3, 1.20), 1.20)
        self.assertIs(failed.status, TaskStatus.FAILED)
        self.assertEqual(failed.message, "FAILED:TASK_TIMEOUT")
        self.assertEqual(failed.motion.yaw, 0.0)
        self.assertIsNone(failed.gimbal)
        self.assertIsNone(task.target_pitch)
        self.assertIsNone(task.pending_evidence_request)
        self.assertFalse(task.acknowledge_evidence(stale_request.request_id, True))
        set_current(task, 4, 1.21, candidate("2"))
        task.step(packet(4, 1.21), 1.21)
        self.assertEqual(task.target_id, "2")

    def test_evidence_pending_steps_do_not_accumulate_pitch(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        settings = replace(BASE_CONFIG, aim_stable_frames=1)
        item = candidate(y=80)
        task = self.begin(item, settings, runtime)
        set_current(task, 2, 1.10, item)
        task.step(packet(2, 1.10), 1.10)
        held_pitch = task.target_pitch
        centered = candidate()
        set_current(task, 3, 1.20, centered)
        locked = task.step(packet(3, 1.20), 1.20)
        self.assertIs(task.state, MarkerState.EVIDENCE_PENDING)
        self.assertIsNone(locked.gimbal)
        pending = task.step(packet(4, 1.30), 1.30)
        self.assertIsNone(pending.gimbal)
        self.assertEqual(task.target_pitch, held_pitch)

    def test_completed_task_resets_pitch_before_next_marker(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        task = NumberMarkerTask(
            replace(BASE_CONFIG, aim_stable_frames=1), runtime_settings=runtime
        )
        result = advance_to_evidence(task)
        self.assertIs(result.status, TaskStatus.RUNNING)
        request = task.take_evidence_request()
        task.acknowledge_evidence(request.request_id, True)
        completed = task.step(packet(3, 1.03), 1.03)
        self.assertIs(completed.status, TaskStatus.COMPLETED)
        self.assertIsNone(task.target_pitch)
        next_item = candidate("2")
        set_current(task, 4, 1.04, next_item)
        task.step(packet(4, 1.04), 1.04)
        self.assertEqual(task.target_pitch, -10.0)

    def test_failed_task_resets_pitch_before_next_marker(self):
        runtime = replace(CONFIG, gimbal_pitch=-10)
        task = NumberMarkerTask(
            replace(BASE_CONFIG, aim_stable_frames=1), runtime_settings=runtime
        )
        advance_to_evidence(task)
        request = task.take_evidence_request()
        task.acknowledge_evidence(request.request_id, False)
        failed = task.step(packet(3, 1.03), 1.03)
        self.assertIs(failed.status, TaskStatus.FAILED)
        self.assertIsNone(task.target_pitch)
        next_item = candidate("2")
        set_current(task, 4, 1.04, next_item)
        task.step(packet(4, 1.04), 1.04)
        self.assertEqual(task.target_pitch, -10.0)

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
        task = self.begin(candidate(x=180, y=80), settings)
        set_current(task, 2, 1.01, candidate(x=180, y=80))
        task.step(packet(2, 1.01), 1.01)
        task.update_candidates(())
        task.step(packet(3, 1.02), 1.02)
        update = task.step(packet(4, 1.08), 1.08)
        self.assertIs(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion.yaw, 0.0)
        self.assertIsNone(update.gimbal)
        self.assertIsNone(task.target_pitch)
        self.assertIsNone(task.pending_evidence_request)
        set_current(task, 5, 1.09, candidate("2"))
        task.step(packet(5, 1.09), 1.09)
        self.assertEqual(task.target_id, "2")

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


class TrackingHysteresisTests(unittest.TestCase):
    def feed(self, task, sequence, now, ratio, marker_id="1", centered=True):
        item = candidate(
            marker_id, x=WIDTH / 2 if centered else 460,
            width=WIDTH * ratio,
        )
        set_current(task, sequence, now, item)
        return task.step(packet(sequence, now), now)

    def test_first_trigger_remains_strictly_above_020(self):
        task = NumberMarkerTask(BASE_CONFIG)
        self.assertIs(self.feed(task, 1, 1.0, 0.20).status, TaskStatus.NOT_TRIGGERED)
        self.assertIsNone(task.target_id)
        self.assertIs(self.feed(task, 2, 1.01, 0.2001).status, TaskStatus.RUNNING)
        self.assertEqual(task.target_id, "1")

    def test_019_after_lock_keeps_same_target_without_failure(self):
        task = NumberMarkerTask(BASE_CONFIG)
        self.feed(task, 1, 1.0, 0.25)
        update = self.feed(task, 2, 1.01, 0.19, centered=False)
        self.assertIs(update.status, TaskStatus.RUNNING)
        self.assertEqual(update.message, "AIMING:TRACKING_BELOW_TRIGGER")
        self.assertEqual(task.target_id, "1")
        self.assertIsNone(task.pending_evidence_request)

    def test_tracking_boundary_017_holds_but_0169_enters_zero_motion_grace(self):
        task = NumberMarkerTask(replace(BASE_CONFIG, aim_stable_frames=1))
        self.feed(task, 1, 1.0, 0.25)
        held = self.feed(task, 2, 1.01, 0.17)
        self.assertIs(held.status, TaskStatus.RUNNING)
        self.assertIsNone(task.pending_evidence_request)
        lost = self.feed(task, 3, 1.02, 0.169, centered=False)
        self.assertEqual(lost.message, "TARGET_LOST:HOLD")
        self.assertEqual(lost.motion.yaw, 0.0)
        self.assertIsNone(task.pending_evidence_request)

    def test_locked_exactly_020_cannot_create_evidence(self):
        task = NumberMarkerTask(replace(BASE_CONFIG, aim_stable_frames=1))
        self.feed(task, 1, 1.0, 0.25)
        for sequence in (2, 3):
            update = self.feed(task, sequence, 1.0 + sequence * 0.01, 0.20)
            self.assertIs(update.status, TaskStatus.RUNNING)
            self.assertIsNone(task.pending_evidence_request)
        self.feed(task, 4, 1.04, 0.2001)
        self.assertIsNotNone(task.pending_evidence_request)

    def test_recovery_above_trigger_retains_id_and_restarts_stability(self):
        task = NumberMarkerTask(replace(BASE_CONFIG, aim_stable_frames=2))
        self.feed(task, 1, 1.0, 0.25)
        self.feed(task, 2, 1.01, 0.18)
        set_current(task, 3, 1.02,
                    candidate("1", width=WIDTH * 0.23),
                    candidate("2", width=WIDTH * 0.40))
        recovered = task.step(packet(3, 1.02), 1.02)
        self.assertIs(recovered.status, TaskStatus.RUNNING)
        self.assertEqual(task.target_id, "1")
        self.assertIsNone(task.pending_evidence_request)
        self.assertIn("TRACKING_RECOVERED", [e["event"] for e in task.diagnostic_events])
        self.feed(task, 4, 1.03, 0.23)
        self.assertEqual(task.pending_evidence_request.marker_id, "1")

    def test_evidence_gate_requires_new_above_trigger_stable_frames(self):
        task = NumberMarkerTask(replace(BASE_CONFIG, aim_stable_frames=3))
        self.feed(task, 1, 1.0, 0.25)
        for sequence in (2, 3, 4, 5):
            self.feed(task, sequence, 1.0 + sequence * 0.01, 0.18)
            self.assertIsNone(task.pending_evidence_request)
            self.assertNotIn("1", task.aimed_ids)
        for sequence in (6, 7):
            self.feed(task, sequence, 1.0 + sequence * 0.01, 0.23)
            self.assertIsNone(task.pending_evidence_request)
        self.feed(task, 8, 1.08, 0.23)
        self.assertEqual(task.pending_evidence_request.marker_id, "1")
        self.assertIn("1", task.aimed_ids)
        self.assertNotIn("1", task.saved_ids)

    def test_below_tracking_threshold_uses_existing_030_timeout(self):
        task = NumberMarkerTask(BASE_CONFIG)
        self.feed(task, 1, 1.0, 0.25)
        self.feed(task, 2, 1.01, 0.169)
        task.update_candidates(())
        self.assertIs(task.step(packet(3, 1.30), 1.30).status, TaskStatus.RUNNING)
        failed = task.step(packet(4, 1.32), 1.32)
        self.assertIs(failed.status, TaskStatus.FAILED)
        self.assertEqual(failed.message, "FAILED:TARGET_LOST_TIMEOUT")
        self.assertEqual(failed.motion.yaw, 0.0)
        self.assertIsNone(task.pending_evidence_request)

    def test_transition_diagnostics_include_bounded_pre_failure_trace(self):
        task = NumberMarkerTask(BASE_CONFIG)
        output = io.StringIO()
        with redirect_stderr(output):
            self.feed(task, 1, 1.0, 0.25)
            self.feed(task, 2, 1.01, 0.18)
            self.feed(task, 3, 1.02, 0.23)
            self.feed(task, 4, 1.03, 0.169)
            task.update_candidates(())
            task.step(packet(5, 1.34), 1.34)
        lines = [json.loads(line.split(" ", 1)[1]) for line in output.getvalue().splitlines()]
        events = [line["event"] for line in lines]
        self.assertEqual(
            [event for event in events if event in {
                "TARGET_LOCKED", "TRACKING_BELOW_TRIGGER", "TRACKING_RECOVERED",
                "TARGET_LOST_GRACE", "TARGET_LOST_TIMEOUT",
            }],
            ["TARGET_LOCKED", "TRACKING_BELOW_TRIGGER", "TRACKING_RECOVERED",
             "TARGET_LOST_GRACE", "TARGET_LOST_TIMEOUT"],
        )
        self.assertEqual(lines[0]["locked_target_id"], "1")
        self.assertEqual(lines[0]["locked_target_width_ratio"], 0.25)
        self.assertEqual(lines[-1]["frame_sequence"], 5)
        self.assertGreater(lines[-1]["target_lost_age"], 0.30)
        self.assertTrue(any(row["current_target_width_ratio"] == 0.169
                            for row in lines[-1]["recent_observations"]))
        self.assertEqual(len(task.diagnostic_events), len(lines))

    def test_diagnostics_preserve_age_of_rejected_same_id_observation(self):
        task = NumberMarkerTask(BASE_CONFIG)
        with redirect_stderr(io.StringIO()):
            self.feed(task, 1, 1.0, 0.25)
            task.update_candidates((candidate(now=1.0, sequence=None),))
            update = task.step(packet(2, 1.20), 1.20)
        self.assertEqual(update.message, "TARGET_LOST:HOLD")
        lost = task.diagnostic_events[-1]
        self.assertEqual(lost["event"], "TARGET_LOST_GRACE")
        self.assertAlmostEqual(lost["observation_age"], 0.20)
        self.assertEqual(lost["frame_sequence"], 2)


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

    def test_default_annotation_uses_actual_id_and_custom_team_still_works(self):
        for marker_id in ("2", "3"):
            task = NumberMarkerTask(replace(NumberMarkerConfig(), aim_stable_frames=1))
            advance_to_evidence(task, marker_id=marker_id)
            request = task.take_evidence_request()
            self.assertEqual(
                request.annotation,
                "Team 03 detects a marker with ID of {}".format(marker_id),
            )
        custom = NumberMarkerTask(
            replace(NumberMarkerConfig(), team_number="17", aim_stable_frames=1)
        )
        advance_to_evidence(custom, marker_id="5")
        self.assertEqual(
            custom.take_evidence_request().annotation,
            "Team 17 detects a marker with ID of 5",
        )

    def test_request_uses_locking_frame_and_full_frame_pixel_coordinates(self):
        task = NumberMarkerTask(replace(NumberMarkerConfig(), aim_stable_frames=1))
        item = candidate("2", x=320, y=180, width=160, height=100)
        set_current(task, 1, 1.0, item)
        task.step(packet(1, 1.0), 1.0)
        image = np.full((HEIGHT, WIDTH, 3), 27, dtype=np.uint8)
        image[20, 30] = (2, 3, 4)
        original = image.copy()
        locked_frame = FramePacket(image, 2, 1.01)
        set_current(task, 2, 1.01, item)
        update = task.step(locked_frame, 1.01)
        request = task.take_evidence_request()

        self.assertIs(update.status, TaskStatus.RUNNING)
        self.assertEqual(request.marker_id, "2")
        self.assertEqual(request.request_id, "marker:2:frame:2:attempt:1")
        self.assertEqual(request.frame_sequence, 2)
        self.assertEqual(request.captured_at, 1.01)
        self.assertEqual(request.detection.center, (320, 180))
        self.assertEqual(request.detection.box, (240, 130, 400, 230))
        self.assertEqual(request.annotation, "Team 03 detects a marker with ID of 2")
        self.assertEqual(request.image.shape, (HEIGHT, WIDTH, 3))
        self.assertTrue(np.array_equal(request.image, original))
        image[:] = 99
        self.assertTrue(np.array_equal(request.image, original))

    def test_integration_renderer_preserves_full_scene_and_request_image(self):
        task, request = self.task_at_evidence()
        before = request.image.copy()
        shown = render_task_evidence(request)
        self.assertEqual(shown.shape, before.shape)
        self.assertTrue(np.array_equal(request.image, before))
        self.assertFalse(np.array_equal(shown, before))

    def test_pending_keeps_ownership_and_zero_motion_until_real_ack(self):
        task = NumberMarkerTask(replace(NumberMarkerConfig(), aim_stable_frames=1))
        locked = advance_to_evidence(task, marker_id="2")
        self.assertIs(task.state, MarkerState.EVIDENCE_PENDING)
        self.assertIs(locked.status, TaskStatus.RUNNING)
        self.assertEqual(locked.motion.forward, 0.0)
        self.assertEqual(locked.motion.lateral, 0.0)
        self.assertEqual(locked.motion.yaw, 0.0)
        self.assertIsNone(locked.gimbal)
        original_pitch = task.target_pitch
        for sequence, now in ((3, 1.02), (4, 1.03)):
            set_current(task, sequence, now, candidate("2", x=460, y=80))
            pending = task.step(packet(sequence, now), now)
            self.assertIs(pending.status, TaskStatus.RUNNING)
            self.assertEqual(pending.motion.yaw, 0.0)
            self.assertIsNone(pending.gimbal)
            self.assertEqual(task.target_pitch, original_pitch)
        request = task.take_evidence_request()
        self.assertTrue(task.acknowledge_evidence(request.request_id, True))
        completed = task.step(packet(5, 1.04), 1.04)
        self.assertIs(completed.status, TaskStatus.COMPLETED)
        self.assertEqual(completed.motion.yaw, 0.0)

    def test_retry_ids_are_unique_old_ack_is_rejected_and_attempts_end(self):
        settings = replace(NumberMarkerConfig(), aim_stable_frames=1, max_evidence_attempts=3)
        task, first = self.task_at_evidence(settings)
        self.assertEqual(first.request_id, "marker:1:frame:2:attempt:1")
        self.assertTrue(task.acknowledge_evidence(first.request_id, False))
        second = task.take_evidence_request()
        self.assertEqual(second.attempt, 2)
        self.assertNotEqual(second.request_id, first.request_id)
        self.assertFalse(task.acknowledge_evidence(first.request_id, True))
        self.assertNotIn("1", task.saved_ids)
        self.assertTrue(task.acknowledge_evidence(second.request_id, False))
        third = task.take_evidence_request()
        self.assertEqual(third.attempt, 3)
        self.assertEqual(len({first.request_id, second.request_id, third.request_id}), 3)
        self.assertFalse(task.acknowledge_evidence(second.request_id, True))
        self.assertTrue(task.acknowledge_evidence(third.request_id, False))
        self.assertFalse(task.acknowledge_evidence(third.request_id, True))
        failed = task.step(packet(3, 1.03), 1.03)
        self.assertIs(failed.status, TaskStatus.FAILED)
        self.assertEqual(failed.message, "FAILED:EVIDENCE_WRITE_FAILED")
        self.assertEqual(failed.motion.yaw, 0.0)
        self.assertIsNone(task.pending_evidence_request)
        self.assertNotIn("1", task.saved_ids)

    def test_evidence_write_failure_clears_queued_request_at_terminal_step(self):
        task = NumberMarkerTask(replace(NumberMarkerConfig(), aim_stable_frames=1))
        advance_to_evidence(task)
        request = task.pending_evidence_request
        self.assertIsNotNone(request)
        self.assertTrue(task.acknowledge_evidence(request.request_id, False))
        failed = task.step(packet(3, 1.03), 1.03)
        self.assertIs(failed.status, TaskStatus.FAILED)
        self.assertEqual(failed.motion.yaw, 0.0)
        self.assertIsNone(failed.gimbal)
        self.assertIsNone(task.target_pitch)
        self.assertIsNone(task.pending_evidence_request)
        self.assertFalse(task.acknowledge_evidence(request.request_id, True))
        next_item = candidate("2")
        set_current(task, 4, 1.04, next_item)
        task.step(packet(4, 1.04), 1.04)
        self.assertEqual(task.target_id, "2")

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
        self.assertEqual(update.message, "FAILED:EVIDENCE_WRITE_FAILED")
        self.assertEqual(update.motion.yaw, 0.0)
        self.assertIsNone(task.pending_evidence_request)
        self.assertIn("1", task.aimed_ids)
        self.assertNotIn("1", task.saved_ids)

    def test_completed_id_is_not_scored_again_but_new_id_is_available(self):
        task = NumberMarkerTask(replace(NumberMarkerConfig(), aim_stable_frames=1))
        advance_to_evidence(task, marker_id="2")
        first = task.take_evidence_request()
        self.assertTrue(task.acknowledge_evidence(first.request_id, True))
        self.assertIs(task.step(packet(3, 1.03), 1.03).status, TaskStatus.COMPLETED)
        self.assertEqual(task.aimed_ids, {"2"})
        self.assertEqual(task.saved_ids, {"2"})
        set_current(task, 4, 1.04, candidate("2"))
        duplicate = task.step(packet(4, 1.04), 1.04)
        self.assertIs(duplicate.status, TaskStatus.NOT_TRIGGERED)
        self.assertIsNone(task.pending_evidence_request)
        set_current(task, 5, 1.05, candidate("3"))
        fresh = task.step(packet(5, 1.05), 1.05)
        self.assertIs(fresh.status, TaskStatus.RUNNING)
        self.assertEqual(task.target_id, "3")

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
        task = NumberMarkerTask(NumberMarkerConfig(aim_stable_frames=1, team_number=None))
        update = advance_to_evidence(task)
        self.assertIs(update.status, TaskStatus.FAILED)
        self.assertIsNone(task.pending_evidence_request)
        self.assertEqual(task.saved_ids, set())


class DiagnosticsOnlyTests(unittest.TestCase):
    def feed(self, task, sequence, now, ratio=0.25, x=WIDTH / 2, y=HEIGHT / 2):
        set_current(task, sequence, now, candidate(x=x, y=y, width=WIDTH * ratio))
        return task.step(packet(sequence, now), now)

    def events(self, task, name):
        return [item for item in task.diagnostic_events if item["event"] == name]

    def test_trigger_boundary_and_locked_tracking_are_unchanged(self):
        task = NumberMarkerTask(BASE_CONFIG)
        with redirect_stderr(io.StringIO()):
            self.assertIs(self.feed(task, 1, 1.0, 0.20).status, TaskStatus.NOT_TRIGGERED)
            self.assertEqual(self.feed(task, 2, 1.01, 0.2001).message,
                             "STOPPING:TARGET_FOUND")
            held = self.feed(task, 3, 1.02, 0.19, x=460)
        self.assertEqual(held.message, "AIMING:TRACKING_BELOW_TRIGGER")
        self.assertEqual(task.target_id, "1")
        self.assertIsNone(task.pending_evidence_request)
        trace = list(task._recent_observations)
        self.assertEqual([row["frame_sequence"] for row in trace], [2, 3])
        self.assertAlmostEqual(trace[-1]["current_target_width_ratio"], 0.19)
        self.assertEqual(trace[-1]["phase"], "AIMING:TRACKING_BELOW_TRIGGER")
        self.assertEqual(trace[-1]["yaw_command"], held.motion.yaw)
        self.assertEqual(trace[-1]["target_id"], "1")
        self.assertEqual(trace[-1]["target_center_x"], 460)
        self.assertEqual(trace[-1]["frame_center_x"], WIDTH / 2)
        self.assertEqual(trace[-1]["x_error_px"], 140)
        self.assertEqual(trace[-1]["task_phase"], "AIMING:TRACKING_BELOW_TRIGGER")
        self.assertEqual(trace[-1]["candidate_source_sequence"], 3)

    def test_center_transitions_and_stability_increments_are_precise(self):
        task = NumberMarkerTask(BASE_CONFIG)
        with redirect_stderr(io.StringIO()):
            self.feed(task, 1, 1.0, x=460)
            self.feed(task, 2, 1.01, x=460)
            self.feed(task, 3, 1.02)
            self.feed(task, 4, 1.03)
            self.feed(task, 5, 1.04, x=460)
            self.feed(task, 6, 1.05)
            self.feed(task, 7, 1.06)
            final = self.feed(task, 8, 1.07)
        self.assertEqual([e["event"] for e in task.diagnostic_events
                          if e["event"].startswith("CENTER_")],
                         ["CENTER_ENTER", "CENTER_EXIT", "CENTER_ENTER"])
        self.assertEqual([e["stable_frames"] for e in self.events(task, "STABLE_FRAME_INCREMENT")],
                         [1, 2, 1, 2, 3])
        resets = self.events(task, "STABLE_FRAMES_RESET")
        self.assertEqual([(e["stable_frames"], e["reset_reason"]) for e in resets],
                         [(0, "CENTER_LOST")])
        self.assertEqual(final.message, "AIM_LOCKED:EVIDENCE_PENDING")

    def test_width_reset_and_evidence_gate_event_order(self):
        task = NumberMarkerTask(BASE_CONFIG)
        with redirect_stderr(io.StringIO()):
            self.feed(task, 1, 1.0)
            self.feed(task, 2, 1.01)
            self.feed(task, 3, 1.02, 0.19)
            self.feed(task, 4, 1.03, 0.23)
            self.feed(task, 5, 1.04, 0.23)
            self.assertIsNone(task.pending_evidence_request)
            final = self.feed(task, 6, 1.05, 0.23)
        self.assertEqual(self.events(task, "STABLE_FRAMES_RESET")[0]["reset_reason"],
                         "WIDTH_BELOW_EVIDENCE_GATE")
        self.assertEqual(final.message, "AIM_LOCKED:EVIDENCE_PENDING")
        names = [event["event"] for event in task.diagnostic_events]
        self.assertLess(names.index("EVIDENCE_READY"), names.index("EVIDENCE_REQUESTED"))
        self.assertEqual(self.events(task, "EVIDENCE_READY")[0]["stable_frames"], 3)
        self.assertGreater(self.events(task, "EVIDENCE_REQUESTED")[0]
                           ["current_target_width_ratio"], 0.20)
        self.assertIsNotNone(task.pending_evidence_request)

    def test_timeout_has_bounded_one_second_trace_and_state(self):
        task = NumberMarkerTask(BASE_CONFIG)
        output = io.StringIO()
        with redirect_stderr(output):
            self.feed(task, 1, 1.0, x=460)
            for sequence in range(2, 162):
                self.feed(task, sequence, 1.0 + (sequence - 1) * 0.05, x=460)
            failed = self.feed(task, 162, 9.05, x=460)
        self.assertEqual(failed.message, "FAILED:TASK_TIMEOUT")
        timeout = self.events(task, "TASK_TIMEOUT")[0]
        trace = timeout["recent_trace"]
        self.assertLessEqual(len(trace), 40)
        self.assertGreaterEqual(trace[-1]["timestamp_monotonic_s"] -
                                trace[0]["timestamp_monotonic_s"], 1.0)
        self.assertEqual(timeout["locked_target_id"], "1")
        self.assertEqual(timeout["task_state"], "FAILED")
        self.assertEqual(timeout["phase"], "FAILED:TASK_TIMEOUT")
        self.assertLess(timeout["task_timeout_remaining_s"], 0.0)
        self.assertEqual(timeout["last_observation"]["phase"], "AIMING")
        self.assertIn('"recent_trace"', output.getvalue())

    def test_diagnostic_serialization_failure_does_not_change_control(self):
        reference = NumberMarkerTask(BASE_CONFIG)
        faulty = NumberMarkerTask(BASE_CONFIG)
        with redirect_stderr(io.StringIO()):
            expected_lock = self.feed(reference, 1, 1.0, x=460)
            expected_aim = self.feed(reference, 2, 1.01, x=460)
        with patch.object(number_marker.json, "dumps", side_effect=RuntimeError("log failed")):
            actual_lock = self.feed(faulty, 1, 1.0, x=460)
            actual_aim = self.feed(faulty, 2, 1.01, x=460)
        self.assertEqual(actual_lock, expected_lock)
        self.assertEqual(actual_aim, expected_aim)
        self.assertEqual(faulty.state, reference.state)
        self.assertEqual(faulty.target_pitch, reference.target_pitch)

    def test_stderr_and_history_failures_do_not_change_control(self):
        class BrokenStream:
            def write(self, _text):
                raise RuntimeError("stderr unavailable")

        class BrokenHistory:
            def clear(self):
                raise RuntimeError("history unavailable")

            def __bool__(self):
                raise RuntimeError("history unavailable")

        reference = NumberMarkerTask(BASE_CONFIG)
        faulty = NumberMarkerTask(BASE_CONFIG)
        faulty._recent_observations = BrokenHistory()
        with redirect_stderr(io.StringIO()):
            expected_lock = self.feed(reference, 1, 1.0, x=460)
            expected_aim = self.feed(reference, 2, 1.01, x=460)
        with patch.object(number_marker.sys, "stderr", BrokenStream()):
            actual_lock = self.feed(faulty, 1, 1.0, x=460)
            actual_aim = self.feed(faulty, 2, 1.01, x=460)
        self.assertEqual(actual_lock, expected_lock)
        self.assertEqual(actual_aim, expected_aim)
        self.assertEqual(faulty.state, reference.state)


class CoordinatorIntegrationTests(unittest.TestCase):
    def test_vertical_request_reaches_existing_gimbal_output(self):
        task = NumberMarkerTask(BASE_CONFIG)
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)

        task.update_candidates((candidate(y=80, now=1.05),))
        harness.feed_line(1.05)
        task.update_candidates((candidate(y=80, now=1.15),))
        decision = harness.feed_line(1.15)

        self.assertEqual(decision.owner, "external")
        self.assertIsInstance(decision.task_update.gimbal, GimbalCommand)
        self.assertGreater(harness.gimbal.last_move()["pitch"], CONFIG.gimbal_pitch)

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
