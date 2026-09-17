"""Focused image-only checks for the no-calibration gap approach."""

import unittest

from config import CONFIG
from models import FramePacket, TaskStatus
from route import (
    ALIGNING,
    LOW_APPROACH,
    LOW_APPROACH_SPEED,
    LOW_NEAR_SPEED,
    RouteTask,
)
from route_detector import RouteVision
from tests.test_route import segment_frame, settle_into_bridge, start_and_trigger


def endpoint_frame(row):
    return segment_frame((310, row), (610, row))


class LowViewApproachTests(unittest.TestCase):
    def test_full_task_path_enters_low_view_before_turn(self):
        task, harness, _ = start_and_trigger()
        settle_into_bridge(harness)
        for now in (2.4, 2.6, 2.8, 3.0, 3.05, 3.1):
            decision = harness.feed_image(now, endpoint_frame(310))
        self.assertEqual(task.state, LOW_APPROACH)
        self.assertEqual(decision.command.forward, 0.0)
        self.assertEqual(harness.gimbal.last_move()["pitch"], CONFIG.gimbal_pitch)

        for now in (3.7, 3.75, 3.8):
            decision = harness.feed_image(now, endpoint_frame(260))
        self.assertEqual(decision.command.forward, LOW_APPROACH_SPEED)
        for now in (3.85, 3.9, 3.95):
            decision = harness.feed_image(now, endpoint_frame(325))
        self.assertEqual(task.state, ALIGNING)
        self.assertEqual(decision.command.forward, 0.0)

    def setUp(self):
        self.task = RouteTask()
        self.task._candidate = RouteVision(CONFIG.vision).candidates(
            endpoint_frame(310)
        )[0]
        self.task._start_low_approach(1.0)

    def feed(self, sequence, now, row):
        return self.task._step_low_approach(
            FramePacket(endpoint_frame(row), sequence, now), now
        )

    def test_stops_for_gimbal_then_approaches_and_turns_on_fresh_rows(self):
        settling = self.feed(1, 1.2, 310)
        self.assertEqual(self.task.state, LOW_APPROACH)
        self.assertEqual(settling.motion.forward, 0.0)
        self.assertEqual(settling.gimbal.pitch, CONFIG.gimbal_pitch)

        self.feed(2, 1.8, 260)
        self.feed(3, 1.85, 260)
        far = self.feed(4, 1.9, 260)
        self.assertEqual(far.motion.forward, LOW_APPROACH_SPEED)
        self.assertTrue(far.detection.valid)
        near = self.feed(5, 1.95, 290)
        self.assertEqual(near.motion.forward, LOW_NEAR_SPEED)

        self.feed(6, 2.0, 325)
        duplicate = self.feed(6, 2.01, 325)
        self.assertEqual(duplicate.motion.forward, LOW_NEAR_SPEED)
        self.assertEqual(self.task._low_turn_frames, 1)
        self.feed(7, 2.05, 325)
        turn = self.feed(8, 2.1, 325)
        self.assertEqual(turn.motion.forward, 0.0)
        self.assertEqual(self.task.state, ALIGNING)
        self.assertEqual(self.task._align_phase, "base_rotate")

    def test_lost_low_view_endpoint_stops_and_fails_without_blind_motion(self):
        for sequence, now in ((2, 1.8), (3, 1.85), (4, 1.9)):
            self.feed(sequence, now, 260)
        blank = segment_frame((0, 0), (0, 0))
        waiting = self.task._step_low_approach(
            FramePacket(blank, 5, 1.95), 1.95
        )
        self.assertEqual(waiting.motion.forward, 0.0)
        failed = self.task._step_low_approach(
            FramePacket(blank, 6, 2.7), 2.7
        )
        self.assertEqual(failed.status, TaskStatus.FAILED)
        self.assertEqual(failed.motion.forward, 0.0)

    def test_opposite_endpoint_is_not_substituted_after_view_change(self):
        # The far end of the same horizontal strip points the other way.
        # It must not replace the gap-facing endpoint merely due to score.
        for sequence, now in ((2, 1.8), (3, 1.85), (4, 1.9)):
            self.feed(sequence, now, 260)
        self.assertLess(self.task._candidate.entry_endpoint.point[0], 400)


if __name__ == "__main__":
    unittest.main()
