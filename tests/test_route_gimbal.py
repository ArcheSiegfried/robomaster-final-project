"""Small synthetic checks for the route-only side-looking experiment."""

import types
import time
import unittest

from config import CONFIG
from main import build_coordinator
from models import FramePacket, GimbalCommand, TaskStatus
from motion_output import MotionOutput
from route import REACQUIRING
from route_detector import RouteVision
from route_gimbal import (
    BODY_TURN,
    SIDE_ADVANCE,
    SIDE_AIM,
    GimbalAlignedRouteTask,
)
from route_gimbal_output import RouteGimbalOutput
from runtime import LineFollower
from tests.task_harness import (
    FakeChassis, FakeGimbal, assert_module_source_is_clean,
)
from tests.test_route import segment_frame


def frame(sequence, now, x):
    return FramePacket(segment_frame((x, 295), (x, 80)), sequence, now)


def aimed_task():
    task = GimbalAlignedRouteTask()
    # The physical end leads left; its position need not itself be left of
    # image center. This was a real source of reversed body turns.
    task._candidate = RouteVision(CONFIG.vision).candidates(
        segment_frame((145, 310), (445, 310))
    )[0]
    task.started_at = 1.0
    task._start_low_approach(1.0)
    return task


class SideLookingRouteTests(unittest.TestCase):
    def test_task_has_no_sdk_or_second_camera_access(self):
        assert_module_source_is_clean(self, "route_gimbal.py")

    def test_logged_endpoint_just_below_old_near_band_starts_side_aim(self):
        task = GimbalAlignedRouteTask()
        image = segment_frame((130, 280), (430, 280))
        packet = FramePacket(image, 1, 1.0)
        candidates = task._candidate_variants(
            RouteVision(CONFIG.vision).candidates(image), 640, 360
        )
        task._candidate = next(
            item for item in candidates if item.entry_endpoint.tangent_deg < 0
        )
        task._candidate_frames = 3
        task._candidate_seen_far = True
        task._pose_forward = 0.16
        task._old_tangent_world = 0.0
        self.assertLess(task._candidate.bottom_ratio, 0.80)
        self.assertTrue(task._candidate_ready(packet)[0])
        task.started_at = 1.0
        task._start_low_approach(1.0)
        self.assertEqual(task.state, SIDE_AIM)
        self.assertLess(task._aim_yaw, -75.0)
        self.assertEqual(task.step(packet, 1.4).gimbal.yaw, task._aim_yaw)

    def test_side_aim_does_not_accept_unconfirmed_fragment(self):
        task = GimbalAlignedRouteTask()
        image = segment_frame((130, 280), (430, 280))
        candidate = task._candidate_variants(
            RouteVision(CONFIG.vision).candidates(image), 640, 360
        )[0]
        task._candidate = candidate
        task._candidate_frames = 3
        task._pose_forward = 0.16
        task._old_tangent_world = 0.0
        packet = FramePacket(image, 1, 1.0)
        self.assertFalse(task._candidate_ready(packet)[0])
        task._candidate_seen_far = True
        task._candidate_frames = 2
        self.assertFalse(task._candidate_ready(packet)[0])

    def test_one_aim_forward_crossing_then_body_turn(self):
        task = aimed_task()
        self.assertEqual(task.state, SIDE_AIM)
        self.assertLess(task._aim_yaw, -75.0)
        stopping = task.step(frame(1, 1.1, 500), 1.1)
        self.assertEqual(stopping.gimbal.yaw, 0.0)
        waiting = task.step(frame(2, 1.5, 500), 1.5)
        self.assertEqual(waiting.motion.forward, 0.0)
        self.assertEqual(waiting.gimbal.yaw, task._aim_yaw)

        crossing = task.step(frame(3, 3.4, 500), 3.4)
        self.assertEqual(task.state, SIDE_ADVANCE)
        self.assertGreater(crossing.motion.forward, 0.0)
        self.assertEqual(crossing.motion.lateral, 0.0)
        self.assertEqual(crossing.motion.yaw, 0.0)
        task.step(frame(4, 3.5, 420), 3.5)
        task.step(frame(5, 3.6, 340), 3.6)
        task.step(frame(6, 3.7, 320), 3.7)
        task.step(frame(7, 3.8, 320), 3.8)
        centered = task.step(frame(8, 3.9, 320), 3.9)
        self.assertEqual(task.state, BODY_TURN)
        self.assertEqual(centered.motion.forward, 0.0)

        now = 3.9
        for sequence in range(9, 75):
            now += 0.1
            update = task.step(frame(sequence, now, 320), now)
            if task.state == REACQUIRING:
                break
            self.assertEqual(update.motion.forward, 0.0)
            self.assertEqual(update.motion.lateral, 0.0)
            self.assertLess(update.motion.yaw, 0.0)
        self.assertEqual(task.state, REACQUIRING)
        self.assertEqual(update.gimbal.yaw, 0.0)
        self.assertEqual(update.motion.yaw, 0.0)

    def test_wrong_side_or_missing_line_stops_without_blind_advance(self):
        task = aimed_task()
        task.step(frame(1, 1.5, 160), 1.5)
        wrong_side = task.step(frame(2, 3.4, 160), 3.4)
        self.assertEqual(wrong_side.status, TaskStatus.FAILED)
        self.assertEqual(wrong_side.motion.forward, 0.0)

        task = aimed_task()
        blank = FramePacket(segment_frame((0, 0), (0, 0)), 1, 1.5)
        task.step(blank, 1.5)
        waiting = task.step(blank, 3.4)
        self.assertEqual(waiting.motion.forward, 0.0)
        failed = task.step(blank, 4.0)
        self.assertEqual(failed.status, TaskStatus.FAILED)
        self.assertEqual(failed.motion.forward, 0.0)

    def test_right_turn_uses_opposite_side_view_sign(self):
        task = GimbalAlignedRouteTask()
        task._candidate = RouteVision(CONFIG.vision).candidates(
            segment_frame((195, 310), (495, 310))
        )[0]
        task.started_at = 1.0
        task._start_low_approach(1.0)
        self.assertGreater(task._aim_yaw, 75.0)
        task.step(frame(1, 1.5, 160), 1.5)
        crossing = task.step(frame(2, 3.4, 160), 3.4)
        self.assertEqual(task.state, SIDE_ADVANCE)
        self.assertGreater(crossing.motion.forward, 0.0)
        self.assertEqual(crossing.motion.lateral, 0.0)

    def test_side_approach_has_real_time_limit(self):
        task = aimed_task()
        task.step(frame(1, 1.5, 500), 1.5)
        task.step(frame(2, 3.4, 500), 3.4)
        failed = task.step(frame(3, 7.3, 500), 7.3)
        self.assertEqual(failed.status, TaskStatus.FAILED)
        self.assertEqual(failed.motion.forward, 0.0)


class _AngleGimbal(FakeGimbal):
    def __init__(self):
        super().__init__()
        self.unsubscribed = False

    def sub_angle(self, freq, callback):
        callback((0.0, 0.0, 0.0, 35.0))
        return True

    def unsub_angle(self):
        self.unsubscribed = True
        return True


class _FakeRobot:
    def __init__(self):
        self.gimbal = _AngleGimbal()
        self.modes = []

    def set_robot_mode(self, mode):
        self.modes.append(mode)
        return True


class ModeAwareGimbalTests(unittest.TestCase):
    def test_experiment_uses_real_coordinator_injection(self):
        task = GimbalAlignedRouteTask()
        coordinator = build_coordinator(
            LineFollower(CONFIG),
            MotionOutput(FakeChassis(), CONFIG),
            capture_directory=None,
            motion_tasks_override=(task,),
        )
        self.assertEqual(coordinator.motion_tasks, (task,))

    def test_switches_free_for_one_aim_and_restores_chassis_lead(self):
        robot = _FakeRobot()
        modes = types.SimpleNamespace(FREE="free", CHASSIS_LEAD="chassis")
        output = RouteGimbalOutput(robot, CONFIG, modes)
        aim = GimbalCommand(pitch=-12, yaw=-90)
        output.send(aim)
        output.send(aim)
        self.assertEqual(robot.modes, ["free"])
        # SDK moveto uses the power-on frame, not the current body frame.
        self.assertEqual(robot.gimbal.last_move()["yaw"], -55)
        output.restore_line_view()
        self.assertEqual(robot.modes, ["free", "chassis"])
        self.assertEqual(robot.gimbal.last_move()["pitch"], CONFIG.gimbal_pitch)
        output.close()
        self.assertTrue(robot.gimbal.unsubscribed)

    def test_failure_restore_also_returns_to_chassis_lead(self):
        robot = _FakeRobot()
        modes = types.SimpleNamespace(FREE="free", CHASSIS_LEAD="chassis")
        output = RouteGimbalOutput(robot, CONFIG, modes)
        output.send(GimbalCommand(pitch=-12, yaw=90))
        output.restore_line_view()
        self.assertEqual(robot.modes, ["free", "chassis"])

    def test_stale_yaw_refuses_aim_before_mode_change(self):
        robot = _FakeRobot()
        modes = types.SimpleNamespace(FREE="free", CHASSIS_LEAD="chassis")
        output = RouteGimbalOutput(robot, CONFIG, modes)
        output._angle_at = time.monotonic() - 1.0
        with self.assertRaisesRegex(RuntimeError, "no fresh gimbal yaw"):
            output.send(GimbalCommand(pitch=-12, yaw=-90))
        self.assertEqual(robot.modes, [])


if __name__ == "__main__":
    unittest.main()
