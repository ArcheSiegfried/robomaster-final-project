import pathlib
import queue
import sys
import time
import unittest

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from camera_source import LatestFrameSource
from config import CONFIG
from controller import LineController
from line_detector import LineDetector
from models import MotionCommand
from motion_output import MotionOutput
from runtime import (
    COASTING,
    LINE_LOST,
    STOPPED,
    TRACKING,
    VIDEO_LOST,
    LineFollower,
)
from examples.offline_takeover import run_demo
from models import TaskStatus, TaskUpdate, VisualDetection


def line_frame(x=320):
    image = np.full((360, 640, 3), 210, np.uint8)
    cv2.line(image, (x, 350), (x, 190), (255, 0, 0), 24)
    return image


def blank_frame():
    return np.full((360, 640, 3), 210, np.uint8)


class FakeChassis:
    def __init__(self):
        self.calls = []

    def drive_speed(self, **kwargs):
        self.calls.append(("speed", kwargs))

    def drive_wheels(self, **kwargs):
        self.calls.append(("wheels", kwargs))


class FakeCamera:
    def __init__(self):
        self.frames = queue.Queue()

    def read_cv2_image(self, strategy, timeout):
        return self.frames.get(timeout=timeout)


class FailingCamera:
    def read_cv2_image(self, strategy, timeout):
        raise RuntimeError("camera failed")


class OfflineTests(unittest.TestCase):
    def test_direction_and_limits(self):
        right = LineDetector(CONFIG.vision).detect(line_frame(420))
        left = LineDetector(CONFIG.vision).detect(line_frame(220))
        self.assertTrue(right.valid and left.valid)
        controller = LineController(CONFIG.control)
        controller.reset(1.0)
        right_command = controller.track(
            right.error, right.heading, 1.05
        )
        controller.reset(1.0)
        left_command = controller.track(left.error, left.heading, 1.05)
        self.assertGreater(right_command.yaw, 0)
        self.assertLess(left_command.yaw, 0)
        self.assertLessEqual(
            abs(right_command.yaw), CONFIG.control.max_yaw_speed
        )

    def test_rejects_large_irrelevant_blob(self):
        image = line_frame(320)
        cv2.rectangle(image, (10, 200), (230, 350), (255, 0, 0), -1)
        detection = LineDetector(CONFIG.vision).detect(image)
        self.assertTrue(detection.valid)
        self.assertLess(abs(detection.error), 0.10)

    def test_brief_loss_slows_and_times_out_without_search(self):
        follower = LineFollower(CONFIG)
        follower.process_frame(line_frame(), 1.0)
        self.assertTrue(follower.resume(1.0))
        moving = follower.process_frame(line_frame(380), 1.05)
        missed = follower.process_frame(blank_frame(), 1.10)
        self.assertEqual(missed.state, COASTING)
        self.assertLessEqual(
            missed.command.forward, moving.command.forward
        )
        self.assertLessEqual(
            abs(missed.command.yaw), CONFIG.control.lost_max_yaw
        )
        timed_out = follower.process_frame(
            blank_frame(),
            1.05 + CONFIG.control.lost_grace_seconds + 0.01,
        )
        self.assertEqual(timed_out.state, LINE_LOST)
        self.assertTrue(timed_out.force_stop)
        self.assertEqual(timed_out.command, MotionCommand())

    def test_recovery_is_rate_limited(self):
        follower = LineFollower(CONFIG)
        follower.process_frame(line_frame(), 2.0)
        follower.resume(2.0)
        follower.process_frame(line_frame(390), 2.05)
        coast = follower.process_frame(blank_frame(), 2.10)
        recovered = follower.process_frame(line_frame(390), 2.15)
        self.assertEqual(recovered.state, TRACKING)
        self.assertLessEqual(
            abs(recovered.command.yaw - coast.command.yaw),
            CONFIG.control.max_yaw_rate * 0.05 + 1e-9,
        )
        self.assertLessEqual(
            recovered.command.forward,
            coast.command.forward
            + CONFIG.control.max_forward_acceleration * 0.05
            + 1e-9,
        )

    def test_pause_cannot_be_overridden_by_detection(self):
        follower = LineFollower(CONFIG)
        follower.process_frame(line_frame(), 3.0)
        follower.resume(3.0)
        follower.pause(3.1)
        decision = follower.process_frame(line_frame(400), 3.2)
        self.assertEqual(decision.state, STOPPED)
        self.assertEqual(decision.command, MotionCommand())

    def test_fault_requires_reset_and_fresh_valid_line(self):
        follower = LineFollower(CONFIG)
        follower.process_frame(line_frame(), 4.0)
        follower.resume(4.0)
        follower.process_frame(blank_frame(), 4.5)
        self.assertFalse(follower.resume(4.5))
        follower.reset_fault(4.6)
        self.assertFalse(follower.resume(4.6))
        follower.process_frame(line_frame(), 4.7)
        self.assertTrue(follower.resume(4.7))

    def test_stale_valid_line_cannot_resume(self):
        follower = LineFollower(CONFIG)
        follower.process_frame(line_frame(), 4.0)
        self.assertFalse(
            follower.resume(4.0 + CONFIG.resume_detection_max_age + 0.01)
        )

    def test_video_loss_is_separate_and_latched(self):
        follower = LineFollower(CONFIG)
        follower.process_frame(line_frame(), 5.0)
        follower.resume(5.0)
        decision = follower.process_video_gap(
            CONFIG.video_gap_stop_seconds, 5.3
        )
        self.assertEqual(decision.state, VIDEO_LOST)
        self.assertTrue(decision.force_stop)
        after = follower.process_frame(line_frame(), 5.4)
        self.assertEqual(after.state, VIDEO_LOST)

    def test_latest_frame_source_returns_a_new_packet(self):
        camera = FakeCamera()
        source = LatestFrameSource(camera, "newest", 0.01)
        camera.frames.put(line_frame(300))
        camera.frames.put(line_frame(340))
        source.start()
        deadline = time.monotonic() + 0.3
        packet = None
        while packet is None and time.monotonic() < deadline:
            packet = source.wait_after(0, 0.02)
        self.assertIsNotNone(packet)
        source.close()

    def test_camera_failure_is_propagated_to_control_thread(self):
        source = LatestFrameSource(FailingCamera(), "newest", 0.01)
        source.start()
        with self.assertRaisesRegex(RuntimeError, "camera failed"):
            source.wait_after(0, 0.1)
        source.close()

    def test_single_motion_outlet_and_hard_stop(self):
        chassis = FakeChassis()
        output = MotionOutput(chassis, CONFIG)
        output.claim("external")
        with self.assertRaises(RuntimeError):
            output.send("line", MotionCommand(0.2, 0, 0))
        output.send("external", MotionCommand(0.1, 0, 5))
        output.hard_stop()
        self.assertEqual(chassis.calls[-1][0], "wheels")
        self.assertEqual(
            chassis.calls[-1][1],
            {"w1": 0, "w2": 0, "w3": 0, "w4": 0},
        )

    def test_importing_main_does_not_import_robomaster(self):
        sys.modules.pop("main", None)
        sys.modules.pop("robomaster", None)
        __import__("main")
        self.assertNotIn("robomaster", sys.modules)

    def test_common_task_contracts_represent_no_result_and_status(self):
        missing = VisualDetection.no_result("marker")
        update = TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=missing)
        self.assertFalse(update.detection.valid)
        self.assertIsNone(update.motion)

    def test_offline_takeover_example_completes_and_resumes(self):
        self.assertEqual(
            run_demo(),
            [
                "line:TRACKING",
                "owner:external",
                "task:running",
                "task:completed",
                "line:TRACKING",
            ],
        )


if __name__ == "__main__":
    unittest.main()
