"""红绿灯识别与停车/放行（WP3 / Issue #3）。

契约测试在前，成员用例区在后。骨架已经把本文件注册进 task_registry，
你只要替换上面的实现即可，不需要改 main.py。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
)

from traffic_light import TrafficLightTask  # noqa: E402


class TrafficLightContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "traffic_light.py")

    def test_does_not_take_over_on_a_plain_line_frame(self):
        """合成帧里只有一条蓝线，没有任何灯。

        这条测试在你实现完之后**仍然必须通过**：没看到灯就不许接管，
        更不许把"没看到红灯"当成绿灯放行。
        """
        assert_inert_through_harness(self, TrafficLightTask())


# ============================================================================
# 成员用例区
# ============================================================================

import cv2
import numpy as np

from models import FramePacket, TaskStatus
from traffic_light import (
    CONFIRM_GREEN,
    HOLDING_RED,
    TrafficLightConfig,
    TrafficLightDetector,
    TrafficLightTask,
)

WIDTH, HEIGHT = 640, 360
GRAY = (210, 210, 210)

def blank_frame():
    return np.full((HEIGHT, WIDTH, 3), GRAY, np.uint8)


def line_frame(x=320):
    image = blank_frame()
    cv2.line(image, (x, HEIGHT - 10), (x, 190), (255, 0, 0), 24)
    return image


def light_frame(color="red", center=(320, 120), radius=22):
    image = blank_frame()
    bgr = {"red": (0, 0, 220), "green": (0, 200, 0)}.get(color)
    if bgr is not None:
        cv2.circle(image, center, radius, bgr, -1)
    return image


def packet(image, seq, t):
    return FramePacket(image, seq, t)





# --------------------------------------------------------------------------

class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.detector = TrafficLightDetector()

    def test_detects_red(self):
        result = self.detector.detect(light_frame("red"))
        self.assertTrue(result.valid)
        self.assertEqual(result.color, "red")

    def test_detects_green(self):
        result = self.detector.detect(light_frame("green"))
        self.assertTrue(result.valid)
        self.assertEqual(result.color, "green")

    def test_no_result_on_blank_and_blue_line(self):
        self.assertFalse(self.detector.detect(blank_frame()).valid)
        self.assertFalse(self.detector.detect(line_frame()).valid)

    def test_no_result_when_blob_too_small(self):
        result = self.detector.detect(light_frame("red", radius=3))
        self.assertFalse(result.valid)

    def test_red_wins_when_both_colours_visible(self):
        image = blank_frame()
        cv2.circle(image, (260, 120), 22, (0, 0, 220), -1)
        cv2.circle(image, (380, 120), 22, (0, 200, 0), -1)
        result = self.detector.detect(image)
        self.assertTrue(result.valid)
        self.assertEqual(result.color, "red")

    def test_center_and_box_are_full_frame_coordinates(self):
        result = self.detector.detect(light_frame("red", center=(320, 120)))
        self.assertTrue(result.valid)
        cx, cy = result.center
        self.assertGreaterEqual(cx, 64)
        self.assertGreaterEqual(cy, 0)
        self.assertLess(cx, 576)
        self.assertLess(cy, 216)
        left, top, right, bottom = result.box
        self.assertGreaterEqual(left, 64)
        self.assertLess(right, 576)


# --------------------------------------------------------------------------
# Task state machine tests
# --------------------------------------------------------------------------

class TaskTests(unittest.TestCase):
    def frames(self, sequence, start, step=0.05):
        """Iterate (image, seq, t) from a list of colour names."""
        for i, name in enumerate(sequence):
            if name == "red":
                image = light_frame("red")
            elif name == "green":
                image = light_frame("green")
            elif name == "line":
                image = line_frame()
            else:
                image = blank_frame()
            yield image, i, start + i * step

    def run_sequence(self, names, start=10.0):
        updates = []
        task = TrafficLightTask()
        for image, i, t in self.frames(names, start):
            updates.append(task.step(packet(image, i, t), t))
        return task, updates

    def test_red_holds_with_zero_motion(self):
        task, updates = self.run_sequence(["red", "red", "red", "red"])
        self.assertEqual(updates[0].status, TaskStatus.NOT_TRIGGERED)
        for update in updates[1:]:
            self.assertEqual(update.status, TaskStatus.RUNNING)
            self.assertIsNotNone(update.motion)
            self.assertEqual(update.motion.forward, 0.0)
            self.assertEqual(update.motion.lateral, 0.0)
            self.assertEqual(update.motion.yaw, 0.0)

    def test_green_release_requires_confirmation(self):
        # Two red frames -> hold; then five green frames -> COMPLETED.
        seq = ["red", "red", "green", "green", "green", "green", "green"]
        task, updates = self.run_sequence(seq)
        self.assertEqual(updates[0].status, TaskStatus.NOT_TRIGGERED)
        self.assertEqual(updates[1].status, TaskStatus.RUNNING)
        # While stopped, green confirmation keeps RUNNING with zero motion.
        for update in updates[2:-1]:
            self.assertEqual(update.status, TaskStatus.RUNNING)
            self.assertEqual(update.motion.forward, 0.0)
        self.assertEqual(updates[-1].status, TaskStatus.COMPLETED)

    def test_single_green_blip_never_releases(self):
        # One green frame while holding is never enough: the counter resets.
        seq = ["red", "red", "green", "red", "red"]
        task, updates = self.run_sequence(seq)
        self.assertEqual(updates[0].status, TaskStatus.NOT_TRIGGERED)
        for update in updates[1:]:
            self.assertEqual(update.status, TaskStatus.RUNNING)
            self.assertEqual(update.motion.forward, 0.0)
        # Five fresh consecutive green frames are required before release.
        for i in range(4):
            update = task.step(
                packet(light_frame("green"), 10 + i, 12.0 + i * 0.05),
                12.0 + i * 0.05,
            )
            self.assertEqual(update.status, TaskStatus.RUNNING)
        final = task.step(packet(light_frame("green"), 14, 12.2), 12.2)
        self.assertEqual(final.status, TaskStatus.COMPLETED)

    def test_green_confirmation_interrupted_by_red(self):
        seq = ["red", "red", "green", "green", "red", "red", "red"]
        task, updates = self.run_sequence(seq)
        self.assertEqual(updates[2].status, TaskStatus.RUNNING)
        self.assertEqual(updates[3].status, TaskStatus.RUNNING)
        # Back to red: still holding, no release.
        for update in updates[4:]:
            self.assertEqual(update.status, TaskStatus.RUNNING)
            self.assertEqual(update.motion.forward, 0.0)

    def test_light_lost_while_holding_stays_stopped(self):
        seq = ["red", "red", "none", "none", "none"]
        task, updates = self.run_sequence(seq)
        for update in updates[1:]:
            self.assertEqual(update.status, TaskStatus.RUNNING)
            self.assertEqual(update.motion.forward, 0.0)

    def test_hold_timeout_fails(self):
        task = TrafficLightTask()
        t = 20.0
        task.step(packet(light_frame("red"), 0, t), t)
        task.step(packet(light_frame("red"), 1, t + 0.05), t + 0.05)
        failed = None
        for i in range(400):
            t += 0.05
            update = task.step(packet(blank_frame(), 2 + i, t), t)
            if update.status == TaskStatus.FAILED:
                failed = update
                break
        self.assertIsNotNone(failed)
        self.assertEqual(failed.status, TaskStatus.FAILED)

    def test_green_from_idle_completes_without_takeover(self):
        seq = ["green", "green", "green", "green", "green"]
        task, updates = self.run_sequence(seq)
        for update in updates[:-1]:
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertEqual(updates[-1].status, TaskStatus.COMPLETED)

    def test_green_blip_from_idle_does_not_trigger(self):
        seq = ["green", "none", "none"]
        task, updates = self.run_sequence(seq)
        for update in updates:
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
            self.assertIsNone(update.motion)

    def test_no_light_never_interferes(self):
        task = TrafficLightTask()
        for i in range(12):
            update = task.step(packet(blank_frame(), i, 5.0 + i * 0.05), 5.0 + i * 0.05)
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
            self.assertIsNone(update.motion)

    def test_red_priority_setting_allows_green_first(self):
        settings = TrafficLightConfig(red_priority=False)
        task = TrafficLightTask(settings)
        image = blank_frame()
        cv2.circle(image, (260, 120), 22, (0, 0, 220), -1)
        cv2.circle(image, (380, 120), 22, (0, 200, 0), -1)
        update = task.step(packet(image, 0, 1.0), 1.0)
        self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)

    def test_reset_returns_to_idle(self):
        task = TrafficLightTask()
        task.step(packet(light_frame("red"), 0, 1.0), 1.0)
        task.step(packet(light_frame("red"), 1, 1.05), 1.05)
        self.assertEqual(task._state, HOLDING_RED)
        task.reset()
        self.assertEqual(task._state, "idle")
        update = task.step(packet(blank_frame(), 2, 1.10), 1.10)
        self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)

    def test_confirm_green_state_visible(self):
        task = TrafficLightTask()
        task.step(packet(light_frame("red"), 0, 1.0), 1.0)
        task.step(packet(light_frame("red"), 1, 1.05), 1.05)
        task.step(packet(light_frame("green"), 2, 1.10), 1.10)
        self.assertEqual(task._state, CONFIRM_GREEN)

    def test_detection_reported_in_updates(self):
        task = TrafficLightTask()
        update = task.step(packet(light_frame("red"), 0, 1.0), 1.0)
        self.assertIsNotNone(update.detection)
        self.assertEqual(update.detection.kind, "traffic_light")
        update = task.step(packet(light_frame("red"), 1, 1.05), 1.05)
        self.assertTrue(update.detection.valid)
        self.assertEqual(update.detection.color, "red")


# --------------------------------------------------------------------------
# Integration: task plugged into the real framework with a fake chassis
# --------------------------------------------------------------------------

class IntegrationTests(unittest.TestCase):
    def test_full_flow_in_framework(self):
        from config import CONFIG
        from motion_output import MotionOutput
        from runtime import TRACKING, LineFollower
        from examples.offline_takeover import FakeChassis

        follower = LineFollower(CONFIG)
        output = MotionOutput(FakeChassis(), CONFIG)
        task = TrafficLightTask()

        # 1. Normal line following is running.
        first = packet(line_frame(320), 1, 1.00)
        follower.process_frame(first.image, first.captured_at)
        self.assertTrue(follower.resume(first.captured_at))

        # 2. Red light: coordinator pauses the line, task takes over.
        follower.pause(1.10)
        output.claim("external")
        t = 1.10
        task.step(packet(light_frame("red"), 2, t), t)
        t += 0.05
        update = task.step(packet(light_frame("red"), 3, t), t)
        self.assertEqual(update.status, TaskStatus.RUNNING)
        if update.motion is not None:
            output.send("external", update.motion)

        # 3. Green light: five confirmed frames -> COMPLETED.
        last = None
        for i in range(5):
            t += 0.05
            last = task.step(packet(light_frame("green"), 4 + i, t), t)
            if last.motion is not None:
                output.send("external", last.motion)
        self.assertEqual(last.status, TaskStatus.COMPLETED)

        # 4. Hand back: zero motion, return ownership, clear history,
        #    validate a fresh line, then explicit resume.
        output.hard_stop()
        output.claim("line")
        follower.reset_fault(t)
        fresh = packet(line_frame(330), 9, t + 0.01)
        follower.process_frame(fresh.image, fresh.captured_at)
        self.assertTrue(follower.resume(fresh.captured_at))
        self.assertEqual(follower.state, TRACKING)




if __name__ == "__main__":
    unittest.main()
