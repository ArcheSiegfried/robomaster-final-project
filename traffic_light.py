"""WP3 traffic light module: distinguish red / green / none.

The car must hold still at a red light and may only be released after a
green light has been confirmed over several consecutive frames.

Design assumptions (module-local, adjustable, see TrafficLightConfig):
- The light is located in the upper part of the frame (ROI defaults to the
  top 60% of the image; the blue line lives in the lower half, so it does
  not overlap the detection region).
- Red and green lamps appear as saturated HSV blobs of plausible size.
  Blobs that are too small, too large or in the wrong place are discarded.
- Red and green never light at the same time; if both are detected (glare,
  other objects), "red" wins by default: stopping is always safer than an
  unconfirmed release.
- A single green frame is never enough to release. Release (COMPLETED)
  requires `green_confirm_frames` consecutive green frames.
- "No red seen" is never treated as "green". While holding at red, if the
  light disappears we keep holding and fail after `max_hold_seconds`.

Contract (MODULE_GUIDE v0.1): this module never connects to the camera, the
SDK or the chassis. It only returns TaskUpdate requests; step() is
non-blocking (no sleep, no long loops, work is confined to its own ROI).
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from models import (
    FramePacket,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)

KIND = "traffic_light"
RED = "red"
GREEN = "green"

STOP_COMMAND = MotionCommand()

IDLE = "idle"
HOLDING_RED = "holding_red"
CONFIRM_GREEN = "confirm_green"


@dataclass(frozen=True)
class TrafficLightConfig:
    """All field-dependent values live here; tune on the real course."""

    # Detection ROI (fractions of the full frame). Light sits above the line.
    roi_left: float = 0.10
    roi_right: float = 0.90
    roi_top: float = 0.00
    roi_bottom: float = 0.60

    # Red spans the two hue ends of the HSV wheel.
    red_hsv_ranges: Tuple[Tuple[Tuple[int, int, int], Tuple[int, int, int]], ...] = (
        ((0, 100, 60), (10, 255, 255)),
        ((170, 100, 60), (179, 255, 255)),
    )
    green_hsv_lower: Tuple[int, int, int] = (35, 80, 50)
    green_hsv_upper: Tuple[int, int, int] = (85, 255, 255)

    open_kernel: int = 3
    close_kernel: int = 7

    min_area: float = 60.0
    max_area_ratio: float = 0.12
    min_vertical_coverage: float = 0.06

    vertical_weight: float = 1.2
    center_weight: float = 0.8
    area_weight: float = 0.4

    # If both colours are visible in one frame, treat it as red (conservative).
    red_priority: bool = True

    # Confirmation policy.
    red_confirm_frames: int = 2
    green_confirm_frames: int = 5

    # While holding at red, give up after this many seconds without a green.
    max_hold_seconds: float = 15.0


class TrafficLightDetector:
    """Pure colour detector; returns a VisualDetection, never a motion."""

    def __init__(self, settings: Optional[TrafficLightConfig] = None) -> None:
        self.settings = settings or TrafficLightConfig()

    @staticmethod
    def _kernel(size: int) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def _mask(self, hsv: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        s = self.settings
        red_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in s.red_hsv_ranges:
            red_mask = cv2.bitwise_or(
                red_mask,
                cv2.inRange(
                    hsv,
                    np.array(lower, dtype=np.uint8),
                    np.array(upper, dtype=np.uint8),
                ),
            )
        green_mask = cv2.inRange(
            hsv,
            np.array(s.green_hsv_lower, dtype=np.uint8),
            np.array(s.green_hsv_upper, dtype=np.uint8),
        )
        kernel_open = self._kernel(s.open_kernel)
        kernel_close = self._kernel(s.close_kernel)
        red_mask = cv2.morphologyEx(
            red_mask, cv2.MORPH_OPEN, kernel_open
        )
        red_mask = cv2.morphologyEx(
            red_mask, cv2.MORPH_CLOSE, kernel_close
        )
        green_mask = cv2.morphologyEx(
            green_mask, cv2.MORPH_OPEN, kernel_open
        )
        green_mask = cv2.morphologyEx(
            green_mask, cv2.MORPH_CLOSE, kernel_close
        )
        return red_mask, green_mask

    def _score(
        self, mask: np.ndarray, width: int, height: int
    ) -> List[Tuple[float, Tuple[int, int], Tuple[int, int, int, int]]]:
        """Score blobs by size, vertical coverage and horizontal centering."""
        s = self.settings
        frame_area = max(width * height, 1)
        candidates = []
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if area < s.min_area or area > frame_area * s.max_area_ratio:
                continue
            coverage = h / max(height, 1)
            if coverage < s.min_vertical_coverage:
                continue
            center_x = x + w / 2.0
            center_y = y + h / 2.0
            center_distance = abs(center_x - width / 2.0) / max(width / 2.0, 1)
            score = (
                s.vertical_weight * min(coverage, 1.0)
                + s.center_weight * (1.0 - min(center_distance, 1.0))
                + s.area_weight * min((area / frame_area) / 0.10, 1.0)
            )
            candidates.append(
                (
                    float(score),
                    (int(round(center_x)), int(round(center_y))),
                    (x, y, x + w, y + h),
                )
            )
        return candidates

    def detect(self, frame: np.ndarray) -> VisualDetection:
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must be a non-empty BGR image")
        height, width = frame.shape[:2]
        s = self.settings
        left = int(width * s.roi_left)
        right = int(width * s.roi_right)
        top = int(height * s.roi_top)
        bottom = int(height * s.roi_bottom)
        if right <= left or bottom <= top:
            return VisualDetection.no_result("traffic_light")
        roi = frame[top:bottom, left:right]
        if roi.size == 0:
            return VisualDetection.no_result("traffic_light")

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        red_mask, green_mask = self._mask(hsv)
        roi_height, roi_width = roi.shape[:2]
        red_candidates = self._score(red_mask, roi_width, roi_height)
        green_candidates = self._score(green_mask, roi_width, roi_height)

        best_red = max(red_candidates, default=None)
        best_green = max(green_candidates, default=None)
        if best_red is not None and best_green is not None:
            # Both visible: conservative choice, never release on doubt.
            if s.red_priority:
                best_green = None
            else:
                best_red = None

        if best_red is not None:
            return self._detection("red", best_red, left, top)
        if best_green is not None:
            return self._detection("green", best_green, left, top)
        return VisualDetection.no_result("traffic_light")

    @staticmethod
    def _detection(
        color: str,
        candidate: Tuple[float, Tuple[int, int], Tuple[int, int, int, int]],
        left: int,
        top: int,
    ) -> VisualDetection:
        score, center, box = candidate
        full_center = (center[0] + left, center[1] + top)
        full_box = (box[0] + left, box[1] + top, box[2] + left, box[3] + top)
        return VisualDetection(
            valid=True,
            kind="traffic_light",
            center=full_center,
            color=color,
            confidence=min(1.0, max(0.0, score / 4.0)),
            box=full_box,
        )


class TrafficLightTask:
    """Stateful light task; returns TaskUpdate once per frame, non-blocking.

    Status flow (rule: once RUNNING, stay RUNNING until COMPLETED/FAILED):
      idle -> holding_red (red confirmed)      -> RUNNING, zero motion
      holding_red -> confirm_green (green seen) -> RUNNING, zero motion
      confirm_green (from hold, confirmed)      -> COMPLETED (release)
      idle -> confirm_green (green seen)        -> NOT_TRIGGERED until
                                                   confirmed -> COMPLETED
    """

    name = KIND

    def __init__(
        self,
        settings: Optional[TrafficLightConfig] = None,
        detector: Optional[TrafficLightDetector] = None,
    ) -> None:
        self.settings = settings or TrafficLightConfig()
        self.detector = detector or TrafficLightDetector(self.settings)
        self.reset()

    def reset(self) -> None:
        self._state = IDLE
        self._red_streak = 0
        self._green_streak = 0
        self._state_start = 0.0
        self._from_stop = False

    def _enter_hold(self, now: float) -> None:
        self._state = HOLDING_RED
        self._state_start = now
        self._red_streak = 0
        self._green_streak = 0
        self._from_stop = True

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        detection = self.detector.detect(frame.image)
        s = self.settings
        color = detection.color if detection.valid else None

        if self._state == IDLE:
            if color == "red":
                self._red_streak += 1
                if self._red_streak >= s.red_confirm_frames:
                    self._enter_hold(now)
                    return TaskUpdate(
                        TaskStatus.RUNNING,
                        motion=STOP_COMMAND,
                        detection=detection,
                        message="red confirmed; holding",
                    )
                return TaskUpdate(
                    TaskStatus.NOT_TRIGGERED,
                    detection=detection,
                    message="red seen; confirming",
                )
            if color == "green":
                self._state = CONFIRM_GREEN
                self._state_start = now
                self._from_stop = False
                self._green_streak = 1
                if self._green_streak >= s.green_confirm_frames:
                    self._state = IDLE
                    return TaskUpdate(
                        TaskStatus.COMPLETED,
                        detection=detection,
                        message="green confirmed; release",
                    )
                return TaskUpdate(
                    TaskStatus.NOT_TRIGGERED,
                    detection=detection,
                    message="green seen; confirming",
                )
            self._red_streak = 0
            self._green_streak = 0
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED, detection=detection
            )

        if self._state == HOLDING_RED:
            if color == "green":
                self._state = CONFIRM_GREEN
                self._state_start = now
                self._from_stop = True
                self._green_streak = 1
                return TaskUpdate(
                    TaskStatus.RUNNING,
                    motion=STOP_COMMAND,
                    detection=detection,
                    message="green seen while holding; confirming",
                )
            if color == "red":
                return TaskUpdate(
                    TaskStatus.RUNNING,
                    motion=STOP_COMMAND,
                    detection=detection,
                    message="holding at red",
                )
            # Light disappeared while holding: never treat "no red" as green.
            if now - self._state_start > s.max_hold_seconds:
                self._state = IDLE
                return TaskUpdate(
                    TaskStatus.FAILED,
                    detection=detection,
                    message="red hold timeout; failed",
                )
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=STOP_COMMAND,
                detection=detection,
                message="holding at red (light lost)",
            )

        # CONFIRM_GREEN
        if color == "green":
            self._green_streak += 1
            if self._green_streak >= s.green_confirm_frames:
                self._state = IDLE
                self._red_streak = 0
                self._green_streak = 0
                return TaskUpdate(
                    TaskStatus.COMPLETED,
                    detection=detection,
                    message="green confirmed; release",
                )
            if self._from_stop:
                return TaskUpdate(
                    TaskStatus.RUNNING,
                    motion=STOP_COMMAND,
                    detection=detection,
                    message="confirming green while holding",
                )
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED,
                detection=detection,
                message="confirming green",
            )
        if color == "red":
            self._red_streak += 1
            if self._red_streak >= s.red_confirm_frames:
                self._enter_hold(now)
                return TaskUpdate(
                    TaskStatus.RUNNING,
                    motion=STOP_COMMAND,
                    detection=detection,
                    message="back to red; holding",
                )
            if self._from_stop:
                return TaskUpdate(
                    TaskStatus.RUNNING,
                    motion=STOP_COMMAND,
                    detection=detection,
                    message="confirming green (red blip)",
                )
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED,
                detection=detection,
                message="confirming green (red blip)",
            )
        # Light disappeared mid-confirmation.
        if self._from_stop:
            self._enter_hold(now)
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=STOP_COMMAND,
                detection=detection,
                message="green confirmation lost; holding",
            )
        self._state = IDLE
        self._green_streak = 0
        return TaskUpdate(
            TaskStatus.NOT_TRIGGERED,
            detection=detection,
            message="green confirmation lost",
        )
