"""Pause/resume, short-loss tolerance and fail-locked stop states."""

import time
from typing import Optional

from config import RuntimeConfig
from controller import LineController
from line_detector import LineDetector
from models import LineDetection, RuntimeDecision, STOP_COMMAND

STOPPED = "STOPPED"
TRACKING = "TRACKING"
COASTING = "COASTING"
LINE_LOST = "LINE_LOST"
VIDEO_LOST = "VIDEO_LOST"


class LineFollower:
    def __init__(self, settings: RuntimeConfig) -> None:
        self.settings = settings
        self.detector = LineDetector(settings.vision)
        self.controller = LineController(settings.control)
        self.state = STOPPED
        self._last_detection: Optional[LineDetection] = None
        self._last_detection_time: Optional[float] = None
        self._last_valid_time: Optional[float] = None

    @property
    def motion_enabled(self) -> bool:
        return self.state in (TRACKING, COASTING)

    def process_frame(
        self, frame, captured_at: Optional[float] = None
    ) -> RuntimeDecision:
        now = time.monotonic() if captured_at is None else captured_at
        detection = self.detector.detect(frame)
        self._last_detection = detection
        self._last_detection_time = now

        if detection.valid:
            was_coasting = self.state == COASTING
            self._last_valid_time = now
            if not self.motion_enabled:
                return RuntimeDecision(
                    self.state,
                    detection,
                    message="valid line; explicit resume required",
                )
            self.state = TRACKING
            command = self.controller.track(
                detection.error,
                detection.heading,
                now,
                recovering=was_coasting,
            )
            return RuntimeDecision(self.state, detection, command)

        if not self.motion_enabled:
            return RuntimeDecision(self.state, detection)
        elapsed = (
            float("inf")
            if self._last_valid_time is None
            else max(0.0, now - self._last_valid_time)
        )
        if elapsed <= self.settings.control.lost_grace_seconds:
            self.state = COASTING
            return RuntimeDecision(
                self.state,
                detection,
                self.controller.coast(elapsed, now),
                message="brief line miss",
            )

        self.state = LINE_LOST
        self.controller.stop(now)
        self.detector.reset()
        return RuntimeDecision(
            self.state,
            detection,
            STOP_COMMAND,
            True,
            "line-loss timeout; reset and resume required",
        )

    def process_video_gap(
        self, frame_age: float, timestamp: Optional[float] = None
    ) -> RuntimeDecision:
        now = time.monotonic() if timestamp is None else timestamp
        detection = self._last_detection or LineDetector.empty_detection()
        if frame_age >= self.settings.video_gap_stop_seconds:
            self.state = VIDEO_LOST
            self.controller.stop(now)
            self.detector.reset()
            return RuntimeDecision(
                self.state,
                detection,
                STOP_COMMAND,
                True,
                "video stream timeout; reset and resume required",
            )
        # This is not an image detection miss. The SDK timeout expires the last
        # drive command while the loop waits for a genuinely new camera frame.
        return RuntimeDecision(
            self.state,
            detection,
            STOP_COMMAND,
            False,
            "waiting for a new camera frame",
        )

    def pause(self, timestamp: Optional[float] = None) -> None:
        now = time.monotonic() if timestamp is None else timestamp
        self.state = STOPPED
        self.controller.stop(now)
        self.detector.reset()
        self._last_detection = None
        self._last_detection_time = None
        self._last_valid_time = None

    def reset_fault(self, timestamp: Optional[float] = None) -> None:
        self.pause(timestamp)

    def resume(self, timestamp: Optional[float] = None) -> bool:
        now = time.monotonic() if timestamp is None else timestamp
        fresh = (
            self._last_detection_time is not None
            and now - self._last_detection_time
            <= self.settings.resume_detection_max_age
        )
        if (
            self.state != STOPPED
            or not fresh
            or self._last_detection is None
            or not self._last_detection.valid
        ):
            return False
        self.controller.reset(now)
        self.detector.reset()
        self._last_valid_time = now
        self.state = TRACKING
        return True
