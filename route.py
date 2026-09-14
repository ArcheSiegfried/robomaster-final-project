"""Bounded long-gap recovery task (WP6a / Issue #6).

The base follower owns brief misses. This task starts only after that grace
period, raises the shared camera view through a typed request, and performs one
small non-blocking recovery step per frame.
"""

from typing import Optional, Tuple

import cv2
import numpy as np

from config import CONFIG
from line_detector import LineDetector
from models import (
    FramePacket,
    GimbalCommand,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)

KIND = "route"
MONITORING = "monitoring"
RAISING_VIEW = "raising_view"
BRIDGING = "bridging"
SEARCHING = "searching"
REACQUIRING = "reacquiring"

# Conservative offline defaults. None is claimed as real-robot tuned.
TRIGGER_MARGIN_SECONDS = 0.05
VIEW_SETTLE_SECONDS = 0.45
TOTAL_RECOVERY_SECONDS = 5.0
BRIDGE_MAX_SECONDS = 1.8
FRAGMENT_LOSS_SECONDS = 0.18
BRIDGE_FORWARD_SPEED = 0.07
BRIDGE_YAW_GAIN = 32.0
BRIDGE_MAX_YAW = 25.0
SEARCH_YAW_SPEED = 45.0
INITIAL_SWEEP_SECONDS = 0.45
SWEEP_STEP_SECONDS = 0.45
REACQUIRE_STABLE_FRAMES = 3
REACQUIRE_MAX_CENTER_JUMP = 0.20
FAR_CONFIRM_FRAMES = 2
SEARCH_ROI_TOP = 0.25
SEARCH_ROI_BOTTOM = 0.96


class RouteTask:
    """Recover a configured-colour route without touching robot hardware."""

    name = "route"

    def __init__(self, settings: Optional[object] = None) -> None:
        self.settings = CONFIG if settings is None else settings
        vision = getattr(self.settings, "vision", CONFIG.vision)
        control = getattr(self.settings, "control", CONFIG.control)
        self._line_detector = LineDetector(vision)
        self._vision = vision
        self._lost_grace = float(control.lost_grace_seconds)
        self._search_pitch = float(
            getattr(self.settings, "gimbal_search_pitch", CONFIG.gimbal_search_pitch)
        )
        self._search_yaw = float(
            getattr(self.settings, "gimbal_yaw", CONFIG.gimbal_yaw)
        )

        self.last_detection = VisualDetection.no_result(KIND)
        self.state = MONITORING
        self.started_at: Optional[float] = None
        self._phase_started_at: Optional[float] = None
        self._last_clear_at: Optional[float] = None
        self._last_tracking_direction = 1.0
        self._search_direction = 1.0
        self._last_fragment_at: Optional[float] = None
        self._candidate = VisualDetection.no_result(KIND)
        self._candidate_near = False
        self._clear_line = False
        self._clear_error = 0.0
        self._far_frames = 0
        self._far_center: Optional[Tuple[int, int]] = None
        self._stable_frames = 0
        self._stable_center: Optional[Tuple[int, int]] = None
        self._last_counted_sequence: Optional[int] = None

    @property
    def active(self) -> bool:
        return self.state != MONITORING

    def _reset(self) -> None:
        self.state = MONITORING
        self.started_at = None
        self._phase_started_at = None
        self._last_fragment_at = None
        self._far_frames = 0
        self._far_center = None
        self._stable_frames = 0
        self._stable_center = None
        self._last_counted_sequence = None
        self._line_detector.reset()

    @staticmethod
    def _kernel(size: int) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def _search_candidate(self, frame: np.ndarray) -> VisualDetection:
        height, width = frame.shape[:2]
        top = max(0, min(height - 1, int(height * SEARCH_ROI_TOP)))
        bottom = max(top + 1, min(height, int(height * SEARCH_ROI_BOTTOM)))
        roi = frame[top:bottom]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array(self._vision.hsv_lower, dtype=np.uint8),
            np.array(self._vision.hsv_upper, dtype=np.uint8),
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, self._kernel(self._vision.open_kernel)
        )
        # A smaller closing kernel preserves a physical gap instead of joining
        # it into one artificial contour.
        close_size = min(int(self._vision.close_kernel), 7)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, self._kernel(close_size)
        )
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        roi_height, roi_width = mask.shape
        minimum_area = max(45.0, float(self._vision.min_area) * 0.55)
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < minimum_area:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width < 4 or box_height < 5:
                continue
            if area > roi_width * roi_height * self._vision.max_area_ratio:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            center_x = moments["m10"] / moments["m00"]
            center_y = moments["m01"] / moments["m00"]
            center_distance = abs(center_x - roi_width / 2) / max(roi_width / 2, 1)
            if center_distance > 0.78:
                continue
            bottom_ratio = (y + box_height) / max(roi_height, 1)
            vertical = box_height / max(roi_height, 1)
            score = (
                min(area / 500.0, 1.0)
                + 0.8 * (1.0 - center_distance)
                + 0.7 * bottom_ratio
                + 0.5 * min(vertical / 0.35, 1.0)
            )
            candidates.append(
                (score, x, y, box_width, box_height, center_x, center_y)
            )

        if not candidates:
            self._candidate_near = False
            return VisualDetection.no_result(KIND)

        score, x, y, box_width, box_height, center_x, center_y = max(
            candidates, key=lambda item: item[0]
        )
        bottom_ratio = (y + box_height) / max(roi_height, 1)
        vertical = box_height / max(roi_height, 1)
        self._candidate_near = bottom_ratio >= 0.72 or vertical >= 0.30
        return VisualDetection(
            valid=True,
            kind=KIND,
            center=(int(round(center_x)), int(round(center_y + top))),
            confidence=min(1.0, max(0.0, score / 3.0)),
            box=(x, y + top, x + box_width, y + top + box_height),
        )

    def detect(self, frame: np.ndarray) -> VisualDetection:
        """Inspect the shared image; a normal clear line remains line-owned."""
        line = self._line_detector.detect(frame)
        self._clear_line = bool(line.valid)
        self._clear_error = float(line.error) if line.valid else 0.0
        self._candidate = self._search_candidate(frame)
        if line.valid:
            self._candidate_near = True
            point = line.near_point or line.far_point
            self._candidate = VisualDetection(
                valid=True,
                kind=KIND,
                center=point,
                confidence=line.confidence,
            )
            return VisualDetection.no_result(KIND)
        return self._candidate

    def _remember_clear_line(self, now: float) -> None:
        self._last_clear_at = now
        if abs(self._clear_error) >= 0.04:
            self._last_tracking_direction = 1.0 if self._clear_error > 0.0 else -1.0

    @staticmethod
    def _candidate_is_continuous(
        center: Optional[Tuple[int, int]],
        previous: Optional[Tuple[int, int]],
        frame_width: int,
        maximum_jump: float,
    ) -> bool:
        if center is None or previous is None:
            return True
        return abs(center[0] - previous[0]) / max(frame_width, 1) <= maximum_jump

    def _update_far_confirmation(self, frame: FramePacket) -> None:
        if not self._candidate.valid or self._candidate_near:
            self._far_frames = 0
            self._far_center = None
            return
        if self._candidate_is_continuous(
            self._candidate.center,
            self._far_center,
            frame.image.shape[1],
            REACQUIRE_MAX_CENTER_JUMP,
        ):
            self._far_frames += 1
        else:
            self._far_frames = 1
        self._far_center = self._candidate.center

    def _begin(self, now: float) -> None:
        self.state = RAISING_VIEW
        self.started_at = now
        self._phase_started_at = now
        self._search_direction = self._last_tracking_direction
        self._stable_frames = 0
        self._stable_center = None
        self._last_counted_sequence = None
        self._line_detector.reset()

    def _gimbal_request(self) -> GimbalCommand:
        return GimbalCommand(pitch=self._search_pitch, yaw=self._search_yaw)

    def _running(
        self,
        motion: MotionCommand,
        message: str,
        detection: Optional[VisualDetection] = None,
    ) -> TaskUpdate:
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=motion,
            detection=self.last_detection if detection is None else detection,
            message=message,
            gimbal=self._gimbal_request(),
        )

    def _finish(self, status: TaskStatus, message: str) -> TaskUpdate:
        detection = self._candidate if self._candidate.valid else self.last_detection
        self._reset()
        return TaskUpdate(
            status,
            motion=MotionCommand(),
            detection=detection,
            message=message,
        )

    def _search_direction_at(self, now: float) -> float:
        elapsed = max(0.0, now - float(self._phase_started_at))
        if elapsed < INITIAL_SWEEP_SECONDS:
            return self._search_direction
        after_initial = elapsed - INITIAL_SWEEP_SECONDS
        segment = 0
        duration = SWEEP_STEP_SECONDS * 2.0
        while after_initial >= duration:
            after_initial -= duration
            segment += 1
            duration += SWEEP_STEP_SECONDS
        return (
            -self._search_direction
            if segment % 2 == 0
            else self._search_direction
        )

    def _start_reacquiring(self, now: float) -> None:
        self.state = REACQUIRING
        self._phase_started_at = now
        self._stable_frames = 0
        self._stable_center = None
        self._last_counted_sequence = None

    def _step_reacquiring(self, frame: FramePacket, now: float) -> TaskUpdate:
        if not self._candidate.valid or not self._candidate_near:
            self.state = SEARCHING
            self._phase_started_at = now
            self._stable_frames = 0
            self._stable_center = None
            return self._running(MotionCommand(), "reacquire lost; resuming search")

        if frame.sequence != self._last_counted_sequence:
            continuous = self._candidate_is_continuous(
                self._candidate.center,
                self._stable_center,
                frame.image.shape[1],
                REACQUIRE_MAX_CENTER_JUMP,
            )
            self._stable_frames = self._stable_frames + 1 if continuous else 1
            self._stable_center = self._candidate.center
            self._last_counted_sequence = frame.sequence

        if self._stable_frames >= REACQUIRE_STABLE_FRAMES:
            return self._finish(TaskStatus.COMPLETED, "route reacquired and stable")
        return self._running(
            MotionCommand(),
            f"confirming route {self._stable_frames}/{REACQUIRE_STABLE_FRAMES}",
            self._candidate,
        )

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        detection = self.detect(frame.image)
        self.last_detection = detection

        if self.state == MONITORING:
            if self._clear_line:
                self._remember_clear_line(now)
                self._update_far_confirmation(frame)
                return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
            self._update_far_confirmation(frame)
            if self._last_clear_at is None:
                return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
            trigger_delay = self._lost_grace + TRIGGER_MARGIN_SECONDS
            if now - self._last_clear_at <= trigger_delay:
                return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
            self._begin(now)
            return self._running(MotionCommand(), "long gap confirmed; raising view")

        if self.started_at is None or now - self.started_at >= TOTAL_RECOVERY_SECONDS:
            return self._finish(TaskStatus.FAILED, "route recovery timed out")

        if self.state == RAISING_VIEW:
            if now - float(self._phase_started_at) < VIEW_SETTLE_SECONDS:
                return self._running(MotionCommand(), "raising view; chassis stopped")
            if self._candidate.valid and self._candidate_near:
                self._start_reacquiring(now)
                return self._step_reacquiring(frame, now)
            if self._candidate.valid:
                self._update_far_confirmation(frame)
                if self._far_frames < FAR_CONFIRM_FRAMES:
                    return self._running(
                        MotionCommand(),
                        f"confirming far route {self._far_frames}/{FAR_CONFIRM_FRAMES}",
                        self._candidate,
                    )
                self.state = BRIDGING
                self._phase_started_at = now
                self._last_fragment_at = now
            else:
                self.state = SEARCHING
                self._phase_started_at = now

        if self.state == BRIDGING:
            if self._candidate.valid and self._candidate_near:
                self._start_reacquiring(now)
                return self._step_reacquiring(frame, now)
            if self._candidate.valid:
                self._last_fragment_at = now
                center_x = self._candidate.center[0]
                half_width = max(frame.image.shape[1] / 2.0, 1.0)
                error = (center_x - frame.image.shape[1] / 2.0) / half_width
                yaw = max(
                    -BRIDGE_MAX_YAW,
                    min(error * BRIDGE_YAW_GAIN, BRIDGE_MAX_YAW),
                )
                if now - float(self._phase_started_at) >= BRIDGE_MAX_SECONDS:
                    self.state = SEARCHING
                    self._phase_started_at = now
                    return self._running(
                        MotionCommand(), "bridge budget ended; searching"
                    )
                return self._running(
                    MotionCommand(forward=BRIDGE_FORWARD_SPEED, yaw=yaw),
                    "approaching visible route fragment",
                    self._candidate,
                )
            if (
                self._last_fragment_at is None
                or now - self._last_fragment_at >= FRAGMENT_LOSS_SECONDS
            ):
                self.state = SEARCHING
                self._phase_started_at = now
            else:
                return self._running(MotionCommand(), "route fragment briefly missing")

        if self.state == SEARCHING:
            if self._candidate.valid and self._candidate_near:
                self._start_reacquiring(now)
                return self._step_reacquiring(frame, now)
            self._update_far_confirmation(frame)
            if self._far_frames >= FAR_CONFIRM_FRAMES:
                self.state = BRIDGING
                self._phase_started_at = now
                self._last_fragment_at = now
                return self._running(MotionCommand(), "far route confirmed; aligning")
            direction = self._search_direction_at(now)
            return self._running(
                MotionCommand(yaw=direction * SEARCH_YAW_SPEED),
                "bounded alternating route search",
            )

        if self.state == REACQUIRING:
            return self._step_reacquiring(frame, now)

        return self._finish(TaskStatus.FAILED, "invalid route recovery state")
