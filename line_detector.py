"""Configurable HSV line detector; never accesses robot hardware."""

from typing import Optional, Tuple

import cv2
import numpy as np

from config import VisionConfig
from models import LineDetection


class LineDetector:
    def __init__(self, settings: VisionConfig) -> None:
        self.settings = settings
        self._last_center: Optional[Tuple[float, float]] = None

    def reset(self) -> None:
        self._last_center = None

    @staticmethod
    def empty_detection() -> LineDetection:
        return LineDetection(
            False, 0.0, 0.0, 0.0, None, None,
            (0, 0, 0, 0), np.zeros((1, 1), dtype=np.uint8),
        )

    @staticmethod
    def _kernel(size: int) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def _score(
        self, contour: np.ndarray, width: int, height: int
    ) -> Optional[Tuple[float, Tuple[float, float]]]:
        area = cv2.contourArea(contour)
        _, _, _, box_height = cv2.boundingRect(contour)
        frame_area = max(width * height, 1)
        coverage = box_height / max(height, 1)
        if (
            area < self.settings.min_area
            or area > frame_area * self.settings.max_area_ratio
            or coverage < self.settings.min_vertical_coverage
        ):
            return None
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            return None
        center = (
            moments["m10"] / moments["m00"],
            moments["m01"] / moments["m00"],
        )
        center_distance = abs(center[0] - width / 2) / max(width / 2, 1)
        continuity = 0.0
        if self._last_center is not None:
            distance = np.hypot(
                center[0] - self._last_center[0],
                center[1] - self._last_center[1],
            ) / max(width, 1)
            if distance > self.settings.max_continuity_distance:
                return None
            continuity = 1.0 - min(
                distance / self.settings.max_continuity_distance, 1.0
            )
        score = (
            self.settings.continuity_weight * continuity
            + self.settings.center_weight * (1.0 - min(center_distance, 1.0))
            + self.settings.vertical_weight * coverage
            + self.settings.area_weight * min((area / frame_area) / 0.10, 1.0)
        )
        return score, center

    @staticmethod
    def _sample(
        mask: np.ndarray,
        band: Tuple[float, float],
        minimum_pixels: int,
    ) -> Optional[Tuple[float, float]]:
        height = mask.shape[0]
        y0 = max(0, min(height - 1, int(height * band[0])))
        y1 = max(y0 + 1, min(height, int(height * band[1])))
        ys, xs = np.nonzero(mask[y0:y1])
        if len(xs) < minimum_pixels:
            return None
        return float(np.median(xs)), float(np.median(ys) + y0)

    def detect(self, frame: np.ndarray) -> LineDetection:
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must be a non-empty BGR image")
        height, width = frame.shape[:2]
        left = int(width * self.settings.roi_left)
        right = int(width * self.settings.roi_right)
        top = int(height * self.settings.roi_top)
        bottom = int(height * self.settings.roi_bottom)
        roi_rect = (left, top, right, bottom)
        roi = frame[top:bottom, left:right]
        if roi.size == 0:
            raise ValueError("configured ROI is empty")

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array(self.settings.hsv_lower, dtype=np.uint8),
            np.array(self.settings.hsv_upper, dtype=np.uint8),
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, self._kernel(self.settings.open_kernel)
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, self._kernel(self.settings.close_kernel)
        )
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        candidates = []
        for contour in contours:
            scored = self._score(contour, roi.shape[1], roi.shape[0])
            if scored is not None:
                candidates.append((scored[0], scored[1], contour))
        if not candidates:
            return LineDetection(
                False, 0.0, 0.0, 0.0, None, None, roi_rect, mask
            )

        score, center, contour = max(candidates, key=lambda item: item[0])
        selected = np.zeros_like(mask)
        cv2.drawContours(selected, [contour], -1, 255, cv2.FILLED)
        near = self._sample(
            selected, self.settings.near_band, self.settings.min_sample_pixels
        )
        far = self._sample(
            selected, self.settings.far_band, self.settings.min_sample_pixels
        )
        if near is None or far is None:
            return LineDetection(
                False, 0.0, 0.0, 0.0, None, None, roi_rect, mask
            )

        half_width = max(width / 2.0, 1.0)
        near_error = (near[0] + left - width / 2.0) / half_width
        far_error = (far[0] + left - width / 2.0) / half_width
        error = (
            self.settings.near_weight * near_error
            + (1.0 - self.settings.near_weight) * far_error
        )
        heading = far_error - near_error
        self._last_center = center
        confidence = min(1.0, max(0.0, score / 4.0))
        contour_frame = contour.copy()
        contour_frame[:, :, 0] += left
        contour_frame[:, :, 1] += top
        near_point = (
            int(round(near[0] + left)), int(round(near[1] + top))
        )
        far_point = (
            int(round(far[0] + left)), int(round(far[1] + top))
        )
        return LineDetection(
            True,
            float(error),
            float(heading),
            confidence,
            near_point,
            far_point,
            roi_rect,
            mask,
            contour_frame,
        )
