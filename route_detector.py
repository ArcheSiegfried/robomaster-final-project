"""Vision helpers used only by the bounded long-gap recovery task.

The normal :mod:`line_detector` deliberately requires a contour to cross its
near and far sample bands.  That is useful while following a continuous line,
but it rejects the visible end of a disconnected route.  This module keeps the
same configurable HSV segmentation while describing one-ended line fragments
for recovery.  It has no camera or robot dependency.
"""

from dataclasses import dataclass
from math import atan2, degrees
from typing import List, Optional, Tuple

import cv2
import numpy as np

from models import VisualDetection


KIND = "route"
SEARCH_ROI = (0.02, 0.02, 0.98, 0.99)
BOTTOM_ROI = (0.04, 0.74, 0.96, 0.99)
MIN_FRAGMENT_ELONGATION = 2.25
MIN_FRAGMENT_MAJOR_PIXELS = 28.0


@dataclass(frozen=True)
class BottomLineObservation:
    present: bool
    error: float = 0.0
    angle_deg: float = 0.0
    detection: Optional[VisualDetection] = None


@dataclass(frozen=True)
class RouteCandidate:
    detection: VisualDetection
    angle_deg: float
    near: bool
    upper_point: Tuple[int, int]
    lower_point: Tuple[int, int]
    elongation: float
    score: float


class RouteVision:
    """Extract route-end observations and elongated search candidates."""

    def __init__(self, settings) -> None:
        self.settings = settings

    @staticmethod
    def _kernel(size: int) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    @staticmethod
    def _roi_rect(frame: np.ndarray, fractions) -> Tuple[int, int, int, int]:
        height, width = frame.shape[:2]
        left = max(0, min(width - 1, int(width * fractions[0])))
        top = max(0, min(height - 1, int(height * fractions[1])))
        right = max(left + 1, min(width, int(width * fractions[2])))
        bottom = max(top + 1, min(height, int(height * fractions[3])))
        return left, top, right, bottom

    def _mask(self, frame: np.ndarray, rect) -> np.ndarray:
        left, top, right, bottom = rect
        roi = frame[top:bottom, left:right]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array(self.settings.hsv_lower, dtype=np.uint8),
            np.array(self.settings.hsv_upper, dtype=np.uint8),
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self._kernel(self.settings.open_kernel),
        )
        # A large close kernel can join the two physical sides of a gap.
        close_size = min(int(self.settings.close_kernel), 7)
        return cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            self._kernel(close_size),
        )

    @staticmethod
    def _line_geometry(contour: np.ndarray):
        points = contour.reshape(-1, 2).astype(np.float32)
        if len(points) < 2:
            return None
        vx, vy, _, _ = cv2.fitLine(
            points, cv2.DIST_L2, 0, 0.01, 0.01
        ).reshape(-1)
        vx, vy = float(vx), float(vy)
        # The route tangent is undirected.  Prefer the direction that points
        # towards the top of the image, i.e. away from the robot.
        if vy > 0.0 or (abs(vy) < 1e-6 and vx < 0.0):
            vx, vy = -vx, -vy
        angle = degrees(atan2(vx, max(-vy, 1e-6)))
        angle = max(-90.0, min(angle, 90.0))

        projection = points[:, 0] * vx + points[:, 1] * vy
        first = points[int(np.argmin(projection))]
        second = points[int(np.argmax(projection))]
        upper, lower = (first, second) if first[1] <= second[1] else (second, first)
        return (
            angle,
            (int(round(upper[0])), int(round(upper[1]))),
            (int(round(lower[0])), int(round(lower[1]))),
        )

    @staticmethod
    def _elongation(contour: np.ndarray) -> Tuple[float, float, float]:
        (_, _), (side_a, side_b), _ = cv2.minAreaRect(contour)
        major = max(float(side_a), float(side_b))
        minor = max(min(float(side_a), float(side_b)), 1.0)
        return major / minor, major, minor

    def bottom_line(
        self,
        frame: np.ndarray,
        expected_x: Optional[float] = None,
    ) -> BottomLineObservation:
        """Find the old route still passing through the bottom image band."""
        height, width = frame.shape[:2]
        rect = self._roi_rect(frame, BOTTOM_ROI)
        left, top, _, _ = rect
        mask = self._mask(frame, rect)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        minimum_area = max(18.0, float(self.settings.min_area) * 0.20)
        choices = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < minimum_area:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            center_x = float(moments["m10"] / moments["m00"] + left)
            center_y = float(moments["m01"] / moments["m00"] + top)
            continuity = 0.0
            if expected_x is not None:
                distance = abs(center_x - expected_x) / max(width, 1)
                if distance > 0.30:
                    continue
                continuity = 1.0 - distance / 0.30
            choices.append((area + 200.0 * continuity, contour, center_x, center_y))

        if not choices:
            return BottomLineObservation(False)

        _, contour, center_x, center_y = max(choices, key=lambda item: item[0])
        geometry = self._line_geometry(contour)
        angle = 0.0 if geometry is None else geometry[0]
        x, y, box_width, box_height = cv2.boundingRect(contour)
        detection = VisualDetection(
            valid=True,
            kind=KIND,
            center=(int(round(center_x)), int(round(center_y))),
            confidence=1.0,
            box=(left + x, top + y, left + x + box_width, top + y + box_height),
        )
        error = (center_x - width / 2.0) / max(width / 2.0, 1.0)
        return BottomLineObservation(True, float(error), float(angle), detection)

    def candidates(self, frame: np.ndarray) -> List[RouteCandidate]:
        """Return all plausible one-ended route fragments, best first."""
        height, width = frame.shape[:2]
        rect = self._roi_rect(frame, SEARCH_ROI)
        left, top, right, bottom = rect
        mask = self._mask(frame, rect)
        roi_height, roi_width = mask.shape
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )
        minimum_area = max(35.0, float(self.settings.min_area) * 0.40)
        frame_area = max(roi_height * roi_width, 1)
        found: List[RouteCandidate] = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < minimum_area:
                continue
            if area > frame_area * float(self.settings.max_area_ratio):
                continue
            elongation, major, minor = self._elongation(contour)
            if (
                elongation < MIN_FRAGMENT_ELONGATION
                or major < MIN_FRAGMENT_MAJOR_PIXELS
                or minor > min(height, width) * 0.18
            ):
                continue
            geometry = self._line_geometry(contour)
            if geometry is None:
                continue
            angle, upper_local, lower_local = geometry
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            center_x = float(moments["m10"] / moments["m00"] + left)
            center_y = float(moments["m01"] / moments["m00"] + top)
            x, y, box_width, box_height = cv2.boundingRect(contour)
            upper = (upper_local[0] + left, upper_local[1] + top)
            lower = (lower_local[0] + left, lower_local[1] + top)
            bottom_ratio = lower[1] / max(height, 1)
            near = bottom_ratio >= 0.76
            center_distance = abs(center_x - width / 2.0) / max(width / 2.0, 1.0)
            length_score = min(major / max(height * 0.35, 1.0), 1.0)
            shape_score = min((elongation - 1.0) / 5.0, 1.0)
            score = (
                0.34 * length_score
                + 0.30 * shape_score
                + 0.20 * (1.0 - min(center_distance, 1.0))
                + 0.16 * min(bottom_ratio, 1.0)
            )
            detection = VisualDetection(
                valid=True,
                kind=KIND,
                center=(int(round(center_x)), int(round(center_y))),
                confidence=float(max(0.0, min(score, 1.0))),
                box=(left + x, top + y, left + x + box_width, top + y + box_height),
            )
            found.append(
                RouteCandidate(
                    detection=detection,
                    angle_deg=float(angle),
                    near=near,
                    upper_point=upper,
                    lower_point=lower,
                    elongation=float(elongation),
                    score=float(score),
                )
            )

        found.sort(key=lambda item: item.score, reverse=True)
        return found
