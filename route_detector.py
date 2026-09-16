"""Vision helpers used only by the bounded long-gap recovery task.

The normal :mod:`line_detector` deliberately requires a contour to cross its
near and far sample bands.  That is useful while following a continuous line,
but it rejects the visible end of a disconnected route.  This module keeps the
same configurable HSV segmentation while describing one-ended line fragments
for recovery.  It has no camera or robot dependency.
"""

from dataclasses import dataclass
from math import atan2, degrees, hypot
from typing import List, Optional, Tuple

import cv2
import numpy as np

from models import VisualDetection


KIND = "route"
SEARCH_ROI = (0.02, 0.02, 0.98, 0.99)
BOTTOM_ROI = (0.04, 0.74, 0.96, 0.99)
MIN_FRAGMENT_ELONGATION = 2.25
MIN_FRAGMENT_MAJOR_PIXELS = 28.0
# Skeleton endpoints sit roughly half a tape width inside a filled contour.
# A wider margin therefore represents a component that actually touches the
# image boundary; 12 px incorrectly labelled boundary-clipped 20 px tape as
# a physical endpoint.
ENDPOINT_BORDER_PIXELS = 28
MIN_ENDPOINT_BRANCH_PIXELS = 24.0
ENDPOINT_FIT_INNER_PIXELS = 10.0
ENDPOINT_FIT_OUTER_PIXELS = 140.0
MIN_ENDPOINT_FIT_POINTS = 24
MAX_ENDPOINT_FIT_MEDIAN_ERROR = 6.0
# Steering at the farthest visible endpoint makes a connected corner look
# sharper and earlier than it really is.  Follow a point this far along the
# skeleton from the robot-side end instead; the far endpoint remains available
# only for deciding whether the tape physically ends.
CORNER_LOOKAHEAD_PIXELS = 65.0


@dataclass(frozen=True)
class RouteEndpoint:
    """A skeleton end and the directed tangent going into its route branch."""

    point: Tuple[int, int]
    tangent_deg: float
    internal: bool
    branch_length: float


@dataclass(frozen=True)
class RoutePathObservation:
    """Route component connected to the robot-side (bottom) image band."""

    present: bool
    endpoint: Optional[RouteEndpoint] = None
    lookahead_point: Optional[Tuple[int, int]] = None
    error: float = 0.0
    detection: Optional[VisualDetection] = None


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
    bottom_ratio: float
    upper_point: Tuple[int, int]
    lower_point: Tuple[int, int]
    elongation: float
    score: float
    endpoints: Tuple[RouteEndpoint, ...] = ()
    entry_endpoint: Optional[RouteEndpoint] = None


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

    @staticmethod
    def _thin(binary: np.ndarray, max_iterations: int = 80) -> np.ndarray:
        """Zhang-Suen thinning implemented with NumPy (no contrib dependency)."""
        image = (binary > 0).astype(np.uint8)
        for _ in range(max_iterations):
            changed = False
            for phase in (0, 1):
                padded = np.pad(image, 1)
                p2 = padded[:-2, 1:-1]
                p3 = padded[:-2, 2:]
                p4 = padded[1:-1, 2:]
                p5 = padded[2:, 2:]
                p6 = padded[2:, 1:-1]
                p7 = padded[2:, :-2]
                p8 = padded[1:-1, :-2]
                p9 = padded[:-2, :-2]
                neighbours = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
                transitions = (
                    ((p2 == 0) & (p3 == 1)).astype(np.uint8)
                    + ((p3 == 0) & (p4 == 1)).astype(np.uint8)
                    + ((p4 == 0) & (p5 == 1)).astype(np.uint8)
                    + ((p5 == 0) & (p6 == 1)).astype(np.uint8)
                    + ((p6 == 0) & (p7 == 1)).astype(np.uint8)
                    + ((p7 == 0) & (p8 == 1)).astype(np.uint8)
                    + ((p8 == 0) & (p9 == 1)).astype(np.uint8)
                    + ((p9 == 0) & (p2 == 1)).astype(np.uint8)
                )
                if phase == 0:
                    preserve_a = p2 * p4 * p6 == 0
                    preserve_b = p4 * p6 * p8 == 0
                else:
                    preserve_a = p2 * p4 * p8 == 0
                    preserve_b = p2 * p6 * p8 == 0
                remove = (
                    (image == 1)
                    & (neighbours >= 2)
                    & (neighbours <= 6)
                    & (transitions == 1)
                    & preserve_a
                    & preserve_b
                )
                if np.any(remove):
                    image[remove] = 0
                    changed = True
            if not changed:
                break
        return image

    @staticmethod
    def _endpoint_features(
        component: np.ndarray,
        offset: Tuple[int, int],
        frame_shape: Tuple[int, int],
    ) -> Tuple[RouteEndpoint, ...]:
        skeleton = RouteVision._thin(component)
        if not np.any(skeleton):
            return ()
        padded = np.pad(skeleton, 1)
        count = np.zeros_like(skeleton, dtype=np.uint8)
        for dy in range(3):
            for dx in range(3):
                if dx == 1 and dy == 1:
                    continue
                count += padded[dy:dy + skeleton.shape[0], dx:dx + skeleton.shape[1]]
        endpoint_pixels = np.argwhere((skeleton == 1) & (count == 1))
        if len(endpoint_pixels) == 0:
            return ()

        points = np.argwhere(skeleton == 1)
        left, top = offset
        frame_height, frame_width = frame_shape
        features = []
        for y, x in endpoint_pixels:
            distances = np.hypot(points[:, 1] - x, points[:, 0] - y)
            branch_length = float(np.max(distances))
            if branch_length < MIN_ENDPOINT_BRANCH_PIXELS:
                continue

            # The course guarantees a substantial straight segment after a
            # physical gap.  Estimate its direction from that segment instead
            # of the first 20-30 skeleton pixels at the endpoint: the latter
            # was dominated by jagged tape ends and produced unstable ALIGN
            # commands.  Huber fitting suppresses the remaining skeleton spurs.
            fit_band = points[
                (distances >= ENDPOINT_FIT_INNER_PIXELS)
                & (distances <= ENDPOINT_FIT_OUTER_PIXELS)
            ]
            if len(fit_band) < MIN_ENDPOINT_FIT_POINTS:
                continue
            fit_xy = np.column_stack(
                (fit_band[:, 1], fit_band[:, 0])
            ).astype(np.float32)
            vx, vy, x0, y0 = cv2.fitLine(
                fit_xy, cv2.DIST_HUBER, 0, 0.01, 0.01
            ).reshape(-1)
            vx, vy = float(vx), float(vy)
            x0, y0 = float(x0), float(y0)
            residuals = np.abs(
                (fit_xy[:, 0] - x0) * vy
                - (fit_xy[:, 1] - y0) * vx
            )
            if float(np.median(residuals)) > MAX_ENDPOINT_FIT_MEDIAN_ERROR:
                continue

            # Direct the fitted axis from the physical endpoint into the new
            # route.  This keeps left/right 90-degree approaches distinct and
            # lets ALIGN rotate the chassis along the route before centering.
            mean_dx = float(np.mean(fit_xy[:, 0]) - x)
            mean_dy = float(np.mean(fit_xy[:, 1]) - y)
            if vx * mean_dx + vy * mean_dy < 0.0:
                vx, vy = -vx, -vy
            angle = degrees(atan2(vx, -vy))
            while angle > 180.0:
                angle -= 360.0
            while angle <= -180.0:
                angle += 360.0
            full_x = int(x + left)
            full_y = int(y + top)
            internal = (
                ENDPOINT_BORDER_PIXELS <= full_x < frame_width - ENDPOINT_BORDER_PIXELS
                and ENDPOINT_BORDER_PIXELS <= full_y < frame_height - ENDPOINT_BORDER_PIXELS
            )
            features.append(
                RouteEndpoint(
                    point=(full_x, full_y),
                    tangent_deg=float(angle),
                    internal=internal,
                    branch_length=branch_length,
                )
            )
        return tuple(features)

    def connected_path(
        self,
        frame: np.ndarray,
        expected_x: Optional[float] = None,
    ) -> RoutePathObservation:
        """Describe the component connected to the bottom of the current view.

        A far endpoint at an image boundary means the visible route continues
        out of view (for example through a connected right-angle corner).  An
        internal far endpoint is evidence of a physical tape end.
        """
        height, width = frame.shape[:2]
        rect = self._roi_rect(frame, SEARCH_ROI)
        left, top, _, _ = rect
        mask = self._mask(frame, rect)
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
        bottom_start = int(mask.shape[0] * 0.72)
        choices = []
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < max(35, int(float(self.settings.min_area) * 0.35)):
                continue
            ys, xs = np.where(labels == label)
            bottom = ys >= bottom_start
            if not np.any(bottom):
                continue
            anchor_x = float(np.median(xs[bottom]) + left)
            if expected_x is not None and abs(anchor_x - expected_x) / max(width, 1) > 0.34:
                continue
            continuity = 0.0 if expected_x is None else 1.0 - min(abs(anchor_x - expected_x) / (0.34 * width), 1.0)
            choices.append((area + 300.0 * continuity, label, anchor_x))
        if not choices:
            return RoutePathObservation(False)

        _, label, anchor_x = max(choices, key=lambda item: item[0])
        local_x = int(stats[label, cv2.CC_STAT_LEFT])
        local_y = int(stats[label, cv2.CC_STAT_TOP])
        local_width = int(stats[label, cv2.CC_STAT_WIDTH])
        local_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        component = np.where(
            labels[
                local_y:local_y + local_height,
                local_x:local_x + local_width,
            ] == label,
            255,
            0,
        ).astype(np.uint8)
        endpoints = self._endpoint_features(
            component,
            (left + local_x, top + local_y),
            (height, width),
        )
        if not endpoints:
            return RoutePathObservation(False)
        # The robot-side end has the greatest y.  The opposite end describes
        # whether the route genuinely terminates or merely leaves the image.
        robot_end = max(endpoints, key=lambda item: item.point[1])
        far_end = max(
            endpoints,
            key=lambda item: hypot(
                item.point[0] - robot_end.point[0],
                item.point[1] - robot_end.point[1],
            ),
        )
        skeleton = self._thin(component)
        skeleton_y, skeleton_x = np.nonzero(skeleton)
        if skeleton_x.size:
            full_x = skeleton_x + left + local_x
            full_y = skeleton_y + top + local_y
            distances = np.hypot(
                full_x - robot_end.point[0],
                full_y - robot_end.point[1],
            )
            lookahead_index = int(
                np.argmin(np.abs(distances - CORNER_LOOKAHEAD_PIXELS))
            )
            lookahead = (
                int(full_x[lookahead_index]),
                int(full_y[lookahead_index]),
            )
        else:
            lookahead = far_end.point
        x, y, box_width, box_height = (
            local_x + left,
            local_y + top,
            local_width,
            local_height,
        )
        detection = VisualDetection(
            valid=True,
            kind=KIND,
            center=far_end.point,
            confidence=1.0,
            box=(x, y, x + box_width, y + box_height),
        )
        error = (lookahead[0] - width / 2.0) / max(width / 2.0, 1.0)
        return RoutePathObservation(
            True,
            endpoint=far_end,
            lookahead_point=lookahead,
            error=float(error),
            detection=detection,
        )

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
            contour_x, contour_y, contour_width, contour_height = cv2.boundingRect(contour)
            component = np.zeros((contour_height, contour_width), dtype=np.uint8)
            shifted = contour.copy()
            shifted[:, :, 0] -= contour_x
            shifted[:, :, 1] -= contour_y
            cv2.drawContours(component, [shifted], -1, 255, thickness=-1)
            endpoints = self._endpoint_features(
                component,
                (left + contour_x, top + contour_y),
                (height, width),
            )
            if len(endpoints) < 2:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            center_x = float(moments["m10"] / moments["m00"] + left)
            center_y = float(moments["m01"] / moments["m00"] + top)
            x, y, box_width, box_height = cv2.boundingRect(contour)
            upper = (upper_local[0] + left, upper_local[1] + top)
            lower = (lower_local[0] + left, lower_local[1] + top)
            # The gap-facing end is normally the endpoint nearest the robot.
            # RouteTask applies the saved old-tangent and gap-pairing gates
            # before it is ever allowed to command motion toward this point.
            entry_endpoint = max(
                endpoints,
                key=lambda item: (
                    item.point[1] / max(height, 1)
                    - 0.35
                    * abs(item.point[0] - width / 2.0)
                    / max(width / 2.0, 1.0)
                ),
            )
            lower = entry_endpoint.point
            angle = entry_endpoint.tangent_deg
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
                    bottom_ratio=float(bottom_ratio),
                    upper_point=upper,
                    lower_point=lower,
                    elongation=float(elongation),
                    score=float(score),
                    endpoints=endpoints,
                    entry_endpoint=entry_endpoint,
                )
            )

        found.sort(key=lambda item: item.score, reverse=True)
        return found
