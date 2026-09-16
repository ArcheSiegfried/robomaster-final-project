"""Bounded long-gap recovery task (WP6a / Issue #6).

Brief image misses remain the base follower's responsibility.  This task also
recognises the intermediate case where the far sample has vanished but the old
route is still under the bottom of the camera: it crawls to the physical end
before raising the shared view and crossing a bounded blank area.

The module consumes only ``FramePacket`` and returns typed motion/gimbal
requests.  It never connects hardware and every call is non-blocking.
"""

from dataclasses import replace
from math import atan2, cos, degrees, hypot, radians, sin
from typing import List, Optional, Tuple

from config import CONFIG
from line_detector import LineDetector
from models import (
    FramePacket,
    GimbalCommand,
    MotionCommand,
    STOP_COMMAND,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)
from route_detector import (
    BottomLineObservation,
    RouteCandidate,
    RoutePathObservation,
    RouteVision,
)


KIND = "route"
MONITORING = "monitoring"
END_APPROACH = "end_approach"
CORNERING = "cornering"
RAISING_VIEW = "raising_view"
BRIDGING = "bridging"
SEARCHING = "searching"
ALIGNING = "aligning"
CENTERING = "centering"
DOCKING = "docking"
REACQUIRING = "reacquiring"

# Initial values only.  None has been validated on the real course.
TRIGGER_MARGIN_SECONDS = 0.05
END_PENDING_SECONDS = 0.18
END_MISSING_SECONDS = 0.14
CORNER_PATH_LOSS_SECONDS = 0.40
# At 0.08 m/s the previous 2.5 s budget covered only about 0.20 m.  On the
# real camera the far sample can disappear earlier than that, so the task was
# failing at the physical endpoint before it could raise the view.  Keep this
# phase bounded, but allow roughly 0.40 m of bottom-line following.
END_APPROACH_MAX_SECONDS = 5.0
END_APPROACH_SPEED = 0.10
END_APPROACH_YAW_GAIN = 32.0
END_APPROACH_MAX_YAW = 16.0

VIEW_SETTLE_MARGIN_SECONDS = 0.12
TOTAL_RECOVERY_SECONDS = 19.0
BRIDGE_MAX_SECONDS = 3.0
# Real-car feedback showed that the previous 0.12 m/s request could fail to
# overcome the stopped chassis' static friction.  Keep this well inside the
# external-task envelope, but give the bounded crossing a usable start speed.
BRIDGE_MIN_SECONDS = 0.80
BRIDGE_FORWARD_SPEED = 0.15
FRAGMENT_LOSS_SECONDS = 0.35

SEARCH_YAW_SPEED = 45.0
SEARCH_CONFIRM_YAW_SPEED = 30.0
SEARCH_SOFT_LIMIT_DEG = 95.0
SEARCH_HARD_LIMIT_DEG = 100.0
SEARCH_TARGETS_DEG = (30.0, 60.0, 95.0, -30.0, -60.0, -95.0)
SEARCH_TARGET_TOLERANCE_DEG = 2.5

CANDIDATE_CONFIRM_FRAMES = 3
REACQUIRE_STABLE_FRAMES = 3
REACQUIRE_MAX_CENTER_JUMP = 0.14
REACQUIRE_MAX_ANGLE_JUMP = 42.0
NEAR_ENTER_RATIO = 0.78
NEAR_EXIT_RATIO = 0.70
OLD_LINE_CLEAR_DISTANCE = 0.07
MIN_ENDPOINT_GAP_DISTANCE = 0.08
MAX_ENDPOINT_GAP_DISTANCE = 1.40
MIN_LOCK_BRANCH_PIXELS = 100.0
MIN_LOCK_BOTTOM_RATIO = 0.80
ALIGN_ANGLE_DEG = 10.0
CENTER_REALIGN_ANGLE_DEG = 18.0
ALIGN_YAW_GAIN = 0.38
ALIGN_MIN_YAW = 5.0
ALIGN_MAX_YAW = 18.0
ALIGN_VISIBILITY_GAIN = 0.12
ALIGN_MAX_LATERAL = 0.07
DOCK_FORWARD_SPEED = 0.08
DOCK_TARGET_ERROR = 0.07
DOCK_ANGLE_DEG = 25.0
DOCK_MAX_YAW = 22.0
CORNER_FORWARD_SPEED = 0.055
CORNER_MAX_YAW = 38.0
CENTER_TARGET_ERROR = 0.04
CENTER_LATERAL_GAIN = 0.16
CENTER_MAX_LATERAL = 0.12
DOCK_LATERAL_GAIN = 0.12
DOCK_MAX_LATERAL = 0.09
RIGHT_ANGLE_MIN_DEG = 50.0
RIGHT_ANGLE_MAX_DEG = 130.0
MIN_CANDIDATE_ENDPOINT_Y_RATIO = 0.25
HANDOFF_VIEW_TIMEOUT_SECONDS = 2.0
MAX_INTEGRATION_STEP_SECONDS = 0.20
LOCKED_TARGET_LOSS_SECONDS = 0.80


class RouteTask:
    """Find and join a disconnected route with bounded camera-only motion."""

    name = "route"

    def __init__(self, settings: Optional[object] = None) -> None:
        self.settings = CONFIG if settings is None else settings
        self._vision_settings = getattr(self.settings, "vision", CONFIG.vision)
        control = getattr(self.settings, "control", CONFIG.control)
        self._line_detector = LineDetector(self._vision_settings)
        self._route_vision = RouteVision(self._vision_settings)
        self._lost_grace = float(control.lost_grace_seconds)
        self._line_pitch = float(
            getattr(self.settings, "gimbal_pitch", CONFIG.gimbal_pitch)
        )
        self._search_pitch = float(
            getattr(self.settings, "gimbal_search_pitch", CONFIG.gimbal_search_pitch)
        )
        self._search_yaw = float(
            getattr(self.settings, "gimbal_yaw", CONFIG.gimbal_yaw)
        )
        pitch_speed = max(
            float(getattr(self.settings, "gimbal_pitch_speed", 30.0)), 1.0
        )
        configured_settle = float(
            getattr(self.settings, "gimbal_settle_seconds", 0.45)
        )
        physical_travel = abs(self._search_pitch - self._line_pitch) / pitch_speed
        self._view_settle_seconds = max(
            configured_settle, physical_travel + VIEW_SETTLE_MARGIN_SECONDS
        )

        self.last_detection = VisualDetection.no_result(KIND)
        self.state = MONITORING
        self.started_at: Optional[float] = None
        self._phase_started_at: Optional[float] = None
        self._last_clear_at: Optional[float] = None
        self._possible_end_at: Optional[float] = None
        self._bottom_missing_at: Optional[float] = None
        self._last_line_x: Optional[float] = None
        self._last_line_tangent: Optional[float] = None
        self._last_tracking_direction = 1.0
        self._search_direction = 1.0

        self._bottom = BottomLineObservation(False)
        self._path = RoutePathObservation(False)
        self._candidate: Optional[RouteCandidate] = None
        self._candidate_frames = 0
        self._candidate_seen_far = False
        self._candidate_near = False
        self._candidate_center: Optional[Tuple[int, int]] = None
        self._candidate_angle: Optional[float] = None
        self._candidate_last_at: Optional[float] = None
        self._candidate_last_sequence: Optional[int] = None
        self._stable_last_sequence: Optional[int] = None
        self._stable_frames = 0

        # Conservative command integration provides a bounded fallback when
        # the integration layer has not yet supplied chassis odometry.
        self._pose_forward = 0.0
        self._pose_lateral = 0.0
        self._heading_offset = 0.0
        self._last_motion_at: Optional[float] = None
        self._last_motion = STOP_COMMAND
        self._search_targets: List[float] = []
        self._search_target_index = 0
        self._old_tangent_world: Optional[float] = None
        self._corner_missing_at: Optional[float] = None

    @property
    def active(self) -> bool:
        return self.state != MONITORING

    @property
    def estimated_forward_progress(self) -> float:
        """Command-integrated departure progress, for diagnostics/tests."""
        return self._pose_forward

    @property
    def estimated_heading_offset(self) -> float:
        return self._heading_offset

    @property
    def candidate_near(self) -> bool:
        """Hysteresis-filtered near/far classification for diagnostics."""
        return self._candidate_near

    def reset(self) -> None:
        """Cancel all recovery history after a human/video/fault stop."""
        self.state = MONITORING
        self.started_at = None
        self._phase_started_at = None
        self._last_clear_at = None
        self._possible_end_at = None
        self._bottom_missing_at = None
        self._last_line_x = None
        self._last_line_tangent = None
        self._bottom = BottomLineObservation(False)
        self._path = RoutePathObservation(False)
        self._candidate = None
        self._candidate_frames = 0
        self._candidate_seen_far = False
        self._candidate_near = False
        self._candidate_center = None
        self._candidate_angle = None
        self._candidate_last_at = None
        self._candidate_last_sequence = None
        self._stable_last_sequence = None
        self._stable_frames = 0
        self._pose_forward = 0.0
        self._pose_lateral = 0.0
        self._heading_offset = 0.0
        self._last_motion_at = None
        self._last_motion = STOP_COMMAND
        self._search_targets = []
        self._search_target_index = 0
        self._old_tangent_world = None
        self._corner_missing_at = None
        self.last_detection = VisualDetection.no_result(KIND)
        self._line_detector.reset()

    def _integrate_previous_command(self, now: float) -> None:
        if self._last_motion_at is None:
            self._last_motion_at = now
            return
        elapsed = max(
            0.0,
            min(now - self._last_motion_at, MAX_INTEGRATION_STEP_SECONDS),
        )
        heading = radians(self._heading_offset)
        forward = self._last_motion.forward * elapsed
        lateral = self._last_motion.lateral * elapsed
        self._pose_forward += forward * cos(heading) - lateral * sin(heading)
        self._pose_lateral += forward * sin(heading) + lateral * cos(heading)
        self._heading_offset += self._last_motion.yaw * elapsed
        self._heading_offset = max(
            -SEARCH_HARD_LIMIT_DEG,
            min(self._heading_offset, SEARCH_HARD_LIMIT_DEG),
        )
        self._last_motion_at = now

    def _record_motion(self, motion: MotionCommand, now: float) -> None:
        self._last_motion = motion
        self._last_motion_at = now

    @staticmethod
    def _normal_update(detection=None) -> TaskUpdate:
        return TaskUpdate(
            TaskStatus.NOT_TRIGGERED,
            detection=VisualDetection.no_result(KIND)
            if detection is None
            else detection,
        )

    def _gimbal_request(self) -> GimbalCommand:
        return GimbalCommand(pitch=self._search_pitch, yaw=self._search_yaw)

    def _running(
        self,
        now: float,
        motion: MotionCommand,
        message: str,
        detection: Optional[VisualDetection] = None,
        search_view: bool = True,
    ) -> TaskUpdate:
        self._record_motion(motion, now)
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=motion,
            detection=self.last_detection if detection is None else detection,
            message=message,
            gimbal=self._gimbal_request() if search_view else None,
        )

    def _finish(self, status: TaskStatus, message: str) -> TaskUpdate:
        detection = (
            self._candidate.detection
            if self._candidate is not None
            else self.last_detection
        )
        self.reset()
        return TaskUpdate(
            status,
            motion=STOP_COMMAND,
            detection=detection,
            message=message,
        )

    def _remember_clear_line(self, line, now: float) -> None:
        self._last_clear_at = now
        self._possible_end_at = None
        if line.near_point is not None:
            self._last_line_x = float(line.near_point[0])
        if line.near_point is not None and line.far_point is not None:
            dx = float(line.far_point[0] - line.near_point[0])
            dy = float(line.far_point[1] - line.near_point[1])
            self._last_line_tangent = degrees(atan2(dx, max(-dy, 1e-6)))
        if abs(line.error) >= 0.04:
            self._last_tracking_direction = 1.0 if line.error > 0.0 else -1.0

    @staticmethod
    def _center_continuous(
        current: Tuple[int, int],
        previous: Optional[Tuple[int, int]],
        frame_width: int,
    ) -> bool:
        if previous is None:
            return True
        return (
            hypot(current[0] - previous[0], current[1] - previous[1])
            / max(frame_width, 1)
            <= REACQUIRE_MAX_CENTER_JUMP
        )

    @staticmethod
    def _angle_difference(first: float, second: float) -> float:
        """Smallest difference between two undirected line angles."""
        difference = abs(float(first) - float(second)) % 180.0
        return min(difference, 180.0 - difference)

    @staticmethod
    def _directed_angle_difference(first: float, second: float) -> float:
        """Smallest difference between two directed endpoint tangents."""
        return abs((float(first) - float(second) + 180.0) % 360.0 - 180.0)

    def _select_candidate(
        self,
        candidates: List[RouteCandidate],
        frame_width: int,
        frame_height: int,
    ) -> Optional[RouteCandidate]:
        candidates = self._candidate_variants(
            candidates, frame_width, frame_height
        )
        candidates = [
            item for item in candidates if self._candidate_geometry_ok(item)
        ]
        if not candidates:
            return None
        if self._candidate_center is None:
            return max(
                candidates,
                key=lambda item: (
                    item.bottom_ratio,
                    item.entry_endpoint.branch_length,
                    item.score,
                ),
            )

        ranked = []
        for candidate in candidates:
            endpoint = candidate.entry_endpoint
            if endpoint is None:
                continue
            center = endpoint.point
            distance = hypot(
                center[0] - self._candidate_center[0],
                center[1] - self._candidate_center[1],
            ) / max(frame_width, 1)
            angle_jump = (
                0.0
                if self._candidate_angle is None
                else self._directed_angle_difference(
                    candidate.angle_deg, self._candidate_angle
                )
            )
            # Do not jump to an unrelated blue fragment merely because its
            # independent score is high.  A genuinely new candidate may be
            # selected after FRAGMENT_LOSS_SECONDS clears this history.
            if (
                distance > REACQUIRE_MAX_CENTER_JUMP
                or angle_jump > REACQUIRE_MAX_ANGLE_JUMP
            ):
                continue
            continuity = max(0.0, 1.0 - distance / 0.35)
            angle_score = max(0.0, 1.0 - angle_jump / 70.0)
            ranked.append(
                (candidate.score + 0.30 * continuity + 0.15 * angle_score, candidate)
            )
        return max(ranked, key=lambda item: item[0])[1] if ranked else None

    @staticmethod
    def _candidate_variants(
        candidates: List[RouteCandidate],
        frame_width: int,
        frame_height: int,
    ) -> List[RouteCandidate]:
        """Expand both skeleton ends so tracking can hold one physical end.

        A horizontal contour has two opposite directed tangents.  Choosing its
        closest endpoint independently on every frame made the selected end
        alternate left/right and flipped the yaw command between +90/-90.
        """
        variants: List[RouteCandidate] = []
        for candidate in candidates:
            endpoints = candidate.endpoints or (
                (candidate.entry_endpoint,)
                if candidate.entry_endpoint is not None
                else ()
            )
            for endpoint in endpoints:
                if endpoint is None:
                    continue
                bottom_ratio = endpoint.point[1] / max(frame_height, 1)
                center_score = 1.0 - min(
                    abs(endpoint.point[0] - frame_width / 2.0)
                    / max(frame_width / 2.0, 1.0),
                    1.0,
                )
                variants.append(
                    replace(
                        candidate,
                        angle_deg=endpoint.tangent_deg,
                        near=bottom_ratio >= 0.76,
                        bottom_ratio=float(bottom_ratio),
                        lower_point=endpoint.point,
                        entry_endpoint=endpoint,
                        # Use the origin and direction of the same fit.
                        line_point=endpoint.line_point,
                        score=(
                            candidate.score
                            + 0.08 * center_score
                            + 0.12 * min(bottom_ratio, 1.0)
                        ),
                    )
                )
        return variants

    def _candidate_geometry_ok(self, candidate: RouteCandidate) -> bool:
        """Cheap gate applied before temporal tracking can lock onto old tape."""
        endpoint = candidate.entry_endpoint
        if endpoint is None:
            return False
        # Tiny remote pieces must not enter temporal tracking at all.  If they
        # are merely rejected later by _candidate_ready(), their history keeps
        # the real, closer endpoint from being selected.
        if endpoint.branch_length < MIN_LOCK_BRANCH_PIXELS:
            return False
        # A new target must first present a physical internal endpoint.  Once
        # that same target has been tracked, its endpoint may legitimately
        # leave through the bottom edge while the chassis docks onto it.
        if not endpoint.internal and self._candidate_center is None:
            return False
        # Image slope is not ground-plane yaw. After acquisition, use image
        # continuity rather than adding it to command-integrated chassis yaw.
        if self.state in (ALIGNING, CENTERING, DOCKING):
            return True
        if self._old_tangent_world is None:
            return True
        world_tangent = self._heading_offset + endpoint.tangent_deg
        turn = self._angle_difference(world_tangent, self._old_tangent_world)
        return RIGHT_ANGLE_MIN_DEG <= turn <= RIGHT_ANGLE_MAX_DEG

    def _observe_candidate(self, frame: FramePacket, now: float) -> None:
        candidates = self._route_vision.candidates(frame.image)
        selected = self._select_candidate(
            candidates,
            frame.image.shape[1],
            frame.image.shape[0],
        )
        self._candidate = selected
        if selected is None or selected.detection.center is None:
            target_locked = self.state in (ALIGNING, CENTERING, DOCKING)
            self._stable_frames = 0
            self._stable_last_sequence = None
            if not target_locked and (
                self._candidate_last_at is None
                or now - self._candidate_last_at >= FRAGMENT_LOSS_SECONDS
            ):
                self._candidate_frames = 0
                self._candidate_seen_far = False
                self._candidate_near = False
                self._candidate_center = None
                self._candidate_angle = None
                self._candidate_last_sequence = None
            self.last_detection = VisualDetection.no_result(KIND)
            return

        self.last_detection = selected.detection
        center = selected.entry_endpoint.point
        continuous = self._center_continuous(
            center, self._candidate_center, frame.image.shape[1]
        ) and (
            self._candidate_angle is None
            or self._directed_angle_difference(
                selected.angle_deg, self._candidate_angle
            )
            <= REACQUIRE_MAX_ANGLE_JUMP
        )
        if self._candidate_near:
            self._candidate_near = selected.bottom_ratio > NEAR_EXIT_RATIO
        else:
            self._candidate_near = selected.bottom_ratio >= NEAR_ENTER_RATIO
        if frame.sequence != self._candidate_last_sequence:
            self._candidate_frames = self._candidate_frames + 1 if continuous else 1
            self._candidate_seen_far = (
                self._candidate_seen_far or not self._candidate_near
            ) if continuous else not self._candidate_near
            self._candidate_last_sequence = frame.sequence
        self._candidate_center = center
        self._candidate_angle = selected.angle_deg
        self._candidate_last_at = now

    def _candidate_world_position(
        self, candidate: RouteCandidate, frame: FramePacket
    ) -> Tuple[float, float]:
        """Approximate candidate endpoint relative to the saved old endpoint.

        This is deliberately a conservative image/command estimate until a
        calibrated image-to-ground transform or chassis pose is wired by the
        integration owner.  It is still useful as a bounded endpoint-pairing
        gate and is not presented as odometry.
        """
        height, width = frame.image.shape[:2]
        point = (
            candidate.entry_endpoint.point
            if candidate.entry_endpoint is not None
            else candidate.lower_point
        )
        y_ratio = point[1] / max(height, 1)
        x_error = (
            point[0] - width / 2.0
        ) / max(width / 2.0, 1.0)
        local_forward = 0.06 + max(0.0, 1.0 - y_ratio) * 0.80
        local_lateral = x_error * local_forward * 0.85
        heading = radians(self._heading_offset)
        return (
            self._pose_forward
            + local_forward * cos(heading)
            - local_lateral * sin(heading),
            self._pose_lateral
            + local_forward * sin(heading)
            + local_lateral * cos(heading),
        )

    def _candidate_ready(self, frame: FramePacket) -> Tuple[bool, str]:
        if self._candidate is None:
            return False, "no route fragment"
        if self._candidate_frames < CANDIDATE_CONFIRM_FRAMES:
            return False, (
                f"confirming candidate {self._candidate_frames}/"
                f"{CANDIDATE_CONFIRM_FRAMES}"
            )
        endpoint = self._candidate.entry_endpoint
        if endpoint is None or not endpoint.internal:
            return False, "candidate has no internal gap-facing endpoint"
        if endpoint.branch_length < MIN_LOCK_BRANCH_PIXELS:
            return False, "candidate branch is too short to identify a route end"
        height = frame.image.shape[0]
        endpoint_ratio = endpoint.point[1] / max(height, 1)
        if endpoint_ratio < MIN_CANDIDATE_ENDPOINT_Y_RATIO:
            return False, "candidate endpoint is too close to the upper image edge"
        if endpoint_ratio < MIN_LOCK_BOTTOM_RATIO:
            return False, "waiting for gap endpoint to enter the near band"
        if self._old_tangent_world is not None:
            world_tangent = self._heading_offset + endpoint.tangent_deg
            turn = self._angle_difference(world_tangent, self._old_tangent_world)
            if not RIGHT_ANGLE_MIN_DEG <= turn <= RIGHT_ANGLE_MAX_DEG:
                return False, (
                    f"candidate rejected as old/non-right-angle route ({turn:.0f}deg)"
                )
        endpoint_x, endpoint_y = self._candidate_world_position(
            self._candidate, frame
        )
        gap_distance = (endpoint_x * endpoint_x + endpoint_y * endpoint_y) ** 0.5
        if endpoint_x <= 0.02:
            return False, "candidate rejected behind old-route departure gate"
        if not MIN_ENDPOINT_GAP_DISTANCE <= gap_distance <= MAX_ENDPOINT_GAP_DISTANCE:
            return False, (
                f"candidate endpoint gap {gap_distance:.2f}m outside bounded gate"
            )
        if (
            self._candidate_near
            and self._pose_forward < OLD_LINE_CLEAR_DISTANCE
            and not self._candidate_seen_far
        ):
            return False, "near candidate rejected before clearing old route"
        return True, "candidate confirmed ahead of old-route gate"

    def _begin_end_approach(self, now: float) -> None:
        self.state = END_APPROACH
        self.started_at = now
        self._phase_started_at = now
        self._bottom_missing_at = None
        self._pose_forward = 0.0
        self._pose_lateral = 0.0
        self._heading_offset = 0.0
        self._last_motion_at = now
        self._last_motion = STOP_COMMAND
        if self._path.endpoint is not None:
            self._old_tangent_world = self._path.endpoint.tangent_deg

    def _begin_raise(self, now: float) -> None:
        if self.started_at is None:
            self.started_at = now
        # The departure gate is the physical endpoint, not the beginning of
        # END_APPROACH.  Reset the local pose when the bottom route vanishes.
        self._pose_forward = 0.0
        self._pose_lateral = 0.0
        self._heading_offset = 0.0
        self._last_motion_at = now
        self._last_motion = STOP_COMMAND
        if self._old_tangent_world is None:
            self._old_tangent_world = self._last_line_tangent
        self.state = RAISING_VIEW
        self._phase_started_at = now
        self._search_direction = self._last_tracking_direction
        self._candidate_frames = 0
        self._candidate_seen_far = False
        self._candidate_near = False
        self._candidate_center = None
        self._candidate_angle = None
        self._candidate_last_sequence = None
        self._stable_last_sequence = None
        self._line_detector.reset()

    def _begin_search(self, now: float) -> None:
        self.state = SEARCHING
        self._phase_started_at = now
        self._search_targets = [
            self._search_direction * target for target in SEARCH_TARGETS_DEG
        ]
        self._search_target_index = 0

    def _start_align(self, now: float) -> None:
        self.state = ALIGNING
        self._phase_started_at = now
        self._stable_frames = 0
        self._stable_last_sequence = None

    def _start_centering(self, now: float) -> None:
        self.state = CENTERING
        self._phase_started_at = now
        self._stable_frames = 0
        self._stable_last_sequence = None

    def _start_docking(self, now: float) -> None:
        self.state = DOCKING
        self._phase_started_at = now
        self._stable_frames = 0
        self._stable_last_sequence = None

    def _start_reacquiring(self, now: float) -> None:
        self.state = REACQUIRING
        self._phase_started_at = now
        self._stable_frames = 0
        self._stable_last_sequence = None
        self._line_detector.reset()

    @staticmethod
    def _candidate_target_error(
        candidate: RouteCandidate, frame: FramePacket
    ) -> float:
        width = frame.image.shape[1]
        # The contour centroid moves sideways as a long oblique segment enters
        # the image.  Its lower endpoint is the actual place the chassis must
        # reach, so steer to that instead.
        point = (
            candidate.entry_endpoint.point
            if candidate.entry_endpoint is not None
            else candidate.lower_point
        )
        return float(
            (point[0] - width / 2.0)
            / max(width / 2.0, 1.0)
        )

    @staticmethod
    def _candidate_axis_error(
        candidate: RouteCandidate, frame: FramePacket
    ) -> float:
        """Near-field cross-track error of the fitted new-route axis."""
        height, width = frame.image.shape[:2]
        if candidate.line_point is None:
            return RouteTask._candidate_target_error(candidate, frame)
        x0, y0 = candidate.line_point
        angle = radians(candidate.angle_deg)
        vx = sin(angle)
        vy = -cos(angle)
        reference_y = height * 0.84
        axis_x = x0
        if abs(vy) >= 0.25:
            axis_x = x0 + (reference_y - y0) * vx / vy
        return float(
            (axis_x - width / 2.0) / max(width / 2.0, 1.0)
        )

    @staticmethod
    def _alignment_yaw(angle_deg: float) -> float:
        if abs(angle_deg) <= ALIGN_ANGLE_DEG:
            return 0.0
        yaw = max(
            -ALIGN_MAX_YAW,
            min(angle_deg * ALIGN_YAW_GAIN, ALIGN_MAX_YAW),
        )
        if abs(yaw) < ALIGN_MIN_YAW:
            yaw = ALIGN_MIN_YAW if yaw > 0.0 else -ALIGN_MIN_YAW
        return float(yaw)

    def _line_view_running(self, now: float, message: str) -> TaskUpdate:
        self._record_motion(STOP_COMMAND, now)
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=STOP_COMMAND,
            detection=self.last_detection,
            message=message,
            gimbal=GimbalCommand(pitch=self._line_pitch, yaw=self._search_yaw),
        )

    def _step_end_approach(self, line, now: float) -> TaskUpdate:
        if line.valid:
            return self._finish(
                TaskStatus.COMPLETED,
                "complete line returned before physical endpoint",
            )
        if now - float(self._phase_started_at) >= END_APPROACH_MAX_SECONDS:
            return self._finish(
                TaskStatus.FAILED,
                "old route endpoint approach timed out",
            )
        if not self._bottom.present:
            if self._bottom_missing_at is None:
                self._bottom_missing_at = now
            if now - self._bottom_missing_at >= END_MISSING_SECONDS:
                self._begin_raise(now)
                return self._running(
                    now,
                    STOP_COMMAND,
                    "old route endpoint confirmed; raising view",
                )
            return self._running(
                now,
                STOP_COMMAND,
                "bottom route missing; confirming endpoint while stopped",
                search_view=False,
            )

        self._bottom_missing_at = None
        yaw = max(
            -END_APPROACH_MAX_YAW,
            min(
                self._bottom.error * END_APPROACH_YAW_GAIN,
                END_APPROACH_MAX_YAW,
            ),
        )
        return self._running(
            now,
            MotionCommand(forward=END_APPROACH_SPEED, yaw=yaw),
            "following bottom route to its physical endpoint",
            self._bottom.detection,
            search_view=False,
        )

    def _step_search(self, frame: FramePacket, now: float) -> TaskUpdate:
        ready, reason = self._candidate_ready(frame)
        if ready:
            self._start_align(now)
            return self._running(now, STOP_COMMAND, reason, self._candidate.detection)

        if self._search_target_index >= len(self._search_targets):
            return self._finish(TaskStatus.FAILED, "bounded route search exhausted")
        target = self._search_targets[self._search_target_index]
        target = max(-SEARCH_SOFT_LIMIT_DEG, min(target, SEARCH_SOFT_LIMIT_DEG))
        error = target - self._heading_offset
        if abs(error) <= SEARCH_TARGET_TOLERANCE_DEG:
            self._search_target_index += 1
            return self._running(
                now,
                STOP_COMMAND,
                f"search heading {target:.0f}deg inspected; advancing sweep",
            )
        if self._candidate is not None:
            speed = ALIGN_MAX_YAW
        elif abs(error) < 12.0:
            speed = SEARCH_CONFIRM_YAW_SPEED
        else:
            speed = SEARCH_YAW_SPEED
        yaw = speed if error > 0.0 else -speed
        message = f"searching toward {target:.0f}deg"
        if self._candidate is not None:
            message = f"{message}; {reason}"
        return self._running(
            now,
            MotionCommand(yaw=yaw),
            message,
            self._candidate.detection if self._candidate is not None else None,
        )

    def _step_align(self, frame: FramePacket, now: float) -> TaskUpdate:
        if self._candidate is None:
            if (
                self._candidate_last_at is not None
                and now - self._candidate_last_at < LOCKED_TARGET_LOSS_SECONDS
            ):
                return self._running(
                    now, STOP_COMMAND, "route heading briefly missing; stopped"
                )
            return self._finish(
                TaskStatus.FAILED,
                "locked route lost during alignment; stopped without rescan",
            )
        aligned = abs(self._candidate.angle_deg) <= ALIGN_ANGLE_DEG
        if frame.sequence != self._stable_last_sequence:
            self._stable_frames = self._stable_frames + 1 if aligned else 0
            self._stable_last_sequence = frame.sequence
        if self._stable_frames >= REACQUIRE_STABLE_FRAMES:
            self._start_centering(now)
            return self._running(
                now,
                STOP_COMMAND,
                "route heading aligned; centering fitted route axis",
                self._candidate.detection,
            )
        yaw = self._alignment_yaw(self._candidate.angle_deg)
        # Small image-feedback visibility correction, not a metric move to
        # the endpoint. A near-horizontal axis cannot be extrapolated safely
        # to a bottom reference row, so use the tracked endpoint here.
        visibility_error = self._candidate_target_error(self._candidate, frame)
        lateral = max(
            -ALIGN_MAX_LATERAL,
            min(visibility_error * ALIGN_VISIBILITY_GAIN, ALIGN_MAX_LATERAL),
        ) if abs(visibility_error) > CENTER_TARGET_ERROR else 0.0
        return self._running(
            now,
            MotionCommand(lateral=lateral, yaw=yaw),
            f"aligning with visibility correction {self._stable_frames}/"
            f"{REACQUIRE_STABLE_FRAMES}",
            self._candidate.detection,
        )

    def _step_centering(self, frame: FramePacket, now: float) -> TaskUpdate:
        if self._candidate is None:
            if (
                self._candidate_last_at is not None
                and now - self._candidate_last_at < LOCKED_TARGET_LOSS_SECONDS
            ):
                return self._running(
                    now, STOP_COMMAND, "locked route model briefly missing; stopped"
                )
            return self._finish(
                TaskStatus.FAILED,
                "locked route model lost while centering",
            )
        if abs(self._candidate.angle_deg) > CENTER_REALIGN_ANGLE_DEG:
            self._start_align(now)
            return self._running(
                now, STOP_COMMAND, "heading drifted while centering; realigning"
            )
        error = self._candidate_axis_error(self._candidate, frame)
        centered = abs(error) <= CENTER_TARGET_ERROR
        if frame.sequence != self._stable_last_sequence:
            self._stable_frames = self._stable_frames + 1 if centered else 0
            self._stable_last_sequence = frame.sequence
        if self._stable_frames >= REACQUIRE_STABLE_FRAMES:
            self._start_docking(now)
            return self._running(
                now, STOP_COMMAND, "fitted route axis centered; docking"
            )
        lateral = max(
            -CENTER_MAX_LATERAL,
            min(error * CENTER_LATERAL_GAIN, CENTER_MAX_LATERAL),
        )
        return self._running(
            now,
            MotionCommand(lateral=lateral),
            f"centering fitted route axis {self._stable_frames}/"
            f"{REACQUIRE_STABLE_FRAMES}",
            self._candidate.detection,
        )

    def _step_docking(self, frame: FramePacket, now: float) -> TaskUpdate:
        if self._candidate is None:
            if (
                self._candidate_last_at is not None
                and now - self._candidate_last_at < LOCKED_TARGET_LOSS_SECONDS
            ):
                return self._running(
                    now, STOP_COMMAND, "locked route model briefly missing; stopped"
                )
            return self._finish(
                TaskStatus.FAILED,
                "locked route model lost during final approach",
            )
        if abs(self._candidate.angle_deg) > DOCK_ANGLE_DEG:
            self._start_align(now)
            return self._running(
                now,
                STOP_COMMAND,
                "route heading drifted; realigning before final approach",
                self._candidate.detection,
            )
        target_error = self._candidate_axis_error(self._candidate, frame)
        ready = self._candidate_near and abs(target_error) <= DOCK_TARGET_ERROR
        if frame.sequence != self._stable_last_sequence:
            self._stable_frames = self._stable_frames + 1 if ready else 0
            self._stable_last_sequence = frame.sequence
        if self._stable_frames >= REACQUIRE_STABLE_FRAMES:
            self._start_reacquiring(now)
            return self._line_view_running(
                now, "route entry reached; lowering view for line verification"
            )
        yaw = max(
            -DOCK_MAX_YAW,
            min(self._candidate.angle_deg * ALIGN_YAW_GAIN, DOCK_MAX_YAW),
        )
        lateral = max(
            -DOCK_MAX_LATERAL,
            min(target_error * DOCK_LATERAL_GAIN, DOCK_MAX_LATERAL),
        )
        return self._running(
            now,
            MotionCommand(
                forward=DOCK_FORWARD_SPEED,
                lateral=lateral,
                yaw=yaw,
            ),
            f"docking onto route {self._stable_frames}/"
            f"{REACQUIRE_STABLE_FRAMES}",
            self._candidate.detection,
        )

    def _step_corner(self, line, now: float) -> TaskUpdate:
        """Follow a connected path that left the base detector's far band."""
        if line.valid:
            return self._finish(
                TaskStatus.COMPLETED,
                "connected corner returned to the base line detector",
            )
        endpoint = self._path.endpoint
        if not self._path.present or endpoint is None:
            if self._corner_missing_at is None:
                self._corner_missing_at = now
            if now - self._corner_missing_at >= CORNER_PATH_LOSS_SECONDS:
                self._begin_raise(now)
                return self._running(
                    now, STOP_COMMAND, "connected path ended; raising search view"
                )
            return self._running(
                now, STOP_COMMAND, "connected path briefly missing; stopped",
                search_view=False,
            )
        self._corner_missing_at = None
        if endpoint.internal:
            self._begin_end_approach(now)
            return self._running(
                now,
                MotionCommand(forward=END_APPROACH_SPEED),
                "internal route endpoint confirmed after connected corner",
                self._path.detection,
                search_view=False,
            )
        yaw = max(
            -CORNER_MAX_YAW,
            min(self._path.error * 52.0, CORNER_MAX_YAW),
        )
        return self._running(
            now,
            MotionCommand(forward=CORNER_FORWARD_SPEED, yaw=yaw),
            "following connected sharp corner; not treating it as a gap",
            self._path.detection,
            search_view=False,
        )

    def _step_reacquiring(self, line, frame: FramePacket, now: float) -> TaskUpdate:
        elapsed = now - float(self._phase_started_at)
        if elapsed < self._view_settle_seconds:
            return self._line_view_running(
                now, "lowering to line view; chassis stopped"
            )
        if frame.sequence != self._stable_last_sequence:
            self._stable_frames = self._stable_frames + 1 if line.valid else 0
            self._stable_last_sequence = frame.sequence
        if self._stable_frames >= REACQUIRE_STABLE_FRAMES:
            return self._finish(
                TaskStatus.COMPLETED,
                "base line detector confirmed new route",
            )
        if elapsed >= HANDOFF_VIEW_TIMEOUT_SECONDS:
            return self._finish(
                TaskStatus.FAILED,
                "new route was not valid after lowering to line view",
            )
        return self._line_view_running(
            now,
            f"verifying base line {self._stable_frames}/"
            f"{REACQUIRE_STABLE_FRAMES}",
        )

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        self._integrate_previous_command(now)
        line = self._line_detector.detect(frame.image)
        needs_low_view_geometry = (
            not line.valid
            and self.state in (MONITORING, END_APPROACH, CORNERING)
        )
        if needs_low_view_geometry:
            self._bottom = self._route_vision.bottom_line(
                frame.image, self._last_line_x
            )
            self._path = self._route_vision.connected_path(
                frame.image, self._last_line_x
            )
        else:
            self._bottom = BottomLineObservation(False)
            self._path = RoutePathObservation(False)

        if self.state == MONITORING:
            self._record_motion(STOP_COMMAND, now)
            if line.valid:
                self._remember_clear_line(line, now)
                self.last_detection = VisualDetection.no_result(KIND)
                return self._normal_update()

            if self._path.present and self._last_clear_at is not None:
                if self._possible_end_at is None:
                    self._possible_end_at = now
                self.last_detection = self._path.detection
                if now - self._possible_end_at >= END_PENDING_SECONDS:
                    if self._path.endpoint is not None and not self._path.endpoint.internal:
                        self.state = CORNERING
                        self.started_at = now
                        self._phase_started_at = now
                        return self._step_corner(line, now)
                    self._begin_end_approach(now)
                    return self._running(
                        now,
                        MotionCommand(forward=END_APPROACH_SPEED),
                        "physical route endpoint found; approaching it",
                        self._path.detection,
                        search_view=False,
                    )
                return self._normal_update(self._path.detection)

            self._possible_end_at = None
            self.last_detection = VisualDetection.no_result(KIND)
            if self._last_clear_at is None:
                return self._normal_update()
            trigger_delay = self._lost_grace + TRIGGER_MARGIN_SECONDS
            if now - self._last_clear_at <= trigger_delay:
                return self._normal_update()
            self._begin_raise(now)
            return self._running(
                now, STOP_COMMAND, "long blank confirmed; raising search view"
            )

        if self.started_at is None or now - self.started_at >= TOTAL_RECOVERY_SECONDS:
            return self._finish(TaskStatus.FAILED, "route recovery timed out")

        if self.state == END_APPROACH:
            self.last_detection = (
                self._bottom.detection
                if self._bottom.present
                else VisualDetection.no_result(KIND)
            )
            return self._step_end_approach(line, now)

        if self.state == CORNERING:
            self.last_detection = (
                self._path.detection
                if self._path.present
                else VisualDetection.no_result(KIND)
            )
            return self._step_corner(line, now)

        if self.state == RAISING_VIEW:
            if now - float(self._phase_started_at) < self._view_settle_seconds:
                self.last_detection = VisualDetection.no_result(KIND)
                return self._running(
                    now, STOP_COMMAND, "raising view; chassis stopped"
                )
            self.state = BRIDGING
            self._phase_started_at = now
            self._candidate_frames = 0
            self._candidate_seen_far = False
            self._candidate_near = False
            self._candidate_center = None
            self._candidate_angle = None
            self._candidate_last_sequence = None

        self._observe_candidate(frame, now)

        if self.state == BRIDGING:
            bridge_elapsed = now - float(self._phase_started_at)
            # The raised camera can still see the route just left behind.
            # Do not even accumulate confirmation history during clearance:
            # real logs showed candidate_frames already at 15/26 when this
            # phase ended, so the task immediately aligned to a far fragment.
            if bridge_elapsed < BRIDGE_MIN_SECONDS:
                observed = (
                    self._candidate.detection
                    if self._candidate is not None
                    else None
                )
                self._candidate = None
                self._candidate_frames = 0
                self._candidate_seen_far = False
                self._candidate_near = False
                self._candidate_center = None
                self._candidate_angle = None
                self._candidate_last_at = None
                self._candidate_last_sequence = None
                return self._running(
                    now,
                    MotionCommand(forward=BRIDGE_FORWARD_SPEED),
                    "crossing bounded blank; candidate history disabled "
                    "during initial old-line clearance",
                    observed,
                )

            ready, reason = self._candidate_ready(frame)
            if ready and bridge_elapsed >= BRIDGE_MIN_SECONDS:
                self._start_align(now)
                return self._running(
                    now, STOP_COMMAND, reason, self._candidate.detection
                )
            if bridge_elapsed >= BRIDGE_MAX_SECONDS:
                self._begin_search(now)
                return self._running(
                    now, STOP_COMMAND, "blank bridge budget ended; starting fan search"
                )
            message = "crossing bounded blank along old-route tangent"
            if self._candidate is not None:
                message = f"{message}; {reason}"
            return self._running(
                now,
                MotionCommand(forward=BRIDGE_FORWARD_SPEED),
                message,
                self._candidate.detection if self._candidate is not None else None,
            )

        if self.state == SEARCHING:
            return self._step_search(frame, now)

        if self.state == ALIGNING:
            return self._step_align(frame, now)

        if self.state == CENTERING:
            return self._step_centering(frame, now)

        if self.state == DOCKING:
            return self._step_docking(frame, now)

        if self.state == REACQUIRING:
            return self._step_reacquiring(line, frame, now)

        return self._finish(TaskStatus.FAILED, "invalid route recovery state")
