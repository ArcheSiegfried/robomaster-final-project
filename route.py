"""Bounded long-gap recovery task (WP6a / Issue #6).

Brief image misses remain the base follower's responsibility.  This task also
recognises the intermediate case where the far sample has vanished but the old
route is still under the bottom of the camera: it crawls to the physical end
before raising the shared view and crossing a bounded blank area.

The module consumes only ``FramePacket`` and returns typed motion/gimbal
requests.  It never connects hardware and every call is non-blocking.
"""

from math import cos, radians, sin
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
from route_detector import BottomLineObservation, RouteCandidate, RouteVision


KIND = "route"
MONITORING = "monitoring"
END_APPROACH = "end_approach"
RAISING_VIEW = "raising_view"
BRIDGING = "bridging"
SEARCHING = "searching"
APPROACHING = "approaching"
ALIGNING = "aligning"
DOCKING = "docking"
REACQUIRING = "reacquiring"

# Initial values only.  None has been validated on the real course.
TRIGGER_MARGIN_SECONDS = 0.05
END_PENDING_SECONDS = 0.08
END_MISSING_SECONDS = 0.14
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
BRIDGE_FORWARD_SPEED = 0.18
BRIDGE_SLOW_SPEED = 0.14
APPROACH_MIN_SPEED = 0.08
APPROACH_TARGET_GAIN = 34.0
BRIDGE_MAX_YAW = 26.0
FRAGMENT_LOSS_SECONDS = 0.20

SEARCH_YAW_SPEED = 45.0
SEARCH_CONFIRM_YAW_SPEED = 30.0
SEARCH_SOFT_LIMIT_DEG = 95.0
SEARCH_HARD_LIMIT_DEG = 100.0
SEARCH_TARGETS_DEG = (30.0, 60.0, 95.0, -30.0, -60.0, -95.0)
SEARCH_TARGET_TOLERANCE_DEG = 2.5

CANDIDATE_CONFIRM_FRAMES = 3
REACQUIRE_STABLE_FRAMES = 3
REACQUIRE_MAX_CENTER_JUMP = 0.24
REACQUIRE_MAX_ANGLE_JUMP = 42.0
NEAR_ENTER_RATIO = 0.78
NEAR_EXIT_RATIO = 0.70
MAX_APPROACH_ANGLE_DEG = 72.0
OLD_LINE_CLEAR_DISTANCE = 0.07
WIDE_TURN_NEAR_LIMIT_DEG = 78.0
ALIGN_ANGLE_DEG = 18.0
ALIGN_YAW_GAIN = 0.55
ALIGN_MIN_YAW = 8.0
DOCK_FORWARD_SPEED = 0.08
DOCK_TARGET_ERROR = 0.13
DOCK_ANGLE_DEG = 25.0
DOCK_YAW_GAIN = 34.0
DOCK_MAX_YAW = 22.0
HANDOFF_VIEW_TIMEOUT_SECONDS = 2.0
MAX_INTEGRATION_STEP_SECONDS = 0.20


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
        self._last_tracking_direction = 1.0
        self._search_direction = 1.0

        self._bottom = BottomLineObservation(False)
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
        self._bottom = BottomLineObservation(False)
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
            abs(current[0] - previous[0]) / max(frame_width, 1)
            <= REACQUIRE_MAX_CENTER_JUMP
        )

    @staticmethod
    def _angle_difference(first: float, second: float) -> float:
        """Smallest difference between two undirected line angles."""
        difference = abs(float(first) - float(second)) % 180.0
        return min(difference, 180.0 - difference)

    def _select_candidate(
        self, candidates: List[RouteCandidate], frame_width: int
    ) -> Optional[RouteCandidate]:
        if not candidates:
            return None
        if self._candidate_center is None:
            return candidates[0]

        ranked = []
        for candidate in candidates:
            center = candidate.detection.center
            if center is None:
                continue
            distance = abs(center[0] - self._candidate_center[0]) / max(
                frame_width, 1
            )
            angle_jump = (
                0.0
                if self._candidate_angle is None
                else self._angle_difference(
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

    def _observe_candidate(self, frame: FramePacket, now: float) -> None:
        candidates = self._route_vision.candidates(frame.image)
        selected = self._select_candidate(candidates, frame.image.shape[1])
        self._candidate = selected
        if selected is None or selected.detection.center is None:
            if (
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
        center = selected.detection.center
        continuous = self._center_continuous(
            center, self._candidate_center, frame.image.shape[1]
        ) and (
            self._candidate_angle is None
            or self._angle_difference(selected.angle_deg, self._candidate_angle)
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

    def _candidate_departure_position(
        self, candidate: RouteCandidate, frame: FramePacket
    ) -> float:
        """Estimate whether a fragment is ahead of the old-route endpoint."""
        height, width = frame.image.shape[:2]
        y_ratio = candidate.lower_point[1] / max(height, 1)
        x_error = (
            candidate.lower_point[0] - width / 2.0
        ) / max(width / 2.0, 1.0)
        local_forward = 0.06 + max(0.0, 1.0 - y_ratio) * 0.80
        local_lateral = x_error * local_forward * 0.85
        heading = radians(self._heading_offset)
        return (
            self._pose_forward
            + local_forward * cos(heading)
            - local_lateral * sin(heading)
        )

    def _candidate_ready(self, frame: FramePacket) -> Tuple[bool, str]:
        if self._candidate is None:
            return False, "no route fragment"
        if self._candidate_frames < CANDIDATE_CONFIRM_FRAMES:
            return False, (
                f"confirming candidate {self._candidate_frames}/"
                f"{CANDIDATE_CONFIRM_FRAMES}"
            )
        if abs(self._candidate.angle_deg) > MAX_APPROACH_ANGLE_DEG:
            return False, "candidate is too transverse; continuing fan search"
        departure = self._candidate_departure_position(self._candidate, frame)
        if departure <= 0.02:
            return False, "candidate rejected behind old-route departure gate"
        if (
            self._candidate_near
            and abs(self._heading_offset) > WIDE_TURN_NEAR_LIMIT_DEG
            and not self._candidate_seen_far
        ):
            return False, "near candidate rejected during wide turn"
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

    def _start_approach(self, now: float) -> None:
        self.state = APPROACHING
        self._phase_started_at = now

    def _start_align(self, now: float) -> None:
        self.state = ALIGNING
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
        return float(
            (candidate.lower_point[0] - width / 2.0)
            / max(width / 2.0, 1.0)
        )

    @staticmethod
    def _alignment_yaw(angle_deg: float) -> float:
        if abs(angle_deg) <= ALIGN_ANGLE_DEG:
            return 0.0
        yaw = max(
            -SEARCH_CONFIRM_YAW_SPEED,
            min(angle_deg * ALIGN_YAW_GAIN, SEARCH_CONFIRM_YAW_SPEED),
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
            if self._candidate_near:
                self._start_align(now)
            else:
                self._start_approach(now)
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
        speed = (
            SEARCH_CONFIRM_YAW_SPEED
            if abs(error) < 12.0 or self._candidate is not None
            else SEARCH_YAW_SPEED
        )
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

    def _step_approach(self, frame: FramePacket, now: float) -> TaskUpdate:
        if self._candidate is None:
            if (
                self._candidate_last_at is None
                or now - self._candidate_last_at >= FRAGMENT_LOSS_SECONDS
            ):
                self._begin_search(now)
                return self._running(
                    now, STOP_COMMAND, "route fragment lost; returning to search"
                )
            return self._running(
                now, STOP_COMMAND, "route fragment briefly missing; stopped"
            )
        if abs(self._candidate.angle_deg) > MAX_APPROACH_ANGLE_DEG:
            self._begin_search(now)
            return self._running(
                now,
                STOP_COMMAND,
                "route became transverse; returning to fan search",
                self._candidate.detection,
            )
        if self._candidate_near:
            self._start_align(now)
            return self._running(
                now,
                STOP_COMMAND,
                "route endpoint is near; aligning",
                self._candidate.detection,
            )
        target_error = self._candidate_target_error(self._candidate, frame)
        yaw = max(
            -BRIDGE_MAX_YAW,
            min(target_error * APPROACH_TARGET_GAIN, BRIDGE_MAX_YAW),
        )
        forward = (
            APPROACH_MIN_SPEED
            if abs(target_error) > 0.45
            else BRIDGE_SLOW_SPEED
        )
        return self._running(
            now,
            MotionCommand(forward=forward, yaw=yaw),
            "approaching confirmed route endpoint",
            self._candidate.detection,
        )

    def _step_align(self, frame: FramePacket, now: float) -> TaskUpdate:
        if self._candidate is None:
            if (
                self._candidate_last_at is not None
                and now - self._candidate_last_at < FRAGMENT_LOSS_SECONDS
            ):
                return self._running(
                    now, STOP_COMMAND, "route heading briefly missing; stopped"
                )
            self._begin_search(now)
            return self._running(
                now, STOP_COMMAND, "route heading lost; returning to search"
            )
        aligned = abs(self._candidate.angle_deg) <= ALIGN_ANGLE_DEG
        if frame.sequence != self._stable_last_sequence:
            self._stable_frames = self._stable_frames + 1 if aligned else 0
            self._stable_last_sequence = frame.sequence
        if self._stable_frames >= REACQUIRE_STABLE_FRAMES:
            self._start_docking(now)
            return self._running(
                now,
                STOP_COMMAND,
                "route heading aligned; starting final approach",
                self._candidate.detection,
            )
        yaw = self._alignment_yaw(self._candidate.angle_deg)
        return self._running(
            now,
            MotionCommand(yaw=yaw),
            f"aligning route heading {self._stable_frames}/"
            f"{REACQUIRE_STABLE_FRAMES}",
            self._candidate.detection,
        )

    def _step_docking(self, frame: FramePacket, now: float) -> TaskUpdate:
        if self._candidate is None:
            if (
                self._candidate_last_at is not None
                and now - self._candidate_last_at < FRAGMENT_LOSS_SECONDS
            ):
                return self._running(
                    now, STOP_COMMAND, "final-approach route briefly missing; stopped"
                )
            self._begin_search(now)
            return self._running(
                now, STOP_COMMAND, "final-approach route lost; returning to search"
            )
        if abs(self._candidate.angle_deg) > DOCK_ANGLE_DEG:
            self._start_align(now)
            return self._running(
                now,
                STOP_COMMAND,
                "route heading drifted; realigning before final approach",
                self._candidate.detection,
            )
        target_error = self._candidate_target_error(self._candidate, frame)
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
            min(target_error * DOCK_YAW_GAIN, DOCK_MAX_YAW),
        )
        return self._running(
            now,
            MotionCommand(forward=DOCK_FORWARD_SPEED, yaw=yaw),
            f"docking onto route {self._stable_frames}/"
            f"{REACQUIRE_STABLE_FRAMES}",
            self._candidate.detection,
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
        self._bottom = self._route_vision.bottom_line(
            frame.image, self._last_line_x
        )

        if self.state == MONITORING:
            self._record_motion(STOP_COMMAND, now)
            if line.valid:
                self._remember_clear_line(line, now)
                self.last_detection = VisualDetection.no_result(KIND)
                return self._normal_update()

            if self._bottom.present and self._last_clear_at is not None:
                if self._possible_end_at is None:
                    self._possible_end_at = now
                self.last_detection = self._bottom.detection
                if now - self._possible_end_at >= END_PENDING_SECONDS:
                    self._begin_end_approach(now)
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
                        "far route ended; following bottom route to endpoint",
                        self._bottom.detection,
                        search_view=False,
                    )
                return self._normal_update(self._bottom.detection)

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
            ready, reason = self._candidate_ready(frame)
            bridge_elapsed = now - float(self._phase_started_at)
            # The raised camera can still see the route just left behind.
            # Never stop the first crossing merely because that old fragment
            # survives the generic candidate filter for three frames.
            if ready and bridge_elapsed >= BRIDGE_MIN_SECONDS:
                if self._candidate_near:
                    self._start_align(now)
                else:
                    self._start_approach(now)
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
                if bridge_elapsed < BRIDGE_MIN_SECONDS:
                    reason = "ignoring candidate during initial old-line clearance"
                message = f"{message}; {reason}"
            return self._running(
                now,
                MotionCommand(forward=BRIDGE_FORWARD_SPEED),
                message,
                self._candidate.detection if self._candidate is not None else None,
            )

        if self.state == SEARCHING:
            return self._step_search(frame, now)

        if self.state == APPROACHING:
            return self._step_approach(frame, now)

        if self.state == ALIGNING:
            return self._step_align(frame, now)

        if self.state == DOCKING:
            return self._step_docking(frame, now)

        if self.state == REACQUIRING:
            return self._step_reacquiring(line, frame, now)

        return self._finish(TaskStatus.FAILED, "invalid route recovery state")
