"""Experimental right-angle gap recovery for ``route_only_main.py`` only.

The camera looks along the new straight route once. The chassis keeps its old
heading while it moves forward until that route reaches the side-view near
axis. Only then does the chassis turn to the camera's heading. No gap length
or timed translation is used as the turn condition; time remains a hard stop.
"""

from typing import Optional, Tuple

from models import (
    FramePacket,
    GimbalCommand,
    MotionCommand,
    STOP_COMMAND,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)
from route import (
    HANDOFF_VIEW_TIMEOUT_SECONDS,
    KIND,
    LINE_CENTER_MIN_LATERAL,
    LINE_CENTER_MAX_LATERAL,
    MAX_ENDPOINT_GAP_DISTANCE,
    MIN_ENDPOINT_GAP_DISTANCE,
    MIN_LOCK_BRANCH_PIXELS,
    RIGHT_ANGLE_MAX_DEG,
    RIGHT_ANGLE_MIN_DEG,
    TOTAL_RECOVERY_SECONDS,
    RouteTask,
)
from route_detector import RouteCandidate


SIDE_AIM = "side_aim"
SIDE_ADVANCE = "side_advance"
BODY_TURN = "body_turn"

AIM_MIN_DEG = 75.0
AIM_MAX_DEG = 100.0
AIM_ABSOLUTE_LIMIT_DEG = 100.0
AIM_SETTLE_MARGIN_SECONDS = 0.30
STOP_BEFORE_AIM_SECONDS = 0.22
SIDE_LINE_MAX_ANGLE_DEG = 80.0
SIDE_VISIBLE_MIN_ROW = 0.40
SIDE_CENTER_ERROR = 0.08
SIDE_CONFIRM_FRAMES = 3
SIDE_LOSS_SECONDS = 0.55
SIDE_INITIAL_FIND_SECONDS = 2.4
SIDE_ADVANCE_MAX_SECONDS = 3.8
SIDE_ADVANCE_SPEED = 0.12
SIDE_MAX_AXIS_JUMP = 0.35
SIDE_AIM_ENDPOINT_MIN_ROW = 0.77
BODY_TURN_SPEED = 24.0
BODY_TURN_TOLERANCE_DEG = 3.0
BODY_TURN_MAX_SECONDS = 5.0
LOW_GUIDE_MIN_FRAMES = 2
LOW_GUIDE_MAX_ERROR = 0.55
LOW_GUIDE_CENTER_ERROR = 0.07
LOW_GUIDE_LATERAL_GAIN = 0.18
LOW_GUIDE_FORWARD_SPEED = 0.055
LOW_GUIDE_YAW_GAIN = 0.20
LOW_GUIDE_MAX_YAW = 10.0


class GimbalAlignedRouteTask(RouteTask):
    """Reuse old-line detection, but replace the gap joining manoeuvre."""

    name = "route"

    def __init__(self, settings=None) -> None:
        super().__init__(settings)
        self._clear_side_state()

    def _clear_side_state(self) -> None:
        self._aim_yaw: Optional[float] = None
        self._aim_error: Optional[str] = None
        self._aim_requested_at: Optional[float] = None
        self._side_last_seen_at: Optional[float] = None
        self._side_last_error: Optional[float] = None
        self._side_last_sequence: Optional[int] = None
        self._side_confirm_frames = 0
        self._low_guide_frames = 0
        self._low_guide_last_sequence: Optional[int] = None

    def reset(self) -> None:
        super().reset()
        self._clear_side_state()

    def _candidate_ready(self, frame: FramePacket) -> Tuple[bool, str]:
        ready, reason = super()._candidate_ready(frame)
        if ready or reason != "waiting for gap endpoint to enter the near band":
            return ready, reason
        # The raised camera saw the real end repeatedly at row 0.79 in the
        # field log, but the legacy docking gate demanded row 0.80. For a
        # one-look aim we need the tangent, not a near-field docking pose.
        candidate = self._candidate
        endpoint = candidate.entry_endpoint
        if candidate.bottom_ratio < SIDE_AIM_ENDPOINT_MIN_ROW:
            return False, reason
        if self._old_tangent_world is not None:
            world_tangent = self._heading_offset + endpoint.tangent_deg
            turn = self._angle_difference(world_tangent, self._old_tangent_world)
            if not RIGHT_ANGLE_MIN_DEG <= turn <= RIGHT_ANGLE_MAX_DEG:
                return False, "candidate is not a right-angle new route"
        endpoint_x, endpoint_y = self._candidate_world_position(candidate, frame)
        gap_distance = (endpoint_x * endpoint_x + endpoint_y * endpoint_y) ** 0.5
        if endpoint_x <= 0.02:
            return False, "candidate lies behind old-route departure gate"
        if not MIN_ENDPOINT_GAP_DISTANCE <= gap_distance <= MAX_ENDPOINT_GAP_DISTANCE:
            return False, "candidate endpoint is outside bounded gap gate"
        # Unlike a near-field dock, a raised-view endpoint must have been
        # observed farther away before it can become the fixed side-look aim.
        if not self._candidate_seen_far:
            return False, "candidate has no far-to-near history"
        return True, "raised-view endpoint confirmed for one side look"

    def _start_low_approach(self, now: float) -> None:
        """Lock the raised-view endpoint and make one side-looking request."""
        candidate = self._candidate
        self._clear_side_state()
        if candidate is None or candidate.entry_endpoint is None:
            self._aim_error = "new route endpoint vanished before aiming"
        else:
            tangent = candidate.entry_endpoint.tangent_deg
            # A gap-facing endpoint can be on the opposite image side from
            # the route it leads into. Its *directed tangent*, not its x
            # coordinate, determines which way the camera must look.
            side = -1.0 if tangent < 0.0 else 1.0
            relative = side * max(
                AIM_MIN_DEG, min(abs(tangent), AIM_MAX_DEG)
            )
            target = self._heading_offset + relative
            if abs(target) > AIM_ABSOLUTE_LIMIT_DEG:
                self._aim_error = "new line exceeds one-look gimbal yaw limit"
            else:
                self._aim_yaw = target
        self.state = SIDE_AIM
        self._phase_started_at = now
        self._candidate = None
        self._candidate_center = None
        self._candidate_angle = None
        self._candidate_frames = 0
        self._candidate_last_at = None
        self._candidate_last_sequence = None

    def _line_view_running(self, now: float, message: str) -> TaskUpdate:
        # The inherited bridge/search path calls this immediately after
        # _start_low_approach. Override only that transition's gimbal request.
        if self.state != SIDE_AIM:
            return super()._line_view_running(now, message)
        self._record_motion(STOP_COMMAND, now)
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=STOP_COMMAND,
            detection=self.last_detection,
            message="physical gap endpoint locked; stopping before side look",
            gimbal=GimbalCommand(pitch=self._search_pitch, yaw=0.0),
        )

    def _step_search(self, frame: FramePacket, now: float) -> TaskUpdate:
        # This experiment does not sweep or turn the chassis to discover a
        # target. A missing physical endpoint requires an explicit restart.
        return self._finish(
            TaskStatus.FAILED, "new endpoint not visible in bounded bridge; stopped"
        )

    def _side_candidate(self, frame: FramePacket) -> Optional[RouteCandidate]:
        height = frame.image.shape[0]
        candidates = []
        for candidate in self._route_vision.candidates(frame.image):
            if candidate.line_point is None or candidate.detection.box is None:
                continue
            if not candidate.endpoints or max(
                endpoint.branch_length for endpoint in candidate.endpoints
            ) < MIN_LOCK_BRANCH_PIXELS:
                continue
            # Looking along the new line from the side can produce a strongly
            # diagonal image segment. Its physical end may be outside the
            # image, so the old endpoint-only/near-vertical gate is invalid.
            axis_angle = abs(candidate.angle_deg) % 180.0
            axis_angle = min(axis_angle, 180.0 - axis_angle)
            if axis_angle > SIDE_LINE_MAX_ANGLE_DEG:
                continue
            if candidate.detection.box[3] / max(height, 1) < SIDE_VISIBLE_MIN_ROW:
                continue
            error = self._candidate_axis_error(candidate, frame)
            if (
                self._side_last_error is not None
                and abs(error - self._side_last_error) > SIDE_MAX_AXIS_JUMP
            ):
                continue
            continuity = (
                0.0 if self._side_last_error is None
                else 0.2 * (1.0 - abs(error - self._side_last_error))
            )
            candidates.append((candidate.score + continuity, candidate))
        return max(candidates, key=lambda item: item[0])[1] if candidates else None

    def _side_update(
        self, now: float, motion: MotionCommand, message: str,
        detection: Optional[VisualDetection] = None,
        look_side: bool = True,
    ) -> TaskUpdate:
        self._record_motion(motion, now)
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=motion,
            detection=detection or VisualDetection.no_result(KIND),
            message=message,
            gimbal=GimbalCommand(
                pitch=self._search_pitch,
                yaw=self._aim_yaw if look_side else 0.0,
            ),
        )

    def _step_side(self, frame: FramePacket, now: float) -> TaskUpdate:
        if self._aim_error is not None or self._aim_yaw is None:
            return self._finish(
                TaskStatus.FAILED, self._aim_error or "no side-view aim"
            )
        elapsed = now - float(self._phase_started_at)
        if self.state == SIDE_AIM:
            if elapsed < STOP_BEFORE_AIM_SECONDS:
                return self._side_update(
                    now, STOP_COMMAND,
                    "stopped before switching to independent gimbal yaw",
                    look_side=False,
                )
            if self._aim_requested_at is None:
                self._aim_requested_at = now
            yaw_speed = max(float(self.settings.gimbal_yaw_speed), 1.0)
            settle = max(
                self._view_settle_seconds,
                abs(self._aim_yaw) / yaw_speed + AIM_SETTLE_MARGIN_SECONDS,
            )
            if now - self._aim_requested_at < settle:
                return self._side_update(
                    now, STOP_COMMAND, "waiting for one side-looking gimbal aim"
                )
            self.state = SIDE_ADVANCE
            self._phase_started_at = now
            elapsed = 0.0

        if self.state == SIDE_ADVANCE:
            if elapsed >= SIDE_ADVANCE_MAX_SECONDS:
                return self._finish(
                    TaskStatus.FAILED, "side-view forward approach timed out"
                )
            candidate = self._side_candidate(frame)
            self._candidate = candidate
            if candidate is None:
                self.last_detection = VisualDetection.no_result(KIND)
                self._side_confirm_frames = 0
                if self._side_last_seen_at is None:
                    if elapsed >= SIDE_INITIAL_FIND_SECONDS:
                        return self._finish(
                            TaskStatus.FAILED,
                            "new line never entered side view; stopped",
                        )
                    return self._side_update(
                        now, MotionCommand(forward=SIDE_ADVANCE_SPEED),
                        "bounded slow advance while new line enters side view",
                    )
                if now - float(self._side_last_seen_at) >= SIDE_LOSS_SECONDS:
                    return self._finish(
                        TaskStatus.FAILED, "new line lost after side-view acquisition"
                    )
                return self._side_update(
                    now, STOP_COMMAND, "side-view line temporarily lost; stopped"
                )
            self._side_last_seen_at = now
            self.last_detection = candidate.detection
            error = self._candidate_axis_error(candidate, frame)
            self._side_last_error = error
            if abs(error) > SIDE_CENTER_ERROR and error * self._aim_yaw > 0.0:
                return self._finish(
                    TaskStatus.FAILED,
                    "side-view line lies behind forward crossing direction",
                )
            if frame.sequence != self._side_last_sequence:
                self._side_confirm_frames = (
                    self._side_confirm_frames + 1
                    if abs(error) <= SIDE_CENTER_ERROR else 0
                )
                self._side_last_sequence = frame.sequence
            if self._side_confirm_frames >= SIDE_CONFIRM_FRAMES:
                self.state = BODY_TURN
                self._phase_started_at = now
                return self._side_update(
                    now, STOP_COMMAND,
                    "side-view line centered; stopped before body turn",
                    candidate.detection,
                )
            motion = (
                STOP_COMMAND if abs(error) <= SIDE_CENTER_ERROR
                else MotionCommand(forward=SIDE_ADVANCE_SPEED)
            )
            return self._side_update(
                now, motion,
                f"side-view approach, near-axis error {error:+.2f}; "
                f"confirm {self._side_confirm_frames}/{SIDE_CONFIRM_FRAMES}",
                candidate.detection,
            )

        if elapsed >= BODY_TURN_MAX_SECONDS:
            return self._finish(TaskStatus.FAILED, "body turn timed out")
        remaining = self._aim_yaw - self._heading_offset
        if abs(remaining) <= BODY_TURN_TOLERANCE_DEG:
            self._start_reacquiring(now)
            return self._line_view_running(
                now, "body reached camera heading; restoring forward line view"
            )
        yaw = BODY_TURN_SPEED if remaining > 0.0 else -BODY_TURN_SPEED
        return self._side_update(
            now, MotionCommand(yaw=yaw),
            f"body following fixed camera heading; remaining {remaining:+.1f}deg",
        )

    def _step_reacquiring(self, line, frame: FramePacket, now: float) -> TaskUpdate:
        # The normal detector deliberately requires both its near and far ROI
        # bands. Immediately after a right-angle gap, the new route can bend
        # out of its far band despite clearly crossing the lower image. Keep
        # task ownership and use only the near blue component to guide a
        # small, bounded approach; never declare success from this fallback.
        elapsed = now - float(self._phase_started_at)
        settle = 0.0 if self._reacquire_view_already_low else self._view_settle_seconds
        if line.valid or elapsed < settle or elapsed - settle >= HANDOFF_VIEW_TIMEOUT_SECONDS:
            return super()._step_reacquiring(line, frame, now)

        self._stable_frames = 0
        near = self._route_vision.bottom_line(
            frame.image, frame.image.shape[1] / 2.0
        )
        if not near.present or abs(near.error) > LOW_GUIDE_MAX_ERROR:
            self._low_guide_frames = 0
            return self._line_view_running(
                now, "normal line invalid and no safe near-line guide; stopped"
            )
        if frame.sequence != self._low_guide_last_sequence:
            self._low_guide_frames += 1
            self._low_guide_last_sequence = frame.sequence
        if self._low_guide_frames < LOW_GUIDE_MIN_FRAMES:
            return self._line_view_running(
                now, "confirming near blue line before low-view correction"
            )
        self.last_detection = near.detection
        if abs(near.error) > LOW_GUIDE_CENTER_ERROR:
            lateral = max(
                -LINE_CENTER_MAX_LATERAL,
                min(near.error * LOW_GUIDE_LATERAL_GAIN, LINE_CENTER_MAX_LATERAL),
            )
            if abs(lateral) < LINE_CENTER_MIN_LATERAL:
                lateral = LINE_CENTER_MIN_LATERAL if lateral > 0.0 else -LINE_CENTER_MIN_LATERAL
            motion = MotionCommand(lateral=lateral)
        else:
            yaw = max(
                -LOW_GUIDE_MAX_YAW,
                min(near.angle_deg * LOW_GUIDE_YAW_GAIN, LOW_GUIDE_MAX_YAW),
            )
            motion = MotionCommand(forward=LOW_GUIDE_FORWARD_SPEED, yaw=yaw)
        return self._line_view_motion(
            now, motion,
            f"near-line guiding until normal detector recovers; error {near.error:+.2f}",
            near.detection,
        )

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        if self.state in (SIDE_AIM, SIDE_ADVANCE, BODY_TURN):
            self._integrate_previous_command(now)
            if (
                self.started_at is None
                or now - self.started_at >= TOTAL_RECOVERY_SECONDS
            ):
                return self._finish(TaskStatus.FAILED, "route recovery timed out")
            return self._step_side(frame, now)
        return super().step(frame, now)
