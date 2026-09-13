"""Number-marker filtering, target choice, aiming, and evidence handoff.

The task consumes the repository's shared ``FramePacket`` and returns only the
frozen v0.1 task types. It owns no camera, vision subscription, gimbal, chassis,
or file writer. The integration layer may inject current marker observations
and must consume/acknowledge ``EvidenceRequest`` objects.

All defaults in ``NumberMarkerConfig`` are engineering starting values. They
are not claimed as teacher-specified tolerances or hardware-verified settings.
"""

import math
import threading
from dataclasses import dataclass, replace
from enum import Enum
from typing import Callable, FrozenSet, Iterable, Optional, Sequence, Tuple

import cv2
import numpy as np

from models import FramePacket, MotionCommand, TaskStatus, TaskUpdate, VisualDetection

KIND = "number_marker"
VALID_MARKER_IDS: FrozenSet[str] = frozenset(("1", "2", "3", "4", "5"))
SUPPORTED_IDS = tuple(sorted(VALID_MARKER_IDS))

# There is no calibrated marker range or ToF-to-marker association in the
# current integration framework. This explicit name prevents the fallback
# from being mistaken for a real physical-nearest measurement.
NEAREST_PROXY = "NEAREST_PROXY_LARGEST_APPARENT_WIDTH"
MEASURED_DISTANCE = "MEASURED_DISTANCE"


class MarkerState(str, Enum):
    IDLE = "IDLE"
    TARGET_FOUND = "TARGET_FOUND"
    TARGET_TOO_SMALL = "TARGET_TOO_SMALL"
    STOPPING = "STOPPING"
    AIMING = "AIMING"
    AIM_LOCKED = "AIM_LOCKED"
    EVIDENCE_PENDING = "EVIDENCE_PENDING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    LOST = "LOST"


@dataclass(frozen=True)
class NumberMarkerConfig:
    """Module-local settings pending an integration-owned config migration."""

    min_marker_width_ratio: float = 0.20
    aim_stable_frames: int = 3
    # Engineering interpretation of the ambiguous "1/10 marker size" center
    # region: these are full zone fractions, so the error limits are half.
    center_zone_width_fraction: float = 0.10
    center_zone_height_fraction: float = 0.10
    max_frame_age: float = 0.15
    max_observation_age: float = 0.15
    target_lost_timeout: float = 0.30
    max_task_seconds: float = 8.0
    yaw_kp: float = 70.0
    pitch_kp: float = 45.0
    max_aim_yaw_rate: float = 45.0
    max_aim_pitch_rate: float = 30.0
    # Positive image-x error means the marker is to the right. The repository
    # convention says positive chassis yaw turns right, but hardware sign still
    # requires wheels-raised validation.
    yaw_direction_sign: float = 1.0
    # Pitch is an intent only until the integration layer exposes a gimbal
    # command. Its physical sign must be validated on the actual robot.
    pitch_direction_sign: float = 1.0
    max_evidence_attempts: int = 1
    team_number: Optional[str] = None


@dataclass(frozen=True)
class MarkerCandidate:
    """One marker observation in full-frame pixel coordinates."""

    target_id: str
    center: Tuple[float, float]
    width: float
    height: float
    confidence: float = 1.0
    observed_at: Optional[float] = None
    source_sequence: Optional[int] = None
    estimated_distance_m: Optional[float] = None

    def width_ratio(self, frame_width: int) -> float:
        return self.width / float(frame_width)

    def box(self, frame_width: int, frame_height: int) -> Tuple[int, int, int, int]:
        half_width = self.width / 2.0
        half_height = self.height / 2.0
        left = max(0, min(frame_width, int(round(self.center[0] - half_width))))
        top = max(0, min(frame_height, int(round(self.center[1] - half_height))))
        right = max(0, min(frame_width, int(round(self.center[0] + half_width))))
        bottom = max(0, min(frame_height, int(round(self.center[1] + half_height))))
        return left, top, right, bottom

    def to_detection(self, frame_width: int, frame_height: int) -> VisualDetection:
        return VisualDetection(
            valid=True,
            kind=KIND,
            center=(int(round(self.center[0])), int(round(self.center[1]))),
            target_id=self.target_id,
            confidence=max(0.0, min(float(self.confidence), 1.0)),
            box=self.box(frame_width, frame_height),
        )


@dataclass(frozen=True)
class TargetSelection:
    candidate: MarkerCandidate
    strategy: str


@dataclass(frozen=True)
class AimIntent:
    """Both aiming axes, including the axis the current motion type cannot send."""

    horizontal_error: float
    vertical_error: float
    yaw_rate: float
    pitch_rate: float
    gimbal_pitch_required: bool


@dataclass(frozen=True)
class EvidenceRequest:
    """One scoring-image request for the integration-owned evidence writer."""

    request_id: str
    marker_id: str
    frame_sequence: int
    captured_at: float
    detection: VisualDetection
    annotation: str
    text_anchor: Tuple[int, int]
    image: np.ndarray
    attempt: int = 1


ObservationProvider = Callable[[FramePacket, float], Sequence[MarkerCandidate]]


def _finite(value: object) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def marker_candidates_from_normalized(
    marker_info: Iterable[Sequence[object]],
    frame_width: int,
    frame_height: int,
    observed_at: float,
    source_sequence: Optional[int] = None,
) -> Tuple[MarkerCandidate, ...]:
    """Convert verified DJI marker tuples ``(x, y, w, h, info)`` to pixels.

    This pure conversion performs no subscription and no hardware access. Bad
    tuples are dropped so callback corruption cannot preserve an old command.
    """

    converted = []
    if frame_width <= 0 or frame_height <= 0 or _finite(observed_at) is None:
        return ()
    for item in marker_info:
        try:
            x, y, width, height, target_id = item[:5]
        except (TypeError, ValueError, IndexError):
            continue
        x = _finite(x)
        y = _finite(y)
        width = _finite(width)
        height = _finite(height)
        if None in (x, y, width, height):
            continue
        converted.append(
            MarkerCandidate(
                target_id=str(target_id),
                center=(x * frame_width, y * frame_height),
                width=width * frame_width,
                height=height * frame_height,
                observed_at=float(observed_at),
                source_sequence=source_sequence,
            )
        )
    return tuple(converted)


def evaluate_number_markers(
    candidates: Iterable[MarkerCandidate],
    frame_width: int,
    frame_height: int,
    now: float,
    frame_sequence: int,
    aimed_ids: Iterable[str] = (),
    max_observation_age: float = 0.15,
) -> Tuple[MarkerCandidate, ...]:
    """Return fresh, finite IDs 1--5 that have not already been aimed."""

    aimed = {str(value) for value in aimed_ids}
    accepted = []
    for candidate in candidates:
        if not isinstance(candidate, MarkerCandidate):
            continue
        if candidate.target_id not in VALID_MARKER_IDS or candidate.target_id in aimed:
            continue
        values = (
            candidate.center[0],
            candidate.center[1],
            candidate.width,
            candidate.height,
            candidate.confidence,
        )
        if any(_finite(value) is None for value in values):
            continue
        if candidate.width <= 0.0 or candidate.height <= 0.0:
            continue
        if not (0.0 <= candidate.center[0] <= frame_width):
            continue
        if not (0.0 <= candidate.center[1] <= frame_height):
            continue
        if candidate.source_sequence is not None and candidate.source_sequence != frame_sequence:
            continue
        if candidate.observed_at is None and candidate.source_sequence is None:
            # An asynchronous observation with no timestamp or frame identity
            # can never be proven fresh, so it is unsafe to act on it.
            continue
        if candidate.observed_at is not None:
            age = now - candidate.observed_at
            if not math.isfinite(age) or age < 0.0 or age > max_observation_age:
                continue
        if candidate.estimated_distance_m is not None:
            distance = _finite(candidate.estimated_distance_m)
            if distance is None or distance <= 0.0:
                continue
        accepted.append(candidate)
    return tuple(accepted)


def select_target_marker(candidates: Sequence[MarkerCandidate]) -> Optional[TargetSelection]:
    """Select physical nearest only when every candidate has real distance.

    Without complete positive distance estimates, the isolated fallback chooses
    the largest apparent marker width and labels the decision ``NEAREST_PROXY``.
    It is an explicit engineering proxy, not proof of physical distance.
    """

    if not candidates:
        return None
    if all(candidate.estimated_distance_m is not None for candidate in candidates):
        chosen = min(
            candidates,
            key=lambda item: (
                float(item.estimated_distance_m),
                -item.width,
                item.target_id,
            ),
        )
        return TargetSelection(chosen, MEASURED_DISTANCE)
    chosen = max(candidates, key=lambda item: (item.width, item.height, item.target_id))
    return TargetSelection(chosen, NEAREST_PROXY)


def is_marker_eligible(
    candidate: MarkerCandidate,
    frame_width: int,
    minimum_ratio: float = 0.20,
) -> bool:
    """Apply the Final requirement's strict ``width / frame_width > 0.20``."""

    return frame_width > 0 and candidate.width_ratio(frame_width) > minimum_ratio


def is_marker_centered(
    candidate: MarkerCandidate,
    frame_width: int,
    frame_height: int,
    settings: Optional[NumberMarkerConfig] = None,
) -> bool:
    """Apply the isolated, configurable center-tolerance interpretation."""

    config = settings or NumberMarkerConfig()
    error_x = abs(candidate.center[0] - frame_width / 2.0)
    error_y = abs(candidate.center[1] - frame_height / 2.0)
    tolerance_x = candidate.width * config.center_zone_width_fraction / 2.0
    tolerance_y = candidate.height * config.center_zone_height_fraction / 2.0
    return error_x <= tolerance_x and error_y <= tolerance_y


def compute_aim_intent(
    candidate: MarkerCandidate,
    frame_width: int,
    frame_height: int,
    settings: Optional[NumberMarkerConfig] = None,
) -> AimIntent:
    """Compute bounded horizontal chassis yaw and separate vertical pitch intent."""

    config = settings or NumberMarkerConfig()
    horizontal_error = (candidate.center[0] - frame_width / 2.0) / (frame_width / 2.0)
    vertical_error = (candidate.center[1] - frame_height / 2.0) / (frame_height / 2.0)
    yaw_rate = config.yaw_direction_sign * config.yaw_kp * horizontal_error
    pitch_rate = config.pitch_direction_sign * config.pitch_kp * vertical_error
    yaw_rate = max(-config.max_aim_yaw_rate, min(yaw_rate, config.max_aim_yaw_rate))
    pitch_rate = max(
        -config.max_aim_pitch_rate, min(pitch_rate, config.max_aim_pitch_rate)
    )
    vertically_centered = is_marker_centered(
        replace(candidate, center=(frame_width / 2.0, candidate.center[1])),
        frame_width,
        frame_height,
        config,
    )
    return AimIntent(
        horizontal_error=horizontal_error,
        vertical_error=vertical_error,
        yaw_rate=yaw_rate,
        pitch_rate=pitch_rate,
        gimbal_pitch_required=not vertically_centered,
    )


def render_evidence_image(request: EvidenceRequest) -> np.ndarray:
    """Annotate a copy while preserving the complete original camera scene."""

    shown = request.image.copy()
    left, top, right, bottom = request.detection.box
    cv2.rectangle(shown, (left, top), (right, bottom), (0, 255, 255), 2)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 2
    (text_width, text_height), _ = cv2.getTextSize(
        request.annotation, font, scale, thickness
    )
    x = max(0, min(shown.shape[1] - text_width, request.text_anchor[0] - text_width // 2))
    y = max(text_height, min(shown.shape[0] - 1, request.text_anchor[1]))
    cv2.putText(shown, request.annotation, (x, y), font, scale, (0, 255, 255), thickness)
    return shown


class NumberMarkerTask:
    """Non-blocking Final number-marker state machine."""

    name = KIND

    def __init__(
        self,
        settings: Optional[NumberMarkerConfig] = None,
        observation_provider: Optional[ObservationProvider] = None,
    ) -> None:
        self.settings = settings or NumberMarkerConfig()
        self._validate_settings()
        self._observation_provider = observation_provider
        self._observation_lock = threading.Lock()
        self._injected_candidates: Tuple[MarkerCandidate, ...] = ()

        self.state = MarkerState.IDLE
        self.last_detection = VisualDetection.no_result(KIND)
        self.last_selection_strategy: Optional[str] = None
        self.last_aim_intent: Optional[AimIntent] = None
        self.aimed_ids = set()
        self.saved_ids = set()

        self._target_id: Optional[str] = None
        self._started_at: Optional[float] = None
        self._lost_since: Optional[float] = None
        self._stable_frames = 0
        self._last_centered_sequence: Optional[int] = None
        self._active_evidence: Optional[EvidenceRequest] = None
        self._queued_evidence: Optional[EvidenceRequest] = None
        self._evidence_outcome: Optional[bool] = None

    def _validate_settings(self) -> None:
        if not 0.0 < self.settings.min_marker_width_ratio < 1.0:
            raise ValueError("min_marker_width_ratio must be between 0 and 1")
        if self.settings.aim_stable_frames < 1:
            raise ValueError("aim_stable_frames must be at least 1")
        if self.settings.max_evidence_attempts < 1:
            raise ValueError("max_evidence_attempts must be at least 1")
        positive = (
            self.settings.center_zone_width_fraction,
            self.settings.center_zone_height_fraction,
            self.settings.max_frame_age,
            self.settings.max_observation_age,
            self.settings.target_lost_timeout,
            self.settings.max_task_seconds,
            self.settings.max_aim_yaw_rate,
            self.settings.max_aim_pitch_rate,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("number-marker timing, tolerance, and rates must be positive")
        finite = (
            self.settings.yaw_kp,
            self.settings.pitch_kp,
            self.settings.yaw_direction_sign,
            self.settings.pitch_direction_sign,
        )
        if any(not math.isfinite(value) for value in finite):
            raise ValueError("number-marker gains and direction signs must be finite")
        if self.settings.yaw_direction_sign == 0.0 or self.settings.pitch_direction_sign == 0.0:
            raise ValueError("number-marker direction signs must be non-zero")

    @property
    def target_id(self) -> Optional[str]:
        return self._target_id

    @property
    def pending_evidence_request(self) -> Optional[EvidenceRequest]:
        """Inspect the queued request without consuming it."""

        with self._observation_lock:
            return self._queued_evidence

    def take_evidence_request(self) -> Optional[EvidenceRequest]:
        """Transfer one request to the integration-owned evidence writer."""

        with self._observation_lock:
            request = self._queued_evidence
            self._queued_evidence = None
            return request

    def acknowledge_evidence(self, request_id: str, saved: bool) -> bool:
        """Record a real writer result; ``saved=False`` never enters saved_ids."""

        with self._observation_lock:
            request = self._active_evidence
            if request is None or request.request_id != request_id:
                return False
            if saved:
                self.saved_ids.add(request.marker_id)
                self._evidence_outcome = True
                return True
            if request.attempt >= self.settings.max_evidence_attempts:
                self._evidence_outcome = False
                return True
            retry = replace(
                request,
                request_id="{}:retry{}".format(request.request_id, request.attempt + 1),
                attempt=request.attempt + 1,
            )
            self._active_evidence = retry
            self._queued_evidence = retry
            self._evidence_outcome = None
            return True

    def update_candidates(self, candidates: Iterable[MarkerCandidate]) -> None:
        """Atomically replace the externally supplied observation snapshot."""

        snapshot = tuple(candidates)
        with self._observation_lock:
            self._injected_candidates = snapshot

    def detect(
        self,
        frame: FramePacket,
        now: float,
        target_id: Optional[str] = None,
    ) -> Optional[TargetSelection]:
        """Evaluate this frame and select one unprocessed valid marker."""

        height, width = frame.image.shape[:2]
        candidates = self._read_candidates(frame, now)
        accepted = evaluate_number_markers(
            candidates,
            width,
            height,
            now,
            frame.sequence,
            aimed_ids=self.aimed_ids if target_id is None else (),
            max_observation_age=self.settings.max_observation_age,
        )
        if target_id is not None:
            accepted = tuple(item for item in accepted if item.target_id == target_id)
        return select_target_marker(accepted)

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        """Run one bounded state transition for the current shared frame."""

        if self.state in (MarkerState.COMPLETED, MarkerState.FAILED):
            self._reset_transient()

        frame_error = self._frame_error(frame, now)
        if frame_error is not None:
            return self._safe_frame_failure(frame_error)

        if self._started_at is not None and now - self._started_at > self.settings.max_task_seconds:
            self.state = MarkerState.FAILED
            return TaskUpdate(
                TaskStatus.FAILED,
                motion=MotionCommand(),
                detection=self.last_detection,
                message="FAILED:TASK_TIMEOUT",
            )

        if self.state is MarkerState.EVIDENCE_PENDING:
            return self._step_evidence_pending()

        try:
            selection = self.detect(frame, now, self._target_id)
        except Exception:
            return self._safe_frame_failure("OBSERVATION_PROVIDER_ERROR")

        if self._target_id is not None:
            if selection is None:
                return self._step_target_lost(now)

        if selection is None:
            self.state = MarkerState.IDLE
            self.last_detection = VisualDetection.no_result(KIND)
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED,
                detection=self.last_detection,
                message="IDLE:NO_VALID_MARKER",
            )

        height, width = frame.image.shape[:2]
        candidate = selection.candidate
        detection = candidate.to_detection(width, height)
        self.last_detection = detection
        self.last_selection_strategy = selection.strategy

        if not is_marker_eligible(candidate, width, self.settings.min_marker_width_ratio):
            self.state = MarkerState.TARGET_TOO_SMALL
            if self._target_id is not None:
                self.state = MarkerState.FAILED
                return TaskUpdate(
                    TaskStatus.FAILED,
                    motion=MotionCommand(),
                    detection=detection,
                    message="FAILED:TARGET_TOO_SMALL",
                )
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED,
                detection=detection,
                message="TARGET_TOO_SMALL",
            )

        if self._target_id is None:
            self.state = MarkerState.TARGET_FOUND
            self._target_id = candidate.target_id
            self._started_at = now
            self._stable_frames = 0
            self._lost_since = None
            self.state = MarkerState.STOPPING
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=None,
                detection=detection,
                message="STOPPING:TARGET_FOUND",
            )

        self._lost_since = None
        self.state = MarkerState.AIMING
        intent = compute_aim_intent(candidate, width, height, self.settings)
        self.last_aim_intent = intent

        if not is_marker_centered(candidate, width, height, self.settings):
            self._stable_frames = 0
            self._last_centered_sequence = None
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=MotionCommand(yaw=intent.yaw_rate),
                detection=detection,
                message=(
                    "AIMING:GIMBAL_PITCH_INTEGRATION_REQUIRED"
                    if intent.gimbal_pitch_required
                    else "AIMING"
                ),
            )

        if self._last_centered_sequence != frame.sequence:
            self._stable_frames += 1
            self._last_centered_sequence = frame.sequence
        if self._stable_frames < self.settings.aim_stable_frames:
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=MotionCommand(),
                detection=detection,
                message="AIMING:STABLE_{}/{}".format(
                    self._stable_frames, self.settings.aim_stable_frames
                ),
            )

        self.state = MarkerState.AIM_LOCKED
        self.aimed_ids.add(candidate.target_id)
        if not self.settings.team_number:
            self.state = MarkerState.FAILED
            return TaskUpdate(
                TaskStatus.FAILED,
                motion=MotionCommand(),
                detection=detection,
                message="FAILED:TEAM_NUMBER_REQUIRED_FOR_EVIDENCE",
            )
        request = self._make_evidence_request(frame, detection)
        with self._observation_lock:
            self._active_evidence = request
            self._queued_evidence = request
            self._evidence_outcome = None
        self.state = MarkerState.EVIDENCE_PENDING
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(),
            detection=detection,
            message="AIM_LOCKED:EVIDENCE_PENDING",
        )

    def _read_candidates(
        self, frame: FramePacket, now: float
    ) -> Tuple[MarkerCandidate, ...]:
        if self._observation_provider is None:
            with self._observation_lock:
                return self._injected_candidates
        return tuple(self._observation_provider(frame, now))

    def _step_target_lost(self, now: float) -> TaskUpdate:
        self.state = MarkerState.LOST
        self._stable_frames = 0
        self._last_centered_sequence = None
        if self._lost_since is None:
            self._lost_since = now
        if now - self._lost_since > self.settings.target_lost_timeout:
            self.state = MarkerState.FAILED
            return TaskUpdate(
                TaskStatus.FAILED,
                motion=MotionCommand(),
                detection=VisualDetection.no_result(KIND),
                message="FAILED:TARGET_LOST_TIMEOUT",
            )
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(),
            detection=VisualDetection.no_result(KIND),
            message="TARGET_LOST:HOLD",
        )

    def _step_evidence_pending(self) -> TaskUpdate:
        with self._observation_lock:
            outcome = self._evidence_outcome
        if outcome is True:
            self.state = MarkerState.COMPLETED
            return TaskUpdate(
                TaskStatus.COMPLETED,
                motion=MotionCommand(),
                detection=self.last_detection,
                message="COMPLETED:EVIDENCE_SAVED",
            )
        if outcome is False:
            self.state = MarkerState.FAILED
            return TaskUpdate(
                TaskStatus.FAILED,
                motion=MotionCommand(),
                detection=self.last_detection,
                message="FAILED:EVIDENCE_WRITE_FAILED",
            )
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(),
            detection=self.last_detection,
            message="EVIDENCE_PENDING:HOLD",
        )

    def _make_evidence_request(
        self, frame: FramePacket, detection: VisualDetection
    ) -> EvidenceRequest:
        marker_id = str(detection.target_id)
        annotation = "Team {} detects a marker with ID of {}".format(
            self.settings.team_number, marker_id
        )
        height, width = frame.image.shape[:2]
        return EvidenceRequest(
            request_id="marker:{}:frame:{}:attempt:1".format(marker_id, frame.sequence),
            marker_id=marker_id,
            frame_sequence=frame.sequence,
            captured_at=frame.captured_at,
            detection=detection,
            annotation=annotation,
            text_anchor=(width // 2, height // 2),
            image=frame.image.copy(),
        )

    def _frame_error(self, frame: FramePacket, now: float) -> Optional[str]:
        if not isinstance(frame, FramePacket):
            return "MISSING_FRAME"
        image = frame.image
        if not isinstance(image, np.ndarray) or image.ndim != 3:
            return "INVALID_FRAME"
        if image.shape[0] <= 0 or image.shape[1] <= 0:
            return "INVALID_FRAME"
        timestamp = _finite(frame.captured_at)
        current = _finite(now)
        if timestamp is None or current is None:
            return "INVALID_FRAME_TIMESTAMP"
        age = current - timestamp
        if age < 0.0 or age > self.settings.max_frame_age:
            return "STALE_FRAME"
        return None

    def _safe_frame_failure(self, reason: str) -> TaskUpdate:
        active = self._target_id is not None
        self.last_aim_intent = None
        if active:
            self.state = MarkerState.FAILED
            return TaskUpdate(
                TaskStatus.FAILED,
                motion=MotionCommand(),
                detection=VisualDetection.no_result(KIND),
                message="FAILED:{}".format(reason),
            )
        self.state = MarkerState.IDLE
        self.last_detection = VisualDetection.no_result(KIND)
        return TaskUpdate(
            TaskStatus.NOT_TRIGGERED,
            detection=self.last_detection,
            message="IDLE:{}".format(reason),
        )

    def _reset_transient(self) -> None:
        self.state = MarkerState.IDLE
        self.last_detection = VisualDetection.no_result(KIND)
        self.last_selection_strategy = None
        self.last_aim_intent = None
        self._target_id = None
        self._started_at = None
        self._lost_since = None
        self._stable_frames = 0
        self._last_centered_sequence = None
        with self._observation_lock:
            self._active_evidence = None
            self._queued_evidence = None
            self._evidence_outcome = None
