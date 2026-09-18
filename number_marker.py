"""Number-marker filtering, target choice, aiming, and evidence handoff.

The task consumes the repository's shared ``FramePacket`` and returns only the
frozen v0.2 task types. It owns no camera, vision subscription, gimbal, chassis,
or file writer. The integration layer may inject current marker observations
and must consume/acknowledge ``EvidenceRequest`` objects.

All defaults in ``NumberMarkerConfig`` are engineering starting values. They
are not claimed as teacher-specified tolerances or hardware-verified settings.
"""

import json
import math
import sys
import threading
from collections import deque
from dataclasses import dataclass, replace
from enum import Enum
from typing import Callable, FrozenSet, Iterable, Optional, Sequence, Tuple

import numpy as np

from config import CONFIG
from models import (
    FramePacket,
    GimbalCommand,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)

KIND = "number_marker"
VALID_MARKER_IDS: FrozenSet[str] = frozenset(("1", "2", "3", "4", "5"))
SUPPORTED_IDS = tuple(sorted(VALID_MARKER_IDS))

# There is no calibrated marker range or ToF-to-marker association in the
# current integration framework. This explicit name prevents the fallback
# from being mistaken for a real physical-nearest measurement.
NEAREST_PROXY = "NEAREST_PROXY_LARGEST_APPARENT_WIDTH"
MEASURED_DISTANCE = "MEASURED_DISTANCE"

# Engineering controls for integrating vertical image error into an absolute
# gimbal target. The physical sign has not been verified on the course robot.
PITCH_FOLLOW_SIGN = -1.0
MAX_PITCH_INTEGRATION_DT = 0.20


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

    #: **计分门槛**：赛题"只瞄准宽度 > 画面宽 1/5 的标识"才**存图**。
    #: 它管的是存图质量，不是"要不要接管"。
    min_marker_width_ratio: float = 0.20
    #: **检测/接管门槛**（2026-09-18 新增）。
    #:
    #: 为什么必须和上面那个分开：实测（captures/run_20260918_175238）
    #: SDK 报出来的标识框只有画面宽的 **3.4%**（22 像素），而接管门槛是 20%，
    #: 于是模块永远 `TARGET_TOO_SMALL`、永远不接管、车也不会靠近，
    #: 标识永远不会变大 —— **死锁，一分不得**。
    #: 这不是"走慢一点"能解决的：要靠得近约 6 倍才够 20%。
    #:
    #: 分开之后：**看到**标识就允许接管去瞄准/靠近（本门槛），
    #: "够不够大、要不要真的存图"仍由 `min_marker_width_ratio` 把关。
    #: 默认取实测值（0.03）再留一点余量。
    trigger_min_marker_width_ratio: float = 0.03
    tracking_min_marker_width_ratio: float = 0.02
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
    # More-negative pitch normally points the camera lower, so this engineering
    # default maps a marker above center toward a more-positive target. Flip
    # only this sign if the real robot behaves oppositely.
    pitch_direction_sign: float = PITCH_FOLLOW_SIGN
    max_pitch_integration_dt: float = MAX_PITCH_INTEGRATION_DT
    max_evidence_attempts: int = 1
    team_number: Optional[str] = "10"


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


class NumberMarkerTask:
    """Non-blocking Final number-marker state machine."""

    name = KIND

    def __init__(
        self,
        settings: Optional[NumberMarkerConfig] = None,
        observation_provider: Optional[ObservationProvider] = None,
        runtime_settings: Optional[object] = None,
    ) -> None:
        self.settings = settings or NumberMarkerConfig()
        self.runtime_settings = CONFIG if runtime_settings is None else runtime_settings
        self._line_pitch = float(
            getattr(self.runtime_settings, "gimbal_pitch", CONFIG.gimbal_pitch)
        )
        self._gimbal_yaw = float(
            getattr(self.runtime_settings, "gimbal_yaw", CONFIG.gimbal_yaw)
        )
        self._pitch_min = float(
            getattr(self.runtime_settings, "gimbal_pitch_min", CONFIG.gimbal_pitch_min)
        )
        self._pitch_max = float(
            getattr(self.runtime_settings, "gimbal_pitch_max", CONFIG.gimbal_pitch_max)
        )
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
        self._locked_target_width_ratio: Optional[float] = None
        self._tracking_below_trigger = False
        self._recent_observations = deque(maxlen=40)
        self._diagnostic_events = deque(maxlen=64)
        self._diagnostic_centered: Optional[bool] = None
        self._last_raw_target: Optional[MarkerCandidate] = None
        self._started_at: Optional[float] = None
        self._lost_since: Optional[float] = None
        self._stable_frames = 0
        self._last_centered_sequence: Optional[int] = None
        self._target_pitch: Optional[float] = None
        self._last_step_at: Optional[float] = None
        self._active_evidence: Optional[EvidenceRequest] = None
        self._queued_evidence: Optional[EvidenceRequest] = None
        self._evidence_outcome: Optional[bool] = None

    def _validate_settings(self) -> None:
        if not 0.0 < self.settings.min_marker_width_ratio < 1.0:
            raise ValueError("min_marker_width_ratio must be between 0 and 1")
        if not 0.0 < self.settings.trigger_min_marker_width_ratio < 1.0:
            raise ValueError("trigger_min_marker_width_ratio must be between 0 and 1")
        # 接管门槛**不得高于**计分门槛：否则就是"看到标识但够不着门槛 → 永远不接管
        # → 车不靠近 → 标识永远不变大"的死锁（实车 3.4% vs 20%）。
        if self.settings.trigger_min_marker_width_ratio > self.settings.min_marker_width_ratio:
            raise ValueError(
                "trigger_min_marker_width_ratio must not exceed min_marker_width_ratio"
            )
        if not 0.0 < self.settings.tracking_min_marker_width_ratio < self.settings.trigger_min_marker_width_ratio:
            raise ValueError("tracking_min_marker_width_ratio must be below the trigger threshold")
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
            self.settings.max_pitch_integration_dt,
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
        runtime_values = (
            self._line_pitch,
            self._gimbal_yaw,
            self._pitch_min,
            self._pitch_max,
        )
        if any(not math.isfinite(value) for value in runtime_values):
            raise ValueError("configured gimbal view and limits must be finite")
        if self._pitch_min > self._pitch_max:
            raise ValueError("gimbal_pitch_min must not exceed gimbal_pitch_max")
        self._line_pitch = self._clamp_pitch(self._line_pitch)

    @property
    def target_id(self) -> Optional[str]:
        return self._target_id

    @property
    def diagnostic_events(self) -> Tuple[dict, ...]:
        """Bounded module-owned event history; the runtime can inspect it safely."""

        return tuple(dict(event) for event in self._diagnostic_events)

    @property
    def target_pitch(self) -> Optional[float]:
        """Current absolute pitch request, exposed for diagnostics and tests."""

        return self._target_pitch

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
            if (
                request is None
                or request.request_id != request_id
                or self._evidence_outcome is not None
            ):
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
        raw_matches = (
            tuple(item for item in candidates
                  if isinstance(item, MarkerCandidate) and item.target_id == target_id)
            if target_id is not None else ()
        )
        self._last_raw_target = (
            max(raw_matches, key=lambda item: _finite(item.width) or 0.0)
            if raw_matches else None
        )
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

    def _note_observation(
        self, event: str, frame: FramePacket, now: float,
        candidate: Optional[MarkerCandidate], *, emit: bool = False,
        include_recent: bool = False, phase: Optional[str] = None,
        yaw_command: Optional[float] = None, reset_reason: Optional[str] = None,
    ) -> None:
        """Record bounded observations; diagnostic errors cannot alter task control."""

        try:
            height, width = frame.image.shape[:2]
            observed_at = None if candidate is None else _finite(candidate.observed_at)
            age = None if observed_at is None else now - observed_at
            current_width = None if candidate is None else _finite(candidate.width)
            ratio = None if current_width is None or width <= 0 else current_width / width
            x = None if candidate is None else _finite(candidate.center[0])
            y = None if candidate is None else _finite(candidate.center[1])
            try:
                source_sequence = (
                    None if candidate is None or candidate.source_sequence is None
                    else int(candidate.source_sequence)
                )
            except (TypeError, ValueError):
                source_sequence = None
            error_x = None if x is None else x - width / 2.0
            error_y = None if y is None else y - height / 2.0
            centered_x = (
                None if error_x is None or candidate is None
                else abs(error_x) <= candidate.width * self.settings.center_zone_width_fraction / 2.0
            )
            centered_y = (
                None if error_y is None or candidate is None
                else abs(error_y) <= candidate.height * self.settings.center_zone_height_fraction / 2.0
            )
            entry = {
                "event": event,
                "timestamp_monotonic_s": round(now, 6),
                "frame_sequence": frame.sequence,
                "target_id": self._target_id,
                "locked_target_id": self._target_id,
                "locked_target_width_ratio": self._locked_target_width_ratio,
                "candidate_id": None if candidate is None else str(candidate.target_id),
                "candidate_source_sequence": source_sequence,
                "current_target_width_ratio": ratio,
                "x_px": x,
                "y_px": y,
                "target_center_x": x,
                "target_center_y": y,
                "frame_center_x": width / 2.0,
                "frame_center_y": height / 2.0,
                "frame_center_x_px": width / 2.0,
                "frame_center_y_px": height / 2.0,
                "x_error_px": error_x,
                "y_error_px": error_y,
                "centered_x": centered_x,
                "centered_y": centered_y,
                "centered": None if centered_x is None or centered_y is None else centered_x and centered_y,
                "stable_frames": self._stable_frames,
                "observation_age": age,
                "target_lost_age": None if self._lost_since is None else now - self._lost_since,
                "tracking_below_trigger": self._tracking_below_trigger,
                "task_state": self.state.value,
                "task_phase": phase,
                "phase": phase,
                "yaw_command": yaw_command,
                "pitch_target": self._target_pitch,
                "task_elapsed_s": None if self._started_at is None else now - self._started_at,
                "task_timeout_remaining_s": (
                    None if self._started_at is None
                    else self.settings.max_task_seconds - (now - self._started_at)
                ),
            }
            if reset_reason is not None:
                entry["reset_reason"] = reset_reason
            if yaw_command is None and phase == "AIMING:TRACKING_BELOW_TRIGGER":
                entry["yaw_command"] = (
                    0.0 if entry["centered"] else self.last_aim_intent.yaw_rate
                )
            if self._target_id is not None:
                if self._recent_observations and self._recent_observations[-1]["frame_sequence"] == frame.sequence:
                    self._recent_observations[-1] = entry
                else:
                    self._recent_observations.append(entry)
            if not emit:
                return
            if include_recent:
                horizon = 1.5 if event == "TASK_TIMEOUT" else 0.5
                recent = [item for item in self._recent_observations
                          if now - item["timestamp_monotonic_s"] <= horizon]
                entry = dict(entry, recent_observations=recent)
                if event == "TASK_TIMEOUT":
                    entry["recent_trace"] = recent
                    entry["last_observation"] = recent[-2] if len(recent) > 1 else None
            self._diagnostic_events.append(entry)
            sys.stderr.write("NUMBER_MARKER_DIAG " + json.dumps(entry, sort_keys=True) + "\n")
        except Exception:
            pass  # Diagnostics must never interrupt motion safety.

    def _note_center_transition(
        self, frame: FramePacket, now: float, candidate: MarkerCandidate,
    ) -> None:
        """Emit center changes only; this has no role in the aiming decision."""

        try:
            height, width = frame.image.shape[:2]
            centered = is_marker_centered(candidate, width, height, self.settings)
            if self._diagnostic_centered is not None and centered != self._diagnostic_centered:
                self._note_observation(
                    "CENTER_ENTER" if centered else "CENTER_EXIT",
                    frame, now, candidate, emit=True,
                )
            self._diagnostic_centered = centered
        except Exception:
            pass

    def _clamp_pitch(self, pitch: float) -> float:
        return max(self._pitch_min, min(float(pitch), self._pitch_max))

    def _begin_pitch_tracking(self, now: float) -> None:
        """Start this marker from the configured line-view pitch."""

        self._target_pitch = self._line_pitch
        self._last_step_at = now

    def _gimbal_request(self) -> GimbalCommand:
        pitch = self._line_pitch if self._target_pitch is None else self._target_pitch
        return GimbalCommand(pitch=pitch, yaw=self._gimbal_yaw)

    def _integrate_pitch(
        self, intent: AimIntent, now: float
    ) -> Optional[GimbalCommand]:
        """Integrate a rate intent into the v0.2 absolute-pitch contract."""

        if self._target_pitch is None or self._last_step_at is None:
            self._begin_pitch_tracking(now)
        elapsed = max(0.0, now - float(self._last_step_at))
        dt = min(elapsed, self.settings.max_pitch_integration_dt)
        # Advance the time origin even when already centered, otherwise a later
        # error would integrate time spent centered and cause a sudden jump.
        self._last_step_at = max(float(self._last_step_at), now)
        if not intent.gimbal_pitch_required or dt <= 0.0:
            return None
        self._target_pitch = self._clamp_pitch(
            float(self._target_pitch) + intent.pitch_rate * dt
        )
        return self._gimbal_request()

    def _reset_pitch_tracking(self) -> None:
        self._target_pitch = None
        self._last_step_at = None

    def _clear_terminal_transients(self) -> None:
        """End this task's pitch/evidence state without touching coordinator ownership."""

        self._reset_pitch_tracking()
        with self._observation_lock:
            self._active_evidence = None
            self._queued_evidence = None
            self._evidence_outcome = None

    def wants_control(self, frame: FramePacket, now: float) -> bool:
        """只读体检：这一帧有没有"够大且没拍过"的标识，值不值得拿运动权。

        这是给协调器的**预约通道**用的（见 `coordinator.RESERVATION_HOOK`）。
        为什么需要它：仲裁是"按顺序问、遇到第一个 RUNNING 就停"，本模块排最后，
        2026-09-18 实车 196 秒里被截断 9 次、**接管 0 次、一张标识照片都没存**
        （标识 5 分/个、满 25 分）。只调注册表顺序救不了：提前就会挡住岔路/绕障。

        严格与 `step()` 的接管判据一致（`is_marker_eligible`，即赛题要求的
        ``width / frame_width > 0.20``）；已经锁定了目标时返回 False，
        避免抢走正在收尾的拍照流程。

        **必须只读且快**：协调器每帧都会调用它。这里的副作用只有
        `_last_raw_target`（`detect()` 本来就会写），不改变状态机。
        """
        if self._target_id is not None:
            return False
        try:
            selection = self.detect(frame, now, None)
        except Exception:
            return False
        if selection is None:
            return False
        try:
            height, width = frame.image.shape[:2]
        except Exception:
            return False
        return is_marker_eligible(
            selection.candidate, width,
            self.settings.trigger_min_marker_width_ratio,
        )

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        """Run one bounded state transition for the current shared frame."""

        if self.state in (MarkerState.COMPLETED, MarkerState.FAILED):
            self._reset_transient()

        frame_error = self._frame_error(frame, now)
        if frame_error is not None:
            return self._safe_frame_failure(frame_error)

        if self._started_at is not None and now - self._started_at > self.settings.max_task_seconds:
            self.state = MarkerState.FAILED
            self._note_observation(
                "TASK_TIMEOUT", frame, now, self._last_raw_target,
                emit=True, include_recent=True, phase="FAILED:TASK_TIMEOUT",
                yaw_command=0.0,
            )
            self._clear_terminal_transients()
            return TaskUpdate(
                TaskStatus.FAILED,
                motion=MotionCommand(),
                detection=self.last_detection,
                message="FAILED:TASK_TIMEOUT",
            )

        if self.state is MarkerState.EVIDENCE_PENDING:
            update = self._step_evidence_pending()
            self._note_observation(
                "FRAME_TRACE", frame, now, None,
                phase=update.message, yaw_command=0.0,
            )
            return update

        try:
            selection = self.detect(frame, now, self._target_id)
        except Exception:
            return self._safe_frame_failure("OBSERVATION_PROVIDER_ERROR")

        if self._target_id is not None:
            if selection is None:
                return self._step_target_lost(frame, now, self._last_raw_target)

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
        ratio = candidate.width_ratio(width)

        if self._target_id is not None:
            if ratio < self.settings.tracking_min_marker_width_ratio:
                return self._step_target_lost(frame, now, candidate)
            # "恢复到可追踪"看的是**检测门槛**（够得着就继续跟），不是计分门槛。
            if (ratio > self.settings.trigger_min_marker_width_ratio
                    and self._tracking_below_trigger):
                self._tracking_below_trigger = False
                self._note_observation("TRACKING_RECOVERED", frame, now, candidate, emit=True)

        # 接管用 trigger_*（看到就接管去瞄准）；min_* 只判"够不够格存图"。
        if self._target_id is None and not is_marker_eligible(
            candidate, width, self.settings.trigger_min_marker_width_ratio
        ):
            self.state = MarkerState.TARGET_TOO_SMALL
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED,
                detection=detection,
                message="TARGET_TOO_SMALL",
            )

        if self._target_id is None:
            self.state = MarkerState.TARGET_FOUND
            self._target_id = candidate.target_id
            self._locked_target_width_ratio = ratio
            self._tracking_below_trigger = False
            try:
                self._recent_observations.clear()
                self._diagnostic_centered = is_marker_centered(
                    candidate, width, height, self.settings,
                )
            except Exception:
                pass
            self._started_at = now
            self._stable_frames = 0
            self._lost_since = None
            self._begin_pitch_tracking(now)
            self.state = MarkerState.STOPPING
            self._note_observation(
                "TARGET_LOCKED", frame, now, candidate, emit=True,
                phase="STOPPING:TARGET_FOUND",
            )
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
        gimbal = self._integrate_pitch(intent, now)
        self._note_center_transition(frame, now, candidate)

        if ratio <= self.settings.min_marker_width_ratio:
            previous_stable_frames = self._stable_frames
            self._stable_frames = 0
            self._last_centered_sequence = None
            if previous_stable_frames:
                self._note_observation(
                    "STABLE_FRAMES_RESET", frame, now, candidate, emit=True,
                    reset_reason="WIDTH_BELOW_EVIDENCE_GATE",
                    phase="AIMING:TRACKING_BELOW_TRIGGER",
                )
            if not self._tracking_below_trigger:
                self._tracking_below_trigger = True
                self._note_observation("TRACKING_BELOW_TRIGGER", frame, now, candidate, emit=True)
            self._note_observation(
                "FRAME_TRACE", frame, now, candidate,
                phase="AIMING:TRACKING_BELOW_TRIGGER",
            )
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=MotionCommand(
                    yaw=0.0 if is_marker_centered(candidate, width, height, self.settings)
                    else intent.yaw_rate
                ),
                gimbal=gimbal,
                detection=detection,
                message="AIMING:TRACKING_BELOW_TRIGGER",
            )

        if not is_marker_centered(candidate, width, height, self.settings):
            previous_stable_frames = self._stable_frames
            self._stable_frames = 0
            self._last_centered_sequence = None
            if previous_stable_frames:
                self._note_observation(
                    "STABLE_FRAMES_RESET", frame, now, candidate, emit=True,
                    reset_reason="CENTER_LOST", phase="AIMING",
                    yaw_command=intent.yaw_rate,
                )
            self._note_observation(
                "FRAME_TRACE", frame, now, candidate,
                phase="AIMING", yaw_command=intent.yaw_rate,
            )
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=MotionCommand(yaw=intent.yaw_rate),
                gimbal=gimbal,
                detection=detection,
                message="AIMING",
            )

        if self._last_centered_sequence != frame.sequence:
            self._stable_frames += 1
            self._last_centered_sequence = frame.sequence
            self._note_observation(
                "STABLE_FRAME_INCREMENT", frame, now, candidate, emit=True,
                phase="AIMING:STABLE_{}/{}".format(
                    self._stable_frames, self.settings.aim_stable_frames,
                ), yaw_command=0.0,
            )
        if self._stable_frames < self.settings.aim_stable_frames:
            self._note_observation(
                "FRAME_TRACE", frame, now, candidate,
                phase="AIMING:STABLE_{}/{}".format(
                    self._stable_frames, self.settings.aim_stable_frames,
                ), yaw_command=0.0,
            )
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=MotionCommand(),
                gimbal=gimbal,
                detection=detection,
                message="AIMING:STABLE_{}/{}".format(
                    self._stable_frames, self.settings.aim_stable_frames
                ),
            )

        self.state = MarkerState.AIM_LOCKED
        self._note_observation(
            "EVIDENCE_READY", frame, now, candidate, emit=True,
            phase="AIM_LOCKED", yaw_command=0.0,
        )
        self.aimed_ids.add(candidate.target_id)
        if not self.settings.team_number:
            self.state = MarkerState.FAILED
            self._clear_terminal_transients()
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
        self._note_observation(
            "EVIDENCE_REQUESTED", frame, now, candidate, emit=True,
            phase="AIM_LOCKED:EVIDENCE_PENDING", yaw_command=0.0,
        )
        self._note_observation(
            "FRAME_TRACE", frame, now, candidate,
            phase="AIM_LOCKED:EVIDENCE_PENDING", yaw_command=0.0,
        )
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(),
            gimbal=gimbal,
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

    def _step_target_lost(
        self, frame: FramePacket, now: float,
        candidate: Optional[MarkerCandidate] = None,
    ) -> TaskUpdate:
        self.state = MarkerState.LOST
        previous_stable_frames = self._stable_frames
        self._stable_frames = 0
        self._last_centered_sequence = None
        self._tracking_below_trigger = True
        if previous_stable_frames:
            self._note_observation(
                "STABLE_FRAMES_RESET", frame, now, candidate, emit=True,
                reset_reason="TARGET_LOST", phase="TARGET_LOST:HOLD",
                yaw_command=0.0,
            )
        # Keep the last absolute target for a brief reacquisition, but move the
        # integration clock forward so lost time can never become pitch motion.
        if self._last_step_at is not None:
            self._last_step_at = max(self._last_step_at, now)
        first_miss = self._lost_since is None
        if first_miss:
            self._lost_since = now
        self._note_observation(
            "TARGET_LOST_GRACE" if first_miss else "TARGET_LOST_HOLD",
            frame, now, candidate, emit=first_miss,
            phase="TARGET_LOST:HOLD", yaw_command=0.0,
        )
        self._note_observation(
            "FRAME_TRACE", frame, now, candidate,
            phase="TARGET_LOST:HOLD", yaw_command=0.0,
        )
        if now - self._lost_since > self.settings.target_lost_timeout:
            self._note_observation(
                "TARGET_LOST_TIMEOUT", frame, now, candidate,
                emit=True, include_recent=True,
            )
            self.state = MarkerState.FAILED
            self._clear_terminal_transients()
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
            self._clear_terminal_transients()
            return TaskUpdate(
                TaskStatus.COMPLETED,
                motion=MotionCommand(),
                detection=self.last_detection,
                message="COMPLETED:EVIDENCE_SAVED",
            )
        if outcome is False:
            self.state = MarkerState.FAILED
            self._clear_terminal_transients()
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
            self._clear_terminal_transients()
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
        self._locked_target_width_ratio = None
        self._tracking_below_trigger = False
        self._last_raw_target = None
        try:
            self._recent_observations.clear()
            self._diagnostic_centered = None
        except Exception:
            pass
        self._started_at = None
        self._lost_since = None
        self._stable_frames = 0
        self._last_centered_sequence = None
        self._clear_terminal_transients()
