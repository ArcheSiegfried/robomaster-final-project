"""Centralized, non-blocking RoboMaster gimbal command outlet."""

import math
from typing import Optional

from config import RuntimeConfig
from models import GimbalCommand


def _finite(value: float, fallback: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(fallback)
    return number if math.isfinite(number) else float(fallback)


class GimbalOutput:
    """Serialize absolute moves; a busy outlet retains only the latest target."""

    MAX_ACTION_RETRIES = 2  # Initial attempt plus at most two failed-action retries.
    _FAILED_STATES = frozenset(
        ("action_failed", "action_rejected", "action_exception", "action_aborted")
    )
    _COMPLETE_STATES = frozenset(("action_succeeded",)) | _FAILED_STATES

    def __init__(self, gimbal, settings: RuntimeConfig) -> None:
        self._gimbal = gimbal
        self._settings = settings
        self._last_command: Optional[GimbalCommand] = None
        self._last_action = None
        self._active_command: Optional[GimbalCommand] = None
        self._pending: Optional[GimbalCommand] = None
        self._settled_command: Optional[GimbalCommand] = None
        self._retry_command: Optional[GimbalCommand] = None
        self._retry_count = 0
        self._exhausted_command: Optional[GimbalCommand] = None
        self._last_error: Optional[str] = None
        self._last_state: Optional[str] = None
        self._counts = dict(
            queued=0, started=0, rejected_by_inflight=0, action_failures=0
        )

    @property
    def last_command(self) -> Optional[GimbalCommand]:
        return self._last_command

    @property
    def last_action_state(self) -> Optional[str]:
        if self._last_action is not None:
            return getattr(self._last_action, "state", self._last_state)
        return self._last_state

    @property
    def busy(self) -> bool:
        return self._last_action is not None

    @property
    def pending(self) -> Optional[GimbalCommand]:
        return self._pending

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    @property
    def at_line_view(self) -> bool:
        return (
            not self.busy
            and self._pending is None
            and self._settled_command == self.line_view
        )

    @property
    def line_view(self) -> GimbalCommand:
        return GimbalCommand(
            pitch=float(self._settings.gimbal_pitch),
            yaw=float(self._settings.gimbal_yaw),
        )

    def stats(self) -> dict:
        """Return a snapshot; callers cannot mutate the outlet's counters."""
        return dict(
            self._counts,
            last_state=self.last_action_state,
            busy=self.busy,
            pending=self._pending,
            last_error=self._last_error,
            at_line_view=self.at_line_view,
        )

    def _clamp(self, command: GimbalCommand) -> GimbalCommand:
        pitch = _finite(command.pitch, self._settings.gimbal_pitch)
        yaw = _finite(command.yaw, self._settings.gimbal_yaw)
        return GimbalCommand(
            pitch=max(
                self._settings.gimbal_pitch_min,
                min(pitch, self._settings.gimbal_pitch_max),
            ),
            yaw=max(
                self._settings.gimbal_yaw_min,
                min(yaw, self._settings.gimbal_yaw_max),
            ),
        )

    def _queue(self, command: GimbalCommand) -> None:
        if command != self._pending:
            self._pending = command
            self._counts["queued"] += 1

    def _failure(self, command: GimbalCommand, reason: str) -> None:
        self._last_error = reason
        self._counts["action_failures"] += 1
        if command != self._retry_command:
            self._retry_command = command
            self._retry_count = 0
        self._retry_count += 1
        if self._pending is not None and self._pending != command:
            return  # A newer target takes priority over retrying an obsolete one.
        if self._retry_count <= self.MAX_ACTION_RETRIES:
            self._queue(command)
        else:
            self._pending = None
            self._exhausted_command = command

    def _start(self, command: GimbalCommand) -> None:
        if command == self._exhausted_command:
            return
        if command != self._retry_command:
            self._retry_command = command
            self._retry_count = 0
            self._exhausted_command = None
        # An attempted move may have changed the physical view even if it fails.
        self._settled_command = None
        try:
            action = self._gimbal.moveto(
                pitch=command.pitch,
                yaw=command.yaw,
                pitch_speed=self._settings.gimbal_pitch_speed,
                yaw_speed=self._settings.gimbal_yaw_speed,
            )
            if action is None:
                raise RuntimeError("gimbal moveto did not return an action")
        except Exception as error:
            reason = str(error)
            if "Robot is already performing" in reason:
                self._last_error = reason
                self._last_state = "action_rejected_by_inflight"
                self._counts["rejected_by_inflight"] += 1
                self._queue(command)  # Retry on a later poll, never in this call.
            else:
                self._last_state = "action_exception"
                self._failure(command, reason)
            return
        self._last_action = action
        self._active_command = command
        self._last_command = command
        self._last_state = getattr(action, "state", None) or "action_started"
        self._counts["started"] += 1

    def poll(self) -> None:
        """Harvest one action and, if idle, start at most one queued target."""
        if self._last_action is not None:
            action = self._last_action
            command = self._active_command
            try:
                state = getattr(action, "state", None)
                failed = (
                    bool(getattr(action, "has_failed", False))
                    or state in self._FAILED_STATES
                )
                # A legacy fake action with no lifecycle fields is already done.
                completed = bool(getattr(action, "is_completed", state is None))
                completed = completed or state in self._COMPLETE_STATES or failed
                reason = getattr(action, "failure_reason", None)
            except Exception as error:
                state, failed, completed, reason = (
                    "action_exception", True, True, str(error)
                )
            self._last_state = state or (
                "action_succeeded" if completed else "action_started"
            )
            if not completed:
                return
            self._last_action = None
            self._active_command = None
            if failed:
                self._failure(command, str(reason or state or "gimbal action failed"))
            else:
                self._settled_command = command
                if command == self._retry_command:
                    self._retry_count = 0
                if command == self._exhausted_command:
                    self._exhausted_command = None
        if self._pending is not None:
            command, self._pending = self._pending, None
            self._start(command)

    def send(self, command: GimbalCommand) -> GimbalCommand:
        """Request an absolute view without waiting or overlapping SDK actions."""
        applied = self._clamp(command)
        if self.busy:
            if applied == self._active_command:
                self._pending = None  # The newest intent cancels an older queued target.
            else:
                self._queue(applied)
        elif self._pending is not None:
            self._queue(applied)
        self.poll()
        if not self.busy and self._pending is None and (
            applied != self._settled_command and applied != self._exhausted_command
        ):
            self._start(applied)
        return applied

    def restore_line_view(self) -> GimbalCommand:
        return self.send(self.line_view)
