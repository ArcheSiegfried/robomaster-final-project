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
    """Apply absolute view requests without blocking the control loop."""

    def __init__(self, gimbal, settings: RuntimeConfig) -> None:
        self._gimbal = gimbal
        self._settings = settings
        self._last_command: Optional[GimbalCommand] = None
        self._last_action = None

    @property
    def last_command(self) -> Optional[GimbalCommand]:
        return self._last_command

    @property
    def last_action_state(self) -> Optional[str]:
        return getattr(self._last_action, "state", None)

    @property
    def line_view(self) -> GimbalCommand:
        return GimbalCommand(
            pitch=float(self._settings.gimbal_pitch),
            yaw=float(self._settings.gimbal_yaw),
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

    def send(self, command: GimbalCommand) -> GimbalCommand:
        """Start one absolute movement and return immediately.

        Identical requests are deduplicated so a per-frame task cannot restart
        the same SDK action continuously.
        """
        applied = self._clamp(command)
        if applied == self._last_command:
            if self.last_action_state in (
                "action_failed",
                "action_rejected",
                "action_exception",
            ):
                raise RuntimeError(
                    "previous gimbal action ended as %s" % self.last_action_state
                )
            return applied
        action = self._gimbal.moveto(
            pitch=applied.pitch,
            yaw=applied.yaw,
            pitch_speed=self._settings.gimbal_pitch_speed,
            yaw_speed=self._settings.gimbal_yaw_speed,
        )
        if action is None:
            raise RuntimeError("gimbal moveto did not return an action")
        self._last_action = action
        self._last_command = applied
        return applied

    def restore_line_view(self) -> GimbalCommand:
        return self.send(self.line_view)
