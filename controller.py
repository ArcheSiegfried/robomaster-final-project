"""Low-speed PD controller with bounded output and bounded loss coasting."""

import time
from typing import Optional

from config import ControlConfig
from models import MotionCommand, STOP_COMMAND


class LineController:
    def __init__(self, settings: ControlConfig) -> None:
        self.settings = settings
        self.reset()

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))

    @staticmethod
    def _approach(current: float, target: float, rate: float, dt: float) -> float:
        change = max(0.0, rate) * max(0.0, dt)
        return current + max(-change, min(target - current, change))

    def reset(self, timestamp: Optional[float] = None) -> None:
        self._last_time = time.monotonic() if timestamp is None else timestamp
        self._last_error: Optional[float] = None
        self._derivative = 0.0
        self._command = STOP_COMMAND
        self._recovery_started: Optional[float] = None

    def stop(self, timestamp: Optional[float] = None) -> MotionCommand:
        self.reset(timestamp)
        return STOP_COMMAND

    def track(
        self,
        error: float,
        heading: float,
        timestamp: float,
        recovering: bool = False,
    ) -> MotionCommand:
        dt = max(0.001, min(timestamp - self._last_time, 0.10))
        error = self._clamp(
            error, -self.settings.max_error, self.settings.max_error
        )
        raw_derivative = (
            0.0
            if self._last_error is None
            else (error - self._last_error) / dt
        )
        raw_derivative = self._clamp(
            raw_derivative,
            -self.settings.max_derivative,
            self.settings.max_derivative,
        )
        alpha = self.settings.derivative_alpha
        self._derivative = (
            (1.0 - alpha) * self._derivative + alpha * raw_derivative
        )
        steering_error = (
            0.0 if abs(error) < self.settings.error_deadband else error
        )
        target_yaw = (
            self.settings.kp * steering_error
            + self.settings.kd * self._derivative
            + self.settings.heading_gain * heading
        )
        target_yaw = self._clamp(
            target_yaw,
            -self.settings.max_yaw_speed,
            self.settings.max_yaw_speed,
        )
        curved = max(abs(error), abs(heading)) >= self.settings.curve_threshold
        target_forward = (
            self.settings.curve_speed
            if curved
            else self.settings.forward_speed
        )

        if recovering and self._recovery_started is None:
            self._recovery_started = timestamp
        if self._recovery_started is not None:
            elapsed = timestamp - self._recovery_started
            ratio = self._clamp(
                elapsed / max(self.settings.recovery_ramp_seconds, 0.001),
                0.0,
                1.0,
            )
            target_forward = self._command.forward + (
                target_forward - self._command.forward
            ) * ratio
            target_yaw = self._command.yaw + (
                target_yaw - self._command.yaw
            ) * ratio
            if ratio >= 1.0:
                self._recovery_started = None

        forward_rate = (
            self.settings.max_forward_acceleration
            if target_forward >= self._command.forward
            else self.settings.max_forward_deceleration
        )
        forward = self._approach(
            self._command.forward, target_forward, forward_rate, dt
        )
        yaw = self._approach(
            self._command.yaw,
            target_yaw,
            self.settings.max_yaw_rate,
            dt,
        )
        self._last_time = timestamp
        self._last_error = error
        self._command = MotionCommand(forward, 0.0, yaw)
        return self._command

    def coast(self, lost_elapsed: float, timestamp: float) -> MotionCommand:
        """Brief image miss only: never accelerates; yaw fades to zero."""
        dt = max(0.001, min(timestamp - self._last_time, 0.10))
        ratio = self._clamp(
            lost_elapsed / max(self.settings.lost_grace_seconds, 0.001),
            0.0,
            1.0,
        )
        target_forward = min(
            self._command.forward, self.settings.lost_forward_speed
        )
        target_yaw = self._clamp(
            self._command.yaw * (1.0 - ratio),
            -self.settings.lost_max_yaw,
            self.settings.lost_max_yaw,
        )
        forward = self._approach(
            self._command.forward,
            target_forward,
            self.settings.max_forward_deceleration,
            dt,
        )
        yaw = self._approach(
            self._command.yaw,
            target_yaw,
            self.settings.max_yaw_rate,
            dt,
        )
        self._last_time = timestamp
        self._command = MotionCommand(forward, 0.0, yaw)
        return self._command
