"""Mode-aware gimbal outlet for the isolated side-looking route experiment.

The normal entry stays in CHASSIS_LEAD. The SDK's ``moveto`` action uses a
chassis-relative yaw coordinate, so the task's relative yaw is sent unchanged
after switching to FREE. Never add the power-on/ground yaw telemetry to it.
"""

import time
from dataclasses import replace

from gimbal_output import GimbalOutput


ANGLE_MAX_AGE_SECONDS = 0.25
MAX_SIDE_YAW_DEG = 105.0
MAX_STARTING_BODY_YAW_DEG = 12.0


class RouteGimbalOutput(GimbalOutput):
    def __init__(self, robot, settings, robot_module):
        experimental_settings = replace(
            settings, gimbal_yaw_min=-MAX_SIDE_YAW_DEG,
            gimbal_yaw_max=MAX_SIDE_YAW_DEG,
        )
        super().__init__(robot.gimbal, experimental_settings)
        self._robot = robot
        self._robot_module = robot_module
        self._free = False
        self._body_yaw = None
        self._angle_at = None
        self._aim_relative = None
        if robot.gimbal.sub_angle(freq=20, callback=self._on_angle) is not True:
            raise RuntimeError("cannot subscribe to gimbal yaw for route-only test")

    def _on_angle(self, angle_info):
        try:
            yaw = float(angle_info[1])
        except (IndexError, TypeError, ValueError):
            return
        self._body_yaw = yaw
        self._angle_at = time.monotonic()

    def _set_mode(self, mode):
        if self._robot.set_robot_mode(mode=mode) is not True:
            raise RuntimeError("cannot change robot mode for route gimbal")

    def send(self, command):
        # Only the side-looking task requests nonzero yaw. The chassis is
        # stopped by the coordinator before this first task request.
        requested_yaw = float(command.yaw)
        if abs(requested_yaw) > MAX_SIDE_YAW_DEG:
            raise RuntimeError("side-look yaw exceeds chassis-relative limit")
        if abs(float(command.yaw)) > 1.0 and not self._free:
            now = time.monotonic()
            if (
                self._angle_at is None
                or now - self._angle_at > ANGLE_MAX_AGE_SECONDS
            ):
                raise RuntimeError("no fresh chassis-relative gimbal yaw before side look")
            if abs(self._body_yaw) > MAX_STARTING_BODY_YAW_DEG:
                raise RuntimeError("gimbal is not facing chassis before side look")
            self._set_mode(self._robot_module.FREE)
            self._free = True
            self._aim_relative = requested_yaw
        elif abs(float(command.yaw)) <= 1.0 and self._free:
            self._set_mode(self._robot_module.CHASSIS_LEAD)
            self._free = False
            self._aim_relative = None
        if self._free:
            if abs(float(command.yaw) - self._aim_relative) > 0.1:
                raise RuntimeError("route-only gimbal aim may be set only once")
        return super().send(command)

    def restore_line_view(self):
        if self._free:
            self._set_mode(self._robot_module.CHASSIS_LEAD)
            self._free = False
            self._aim_relative = None
        return super().restore_line_view()

    def close(self):
        self._robot.gimbal.unsub_angle()
