"""Mode-aware gimbal outlet for the isolated side-looking route experiment.

The normal entry stays in CHASSIS_LEAD. In that mode DJI ignores gimbal yaw,
so a route-only side look must temporarily use FREE. Switching back is part
of restoring the line view, including on task failure and human stop.
"""

import time
from dataclasses import replace

from gimbal_output import GimbalOutput
from models import GimbalCommand


ANGLE_MAX_AGE_SECONDS = 0.25


class RouteGimbalOutput(GimbalOutput):
    def __init__(self, robot, settings, robot_module):
        experimental_settings = replace(
            settings, gimbal_yaw_min=-250, gimbal_yaw_max=250
        )
        super().__init__(robot.gimbal, experimental_settings)
        self._robot = robot
        self._robot_module = robot_module
        self._free = False
        self._ground_yaw = None
        self._angle_at = None
        self._aim_relative = None
        self._aim_absolute = None
        if robot.gimbal.sub_angle(freq=20, callback=self._on_angle) is not True:
            raise RuntimeError("cannot subscribe to gimbal yaw for route-only test")

    def _on_angle(self, angle_info):
        try:
            yaw = float(angle_info[3])
        except (IndexError, TypeError, ValueError):
            return
        self._ground_yaw = yaw
        self._angle_at = time.monotonic()

    def _set_mode(self, mode):
        if self._robot.set_robot_mode(mode=mode) is not True:
            raise RuntimeError("cannot change robot mode for route gimbal")

    def send(self, command):
        # Only the side-looking task requests nonzero yaw. The chassis is
        # stopped by the coordinator before this first task request.
        if abs(float(command.yaw)) > 1.0 and not self._free:
            now = time.monotonic()
            if (
                self._angle_at is None
                or now - self._angle_at > ANGLE_MAX_AGE_SECONDS
            ):
                raise RuntimeError("no fresh gimbal yaw before side look")
            absolute = self._ground_yaw + float(command.yaw)
            if abs(absolute) > 250.0:
                raise RuntimeError("side-look yaw exceeds SDK absolute limit")
            self._set_mode(self._robot_module.FREE)
            self._free = True
            self._aim_relative = float(command.yaw)
            self._aim_absolute = absolute
        elif abs(float(command.yaw)) <= 1.0 and self._free:
            self._set_mode(self._robot_module.CHASSIS_LEAD)
            self._free = False
            self._aim_relative = None
            self._aim_absolute = None
        if self._free:
            if abs(float(command.yaw) - self._aim_relative) > 0.1:
                raise RuntimeError("route-only gimbal aim may be set only once")
            command = GimbalCommand(command.pitch, self._aim_absolute)
        return super().send(command)

    def restore_line_view(self):
        if self._free:
            self._set_mode(self._robot_module.CHASSIS_LEAD)
            self._free = False
            self._aim_relative = None
            self._aim_absolute = None
        return super().restore_line_view()

    def close(self):
        self._robot.gimbal.unsub_angle()
