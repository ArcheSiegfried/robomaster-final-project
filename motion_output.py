"""The only RoboMaster chassis command outlet."""

from config import RuntimeConfig
from models import MotionCommand


class MotionOutput:
    def __init__(self, chassis, settings: RuntimeConfig) -> None:
        self._chassis = chassis
        self._settings = settings
        self._owner = "line"
        self._last_send_accepted = None

    @property
    def owner(self) -> str:
        return self._owner

    @property
    def last_send_accepted(self):
        """Whether the SDK acknowledged the latest speed request.

        ``None`` means no speed request has been made.  A missing synchronous
        acknowledgement is diagnostic information, not a reason to terminate
        the control loop: the next bounded command can still be sent and the
        command timeout remains active.
        """
        return self._last_send_accepted

    def claim(self, owner: str) -> None:
        """Stop before transferring exclusive control to line or external."""
        if owner not in ("line", "external"):
            raise ValueError("owner must be 'line' or 'external'")
        self.hard_stop()
        self._owner = owner

    def send(self, owner: str, command: MotionCommand) -> None:
        if owner != self._owner:
            raise RuntimeError(f"motion outlet belongs to {self._owner}")
        accepted = self._chassis.drive_speed(
            x=command.forward,
            y=command.lateral,
            z=command.yaw,
            timeout=self._settings.command_timeout,
        )
        # Some RoboMaster transports report False for a transient missing
        # synchronous acknowledgement.  Raising here used to tear down the
        # complete program after a single dropped response.  Retain the result
        # for diagnostics while allowing the next bounded control tick.
        self._last_send_accepted = accepted is not False

    def hard_stop(self) -> bool:
        """Immediate stop bypasses controller rate limiting."""
        ok = True
        try:
            self._chassis.drive_speed(x=0, y=0, z=0)
        except Exception:
            ok = False
        try:
            self._chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        except Exception:
            ok = False
        return ok
