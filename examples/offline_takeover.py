"""Non-blocking handoff example. This file never imports the RoboMaster SDK."""

from dataclasses import dataclass
from typing import List

import cv2
import numpy as np

from config import CONFIG
from models import (
    FramePacket,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)
from motion_output import MotionOutput
from runtime import LineFollower


@dataclass
class DemoTask:
    steps: int = 0

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        """Return immediately; a real main loop calls this once per new frame."""
        self.steps += 1
        fake_target = VisualDetection(
            valid=True,
            kind="demo_target",
            center=(frame.image.shape[1] // 2, frame.image.shape[0] // 2),
            target_id="demo-1",
            color="none",
            confidence=1.0,
        )
        if self.steps == 1:
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=MotionCommand(forward=0.0, yaw=12.0),
                detection=fake_target,
                message="one bounded task step",
            )
        return TaskUpdate(
            TaskStatus.COMPLETED,
            detection=fake_target,
            message="demo task complete",
        )


class FakeChassis:
    """Records commands in memory; it has no SDK or hardware connection."""

    def __init__(self) -> None:
        self.calls = []

    def drive_speed(self, **kwargs) -> None:
        self.calls.append(("speed", kwargs))

    def drive_wheels(self, **kwargs) -> None:
        self.calls.append(("wheels", kwargs))


def make_line_frame(x: int = 320) -> np.ndarray:
    image = np.full((360, 640, 3), 210, np.uint8)
    cv2.line(image, (x, 350), (x, 190), (255, 0, 0), 24)
    return image


def run_demo() -> List[str]:
    """Simulate follow -> takeover -> complete -> validate -> resume."""
    follower = LineFollower(CONFIG)
    output = MotionOutput(FakeChassis(), CONFIG)
    trace: List[str] = []

    frame = FramePacket(make_line_frame(), sequence=1, captured_at=1.00)
    follower.process_frame(frame.image, frame.captured_at)
    if not follower.resume(frame.captured_at):
        raise RuntimeError("demo could not start on a valid line")
    tracking = follower.process_frame(make_line_frame(350), 1.05)
    trace.append(f"line:{tracking.state}")

    follower.pause(1.10)
    output.claim("external")
    trace.append(f"owner:{output.owner}")
    task = DemoTask()
    first = task.step(FramePacket(make_line_frame(), 2, 1.10), 1.10)
    if first.motion is not None:
        output.send("external", first.motion)
    trace.append(f"task:{first.status.value}")
    second = task.step(FramePacket(make_line_frame(), 3, 1.15), 1.15)
    trace.append(f"task:{second.status.value}")

    # Stop task motion, return ownership, clear stale history, then use a new
    # frame to validate the line before explicit resume.
    output.hard_stop()
    output.claim("line")
    trace.append(f"owner:{output.owner}")
    follower.reset_fault(1.20)
    fresh = FramePacket(make_line_frame(), 4, 1.21)
    follower.process_frame(fresh.image, fresh.captured_at)
    if not follower.resume(fresh.captured_at):
        raise RuntimeError("demo could not resume on a fresh valid line")
    trace.append(f"line:{follower.state}")
    return trace


if __name__ == "__main__":
    for event in run_demo():
        print(event)
