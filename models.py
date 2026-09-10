"""Small data contracts shared by vision, control, runtime and future tasks."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
import numpy as np

Point = Tuple[int, int]
Box = Tuple[int, int, int, int]

@dataclass(frozen=True)
class FramePacket:
    image: np.ndarray
    sequence: int
    captured_at: float

@dataclass(frozen=True)
class LineDetection:
    valid: bool
    error: float
    heading: float
    confidence: float
    near_point: Optional[Point]
    far_point: Optional[Point]
    roi: Tuple[int, int, int, int]
    mask: np.ndarray
    contour: Optional[np.ndarray] = None

@dataclass(frozen=True)
class MotionCommand:
    forward: float = 0.0
    lateral: float = 0.0
    yaw: float = 0.0

STOP_COMMAND = MotionCommand()


@dataclass(frozen=True)
class VisualDetection:
    """Common result for task detectors; coordinates are full-frame pixels."""

    valid: bool
    kind: str
    center: Optional[Point] = None
    target_id: Optional[str] = None
    color: Optional[str] = None
    confidence: float = 0.0
    box: Optional[Box] = None

    @classmethod
    def no_result(cls, kind: str) -> "VisualDetection":
        return cls(valid=False, kind=kind)


class TaskStatus(str, Enum):
    NOT_TRIGGERED = "not_triggered"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass(frozen=True)
class TaskUpdate:
    """One non-blocking task step returned to the future coordinator."""

    status: TaskStatus
    motion: Optional[MotionCommand] = None
    detection: Optional[VisualDetection] = None
    message: str = ""

@dataclass(frozen=True)
class RuntimeDecision:
    state: str
    detection: LineDetection
    command: MotionCommand = STOP_COMMAND
    force_stop: bool = False
    message: str = ""
