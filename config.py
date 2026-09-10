"""Central configuration. Initial motion values require real-robot tuning."""

from dataclasses import dataclass, field
from typing import Tuple

HSV = Tuple[int, int, int]

@dataclass(frozen=True)
class VisionConfig:
    # Proven blue-tape default from race-v4; replace for the final-course color.
    hsv_lower: HSV = (95, 80, 60)
    hsv_upper: HSV = (135, 255, 255)
    roi_left: float = 0.06
    roi_top: float = 0.54
    roi_right: float = 0.94
    roi_bottom: float = 0.96
    open_kernel: int = 3
    close_kernel: int = 17
    min_area: float = 80.0
    max_area_ratio: float = 0.55
    min_vertical_coverage: float = 0.32
    min_sample_pixels: int = 10
    near_band: Tuple[float, float] = (0.62, 0.96)
    far_band: Tuple[float, float] = (0.18, 0.54)
    near_weight: float = 0.65
    continuity_weight: float = 1.8
    center_weight: float = 0.8
    vertical_weight: float = 1.2
    area_weight: float = 0.4
    max_continuity_distance: float = 0.48

@dataclass(frozen=True)
class ControlConfig:
    # Conservative starting values only; tune with wheels raised, then on course.
    forward_speed: float = 0.32
    curve_speed: float = 0.22
    kp: float = 115.0
    kd: float = 3.0
    heading_gain: float = 70.0
    derivative_alpha: float = 0.30
    error_deadband: float = 0.015
    max_error: float = 1.2
    max_derivative: float = 2.5
    max_yaw_speed: float = 135.0
    max_yaw_rate: float = 500.0
    max_forward_acceleration: float = 0.45
    max_forward_deceleration: float = 1.20
    curve_threshold: float = 0.20
    lost_grace_seconds: float = 0.28
    lost_forward_speed: float = 0.14
    lost_max_yaw: float = 65.0
    recovery_ramp_seconds: float = 0.20

@dataclass(frozen=True)
class RuntimeConfig:
    camera_resolution: str = "360p"
    camera_strategy: str = "newest"
    camera_read_timeout: float = 0.06
    consumer_wait_timeout: float = 0.012
    video_gap_stop_seconds: float = 0.22
    command_timeout: float = 0.15
    resume_detection_max_age: float = 0.15
    gimbal_pitch: int = -25
    gimbal_yaw: int = 0
    display: bool = True
    vision: VisionConfig = field(default_factory=VisionConfig)
    control: ControlConfig = field(default_factory=ControlConfig)

CONFIG = RuntimeConfig()
