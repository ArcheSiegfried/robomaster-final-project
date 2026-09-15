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
class TaskConfig:
    """Safety envelope for external task takeover, not per-module tuning.

    These bounds are enforced by coordinator.TaskCoordinator, so a module bug
    cannot command more than these limits and cannot hold control forever.
    A module may keep its own smaller limits inside its own file.
    """

    # A single continuous takeover longer than this is aborted and released.
    max_task_seconds: float = 20.0
    # After a task releases, wait this long for a fresh valid line before
    # giving up and requiring a human SPACE press.
    release_resume_timeout: float = 2.0
    # One step()/observe() call slower than this is recorded as an error.
    max_step_seconds: float = 0.02
    # Hard caps applied to every task MotionCommand before it is sent.
    task_max_forward: float = 0.30
    task_max_lateral: float = 0.25
    task_max_yaw: float = 90.0


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
    gimbal_search_pitch: int = -5
    gimbal_pitch_min: int = -25
    gimbal_pitch_max: int = 10
    gimbal_yaw_min: int = -30
    gimbal_yaw_max: int = 30
    gimbal_pitch_speed: int = 30
    gimbal_yaw_speed: int = 60
    gimbal_settle_seconds: float = 0.45
    # 数字标识的 SDK marker 订阅（见 marker_source.py）。
    # marker_color: SDK 的 marker 颜色过滤器只能设一个。留空 = 不设过滤器。
    #   实车如果一直收不到 marker，依次试 "red" / "green" / "blue"。
    # marker_coordinate_mode: "auto" 自动判断回调坐标是归一化还是像素；
    #   实车第一次跑请核对 marker_source.stats() 的判断结果。
    marker_color: str = ""
    marker_coordinate_mode: str = "auto"
    # 终端状态反馈（丢线、任务接管、异常、心跳）。False = 完全不打印。
    console_status: bool = True
    #: 心跳行间隔（秒）。只影响"还活着"那行的频率，不影响状态变化行。
    console_heartbeat_seconds: float = 2.0
    display: bool = True
    vision: VisionConfig = field(default_factory=VisionConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    tasks: TaskConfig = field(default_factory=TaskConfig)

CONFIG = RuntimeConfig()
