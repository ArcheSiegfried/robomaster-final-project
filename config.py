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
    """巡线控制参数。

    2026-09-15 按"偏了纠不回来 / 看到弯就冲出去"的实车反馈重标了一轮。取值依据：
    我们的巡线与 race-v4 参考实现同源（见 `VisionConfig` 注释），而参考的 KP=310 /
    KD=4.5 / heading 前馈=260 是在 1.50 m/s 下标定的；它还按速度把转向增益乘
    0.75~2.20 倍（以 0.60 m/s 为 1.0 倍）。我们的运行速度 0.32 m/s 低于它的最低档，
    所以取它 ×0.75 这一档：KP 232 / KD 3.4 / heading 195。

    转向上限不能直接抄它的 550：我们速度只有它的 1/5，同样转弯半径需要的 yaw 更小。
    但原来的 135 会先于比例项卡住（误差 0.5 时比例项只要 57.5，连上限一半都不到），
    所以提到 300；变化率 500 提到 1200（它用 2600）。

    速度不追：`forward_speed` 保持 0.32 不变，只把入弯减速做扎实一点。
    这些仍是**实车必须复验**的起点，不是定标结果。底线由
    `tests/test_line_tuning.py` 守住（谁要调回软档，必须先看见它变红）。
    """

    forward_speed: float = 0.32
    curve_speed: float = 0.20
    kp: float = 232.0
    kd: float = 3.4
    heading_gain: float = 195.0
    derivative_alpha: float = 0.30
    error_deadband: float = 0.012
    max_error: float = 1.2
    max_derivative: float = 3.5
    max_yaw_speed: float = 300.0
    max_yaw_rate: float = 1200.0
    max_forward_acceleration: float = 0.80
    max_forward_deceleration: float = 2.00
    curve_threshold: float = 0.15
    lost_grace_seconds: float = 0.30
    lost_forward_speed: float = 0.14
    lost_max_yaw: float = 90.0
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
    # 终端"竞争探测"间隔（秒）：每到这个时间点，协调器在那**一帧**把所有模块都问一遍，
    # 记录各自想不想接管（胜负规则不变，仍是顺序里第一个 RUNNING），好让操作员看到
    # "谁在竞争、最后判给了谁"。设 0 = 关闭（不额外调用任何模块）。
    claim_probe_seconds: float = 3.0
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
    # 等新帧最多等多久。必须 >= 相机一帧的时长（30fps → 33ms），否则正常的帧间隔
    # 会被当成"没有新帧"，主循环就会去调 coordinator.video_gap()。
    # 实车教训：原来写 0.012s（12ms），实测 75% 的循环拿不到新帧 → 每次任务接管
    # 都在 0.1 秒内被视频间隔误判踢掉（见 coordinator.video_gap）。
    consumer_wait_timeout: float = 0.05
    # AP 模式日志还出现过 0.41s 和 0.50s 后自行恢复的孤立传输间隙。
    # 旧运动命令仍会在 0.15s 后失效（见下面 command_timeout），因此 0.60s 的
    # 视频锁停阈值不会让底盘带着旧命令穿过长间隙。
    # 来源：王炜嘉在 fix/route-only-realtest 的实车测试（2026-09-15）——
    # 原来的 0.22s 会把这类能自恢复的孤立间隙当成视频丢失，任务被误杀。
    video_gap_stop_seconds: float = 0.60
    command_timeout: float = 0.15
    resume_detection_max_age: float = 0.15
    gimbal_pitch: int = -25
    gimbal_yaw: int = 0
    # -5 度在最早的实车恢复跑里画面里墙/天花板太多，路线被压到画面下沿。
    # 取 -12：比巡线视角（-25）看得宽，又不丢掉大部分地面。
    # 来源：王炜嘉在 fix/route-only-realtest 的实车测试（2026-09-15）。
    gimbal_search_pitch: int = -12
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
    #: 心跳行间隔（秒）：**每 3 秒**打一行"现在是谁在开车、它在干什么"。
    #: 只影响心跳，不影响状态变化行（接管/结束/丢线/异常都是立刻打）。
    console_heartbeat_seconds: float = 3.0
    #: 有模块接管时的心跳间隔（秒）。默认与上面相同（统一 3 秒一行）；
    #: 想让接管期间更密就把它调小（例如 0.5）。
    console_task_heartbeat_seconds: float = 3.0
    display: bool = True
    vision: VisionConfig = field(default_factory=VisionConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    tasks: TaskConfig = field(default_factory=TaskConfig)

CONFIG = RuntimeConfig()
