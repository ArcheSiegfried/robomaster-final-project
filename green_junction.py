"""绿灯岔路任务模块：识别岔路、选择可通行的一侧、有限转向后交回巡线。

本文件属于 **任务模块**，只做三件事：

1. 从共享帧里判断"前方是不是一个岔路口"（只在自己的 ROI 里干活）；
2. 按判据选出要走的那一边（判据不成立就失败停车，绝不瞎选）；
3. 输出受限的 ``MotionCommand`` 请求和 ``TaskUpdate`` 状态，转向有硬上限。

它**不做**的事（这是项目红线）：

* 不连相机、不连机器人、不调用任何 SDK；
* 不直接给底盘下命令、不绕过唯一运动出口 ``MotionOutput``；
* 不 ``sleep``、不死循环，``step()`` 每次调用立刻返回；
* 不修改 ``models.py`` / ``camera_source.py`` / ``runtime.py`` / ``motion_output.py`` / ``main.py``。

--------------------------------------------------------------------------
场地假设（没有人知道正式场地长什么样，所以先自己拍板，全部做成可改参数）
--------------------------------------------------------------------------

以下假设写在 ``JunctionConfig`` 里，任何一个都可以单独改，**没有写死在逻辑里**；
运行时判据不成立就 ``FAILED`` 停车，宁可不动也不瞎动。

============  ==========================================================
假设编号       内容
============  ==========================================================
A1            岔路是"蓝色巡线带一分为二"，两条分支都是蓝色（``require_blue_branches``）。
              如果正式赛道用别的颜色/材质，改 HSV 区间即可。
A2            岔路出现在画面上方：分叉带（同一行上被空隙拉开的蓝色带）落在 ROI 的
              ``max_split_fraction`` 以上（``min_split_row_ratio``、``min_split_row_offset``）。
A3            "分叉行"= 同一行上出现两段蓝色带，**空隙真正拉开**
              （``gap_over_tape_ratio``：空隙 ≥ 带宽的 40%）且内侧间距够远
              （``min_branch_separation_px`` 绝对像素 **或** ``min_branch_separation`` 比例），
              并且这个形态最少连续 ``min_split_rows`` 行。刚分叉时两条带还贴着，
              那只是被拉宽的粗线，普通弯道和实线噪点也不会满足这些条件。
A3b           岔路是分叉带上下某一侧连着一条"还没分叉"的带子的"Y"：分叉带上方或
              下方必须有一段不是分叉行的带子（``stem_window_ratio`` 的窗口里过半的
              行有带子但不算分叉），**或者**两条分支的间距沿画面纵向明显变化
              （``min_branch_opening_ratio``：远、近两端间距的比值 ≥ 1.25）。
              两条平行色块（间距恒定、两端都不接带子）不算岔路。
              张开和收拢**两个方向都算**：2026-09-15 实车那次正是"分叉点在画面下方、
              两条分支向上张开"的朝向，老代码只认反方向，所以判成
              ``branches too short``（见下面的修订说明）。
A4            分叉带的上沿离 ROI 底部还要有 ``min_branch_pixels_below`` 像素以上，
              说明车头正前方还有东西可看；分叉点已经贴到画面底部时说明车已经
              开过岔路，这时不触发。
A5            图像中心 = 车头正前方。相机水平视野 ``horizontal_fov_deg`` 默认 70 度，
              用来把像素偏移换算成转向角；这个值只影响转多少，不影响选哪边。
A6            灯的判据由 **3 号（traffic_light.py）** 提供。本模块通过注入的
              ``light_probe(frame, now) -> LightReading`` 读取，自己不认红绿灯
              （``LIGHT_OWNER`` 常量说明这个边界）。没有注入探针时，唯一可用的判据是
              ``fallback_color``，默认 ``"none"`` 表示"没有判据"：**这种配置下本模块
              连接管都不接管**（``require_rule_source``），把岔路让给后面注册的
              ``free_junction``；否则它只可能白停 ``decision_timeout`` 秒再 FAILED。
A7            两条分支都是绿灯、或者灯只报了一个颜色没有报位置时，
              ``fallback_rule`` 决定走哪边；默认 ``"straightest"``（走最接近车头正前方的那边）。
A8            转向是一段有限动作：yaw = 选中分支的偏角 × ``yaw_gain``，被
              ``max_turn_yaw`` 限幅，总时长不超过 ``turn_timeout``；
              转够了并且重新看到线，就 ``COMPLETED`` 交回巡线。
A9            走过一个岔路后 ``rearm_cooldown`` 秒内不再重复触发，避免同一次岔路被处理两遍。
A10           转向时前进速度与转向量成比例（``forward = forward_speed × |yaw| / max_turn_yaw``）：
              没选出分支时 yaw 为 0，前进也必须是 0，也就是"等判据时原地停着"。
              ``forward_speed`` 默认 0.10 m/s，低于骨架的 0.30 m/s 上限。
A11           "线回到画面中央"这个判据**不依赖框架传参**：协调器固定只调
              ``task.step(frame, now)``，所以框架永远不会给 ``line``。优先用传进来的
              ``line``，没有（或不可信）时用本模块自己在**画面近处窄带**里算出来的
              巡线带中心偏差（``near_band_*`` / ``near_min_rows`` / ``near_row_min_run``）。
              近处看到两条带（岔路分支）或看不到带子，都算"线没回来"。
A12           只有"分叉带已经贴到最后一截画面"（``drove_past_fork_row_ratio``）
              才可能判"车已经开过岔路口"：正常进近时车头前方的带子是单条且居中，
              和"开过了"长得一样，不加上这一条就会把正常进近误判成失败。
============  ==========================================================

修订说明（2026-09-15 实车测试报告）：本次改了三处，全部只在本文件里，
**没有改公共接口**（``step(frame, now, line=None)`` 签名不变，``line`` 依然是可选的）。

1. 岔路判据以前只认"两条分支越往车头方向越张开"，这次改成**两个方向都认**，
   并把"两条分支"的判定从"这一行恰好只有两段"放宽到"最左和最右两段"
   （实车画面里噪点常把一条带子切成三四段，老代码直接判"没有分叉行"）；
   同时把"分叉点以下必须有分支延伸到 ROI 底部"这条要求，换成老代码里那条
   除零风险的"张开比例"判据的新写法（见 ``_band_trend_ratio``）。
2. ``line`` 拿不到时，"线回中央"改用画面近处的蓝色带自己算（A11），
   否则 ``TURN → SETTLE → COMPLETED`` 这两道门在实车上恒为 False，只能超时失败。
3. ``_opens_upward`` 里 ``span`` 可能算成 0 → ``sum/span`` 除零（实车日志出现过
   34 条 ``float division by zero``）。新实现里 ``span <= 0`` 直接判否。
"""

from dataclasses import dataclass
from enum import Enum
from typing import Callable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from models import (
    FramePacket,
    LineDetection,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)

#: 红绿灯判据的归属者。本模块只消费结果，不自己认灯（与 3 号的边界）。
LIGHT_OWNER = "traffic_light.py"

#: 零速度。未触发、完成、失败时都返回它，coordinator 收到信号后可以随时硬停车。
STOP = MotionCommand()


# --------------------------------------------------------------------------
# 状态
# --------------------------------------------------------------------------


class JunctionState(Enum):
    """任务状态机。``READY`` 之前的调用返回 ``NOT_TRIGGERED``。"""

    IDLE = "idle"                  # 没看到岔路
    READY = "ready"                # 疑似岔路，正在连续确认
    DECIDE = "decide"              # 已确认岔路，等判据
    TURN = "turn"                  # 按选中的分支转向
    SETTLE = "settle"              # 转向结束，等线回到画面中央
    COMPLETED = "completed"        # 干完了，交回巡线
    FAILED = "failed"              # 判据不成立/超时，停车


class Branch(Enum):
    """岔路的两条分支。"""

    LEFT = "left"
    RIGHT = "right"


class LightColor(Enum):
    """3 号给过来的灯色。``UNKNOWN`` 表示"没看到灯"，绝不当成绿灯。"""

    RED = "red"
    GREEN = "green"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class LightReading:
    """3 号传进来的灯判据。

    ``branch`` 为 ``None`` 表示这一读数没有位置信息（只知道"看到绿灯"，
    不知道是哪一边的绿灯）——这时按 A7 的 ``fallback_rule`` 处理。
    """

    color: LightColor = LightColor.UNKNOWN
    branch: Optional[Branch] = None
    confidence: float = 0.0


#: 灯判据的读取接口。3 号/1 号把它的函数注入进来即可，不需要本模块依赖 3 号的文件。
LightProbe = Callable[[FramePacket, float], Optional[LightReading]]


# --------------------------------------------------------------------------
# 参数
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class JunctionConfig:
    """全部场地相关数字都在这里，换场地只改这里，不改逻辑。"""

    # --- ROI（整幅图的比例，见 A2） ---
    roi_left: float = 0.08
    roi_right: float = 0.92
    roi_top: float = 0.42
    roi_bottom: float = 0.96

    # --- 蓝色巡线带（见 A1；与 config.VisionConfig 的蓝线区间保持一致，可改） ---
    hsv_lower: Tuple[int, int, int] = (95, 80, 60)
    hsv_upper: Tuple[int, int, int] = (135, 255, 255)
    open_kernel: int = 3
    close_kernel: int = 9

    # --- 分叉判据（见 A2、A3、A4） ---
    row_min_run: int = 4             # 一行里多宽才算"一段带"
    run_merge_gap: int = 2           # 小于这个空隙的断点当成同一段（抗噪）
    #: 两段之间的空隙 / 那两段的平均带宽。刚分叉的地方两条带还贴着，
    #: 要等空隙真正拉开才算"分叉行"，否则只是被拉宽的粗线。
    gap_over_tape_ratio: float = 0.40
    #: 两段内侧间距的绝对下限（像素）。这个值沿用 7 号 free_junction.py 在
    #: 2026-09-15 实车上真的确认到岔路时用的 45 像素，比只按比例判更抗透视。
    min_branch_separation_px: int = 45
    min_branch_separation: float = 0.10   # 内侧间距 / ROI 宽度；与上面那条是"或"关系
    #: 两条分支的间距沿画面纵向必须明显变化（张开或收拢都算，见 A3b）：
    #: 远近两端间距的比值（大 / 小）不小于它；两条平行色块不算岔路。
    min_branch_opening_ratio: float = 1.25
    #: 至少要有这么多行分叉带才敢用"间距变化"当判据；行太少时趋势不可信，
    #: 改用"分叉带旁边有没有一段单条带子"来判。
    min_band_rows_for_trend: int = 6
    #: 分叉带上方/下方找"还没分叉的带子"的窗口高度（占 ROI 高度的比例）。
    stem_window_ratio: float = 0.05
    min_split_rows: int = 3          # 两段形态最少连续多少行（7 号实车用的也是 3）
    min_branch_rows: int = 2         # 每条分支最少占据多少行（只影响置信度打分）
    max_split_fraction: float = 0.80 # 分叉带起点在 ROI 内的最大纵向位置（再往下就等于车已开过）
    min_branch_pixels_below: int = 30  # 分叉带上沿到 ROI 底部至少要有这么多像素
    min_split_row_ratio: float = 0.02
    min_split_row_offset: int = 1    # 距 ROI 顶部的安全边距（行）
    max_branch_center_offset: float = 0.85  # 分支中心相对 ROI 中心的允许偏移
    line_center_deadband: float = 0.18      # 线偏多少还算"在中央"
    line_valid_confidence: float = 0.20     # 低于此置信度的线检测不算数
    min_junction_confidence: float = 0.25   # 综合置信度门槛

    # --- 近处巡线带（见 A11：coordinator 永远不传 line，只能自己从画面算） ---
    #: 只看画面最下面这条窄带（整幅图比例）——那里离车头最近，正常应该是一条带子。
    near_band_top: float = 0.88
    near_band_bottom: float = 0.99
    near_min_rows: int = 3        # 窄带里至少要有几行是"恰好一条带"
    near_row_min_run: int = 6     # 窄带里一行多宽才算"一条带"
    near_min_pixels: int = 40     # 窄带里至少要有这么多蓝像素，否则算没看到线
    near_error_deadband: float = 0.20   # 近处带中心的允许偏差（量纲同 line_center_deadband）

    # --- 确认与再触发（见 A8、A9） ---
    confirm_frames: int = 3          # 连续几帧都看到才算真岔路
    confirm_gap_grace: float = 0.25  # 中间漏看到的容忍时间（秒）
    confirm_timeout: float = 2.5     # 确认阶段最长耗时
    rearm_cooldown: float = 2.0      # 走过岔路后的再触发冷却时间（秒）
    decision_timeout: float = 6.0    # 到岔路口后等判据的最长时间
    #: 判定"车已经开过岔路口"需要连续多少帧同时满足：线回来了 + 岔路还在
    #: + 分叉带已经贴到画面最后一截（见 A12）。单帧的巧合不算。
    drove_past_frames: int = 3
    #: 分叉带下沿（两条分支汇成一条带子的地方）要低到 ROI 的这个比例以下，
    #: 才算"车头已经顶到岔路口"。正常进近时这个值小，不会误判成失败。
    drove_past_fork_row_ratio: float = 0.85
    junction_gap_grace: float = 0.60 # 转向中岔路短暂看不见的容忍时间
    total_timeout: float = 12.0      # 单次任务硬上限（骨架另有一层 20 秒上限）

    # --- 转向（见 A5、A8） ---
    forward_speed: float = 0.10      # 转向时的前进速度（m/s），0 表示原地转
    yaw_gain: float = 2.0            # 偏角(度) → yaw(deg/s) 的比例
    max_turn_yaw: float = 75.0       # 转向 yaw 上限（deg/s）
    turn_timeout: float = 3.5        # 转向阶段最长耗时
    turn_min_duration: float = 0.40  # 至少转这么久再判断线是否回来
    settle_timeout: float = 1.5      # 转完后等线回中央的最长时间
    horizontal_fov_deg: float = 70.0 # 相机水平视野，用于像素→角度

    # --- 判据（见 A6、A7） ---
    #: 没有注入 light_probe 时用什么颜色当"可通行"。默认 "none" = 没有判据 → 失败停车。
    fallback_color: str = "none"
    #: 灯判据不能区分左右时怎么选："straightest"（默认）或 "left" / "right" / "none"。
    fallback_rule: str = "straightest"
    #: 没有任何判据来源时（没注入 light_probe，``fallback_color`` 又是 "none"）
    #: 要不要接管。默认 **不接管**：这种配置下本模块不可能成功，接管只会白停
    #: ``decision_timeout`` 秒，还会把岔路从后面注册的模块（free_junction）手里抢走。
    #: 设成 False = 老行为："接管 → 原地停 → 超时 FAILED"。
    require_rule_source: bool = True
    require_blue_branches: bool = True  # A1：关掉就只按亮度/形态找分叉


# --------------------------------------------------------------------------
# 检测结果
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class BranchGeometry:
    """一条分支的几何信息；坐标是整幅图像像素。"""

    side: Branch
    center: Tuple[float, float]
    bearing_deg: float      # 相对图像中心的偏角，右为正
    rows: int               # 参与统计的采样行数


@dataclass(frozen=True)
class JunctionDetection:
    """一帧的岔路检测结果。无效一律用 :meth:`no_result`，不编坐标。"""

    valid: bool
    kind: str = "green_junction"
    message: str = ""
    split_row: Optional[int] = None      # 整幅图像像素 y
    split_center_x: Optional[int] = None # 整幅图像像素 x
    box: Optional[Tuple[int, int, int, int]] = None
    branches: Tuple[BranchGeometry, ...] = ()
    confidence: float = 0.0
    mask: Optional[np.ndarray] = None    # 整幅图像尺寸的蓝色掩膜
    fork_bottom_row: Optional[int] = None  # 分叉带下沿（分支开始汇成一条带的 y）
    band_rows: int = 0                     # 分叉带有多少行
    #: 分叉带下沿在 ROI 里的纵向位置（0=ROI 顶，1=ROI 底）。见 A12。
    band_bottom_ratio: float = 0.0
    #: 远、近两端间距的比值（≥1）；行数不够算不出时是 None。见 A3b。
    trend_ratio: Optional[float] = None
    has_stem: bool = False                 # 分叉带旁边是否有一段单条带子

    @classmethod
    def no_result(cls, message: str = "", mask: Optional[np.ndarray] = None):
        return cls(valid=False, message=message, mask=mask)

    def branch(self, side: Branch) -> Optional[BranchGeometry]:
        for item in self.branches:
            if item.side is side:
                return item
        return None


# --------------------------------------------------------------------------
# 基础工具
# --------------------------------------------------------------------------


def _roi_rect(image: np.ndarray, settings: JunctionConfig) -> Tuple[int, int, int, int]:
    height, width = image.shape[:2]
    left = int(width * settings.roi_left)
    right = int(width * settings.roi_right)
    top = int(height * settings.roi_top)
    bottom = int(height * settings.roi_bottom)
    left = max(0, min(left, width - 1))
    right = max(left + 1, min(right, width))
    top = max(0, min(top, height - 1))
    bottom = max(top + 1, min(bottom, height))
    return left, top, right, bottom


def _kernel(size: int) -> np.ndarray:
    size = max(1, int(size))
    if size % 2 == 0:
        size += 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))


def row_runs(
    row: np.ndarray,
    merge_gap: int = 2,
    min_run: int = 4,
) -> List[Tuple[int, int]]:
    """一行掩膜里的横向连通段（合并小空隙、丢掉太短的段）。

    返回的是这一行里每一段带的 ``(x0, x1)``（闭区间，相对这一行的坐标系）。
    岔路检测和"近处巡线带"共用它，保证两处的"一段带"定义完全一样。
    """
    indices = np.flatnonzero(row)
    if indices.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(indices) > 1)
    starts = np.concatenate(([0], breaks + 1))
    ends = np.concatenate((breaks, [indices.size - 1]))
    runs: List[Tuple[int, int]] = []
    for start, end in zip(starts, ends):
        x0 = int(indices[start])
        x1 = int(indices[end])
        # 合并
        if runs and x0 - runs[-1][1] <= merge_gap:
            runs[-1] = (runs[-1][0], x1)
        else:
            runs.append((x0, x1))
    return [
        (x0, x1)
        for x0, x1 in runs
        if (x1 - x0 + 1) >= max(1, int(min_run))
    ]


def blue_branch_mask(frame: np.ndarray, settings: JunctionConfig) -> np.ndarray:
    """返回整幅图像尺寸的蓝色巡线带掩膜；不做任何硬件访问。"""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be a non-empty BGR image")
    left, top, right, bottom = _roi_rect(frame, settings)
    roi = frame[top:bottom, left:right]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(settings.hsv_lower, dtype=np.uint8),
        np.array(settings.hsv_upper, dtype=np.uint8),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _kernel(settings.open_kernel))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _kernel(settings.close_kernel))
    full = np.zeros(frame.shape[:2], dtype=np.uint8)
    full[top:bottom, left:right] = mask
    return full


def line_is_centered(
    line: Optional[LineDetection],
    deadband: float = 0.18,
    min_confidence: float = 0.20,
) -> bool:
    """线在画面中央附近且置信度够高，才算"巡线已经回来了"。"""
    if line is None or not line.valid:
        return False
    if line.confidence < min_confidence:
        return False
    return abs(line.error) <= deadband


def near_field_line(
    image: np.ndarray,
    settings: Optional[JunctionConfig] = None,
) -> Tuple[Optional[float], str]:
    """只看画面最下方一条窄带，估计巡线带中心相对画面中心的偏差（-1..1）。

    为什么需要它（见 A11）：协调器固定只调 ``task.step(frame, now)``，
    永远不会传 ``line``，所以"线回到中央了没有"这道门在本模块里必须自己算。

    判据刻意保守：

    * 窄带里必须**主要只有一条带子**（``near_min_rows`` 行以上）；
    * 看到两条以上（岔路的两条分支伸到车头前）→ 返回 ``None``，也就是"线没回来"；
    * 蓝色像素太少（``near_min_pixels``）→ 同样算没看到线。

    :return: ``(error, reason)``；``error`` 为 ``None`` 表示这一帧不能判定。
    """
    if image is None or getattr(image, "ndim", 0) != 3 or image.shape[2] != 3:
        return None, "no image"
    settings = settings if settings is not None else JunctionConfig()
    height, width = image.shape[:2]
    left = max(0, min(int(width * settings.roi_left), width - 1))
    right = max(left + 1, min(int(width * settings.roi_right), width))
    y0 = max(0, min(height - 1, int(height * settings.near_band_top)))
    y1 = max(y0 + 1, min(height, int(height * settings.near_band_bottom)))
    strip = image[y0:y1, left:right]
    if strip.size == 0:
        return None, "empty near band"

    hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(settings.hsv_lower, dtype=np.uint8),
        np.array(settings.hsv_upper, dtype=np.uint8),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _kernel(settings.open_kernel))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _kernel(settings.close_kernel))
    if int(np.count_nonzero(mask)) < max(0, int(settings.near_min_pixels)):
        return None, "no tape in the near band"

    centers: List[float] = []
    split_rows = 0
    for row in mask:
        runs = row_runs(row, settings.run_merge_gap, settings.near_row_min_run)
        if len(runs) == 1:
            centers.append((runs[0][0] + runs[0][1]) / 2.0 + left)
        elif len(runs) > 1:
            split_rows += 1
    if len(centers) < max(1, int(settings.near_min_rows)):
        if split_rows:
            return None, "near band is split into branches"
        return None, "no tape in the near band"

    error = (float(np.median(centers)) - width / 2.0) / max(width / 2.0, 1.0)
    return error, "near band error=%.2f" % error


def near_line_is_centered(
    image: np.ndarray,
    settings: Optional[JunctionConfig] = None,
) -> Tuple[bool, str]:
    """``near_field_line`` 的布尔版本：近处那条带子是否就在画面中央附近（见 A11）。"""
    settings = settings if settings is not None else JunctionConfig()
    error, reason = near_field_line(image, settings)
    if error is None:
        return False, reason
    return abs(error) <= settings.near_error_deadband, reason


# --------------------------------------------------------------------------
# 检测器
# --------------------------------------------------------------------------


class JunctionDetector:
    """纯视觉：判断前方是不是岔路，并给出两条分支的偏角。"""

    def __init__(self, settings: Optional[JunctionConfig] = None) -> None:
        self.settings = settings if settings is not None else JunctionConfig()
        self.last_confidence = 0.0

    # -- 行的处理 ---------------------------------------------------------

    def _row_runs(self, row: np.ndarray) -> List[Tuple[int, int]]:
        """一行掩膜里的横向连通段（已按 settings 合并小空隙、丢掉太短的段）。"""
        return row_runs(row, self.settings.run_merge_gap, self.settings.row_min_run)

    def _two_runs(
        self, row: np.ndarray, width: int
    ) -> Optional[Tuple[Tuple[int, int], Tuple[int, int]]]:
        """这一行是不是"被空隙真正拉开的两段"；不够就返回 ``None``。

        只取**最左和最右**两段：实车画面里一条带子常被噪点切成三四段，
        老代码要求"恰好两段"，于是在真岔路上直接判"没有分叉行"。
        被噪声切开的单条带子仍然过不了下面的空隙判据（空隙相对带宽太小）。
        """
        runs = self._row_runs(row)
        if len(runs) < 2:
            return None
        first, second = runs[0], runs[-1]
        gap = second[0] - first[1] - 1
        tape = ((first[1] - first[0] + 1) + (second[1] - second[0] + 1)) / 2.0
        if gap < tape * self.settings.gap_over_tape_ratio:
            # 刚分叉的地方两条带还贴着；这时仍是一条（被拉宽的）线。
            return None
        separation_px = second[0] - first[1]
        if (
            separation_px < self.settings.min_branch_separation_px
            and separation_px / max(width, 1) < self.settings.min_branch_separation
        ):
            return None
        return first, second

    # -- 分叉带 -----------------------------------------------------------

    def _scan_rows(
        self, mask: np.ndarray, rect: Tuple[int, int, int, int]
    ) -> List[Tuple[int, Tuple[Tuple[int, int], Tuple[int, int]]]]:
        """把 ROI 里"是分叉行"的行按顺序列出来（每行带各自的左右两段）。

        扫描范围是整个 ROI：分叉带允许一直伸到 ROI 底部（见 A3b/A12，
        实车那次的朝向就是这样，带子下沿贴到画面底部才算"车顶到口子上了"）。
        起点仍然留安全边距，免得把画面最上面一两行噪声当成分叉。
        """
        left, top, right, bottom = rect
        height = bottom - top
        settings = self.settings
        start = top + max(
            1, int(height * settings.min_split_row_ratio), settings.min_split_row_offset
        )
        rows = []
        for y in range(start, bottom):
            pair = self._two_runs(mask[y, left:right], right - left)
            if pair is not None:
                rows.append((y, pair))
        return rows

    @staticmethod
    def _bands(
        rows: Sequence[Tuple[int, Tuple[Tuple[int, int], Tuple[int, int]]]],
        min_rows: int,
    ) -> List[List[Tuple[int, Tuple[Tuple[int, int], Tuple[int, int]]]]]:
        """把分叉行切成一段段**连续**的带子，只留下够长的。"""
        bands: List[List[Tuple[int, Tuple[Tuple[int, int], Tuple[int, int]]]]] = []
        current: List[Tuple[int, Tuple[Tuple[int, int], Tuple[int, int]]]] = []
        previous_y: Optional[int] = None
        for item in rows:
            y = item[0]
            if previous_y is not None and y != previous_y + 1:
                if len(current) >= min_rows:
                    bands.append(current)
                current = []
            current.append(item)
            previous_y = y
        if len(current) >= min_rows:
            bands.append(current)
        return bands

    @staticmethod
    def _band_strength(
        band: Sequence[Tuple[int, Tuple[Tuple[int, int], Tuple[int, int]]]]
    ) -> float:
        """挑带子时的强度：平均内侧间距（像素），越宽越像岔路。"""
        return sum(float(pair[1][0] - pair[0][1]) for _y, pair in band) / max(len(band), 1)

    def _band_trend_ratio(self, band: Sequence) -> Optional[float]:
        """远、近两端内侧间距的比值（大 / 小，≥1）；行太少返回 ``None``。

        张开（近处更宽）和收拢（近处更窄）**都算**岔路：实车那次是"分叉点在
        画面下方、两条分支向上张开"，也就是往下收拢。平行色块的比值接近 1。

        老代码的 ``_opens_upward`` 在这里会除零：``span`` 可能是 0，
        然后 ``sum(valid[-span:]) / span`` 直接炸（实车日志里那 34 条
        ``float division by zero`` 就是它）。
        """
        settings = self.settings
        separations = [float(pair[1][0] - pair[0][1]) for _y, pair in band]
        if len(separations) < max(2, int(settings.min_band_rows_for_trend)):
            return None
        span = max(1, len(separations) // 3)
        far = sum(separations[:span]) / span     # 画面上方（远）
        near = sum(separations[-span:]) / span   # 画面下方（近）
        if far <= 0.0 or near <= 0.0:
            return None
        return max(far, near) / min(far, near)

    def _has_stem(
        self,
        mask: np.ndarray,
        rect: Tuple[int, int, int, int],
        band_top: int,
        band_bottom: int,
    ) -> bool:
        """分叉带的上方或下方，是不是接着一段"还没分叉"的带子（Y 的那条腿）。

        只看"这一行有没有带子、而且不是分叉行"：刚离开分叉点的地方两条带子
        往往还贴着（空隙不够），那也算腿——所以不能要求"恰好一段"。
        """
        left, top, right, bottom = rect
        width = right - left
        window = max(3, int((bottom - top) * self.settings.stem_window_ratio))
        windows = (
            (max(top, band_top - window), band_top),
            (band_bottom + 1, min(bottom, band_bottom + 1 + window)),
        )
        for start, stop in windows:
            total = 0
            stem = 0
            for y in range(start, stop):
                total += 1
                if self._two_runs(mask[y, left:right], width) is not None:
                    continue
                if self._row_runs(mask[y, left:right]):
                    stem += 1
            if total and stem * 2 >= total:
                return True
        return False

    # -- 分支几何 ---------------------------------------------------------

    def _branch_geometry(
        self,
        band: Sequence[Tuple[int, Tuple[Tuple[int, int], Tuple[int, int]]]],
        rect: Tuple[int, int, int, int],
    ) -> Tuple[Optional[BranchGeometry], Optional[BranchGeometry], str]:
        """用分叉带里的每一行，统计两条分支的中心位置和偏角。

        返回 ``(左, 右, 原因)``；失败时前两项都是 ``None``，原因直接进日志，
        这样下一次实车/离线探针能一眼看出是哪一道判据挡的。
        """
        left, top, right, bottom = rect
        width = right - left
        settings = self.settings
        xs_left: List[float] = []
        ys_left: List[float] = []
        xs_right: List[float] = []
        ys_right: List[float] = []
        for y, pair in band:
            (a0, a1), (b0, b1) = pair
            xs_left.append((a0 + a1) / 2.0 + left)
            ys_left.append(float(y))
            xs_right.append((b0 + b1) / 2.0 + left)
            ys_right.append(float(y))
        rows = len(xs_left)
        if rows < max(1, int(settings.min_branch_rows)):
            return None, None, "band too short for branch geometry"

        half_width = max(width / 2.0, 1.0)
        per_pixel_deg = settings.horizontal_fov_deg / max(width, 1)
        frame_center = left + width / 2.0
        roi_center = frame_center  # ROI 左右基本对称；用整幅图中心当车头正前方

        result: List[BranchGeometry] = []
        for xs, ys, side in (
            (xs_left, ys_left, Branch.LEFT),
            (xs_right, ys_right, Branch.RIGHT),
        ):
            center_x = float(np.mean(xs))
            center_y = float(np.mean(ys))
            if abs(center_x - roi_center) / half_width > settings.max_branch_center_offset:
                return None, None, "branch centre is outside the ROI"
            bearing = (center_x - frame_center) * per_pixel_deg
            result.append(
                BranchGeometry(
                    side=side,
                    center=(center_x, center_y),
                    bearing_deg=float(bearing),
                    rows=rows,
                )
            )
        return result[0], result[1], "ok"

    # -- 对外入口 ---------------------------------------------------------

    def detect(
        self,
        frame: np.ndarray,
        mask: Optional[np.ndarray] = None,
    ) -> JunctionDetection:
        """一帧一次，立即返回。找不到岔路就返回 invalid，不编坐标。"""
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must be a non-empty BGR image")
        settings = self.settings
        if mask is None:
            mask = blue_branch_mask(frame, settings) if settings.require_blue_branches else None
        if mask is None:
            self.last_confidence = 0.0
            return JunctionDetection.no_result("brightness mode not implemented")

        rect = _roi_rect(frame, settings)
        left, top, right, bottom = rect
        box = (left, top, right, bottom)
        if int(np.count_nonzero(mask)) == 0:
            self.last_confidence = 0.0
            return JunctionDetection.no_result("no blue pigment in ROI", mask)

        rows = self._scan_rows(mask, rect)
        bands = self._bands(rows, max(1, int(settings.min_split_rows)))
        height = max(bottom - top, 1)
        # 分叉带的**起点**不能太低（A2）：太低了等于车已经压在口子上，来不及判了。
        start_limit = top + int(height * settings.max_split_fraction)
        bands = [item for item in bands if item[0][0] <= start_limit]
        band: Optional[List[Tuple[int, Tuple[Tuple[int, int], Tuple[int, int]]]]] = None
        if bands:
            # 先要最长的一段，一样长时取间距最大的那段（最像岔路）。
            band = max(bands, key=lambda item: (len(item), self._band_strength(item)))
        if band is None:
            self.last_confidence = 0.0
            message = (
                "fork is already under the car"
                if rows
                else "no two-run row (curve or solid line)"
            )
            return JunctionDetection.no_result(message, mask)

        band_top = band[0][0]
        band_bottom = band[-1][0]
        if bottom - band_top < settings.min_branch_pixels_below:
            # 分叉点已经贴到画面底部，等于车已经开过去了，不处理。
            self.last_confidence = 0.0
            return JunctionDetection.no_result("fork is already under the car", mask)

        pair = band[0][1]
        split_center = (pair[0][1] + pair[1][0]) / 2.0 + left

        left_branch, right_branch, geometry_reason = self._branch_geometry(band, rect)
        if left_branch is None or right_branch is None:
            self.last_confidence = 0.0
            return JunctionDetection.no_result(geometry_reason, mask)

        # A3b：分叉带旁边必须接着一段"还没分叉"的带子（Y 的腿），或者两条分支的
        # 间距沿纵向明显变化（张开/收拢都算）。两条平行色块（间距恒定、两头都不
        # 接带子）会被这一条挡掉。
        trend_ratio = self._band_trend_ratio(band)
        has_stem = self._has_stem(mask, rect, band_top, band_bottom)
        trend_ok = (
            trend_ratio is not None
            and trend_ratio >= settings.min_branch_opening_ratio
        )
        if not trend_ok and not has_stem:
            self.last_confidence = 0.0
            return JunctionDetection.no_result(
                "parallel bands: no stem and no opening (trend=%s)"
                % ("n/a" if trend_ratio is None else "%.2f" % trend_ratio),
                mask,
            )

        height = max(bottom - top, 1)
        row_ratio = (band_top - top) / height
        row_score = max(0.0, 1.0 - row_ratio)
        separation = right_branch.center[0] - left_branch.center[0]
        sep_score = min(1.0, max(0.0, separation) / max(right - left, 1) / 0.30)
        row_count = min(left_branch.rows, right_branch.rows)
        row_score2 = min(1.0, row_count / max(settings.min_branch_rows, 1) / 3.0)
        confidence = 0.2 * row_score + 0.4 * sep_score + 0.4 * row_score2
        self.last_confidence = confidence
        if confidence < settings.min_junction_confidence:
            return JunctionDetection.no_result("junction too weak", mask)

        return JunctionDetection(
            valid=True,
            message="junction confirmed",
            split_row=int(band_top),
            split_center_x=int(round(split_center)),
            box=box,
            branches=(left_branch, right_branch),
            confidence=confidence,
            mask=mask,
            fork_bottom_row=int(band_bottom),
            band_rows=len(band),
            band_bottom_ratio=float((band_bottom - top) / height),
            trend_ratio=trend_ratio,
            has_stem=has_stem,
        )


# --------------------------------------------------------------------------
# 判据：选哪一边
# --------------------------------------------------------------------------


def _straightest(branches: Sequence[BranchGeometry]) -> Optional[BranchGeometry]:
    if not branches:
        return None
    return min(branches, key=lambda item: abs(item.bearing_deg))


def evaluate_branches(
    branches: Sequence[BranchGeometry],
    reading: Optional[LightReading],
    settings: JunctionConfig,
) -> Tuple[Optional[BranchGeometry], str]:
    """按判据选出要走的分支；选不出来就返回 ``(None, 原因)``。

    判据优先级：

    1. 探针给了"哪一边是绿灯" → 走那一边；
    2. 探针只知道颜色、不知道位置 → 颜色必须是绿的，再按 ``fallback_rule`` 选边；
    3. 没有探针 → 用 ``fallback_color`` 当颜色（默认 ``"none"`` = 没有判据 → 失败）。
    """
    if not branches:
        return None, "no branch geometry"

    color: Optional[LightColor] = None
    forced: Optional[BranchGeometry] = None
    if reading is not None:
        color = reading.color
        if reading.branch is not None:
            if reading.color is not LightColor.GREEN:
                return None, f"{reading.branch.value} branch is {reading.color.value}"
            forced = next(
                (item for item in branches if item.side is reading.branch), None
            )
            if forced is None:
                return None, "probe points at a missing branch"
    else:
        fallback = (settings.fallback_color or "none").strip().lower()
        if fallback == "green":
            color = LightColor.GREEN
        elif fallback == "red":
            color = LightColor.RED
        else:
            return None, "no light probe and fallback_color is none"

    if color is LightColor.RED:
        return None, "red light"
    if color is not LightColor.GREEN:
        return None, "light color unknown"

    if forced is not None:
        return forced, "probe selected branch"

    rule = (settings.fallback_rule or "straightest").strip().lower()
    if rule == "left":
        chosen = next((item for item in branches if item.side is Branch.LEFT), None)
        return (chosen, "fallback_rule=left") if chosen else (None, "no left branch")
    if rule == "right":
        chosen = next((item for item in branches if item.side is Branch.RIGHT), None)
        return (chosen, "fallback_rule=right") if chosen else (None, "no right branch")
    if rule == "none":
        return None, "light cannot tell left from right"

    chosen = _straightest(branches)
    return (chosen, "fallback_rule=straightest") if chosen else (None, "no branch")


# --------------------------------------------------------------------------
# 任务状态机
# --------------------------------------------------------------------------


class GreenJunctionTask:
    """6 号的模块主体：插进 coordinator 就能用，单独也能测。

    用法::

        task = GreenJunctionTask(light_probe=traffic_light_module.reading)
        update = task.step(frame_packet, now)      # 每帧一次，立即返回
        # update.status 是 NOT_TRIGGERED / RUNNING / COMPLETED / FAILED
        # update.motion 是给 MotionOutput 的速度请求（可能是 None）

    死规矩（红线 8）：一旦返回 ``RUNNING``，就必须持续返回 ``RUNNING``，
    直到 ``COMPLETED`` 或 ``FAILED``，中间不会改口成 ``NOT_TRIGGERED``。
    """
    name = "green_junction"

    def __init__(
        self,
        settings: Optional[JunctionConfig] = None,
        detector: Optional[JunctionDetector] = None,
        light_probe: Optional[LightProbe] = None,
    ) -> None:
        self.settings = settings if settings is not None else JunctionConfig()
        self.detector = detector if detector is not None else JunctionDetector(self.settings)
        self.light_probe = light_probe
        self.state = JunctionState.IDLE
        self.last_detection: Optional[JunctionDetection] = None
        self.last_line: Optional[LineDetection] = None
        self.last_reading: Optional[LightReading] = None
        self.chosen_branch: Optional[Branch] = None
        self.last_message = "idle"
        self.mask: Optional[np.ndarray] = None

        self._confirm_count = 0
        self._drove_past_frames = 0
        self._idle_since: Optional[float] = None
        self._state_since: Optional[float] = None
        self._last_seen_at: Optional[float] = None
        self._run_started_at: Optional[float] = None
        self._turn_started_at: Optional[float] = None
        self._rearm_ready_at: Optional[float] = None

    # -- 对外状态 ---------------------------------------------------------

    @property
    def active(self) -> bool:
        """``True`` 表示本模块正在接管（coordinator 据此保持 / 交出控制权）。"""
        return self.state in (JunctionState.DECIDE, JunctionState.TURN, JunctionState.SETTLE)

    @property
    def finished(self) -> bool:
        return self.state in (JunctionState.COMPLETED, JunctionState.FAILED)

    @property
    def chosen_bearing_deg(self) -> Optional[float]:
        detection = self.last_detection
        if detection is None or self.chosen_branch is None:
            return None
        branch = detection.branch(self.chosen_branch)
        return None if branch is None else branch.bearing_deg

    def reset(self) -> None:
        """回到初始状态，供 coordinator 在复位时调用。"""
        self.state = JunctionState.IDLE
        self.last_detection = None
        self.last_line = None
        self.last_reading = None
        self.chosen_branch = None
        self.last_message = "idle"
        self.mask = None
        self._confirm_count = 0
        self._drove_past_frames = 0
        self._idle_since = None
        self._state_since = None
        self._last_seen_at = None
        self._run_started_at = None
        self._turn_started_at = None
        self._rearm_ready_at = None

    # -- 内部工具 ---------------------------------------------------------

    def _enter(self, state: JunctionState, now: float) -> None:
        self.state = state
        self._state_since = now
        if state is JunctionState.TURN:
            # 转向总时长从进入 TURN 开始算，跨 TURN / SETTLE 两段都有效。
            self._turn_started_at = now

    def _elapsed(self, now: float) -> float:
        if self._state_since is None:
            return 0.0
        return max(0.0, now - self._state_since)

    def _turn_elapsed(self, now: float) -> float:
        """从进入 TURN 开始算的转向总时长（SETTLE 阶段继续累加）。"""
        if self._turn_started_at is None:
            return 0.0
        return max(0.0, now - self._turn_started_at)

    def _idle_elapsed(self, now: float) -> float:
        if self._idle_since is None:
            return 0.0
        return max(0.0, now - self._idle_since)

    def _visual(self, detection: Optional[JunctionDetection] = None) -> VisualDetection:
        payload = detection if detection is not None else self.last_detection
        if payload is None or not payload.valid:
            return VisualDetection.no_result("green_junction")
        center = (
            int(round(payload.split_center_x)),
            int(payload.split_row),
        )
        return VisualDetection(
            valid=True,
            kind="green_junction",
            center=center,
            target_id=(
                self.chosen_branch.value if self.chosen_branch is not None else None
            ),
            color=None,
            confidence=payload.confidence,
            box=payload.box,
        )

    def _not_triggered(self, message: str) -> TaskUpdate:
        self.last_message = message
        return TaskUpdate(
            TaskStatus.NOT_TRIGGERED,
            motion=STOP,
            detection=self._visual(),
            message=message,
        )

    def _running(self, now: float, message: str) -> TaskUpdate:
        command = self._turn_command(now)
        self.last_message = message
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=command,
            detection=self._visual(),
            message=message,
        )

    def _completed(self, message: str) -> TaskUpdate:
        self.state = JunctionState.COMPLETED
        self.last_message = message
        return TaskUpdate(
            TaskStatus.COMPLETED,
            motion=STOP,
            detection=self._visual(),
            message=message,
        )

    def _failed(self, message: str) -> TaskUpdate:
        self.state = JunctionState.FAILED
        self.last_message = message
        return TaskUpdate(
            TaskStatus.FAILED,
            motion=STOP,
            detection=self._visual(),
            message=message,
        )

    def _turn_command(self, now: float) -> MotionCommand:
        """转向请求：yaw = 选中分支偏角 × 增益，限幅且有硬上限。

        前进速度与转向量成比例（A10）：还没选出分支时 yaw 为 0，前进也为 0，
        也就是等判据期间原地停着，绝不自己往前冲。
        """
        settings = self.settings
        bearing = self.chosen_bearing_deg or 0.0
        yaw = bearing * settings.yaw_gain
        limit = abs(settings.max_turn_yaw)
        yaw = max(-limit, min(limit, yaw))
        ratio = 0.0 if limit <= 0.0 else min(1.0, abs(yaw) / limit)
        return MotionCommand(
            forward=settings.forward_speed * ratio,
            lateral=0.0,
            yaw=yaw,
        )

    def _read_probe(self, frame: FramePacket, now: float) -> Optional[LightReading]:
        if self.light_probe is None:
            return None
        try:
            reading = self.light_probe(frame, now)
        except Exception:
            # 别人的模块出错不能让车失控；按"没有判据"处理。
            return None
        if reading is None:
            return None
        if isinstance(reading, LightReading):
            return reading
        return None

    def _has_rule_source(self) -> bool:
        """有没有任何"走哪边"的判据来源（见 A6 / ``require_rule_source``）。"""
        if not self.settings.require_rule_source:
            return True
        if self.light_probe is not None:
            return True
        fallback = (self.settings.fallback_color or "none").strip().lower()
        return fallback in ("green", "red")

    def _line_centered(self, frame: FramePacket, now: float) -> Tuple[bool, str]:
        """线回到画面中央了没有（见 A11）。

        协调器固定只调 ``step(frame, now)``，``line`` 永远是 ``None``，所以这里
        必须自己从画面算：优先用框架真的传进来的 ``line``（离线测试和别的
        整合方式会传），拿不到或者不可信时，退回画面近处那条窄带
        （:func:`near_line_is_centered`）。

        返回 ``(是否居中, 判据来源或原因)``，第二个值只进日志/消息。
        """
        settings = self.settings
        line = self.last_line
        if (
            line is not None
            and getattr(line, "valid", False)
            and getattr(line, "confidence", 0.0) >= settings.line_valid_confidence
        ):
            centered = line_is_centered(
                line, settings.line_center_deadband, settings.line_valid_confidence
            )
            return centered, "line detector"
        if frame is None or frame.image is None:
            return False, "no image for the near-field line check"
        return near_line_is_centered(frame.image, settings)

    # -- 主入口 -----------------------------------------------------------

    def step(
        self,
        frame: FramePacket,
        now: float,
        line: Optional[LineDetection] = None,
    ) -> TaskUpdate:
        """每来一张新帧调用一次，立即返回。

        :param frame: 框架给的共享帧（本模块只读 ``frame.image``）。
        :param now: 单调时间（秒）。
        :param line: 可选，框架本帧的巡线结果。**不传也能跑**：协调器固定只调
            ``step(frame, now)``，这种情况下"线回到中央了没有"由本模块
            从画面近处的蓝色带自己算（见 A11 / :meth:`_line_centered`）。
        """
        settings = self.settings
        self.last_line = line

        if self.finished:
            return TaskUpdate(
                TaskStatus.COMPLETED
                if self.state is JunctionState.COMPLETED
                else TaskStatus.FAILED,
                motion=STOP,
                detection=self._visual(),
                message=self.last_message,
            )

        if frame is None or frame.image is None:
            return self._handle_missing_image(now)

        mask = blue_branch_mask(frame.image, settings) if settings.require_blue_branches else None
        detection = self.detector.detect(frame.image, mask)
        self.last_detection = detection if detection.valid else None
        self.mask = detection.mask

        if self.state is JunctionState.IDLE:
            return self._step_idle(detection, now)
        if self.state is JunctionState.READY:
            return self._step_ready(detection, now)
        if self.state is JunctionState.DECIDE:
            return self._step_decide(detection, frame, now)
        if self.state is JunctionState.TURN:
            return self._step_turn(detection, frame, now)
        return self._step_settle(detection, frame, now)

    # -- 状态实现 ---------------------------------------------------------

    def _handle_missing_image(self, now: float) -> TaskUpdate:
        if self.active:
            return self._failed("frame image missing while owning control")
        return self._not_triggered("no frame")

    def _step_idle(self, detection: JunctionDetection, now: float) -> TaskUpdate:
        if self._idle_since is None:
            self._idle_since = now
        if not self._has_rule_source():
            # 一个判据来源都没有（既没有探针，fallback_color 又是 "none"）：
            # 本模块在这个岔路口**不可能**成功，接管只会白停 6 秒并把岔路
            # 从后面的模块（例如 free_junction）手里抢走。所以干脆不触发，
            # 让协调器去问下一个模块。想恢复"接管、停住、再失败"的老行为，
            # 把 JunctionConfig(require_rule_source=False) 即可。
            self._confirm_count = 0
            return self._not_triggered("no light rule source; not applicable")
        if self._rearm_ready_at is not None and now < self._rearm_ready_at:
            # 刚走过一个岔路：冷却期内即使又看到岔路形状也不重复触发（A9）。
            self._confirm_count = 0
            return self._not_triggered("rearm cooldown")
        if not detection.valid:
            self._confirm_count = 0
            return self._not_triggered("no junction")
        if self._elapsed(now) > self.settings.total_timeout:
            self._confirm_count = 0
            return self._not_triggered("looked too long without a decision")
        self._confirm_count = 1
        self._last_seen_at = now
        self._enter(JunctionState.READY, now)
        return self._not_triggered("junction candidate, confirming 1/%d" % self.settings.confirm_frames)

    def _step_ready(self, detection: JunctionDetection, now: float) -> TaskUpdate:
        settings = self.settings
        if detection.valid:
            self._confirm_count += 1
            self._last_seen_at = now
            if self._confirm_count >= settings.confirm_frames:
                self.chosen_branch = None
                self._drove_past_frames = 0
                self._run_started_at = now
                self._enter(JunctionState.DECIDE, now)
                return self._running(
                    now,
                    "junction confirmed (%d frames), waiting for light rule"
                    % self._confirm_count,
                )
            return self._not_triggered(
                "junction candidate, confirming %d/%d"
                % (self._confirm_count, settings.confirm_frames)
            )

        gap = 0.0 if self._last_seen_at is None else max(0.0, now - self._last_seen_at)
        if gap <= settings.confirm_gap_grace and self._elapsed(now) <= settings.confirm_timeout:
            return self._not_triggered("junction flicker, holding confirmation")

        self._confirm_count = 0
        self._enter(JunctionState.IDLE, now)
        self._idle_since = now
        return self._not_triggered("junction candidate vanished before confirmation")

    def _step_decide(
        self, detection: JunctionDetection, frame: FramePacket, now: float
    ) -> TaskUpdate:
        settings = self.settings
        if self._run_started_at is not None and now - self._run_started_at > settings.total_timeout:
            return self._failed("task total timeout")

        # "线在画面中央" + "岔路形态还在" + "分叉带已经贴到最后一截画面"连续出现
        # → 车其实已经顶到岔路口了。这时没有分支可选，必须失败停车，不能瞎选一边。
        # 最后那一条（A12）是必须的：正常进近时车头前的带子也是单条且居中，
        # 少了它就会把正常进近误判成"开过了"。
        centered, line_source = self._line_centered(frame, now)
        fork_at_the_car = (
            detection.band_bottom_ratio >= settings.drove_past_fork_row_ratio
        )
        if detection.valid and centered and fork_at_the_car:
            self._drove_past_frames += 1
            if self._drove_past_frames >= settings.drove_past_frames:
                return self._failed("drove past the junction without a decision")
        else:
            self._drove_past_frames = 0

        if not detection.valid:
            self.chosen_branch = None
            if self._elapsed(now) > settings.decision_timeout:
                return self._failed("junction lost before a rule could be applied")
            return self._running(now, "waiting for junction re-detection")

        reading = self._read_probe(frame, now)
        self.last_reading = reading
        chosen, reason = evaluate_branches(detection.branches, reading, settings)
        if chosen is None:
            if self._elapsed(now) > settings.decision_timeout:
                return self._failed("no usable rule: %s" % reason)
            return self._running(now, "waiting for rule: %s" % reason)

        self.chosen_branch = chosen.side
        self._enter(JunctionState.TURN, now)
        return self._running(
            now,
            "take %s branch (bearing %.1f deg, %s)"
            % (chosen.side.value, chosen.bearing_deg, reason),
        )

    def _step_turn(
        self, detection: JunctionDetection, frame: FramePacket, now: float
    ) -> TaskUpdate:
        settings = self.settings
        if self._run_started_at is not None and now - self._run_started_at > settings.total_timeout:
            return self._failed("task total timeout during turn")
        if self._elapsed(now) > settings.turn_timeout:
            return self._failed("turn did not finish inside turn_timeout")

        if detection.valid:
            self._last_seen_at = now
        else:
            gap = 0.0 if self._last_seen_at is None else max(0.0, now - self._last_seen_at)
            if gap > settings.junction_gap_grace:
                return self._failed("lost the junction while turning")

        centered, line_source = self._line_centered(frame, now)
        settled = self._turn_elapsed(now) >= settings.turn_min_duration and centered
        if settled:
            self._enter(JunctionState.SETTLE, now)
            return self._running(
                now, "branch entered, checking line stability (%s)" % line_source
            )

        return self._running(
            now,
            "turning onto %s branch"
            % ("?" if self.chosen_branch is None else self.chosen_branch.value),
        )

    def _step_settle(
        self, detection: JunctionDetection, frame: FramePacket, now: float
    ) -> TaskUpdate:
        settings = self.settings
        if self._run_started_at is not None and now - self._run_started_at > settings.total_timeout:
            return self._failed("task total timeout while settling")

        centered, line_source = self._line_centered(frame, now)
        stable = centered and self._turn_elapsed(now) >= settings.turn_min_duration
        if stable:
            self._rearm_ready_at = now + settings.rearm_cooldown
            return self._completed("junction passed; line reacquired (%s)" % line_source)

        if self._elapsed(now) > settings.settle_timeout:
            return self._failed("line did not return after the turn (%s)" % line_source)

        return self._running(now, "settling on the new branch")


def detections_for_log(detection: Optional[JunctionDetection]) -> dict:
    """给 evidence/日志用的小摘要，不含大数组。"""
    if detection is None or not detection.valid:
        return {"valid": False}
    return {
        "valid": True,
        "split_row": detection.split_row,
        "split_center_x": detection.split_center_x,
        "confidence": round(float(detection.confidence), 4),
        "fork_bottom_row": detection.fork_bottom_row,
        "band_rows": detection.band_rows,
        "band_bottom_ratio": round(float(detection.band_bottom_ratio), 3),
        "trend_ratio": (
            None if detection.trend_ratio is None else round(float(detection.trend_ratio), 3)
        ),
        "has_stem": bool(detection.has_stem),
        "branches": [
            {
                "side": item.side.value,
                "center": [round(item.center[0], 1), round(item.center[1], 1)],
                "bearing_deg": round(item.bearing_deg, 2),
                "rows": item.rows,
            }
            for item in detection.branches
        ],
    }
