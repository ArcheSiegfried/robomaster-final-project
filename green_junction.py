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
A2            岔路出现在画面上方：分叉行位于 ROI 的 35% 以下位置
              （``min_split_row_ratio``、``max_split_row_ratio``、``max_split_fraction``）。
A3            "分叉行"= 同一行上出现两段蓝色带，**空隙真正拉开**
              （``gap_over_tape_ratio``：空隙 ≥ 带宽的 40%）且中心距够远
              （``min_branch_separation``），并且这个形态最少连续
              ``min_split_rows`` 行。刚分叉时两条带还贴着，那只是被拉宽的粗线，
              普通弯道和实线噪点也不会满足这些条件。
A4            两条分支一直延伸到 ROI 底部（``min_branch_rows``、``min_branch_pixels_below``），
              也就是车头正前方是两选一的开口；分叉点已经贴到画面底部时说明车已经
              开过岔路，这时不触发。
A5            图像中心 = 车头正前方。相机水平视野 ``horizontal_fov_deg`` 默认 70 度，
              用来把像素偏移换算成转向角；这个值只影响转多少，不影响选哪边。
A6            灯的判据由 **3 号（traffic_light.py）** 提供。本模块通过注入的
              ``light_probe(frame, now) -> LightReading`` 读取，自己不认红绿灯
              （``LIGHT_OWNER`` 常量说明这个边界）。没有注入探针时，唯一可用的判据是
              ``fallback_color``，默认 ``"none"`` 表示"没有判据"→ 失败停车。
A7            两条分支都是绿灯、或者灯只报了一个颜色没有报位置时，
              ``fallback_rule`` 决定走哪边；默认 ``"straightest"``（走最接近车头正前方的那边）。
A8            转向是一段有限动作：yaw = 选中分支的偏角 × ``yaw_gain``，被
              ``max_turn_yaw`` 限幅，总时长不超过 ``turn_timeout``；
              转够了并且重新看到线，就 ``COMPLETED`` 交回巡线。
A9            走过一个岔路后 ``rearm_cooldown`` 秒内不再重复触发，避免同一次岔路被处理两遍。
A10           转向时前进速度与转向量成比例（``forward = forward_speed × |yaw| / max_turn_yaw``）：
              没选出分支时 yaw 为 0，前进也必须是 0，也就是"等判据时原地停着"。
              ``forward_speed`` 默认 0.10 m/s，低于骨架的 0.30 m/s 上限。
============  ==========================================================
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
    min_branch_separation: float = 0.10   # 两段中心距 / ROI 宽度，低于此值不算岔路
    #: 最靠近车头处的两条分支间距，至少要是远处间距的这么多倍（真的在张开）。
    min_branch_opening_ratio: float = 1.25
    min_split_rows: int = 2          # 两段形态最少连续多少行
    min_branch_rows: int = 2         # 每条分支最少占据多少采样行
    max_split_fraction: float = 0.80 # 分叉行在 ROI 内的最大纵向位置（再往下就等于车已开过）
    min_branch_pixels_below: int = 30  # 分叉行以下至少还要看得到这么多像素的两条分支
    min_split_row_ratio: float = 0.02
    min_split_row_offset: int = 1    # 距 ROI 顶部的安全边距（行）
    max_branch_center_offset: float = 0.85  # 分支中心相对 ROI 中心的允许偏移
    line_center_deadband: float = 0.18      # 线偏多少还算"在中央"
    line_valid_confidence: float = 0.20     # 低于此置信度的线检测不算数
    min_junction_confidence: float = 0.25   # 综合置信度门槛

    # --- 确认与再触发（见 A8、A9） ---
    confirm_frames: int = 3          # 连续几帧都看到才算真岔路
    confirm_gap_grace: float = 0.25  # 中间漏看到的容忍时间（秒）
    confirm_timeout: float = 2.5     # 确认阶段最长耗时
    rearm_cooldown: float = 2.0      # 走过岔路后的再触发冷却时间（秒）
    decision_timeout: float = 6.0    # 到岔路口后等判据的最长时间
    #: 判定"车已经开过岔路口"需要连续多少帧同时满足：线回来了 + 岔路还在。
    #: 单帧的巧合不算，要连续犯这个矛盾才失败。
    drove_past_frames: int = 3
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
            if runs and x0 - runs[-1][1] <= self.settings.run_merge_gap:
                runs[-1] = (runs[-1][0], x1)
            else:
                runs.append((x0, x1))
        return [
            (x0, x1)
            for x0, x1 in runs
            if (x1 - x0 + 1) >= self.settings.row_min_run
        ]

    def _two_runs(self, row: np.ndarray, width: int) -> Optional[Tuple[Tuple[int, int], Tuple[int, int]]]:
        """这一行是不是"被空隙真正拉开的两段"；空隙不够就当没看见。"""
        runs = self._row_runs(row)
        if len(runs) != 2:
            return None
        first, second = runs
        gap = second[0] - first[1] - 1
        tape = ((first[1] - first[0] + 1) + (second[1] - second[0] + 1)) / 2.0
        if gap < tape * self.settings.gap_over_tape_ratio:
            # 刚分叉的地方两条带还贴着；这时仍是一条（被拉宽的）线。
            return None
        separation = (second[0] - first[1]) / max(width, 1)
        if separation < self.settings.min_branch_separation:
            return None
        return first, second

    # -- 分叉行 -----------------------------------------------------------

    def _find_split(self, mask: np.ndarray, rect: Tuple[int, int, int, int]):
        """从 ROI 顶部往下找第一处稳定分叉行；找不到返回 ``None``。"""
        left, top, right, bottom = rect
        height = bottom - top
        settings = self.settings
        start = top + max(
            1, int(height * settings.min_split_row_ratio), settings.min_split_row_offset
        )
        limit = top + int(height * settings.max_split_fraction)
        limit = max(start, min(limit, bottom - 1))
        confirmed = 0
        for y in range(start, limit + 1):
            if self._two_runs(mask[y, left:right], right - left) is None:
                confirmed = 0
                continue
            confirmed += 1
            if confirmed >= settings.min_split_rows:
                first_row = y - confirmed + 1
                if bottom - first_row < settings.min_branch_pixels_below:
                    # 分叉点已经贴到画面底部，等于车已经开过去了，不处理。
                    return None
                return first_row
        return None

    # -- 分支几何 ---------------------------------------------------------

    def _opens_upward(
        self,
        mask: np.ndarray,
        rect: Tuple[int, int, int, int],
        split_row: int,
        samples: Sequence[int],
    ) -> bool:
        """检查两条分支的间距是否越往近处越大（真正的岔路会张开）。"""
        left, top, right, bottom = rect
        width = right - left
        separations: List[float] = []
        for y in samples:
            pair = self._two_runs(mask[y, left:right], width)
            if pair is None:
                separations.append(float("nan"))
                continue
            separations.append(float(pair[1][0] - pair[0][1]))
        valid = [value for value in separations if value == value]  # 去掉 nan
        span = min(len(valid) // 3, max(1, len(valid) // 4))
        if len(valid) < 2 * span:
            return False
        near = sum(valid[-span:]) / span
        far = sum(valid[:span]) / span
        return near >= far * self.settings.min_branch_opening_ratio

    def _branch_geometry(
        self,
        mask: np.ndarray,
        rect: Tuple[int, int, int, int],
        split_row: int,
    ) -> Tuple[Optional[BranchGeometry], Optional[BranchGeometry]]:
        """统计分叉行以下每一个采样行里两条分支的中心位置。"""
        left, top, right, bottom = rect
        width = right - left
        settings = self.settings
        span = bottom - split_row
        if span <= 0:
            return None, None
        step = max(1, span // 12)
        samples = list(range(split_row + step, bottom, step))
        if not samples:
            samples = [split_row + 1] if split_row + 1 < bottom else []
        if len(samples) < settings.min_branch_rows:
            return None, None

        bucket: Tuple[List[float], List[float], List[float], List[float]] = ([], [], [], [])
        for y in samples:
            pair = self._two_runs(mask[y, left:right], width)
            if pair is None:
                continue
            (a0, a1), (b0, b1) = pair
            bucket[0].append((a0 + a1) / 2.0 + left)
            bucket[1].append(float(y))
            bucket[2].append((b0 + b1) / 2.0 + left)
            bucket[3].append(float(y))
        rows = len(bucket[0])
        if rows < settings.min_branch_rows:
            return None, None

        # 两条分支越往车头方向越张开。两个平行的色块（不是岔路）间距恒定，
        # 会被这一条挡掉。
        if not self._opens_upward(mask, rect, split_row, samples):
            return None, None

        half_width = max(width / 2.0, 1.0)
        per_pixel_deg = settings.horizontal_fov_deg / max(width, 1)
        frame_center = left + width / 2.0
        roi_center = frame_center  # ROI 左右基本对称；用整幅图中心当车头正前方

        result: List[BranchGeometry] = []
        for xs, ys, side in (
            (bucket[0], bucket[1], Branch.LEFT),
            (bucket[2], bucket[3], Branch.RIGHT),
        ):
            center_x = float(np.mean(xs))
            center_y = float(np.mean(ys))
            if abs(center_x - roi_center) / half_width > settings.max_branch_center_offset:
                return None, None
            bearing = (center_x - frame_center) * per_pixel_deg
            result.append(
                BranchGeometry(
                    side=side,
                    center=(center_x, center_y),
                    bearing_deg=float(bearing),
                    rows=rows,
                )
            )
        return result[0], result[1]

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

        split_row = self._find_split(mask, rect)
        if split_row is None:
            self.last_confidence = 0.0
            return JunctionDetection.no_result("no two-run row (curve or solid line)", mask)

        pair = self._two_runs(mask[split_row, left:right], right - left)
        if pair is None:
            self.last_confidence = 0.0
            return JunctionDetection.no_result("split row lost", mask)
        split_center = (pair[0][1] + pair[1][0]) / 2.0 + left

        left_branch, right_branch = self._branch_geometry(mask, rect, split_row)
        if left_branch is None or right_branch is None:
            self.last_confidence = 0.0
            return JunctionDetection.no_result("branches too short", mask)

        height = max(bottom - top, 1)
        row_ratio = (split_row - top) / height
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
            split_row=int(split_row),
            split_center_x=int(round(split_center)),
            box=box,
            branches=(left_branch, right_branch),
            confidence=confidence,
            mask=mask,
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
        :param line: 可选，框架本帧的巡线结果；用来判断"转向完线回来了没有"。
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
            return self._step_turn(detection, now)
        return self._step_settle(detection, frame, now)

    # -- 状态实现 ---------------------------------------------------------

    def _handle_missing_image(self, now: float) -> TaskUpdate:
        if self.active:
            return self._failed("frame image missing while owning control")
        return self._not_triggered("no frame")

    def _step_idle(self, detection: JunctionDetection, now: float) -> TaskUpdate:
        if self._idle_since is None:
            self._idle_since = now
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

        # "线在画面中央" + "岔路形态还在"连续出现 → 车其实已经开过了岔路口，
        # 只是没发现。这时没有分支可选，必须失败停车，不能瞎选一边。
        if detection.valid and line_is_centered(
            self.last_line,
            settings.line_center_deadband,
            settings.line_valid_confidence,
        ):
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

    def _step_turn(self, detection: JunctionDetection, now: float) -> TaskUpdate:
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

        settled = (
            self._turn_elapsed(now) >= settings.turn_min_duration
            and line_is_centered(
                self.last_line,
                settings.line_center_deadband,
                settings.line_valid_confidence,
            )
        )
        if settled:
            self._enter(JunctionState.SETTLE, now)
            return self._running(now, "branch entered, checking line stability")

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

        stable = line_is_centered(
            self.last_line,
            settings.line_center_deadband,
            settings.line_valid_confidence,
        )
        if stable and self._turn_elapsed(now) >= settings.turn_min_duration:
            self._rearm_ready_at = now + settings.rearm_cooldown
            return self._completed("junction passed; line reacquired")

        if self._elapsed(now) > settings.settle_timeout:
            return self._failed("line did not return after the turn")

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
