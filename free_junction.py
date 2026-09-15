"""无拥堵岔路（WP6b / Issue #6）—— 7 号 王炜嘉。

走到岔路口 → 判断哪一侧没有拥堵 → 选边进入 → 完成后交回巡线。
**本模块不管灯色**：灯是 3 号（`traffic_light.py`）和 6 号（`green_junction.py`）的边界，
这里只做"拥堵判据"。

它只做三件事
------------
1. 从共享帧里判断"前方是不是一个岔路口"（只在自己的 ROI 里干活）；
2. 按判据选出要走的那一边（**判据不成立就停车报失败，绝不瞎选**）；
3. 输出受限的 `MotionCommand` 请求与 `TaskUpdate` 状态，动作拆成每帧一小步。

它不做的事（项目红线）
----------------------
* 不连相机、不连机器人、不调用任何 SDK；
* 不直接给底盘下命令、不绕过唯一运动出口；
* 不阻塞、不死循环，`step()` 每次调用立刻返回；
* 不改 `models.py` / `camera_source.py` / `runtime.py` / `motion_output.py` / `main.py`。

场地假设（正式规则还没有可靠来源，所以先自己拍板，全部做成可改参数）
--------------------------------------------------------------------
参数都在 `FreeJunctionConfig` 里，任何一条都能单独改，**逻辑里没有写死数字**；
运行时判据不成立就 `FAILED` 停车，宁可不动也不瞎动。

============  ==========================================================
假设编号       内容
============  ==========================================================
A1            岔路 = 蓝色巡线带一分为二。HSV 区间沿用 `config.VisionConfig`
              的已验证默认值；正式赛道换颜色/材质时只改这里。
A2            分叉形态：某一行上出现两段蓝带，且**空隙真正拉开**
              （`min_gap_over_tape`：空隙 ≥ 带宽的 40%）并且中心距够远
              （`min_branch_separation_px`）。同一形态最少要出现
              `min_fork_rows` 行，才算是岔路。
A3            两条分支必须**向上延伸到 ROI 顶部**（`top_band_ratio`、
              `min_branch_pixels`）：刚分叉时两段带子还贴着，那只是被拉宽的
              粗线；普通弯道、实线噪点、断线缺口都不满足 A2+A3。
A4            岔路连续出现 `confirm_frames` 帧才触发；只闪一帧不算。
A5            "拥堵"没有可靠来源（TASKS.md 把它列为尚待确认/可降级项），
              所以本模块给出两种可切换的判据，**默认都不猜**：
                * `decision_rule="vision"`：看分叉行上方那条带子里，左右两侧
                  "可走像素"（够亮的地面 + 蓝线）的比例；被占比例超过
                  `blocked_ratio` 的一侧判为堵，走另一侧。两侧都通且差距小于
                  `free_margin`、或者两侧都堵 → **判据不成立 → 停车报失败**。
                * `decision_rule="fixed"`：固定走 `fixed_branch`（场地规则明确
                  "永远走某一侧"时用这个）。
              `fallback_branch` 是"视觉判不出时改走某一边"的兜底；默认 `None`
              （不兜底，判不出就停）。
A6            选路是一段有限动作：对准（APPROACH）→ 转向（TURN）→ 出岔路
              （EXIT），每段都有独立超时；总时长不超过 `max_task_seconds`，
              而且必须短于骨架的 20 秒硬上限。
A7            转向符号沿用项目约定：**yaw 正值右转、负值左转**（`MotionCommand`
              注释），转向量按选中分支偏角 × `yaw_gain` 计算并限幅。
A8            走过一个岔路后 `rearm_cooldown` 秒内、并且要等岔路从画面里
              消失 `rearm_clear_frames` 帧，才允许处理下一个岔路，
              避免同一次岔路被处理两遍、原地反复触发。
A9            接管期间 ROI 里连续 `lost_line_frames` 帧一条蓝线都没有 →
              立刻停车报失败（`abort_on_line_lost`），绝不盲开。
A10           等判据期间车是**原地停着**的（DECIDE 阶段零速度），
              只有判据成立才允许往前动。

未验证项（写清楚，不装）
------------------------
* 没有实车、没有正式场地：所有阈值（HSV、`floor_min_v`、`blocked_ratio`、
  `min_branch_separation_px`）都是在合成画面上定的，必须在正式录像/实车重新标定；
* "拥堵"的物理定义（另一侧有车？有路障？）没有可靠来源，A5 的视觉判据是
  自定假设；
* 转向符号（A7）与转向时长没有实车确认，首次上车必须**架空车轮**验证符号；
* 岔路几何（分叉角度、到分叉点的距离）没有场地数据，A6 的时长是估计值。

单独自测
--------
    python scripts/check_module.py free_junction
    python -m unittest tests.test_free_junction -v
"""

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from models import (
    FramePacket,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)

#: 本模块的检测类别名。
KIND = "free_junction"

#: 两条分支的取值，同时也是 `VisualDetection.target_id` 的取值。
BRANCH_LEFT = "left"
BRANCH_RIGHT = "right"

#: 零速度。未触发、等待判据、完成、失败都返回它。
STOP = MotionCommand()


# ---------------------------------------------------------------------------
# 状态
# ---------------------------------------------------------------------------


class JunctionState(Enum):
    """任务状态机。`IDLE` 之外的阶段都算"正在接管"。"""

    IDLE = "idle"              # 没看到岔路 / 还在等上一次的封锁解除
    DECIDE = "decide"          # 岔路已确认，原地等判据
    APPROACH = "approach"      # 对准选中分支，每帧只修一点点
    TURN = "turn"              # 有限转向
    EXIT = "exit"              # 直行离开岔路口
    COMPLETED = "completed"    # 走完了
    FAILED = "failed"          # 判据不成立 / 超时 / 丢线，停车


class Branch(Enum):
    """岔路的两条分支。"""

    LEFT = "left"
    RIGHT = "right"

    @property
    def sign(self) -> float:
        """转向符号：按项目约定 yaw 正值右转、负值左转。"""
        return 1.0 if self is Branch.RIGHT else -1.0


# ---------------------------------------------------------------------------
# 参数
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FreeJunctionConfig:
    """free_junction 的全部可调参数（默认值与 `config.VisionConfig` 对齐）。"""

    # ---- ROI：只在画面下半部这块里干活，整张图不做重算法 ----
    roi_left: float = 0.06
    roi_top: float = 0.54
    roi_right: float = 0.94
    roi_bottom: float = 0.96

    # ---- 蓝线颜色管线（沿用已验证的蓝色胶带默认值）----
    hsv_lower: Tuple[int, int, int] = (95, 80, 60)
    hsv_upper: Tuple[int, int, int] = (135, 255, 255)
    open_kernel: int = 3
    # 故意比 line_detector 的默认值小：闭运算核太大（例如 17）会把岔路的
    # 两条分支重新粘成一条，那样就检测不出分叉了。
    close_kernel: int = 5
    min_blue_ratio: float = 0.004   # ROI 里蓝线占比低于它，当作"这一帧没有线"

    # ---- 岔路几何（A2 / A3 / A4）----
    scan_row_ratio: float = 0.50      # 在 ROI 高度 50% 处横扫
    scan_rows: int = 7                # 横扫几行
    merge_gap_px: int = 6             # 同一行里隔得比它近的蓝色像素算同一段
    min_run_px: int = 5               # 短于它的碎块丢掉
    min_branch_separation_px: int = 45   # 两段带子的中心距下限
    min_gap_over_tape: float = 0.40      # 空隙 ≥ 带宽的 40% 才算真分开
    min_fork_rows: int = 3               # 最少要有几行满足分叉形态
    confirm_frames: int = 4              # 连续几帧看到岔路才触发
    rearm_clear_frames: int = 8          # 岔路消失几帧后才允许再次触发
    top_band_ratio: float = 0.30         # 用 ROI 上方 30% 检查分支是否延伸上来
    min_branch_pixels: int = 12          # 上带里每一侧至少要有多少蓝像素
    require_branches_reach_top: bool = True

    # ---- 拥堵判据（A5）----
    decision_rule: str = "vision"     # "vision" | "fixed"
    fixed_branch: Optional[str] = None    # decision_rule="fixed" 时走哪边
    fallback_branch: Optional[str] = None  # 视觉判不出时走哪边；None = 停车报失败
    floor_min_v: int = 90             # 亮于它算可走（地面/胶带），暗于它算被占住
    blocked_ratio: float = 0.45       # 一侧被占比例 ≥ 它 → 判为堵
    free_margin: float = 0.08         # 两侧都通时，差距超过它才敢挑一边
    side_band_ratio: float = 0.45     # 用分叉行上方这条带子判拥堵

    # ---- 动作（A6 / A7，全部远低于骨架限幅）----
    decide_timeout: float = 0.6       # 原地等判据的最长时间
    approach_forward: float = 0.10
    approach_yaw_gain: float = 45.0
    max_approach_yaw: float = 25.0
    approach_timeout: float = 1.5
    turn_forward: float = 0.06
    turn_yaw: float = 45.0
    turn_seconds: float = 1.2
    turn_timeout: float = 2.5
    exit_forward: float = 0.15
    exit_seconds: float = 1.0
    exit_timeout: float = 2.0

    # ---- 自己先裁一遍限幅（骨架还会再裁一次）----
    max_forward: float = 0.30
    max_lateral: float = 0.25
    max_yaw: float = 90.0

    # ---- 安全与时效（A8 / A9）----
    max_task_seconds: float = 12.0    # 骨架 20 秒硬上限，这里留足余量
    rearm_cooldown: float = 2.0
    lost_line_frames: int = 3
    abort_on_line_lost: bool = True


# ---------------------------------------------------------------------------
# 检测结果
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ForkDetection:
    """一帧的岔路检测结果。坐标都是**整幅图像**像素（与 `VisualDetection` 一致）。"""

    valid: bool
    split_row: int = 0
    split_x: int = 0
    left_x: int = 0
    right_x: int = 0
    gap_px: int = 0
    separation_px: int = 0
    left_free: float = 0.0
    right_free: float = 0.0
    confidence: float = 0.0
    box: Tuple[int, int, int, int] = (0, 0, 0, 0)
    #: ROI 里蓝线像素占比。即使判不出岔路也会填，用来判断"是不是整条线都不见了"。
    blue_ratio: float = 0.0
    #: 整幅图像宽度，用来把像素偏差换算成归一化误差（对准阶段要用）。
    frame_width: int = 0

    @classmethod
    def empty(cls) -> "ForkDetection":
        return cls(valid=False)


def _clamp(value: float, low: float, high: float) -> float:
    """把数裁进 [low, high]；nan / inf 一律变成 0。"""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not np.isfinite(number):
        return 0.0
    return max(low, min(high, number))


def _row_runs(row_mask: np.ndarray, merge_gap: int, min_run: int) -> List[Tuple[int, int]]:
    """把一行二值像素切成若干段，返回 [(起点, 终点), ...]。

    只用 numpy 算，不写逐像素循环：一行几百个像素是微秒级的事。
    """
    indexes = np.flatnonzero(row_mask)
    if indexes.size == 0:
        return []
    cut = np.flatnonzero(np.diff(indexes) > max(1, int(merge_gap)))
    starts = np.concatenate(([0], cut + 1))
    ends = np.concatenate((cut, [indexes.size - 1]))
    runs: List[Tuple[int, int]] = []
    for start, end in zip(starts, ends):
        if int(indexes[end]) - int(indexes[start]) + 1 >= max(1, int(min_run)):
            runs.append((int(indexes[start]), int(indexes[end])))
    return runs


def _spread_rows(center: int, height: int, count: int) -> List[int]:
    """在 center 附近取 count 行（夹在 [0, height-1] 内）。"""
    count = max(1, int(count))
    height = int(height)
    if height <= 1:
        return [0]
    if count == 1:
        return [max(0, min(height - 1, int(center)))]
    low = max(0, int(center) - count // 2)
    high = min(height - 1, low + count - 1)
    low = max(0, high - count + 1)
    return list(range(low, high + 1))


# ---------------------------------------------------------------------------
# 岔路检测器
# ---------------------------------------------------------------------------


class FreeJunctionDetector:
    """只做一件事：在一张 BGR 图上找"一分为二"的蓝色带子。

    不做拥堵判断（那是任务层的判据），也不改任何状态。
    """

    def __init__(self, settings: Optional[FreeJunctionConfig] = None) -> None:
        self.settings = settings if settings is not None else FreeJunctionConfig()

    # -- 颜色 -------------------------------------------------------------

    @staticmethod
    def _kernel(size: int) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def blue_mask(self, roi: np.ndarray) -> np.ndarray:
        """ROI 里的蓝色带子掩码（uint8 0/255）。"""
        settings = self.settings
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array(settings.hsv_lower, dtype=np.uint8),
            np.array(settings.hsv_upper, dtype=np.uint8),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel(settings.open_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel(settings.close_kernel))
        return mask

    # -- 主入口 -----------------------------------------------------------

    def detect(self, image: Optional[np.ndarray]) -> ForkDetection:
        """返回这一帧的岔路检测；不是岔路就返回 `ForkDetection.empty()`。"""
        settings = self.settings
        if image is None:
            return ForkDetection.empty()
        frame = np.asarray(image)
        if frame.ndim != 3 or frame.shape[2] != 3:
            return ForkDetection.empty()
        height, width = frame.shape[:2]
        left = int(width * settings.roi_left)
        right = int(width * settings.roi_right)
        top = int(height * settings.roi_top)
        bottom = int(height * settings.roi_bottom)
        if right - left < 16 or bottom - top < 16:
            return ForkDetection.empty()
        roi = frame[top:bottom, left:right]
        mask = self.blue_mask(roi)
        roi_height, roi_width = mask.shape[:2]
        blue_ratio = float(np.count_nonzero(mask)) / float(max(1, mask.size))

        scan_row = int(_clamp(settings.scan_row_ratio, 0.0, 1.0) * (roi_height - 1))
        fork_rows: List[Tuple[int, Tuple[int, int], Tuple[int, int]]] = []
        for row in _spread_rows(scan_row, roi_height, settings.scan_rows):
            runs = _row_runs(mask[row] > 0, settings.merge_gap_px, settings.min_run_px)
            if len(runs) < 2:
                continue
            first, last = runs[0], runs[-1]
            separation = ((last[0] + last[1]) - (first[0] + first[1])) / 2.0
            tape = min(first[1] - first[0] + 1, last[1] - last[0] + 1)
            gap = last[0] - first[1]
            if separation < settings.min_branch_separation_px:
                continue
            if gap < settings.min_gap_over_tape * tape:
                continue
            fork_rows.append((row, first, last))

        if len(fork_rows) < max(1, int(settings.min_fork_rows)):
            return ForkDetection(valid=False, blue_ratio=blue_ratio)

        # 取分离得最开的那一行作为分叉行。
        fork_rows.sort(key=lambda item: (item[2][0] + item[2][1]) - (item[1][0] + item[1][1]))
        row, first, last = fork_rows[-1]

        if settings.require_branches_reach_top:
            if not self._branches_reach_top(mask):
                return ForkDetection(valid=False, blue_ratio=blue_ratio)

        left_x = int(round((first[0] + first[1]) / 2.0)) + left
        right_x = int(round((last[0] + last[1]) / 2.0)) + left
        split_x = int(round((left_x + right_x) / 2.0))
        split_row = row + top
        left_free, right_free = self._free_ratios(mask, roi, row, first, last)
        separation = right_x - left_x

        confidence = min(
            1.0,
            separation / max(1.0, float(settings.min_branch_separation_px) * 2.0),
        )
        box = (
            max(0, left_x - 20),
            top,
            min(width, right_x + 20),
            bottom,
        )
        return ForkDetection(
            valid=True,
            split_row=split_row,
            split_x=split_x,
            left_x=left_x,
            right_x=right_x,
            gap_px=int(max(0, last[0] - first[1])),
            separation_px=int(separation),
            left_free=left_free,
            right_free=right_free,
            confidence=confidence,
            box=box,
            blue_ratio=blue_ratio,
            frame_width=width,
        )

    # -- 内部 -------------------------------------------------------------

    def _branches_reach_top(self, mask: np.ndarray) -> bool:
        """A3：左右两半在 ROI 顶部那条带子里都必须有蓝像素。"""
        settings = self.settings
        roi_height, roi_width = mask.shape[:2]
        band = max(1, int(_clamp(settings.top_band_ratio, 0.05, 1.0) * roi_height))
        middle = roi_width // 2
        if middle < 2 or roi_width - middle < 2:
            return False
        left_pixels = int(np.count_nonzero(mask[:band, :middle]))
        right_pixels = int(np.count_nonzero(mask[:band, middle:]))
        need = max(1, int(settings.min_branch_pixels))
        return left_pixels >= need and right_pixels >= need

    def _free_ratios(
        self,
        mask: np.ndarray,
        roi: np.ndarray,
        split_row: int,
        first: Tuple[int, int],
        last: Tuple[int, int],
    ) -> Tuple[float, float]:
        """分叉行上方那条带子里，左右两侧"可走像素"的比例（0~1，越大越通）。"""
        settings = self.settings
        roi_height, roi_width = mask.shape[:2]
        band_top = int(max(0, split_row - _clamp(settings.side_band_ratio, 0.05, 1.0) * roi_height))
        band_bottom = int(max(band_top + 1, min(roi_height, split_row)))
        divider = int(round((first[0] + last[0]) / 2.0))
        divider = max(4, min(roi_width - 4, divider))
        if band_bottom - band_top < 2:
            return 0.0, 0.0

        brightness = roi.max(axis=2)          # 便宜又够用的亮度近似
        walkable = np.logical_or(brightness >= int(settings.floor_min_v), mask > 0)
        left_band = walkable[band_top:band_bottom, :divider]
        right_band = walkable[band_top:band_bottom, divider:]
        if left_band.size == 0 or right_band.size == 0:
            return 0.0, 0.0
        return float(left_band.mean()), float(right_band.mean())


# ---------------------------------------------------------------------------
# 任务
# ---------------------------------------------------------------------------


class FreeJunctionTask:
    """7 号的模块主体：插进 coordinator 就能用，单独也能测。

    用法::

        task = FreeJunctionTask()
        update = task.step(frame_packet, now)      # 每帧一次，立即返回

    死规矩（红线 8）：一旦返回 ``RUNNING``，就必须持续返回 ``RUNNING``，
    直到 ``COMPLETED`` 或 ``FAILED``，中间不会改口成 ``NOT_TRIGGERED``。
    """

    name = "free_junction"

    def __init__(
        self,
        settings: Optional[FreeJunctionConfig] = None,
        detector: Optional[FreeJunctionDetector] = None,
    ) -> None:
        self.settings = settings if settings is not None else FreeJunctionConfig()
        self.detector = (
            detector if detector is not None else FreeJunctionDetector(self.settings)
        )
        self.state = JunctionState.IDLE
        self.last_detection: Optional[ForkDetection] = None
        self.last_visual = VisualDetection.no_result(KIND)
        self.chosen_branch: Optional[Branch] = None
        self.last_reason = "idle"
        self.last_message = "idle"
        self.last_outcome: Optional[TaskStatus] = None

        self._confirm_count = 0
        self._clear_count = 0
        self._lost_count = 0
        self._last_blue_ratio = 0.0
        self._state_since: Optional[float] = None
        self._run_started_at: Optional[float] = None
        self._rearm_ready_at: Optional[float] = None

    # -- 对外状态 ---------------------------------------------------------

    @property
    def active(self) -> bool:
        """``True`` 表示本模块正在接管（coordinator 据此保持 / 交出控制权）。"""
        return self.state in (
            JunctionState.DECIDE,
            JunctionState.APPROACH,
            JunctionState.TURN,
            JunctionState.EXIT,
        )

    @property
    def finished(self) -> bool:
        return self.state in (JunctionState.COMPLETED, JunctionState.FAILED)

    def reset(self) -> None:
        """回到初始状态，供 coordinator 复位时调用。"""
        self.state = JunctionState.IDLE
        self.last_detection = None
        self.last_visual = VisualDetection.no_result(KIND)
        self.chosen_branch = None
        self.last_reason = "idle"
        self.last_message = "idle"
        self.last_outcome = None
        self._confirm_count = 0
        self._clear_count = 0
        self._lost_count = 0
        self._last_blue_ratio = 0.0
        self._state_since = None
        self._run_started_at = None
        self._rearm_ready_at = None

    def detect(self, frame: Optional[np.ndarray]) -> VisualDetection:
        """给外面看的检测结果：岔路位置 + 判出的可通行一侧。

        不是岔路、或拥堵判据不成立时返回 ``VisualDetection.no_result(KIND)``，
        不用虚构坐标冒充结果。
        """
        fork = self.detector.detect(frame)
        self.last_detection = fork if fork.valid else None
        branch: Optional[Branch] = None
        if fork.valid:
            branch, reason = self._choose_branch(fork)
            self.last_reason = reason
        else:
            self.last_reason = "no junction"
        self.last_visual = self._visual(fork, branch)
        return self.last_visual

    # -- 主入口 -----------------------------------------------------------

    def step(self, frame: Optional[FramePacket], now: float) -> TaskUpdate:
        """每来一张新帧调用一次，立即返回。"""
        moment = float(now)
        if frame is None or getattr(frame, "image", None) is None:
            return self._handle_missing_image(moment)

        fork = self.detector.detect(frame.image)
        self.last_detection = fork if fork.valid else None
        self._last_blue_ratio = fork.blue_ratio
        self.last_visual = self._visual(fork, self.chosen_branch)

        if self.state is JunctionState.IDLE:
            return self._step_idle(fork, moment)
        return self._step_active(fork, moment)

    # -- IDLE -------------------------------------------------------------

    def _step_idle(self, fork: ForkDetection, now: float) -> TaskUpdate:
        settings = self.settings

        if self._rearm_ready_at is not None:
            self._clear_count = 0 if fork.valid else self._clear_count + 1
            cooling = now < self._rearm_ready_at
            not_cleared = self._clear_count < max(1, int(settings.rearm_clear_frames))
            if cooling or not_cleared:
                self._confirm_count = 0
                return self._not_triggered("rearm gate")
            self._rearm_ready_at = None
            self._clear_count = 0

        if not fork.valid:
            self._confirm_count = 0
            return self._not_triggered("no junction")

        self._confirm_count += 1
        need = max(1, int(settings.confirm_frames))
        if self._confirm_count < need:
            return self._not_triggered(
                "confirming junction %d/%d" % (self._confirm_count, need)
            )

        # 判据成立就进入等判据阶段；判不出就原地停着等，超时再失败（A5/A10）。
        self.chosen_branch, reason = self._choose_branch(fork)
        self.last_reason = reason
        self._enter(JunctionState.DECIDE, now)
        self._run_started_at = now
        return self._running(now, "junction confirmed, deciding (%s)" % reason)

    # -- 接管中的状态机 ---------------------------------------------------

    def _step_active(self, fork: ForkDetection, now: float) -> TaskUpdate:
        settings = self.settings

        if self._run_started_at is not None and now - self._run_started_at > settings.max_task_seconds:
            return self._fail(now, "task time budget exceeded")

        if settings.abort_on_line_lost:
            if fork.valid or self._blue_present():
                self._lost_count = 0
            else:
                self._lost_count += 1
                if self._lost_count >= max(1, int(settings.lost_line_frames)):
                    return self._fail(now, "line lost while owning control")

        if self.state is JunctionState.DECIDE:
            if fork.valid:
                self.chosen_branch, reason = self._choose_branch(fork)
                self.last_reason = reason
            if self.chosen_branch is not None:
                self._enter(JunctionState.APPROACH, now)
                return self._running(now, "taking the %s branch" % self.chosen_branch.value)
            if self._elapsed(now) >= settings.decide_timeout:
                return self._fail(now, "criterion unavailable: %s" % self.last_reason)
            return self._running(now, "waiting for a usable criterion")

        if self.state is JunctionState.APPROACH:
            gone = self._fork_gone(fork)
            if gone or self._elapsed(now) >= settings.approach_timeout * 0.75:
                self._enter(JunctionState.TURN, now)
            else:
                return self._running(now, "aligning with the %s branch" % self._branch_name())

        if self.state is JunctionState.TURN:
            if self._elapsed(now) >= settings.turn_timeout:
                return self._fail(now, "turn timed out")
            if self._elapsed(now) >= settings.turn_seconds:
                self._enter(JunctionState.EXIT, now)
            else:
                return self._running(now, "turning into the %s branch" % self._branch_name())

        return self._finish_exit(now)

    # -- 判据 -------------------------------------------------------------

    def _choose_branch(self, fork: ForkDetection) -> Tuple[Optional[Branch], str]:
        """返回 (分支, 理由)。分支为 None 表示判据不成立 —— 那就别动。"""
        settings = self.settings

        if settings.decision_rule == "fixed":
            fixed = self._as_branch(settings.fixed_branch)
            if fixed is None:
                return None, "fixed rule without a side configured"
            return fixed, "fixed rule"

        if settings.decision_rule != "vision":
            return None, "unknown decision rule %r" % (settings.decision_rule,)

        occupied_left = 1.0 - fork.left_free
        occupied_right = 1.0 - fork.right_free
        left_blocked = occupied_left >= settings.blocked_ratio
        right_blocked = occupied_right >= settings.blocked_ratio

        if left_blocked and not right_blocked:
            return Branch.RIGHT, "left side blocked"
        if right_blocked and not left_blocked:
            return Branch.LEFT, "right side blocked"
        if not left_blocked and not right_blocked:
            if fork.left_free - fork.right_free >= settings.free_margin:
                return Branch.LEFT, "both open, left clearly freer"
            if fork.right_free - fork.left_free >= settings.free_margin:
                return Branch.RIGHT, "both open, right clearly freer"
            return self._fallback("both sides open, no clear winner")
        return self._fallback("both sides blocked")

    def _fallback(self, reason: str) -> Tuple[Optional[Branch], str]:
        branch = self._as_branch(self.settings.fallback_branch)
        if branch is None:
            return None, reason
        return branch, reason + " (fallback)"

    @staticmethod
    def _as_branch(value: Optional[str]) -> Optional[Branch]:
        for branch in Branch:
            if value == branch.value:
                return branch
        return None

    # -- 运动请求 ---------------------------------------------------------

    def _motion(self, now: float) -> MotionCommand:
        settings = self.settings
        if self.state is JunctionState.DECIDE:
            # 等判据期间原地停着，绝不自己往前冲（A10）。
            return STOP
        if self.state is JunctionState.APPROACH:
            return MotionCommand(
                forward=_clamp(settings.approach_forward, 0.0, settings.max_forward),
                lateral=0.0,
                yaw=_clamp(self._approach_yaw(), -settings.max_approach_yaw, settings.max_approach_yaw),
            )
        if self.state is JunctionState.TURN:
            yaw = settings.turn_yaw * self._branch_sign()
            return MotionCommand(
                forward=_clamp(settings.turn_forward, 0.0, settings.max_forward),
                lateral=0.0,
                yaw=_clamp(yaw, -settings.max_yaw, settings.max_yaw),
            )
        return MotionCommand(
            forward=_clamp(settings.exit_forward, 0.0, settings.max_forward),
            lateral=0.0,
            yaw=0.0,
        )

    def _approach_yaw(self) -> float:
        """对准阶段：往选中分支偏一点点，每帧只修一点（A7）。

        误差以**整幅图像宽度**归一化（和 `LineDetection.error` 同口径）：
        目标在右半边 → 误差为正 → yaw 为正（正值右转）。
        """
        settings = self.settings
        fork = self.last_detection
        if fork is None or self.chosen_branch is None or fork.frame_width <= 0:
            return 0.0
        target = fork.left_x if self.chosen_branch is Branch.LEFT else fork.right_x
        half_width = max(1.0, fork.frame_width / 2.0)
        error = (target - fork.frame_width / 2.0) / half_width
        return settings.approach_yaw_gain * error

    def _branch_sign(self) -> float:
        if self.chosen_branch is None:
            return 0.0
        return self.chosen_branch.sign

    def _branch_name(self) -> str:
        return self.chosen_branch.value if self.chosen_branch is not None else "unknown"

    # -- 每帧小工具 -------------------------------------------------------

    def _blue_present(self) -> bool:
        """ROI 里还有没有蓝线（用来判断"接管中是不是整条线都不见了"）。"""
        fork = self.last_detection
        if fork is not None and fork.valid:
            return True
        return self._last_blue_ratio >= self.settings.min_blue_ratio

    def _fork_gone(self, fork: ForkDetection) -> bool:
        """分叉是否已经消失（说明车已经开进分支里了）。"""
        if fork.valid:
            self._clear_count = 0
            return False
        self._clear_count += 1
        return self._clear_count >= 2

    def _finish_exit(self, now: float) -> TaskUpdate:
        """EXIT 阶段：直行一小段，走完就算完成，走太久就失败。"""
        if self._elapsed(now) >= self.settings.exit_timeout:
            return self._fail(now, "exit timed out")
        if self._elapsed(now) >= self.settings.exit_seconds:
            return self._complete(now, "junction cleared")
        return self._running(now, "leaving the junction")

    def _elapsed(self, now: float) -> float:
        if self._state_since is None:
            return 0.0
        return max(0.0, now - self._state_since)

    def _enter(self, state: JunctionState, now: float) -> None:
        self.state = state
        self._state_since = now
        self._lost_count = 0
        self._clear_count = 0

    # -- 输出 -------------------------------------------------------------

    def _visual(
        self, fork: ForkDetection, branch: Optional[Branch] = None
    ) -> VisualDetection:
        if not fork.valid:
            return VisualDetection.no_result(KIND)
        chosen = branch if branch is not None else self.chosen_branch
        return VisualDetection(
            valid=True,
            kind=KIND,
            center=(int(fork.split_x), int(fork.split_row)),
            target_id=chosen.value if chosen is not None else None,
            color=None,
            confidence=float(fork.confidence),
            box=fork.box,
        )

    def _not_triggered(self, message: str) -> TaskUpdate:
        self.last_message = message
        return TaskUpdate(
            TaskStatus.NOT_TRIGGERED,
            motion=STOP,
            detection=self.last_visual,
            message=message,
        )

    def _running(self, now: float, message: str) -> TaskUpdate:
        self.last_message = message
        self.last_visual = self._visual(
            self.last_detection if self.last_detection is not None else ForkDetection.empty(),
            self.chosen_branch,
        )
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=self._motion(now),
            detection=self.last_visual,
            message=message,
        )

    def _complete(self, now: float, message: str) -> TaskUpdate:
        return self._settle(now, TaskStatus.COMPLETED, message)

    def _fail(self, now: float, message: str) -> TaskUpdate:
        return self._settle(now, TaskStatus.FAILED, message)

    def _settle(self, now: float, status: TaskStatus, message: str) -> TaskUpdate:
        """收尾：零速度、回到 IDLE，并记下封锁时间。

        交回控制权之后再被问到时，`_step_idle` 会先过 A8 的封锁闸门
        （冷却 + 等岔路从画面里消失），所以不会原地反复触发。
        """
        self.state = JunctionState.IDLE
        self.last_outcome = status
        self.last_message = message
        # 故意不覆盖 last_reason：选边的理由要留着当证据（PR / 复盘要用）。
        self._state_since = None
        self._run_started_at = None
        self._confirm_count = 0
        self._clear_count = 0
        self._lost_count = 0
        self._rearm_ready_at = now + float(self.settings.rearm_cooldown)
        return TaskUpdate(
            status,
            motion=STOP,
            detection=self.last_visual,
            message=message,
        )

    def _handle_missing_image(self, now: float) -> TaskUpdate:
        if self.active:
            return self._fail(now, "frame image missing while owning control")
        return self._not_triggered("no frame")
