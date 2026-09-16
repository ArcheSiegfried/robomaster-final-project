"""无拥堵岔路（WP6b / Issue #6）—— 7 号 王炜嘉。

走到岔路口 → 判断哪一条分支上停着车（"拥堵"）→ 走另一条 → 完成后交回巡线。
**本模块不管灯色**：灯是 3 号（`traffic_light.py`）和 6 号（`green_junction.py`）的边界。

本次修改（v2）依据两件事
------------------------

**1. 官方定义（2026-09 明确）**："拥堵" = **一条分支上停着一辆静止的同型小车**
（另一台 RoboMaster 同型车，约 30~32 cm 长、24 cm 宽，不会移动）。
所以判据不再是"这一片暗不暗"（v1 的老毛病：亮色/彩色障碍看不见、一条阴影就能让两侧都判堵），
而是**在分支走廊里找一辆车**。

**2. 2026-09-15 实车测试报告**（`王炜嘉_实车测试报告.md`）：

* 报告查到的"接管 20 次、0 次收尾"根因在**集成层**（`consumer_wait_timeout` 12 ms 与
  30 fps 相机不匹配 → `video_gap()` 无条件踢任务），集成负责人已在 `d8c2b85` 修掉；
* 报告要我自查的那一点是真的 bug：**被迫结束时协调器会调用 `reset()`**
  （见 `coordinator._reset_task`），而 v1 的 `reset()` 把"封锁闸门"一起清掉了 →
  被踢出来 4 帧后又立刻触发 → 同一次运行里反复接管。**v2 的 `reset()` 会重新拉起封锁闸门**；
* 报告还提到"运行记录没写模块给的 `message`"——集成侧已在补。所以 v2 把
  **判据读数写进 `TaskUpdate.message`**，实车记录里能直接看到"为什么这么选"。

**3. 2026-09-16 两次实车**（16:52:53 / 16:56:06，`captures/run_*` + 屏幕录制，
用户提供的 `run_wrong/`）。16:52:53 那次的运行记录里写着
`junction confirmed, deciding (right branch …)` —— 把**空着的那条**报成"有车"，
于是去走了真堵的那条（画面证据：车确实往左拐，朝那辆停着的车去了）。
把当时的现场画面拿出来跑判据，查到一个 v2 的硬伤：

* **检测区域不对**：停着的那辆车在画面**上方**（y≈36~159），而 v2 的走廊只在岔路
  ROI（y 0.54~0.96）里面 —— 车整个在区域外，"真堵的那条反而看不见"。
  → v3 改成**整帧高度的一条检测带**（`blockage_top_ratio` 0.10 ~
  `blockage_bottom_ratio` 0.86），下沿压到 0.86 是为了排掉我们自己车头常驻的橙色轮子。
* **判据被地砖纹理淹没**：现场是**花岗岩地砖**，细密黑点让 Canny 连成"一整片地毯"
  （结构像素占 67%~80%，一个框就把整条走廊圈住），真车反而没有独立轮廓。
  → v3 默认改成**大尺度局部对比度**：车 = "比周围地面暗一大块（或亮一大块）"，
  地砖黑点/反光是 1~3 像素的小尺度纹理，被大核平均掉；Canny 结构判据降级为可选
  （`use_structure=False`）。形状闸门也按真车数据重新标定。
  真车画面实测（4 帧结果一致）：停着车那条 → 宽 0.39 高 0.36 面积 0.14 下沿 0.44；
  空着那条 → 宽 0.15~0.28 高 0.06~0.16 面积 0.02~0.03 下沿 0.14~0.21。
  同一次修复还顺手修了：**转弯/出岔路时把"胶带跑出 ROI"误判成丢线而停车**
  （真车两次都在 TURN/EXIT 里走 0.8 秒就 FAILED），以及**原地多转几十度**。

职责边界（违反会被 `tests/test_task_contract.py` 直接拦下）
----------------------------------------------------------
* 只接收主流程给的 `FramePacket`，绝不自己开相机或视频流；
* 只返回 `TaskUpdate` / `MotionCommand` / `VisualDetection`，绝不调用 SDK，
  也不接触底盘的唯一运动出口；
* `step()` 必须立刻返回：选路动作拆成每帧一小步，不阻塞。

必须实现的接口（签名已冻结，不要改）::

    class FreeJunctionTask:
        name = "free_junction"
        def step(self, frame: FramePacket, now: float) -> TaskUpdate

返回约定：
  * 没到岔路 / 两条分支都没有车（不是本模块的场景）→ `TaskUpdate(TaskStatus.NOT_TRIGGERED)`
  * 正在选路 → `TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(...))`
  * 选完并通过 → `TaskUpdate(TaskStatus.COMPLETED)`
  * 判据不成立（两条都有车）或超时丢线 → `TaskUpdate(TaskStatus.FAILED)`
  一旦返回 RUNNING，就必须持续返回 RUNNING，直到 COMPLETED 或 FAILED。

场地假设（全部是可改参数，逻辑里没有写死数字）
----------------------------------------------
============  ==========================================================
假设编号       内容
============  ==========================================================
A1            岔路 = 蓝色巡线带"一分为二"：下面一条主干、上面两条分支向上张开。
              HSV 区间沿用 `config.VisionConfig` 的已验证默认值。
A2            分叉行的判据：同一行上"最左段"和"最右段"的**中心距**够远
              （`min_branch_separation_px`），且**空隙 ≥ 带宽的 40%**
              （`min_gap_over_tape`）。用最左+最右而不是"恰好两段"，是因为实车画面里
              一条带子常被噪点切成三四段（6 号在他们的实车报告里踩过同一个坑）。
A3            真岔路的两条分支是**越往上越张开**的：分叉行上方那条带子的间距
              要比分叉行本身明显更大（`min_divergence_ratio`）。噪点把一条带子切碎
              不会满足这一条，普通弯道也不会。
A4            岔路连续出现 `confirm_frames` 帧、同一个拥堵读数连续出现
              `blockage_confirm_frames` 帧，才考虑触发；只闪一帧不算。
A5            **"拥堵"= 那条分支的走廊里停着一辆同型小车**（官方定义）。判据看
              **“比周围地面暗/亮一大块”**（大尺度局部对比度，v3 起），**不看颜色** ——
              理由和 4 号 `obstacle.py` 一样：颜色判据在现场没有分离度；
              而"任何够暗的像素都算障碍"会把影子和车自己的阴影全算进去。
              形状判据全部相对**检测区域**尺寸：外接框宽 ≥ `min_vehicle_width_ratio`、
              高 ≥ `min_vehicle_height_ratio`、面积 ≥ `min_vehicle_area_ratio`、
              框内有东西的像素 ≥ `min_vehicle_density`，再要求框的下沿不低于
              `vehicle_min_bottom_ratio`（不然"远处一整面墙"也会算进来）。
              检测区域是整帧高度的一条带（`blockage_top_ratio` ~
              `blockage_bottom_ratio`），**不是**岔路 ROI：真车实测那辆车在
              y≈36~159，而 ROI 只有 y 0.54~0.96，"只按 ROI 找"会看不见真堵的那条。
A6            两条分支**只有一条**有车 → 走另一条；**两条都有车** → 判据不成立 →
              停车报失败（或走 `fallback_branch`）；**两条都没有车** → 这不是
              "无拥堵岔路"场景 → **不接管**（`require_blockage_to_trigger`），
              把控制权留给巡线 / 6 号，避免无谓地在场地中间停车。
A7            选路是**闭环**有限动作：DECIDE（原地等判据）→ APPROACH（对准）→
              TURN（有限转向）→ EXIT（直行离开），每段都有独立超时，
              总时长不超过 `max_task_seconds`（远小于骨架 20 s 硬上限）。
A8            转向符号沿用项目约定：**yaw 正值右转、负值左转**。
A9            走完一个岔路后 `rearm_cooldown` 秒内、而且岔路要从画面里消失
              `rearm_clear_frames` 帧，才允许处理下一个岔路。
A10           **被迫结束（人工 SPACE / 视频中断 / 协调器超时）后也要封锁**：
              协调器会调 `reset()`，v2 的 `reset()` 会重新拉起封锁闸门，
              避免"被踢出来 4 帧后又立刻接管"（2026-09-15 实车报告里的 20 次接管）。
A11           接管期间连续 `lost_line_frames` 帧一条蓝线都没有 →
              立刻停车报失败（`abort_on_line_lost`），绝不盲开。
A12           所有阈值都是在合成画面上定的，**必须在正式场地重新标定**
              （见 `实车调试手册_free_junction.md`）。

单独自测::

    python scripts/check_module.py free_junction
    python -m unittest tests.test_free_junction -v
"""

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

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

    IDLE = "idle"              # 没看到岔路 / 封锁中 / 两条分支都没车
    DECIDE = "decide"          # 岔路 + 有车已确认，原地稳定判据
    APPROACH = "approach"      # 对准选中分支
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
        """转向符号：项目约定 yaw 正值右转、负值左转。"""
        return 1.0 if self is Branch.RIGHT else -1.0

    @property
    def other(self) -> "Branch":
        """另一条分支（"这边堵就走那边"）。"""
        return Branch.LEFT if self is Branch.RIGHT else Branch.RIGHT


#: 拥堵判据的四种读数，用字符串便于写进 message 里给实车记录看。
BLOCKAGE_NONE = "none"        # 两条分支都没有车
BLOCKAGE_LEFT = "left"        # 左分支上有车
BLOCKAGE_RIGHT = "right"      # 右分支上有车
BLOCKAGE_BOTH = "both"        # 两条都有车


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

    # ---- 找车的检测区域（整帧比例）：**故意比岔路 ROI 高得多** ----
    # 2026-09-16 真车实测：停着的那辆车在画面左上方（y≈36~159），
    # 而岔路 ROI 只有 y 0.54~0.96，车整个在走廊外面 → "真堵的那条反而看不见"。
    # 上沿取 0.10 把远处那辆车包进来；下沿取 0.86 是为了排掉我们自己车头的
    # 橙色轮子/车体（它常驻在画面最下方正中）。
    blockage_top_ratio: float = 0.10
    blockage_bottom_ratio: float = 0.86

    # ---- 蓝线颜色管线（沿用已验证的蓝色胶带默认值）----
    hsv_lower: Tuple[int, int, int] = (95, 80, 60)
    hsv_upper: Tuple[int, int, int] = (135, 255, 255)
    open_kernel: int = 3
    # 故意比 line_detector 的默认值小：闭运算核太大（例如 17）会把岔路的
    # 两条分支重新粘成一条，那样就检测不出分叉了。
    close_kernel: int = 5
    min_blue_ratio: float = 0.004   # ROI 里蓝线占比低于它，当作"这一帧没有线"

    # ---- 岔路几何（A2 / A3 / A4）----
    # 整列扫描：分叉行在 ROI 里哪一行都可能（车离得远，分叉点就靠上）。
    # v1 只看 ROI 50% 处那 7 行，实测"岔路远一点就完全不触发"。
    min_branch_separation_px: int = 45   # 两段带子的中心距下限
    min_gap_over_tape: float = 0.40      # 空隙 ≥ 带宽的 40% 才算真分开
    merge_gap_px: int = 6                # 同一行里隔得比它近的蓝色像素算同一段
    min_run_px: int = 5                  # 短于它的碎块丢掉
    min_fork_rows: int = 3               # 最少要有几行满足分叉形态
    min_divergence_ratio: float = 0.25   # 最上面一行间距要比分叉行大这么多（A3）
    confirm_frames: int = 4              # 连续几帧看到岔路才算数
    blockage_confirm_frames: int = 2     # 同一个拥堵读数连续几帧才算数
    decide_confirm_frames: int = 1       # 接管后再稳定几帧就落子
    rearm_clear_frames: int = 8          # 岔路消失几帧后才允许再次触发

    # ---- 拥堵判据：分支走廊里有没有一辆车（A5）----
    decision_rule: str = "vehicle"     # "vehicle" | "fixed"
    fixed_branch: Optional[str] = None      # decision_rule="fixed" 时走哪边
    fallback_branch: Optional[str] = None   # 两条都有车时走哪边；None = 停车报失败
    require_blockage_to_trigger: bool = True  # 两条都没车就不接管（A6）

    # 旧的"走廊"参数（v2 用来把 ROI 下部切块）。v3 改成整帧高度的检测带之后不再使用，
    # 保留名字只为兼容，值不影响任何行为。
    corridor_height_ratio: float = 1.0
    corridor_bottom_margin: float = 0.05
    corridor_side_margin: float = 0.0

    # ---- 判据一（默认）：大尺度局部对比度 ----
    # 车 = "比周围地面暗一大块（或亮一大块）"；地砖的黑点、反光是**小尺度**纹理，
    # 会被大核平均掉。2026-09-16 真车实测（4 帧结果一致）：
    #   停着车那条 → 宽0.39 高0.36 面积0.14 下沿0.44（每帧都是同一个框）
    #   空着那条   → 宽0.15~0.28 高0.06~0.16 面积0.02~0.03 下沿0.14~0.21
    use_local_contrast: bool = True
    contrast_kernel_ratio: float = 0.15   # "周围地面"的大核 = 走廊高度 × 它
    contrast_threshold: int = 45          # |周围 − 自己| 超过它才算"一块东西"
    contrast_blur_sigma: float = 1.0      # 先抹掉 1 像素级的噪点
    contrast_close: int = 9               # 把同一块里的空洞补一补

    # ---- 判据二（默认关）：Canny 结构判据 ----
    # 4 号 obstacle.py 的路子。但现场是**花岗岩地砖**：细纹理把 Canny 连成"一整片地毯"
    # （实测结构像素占 67%~80%，一个框就把整条走廊圈住），真车反而被淹没 —— 默认关。
    use_structure: bool = False
    edge_low: int = 40                    # Canny 低阈值
    edge_high: int = 110                  # Canny 高阈值
    edge_dilate: int = 3                  # 边缘加粗，后面才连得成块
    structure_close: int = 15             # 把边缘连成整块的核大小
    #: 找车之前，把蓝带**连同边缘**一起抹平多宽（像素）。两个判据共用。
    #: HSV 掩码只盖得住胶带芯，边缘的抗锯齿混合像素在掩码外面；不抹掉它们，
    #: 判据就会把胶带自己的轮廓当成"一块东西"
    #: （2026-09-16 真车实测：干净的那条分支被报成"有车"，于是走了堵的那条）。
    tape_clear_px: int = 5

    # ---- 形状闸门（全部相对走廊尺寸），按 2026-09-16 真车实测重新标定 ----
    min_vehicle_width_ratio: float = 0.20   # 车 0.39 / 空 0.15~0.28
    min_vehicle_height_ratio: float = 0.25  # 车 0.36 / 空 0.06~0.16 ← 主要靠这道分开
    min_vehicle_area_ratio: float = 0.08    # 车 0.14 / 空 0.02~0.03
    min_vehicle_density: float = 0.05       # 框内"有东西"的像素占比（太稀的空框不算）
    max_vehicle_width_ratio: float = 0.95   # 上限只当兜底（车很近时会顶满走廊）
    max_vehicle_area_ratio: float = 0.90
    vehicle_min_bottom_ratio: float = 0.35  # 车 0.44 / 空 0.14~0.21 → 0.35 两边都分得开
    reject_skin_like: bool = True         # 皮肤色占比过半的候选丢掉（同 obstacle.py）
    skin_reject_ratio: float = 0.50
    use_color_ranges: bool = False        # 颜色判据默认关（A5 的理由）
    vehicle_hsv_ranges: Tuple[Tuple[Tuple[int, int, int], Tuple[int, int, int]], ...] = (
        ((5, 90, 80), (25, 255, 255)),
    )

    # ---- 动作（A7 / A8，全部远低于骨架限幅）----
    decide_timeout: float = 0.6       # 原地稳定判据的最长时间
    approach_forward: float = 0.10
    approach_yaw_gain: float = 45.0
    max_approach_yaw: float = 25.0
    approach_seconds_max: float = 1.5
    turn_forward: float = 0.06
    turn_yaw: float = 45.0
    turn_seconds: float = 1.2
    turn_timeout: float = 2.5
    #: 转向至少要转这么久，才允许"看到线回到中央就收工"。
    #: 真车实测：只按时间转固定角度会出现"原地多转几十度"；而线一旦回到车头
    #: 正前方，就说明已经拐进分支了，不必把剩下的角度转满。
    turn_min_seconds: float = 0.5
    #: 闭环转向的增益：yaw = 增益 × 胶带偏移（-1..1），再限幅到 `turn_yaw`。
    #: 看得到胶带就朝它转、越接近中央转得越慢；看不到才按选定方向满速转。
    turn_steer_gain: float = 90.0
    #: EXIT 阶段顺线修正的增益（比转向温柔）：交回巡线时车头尽量正对着线，
    #: 免得"拐是拐过去了，但斜着出线"导致底座拿不到有效线（2026-09-16 真车症状）。
    exit_steer_gain: float = 45.0
    #: "线回到画面中央"的容差（相对画面宽度）与需要的连续帧数。
    center_tolerance_ratio: float = 0.18
    center_confirm_frames: int = 2
    exit_forward: float = 0.15
    exit_seconds: float = 1.0
    exit_timeout: float = 2.0

    # ---- 自己先裁一遍限幅（骨架还会再裁一次）----
    max_forward: float = 0.30
    max_lateral: float = 0.25
    max_yaw: float = 90.0

    # ---- 安全与时效（A9 / A10 / A11）----
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
    divergence: float = 0.0
    confidence: float = 0.0
    box: Tuple[int, int, int, int] = (0, 0, 0, 0)
    #: ROI 里蓝线像素占比。即使判不出岔路也会填，用来判断"是不是整条线都不见了"。
    blue_ratio: float = 0.0
    #: 整幅图像宽度，用来把像素偏差换算成归一化误差（对准阶段要用）。
    frame_width: int = 0

    @classmethod
    def empty(cls) -> "ForkDetection":
        return cls(valid=False)


@dataclass(frozen=True)
class BlockageReading:
    """两条分支上"有没有车"的读数。`reading` 取 BLOCKAGE_* 四个值之一。"""

    reading: str
    left_evidence: float = 0.0     # 左侧候车框面积占走廊的比例（0 表示没找到）
    right_evidence: float = 0.0
    left_box: Optional[Tuple[int, int, int, int]] = None
    right_box: Optional[Tuple[int, int, int, int]] = None

    @property
    def blocked(self) -> bool:
        return self.reading != BLOCKAGE_NONE

    def describe(self) -> str:
        """一行短描述，直接塞进 message —— 实车运行记录里就能看见判据读数。"""
        left = "yes" if self.reading in (BLOCKAGE_LEFT, BLOCKAGE_BOTH) else "no"
        right = "yes" if self.reading in (BLOCKAGE_RIGHT, BLOCKAGE_BOTH) else "no"
        return "vehicle L=%s(%.2f) R=%s(%.2f)" % (
            left, self.left_evidence, right, self.right_evidence
        )


def _clamp(value: float, low: float, high: float) -> float:
    """把数裁进 [low, high]；nan / inf 一律变成 0。"""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not np.isfinite(number):
        return 0.0
    return max(low, min(high, number))


def _odd_kernel(size: int) -> np.ndarray:
    size = max(1, int(size))
    if size % 2 == 0:
        size += 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))


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


def _separated_runs(
    row_mask: np.ndarray, merge_gap: int, min_run: int,
    min_separation: float, min_gap_over_tape: float,
) -> Optional[Tuple[Tuple[int, int], Tuple[int, int], float]]:
    """这一行里"最左段 + 最右段"够不够开（A2）。

    返回 (最左段, 最右段, 中心距)，不够开就返回 None。
    用最左+最右而不是"恰好两段"：实车画面里一条带子常被噪点切成三四段。
    """
    runs = _row_runs(row_mask, merge_gap, min_run)
    if len(runs) < 2:
        return None
    first, last = runs[0], runs[-1]
    separation = ((last[0] + last[1]) - (first[0] + first[1])) / 2.0
    if separation < min_separation:
        return None
    tape = min(first[1] - first[0] + 1, last[1] - last[0] + 1)
    if (last[0] - first[1]) < min_gap_over_tape * tape:
        return None
    return first, last, separation


# ---------------------------------------------------------------------------
# 岔路检测器
# ---------------------------------------------------------------------------


def _blue_mask(roi: np.ndarray, settings: FreeJunctionConfig) -> np.ndarray:
    """一块图里的蓝色带子掩码（bool）。岔路检测和找车都用这一套 HSV。"""
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(settings.hsv_lower, dtype=np.uint8),
        np.array(settings.hsv_upper, dtype=np.uint8),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _odd_kernel(settings.open_kernel))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _odd_kernel(settings.close_kernel))
    return mask > 0


class FreeJunctionDetector:
    """只做一件事：在一张 BGR 图上找"一分为二"的蓝色带子（A1~A4）。

    不做拥堵判断（那是 `VehicleDetector` 的事），也不改任何状态。
    """

    def __init__(self, settings: Optional[FreeJunctionConfig] = None) -> None:
        self.settings = settings if settings is not None else FreeJunctionConfig()

    def blue_mask(self, roi: np.ndarray) -> np.ndarray:
        """ROI 里的蓝色带子掩码（bool）。"""
        return _blue_mask(roi, self.settings)

    def roi_rect(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """把比例 ROI 换算成像素框 (left, top, right, bottom)。"""
        settings = self.settings
        height, width = frame.shape[:2]
        left = int(width * settings.roi_left)
        right = int(width * settings.roi_right)
        top = int(height * settings.roi_top)
        bottom = int(height * settings.roi_bottom)
        if right - left < 16 or bottom - top < 16:
            return None
        return left, top, right, bottom

    def analyze(self, image: Optional[np.ndarray]):
        """一次算完：返回 (岔路检测, ROI 图, ROI 蓝线掩码, ROI 像素框)。

        任务层要"岔路 + 蓝线掩码"两样东西，这里一次算完，
        避免同一帧把 HSV + 形态学算两遍。
        """
        empty = (ForkDetection.empty(), None, None, None)
        if image is None:
            return empty
        frame = np.asarray(image)
        if frame.ndim != 3 or frame.shape[2] != 3:
            return empty
        rect = self.roi_rect(frame)
        if rect is None:
            return empty
        left, top, right, bottom = rect
        roi = frame[top:bottom, left:right]
        mask = self.blue_mask(roi)
        fork = self._locate(roi, mask, left, top, frame.shape[1])
        return fork, roi, mask, rect

    def detect(self, image: Optional[np.ndarray]) -> ForkDetection:
        """只要岔路检测结果（外面调试用）。"""
        fork, _roi, _mask, _rect = self.analyze(image)
        return fork

    def _locate(self, roi, mask, left, top, frame_width) -> ForkDetection:
        """在已经算好的 ROI 掩码里找分叉（整列扫描 + 张开度判据）。"""
        settings = self.settings
        roi_height, roi_width = mask.shape[:2]
        blue_ratio = float(np.count_nonzero(mask)) / float(max(1, mask.size))
        empty = ForkDetection(valid=False, blue_ratio=blue_ratio, frame_width=frame_width)

        # A2：整列扫描（v1 只看 50% 处那 7 行，远一点的岔路会整个漏掉）。
        rows: List[Tuple[int, Tuple[int, int], Tuple[int, int], float]] = []
        for row in range(roi_height):
            found = _separated_runs(
                mask[row], settings.merge_gap_px, settings.min_run_px,
                settings.min_branch_separation_px, settings.min_gap_over_tape,
            )
            if found is not None:
                first, last, separation = found
                rows.append((row, first, last, separation))

        if len(rows) < max(1, int(settings.min_fork_rows)):
            return empty

        # 分叉行 = 最靠下（y 最大）那一行；最上面一行用来验"越往上越张开"（A3）。
        split_row, first, last, separation = rows[-1]
        top_row, _top_first, _top_last, top_separation = rows[0]
        divergence = 0.0
        if separation > 0.0:
            divergence = top_separation / separation - 1.0
        if top_row >= split_row or divergence < settings.min_divergence_ratio:
            return empty

        left_x = int(round((first[0] + first[1]) / 2.0)) + left
        right_x = int(round((last[0] + last[1]) / 2.0)) + left
        split_x = int(round((left_x + right_x) / 2.0))
        confidence = min(
            1.0, separation / max(1.0, float(settings.min_branch_separation_px) * 2.0)
        )
        return ForkDetection(
            valid=True,
            split_row=split_row + top,
            split_x=split_x,
            left_x=left_x,
            right_x=right_x,
            gap_px=int(max(0, last[0] - first[1])),
            separation_px=int(separation),
            divergence=round(float(divergence), 3),
            confidence=confidence,
            box=(max(0, left_x - 20), top, min(frame_width, right_x + 20), top + roi_height),
            blue_ratio=blue_ratio,
            frame_width=frame_width,
        )


# ---------------------------------------------------------------------------
# 拥堵判据：分支走廊里有没有一辆同型小车（A5）
# ---------------------------------------------------------------------------


class VehicleDetector:
    """在一个"分支走廊"里找一辆车：**结构判据，不看颜色**。

    思路与 4 号 `obstacle.py` 相同（他们用 426 张现场画面标定过同一辆车）：

    * 硬边缘（Canny）→ 加粗 → 闭运算连成块：车立在地面上，轮廓边缘很"硬"，
      而影子、反光是渐变的，连不成块；
    * 判据量的是**外接框**，不是填充面积：车常被画面下沿切掉一块，轮廓是开口的；
    * 先把蓝线像素抹成周围地面的灰度再找边缘，否则横穿车身的蓝线会把轮廓切成两半；
    * 皮肤色占比过半的候选直接丢掉（现场最容易误检的是人的手）。

    刻意**不**用"任何够暗的像素都算障碍"这种颜色判据：`obstacle.py` 实测那会把影子
    和车自己的阴影全部算成障碍（v1 的 `floor_min_v` 就是这个毛病）。
    """

    def __init__(self, settings: Optional[FreeJunctionConfig] = None) -> None:
        self.settings = settings if settings is not None else FreeJunctionConfig()

    def _erase_tape(self, gray: np.ndarray, line: Optional[np.ndarray]) -> np.ndarray:
        """把蓝带**连同边缘**抹成周围地面的灰度。

        HSV 掩码只盖得住胶带芯，边缘的抗锯齿混合像素在掩码外面；不抹掉它们，
        判据就会把胶带自己的轮廓当成"一块东西"（2026-09-16 真车实测踩过这个坑：
        干净的那条分支被报成"有车"，于是走了堵的那条）。
        """
        if line is None or not bool(np.any(line)):
            return gray
        tape = np.zeros(gray.shape, dtype=np.uint8)
        tape[line] = 255
        grow = int(self.settings.tape_clear_px)
        if grow >= 3:
            tape = cv2.dilate(tape, _odd_kernel(grow))
        tape = tape > 0
        floor = gray[~tape]
        if not floor.size:
            return gray
        out = gray.copy()
        out[tape] = int(np.median(floor))
        return out

    def contrast_mask(self, region: np.ndarray, line: Optional[np.ndarray]) -> np.ndarray:
        """大尺度局部对比度：车 = "比周围地面暗一大块（或亮一大块）"。

        * 大核均值模糊得到"周围地面"的亮度水平，再和原图求 |差|；
        * 车是一两百像素的大块，差值很大；地砖的黑点、反光是 1~3 像素的小尺度纹理，
          被大核平均进地面里，差值很小 → 自然被抹掉；
        * 取**绝对值** → 深色车、浅色车都能抓（这才是"不看颜色"的正确做法）。
        """
        settings = self.settings
        gray = self._erase_tape(cv2.cvtColor(region, cv2.COLOR_BGR2GRAY), line)
        sigma = float(settings.contrast_blur_sigma)
        if sigma > 0.0:
            gray = cv2.GaussianBlur(gray, (0, 0), sigmaX=sigma)
        height = int(gray.shape[0])
        kernel = int(max(3, round(_clamp(settings.contrast_kernel_ratio, 0.05, 0.5) * height)))
        if kernel % 2 == 0:
            kernel += 1
        background = cv2.blur(gray, (kernel, kernel))
        mask = cv2.absdiff(background, gray) > int(settings.contrast_threshold)
        mask = mask.astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _odd_kernel(3))
        if settings.contrast_close >= 3:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, _odd_kernel(settings.contrast_close)
            )
        return mask

    def structure_mask(self, region: np.ndarray, line: Optional[np.ndarray]) -> np.ndarray:
        settings = self.settings
        gray = self._erase_tape(cv2.cvtColor(region, cv2.COLOR_BGR2GRAY), line)
        edges = cv2.Canny(gray, int(settings.edge_low), int(settings.edge_high))
        if settings.edge_dilate >= 3:
            edges = cv2.dilate(edges, _odd_kernel(settings.edge_dilate))
        if settings.structure_close >= 3:
            edges = cv2.morphologyEx(
                edges, cv2.MORPH_CLOSE, _odd_kernel(settings.structure_close)
            )
        return edges

    def color_mask(self, region: np.ndarray) -> np.ndarray:
        """颜色判据（默认关闭，见 A5）。"""
        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        mask = np.zeros(region.shape[:2], np.uint8)
        for lower, upper in self.settings.vehicle_hsv_ranges:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(
                    hsv,
                    np.array(lower, dtype=np.uint8),
                    np.array(upper, dtype=np.uint8),
                ),
            )
        return mask

    def build_mask(self, region: np.ndarray) -> np.ndarray:
        """把一块区域一次算成"可能是车"的掩码（两个判据按开关组合）。

        左右两条走廊共用同一块区域，所以**整块只算一次**再按分界线切开 ——
        真车日志里曾经因为每侧各算一遍，step() 花了 31ms（预算 20ms）。
        """
        settings = self.settings
        if region is None or region.size == 0:
            return np.zeros((0, 0), np.uint8)
        height, width = region.shape[:2]
        mask = np.zeros((height, width), np.uint8)
        if height < 8 or width < 8:
            return mask
        line = _blue_mask(region, settings)
        # 判据一（默认）：大尺度局部对比度。判据二（默认关）：Canny 结构。
        if settings.use_local_contrast:
            mask = cv2.bitwise_or(mask, self.contrast_mask(region, line))
        if settings.use_structure:
            mask = cv2.bitwise_or(mask, self.structure_mask(region, line))
        if settings.use_color_ranges:
            mask = cv2.bitwise_or(mask, self.color_mask(region))
        return mask

    def pick(
        self, mask: np.ndarray, region: np.ndarray
    ) -> Tuple[bool, float, Optional[Tuple[int, int, int, int]]]:
        """在掩码里挑一个"像车"的外接框（形状闸门 + 肤色剔除）。"""
        settings = self.settings
        if mask is None or mask.size == 0 or region is None:
            return False, 0.0, None
        height, width = mask.shape[:2]
        if height < 8 or width < 8 or not bool(np.any(mask)):
            return False, 0.0, None

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        region_area = float(max(1, height * width))
        skin = self._skin_mask(region) if settings.reject_skin_like else None
        best: Optional[Tuple[float, Tuple[int, int, int, int]]] = None
        for contour in contours:
            box = cv2.boundingRect(contour)
            score = self._score_box(mask, skin, box, height, width, region_area)
            if score is None:
                continue
            if best is None or score > best[0]:
                best = (score, box)

        if best is None:
            return False, 0.0, None
        score, box = best
        x, y, box_width, box_height = box
        return True, round(float(score), 3), (
            int(x), int(y), int(x + box_width), int(y + box_height)
        )

    def detect(
        self, region: Optional[np.ndarray], line: Optional[np.ndarray] = None
    ) -> Tuple[bool, float, Optional[Tuple[int, int, int, int]]]:
        """返回 (这块区域里有没有车, 证据强度 0~1, 外接框或 None)。

        `line` 参数只为兼容旧调用保留；现在蓝带掩码在 `build_mask` 里自己算。
        """
        if region is None or region.size == 0:
            return False, 0.0, None
        return self.pick(self.build_mask(region), region)

    def _skin_mask(self, region: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        lower = np.array((0, 25, 60), dtype=np.uint8)
        upper = np.array((25, 190, 255), dtype=np.uint8)
        return cv2.inRange(hsv, lower, upper)

    def _score_box(
        self, mask: np.ndarray, skin: Optional[np.ndarray],
        box: Tuple[int, int, int, int], height: int, width: int, region_area: float,
    ) -> Optional[float]:
        """形状判据（全部相对走廊尺寸）：过了返回分数，没过返回 None。"""
        settings = self.settings
        x, y, box_width, box_height = box
        if box_width <= 0 or box_height <= 0:
            return None
        width_ratio = box_width / float(width)
        height_ratio = box_height / float(height)
        area_ratio = (box_width * box_height) / region_area
        if width_ratio < settings.min_vehicle_width_ratio:
            return None
        if height_ratio < settings.min_vehicle_height_ratio:
            return None
        if area_ratio < settings.min_vehicle_area_ratio:
            return None
        if area_ratio > settings.max_vehicle_area_ratio:
            return None
        if width_ratio > settings.max_vehicle_width_ratio:
            return None
        # "不远处"：框的下沿要在走廊偏下的位置，排除远处的背景墙 / 看台（A5）。
        bottom_ratio = (y + box_height) / float(height)
        if bottom_ratio < settings.vehicle_min_bottom_ratio:
            return None
        patch = mask[y:y + box_height, x:x + box_width]
        if patch.size == 0:
            return None
        density = float(np.count_nonzero(patch)) / float(patch.size)
        if density < settings.min_vehicle_density:
            return None
        if skin is not None:
            skin_patch = skin[y:y + box_height, x:x + box_width]
            if skin_patch.size:
                skin_ratio = float(np.count_nonzero(skin_patch)) / float(skin_patch.size)
                if skin_ratio > settings.skin_reject_ratio:
                    return None
        return area_ratio


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
        vehicle_detector: Optional[VehicleDetector] = None,
    ) -> None:
        self.settings = settings if settings is not None else FreeJunctionConfig()
        self.detector = (
            detector if detector is not None else FreeJunctionDetector(self.settings)
        )
        self.vehicle = (
            vehicle_detector if vehicle_detector is not None
            else VehicleDetector(self.settings)
        )
        self.state = JunctionState.IDLE
        self.last_detection: Optional[ForkDetection] = None
        self.last_blockage = BlockageReading(BLOCKAGE_NONE)
        self.last_visual = VisualDetection.no_result(KIND)
        self.chosen_branch: Optional[Branch] = None
        self.last_reason = "idle"
        self.last_message = "idle"
        self.last_outcome: Optional[TaskStatus] = None
        #: 被迫结束（协调器调 reset()）的次数。实车记录里用它看"是不是在被反复打断"。
        self.interruptions = 0

        self._confirm_count = 0
        self._blockage_key = BLOCKAGE_NONE
        self._blockage_count = 0
        self._decide_count = 0
        self._clear_count = 0
        self._lost_count = 0
        self._center_count = 0
        self._last_blue_ratio = 0.0
        self._last_line_mask = None
        self._state_since: Optional[float] = None
        self._run_started_at: Optional[float] = None
        self._rearm_ready_at: Optional[float] = None
        self._last_now: Optional[float] = None

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
        """回到初始状态。

        **被迫结束也会走这里**（协调器 `_reset_task`：人工 SPACE / 视频中断 /
        协调器超时）。v1 在这里把封锁闸门一起清掉，于是"被踢出来 4 帧后又立刻接管"
        （2026-09-15 实车报告：同一次运行 20 次接管、0 次收尾）。v2 的处理：

        * 状态清干净、运动请求归零；
        * **重新拉起封锁闸门**（A10）：要等 `rearm_cooldown` 秒，
          而且要等岔路从画面里消失 `rearm_clear_frames` 帧，两个条件都满足才允许再触发。
        """
        self.interruptions += 1
        self.state = JunctionState.IDLE
        self.last_detection = None
        self.last_blockage = BlockageReading(BLOCKAGE_NONE)
        self.last_visual = VisualDetection.no_result(KIND)
        self.chosen_branch = None
        self.last_reason = "reset"
        self.last_message = "reset"
        self.last_outcome = None
        self._decide_count = 0
        self._center_count = 0
        self._last_line_mask = None
        self._state_since = None
        self._run_started_at = None
        self._arm_rearm()

    def detect(self, image: Optional[np.ndarray]) -> VisualDetection:
        """给外面看的检测结果：岔路位置 + 判出的可通行一侧（只读，不改状态）。

        不是岔路、或判据不成立时返回 ``VisualDetection.no_result(KIND)``，
        不用虚构坐标冒充结果。
        """
        fork, roi, line, rect = self.detector.analyze(image)
        reading = self._read_blockage(fork, image, rect)
        self.last_blockage = reading
        branch: Optional[Branch] = None
        reason = "no junction"
        if fork.valid:
            branch, reason = self._choose_branch(reading)
        self.last_reason = reason
        return self._visual(fork, branch)

    # -- 主入口 -----------------------------------------------------------

    def step(self, frame: Optional[FramePacket], now: float) -> TaskUpdate:
        """每来一张新帧调用一次，立即返回。"""
        moment = float(now)
        self._last_now = moment
        if frame is None or getattr(frame, "image", None) is None:
            return self._handle_missing_image(moment)

        fork, roi, line, rect = self.detector.analyze(frame.image)
        self.last_detection = fork if fork.valid else None
        self._last_blue_ratio = fork.blue_ratio
        self._last_line_mask = line
        reading = self._read_blockage(fork, frame.image, rect)
        self.last_blockage = reading
        self.last_visual = self._visual(fork, self.chosen_branch)

        if self.state is JunctionState.IDLE:
            return self._step_idle(fork, reading, moment)
        return self._step_active(fork, reading, moment)

    # -- 拥堵读数 ---------------------------------------------------------

    def _read_blockage(self, fork: ForkDetection, frame, rect) -> BlockageReading:
        """读两条分支走廊：哪边停着车（A5）。

        区域 = **整帧高度上的一条带**（`blockage_top_ratio` ~ `blockage_bottom_ratio`），
        横向沿用岔路 ROI 的左右边界，再以分叉点 `split_x` 分成左右两块。

        为什么不像 v2 那样直接用岔路 ROI：2026-09-16 真车实测，停着的那辆车在
        **画面上方**（y≈36~159），而岔路 ROI 只有 y 0.54~0.96 —— 车整个在区域外面，
        于是"真堵的那条反而报没有车"。上沿抬到 0.10 把远处的车包进来，
        下沿压到 0.86 是为了排掉我们自己车头常驻的橙色轮子/车体。
        """
        settings = self.settings
        if not fork.valid or frame is None or rect is None:
            return BlockageReading(BLOCKAGE_NONE)
        image = np.asarray(frame)
        if image.ndim != 3 or image.shape[2] != 3:
            return BlockageReading(BLOCKAGE_NONE)
        height = int(image.shape[0])

        left_edge, right_edge = int(rect[0]), int(rect[2])
        top = int(_clamp(settings.blockage_top_ratio, 0.0, 0.9) * height)
        bottom = int(_clamp(settings.blockage_bottom_ratio, 0.1, 1.0) * height)
        if bottom - top < 16 or right_edge - left_edge < 16:
            return BlockageReading(BLOCKAGE_NONE)
        region = image[top:bottom, left_edge:right_edge]
        region_width = int(region.shape[1])

        divider = int(max(6, min(region_width - 6, int(fork.split_x) - left_edge)))
        left_region = region[:, :divider]
        right_region = region[:, divider:]

        # 整块区域**只算一次**掩码，再按分界线切开给左右两侧用（省一半时间）。
        mask = self.vehicle.build_mask(region)
        left_blocked, left_score, left_box = self.vehicle.pick(mask[:, :divider], left_region)
        right_blocked, right_score, right_box = self.vehicle.pick(mask[:, divider:], right_region)

        if left_blocked and right_blocked:
            reading = BLOCKAGE_BOTH
        elif left_blocked:
            reading = BLOCKAGE_LEFT
        elif right_blocked:
            reading = BLOCKAGE_RIGHT
        else:
            reading = BLOCKAGE_NONE

        return BlockageReading(
            reading=reading,
            left_evidence=left_score,
            right_evidence=right_score,
            left_box=self._to_frame(left_box, left_edge, top),
            right_box=self._to_frame(right_box, left_edge + divider, top),
        )

    @staticmethod
    def _to_frame(box, offset_x, offset_y):
        """区域内的外接框坐标 → 整幅图像坐标（给 evidence / 复盘用）。"""
        if box is None:
            return None
        x0, y0, x1, y1 = box
        return (x0 + offset_x, y0 + offset_y, x1 + offset_x, y1 + offset_y)

    def _choose_branch(self, reading: BlockageReading) -> Tuple[Optional[Branch], str]:
        """返回 (走哪条分支, 理由)。分支为 None 表示判据不成立 —— 那就别动。"""
        settings = self.settings

        if settings.decision_rule == "fixed":
            fixed = self._as_branch(settings.fixed_branch)
            if fixed is None:
                return None, "fixed rule without a side configured"
            return fixed, "fixed rule"

        if settings.decision_rule != "vehicle":
            return None, "unknown decision rule %r" % (settings.decision_rule,)

        if reading.reading == BLOCKAGE_LEFT:
            return Branch.RIGHT, "left branch blocked by a vehicle (%.2f)" % reading.left_evidence
        if reading.reading == BLOCKAGE_RIGHT:
            return Branch.LEFT, "right branch blocked by a vehicle (%.2f)" % reading.right_evidence
        if reading.reading == BLOCKAGE_BOTH:
            return self._fallback("both branches blocked by vehicles")
        return None, "no vehicle on either branch"

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

    # -- IDLE -------------------------------------------------------------

    def _step_idle(
        self, fork: ForkDetection, reading: BlockageReading, now: float
    ) -> TaskUpdate:
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

        # A6：两条分支都没有车 = 这不是"无拥堵岔路"场景 → 不接管，
        # 把控制权留给巡线 / 6 号，避免无谓地在场地中间停车。
        if not reading.blocked and settings.require_blockage_to_trigger:
            self._blockage_key = BLOCKAGE_NONE
            self._blockage_count = 0
            return self._not_triggered("junction with no vehicle on either branch")

        if reading.reading != self._blockage_key:
            self._blockage_key = reading.reading
            self._blockage_count = 1
        else:
            self._blockage_count += 1
        blocked_need = max(1, int(settings.blockage_confirm_frames))
        if self._blockage_count < blocked_need:
            return self._not_triggered(
                "confirming blockage %d/%d (%s)"
                % (self._blockage_count, blocked_need, reading.describe())
            )

        self.chosen_branch, reason = self._choose_branch(reading)
        self.last_reason = reason
        self._enter(JunctionState.DECIDE, now)
        self._run_started_at = now
        self._decide_count = 0
        return self._running(
            now, "junction confirmed, deciding (%s; %s)" % (reason, reading.describe())
        )

    # -- 接管中的状态机 ---------------------------------------------------

    def _step_active(
        self, fork: ForkDetection, reading: BlockageReading, now: float
    ) -> TaskUpdate:
        settings = self.settings

        if (self._run_started_at is not None
                and now - self._run_started_at > settings.max_task_seconds):
            return self._fail(now, "task time budget exceeded")

        # 只在"车头前面必须看得见线"的阶段才因为丢线认输：DECIDE / APPROACH。
        # TURN / EXIT 时车头一转，胶带本来就可能跑出 ROI —— 那不是"线没了"，
        # 再按丢线停车就会在岔路口中间白停一次（2026-09-16 真车：两次都是在
        # TURN/EXIT 里走了 0.8 s 就 FAILED）。这两段仍有各自的超时兜底。
        line_watch = self.state in (JunctionState.DECIDE, JunctionState.APPROACH)
        if settings.abort_on_line_lost and line_watch:
            if fork.valid or self._blue_present():
                self._lost_count = 0
            else:
                self._lost_count += 1
                if self._lost_count >= max(1, int(settings.lost_line_frames)):
                    return self._fail(now, "line lost while owning control")
        else:
            self._lost_count = 0

        if self.state is JunctionState.DECIDE:
            branch, reason = self._choose_branch(reading)
            self.last_reason = reason
            self._decide_count += 1
            if (branch is not None
                    and self._decide_count >= max(1, int(settings.decide_confirm_frames))):
                self.chosen_branch = branch
                self._enter(JunctionState.APPROACH, now)
                return self._running(
                    now, "taking the %s branch (%s; %s)"
                    % (branch.value, reason, reading.describe())
                )
            if self._elapsed(now) >= settings.decide_timeout:
                return self._fail(
                    now, "criterion unavailable: %s (%s)" % (reason, reading.describe())
                )
            return self._running(
                now, "waiting for a usable criterion (%s)" % reading.describe()
            )

        if self.state is JunctionState.APPROACH:
            gone = self._fork_gone(fork)
            if gone or self._elapsed(now) >= settings.approach_seconds_max * 0.75:
                self._enter(JunctionState.TURN, now)
            else:
                return self._running(now, "aligning with the %s branch" % self._branch_name())

        if self.state is JunctionState.TURN:
            if self._elapsed(now) >= settings.turn_timeout:
                return self._fail(now, "turn timed out")
            # **闭环转向**：盯住"车头正前方那条胶带"，朝它转；它回到中央就收工。
            # 不再"按秒表转固定角度"——真车实测那样会转过头：2026-09-16 17:26
            # 那次转了约 50°，而岔路实际角度小得多，出线之后底座接不回来。
            offset = self._turn_offset(fork)
            if offset is None:
                # 还看不到单条胶带（车还压在岔路口上）：按选定方向继续转，
                # 但**时间兜底照样要管**（否则会一路转到 turn_timeout 才失败）。
                self._center_count = 0
            elif abs(offset) <= float(settings.center_tolerance_ratio):
                self._center_count += 1
            else:
                self._center_count = 0
            centered = self._center_count >= max(1, int(settings.center_confirm_frames))
            if self._elapsed(now) >= settings.turn_seconds:
                self._enter(JunctionState.EXIT, now)
            elif centered and self._elapsed(now) >= settings.turn_min_seconds:
                self._enter(JunctionState.EXIT, now)
            else:
                return self._running(
                    now, "turning into the %s branch" % self._branch_name()
                )

        return self._finish_exit(now)

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
        self._center_count = 0

    # -- 运动请求 ---------------------------------------------------------

    def _motion(self, now: float) -> MotionCommand:
        settings = self.settings
        if self.state is JunctionState.DECIDE:
            # 等判据期间原地停着，绝不自己往前冲（A7）。
            return STOP
        if self.state is JunctionState.APPROACH:
            return MotionCommand(
                forward=_clamp(settings.approach_forward, 0.0, settings.max_forward),
                lateral=0.0,
                yaw=_clamp(
                    self._approach_yaw(), -settings.max_approach_yaw,
                    settings.max_approach_yaw,
                ),
            )
        if self.state is JunctionState.TURN:
            # 闭环：看得到岔路就朝"选中那条分支"转，岔路没了就朝车头前的胶带转；
            # 越接近正前方转得越慢，到中央就收工。
            offset = self._turn_offset(self.last_detection)
            if offset is None:
                yaw = settings.turn_yaw * self._branch_sign()
            else:
                yaw = _clamp(
                    settings.turn_steer_gain * offset,
                    -settings.turn_yaw,
                    settings.turn_yaw,
                )
            return MotionCommand(
                forward=_clamp(settings.turn_forward, 0.0, settings.max_forward),
                lateral=0.0,
                yaw=_clamp(yaw, -settings.max_yaw, settings.max_yaw),
            )
        # EXIT：直行离开岔路口；看得到胶带就**轻轻顺一下**，保证交回时车正对着线。
        offset = self._tape_offset()
        yaw = 0.0
        if offset is not None:
            yaw = _clamp(
                settings.exit_steer_gain * offset,
                -settings.max_approach_yaw,
                settings.max_approach_yaw,
            )
        return MotionCommand(
            forward=_clamp(settings.exit_forward, 0.0, settings.max_forward),
            lateral=0.0,
            yaw=_clamp(yaw, -settings.max_yaw, settings.max_yaw),
        )

    def _approach_yaw(self) -> float:
        """对准阶段：往选中分支偏一点点，每帧只修一点（A8）。

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

    def _tape_offset(self) -> Optional[float]:
        """车头正前方那条胶带偏了多少：-1 = 最左，0 = 正中，+1 = 最右。

        只算 ROI 底部那一小条，而且必须**只有一段**（出现两段说明车还压在岔路口上）。
        看不到、或不止一段 → 返回 None。
        """
        line = self._last_line_mask
        if line is None:
            return None
        height, width = line.shape[:2]
        if height <= 0 or width <= 0:
            return None
        band = line[int(height * 0.75):, :]
        if band.size == 0:
            return None
        runs = _row_runs(
            band[band.shape[0] // 2], self.settings.merge_gap_px, self.settings.min_run_px
        )
        if len(runs) != 1:
            return None
        center = (runs[0][0] + runs[0][1]) / 2.0
        half = max(1.0, width / 2.0)
        return (center - width / 2.0) / half

    def _turn_offset(self, fork: Optional[ForkDetection]) -> Optional[float]:
        """转弯阶段"还要往哪边转多少"（-1..1）。

        * 还看得见岔路 → 用**选中那条分支**的偏角（分支还没摆到正前方就继续转）；
        * 岔路已经从画面里消失 → 用**车头前方那条胶带**的偏移（这时它就是分支的胶带）。

        为什么不能只看胶带：刚进转弯时车头前面那条是**主干**，正的、就在中央，
        只按它判会得出"已经对齐"→ 根本不转（2026-09-16 本地测试踩出来过）。
        """
        if fork is not None and fork.valid and fork.frame_width > 0:
            target = fork.left_x if self.chosen_branch is Branch.LEFT else fork.right_x
            half = max(1.0, fork.frame_width / 2.0)
            return (float(target) - fork.frame_width / 2.0) / half
        return self._tape_offset()

    def _tape_centered(self) -> bool:
        """那条胶带是不是已经回到画面中央附近（用来判断"已经拐进分支"）。"""
        offset = self._tape_offset()
        if offset is None:
            return False
        return abs(offset) <= float(self.settings.center_tolerance_ratio)

    def _fork_gone(self, fork: ForkDetection) -> bool:
        """分叉是否已经消失（说明车已经开进分支里了）。"""
        if fork.valid:
            self._clear_count = 0
            return False
        self._clear_count += 1
        return self._clear_count >= 2

    def _arm_rearm(self) -> None:
        """拉起封锁闸门（A9 / A10）：冷却时间 + 等岔路从画面里消失。"""
        base = self._last_now if self._last_now is not None else 0.0
        self._rearm_ready_at = base + float(self.settings.rearm_cooldown)
        self._clear_count = 0
        self._confirm_count = 0
        self._blockage_count = 0
        self._blockage_key = BLOCKAGE_NONE

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
        """收尾：零速度、回到 IDLE，并拉起封锁闸门（A9）。"""
        self.state = JunctionState.IDLE
        self.last_outcome = status
        self.last_message = message
        # 故意不覆盖 last_reason：选边的理由要留着当证据（PR / 复盘要用）。
        self._state_since = None
        self._run_started_at = None
        self._arm_rearm()
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
