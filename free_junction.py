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
A5            **"拥堵"= 那条分支的走廊里停着一辆大疆 RoboMaster S1/EP 小车**
              （官方定义）。判据有两种来源，由 `blockage_source` 选：

              * `"vision"`（默认）：**按这辆车的样貌认** —— 深色车体
                （V<=`robot_dark_v_max`）+ 高饱和彩色装甲/灯（S>=`robot_accent_s_min`）。
                两个比例都**尺度无关**（真车实测：车 深色 0.61~0.70、彩色 0.25~0.34；
                空地/暗墙彩色 **0.000**），所以车远到 1~2 米也认得出。
                胶带连同边缘先抹掉，免得"胶带+影子"被凑成一辆车。
              * `"sdk"`：**只用官方 SDK 的识别结果**（机器人识别 / 视觉标签），
                由集成层订阅、`main.py` 每帧推给本模块（见 `update_robot_observations()`）。
                这条路上本模块**不看长宽高、不看颜色、不看形状**，只信官方读数。
              * `"sdk_or_vision"`：有新鲜官方读数就用它，没有就退回画面判据。

              检测区域是整帧高度的一条带（`blockage_top_ratio` ~
              `blockage_bottom_ratio`），**不是**岔路 ROI：真车实测那辆车在
              y≈36~159，而 ROI 只有 y 0.54~0.96，"只按 ROI 找"会看不见真堵的那条。
A6            两条分支**只有一条**有车 → 走另一条；**两条都有车** → 判据不成立 →
              停车报失败（或走 `fallback_branch`）；**两条都没有车** → 这不是
              "无拥堵岔路"场景 → **不接管**（`require_blockage_to_trigger`），
              把控制权留给巡线 / 6 号，避免无谓地在场地中间停车。
A7            选路是一段有限动作：DECIDE（原地等判据）→ APPROACH（对准）→
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

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Iterable, List, Optional, Tuple

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

#: 官方 SDK 报来的坐标是**归一化**还是**像素**，文档没写。归一化坐标不会超过它
#: —— 与集成层 `marker_source.py` 用的是同一条规则（NORMALIZED_MAX）。
NORMALIZED_MAX = 1.5


@dataclass(frozen=True)
class SdkSighting:
    """官方识别（机器人识别 / 视觉标签）报来的一条观测，坐标已换算成**整帧像素**。

    为什么要有这个类
    ----------------
    任务模块**不许碰 SDK**（红线 8；`tests/task_harness.py` 的
    `FORBIDDEN_PATTERNS` 里有 ``\\brobomaster\\b``，`scripts/check_module.py`
    会把它查出来）。所以官方读数由**集成层**订阅，再由 `main.py` 的
    `feed_robot_observations()` 每帧推给实现了 `update_robot_observations()` 的任务
    —— 和 5 号 `obstacle.py` 走的是同一条通路（`robot_source.py` 订阅 SDK 的
    **机器人识别**，`name="robot"`），**不需要改 `main.py` / `task_registry.py`**。
    本模块只消费这个纯数据。

    ⚠️ 这条通路**必须**用 `update_robot_observations` 这个名字：`main.py` 的
    `feed_marker_observations()` 推的是 SDK 的**视觉标签(marker)**，而"墙上的标签"
    和"岔路上停着一辆车"是两回事 —— 用 `update_candidates` 接会把标签当成车
    （2026-09-17 的接口适配就是为此改的名）。
    """

    center: Tuple[float, float]
    width: float
    height: float
    #: 回调接收时刻（单调钟，和协调器的 `now` 同一个钟）。None = 没有时间戳。
    observed_at: Optional[float] = None
    #: 视觉标签识别到的标签（机器人识别时为空）。
    label: str = ""

    def box(self) -> Tuple[int, int, int, int]:
        """换算成整帧像素的外接框（x0, y0, x1, y1），只用于日志和证据。"""
        half_w = max(0.0, float(self.width) / 2.0)
        half_h = max(0.0, float(self.height) / 2.0)
        return (
            int(round(self.center[0] - half_w)),
            int(round(self.center[1] - half_h)),
            int(round(self.center[0] + half_w)),
            int(round(self.center[1] + half_h)),
        )

    def width_ratio(self, frame_width: int) -> float:
        """它占画面宽多少（只用来写日志：判定**不看尺寸**）。"""
        if frame_width <= 0:
            return 0.0
        return float(self.width) / float(frame_width)


def _observation_parts(item):
    """外部推来的一条观测 → ``(x, y, w, h, observed_at, label)``；看不懂返回 None。

    认两种既有写法（都是为了对接不同层的约定，且绝不抛异常）：

    * 对象式（`number_marker.MarkerCandidate`）：``center`` + ``width`` +
      ``height``（+ ``observed_at`` / ``target_id``）；
    * SDK 原始行：``(x, y, w, h)``（机器人识别）或 ``(x, y, w, h, 标签)``（视觉标签）。

    注意 SDK 回调里的 x/y 是**中心点**（已对 `robomaster/vision.py` 核实）。
    """
    if item is None:
        return None
    center = getattr(item, "center", None)
    width = getattr(item, "width", None)
    height = getattr(item, "height", None)
    label = getattr(item, "target_id", "")
    observed_at = getattr(item, "observed_at", None)
    if center is None or width is None or height is None:
        try:
            row = tuple(item)
        except TypeError:
            return None
        if len(row) < 4:
            return None
        center = (row[0], row[1])
        width, height = row[2], row[3]
        label = row[4] if len(row) > 4 else ""
        observed_at = None
    try:
        x = float(center[0])
        y = float(center[1])
        w = float(width)
        h = float(height)
    except (TypeError, ValueError, IndexError):
        return None
    stamp: Optional[float] = None
    if observed_at is not None:
        try:
            stamp = float(observed_at)
        except (TypeError, ValueError):
            stamp = None
        if stamp is not None and not math.isfinite(stamp):
            stamp = None
    if not all(math.isfinite(value) for value in (x, y, w, h)):
        return None
    return x, y, w, h, stamp, ("" if label is None else str(label))


def observation_is_normalized(item) -> bool:
    """这条观测用的是归一化坐标吗（四个数都不超过 `NORMALIZED_MAX`）。"""
    parts = _observation_parts(item)
    if parts is None:
        return False
    return max(abs(value) for value in parts[:4]) <= NORMALIZED_MAX


def sighting_from_observation(
    item, frame_width: int, frame_height: int
) -> Optional[SdkSighting]:
    """外部观测 → `SdkSighting`（归一化坐标按整帧尺寸换算；看不懂返回 None）。"""
    parts = _observation_parts(item)
    if parts is None:
        return None
    x, y, w, h, stamp, label = parts
    if frame_width > 0 and frame_height > 0 and observation_is_normalized(item):
        x *= float(frame_width)
        y *= float(frame_height)
        w *= float(frame_width)
        h *= float(frame_height)
    return SdkSighting(
        center=(x, y), width=w, height=h, observed_at=stamp, label=label
    )


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
    #: 岔路检测 ROI 的上沿。0.54 -> 0.42 是**为了更早看到岔路**：
    #: 车离岔路还远时，分叉点在画面上方，压到 0.54 就看不见了 → 等到很近才触发。
    roi_top: float = 0.42
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
    min_fork_rows: int = 2               # 最少要有几行满足分叉形态（更早触发）
    min_divergence_ratio: float = 0.25   # 最上面一行间距要比分叉行大这么多（A3）
    confirm_frames: int = 2              # 连续几帧看到岔路才算数（求快：4 -> 2）
    blockage_confirm_frames: int = 1     # 同一个拥堵读数连续几帧才算数（求快：2 -> 1）
    decide_confirm_frames: int = 1       # 接管后再稳定几帧就落子
    rearm_clear_frames: int = 8          # 岔路消失几帧后才允许再次触发

    # ---- 拥堵判据：分支走廊里有没有一辆车（A5）----
    decision_rule: str = "vehicle"     # "vehicle" | "fixed"
    fixed_branch: Optional[str] = None      # decision_rule="fixed" 时走哪边
    fallback_branch: Optional[str] = None   # 两条都有车时走哪边；None = 停车报失败
    require_blockage_to_trigger: bool = True  # 两条都没车就不接管（A6）

    # ---- 拥堵判据的**来源**（A5）----
    #: **默认 `"sdk_or_vision"`：官方 SDK 的机器人识别说了算**（大疆 SDK 里
    #: `vision.sub_detect_info(name="robot")` 就是"识别同款 RoboMaster 小车"的接口）。
    #: 只要主循环把官方读数推给本模块（见 `update_robot_observations()`），判据就**只**看它：
    #: 不看颜色、不看长宽高、不看形状。
    #: 官方读数这一帧没到（例如集成层还没订阅 robot 识别）→ 退回画面判据兜底，
    #: 并在 message 里写明 `official robot detection unavailable`，一眼能看出用的是哪套。
    #: ``"sdk"`` = 只用官方读数（没订阅就永远不接管 —— 只用于确认订阅是否通了）；
    #: ``"vision"`` = 只用画面判据（旧行为）。
    blockage_source: str = "sdk_or_vision"
    #: 官方读数的保鲜窗口（秒）：回调比这还旧就当作"这一帧没看到"。
    #: 宁可退回"没有判据"（不接管），也不拿一条过期读数决定往哪边拐。
    sdk_observation_hold_seconds: float = 0.35

    # 旧的"走廊"参数（v2 用来把 ROI 下部切块）。v3 改成整帧高度的检测带之后不再使用，
    # 保留名字只为兼容，值不影响任何行为。
    corridor_height_ratio: float = 1.0
    corridor_bottom_margin: float = 0.05
    corridor_side_margin: float = 0.0

    # ---- 判据一（默认）：按 **RoboMaster S1 / EP 小车的特征**认车 ----
    # 真车画面实测（2026-09-16，两帧一致，车框 124x124）：
    #   停着的小车：深色像素(V<=90)占 0.61~0.70、高饱和彩色(S>=100)占 0.25~0.34
    #               （其中蓝 0.10、绿 0.07~0.14 —— 装甲灯/彩色贴纸）
    #   空地：      深色占 0.04~0.08、高饱和彩色占 **0.000**（花岗岩地砖没有鲜艳色）
    # 所以"**深色车体 + 高饱和彩色装甲/灯**"这两条合起来就是 S1/EP 的特征：
    #   * 地砖、影子、反光、白墙 → 没有鲜艳色 → 不会误判；
    #   * 两个比例都是**尺度无关**的 → 车离 1 米、2 米照样认得出，
    #     所以尺寸闸门可以放得很松（robot_min_*）。
    use_robot_signature: bool = True
    robot_dark_v_max: int = 90            # "车体深色"的亮度上限（空地中位数 137~144）
    robot_dark_min_ratio: float = 0.35    # 候选块里深色像素至少占这么多（车 0.61~0.70）
    robot_accent_s_min: int = 100         # "装甲/灯/贴纸"的饱和度下限
    robot_accent_min_ratio: float = 0.06  # 高饱和彩色至少占这么多（车 0.25~0.34，空地 0）
    robot_min_aspect: float = 0.5         # 外接框 宽/高 的允许范围（车约 0.75~1.2）
    robot_max_aspect: float = 2.5
    robot_min_width_ratio: float = 0.10   # 尺寸闸门放得很松：远一点的车也要能过
    robot_min_height_ratio: float = 0.10
    robot_min_area_ratio: float = 0.015
    #: 候选框面积占走廊的上限（只当兜底：真车候选实测 ≤ 0.66，含"车和墙连成一块"的
    #: 极端帧；换场地后那条墙裙误报是 0.84~0.85 —— 挡住它靠的是下面那条**组合判据**，
    #: 不是这个上限，否则会把"车+墙连成一块"的真检测一起挡掉，2026-09-17 试过）。
    robot_max_area_ratio: float = 0.90
    #: **"显眼候选"的彩色门槛**：满足下面任一条的候选，彩色占比必须 ≥ 这个值：
    #:   * 面积占比 ≥ `robot_large_area_ratio`（0.5）—— 墙/暗带糊成一大块；
    #:   * 长宽比 ≥ `robot_wide_aspect_ratio`（1.8）—— 门框、踢脚线那种宽扁条。
    #: 为什么必须"面积/形状 + 彩色"两条一起看（2026-09-17 换场地实测，337 张画面）：
    #:   * 墙 + 墙脚阴影带 + 木门：面积 0.84~0.85、彩色 **0.074** → 挡掉；
    #:   * 门框 + 暗墙边：长宽比 2.24、彩色 **0.062** → 挡掉；
    #:   * 真车（含"车和墙连成一块"、以及宽扁视角那一帧）：彩色 **0.22~0.27** → 照旧认出。
    #: 单看面积或单看长宽比都会误伤真车（实测：真车面积最大 0.80、长宽比也能到 2.24），
    #: 单看彩色也不行（真车最低 0.066，和误报的 0.062~0.074 重叠）。
    robot_large_area_ratio: float = 0.5
    robot_wide_aspect_ratio: float = 1.8
    robot_strong_accent_min_ratio: float = 0.15
    #: **"深色块附近有没有彩色"** 的邻域大小（像素）：装甲/灯就长在车身上，
    #: 所以只需要很小的邻域。现场实测：暗墙、门框、踢脚线也是"深色"，
    #: 光看深色会把背景一起圈进来；而彩色只有小车和胶带有（空地和墙是 0.000）。
    robot_accent_grow_px: int = 9
    robot_close_px: int = 5               # 深色块的闭运算（别太大，免得又粘背景）

    # ---- 兜底判据：**"这条支路上有个车大小的深色块挡道"**（完全不看颜色）----
    #: 2026-09-18 新场地实车：停着的那辆车**整体深灰、没有任何高饱和装甲/灯**
    #: （车框内 S>=100 只占 **0.004**，旧场地那辆是 0.162），于是上面"深色 + 彩色"
    #: 的 S1/EP 判据**整车都找不到**（掩码 0 像素）→ `reading=none` → 不接管
    #: → 巡线自己把车开进左边那条堵着的支路（两次实车都这样，接管记录里全是
    #: `LINE_FOLLOWING`、终端写着"不想 -> free_junction"）。
    #: 官方 SDK 那条路在这种场地上也帮不上：报告里 `robots_in_snapshot 0`、
    #: 292 次回调**全是空的**（EP 的机器人识别看不到它）。
    #: 所以再加一条**不看颜色**的判据，只问四件事，四条都成立才算"这条支路被堵"：
    #:   1) 支路走廊里有个**车大小**的深色块（面积/长宽比/深色占比过闸门）；
    #:   2) 它**立在地面上**：框正下方是地面（不是又一块深色），且底边不在判据带最下沿
    #:      （最下沿那是我们自己的车头/影子）；
    #:   3) **那条支路的胶带从下方通向它**（胶带像素够多）—— "车压在胶带上"的特征；
    #:   4) 顶到判据带上沿没关系（现场那辆车正好和上方暗背景连成一片），
    #:      所以这一条**不设**：靠 2)、3) 两条把墙裙/门框/远处暗带挡在外面。
    #: 全部语料回放（新场地 2 帧 + 旧场地 122 个有岔路的帧）：新场地两帧都判对
    #: （`left`），旧场地 **right/both 误报 0 次**（安全指标：空的那侧绝不能被误判）。
    occluder_enabled: bool = True
    #: 深色阈值怎么定：
    #:   * ``"fixed"``（**默认**）—— 用 `occluder_dark_v_max`（90）。
    #:     浅灰地面（考试场地）下车比地面暗得多，固定阈值反而最稳；
    #:   * ``"adaptive"`` —— 从这条带的亮度分布推（`p60 - margin`，夹到 floor~ceil）。
    #:     **实测是有害的**：124 帧逐帧 A/B 里它放出 3 帧"把空侧误判成 both"
    #:     （深色地面上阈值被抬高，地面自己也算成深色）；固定阈值那版是 0 帧。
    #:     所以只在明确换到**深色地面**且固定阈值失灵时才考虑打开，并且必须重新跑
    #:     `.local/probe_ab_integration_new.py` 验证。
    occluder_dark_mode: str = "fixed"
    occluder_dark_v_max: int = 90            # mode="fixed" 时的阈值
    occluder_adaptive_margin: int = 40       # 自适应：地面亮度 - 这个值
    occluder_adaptive_floor: int = 45        # 自适应阈值下限（别低到分不开车）
    #: 自适应阈值上限。**别调高**：调到 130 时旧语料立刻出现 3 帧把空侧误判
    #: （深色地面 + 放宽阈值 = 地面本身也被算成深色）。浅灰地面（考试场地）下
    #: "地面 200-40=160" 会被夹到 105，而深色车体（V 40~110）照样过。
    occluder_adaptive_ceil: int = 105
    occluder_dark_min_ratio: float = 0.45    # 框内（抠掉胶带后）深色像素占比（实测车 0.59~0.60）
    #: 面积门槛按"**车在岔路口外约 1 米**"标定（考试规范的距离）：
    #: 实测那一帧里车占判据带 **8.7%~11.9%**（练习场地车离得更近）；按几何推算
    #: 1 米外约 **4.2%**、1.5 米约 1.9% —— 所以 0.030 这条线覆盖到约 1.2~1.3 米。
    #: 曾经为了"1 米"把它降到 0.015，结果旧语料立刻出现 3 帧把空侧误判 → 回退。
    occluder_min_area_ratio: float = 0.030
    occluder_max_area_ratio: float = 0.55
    #: 最小**高度/宽度**占比：挡住又扁又碎的小块（面积和长宽比都过得去的那种）。
    #: 1 米外的 EP 车高约 72 像素 = 判据带的 26%，所以 0.12 既挡碎块又留足余量。
    occluder_min_height_ratio: float = 0.12
    occluder_min_width_ratio: float = 0.05
    occluder_min_aspect: float = 0.5         # 外接框 宽/高
    occluder_max_aspect: float = 3.0
    occluder_max_bottom_ratio: float = 0.80  # 底边低于判据带这个比例 = 太近，算我方车头/影子
    occluder_floor_band_px: int = 5          # 框正下方查多高（work 像素）
    occluder_floor_max_dark_ratio: float = 0.45   # 那一条里深色占比要低于这个（下面得是地面）
    occluder_tape_window_px: int = 45        # 框下方查胶带的高度（work 像素）
    occluder_tape_min_pixels: int = 150      # 胶带像素下限，按**整帧等效像素**算（真车 924，误报 32）
    occluder_tape_grow_px: int = 3           # 算深色块时把胶带撑开这么多（保住"车+穿过它的远处胶带"整块）
    occluder_min_pixels: int = 40            # 框内有效像素太少就不算（work 像素）
    #: **判据的运算尺度**：<1 = 先把检测带缩小再算（`1.0` = 不缩放）。
    #: 两个比例（深色占比、彩色占比）都是**尺度无关**的，离线实测把整幅画面缩到
    #: 0.45 倍仍然判对，所以缩放几乎不损失判据能力，却把这一步的耗时按面积降下来
    #: —— 实车日志里 `free_junction step was slow: 0.031s` 就是这一步造成的。
    vehicle_downscale: float = 0.5
    #: 检测带小于这么多像素就不缩放（小图/合成帧保持原样，行为可复现）。
    vehicle_downscale_min_pixels: int = 60000

    # ---- 岔路检测的运算尺度（2026-09-17 试过后**默认关掉**）----
    #: <1 = 先把岔路 ROI 缩小再算。**默认 1.0 = 不缩放**，原因见下。
    #:
    #: 试过 0.5，在 184 张实车画面上做等价性比对（`.local/probe_fork_scale.py`）：
    #:   * **1 帧直接漏掉岔路**（原尺寸认得出、半尺寸认不出）；
    #:   * 51 帧几何差 > 8 px，最大 **112 px**，而且偏差最大的正是"瞄准用的
    #:     `left_x` / `right_x`"（braches 张开最大那一行在缩小图上会落到别的行）。
    #: 岔路检测是**触发条件**，漏一帧就等于不接管、车直接开向障碍物 —— 这点提速
    #: 不值得冒这个险，所以整条链路保留但默认不启用。要试就自己把它调小，
    #: 并**先跑 `.local/probe_fork_scale.py`** 看那三项差异是否都可接受。
    fork_downscale: float = 1.0
    #: ROI 小于这么多像素就不缩放（小图/合成帧保持原样）。
    fork_downscale_min_pixels: int = 40000

    # ---- 判据二（默认关）：大尺度局部对比度（不看颜色，当兜底用）----
    use_local_contrast: bool = False
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
    # 要求 3：判完哪条堵之后**只做微调** —— 微微转、微微前进，一路看着胶带，
    # 直到岔路标志离开视野、且视野里重新有可用的蓝线，再交回巡线。
    # 所以下面所有速度/角速度都比骨架限幅小一个量级。
    decide_timeout: float = 0.25      # 原地稳定判据的最长时间（求快：0.6 -> 0.25）
    approach_forward: float = 0.08
    #: 对准阶段的增益与限幅。**2026-09-17 12:13~12:18 四次实车全部"转过头再拉回"**
    #: （对准 2 s 一直被 18 deg/s 打满 → 转过头 → 后面靠脚下那条线拉回来，
    #: 用掉 3.2~3.7 s）。所以增益 45→28、限幅 18→12：让它是"比例控制"、
    #: 快对准时自动慢下来，而不是一路满舵。
    #: **摄像头水平视场角（度）**：把"分支偏了多少像素"换算成**角度**用。
    #: 判据必须是角度，才不随场地/远近走样 —— 2026-09-17 19:48/19:49 两次实车：
    #: 那条岔路很"浅"，右支只比画面中心偏 51 px，旧的"像素/半宽"口径只有 0.16，
    #: 换算成 yaw 才 3.5 deg/s（再乘"岔路还远"的折扣只剩 ~1），车几乎直着开进
    #: 左边那条"有车"的分支；换成角度就是 **15°**，看得见的偏差就该转得动。
    camera_hfov_degrees: float = 120.0
    #: 瞄准增益：**每 1° 偏差给多少 deg/s**（1.0 = 偏多少度就转多少度/秒）。
    approach_yaw_gain: float = 1.0
    max_approach_yaw: float = 12.0
    approach_seconds_max: float = 2.0
    #: 转弯时的前进速度。**2026-09-17 17:24 / 17:26 / 17:27 三次实车的共同问题**：
    #: 摄像头装在车头前上方、看得比车头远 0.5~0.8 m，所以"看到岔路"时车头其实
    #: 还没到路口；原来的 0.04 太小 —— 车几乎是**原地**把方向转完，等车头到岔路口
    #: 时早就转过去了（17:24 那次交回巡线后直接 LINE_LOST）。
    #: 提到与 APPROACH 同速：边转边前进，转弯摊在更长的一段路上。
    turn_forward: float = 0.08
    turn_yaw: float = 14.0            # 微转（最大角速度；45 -> 20 -> 14，越改越稳）
    turn_seconds: float = 2.5         # 只是**上限**：正常会提前收工
    turn_timeout: float = 3.5
    #: 岔路还"远"时的转向打折比例与"算近"的门槛（见 `_turn_rate_scale`）。
    #: 17:24 那次的画面量出来：接管那一刻岔路的路口还在画面 y≈250/360（画面中央偏下），
    #: 车头离路口还有 0.5~0.8 m —— 一看到就满舵转，自然会拐早。
    #: 所以岔路还在 ROI 上部时按 `turn_far_yaw_scale` 打折，越靠近越给足：
    #: 折扣按"岔路在 ROI 里落到多低"线性插值（0.6 → 1.0），不是一刀切 ——
    #: 一刀切会把"浅岔路本来就不大的偏角"再削一半，车就几乎不转了
    #: （2026-09-17 19:48/19:49 两次实车：yaw 只有 ±1）。
    turn_far_yaw_scale: float = 0.6
    fork_close_ratio: float = 0.72
    #: 转向至少要转这么久，才允许"看到线回到中央就收工"。
    turn_min_seconds: float = 0.3
    #: 闭环转向的增益：yaw = 增益 × **偏角（度）**，再限幅到 `turn_yaw`。
    #: 看得到岔路就朝"选中那条分支"转；岔路没了就朝车头前那条胶带转。
    #: 越接近正前方转得越慢 —— 这样不会转过头（真车 2026-09-16 17:26 的教训）。
    turn_steer_gain: float = 1.0
    #: **"已经对准"的角度容差（度）**：选中分支在车头正前方这么多度以内算对准。
    #: 旧的写法是"相对画面宽度的 0.22"，换算过来约等于 **21°** —— 太松了：
    #: 实测那条浅岔路的右支偏 15°，却被判成"已经对准"，TURN 只跑了 0.3 s 就收工
    #: （2026-09-17 19:48 那次 1.4 s 就"完成"、车几乎没转）。
    center_tolerance_degrees: float = 8.0
    center_confirm_frames: int = 2
    #: 交回前的前进速度。0.10 → **0.14**（2026-09-17 20:55~21:03 八次实车实测）：
    #: 那八次全部成功，但有 **6 次是靠 `exit_seconds` 的 1 s 上限交回的**（不是
    #: "岔路离开视野 + 视野里有单根清晰的线"这个条件）—— 也就是"没确认就交回"。
    #: 岔路在 EXIT 开始时还在脚下（分叉行落在 ROI 最底部），再走几厘米才会离开视野；
    #: 同样 1 s 里从 10 cm 提到 14 cm，条件就更容易真正成立（交回更有把握），
    #: 而且总用时不变。仍然是慢速（骨架限幅 0.30）。
    exit_forward: float = 0.14
    exit_seconds: float = 1.0
    exit_timeout: float = 2.5
    #: EXIT 阶段顺线修正的增益（每 1° 偏差给多少 deg/s，比转向更温柔）。
    exit_steer_gain: float = 1.0
    #: 交回条件：岔路标志已经离开视野、视野里重新有**一条清晰的单根蓝线**，
    #: 连续这么多帧 → COMPLETED（"只有蓝线像素"不够，见 `_finish_exit`）。
    handback_confirm_frames: int = 3

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
    #: 分叉点所在的行（ROI 里最靠下的那行分叉）与它的横向中点，用来把走廊分左右。
    split_row: int = 0
    split_x: int = 0
    #: **两条分支各自的方向** —— 取张开最大那一行（最上面那行）的段中心，对准/转弯时瞄它。
    left_x: int = 0
    right_x: int = 0
    gap_px: int = 0
    separation_px: int = 0
    divergence: float = 0.0
    confidence: float = 0.0
    box: Tuple[int, int, int, int] = (0, 0, 0, 0)
    #: ROI 里蓝线像素占比。即使判不出岔路也会填，用来判断"是不是整条线都不见了"。
    blue_ratio: float = 0.0
    #: 整幅图像宽度/高度，用来把像素偏差换算成归一化误差、以及判断"岔路离得多近"。
    frame_width: int = 0
    frame_height: int = 0

    @classmethod
    def empty(cls) -> "ForkDetection":
        return cls(valid=False)


@dataclass(frozen=True)
class BlockageReading:
    """两条分支上"有没有车"的读数。`reading` 取 BLOCKAGE_* 四个值之一。"""

    reading: str
    left_evidence: float = 0.0     # 左侧证据：画面判据=面积占比；官方读数=宽占比
    right_evidence: float = 0.0
    left_box: Optional[Tuple[int, int, int, int]] = None
    right_box: Optional[Tuple[int, int, int, int]] = None
    #: 这条读数是谁给的：``"vision"``（本模块画面判据）或 ``"sdk"``（官方识别）。
    source: str = "vision"

    @property
    def blocked(self) -> bool:
        return self.reading != BLOCKAGE_NONE

    def describe(self) -> str:
        """一行短描述，直接塞进 message —— 实车运行记录里就能看见判据读数。"""
        left = "yes" if self.reading in (BLOCKAGE_LEFT, BLOCKAGE_BOTH) else "no"
        right = "yes" if self.reading in (BLOCKAGE_RIGHT, BLOCKAGE_BOTH) else "no"
        # 画面判据的措辞保持不变（历史运行记录/测试都按它比对）；
        # 官方读数单独标出来，复盘时一眼能看出这条判据是谁给的。
        label = "official sighting" if self.source == "sdk" else "vehicle"
        return "%s L=%s(%.2f) R=%s(%.2f)" % (
            label, left, self.left_evidence, right, self.right_evidence
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


def _scaled_px(value: float, scale: float) -> int:
    """把"以原图为单位的像素参数"换算到缩放后的图上（`:meth:`FreeJunctionConfig
    .vehicle_downscale` 缩小时，形态学核也要跟着缩，否则相对尺寸会翻倍）。"""
    try:
        factor = float(scale)
    except (TypeError, ValueError):
        factor = 1.0
    if not math.isfinite(factor) or factor <= 0.0:
        factor = 1.0
    return max(1, int(round(float(value) * factor)))


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


def _fork_rows(
    mask: np.ndarray, merge_gap: int, min_run: int,
    min_separation: float, min_gap_over_tape: float,
) -> List[Tuple[int, Tuple[int, int], Tuple[int, int], float]]:
    """整列扫描的**加速版**：先向量化筛掉无关行，再对候选行做精确判定。

    为什么要改（2026-09-16 19:58 实车运行的教训）
    --------------------------------------------
    原来每一行都调一次 `_separated_runs`（194 行 × 一次 numpy 调用），
    实车日志里 `free_junction step was slow: 0.031s (limit 0.020s)` 主要就是这段。
    现在先用**一趟整块运算**把不可能有分叉的行滤掉，只对少数候选行精算。

    两道预筛都**只是必要条件**，绝不会漏掉真分叉行：

    1. 这一行至少有 2 段（按 `merge_gap` 合并、按 `min_run` 过滤都只会让段数变少）；
    2. 这一行最左蓝像素到最右蓝像素的跨度 ≥ `min_separation`
       （两段中心之间的距离一定不超过这个跨度）。

    实车常见的"只有一条带子"帧在第 2 条就被全部滤掉，于是精确判定一次都不用做。
    """
    good = mask > 0
    if not bool(np.any(good)):
        return []
    width = int(good.shape[1])
    columns = np.arange(width, dtype=np.int32)
    # 每行最左 / 最右的蓝像素列号（空行给哨兵值，后面会被其它条件滤掉）。
    first_col = np.where(good, columns, width).min(axis=1)
    last_col = np.where(good, columns, -1).max(axis=1)
    # 每行有几段：**两边各补一个 False** 再数 False->True 的跳变，
    # 这样"贴着 ROI 左右边缘的那一段"也数得到（少补一边就会漏，真车踩过）。
    padded = np.zeros((good.shape[0], width + 2), dtype=np.int8)
    padded[:, 1:-1] = good
    runs_per_row = np.count_nonzero(np.diff(padded, axis=1) == 1, axis=1)
    candidate = np.logical_and(
        runs_per_row >= 2, (last_col - first_col) >= min_separation
    )

    rows: List[Tuple[int, Tuple[int, int], Tuple[int, int], float]] = []
    for row in np.flatnonzero(candidate):
        found = _separated_runs(
            mask[row], merge_gap, min_run, min_separation, min_gap_over_tape
        )
        if found is not None:
            first, last, separation = found
            rows.append((int(row), first, last, separation))
    return rows


# ---------------------------------------------------------------------------
# 岔路检测器
# ---------------------------------------------------------------------------


def _blue_mask(
    roi: np.ndarray, settings: FreeJunctionConfig, scale: float = 1.0
) -> np.ndarray:
    """一块图里的蓝色带子掩码（bool）。岔路检测和找车都用这一套 HSV。

    `scale` < 1 表示这张图是缩放过的：形态学核要**同比缩放**，核的相对大小才不变
    —— 否则在缩小图上用原尺寸的核，等于把闭运算核放大了一倍，会把岔路的两条分支
    重新粘成一条（config 里 `close_kernel` 的注释专门警告过这件事）。
    """
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(settings.hsv_lower, dtype=np.uint8),
        np.array(settings.hsv_upper, dtype=np.uint8),
    )
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, _odd_kernel(_scaled_px(settings.open_kernel, scale))
    )
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, _odd_kernel(_scaled_px(settings.close_kernel, scale))
    )
    return mask > 0


class FreeJunctionDetector:
    """只做一件事：在一张 BGR 图上找"一分为二"的蓝色带子（A1~A4）。

    不做拥堵判断（那是 `VehicleDetector` 的事），也不改任何状态。
    """

    def __init__(self, settings: Optional[FreeJunctionConfig] = None) -> None:
        self.settings = settings if settings is not None else FreeJunctionConfig()

    def blue_mask(self, roi: np.ndarray, scale: float = 1.0) -> np.ndarray:
        """ROI 里的蓝色带子掩码（bool）。"""
        return _blue_mask(roi, self.settings, scale)

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
        # **岔路检测也在缩小的 ROI 上算**（`fork_downscale`）：分叉几何全是相对量，
        # 缩放不改变判据，但把这一步的耗时按面积降下来。掩码最后放回 ROI 原尺寸，
        # 这样 `analyze()` 的返回值和以前一样（外面拿它算"脚下那条线偏多少"，与尺度无关）。
        factor = self._fork_scale(roi)
        work = roi
        if factor != 1.0:
            work = cv2.resize(roi, None, fx=factor, fy=factor, interpolation=cv2.INTER_AREA)
        mask = self.blue_mask(work, factor)
        fork = self._locate(
            work, mask, left, top, frame.shape[1], factor, frame.shape[0]
        )
        if factor != 1.0 and mask.size:
            mask = cv2.resize(
                mask.astype(np.uint8) * 255,
                (int(roi.shape[1]), int(roi.shape[0])),
                interpolation=cv2.INTER_NEAREST,
            ) > 0
        return fork, roi, mask, rect

    def _fork_scale(self, roi: np.ndarray) -> float:
        """岔路检测要在多小的 ROI 上算（1.0 = 原尺寸，见 `fork_downscale`）。"""
        settings = self.settings
        try:
            scale = float(getattr(settings, "fork_downscale", 1.0))
        except (TypeError, ValueError):
            return 1.0
        if not math.isfinite(scale) or not 0.0 < scale < 1.0:
            return 1.0
        if roi is None or getattr(roi, "size", 0) == 0:
            return 1.0
        minimum = int(getattr(settings, "fork_downscale_min_pixels", 0) or 0)
        if int(roi.shape[0]) * int(roi.shape[1]) < max(0, minimum):
            return 1.0
        return scale
        return fork, roi, mask, rect

    def detect(self, image: Optional[np.ndarray]) -> ForkDetection:
        """只要岔路检测结果（外面调试用）。"""
        fork, _roi, _mask, _rect = self.analyze(image)
        return fork

    def _locate(
        self, roi, mask, left, top, frame_width, factor: float = 1.0,
        frame_height: int = 0,
    ) -> ForkDetection:
        """在已经算好的 ROI 掩码里找分叉（整列扫描 + 张开度判据）。

        `factor` < 1 表示这张掩码是缩放过的：**像素阈值同比缩放**（判据全是相对量），
        结果坐标再换算回整帧像素，所以对外看到的 `ForkDetection` 与不缩放时同义。
        """
        settings = self.settings
        try:
            scale = float(factor)
        except (TypeError, ValueError):
            scale = 1.0
        if not math.isfinite(scale) or scale <= 0.0:
            scale = 1.0
        roi_height, roi_width = mask.shape[:2]
        blue_ratio = float(np.count_nonzero(mask)) / float(max(1, mask.size))
        empty = ForkDetection(
            valid=False, blue_ratio=blue_ratio, frame_width=frame_width,
            frame_height=int(frame_height),
        )

        # A2：整列扫描（v1 只看 50% 处那 7 行，远一点的岔路会整个漏掉）。
        # 具体实现在 `_fork_rows()`：先向量化筛掉无关行，只对候选行精算 —— 逐行
        # 调用 numpy 是实车 step() 超预算的主因（2026-09-16 19:58 那次运行）。
        min_separation = float(settings.min_branch_separation_px) * scale
        rows = _fork_rows(
            mask, _scaled_px(settings.merge_gap_px, scale),
            _scaled_px(settings.min_run_px, scale),
            min_separation, settings.min_gap_over_tape,
        )

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

        # **`left_x` / `right_x` 要的是"两条分支各自往哪边走"**，所以取**张开最大的
        # 那几行**（最上面那四分之一，至少 3 行）的段中心平均。
        #
        # * 不能取分叉行：那两行上分支刚分开、几乎重合，拿它的中心当瞄准点等于
        #   "几乎直行" —— 2026-09-17 实车就是这么翻的（选了右支、yaw 只有 +2.5，
        #   车最后开进左边那条有车的分支）；
        # * 也不能只取最上面一行：单行会被噪声带偏，2026-09-17 12:18 那次实车
        #   TURN 阶段的 yaw 出现过 20/12/20 来回跳。
        # `split_x`（分左右走廊用）仍然是分叉行的中点。
        head = rows[: max(1, min(len(rows), max(3, len(rows) // 4)))]

        def to_frame(value: float) -> int:
            return int(round(float(value) / scale))

        def centre(run) -> float:
            return (run[0] + run[1]) / 2.0

        split_x = to_frame((first[0] + first[1] + last[0] + last[1]) / 4.0) + left
        left_x = to_frame(sum(centre(row[1]) for row in head) / len(head)) + left
        right_x = to_frame(sum(centre(row[2]) for row in head) / len(head)) + left
        confidence = min(1.0, separation / max(1.0, min_separation * 2.0))
        return ForkDetection(
            valid=True,
            split_row=to_frame(split_row) + top,
            split_x=split_x,
            left_x=left_x,
            right_x=right_x,
            gap_px=int(max(0, last[0] - first[1]) / scale),
            separation_px=int(round(separation / scale)),
            divergence=round(float(divergence), 3),
            confidence=confidence,
            box=(
                max(0, left_x - 20),
                top,
                min(frame_width, right_x + 20),
                top + to_frame(roi_height),
            ),
            blue_ratio=blue_ratio,
            frame_width=frame_width,
            frame_height=int(frame_height),
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

    def mask_kind(self) -> str:
        """当前用哪套判据（决定闸门用哪一组）。"""
        return "robot" if self.settings.use_robot_signature else "contrast"

    def robot_mask(
        self, region: np.ndarray, line: Optional[np.ndarray], scale: float = 1.0
    ) -> np.ndarray:
        """按 **RoboMaster S1 / EP 小车**的特征做掩码：深色车体 ∪ 高饱和彩色装甲/灯。

        真车画面实测（2026-09-16，车框 124x124，两帧一致）：

        ==================  ==================  ==========
        特征                 停着的小车           空地
        ==================  ==================  ==========
        深色 V<=90 占比       0.61 ~ 0.70         0.04 ~ 0.08
        高饱和 S>=100 占比    0.25 ~ 0.34         **0.000**
        其中蓝 / 绿           0.10 / 0.07~0.14    0
        ==================  ==================  ==========

        所以"深色车体 + 鲜艳的装甲/灯"这两条一合，S1/EP 就出来了；而花岗岩地砖、
        影子、反光、白墙都**没有鲜艳色**，进不来。这两个比例还是**尺度无关**的，
        车离得远（1 米、2 米）一样成立 —— 这是"1 米间距也能认出来"的关键。
        """
        settings = self.settings
        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        value, sat = hsv[:, :, 2], hsv[:, :, 1]
        dark = value <= int(settings.robot_dark_v_max)
        accent = sat >= int(settings.robot_accent_s_min)
        if line is not None and bool(np.any(line)):
            # 胶带自己也是高饱和的（蓝色），而且它旁边常有一条暗影子：
            # 把胶带（含边缘）抠掉，免得"胶带+影子"被凑成一辆车。
            tape = np.zeros(dark.shape, dtype=np.uint8)
            tape[line] = 255
            grow = _scaled_px(settings.tape_clear_px, scale)
            if grow >= 3:
                tape = cv2.dilate(tape, _odd_kernel(grow))
            inside_tape = tape > 0
            accent = np.logical_and(accent, np.logical_not(inside_tape))
            dark = np.logical_and(dark, np.logical_not(inside_tape))
        # 快速出口：这一块里根本没有"胶带之外的鲜艳色" → 不可能是 S1/EP。
        # 现场实测空地和暗墙的高饱和像素是 **0.000**，所以常见帧在这里就结束了，
        # 后面的形态学 + 连通块一个都不用做。
        if not bool(np.any(accent)):
            return np.zeros(dark.shape, np.uint8)
        # **"深色车体" + "它身上/旁边有彩色装甲"** —— 注意是**连通块**层面的判断：
        # 暗墙、门框、踢脚线也是深色，光看深色会把背景一起圈进来；而彩色（装甲灯、
        # 彩色件）在场地上只有小车有（空地和墙的高饱和像素实测是 0.000）。
        # 所以：**被彩色碰到的那一整块深色**才算一辆车 —— 只保留彩色旁边几行像素
        # 是不行的（那会把车体切成两条细边，长宽比直接出界）。
        dark_u8 = (dark.astype(np.uint8)) * 255
        dark_u8 = cv2.morphologyEx(dark_u8, cv2.MORPH_OPEN, _odd_kernel(3))
        close = _scaled_px(settings.robot_close_px, scale)
        if close >= 3:
            dark_u8 = cv2.morphologyEx(dark_u8, cv2.MORPH_CLOSE, _odd_kernel(close))
        accent_u8 = (accent.astype(np.uint8)) * 255
        accent_core = cv2.morphologyEx(accent_u8, cv2.MORPH_OPEN, _odd_kernel(3))
        grow = _scaled_px(settings.robot_accent_grow_px, scale)
        accent_near = accent_core
        if grow >= 3:
            accent_near = cv2.dilate(accent_core, _odd_kernel(grow))
        if not bool(np.any(accent_core)) or not bool(np.any(dark_u8)):
            return np.zeros(dark.shape, np.uint8)
        count, labels, _stats, _centroids = cv2.connectedComponentsWithStats(dark_u8, 8)
        if count <= 1:
            return np.zeros(dark.shape, np.uint8)
        touched = np.unique(labels[accent_near > 0])
        hit = np.zeros(count, dtype=bool)
        hit[touched] = True
        hit[0] = False                     # 0 号是背景
        if not bool(np.any(hit)):
            return np.zeros(dark.shape, np.uint8)
        # 车体整块留下，再把**长在它身上的彩色装甲/灯**一起并进来
        # （装甲在图上盖住车体，所以它们是"深色连通块"之外的像素，不并进来
        #  外接框就会比车小一圈，彩色占比也就量不准了）。
        kept = (hit[labels].astype(np.uint8)) * 255
        grown = kept
        if grow >= 3:
            grown = cv2.dilate(kept, _odd_kernel(grow))
        return cv2.bitwise_or(kept, cv2.bitwise_and(accent_core, grown))

    def build_mask(
        self, region: np.ndarray, line: Optional[np.ndarray] = None, scale: float = 1.0
    ) -> np.ndarray:
        """把一块区域算成"可能是车"的掩码（判据按开关组合；整块只算一次）。

        `scale` 是这块图相对原图的缩放比（`:attr:`FreeJunctionConfig.vehicle_downscale``）：
        只有用到**像素单位**参数的判据才需要它（现在的 S1/EP 判据是）。
        """
        settings = self.settings
        if region is None or region.size == 0:
            return np.zeros((0, 0), np.uint8)
        height, width = region.shape[:2]
        mask = np.zeros((height, width), np.uint8)
        if height < 8 or width < 8:
            return mask
        if line is None:
            line = _blue_mask(region, settings)
        if settings.use_robot_signature:
            mask = cv2.bitwise_or(mask, self.robot_mask(region, line, scale))
        if settings.use_local_contrast:
            mask = cv2.bitwise_or(mask, self.contrast_mask(region, line))
        if settings.use_structure:
            mask = cv2.bitwise_or(mask, self.structure_mask(region, line))
        if settings.use_color_ranges:
            mask = cv2.bitwise_or(mask, self.color_mask(region))
        return mask

    def pick(
        self, mask: np.ndarray, region: np.ndarray, line: Optional[np.ndarray] = None
    ) -> Tuple[bool, float, Optional[Tuple[int, int, int, int]]]:
        """在掩码里挑一个"像车"的外接框（按当前判据过闸门）。"""
        settings = self.settings
        if mask is None or mask.size == 0 or region is None:
            return False, 0.0, None
        height, width = mask.shape[:2]
        if height < 8 or width < 8 or not bool(np.any(mask)):
            return False, 0.0, None
        if line is None:
            line = _blue_mask(region, settings)

        kind = self.mask_kind()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        region_area = float(max(1, height * width))
        skin = self._skin_mask(region) if settings.reject_skin_like else None
        best: Optional[Tuple[float, Tuple[int, int, int, int]]] = None
        for contour in contours:
            box = cv2.boundingRect(contour)
            if kind == "robot":
                score = self._robot_box_ok(
                    region, line, skin, mask, box, height, width, region_area
                )
            else:
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

    def dark_limit(self, value: np.ndarray) -> int:
        """这一帧该把多暗算"深色"（`occluder_dark_mode`）。

        `"adaptive"`：取这条带亮度的 **p60 当"地面参考"**，往下减
        `occluder_adaptive_margin`，再夹到 `[floor, ceil]`。
        这样"浅灰地面（考试场地）"和"深色水磨石（练习场地）"用同一套参数都成立：
        车永远比它脚下的地面暗一截，判据问的是"**比地面暗多少**"，不是绝对亮度。

        `"fixed"`：直接用 `occluder_dark_v_max`（旧行为）。
        """
        settings = self.settings
        mode = str(getattr(settings, "occluder_dark_mode", "fixed")).strip().lower()
        fixed = int(settings.occluder_dark_v_max)
        if mode != "adaptive" or value is None or getattr(value, "size", 0) == 0:
            return fixed
        try:
            reference = float(np.percentile(value, 60))
        except Exception:
            return fixed
        if not math.isfinite(reference):
            return fixed
        limit = reference - float(settings.occluder_adaptive_margin)
        limit = max(float(settings.occluder_adaptive_floor),
                    min(float(settings.occluder_adaptive_ceil), limit))
        return int(round(limit))

    def pick_occluder(
        self, region: np.ndarray, line: Optional[np.ndarray] = None, scale: float = 1.0
    ) -> Tuple[bool, float, Optional[Tuple[int, int, int, int]]]:
        """**不看颜色**的兜底判据：这条支路上有没有"车大小的深色块挡在胶带前面"。

        背景（为什么需要它）见 :attr:`FreeJunctionConfig.occluder_enabled`：
        2026-09-18 新场地那辆车**没有彩色装甲**，靠颜色认车的判据整车失效。

        这条判据只问四件事，四条都成立才算"被堵"：
          1. 深色块（V <= `occluder_dark_v_max`）的**连通块**有车那么大（面积/长宽比过闸门）；
          2. 框内（**抠掉胶带像素**后）深色占比够高（`occluder_dark_min_ratio`）；
          3. 它立在地面上：框**正下方**那一条不是深色，且底边不在判据带最下沿
             （最下沿是我们自己的车头/影子，实测就是这么误报的）；
          4. 框**正下方能看到本侧胶带**且像素够多 —— "车压在胶带上"的几何特征
             （实测：真车下方 924 帧像素，误报只有 32）。

        注意第 1 步的连通块用的是**未抠胶带**的深色掩码：现场那辆车常和"穿过去
        的远处胶带/暗背景"连成一片，先抠胶带会把车体切碎（2026-09-18 踩过）。
        """
        settings = self.settings
        if not bool(getattr(settings, "occluder_enabled", True)):
            return False, 0.0, None
        if region is None or region.size == 0:
            return False, 0.0, None
        height, width = region.shape[:2]
        if height < 8 or width < 8:
            return False, 0.0, None
        if line is None:
            line = _blue_mask(region, settings)
        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        limit = self.dark_limit(hsv[:, :, 2])
        raw = (hsv[:, :, 2] <= limit).astype(np.uint8)
        tape = np.zeros(raw.shape, np.uint8)
        if line is not None and bool(np.any(line)):
            tape[line] = 255
            grow = _scaled_px(settings.occluder_tape_grow_px, scale)
            if grow >= 3:
                tape = cv2.dilate(tape, _odd_kernel(grow))
        tape_here = tape > 0
        if not bool(np.any(raw)):
            return False, 0.0, None
        floor_band = max(1, _scaled_px(settings.occluder_floor_band_px, scale))
        tape_window = max(1, _scaled_px(settings.occluder_tape_window_px, scale))
        area = float(max(1, height * width))
        tape_floor = float(max(1, settings.occluder_tape_min_pixels)) * max(1e-6, scale * scale)
        best: Optional[Tuple[float, Tuple[int, int, int, int]]] = None
        contours, _ = cv2.findContours(raw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width <= 0 or box_height <= 0:
                continue
            area_ratio = (box_width * box_height) / area
            if not (settings.occluder_min_area_ratio <= area_ratio
                    <= settings.occluder_max_area_ratio):
                continue
            if box_width < settings.occluder_min_width_ratio * width:
                continue
            if box_height < settings.occluder_min_height_ratio * height:
                continue
            aspect = box_width / float(box_height)
            if not (settings.occluder_min_aspect <= aspect <= settings.occluder_max_aspect):
                continue
            if (y + box_height) / float(height) > settings.occluder_max_bottom_ratio:
                continue                                    # 太靠下 = 我方车头/影子
            patch = raw[y:y + box_height, x:x + box_width].astype(bool)
            patch_tape = tape_here[y:y + box_height, x:x + box_width]
            free = np.logical_not(patch_tape)
            total = int(np.count_nonzero(free))
            if total < int(settings.occluder_min_pixels):
                continue
            density = float(np.count_nonzero(np.logical_and(patch, free))) / float(total)
            if density < settings.occluder_dark_min_ratio:
                continue
            below = raw[y + box_height:y + box_height + floor_band, x:x + box_width]
            if below.size:
                below_dark = float(np.count_nonzero(below)) / float(below.size)
                if below_dark > settings.occluder_floor_max_dark_ratio:
                    continue                                # 下面还是深色 → 不是地面上的东西
            window = tape[y + box_height:min(height, y + box_height + tape_window),
                          max(0, x - 10):min(width, x + box_width + 10)]
            if window.size == 0:
                continue
            # 换算成"整帧等效像素"再比阈值，缩放才不会改变判据强弱。
            if float(np.count_nonzero(window)) < tape_floor:
                continue                                    # 胶带没通向它 → 不是堵在支路上的车
            if best is None or density > best[0]:
                best = (density, (x, y, x + box_width, y + box_height))
        if best is None:
            return False, 0.0, None
        return True, round(float(best[0]), 3), best[1]

    def detect(
        self, region: Optional[np.ndarray], line: Optional[np.ndarray] = None
    ) -> Tuple[bool, float, Optional[Tuple[int, int, int, int]]]:
        """返回 (这块区域里有没有车, 证据强度, 外接框或 None)。"""
        if region is None or region.size == 0:
            return False, 0.0, None
        if line is None:
            line = _blue_mask(region, self.settings)
        return self.pick(self.build_mask(region, line), region, line)

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

    def _robot_box_ok(
        self, region: np.ndarray, line: Optional[np.ndarray], skin: Optional[np.ndarray],
        mask: np.ndarray, box: Tuple[int, int, int, int], height: int, width: int,
        region_area: float,
    ) -> Optional[float]:
        """**S1/EP 专用闸门**：尺寸（松）+ 长宽比 + 不肤色 + 深色车体 + 高饱和彩色装甲。

        和"局部对比度"那套的区别：这里判的是**颜色构成**，不是"比周围暗一块"。
        因为深色占比和高饱和占比都是**尺度无关**的（而且只统计候选块自己），
        所以尺寸闸门可以放得很松（`robot_min_*` 只要 0.10/0.015），
        车远到 1~2 米也照样能过。
        """
        settings = self.settings
        x, y, box_width, box_height = box
        if box_width <= 0 or box_height <= 0:
            return None
        # 尺寸：只挡掉明显不是车的小碎块（远距离的车很小，所以门槛很低）。
        if box_width < settings.robot_min_width_ratio * width:
            return None
        if box_height < settings.robot_min_height_ratio * height:
            return None
        area_ratio = (box_width * box_height) / region_area
        if area_ratio < settings.robot_min_area_ratio:
            return None
        if area_ratio > settings.robot_max_area_ratio:
            return None
        # 长宽比：S1/EP 俯视/斜视大致 0.5~2.5，太细长的一定不是车。
        aspect = box_width / float(box_height)
        if aspect < settings.robot_min_aspect or aspect > settings.robot_max_aspect:
            return None
        # 手：肤色占比过半直接丢。
        if skin is not None:
            skin_patch = skin[y:y + box_height, x:x + box_width]
            if skin_patch.size:
                skin_ratio = float(np.count_nonzero(skin_patch)) / float(skin_patch.size)
                if skin_ratio > settings.skin_reject_ratio:
                    return None
        # 颜色构成 —— 这才是"是不是 S1/EP"的依据。
        # **只统计候选块自己**（mask 里的像素 = 深色车体 + 长在它身上的彩色装甲），
        # 不按整框算：现场里车常和暗墙/踢脚线连成一块（框能大到 271x273），
        # 按整框算比例会被背景稀释，车稍微暗一点就掉出闸门。
        patch_hsv = cv2.cvtColor(
            region[y:y + box_height, x:x + box_width], cv2.COLOR_BGR2HSV
        )
        value = patch_hsv[:, :, 2]
        sat = patch_hsv[:, :, 1]
        inside = mask[y:y + box_height, x:x + box_width] > 0
        if line is not None:
            tape_patch = line[y:y + box_height, x:x + box_width]
            valid = np.logical_and(np.logical_not(tape_patch), inside)
        else:
            valid = inside
        total = float(max(1, int(np.count_nonzero(valid))))
        dark_ratio = float(
            np.count_nonzero(np.logical_and(value <= settings.robot_dark_v_max, valid))
        ) / total
        accent_ratio = float(
            np.count_nonzero(np.logical_and(sat >= settings.robot_accent_s_min, valid))
        ) / total
        if dark_ratio < settings.robot_dark_min_ratio:
            return None
        if accent_ratio < settings.robot_accent_min_ratio:
            return None
        # **"显眼"候选要彩色更多**：大面积（墙/暗带糊成一块）或宽扁（门框、踢脚线）
        # 的候选，必须真的带够鲜艳色才算车。这一条同时挡住了 2026-09-17 换场地后
        # 出现的那两类误报（墙裙 0.074、门框 0.062），而真车（0.22~0.27）照旧过。
        prominent = (area_ratio >= settings.robot_large_area_ratio
                     or aspect >= settings.robot_wide_aspect_ratio)
        if prominent and accent_ratio < settings.robot_strong_accent_min_ratio:
            return None
        # 分数：越"又黑又有鲜艳装甲"越像（0~2）。
        return dark_ratio + accent_ratio


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
        #: 官方 SDK 观测的快照（由 `main.py` 每帧推送，见 `update_robot_observations()`）。
        #: 回调可能来自 SDK 自己的线程，所以读写都要过这把锁。
        self._sdk_lock = threading.Lock()
        self._sdk_rows: Tuple = ()
        self._sdk_pushed_at: Optional[float] = None
        #: 最近一次官方读数被判成哪种坐标（"normalized"/"pixels"/""）——实车排查用。
        self.last_sdk_mode = ""
        #: 从启动到现在，官方识别**一共报过几条**"看到一辆 RoboMaster 小车"。
        #: 一直是 0 就说明集成层还没订阅 robot 识别（那时用的是画面判据兜底）——
        #: 主循环的 message 会直接写出来，实车上一眼可见。
        self.official_sightings = 0
        #: 官方读数里"落在判据带之外"的条数（最新一帧）。>0 时主循环 message 会说明：
        #: 官方确实看到了车，只是位置不在带内 —— 这种读数现在也采信（见
        #: `_read_sdk_blockage`），所以这句话主要是给实车排查用的。
        self.sdk_outside_band = 0
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
        self._handback_count = 0
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
        self._handback_count = 0
        self._last_line_mask = None
        self._state_since = None
        self._run_started_at = None
        with self._sdk_lock:
            # 被迫结束后把官方读数也丢掉：那是上一次接管时的画面，不能接着用。
            self._sdk_rows = ()
            self._sdk_pushed_at = None
        self.last_sdk_mode = ""
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
        # 封锁期内（刚走完一个岔路 / 刚被人工或协调器踢出来）这一帧必定是
        # "不接管"，而看一次画面（岔路检测）是整帧里最贵的一步 —— 那就别看。
        # 实车日志里 `free_junction step was slow` 有相当一部分就出在这些帧上
        # （2026-09-16 19:58 的 14.7s/15.8s、2026-09-17 11:43 的 18.9s）。
        if self.state is JunctionState.IDLE and self._rearm_cooling(moment):
            self._confirm_count = 0
            return self._not_triggered("rearm gate")

        fork, roi, line, rect = self.detector.analyze(frame.image)
        self.last_detection = fork if fork.valid else None
        self._last_blue_ratio = fork.blue_ratio
        self._last_line_mask = line
        # 只有"要不要接管"（IDLE）和"往哪边拐"（DECIDE）需要判据；APPROACH / TURN /
        # EXIT 用的是**已经定下来的分支**，再算一次纯属白花时间（实车日志里
        # `free_junction step was slow` 有几次就落在这些阶段）。
        if self._wants_blockage_reading():
            self.last_blockage = self._read_blockage(fork, frame.image, rect, moment)
        reading = self.last_blockage
        self.last_visual = self._visual(fork, self.chosen_branch)

        if self.state is JunctionState.IDLE:
            return self._step_idle(fork, reading, moment)
        return self._step_active(fork, reading, moment)

    # -- 官方 SDK 观测（集成层推来的纯数据）--------------------------------

    def update_robot_observations(
        self,
        candidates: Iterable = (),
        observed_at: Optional[float] = None,
        now: Optional[float] = None,
    ) -> None:
        """主循环推来的**官方 SDK 机器人识别**观测快照（与 5 号 `obstacle.py` 同一套接口）。

        接这条通路**不需要改 `main.py` / `task_registry.py`**：`main.py` 的
        `feed_robot_observations()` 每帧对任何实现了本方法的名字调一次
        （``push(rows, observed_at)`` —— **位置参数**，喂在 `coordinator.step()` 之前）。

        ⚠️ **不要**改成 `update_candidates`：那个名字属于 `number_marker` 的
        **视觉标签(marker)** 通道（`feed_marker_observations()`），标签不是车。

        时间戳参数**两个名字都收**（``observed_at`` 和 ``now``，位置传入也行）：
        集成层按位置传值，而本项目其它模块的写法两种都有 —— 2026-09-18 就因为
        只收 ``now`` 让一处调用炸了 `TypeError`，没必要再踩第二次。

        本模块只**存快照**：不订阅、不碰 SDK、不做判定 —— 判定在
        `_read_blockage()` 里，而且只在 `blockage_source` 选了官方读数时才用。

        :param candidates: 可迭代，元素可以是 `MarkerCandidate` 这类对象
            （``center``/``width``/``height``/``observed_at``），也可以是 SDK 原始行
            ``(x, y, w, h)``（机器人识别）或 ``(x, y, w, h, 标签)``（视觉标签）。
            坐标是归一化还是像素都能认（见 `sighting_from_observation`）。
        :param observed_at: 官方回调的**接收时刻**（集成层就是这么传的）。
        :param now: 同上（兼容旧写法）；两个都不给就用单调钟。
        """
        snapshot = self._as_row_tuple(candidates)
        stamp = self._as_stamp(observed_at if observed_at is not None else now)
        with self._sdk_lock:
            self._sdk_rows = snapshot
            self._sdk_pushed_at = stamp

    @staticmethod
    def _as_row_tuple(rows: Iterable) -> Tuple:
        try:
            return tuple(rows) if rows is not None else ()
        except TypeError:
            return ()

    @staticmethod
    def _as_stamp(value: Optional[float]) -> Optional[float]:
        if value is None:
            return time.monotonic()
        try:
            stamp = float(value)
        except (TypeError, ValueError):
            return None
        return stamp if math.isfinite(stamp) else None

    def _sdk_sightings(
        self, frame_width: int, frame_height: int, now: float
    ) -> List[SdkSighting]:
        """快照 → 整帧像素坐标的观测列表，**顺带扔掉过期的**。

        过期这一条很关键：官方识别是"有就推"的回调，车拐过去之后旧读数可能还挂在
        快照里；拿一条过期读数去选路，比"没有判据"危险得多。
        """
        with self._sdk_lock:
            rows = self._sdk_rows
            pushed_at = self._sdk_pushed_at
        hold = max(0.0, float(self.settings.sdk_observation_hold_seconds))
        sightings: List[SdkSighting] = []
        mode = ""
        for row in rows:
            sighting = sighting_from_observation(row, frame_width, frame_height)
            if sighting is None:
                continue
            stamp = sighting.observed_at if sighting.observed_at is not None else pushed_at
            if stamp is None:
                continue            # 没时间戳 = 不知道新不新鲜 → 不敢用
            age = float(now) - float(stamp)
            if not math.isfinite(age) or age < 0.0 or age > hold:
                continue
            if observation_is_normalized(row):
                mode = mode or "normalized"
            else:
                mode = "pixels"
            sightings.append(sighting)
        self.last_sdk_mode = mode
        return sightings

    # -- 拥堵读数 ---------------------------------------------------------

    def _wants_blockage_reading(self) -> bool:
        """这一帧要不要算"哪条分支堵"（算一次 5~9 ms，别在不需要时白算）。

        * IDLE：要 —— 决定要不要接管；
        * DECIDE：要 —— 落子前让判据稳定；
        * APPROACH / TURN / EXIT / 终态：**不要** —— 走的哪条分支早就定了，
          这一段再看判据对结果没有任何影响（实车日志里 step 超预算有几次就在这儿）。
        """
        return self.state in (JunctionState.IDLE, JunctionState.DECIDE)

    def _read_blockage(
        self, fork: ForkDetection, frame, rect, now: Optional[float] = None
    ) -> BlockageReading:
        """读两条分支走廊：哪边停着车（A5）。

        按 `blockage_source` 选判据来源：默认自己的画面判据；
        选了官方读数（`"sdk"` / `"sdk_or_vision"`）时先看官方读数。

        区域 = **整帧高度上的一条带**（`blockage_top_ratio` ~ `blockage_bottom_ratio`），
        横向沿用岔路 ROI 的左右边界，再以分叉点 `split_x` 分成左右两块。
        两种来源**用同一条带、同一个分叉点**，所以左右的定义完全一致。

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

        source = str(getattr(settings, "blockage_source", "vision")).strip().lower()
        if source not in ("vision", "sdk", "sdk_or_vision"):
            source = "vision"
        moment = self._last_now if now is None else float(now)
        if source != "vision":
            official = self._read_sdk_blockage(fork, image, rect, moment)
            # "sdk"：官方说没有就是没有（这是它的判据，不再退回画面）；
            # "sdk_or_vision"：官方这一帧没读数 → 退回画面判据兜底。
            if source == "sdk" or official.blocked:
                return official

        left_edge, right_edge = int(rect[0]), int(rect[2])
        top = int(_clamp(settings.blockage_top_ratio, 0.0, 0.9) * height)
        bottom = int(_clamp(settings.blockage_bottom_ratio, 0.1, 1.0) * height)
        if bottom - top < 16 or right_edge - left_edge < 16:
            return BlockageReading(BLOCKAGE_NONE)
        region = image[top:bottom, left_edge:right_edge]

        # **在缩小的图上算判据**（`vehicle_downscale`）：深色占比、彩色占比都是
        # 尺度无关的，离线实测整幅画面缩到 0.45 倍仍判对；而这一步是 step() 的大头
        # （实车 2026-09-16 19:58 那次运行里 `free_junction step was slow` 就是它）。
        # 像素单位的形态学核跟着缩放，框再换算回整帧坐标。
        factor = self._vehicle_scale(region)
        work = region
        if factor != 1.0:
            work = cv2.resize(
                region,
                (
                    max(8, int(round(region.shape[1] * factor))),
                    max(8, int(round(region.shape[0] * factor))),
                ),
                interpolation=cv2.INTER_AREA,
            )
        work_width = int(work.shape[1])

        divider = int(
            max(6, min(work_width - 6, int(round((int(fork.split_x) - left_edge) * factor))))
        )
        left_region = work[:, :divider]
        right_region = work[:, divider:]

        # 蓝带掩码在这块区域里算**一次**，左右两侧共用（省一半时间，真车日志里
        # step() 曾经因为每侧各算一遍而超预算）。判据本身在 VehicleDetector 里。
        line_region = _blue_mask(work, self.settings)
        mask = self.vehicle.build_mask(work, line_region, factor)
        left_blocked, left_score, left_box = self.vehicle.pick(
            mask[:, :divider], left_region, line_region[:, :divider]
        )
        right_blocked, right_score, right_box = self.vehicle.pick(
            mask[:, divider:], right_region, line_region[:, divider:]
        )
        # 颜色判据没认出来的那一侧，再走一遍**不看颜色**的兜底判据：
        # 现场（2026-09-18）那辆停着的车没有彩色装甲，只靠颜色整车都找不到。
        if not left_blocked:
            left_blocked, left_score, left_box = self.vehicle.pick_occluder(
                left_region, line_region[:, :divider], factor
            )
        if not right_blocked:
            right_blocked, right_score, right_box = self.vehicle.pick_occluder(
                right_region, line_region[:, divider:], factor
            )

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
            left_box=self._to_frame(left_box, left_edge, top, factor),
            right_box=self._to_frame(
                right_box, left_edge + int(round(divider / factor)), top, factor
            ),
        )

    def _vehicle_scale(self, region) -> float:
        """判据要在多小的图上算（1.0 = 原尺寸，见 `vehicle_downscale`）。"""
        settings = self.settings
        try:
            scale = float(getattr(settings, "vehicle_downscale", 1.0))
        except (TypeError, ValueError):
            return 1.0
        if not math.isfinite(scale) or not 0.0 < scale < 1.0:
            return 1.0
        if region is None or getattr(region, "size", 0) == 0:
            return 1.0
        minimum = int(getattr(settings, "vehicle_downscale_min_pixels", 0) or 0)
        if int(region.shape[0]) * int(region.shape[1]) < max(0, minimum):
            return 1.0
        return scale

    def _official_note(self) -> str:
        """官方识别一直没读数时，给 message 加一句"现在用的是画面判据兜底"。

        实车上判断"集成层到底有没有把 robot 识别接进来"就靠这句话：
        看到它 = 官方读数没到（或者没订阅），消息里那套读数来自画面判据。
        """
        source = str(getattr(self.settings, "blockage_source", "")).strip().lower()
        if source not in ("sdk", "sdk_or_vision"):
            return ""
        if self.official_sightings > 0:
            if self.sdk_outside_band:
                return ("official robot detected outside the junction band (%d); "
                        % self.sdk_outside_band)
            return ""
        return "official robot detection unavailable (0 sightings so far; picture criterion); "

    def _read_sdk_blockage(
        self, fork: ForkDetection, image, rect, now: float
    ) -> BlockageReading:
        """**官方识别**读数 → 左右分支堵不堵（`blockage_source="sdk"`）。

        判据只有一条：官方 SDK 在**某条分支的走廊里**报出了一辆 RoboMaster 小车
        （机器人识别），或者一个视觉标签。**不看长宽高、不看颜色、不看形状** ——
        尺寸只用来记日志（宽占比），不参与判定；这样车离岔路口 1 米、2 米都一样成立。

        左右的定义和画面判据完全一致：同一条检测带 + 同一个分叉点 `split_x`。
        """
        settings = self.settings
        height, width = int(image.shape[0]), int(image.shape[1])
        if height <= 0 or width <= 0:
            return BlockageReading(BLOCKAGE_NONE)
        left_edge, right_edge = int(rect[0]), int(rect[2])
        top = int(_clamp(settings.blockage_top_ratio, 0.0, 0.9) * height)
        bottom = int(_clamp(settings.blockage_bottom_ratio, 0.1, 1.0) * height)
        split = int(fork.split_x)

        left_score = 0.0
        right_score = 0.0
        left_box: Optional[Tuple[int, int, int, int]] = None
        right_box: Optional[Tuple[int, int, int, int]] = None
        sightings = self._sdk_sightings(width, height, now)
        if sightings:
            self.official_sightings += len(sightings)
        in_band = []
        far = []
        for sighting in sightings:
            center_x, center_y = sighting.center
            inside = (top <= center_y <= bottom) and (left_edge <= center_x <= right_edge)
            if inside:
                in_band.append(sighting)
            elif center_y < top:
                # 比判据带更远（更靠画面上方）的官方读数**只计数、不采信**：
                # 计数是给实车排查用的（message 会写"official robot detected outside
                # the junction band"），采信它会让场外无关的机器人抢戏。
                # 判据带本身已经覆盖画面 0.10~0.86，1 米外的车也在里面。
                far.append(sighting)
        self.sdk_outside_band = len(far)
        for sighting in in_band:
            center_x, _center_y = sighting.center
            ratio = sighting.width_ratio(width)
            if center_x < split:
                if left_box is None or ratio > left_score:
                    left_score, left_box = ratio, sighting.box()
            else:
                if right_box is None or ratio > right_score:
                    right_score, right_box = ratio, sighting.box()

        if left_box is not None and right_box is not None:
            reading = BLOCKAGE_BOTH
        elif left_box is not None:
            reading = BLOCKAGE_LEFT
        elif right_box is not None:
            reading = BLOCKAGE_RIGHT
        else:
            reading = BLOCKAGE_NONE
        return BlockageReading(
            reading=reading,
            left_evidence=round(float(left_score), 3),
            right_evidence=round(float(right_score), 3),
            left_box=left_box,
            right_box=right_box,
            source="sdk",
        )

    @staticmethod
    def _to_frame(box, offset_x, offset_y, scale: float = 1.0):
        """区域内（可能已缩放）的外接框坐标 → 整幅图像坐标（给 evidence / 复盘用）。"""
        if box is None:
            return None
        try:
            factor = float(scale)
        except (TypeError, ValueError):
            factor = 1.0
        if not math.isfinite(factor) or factor <= 0.0:
            factor = 1.0
        x0, y0, x1, y1 = box
        return (
            int(round(x0 / factor)) + offset_x,
            int(round(y0 / factor)) + offset_y,
            int(round(x1 / factor)) + offset_x,
            int(round(y1 / factor)) + offset_y,
        )

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
            return Branch.RIGHT, "left branch blocked by %s" % self._blocker_text(
                reading, reading.left_evidence)
        if reading.reading == BLOCKAGE_RIGHT:
            return Branch.LEFT, "right branch blocked by %s" % self._blocker_text(
                reading, reading.right_evidence)
        if reading.reading == BLOCKAGE_BOTH:
            return self._fallback("both branches blocked by vehicles")
        return None, "no vehicle on either branch"

    @staticmethod
    def _blocker_text(reading: BlockageReading, evidence: float) -> str:
        """理由里写清楚这条判据是**谁**给的（实车复盘时一眼能分）。"""
        if getattr(reading, "source", "vision") == "sdk":
            return "a RoboMaster vehicle seen by the official detector (width %.2f)" % evidence
        return "a vehicle (%.2f)" % evidence

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
            now, "%sjunction confirmed, deciding (%s; %s)"
            % (self._official_note(), reason, reading.describe())
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
                    now, "%staking the %s branch (%s; %s)"
                    % (self._official_note(), branch.value, reason, reading.describe())
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
            # 已经对准了就别再转完整个 APPROACH：原来一定要转满 1.5 s（或等岔路消失），
            # 结果带着满舵转过头，还得靠后面一段把车身拉回来
            # （2026-09-17 12:13~12:18 四次实车都是这个形状）。
            offset = self._turn_offset(fork)
            if offset is not None and abs(offset) <= float(settings.center_tolerance_degrees):
                self._center_count += 1
            else:
                self._center_count = 0
            aligned = self._center_count >= max(1, int(settings.center_confirm_frames))
            if (gone
                    or aligned
                    or self._elapsed(now) >= settings.approach_seconds_max * 0.75):
                self._enter(JunctionState.TURN, now)
            else:
                return self._running(now, "aligning with the %s branch" % self._branch_name())

        if self.state is JunctionState.TURN:
            if self._elapsed(now) >= settings.turn_timeout:
                return self._fail(now, "turn timed out")
            # **闭环微转**：盯住"选中那条分支"，朝它转；偏得越多转得越快（限幅很小）。
            offset = self._turn_offset(fork)
            # 收工条件：**岔路已经到跟前**、而且选中分支在正前方容差内。
            # 岔路还在远处时不许收工 —— 那时车头还没到路口，收工就等于"没拐"
            # （2026-09-17 19:48 那次 TURN 只跑了 0.3 s，车几乎直行开进左支）。
            # 岔路已经从画面里消失（车进了分支）→ 这时看脚下胶带就够了。
            fork_visible = fork is not None and fork.valid
            ready = (not fork_visible) or self._fork_is_close(fork)
            if offset is None or not ready:
                self._center_count = 0
            elif abs(offset) <= float(settings.center_tolerance_degrees):
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

        return self._finish_exit(now, fork)

    def _finish_exit(self, now: float, fork: Optional[ForkDetection] = None) -> TaskUpdate:
        """EXIT 阶段：**微微前进 + 顺线**，直到可以安全交回巡线。

        交回条件（要求 3）：**岔路标志已经离开视野**，而且视野里重新有
        **一条清晰、单根的蓝线**（脚下那一小条里恰好只有一段胶带），
        连续 `handback_confirm_frames` 帧 → COMPLETED。

        为什么不是"有蓝线像素就行"：车还压在岔路口上时，脚下常常同时有一段主干和
        一段分支（两段），底层巡线拿到这种画面会自己判丢线 —— 交回也没用。
        时间到了但条件还没满足 → 也交回（COMPLETED），把控制权还给巡线；
        拖过 `exit_timeout` → FAILED 停车。
        """
        settings = self.settings
        fork_gone = fork is None or not fork.valid
        line_back = self._last_blue_ratio >= settings.min_blue_ratio
        single_line = self._tape_offset() is not None
        if fork_gone and line_back and single_line:
            self._handback_count += 1
        else:
            self._handback_count = 0
        if self._handback_count >= max(1, int(settings.handback_confirm_frames)):
            return self._complete(now, "junction left the view and the line is back")
        if self._elapsed(now) >= settings.exit_timeout:
            return self._fail(now, "exit timed out")
        if self._elapsed(now) >= settings.exit_seconds:
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
        self._handback_count = 0

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
                    self._approach_yaw() * self._turn_rate_scale(self.last_detection),
                    -settings.max_approach_yaw, settings.max_approach_yaw,
                ),
            )
        if self.state is JunctionState.TURN:
            # 闭环微转：看得到岔路就朝"选中那条分支"转，岔路没了就朝车头前的胶带转；
            # 越接近正前方转得越慢（turn_yaw 本身也已经压到 20 deg/s）。
            offset = self._turn_offset(self.last_detection)
            if offset is None:
                yaw = settings.turn_yaw * self._branch_sign()
            else:
                yaw = _clamp(
                    settings.turn_steer_gain * offset,
                    -settings.turn_yaw,
                    settings.turn_yaw,
                )
            yaw *= self._turn_rate_scale(self.last_detection)
            return MotionCommand(
                forward=_clamp(settings.turn_forward, 0.0, settings.max_forward),
                lateral=0.0,
                yaw=_clamp(yaw, -settings.max_yaw, settings.max_yaw),
            )
        # EXIT：微微前进 + 顺线（很温柔），保证交回巡线时车头正对着线。
        # 还看得见岔路时（说明车还压在岔路口上）继续盯**选中那条分支** ——
        # 只看脚下那条胶带是不够的：那一刻脚下的可能还是主干，顺着它走会拐回原路
        # （2026-09-17 实车就是"选了右支、脚下却顺着左边那条走掉了"）。
        offset = self._turn_offset(self.last_detection)
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

    def _turn_rate_scale(self, fork: Optional[ForkDetection]) -> float:
        """岔路还"远"的时候把转向速率**按距离线性打折**（远处 `turn_far_yaw_scale`，
        到脚下 1.0）。

        为什么要打折：摄像头装在车头前上方，比车头早 0.5~0.8 m 看到岔路
        （2026-09-17 17:24 的画面量出来：接管那一刻岔路的路口还在画面 y≈250/360，
        车头离路口还有半米多）。一看到就满舵转，等车头到岔路口时方向早就转过去了
        —— 车会"拐早"、切进分支内侧，交回巡线后直接 `LINE_LOST`。

        为什么是**线性**而不是一刀切：一刀切会把"本来就不大的偏角"再削一半，
        浅岔路下 yaw 只剩 ±1、车几乎不转（2026-09-17 19:48/19:49 两次实车）。
        所以按"岔路在 ROI 里落到多低"从 `turn_far_yaw_scale` 线性升到 1.0。

        岔路看不见了（已经开进分支）→ 不打折，交给"顺脚下那条线"的闭环去修。
        """
        settings = self.settings
        if fork is None or not fork.valid or fork.frame_height <= 0:
            return 1.0
        try:
            scale = float(settings.turn_far_yaw_scale)
        except (TypeError, ValueError):
            return 1.0
        if not math.isfinite(scale):
            return 1.0
        cheap = _clamp(scale, 0.0, 1.0)
        near = self._fork_proximity(fork)
        return cheap + (1.0 - cheap) * near

    def _split_offset_degrees(self, fork: ForkDetection) -> float:
        """**分叉点（主干方向）**相对车头偏了多少度（右为正）—— "车对着岔路口吗"。"""
        if fork.frame_width <= 0:
            return 0.0
        return self._pixels_to_degrees(
            float(fork.split_x) - float(fork.frame_width) / 2.0, fork.frame_width
        )

    def _fork_proximity(self, fork: Optional[ForkDetection]) -> float:
        """岔路口离车头多近了：0 = 刚在画面里出现（远），1 = 已经在车头跟前。"""
        if fork is None or not fork.valid or fork.frame_height <= 0:
            return 0.0
        height = int(fork.frame_height)
        top = int(_clamp(self.settings.roi_top, 0.0, 1.0) * height)
        bottom = int(_clamp(self.settings.roi_bottom, 0.0, 1.0) * height)
        if bottom - top <= 0:
            return 0.0
        close_row = top + _clamp(self.settings.fork_close_ratio, 0.0, 1.0) * (bottom - top)
        span = max(1.0, float(close_row - top))
        return _clamp((float(fork.split_row) - top) / span, 0.0, 1.0)

    def _fork_is_close(self, fork: Optional[ForkDetection]) -> bool:
        """岔路口是不是已经到车头跟前了（分叉行落进 ROI 下部 `fork_close_ratio`）。"""
        return self._fork_proximity(fork) >= 1.0

    def _steer_offset_degrees(self, fork: Optional[ForkDetection]) -> Optional[float]:
        """转向/瞄准的目标偏角（度，右为正）—— **按远近把两个目标混起来**：

        * 岔路还远 → 主要瞄**分叉点**（= 主干方向）：先把车顺着主干开到岔路口；
        * 岔路到跟前 → 主要瞄**选中那条分支**：这一步才是"拐进去"。

        为什么要有这个过渡（两个实车教训，方向刚好相反）：
        * 一看到岔路就朝分支打 —— 摄像头比车头早半米看到它，车头还没到路口方向就转过去了，
          于是"拐早"、切进分支内侧（2026-09-17 17:24）；
        * 反过来只瞄"脚下那根线"，或者"没到跟前就完全不瞄分支" —— 浅岔路下 yaw 只剩
          ±1 甚至 0，车几乎直行，直接开进左边那条有车的分支
          （2026-09-17 19:48 / 19:49）。
        所以用**线性混合**而不是硬切换：远的时候主干占多数（不会拐早），
        越近分支分量越大（到跟前就是纯分支方向），中间任何时刻都有一点分支分量。
        """
        if fork is None or not fork.valid or fork.frame_width <= 0:
            return None
        split = self._split_offset_degrees(fork)
        branch = self._aim_offset_degrees(fork)
        near = self._fork_proximity(fork)
        return split + (branch - split) * near

    def _focal_px(self, frame_width: int) -> float:
        """水平焦距（像素）：由水平视场角推出来，用来把像素偏差换算成角度。"""
        hfov = float(getattr(self.settings, "camera_hfov_degrees", 120.0))
        if not math.isfinite(hfov) or not 1.0 < hfov < 179.0:
            hfov = 120.0
        return max(1.0, (max(1, int(frame_width)) / 2.0) / math.tan(math.radians(hfov / 2.0)))

    def _pixels_to_degrees(self, offset_px: float, frame_width: int) -> float:
        """横向偏了多少像素 → 偏了多少度（右为正）。"""
        if not math.isfinite(float(offset_px)):
            return 0.0
        return math.degrees(math.atan2(float(offset_px), self._focal_px(frame_width)))

    def _aim_offset_degrees(self, fork: ForkDetection) -> float:
        """**选中那条分支**相对车头方向偏了多少度（右为正）。

        目标取"分支张开后的方向"（`left_x` / `right_x`，mask 里张开最大的那几行）；
        参考点是**画面中心 = 车头方向**（相机就装在车头上）。

        为什么参考点不能用"脚下那根线"：那是**横向位置**，不是**朝向**。
        车贴着线的一侧走时，"脚下这根线"会把右支的偏角算没 ——
        2026-09-17 19:48/19:49 两次实车就是这么直着开进左边拥堵支的
        （算出来 yaw 只有 ±1，车几乎没转）。
        """
        if self.chosen_branch is None or fork.frame_width <= 0:
            return 0.0
        target = fork.left_x if self.chosen_branch is Branch.LEFT else fork.right_x
        return self._pixels_to_degrees(
            float(target) - float(fork.frame_width) / 2.0, fork.frame_width
        )

    def _approach_yaw(self) -> float:
        """对准阶段：往选中分支偏一点点，每帧只修一点（A8）。

        单位：`approach_yaw_gain` 是"每 1° 偏差给多少 deg/s"，默认 1.0
        （偏 15° 就给 15 deg/s，再由 `max_approach_yaw` 裁到 12）。
        符号沿用项目约定：偏差在右边为正 → yaw 为正（正值右转），
        与巡线控制器一致（`LineController.track` 里 `target_yaw = kp * error`，
        而 `error` 是"线偏右为正"）。
        """
        fork = self.last_detection
        if fork is None:
            return 0.0
        return self.settings.approach_yaw_gain * self._aim_offset_degrees(fork)

    def _aim_reference_x(self, fork: ForkDetection) -> float:
        """保留给测试/排查用的"参考点"：车头正前方（画面中心）。

        （2026-09-17 曾经把它改成"脚下那根线"，结果是**朝向目标被当成了位置目标**，
        浅岔路下 yaw 只有 ±1 —— 已改回画面中心，见 `_aim_offset_degrees`。）
        """
        return float(fork.frame_width) / 2.0

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
        """转弯/退出阶段"还要往哪边转多少" —— 单位是**度**（右为正）。

        * 还看得见岔路 → `_steer_offset_degrees`（远瞄主干、近瞄选中分支）；
        * 岔路已经从画面里消失 → 车头前方那条胶带的偏角（这时它就是分支的胶带）。

        为什么不能只看胶带：刚进转弯时车头前面那条是**主干**，正的、就在中央，
        只按它判会得出"已经对齐"→ 根本不转（2026-09-16 本地测试踩出来过）。
        """
        offset = self._steer_offset_degrees(fork)
        if offset is not None:
            return offset
        tape = self._tape_offset()
        if tape is None:
            return None
        line = self._last_line_mask
        roi_width = 0 if line is None else int(line.shape[1])
        if roi_width <= 0:
            return None
        frame_width = int(fork.frame_width) if fork is not None else 0
        if frame_width <= 0:
            return None
        return self._pixels_to_degrees(tape * (roi_width / 2.0), frame_width)

    def _fork_gone(self, fork: ForkDetection) -> bool:
        """分叉是否已经消失（说明车已经开进分支里了）。"""
        if fork.valid:
            self._clear_count = 0
            return False
        self._clear_count += 1
        return self._clear_count >= 2

    def _rearm_cooling(self, now: float) -> bool:
        """封锁闸门的**冷却期**内吗（这段时间看不到岔路也不许再接管）。

        冷却期内 `step()` 直接返回，连画面都不看：反正结果一定是"不接管"，
        而岔路检测是整帧最贵的一步。冷却结束后才开始要求"连续若干帧看不见岔路"。
        """
        return self._rearm_ready_at is not None and float(now) < self._rearm_ready_at

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
