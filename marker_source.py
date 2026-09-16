"""DJI 官方 marker 订阅 → 数字标识模块的观测来源（集成层基础设施）。

为什么它在集成层
----------------
任务模块按契约**不得直接调用 RoboMaster SDK**，而"画面里哪个位置有个数字几"
这个数据只有 SDK 的视觉订阅能给。所以这里做成一个集中适配器，由主循环每帧把
新鲜候选推给需要它的任务（`NumberMarkerTask.update_candidates()`）。

它只做三件事：
  1. 订阅 marker 识别；
  2. 把每次回调**连同接收时刻**存成一个快照；
  3. `candidates(frame, now)` 把快照转成候选（全帧像素坐标）。

它不做识别筛选、不做运动判定、不碰底盘或云台——那些都是 `number_marker.py` 的职责。

SDK 事实（已对 `robomaster/vision.py` 核实）
-------------------------------------------
* 回调签名是 ``callback(rect_info)``，``rect_info`` 是
  ``[(x, y, w, h, marker), ...]``，x/y 是**中心点**。
* 回调运行在 **SDK 自己的线程**里，所以这里的缓存必须加锁。
* `sub_detect_info(name="marker")` 会顺带调用 `_set_color()`；颜色**只能设一个**，
  不设时 SDK 会打一条 warning 并跳过（不影响订阅本身）。

实车必看
--------
* 模块要求观测年龄 <= 0.15 s、目标丢失超时 0.30 s，所以**回调频率必须 >= 约 3.5 Hz**，
  否则瞄准阶段会 `TARGET_LOST_TIMEOUT`。真实频率用 `stats()["callback_hz"]` 看。
* 坐标是归一化还是像素，SDK 文档没写。默认 `auto` 自动判断；实车第一次跑请用
  `stats()["coordinate_mode"]` 确认判断正确，必要时显式指定。
* 颜色：默认不设过滤器。如果一直收不到任何 marker，把 `CONFIG.marker_color`
  依次改成 "red" / "green" / "blue" 再试。

单独自测：python -m unittest tests.test_marker_source
"""

import math
import threading
import time
from typing import Optional, Tuple

from number_marker import MarkerCandidate, marker_candidates_from_normalized

MARKER = "marker"
MARKER_COLORS = ("red", "green", "blue")

#: 自动判断坐标模式的门槛：归一化坐标不会超过它，像素坐标几乎一定超过。
NORMALIZED_MAX = 1.5

#: 低于这个回调频率，模块的观测新鲜度要求就守不住了（0.30s 丢失超时）。
MIN_USEFUL_HZ = 3.5

#: 数字标识模块要求的"标记宽度占画面宽的比例"下限（与 number_marker 的同名设置一致）。
#: 低于它模块会一直判 TARGET_TOO_SMALL：看到标识也不停车、不拍照。
MIN_USEFUL_WIDTH_RATIO = 0.20


def _finite(value) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _candidates_from_pixels(
    rows, received_at: float
) -> Tuple[MarkerCandidate, ...]:
    """像素坐标的回调元组 → 候选（不做任何 ID 过滤）。"""
    converted = []
    for row in rows:
        try:
            x, y, width, height, target_id = row[:5]
        except (TypeError, ValueError, IndexError):
            continue
        x = _finite(x)
        y = _finite(y)
        width = _finite(width)
        height = _finite(height)
        if None in (x, y, width, height):
            continue
        converted.append(
            MarkerCandidate(
                target_id=str(target_id),
                center=(x, y),
                width=width,
                height=height,
                observed_at=float(received_at),
            )
        )
    return tuple(converted)


class MarkerObservationSource:
    """集中式 marker 观测缓存。没有 vision 对象时是安全的空操作。"""

    name = "marker_source"

    def __init__(
        self,
        vision=None,
        color: str = "",
        coordinate_mode: str = "auto",
    ) -> None:
        self._vision = vision
        self.color = color if color in MARKER_COLORS else ""
        self.coordinate_mode = (
            coordinate_mode if coordinate_mode in ("auto", "normalized", "pixels")
            else "auto"
        )

        self._lock = threading.Lock()
        self._marker_info: tuple = ()
        self._received_at: Optional[float] = None
        self._resolved_mode: Optional[str] = None

        self._subscribed = False
        self.subscribe_result: Optional[bool] = None
        self.callbacks = 0
        self.empty_callbacks = 0
        self.first_callback_at: Optional[float] = None
        self.last_callback_at: Optional[float] = None
        # 宽度诊断：数字标识模块要求"标记宽度 > 画面宽 20%"，否则它会一直判
        # TARGET_TOO_SMALL 而不接管（不拍照、不停车）。只记"快照里有几个 marker"
        # 回答不了"为什么没接管"，所以这里把看到过的最宽候选也记下来。
        self.observed_candidates = 0
        self._max_width_ratio: Optional[float] = None
        self._max_width_target: str = ""

    # ------------------------------------------------------------------
    # 对外
    # ------------------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._subscribed

    @property
    def coordinate_mode_resolved(self) -> str:
        return self._resolved_mode or self.coordinate_mode

    def start(self) -> bool:
        """订阅 marker 识别。失败只返回 False，绝不抛异常。"""
        if self._vision is None:
            self.subscribe_result = False
            return False
        try:
            if self.color:
                result = self._vision.sub_detect_info(
                    name=MARKER, color=self.color, callback=self._on_detect_info
                )
            else:
                # 不传 color：SDK 只会打一条 warning，订阅照旧。
                result = self._vision.sub_detect_info(
                    name=MARKER, callback=self._on_detect_info
                )
        except Exception:
            result = False
        self.subscribe_result = bool(result)
        self._subscribed = self.subscribe_result
        return self._subscribed

    def stop(self) -> None:
        """退订。绝不抛异常。"""
        if self._vision is not None and self._subscribed:
            try:
                self._vision.unsub_detect_info(MARKER)
            except Exception:
                pass
        self._subscribed = False

    def candidates(self, frame, now: float) -> Tuple[MarkerCandidate, ...]:
        """把最近一次回调转成候选；没有快照就返回空元组。

        注意 ``observed_at`` 用的是**回调接收时刻**，不是这一帧的时刻——
        模块就是靠它判断观测是否过期，所以这里绝不能刷新成 now。
        """
        with self._lock:
            rows = self._marker_info
            received_at = self._received_at
        if not rows or received_at is None:
            return ()

        image = getattr(frame, "image", None)
        if image is None or getattr(image, "ndim", 0) != 3:
            return ()
        height, width = image.shape[:2]
        if height <= 0 or width <= 0:
            return ()

        mode = self._resolve_mode(rows)
        if mode == "normalized":
            result = marker_candidates_from_normalized(
                rows, width, height, received_at, None
            )
        else:
            result = _candidates_from_pixels(rows, received_at)
        self._note_width(result, width)
        return result

    def _note_width(self, candidates, frame_width: int) -> None:
        """记下"这一帧看到的候选有多宽"——回答"为什么看到了却不接管"。"""
        if not candidates or frame_width <= 0:
            return
        widest = max(candidates, key=lambda item: item.width)
        ratio = widest.width / float(frame_width)
        with self._lock:
            self.observed_candidates += len(candidates)
            if self._max_width_ratio is None or ratio > self._max_width_ratio:
                self._max_width_ratio = ratio
                self._max_width_target = "%s（%.0f 像素）" % (
                    widest.target_id, widest.width)

    def stats(self) -> dict:
        """给实车排查用：频率、坐标模式、快照里几个 marker、看到过的最宽候选。"""
        with self._lock:
            callbacks = self.callbacks
            empty = self.empty_callbacks
            first = self.first_callback_at
            last = self.last_callback_at
            snapshot = len(self._marker_info)
            observed = self.observed_candidates
            max_ratio = self._max_width_ratio
            widest = self._max_width_target
        rate = None
        if first is not None and last is not None and last > first:
            rate = round((callbacks - 1) / (last - first), 2)
        return {
            "subscribed": self._subscribed,
            "subscribe_result": self.subscribe_result,
            "color": self.color or "(未设过滤器)",
            "coordinate_mode": self.coordinate_mode_resolved,
            "callbacks": callbacks,
            "empty_callbacks": empty,
            "callback_hz": rate,
            "markers_in_snapshot": snapshot,
            "observed_candidates": observed,
            "max_width_ratio": (
                None if max_ratio is None else round(max_ratio, 3)
            ),
            "widest_target": widest or "(没有看到候选)",
        }

    def _width_warning(self) -> str:
        """看到的候选一直太窄时，直接说出这是模块不接管的原因。"""
        stats = self.stats()
        ratio = stats["max_width_ratio"]
        if ratio is None or ratio > MIN_USEFUL_WIDTH_RATIO:
            return ""
        return (
            "看到过的最宽候选只有画面宽的 %.1f%%（目标 %s），低于模块要求的 "
            "%.0f%%：数字标识会一直判 TARGET_TOO_SMALL —— 看到标识也不会停车、"
            "不会拍照。把车开近些，或核对 SDK 报的宽度是不是卡片宽度。"
            % (ratio * 100.0, stats["widest_target"],
               MIN_USEFUL_WIDTH_RATIO * 100.0)
        )

    def rate_warning(self) -> str:
        """回调太慢或候选太窄时给一句人话，说明为什么数字标识会失手。"""
        width = self._width_warning()
        if width:
            return width
        rate = self.stats()["callback_hz"]
        if rate is None:
            return "还没有收到任何 marker 回调。"
        if rate < MIN_USEFUL_HZ:
            return (
                "marker 回调只有 %.2f Hz，低于模块要求的约 %.1f Hz："
                "瞄准阶段会 TARGET_LOST_TIMEOUT。" % (rate, MIN_USEFUL_HZ)
            )
        return ""

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _on_detect_info(self, marker_info) -> None:
        """SDK 线程回调。只更新快照，不做任何判定。"""
        now = time.monotonic()
        try:
            rows = tuple(tuple(row) for row in (marker_info or ()))
        except TypeError:
            rows = ()
        with self._lock:
            self.callbacks += 1
            if self.first_callback_at is None:
                self.first_callback_at = now
            self.last_callback_at = now
            if not rows:
                # 空回调 = 这一帧没有 marker。必须清掉旧快照，否则会拿着一个
                # 早就消失的目标当新鲜观测。
                self.empty_callbacks += 1
                self._marker_info = ()
                self._received_at = now
                return
            self._marker_info = rows
            self._received_at = now

    def _resolve_mode(self, rows) -> str:
        if self.coordinate_mode != "auto":
            return self.coordinate_mode
        if self._resolved_mode is None:
            biggest = 0.0
            for row in rows:
                for value in row[:4]:
                    number = _finite(value)
                    if number is not None:
                        biggest = max(biggest, abs(number))
            self._resolved_mode = "normalized" if biggest <= NORMALIZED_MAX else "pixels"
        return self._resolved_mode
