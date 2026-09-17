"""DJI 官方"机器人识别"订阅 → 障碍物模块的观测来源（集成层基础设施）。

为什么它在集成层
----------------
任务模块按契约**不得直接调用 RoboMaster SDK**，而"画面里哪个位置有一台同型机器人、
框有多大"这个数据只有 SDK 的视觉订阅能给（自己用颜色/边缘猜，误触发率下不来）。
所以这里做成一个集中适配器，由主循环每帧把最新观测推给需要它的任务
（`ObstacleTask.update_robot_observations(rows, observed_at)`）。

它只做三件事：
  1. 订阅 robot 识别；
  2. 把每次回调**连同接收时刻**存成一个快照（空回调会清掉旧快照）；
  3. `observations(frame)` 把快照转成 `(x, y, w, h)`（**中心点 + 宽高**，整帧像素）。

它不做识别筛选、不做运动判定、不碰底盘或云台——那些都是 `obstacle.py` 的职责。

SDK 事实（已对 `robomaster/vision.py` 核实）
-------------------------------------------
* `sub_detect_info(name="robot", callback=cb)` 合法；模块级常量就是 `ROBOT = "robot"`。
* 回调签名 `callback(rect_info)`，robot 的 `rect_info = [(x, y, w, h), ...]`，
  x/y 是**中心点**、w/h 是宽高（与 `obstacle.py` 期望的格式一致）。
* 回调运行在 **SDK 自己的线程**里，所以缓存必须加锁。
* 坐标是归一化还是像素，SDK 文档没写 → 与 marker 通道一样用 `auto` 判断。
* `unsub_detect_info("robot")` 退订。

实车必看
--------
* 模块要求观测年龄 <= `ROBOT_OBSERVATION_MAX_AGE`（0.35 s），所以回调频率至少要有
  约 3 Hz，否则"车在前面"这个信息会一直过期、障碍模块退化成灰度结构判据。
  真实频率看 `stats()["callback_hz"]`。
* 模块还要求框宽 >= 画面宽的 `ROBOT_MIN_WIDTH_RATIO`（6%），太远就不会绕。
  看到过的最宽框记在 `stats()["max_width_ratio"]`，`rate_warning()` 会直接说明原因。

单独自测：python -m unittest tests.test_robot_source
"""

import math
import threading
import time
from typing import Optional, Tuple

#: SDK 的视觉订阅名（`robomaster.vision.ROBOT` 的值）。
ROBOT = "robot"

#: 自动判断坐标模式的门槛：归一化坐标不会超过它，像素坐标几乎一定超过。
NORMALIZED_MAX = 1.5

#: 低于这个回调频率，模块的观测新鲜度要求就守不住了（0.35 s 过期）。
MIN_USEFUL_HZ = 3.0

#: 障碍模块要求的"机器人框宽度占画面宽的比例"下限，必须与
#: `obstacle.ROBOT_MIN_WIDTH_RATIO` 一致（有测试钉死两者相等）。
MIN_USEFUL_WIDTH_RATIO = 0.06


def _finite(value) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


class RobotObservationSource:
    """集中式机器人识别观测缓存。没有 vision 对象时是安全的空操作。"""

    name = "robot_source"

    def __init__(self, vision=None) -> None:
        self._vision = vision

        self._lock = threading.Lock()
        self._rows: tuple = ()
        self._received_at: Optional[float] = None
        self._resolved_mode: Optional[str] = None

        self._subscribed = False
        self.subscribe_result: Optional[bool] = None
        self.callbacks = 0
        self.empty_callbacks = 0
        self.first_callback_at: Optional[float] = None
        self.last_callback_at: Optional[float] = None
        # 宽度诊断：模块要求框宽 >= 画面宽 6%，否则它会一直判"太远"而不绕。
        # 只记"快照里有几个框"回答不了"为什么看到了却不绕"。
        self.observed_boxes = 0
        self._max_width_ratio: Optional[float] = None
        self._max_width_at: Optional[Tuple[float, float]] = None

    # ------------------------------------------------------------------
    # 对外
    # ------------------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._subscribed

    @property
    def coordinate_mode_resolved(self) -> str:
        return self._resolved_mode or "auto"

    def start(self) -> bool:
        """订阅机器人识别。失败只返回 False，绝不抛异常。"""
        if self._vision is None:
            self.subscribe_result = False
            return False
        try:
            result = self._vision.sub_detect_info(
                name=ROBOT, callback=self._on_detect_info
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
                self._vision.unsub_detect_info(ROBOT)
            except Exception:
                pass
        self._subscribed = False

    def observations(self, frame) -> Tuple[Tuple[Tuple[float, float, float, float], ...], Optional[float]]:
        """返回 ``(rows, observed_at)``：``rows`` 每项是 ``(x, y, w, h)``（中心点 + 宽高，像素）。

        ``observed_at`` 是**回调接收时刻**，不是这一帧的时刻 —— 模块就是靠它判断观测
        是否过期，所以这里绝不能刷新成 now。没有快照时返回 ``((), None)``。
        """
        with self._lock:
            rows = self._rows
            received_at = self._received_at
        if not rows or received_at is None:
            return (), None

        image = getattr(frame, "image", None)
        if image is None or getattr(image, "ndim", 0) != 3:
            return (), None
        height, width = image.shape[:2]
        if height <= 0 or width <= 0:
            return (), None

        mode = self._resolve_mode(rows)
        converted = []
        for row in rows:
            try:
                x, y, box_w, box_h = row[:4]
            except (TypeError, ValueError, IndexError):
                continue
            x = _finite(x)
            y = _finite(y)
            box_w = _finite(box_w)
            box_h = _finite(box_h)
            if None in (x, y, box_w, box_h) or box_w <= 0 or box_h <= 0:
                continue
            if mode == "normalized":
                x *= width
                box_w *= width
                y *= height
                box_h *= height
            converted.append((x, y, box_w, box_h))
        result = tuple(converted)
        self._note_width(result, width)
        return result, received_at

    def _note_width(self, rows, frame_width: int) -> None:
        """记下"这一帧看到的框有多宽"——回答"为什么看到车却不绕"。"""
        if not rows or frame_width <= 0:
            return
        widest = max(rows, key=lambda item: item[2])
        ratio = widest[2] / float(frame_width)
        with self._lock:
            self.observed_boxes += len(rows)
            if self._max_width_ratio is None or ratio > self._max_width_ratio:
                self._max_width_ratio = ratio
                self._max_width_at = (round(widest[0]), round(widest[1]))

    def stats(self) -> dict:
        """给实车排查用：频率、坐标模式、快照里几个框、看到过的最宽框。"""
        with self._lock:
            callbacks = self.callbacks
            empty = self.empty_callbacks
            first = self.first_callback_at
            last = self.last_callback_at
            snapshot = len(self._rows)
            observed = self.observed_boxes
            max_ratio = self._max_width_ratio
            widest_at = self._max_width_at
        rate = None
        # 只收到 1 次回调时算不出频率（分母为 0）。这时**不要**报 0 Hz —— 那会
        # 在刚启动的两秒里误报"回调太慢"。次数不够就明确说"测不出来"。
        if callbacks > 1 and first is not None and last is not None and last > first:
            rate = round((callbacks - 1) / (last - first), 2)
        return {
            "subscribed": self._subscribed,
            "subscribe_result": self.subscribe_result,
            "coordinate_mode": self.coordinate_mode_resolved,
            "callbacks": callbacks,
            "empty_callbacks": empty,
            "callback_hz": rate,
            "robots_in_snapshot": snapshot,
            "observed_boxes": observed,
            "max_width_ratio": (None if max_ratio is None else round(max_ratio, 3)),
            "widest_robot_at": (
                "(没有看到机器人)" if widest_at is None else "中心(%d,%d)" % widest_at
            ),
        }

    def rate_warning(self) -> str:
        """回调太慢或框太窄时给一句人话，说明障碍模块为什么会失手。"""
        stats = self.stats()
        ratio = stats["max_width_ratio"]
        if ratio is not None and ratio <= MIN_USEFUL_WIDTH_RATIO:
            return (
                "看到过的最宽机器人框只有画面宽的 %.1f%%（目标 %s），低于模块要求的 "
                "%.0f%%：障碍模块会认为「太远」而不绕。把车开近些再复测。"
                % (ratio * 100.0, stats["widest_robot_at"],
                   MIN_USEFUL_WIDTH_RATIO * 100.0)
            )
        rate = stats["callback_hz"]
        if rate is None:
            if stats["callbacks"] == 0:
                return (
                    "还没有收到任何 robot 回调：障碍模块会退回灰度结构判据"
                    "（误触发率更高）。"
                )
            return (
                "只收到 %d 次 robot 回调，还测不出频率（至少需要 2 次）。"
                % stats["callbacks"]
            )
        if rate < MIN_USEFUL_HZ:
            return (
                "robot 回调只有 %.2f Hz，低于模块要求的约 %.1f Hz：观测会频繁过期，"
                "障碍模块会时有时无地退回灰度结构判据。" % (rate, MIN_USEFUL_HZ)
            )
        return ""

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _on_detect_info(self, rect_info) -> None:
        """SDK 线程回调。只更新快照，不做任何判定。"""
        now = time.monotonic()
        try:
            rows = tuple(tuple(row) for row in (rect_info or ()))
        except TypeError:
            rows = ()
        with self._lock:
            self.callbacks += 1
            if self.first_callback_at is None:
                self.first_callback_at = now
            self.last_callback_at = now
            if not rows:
                # 空回调 = 这一帧没有识别到机器人。必须清掉旧快照，否则会拿着一个
                # 早就开走的车当新鲜观测。
                self.empty_callbacks += 1
                self._rows = ()
                self._received_at = now
                return
            self._rows = rows
            self._received_at = now

    def _resolve_mode(self, rows) -> str:
        if self._resolved_mode is None:
            biggest = 0.0
            for row in rows:
                for value in row[:4]:
                    number = _finite(value)
                    if number is not None:
                        biggest = max(biggest, abs(number))
            self._resolved_mode = "normalized" if biggest <= NORMALIZED_MAX else "pixels"
        return self._resolved_mode
