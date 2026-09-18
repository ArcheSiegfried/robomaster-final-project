"""WP3 traffic light module: distinguish red / green / none.

The car must hold still at a red light and may only be released after a
green light has been confirmed over several consecutive frames.

Design assumptions (module-local, adjustable, see TrafficLightConfig):
- The light is located in the upper part of the frame (ROI defaults to the
  top 60% of the image; the blue line lives in the lower half, so it does
  not overlap the detection region).
- Red and green lamps appear as compact, roughly round HSV blobs. The green
  HSV range is deliberately wide (yellow-green to cyan, low S/V floor) because
  a real LED lamp over-exposes: its core turns white and only a coloured halo
  survives, so a narrow range misses it entirely. Shape filters (aspect ratio
  and fill ratio) keep the wide range from matching long red/green objects.
- Red and green never light at the same time; if both are detected (glare,
  other objects), "red" wins by default: stopping is always safer than an
  unconfirmed release. **Exception, and it is the default: a fork.** When red
  and green are both visible in one frame, the scene is the fork of task 4
  ("left red / right green"), where the car must pick the green side instead
  of stopping. This module reports "no result" there and never takes over
  (`fork_light_competition`), because it only knows colour, not which side the
  lamp is on; choosing the branch belongs to `green_junction`. The self-chosen
  "stop on red / go on green" spot (task 6) has a single lamp and is
  unaffected.
- Green confirmation tolerates dropped frames: a missing green inside
  `green_gap_grace` does NOT reset the streak, so 1-2 frame flicker or a video
  hiccup cannot hold the car forever in front of a lit green lamp.
- "No red seen" is never treated as "green". While stopped, the car keeps
  holding and fails after `max_hold_seconds` of TOTAL stop time; the stop
  clock is not reset by green flicker, so a stuck loop always ends in FAILED.
- A green lamp seen while never having stopped does not take over: the car is
  already driving, and the light only authorises release of a stop.

Contract (MODULE_GUIDE v0.1): this module never connects to the camera, the
SDK or the chassis. It only returns TaskUpdate requests; step() is
non-blocking (no sleep, no long loops, work is confined to its own ROI).
"""

from dataclasses import dataclass, replace
from typing import List, Optional, Tuple

import cv2
import numpy as np

from evidence import make_evidence_photo
from models import (
    FramePacket,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)

KIND = "traffic_light"
RED = "red"
GREEN = "green"

STOP_COMMAND = MotionCommand()


@dataclass(frozen=True)
class EvidenceRequest:
    """One scoring-image request for the integration-owned evidence writer.

    Duck-typed against evidence.py render_task_evidence(); the integration
    layer (main.service_task_evidence) polls take_evidence_request() every
    frame and acknowledges the real write result. The traffic-light task does
    NOT block on this request: a red-light screenshot is a scoring side
    effect, never a gate for stopping or releasing.
    """

    request_id: str
    marker_id: str
    frame_sequence: int
    captured_at: float
    detection: "VisualDetection"
    annotation: str
    text_anchor: Tuple[int, int]
    image: np.ndarray
    attempt: int = 1

IDLE = "idle"
HOLDING_RED = "holding_red"
CONFIRM_GREEN = "confirm_green"


@dataclass(frozen=True)
class TrafficLightConfig:
    """All field-dependent values live here; tune on the real course."""

    # Detection ROI (fractions of the full frame). Light sits above the line.
    roi_left: float = 0.10
    roi_right: float = 0.90
    roi_top: float = 0.00
    roi_bottom: float = 0.60

    # Red spans the two hue ends of the HSV wheel.
    red_hsv_ranges: Tuple[Tuple[Tuple[int, int, int], Tuple[int, int, int]], ...] = (
        ((0, 100, 60), (10, 255, 255)),
        ((170, 100, 60), (179, 255, 255)),
    )
    # Wide green range on purpose: real LED cores over-expose to white and only
    # a coloured halo survives; white balance shifts hue between yellow-green
    # and cyan. Shape filters below stop the wide range from matching long
    # red/green objects.
    green_hsv_lower: Tuple[int, int, int] = (28, 40, 40)
    green_hsv_upper: Tuple[int, int, int] = (100, 255, 255)

    open_kernel: int = 3
    close_kernel: int = 7

    min_area: float = 40.0
    max_area_ratio: float = 0.12
    min_vertical_coverage: float = 0.04

    # Shape constraints: a lamp is roughly round. aspect = min(w,h)/max(w,h),
    # fill = contour area / bounding-rect area.
    min_aspect_ratio: float = 0.6
    min_fill_ratio: float = 0.5

    #: **红灯专用**的径向一致性上限：std(半径)/mean(半径)，从质心量到轮廓。
    #:
    #: **为什么需要它**：2026-09-18 实车把标识当成红灯，车停死 15 秒、`number_marker`
    #: 的拍照也被红灯按优先级截断。那个标识是**正方形**的红色标牌（画着数字 1），
    #: aspect≈0.81、fill≈0.77 完美通过上面两条形状门 —— 那两条本是为"排除长条红物"
    #: 设计的，而正方形两项都满足。圆度（circularity）也救不了：22×27 px 的轮廓
    #: 锯齿太重，实测在 0.68~0.74 之间摇摆，阈值放 0.72 会时灵时不灵。
    #:
    #: 径向一致性对这件事**本来就能分开**，而且不怕锯齿：
    #:   真圆灯（合成 r=6/10/15/28）实测 0.012 ~ 0.070
    #:   误检的方形红色标识（真实场地帧）实测 0.179 ~ 0.291
    #: 取 0.12 两边都留足余量。正方形四角半径比边长大 √2 倍，所以它天然 ≥0.12。
    #:
    #: **为什么只管红灯**：两类误判的后果差一个量级。红色误检会把车**停死**
    #: （红灯 2 帧就接管、然后停到超时），必须严格；绿色误检**不会接管**
    #: （绿灯从 IDLE 永不接管，只在已停在红灯前时用于放行），宽松一点更安全。
    #: 实测（真实场地帧）：这条闸门干掉 7 个红色误检，同时会顺手删掉 19 个
    #: **绿色标识/招牌图块** —— 那些是真实存在的目标，删了就削弱"绿灯放行"，
    #: 所以绿色不套这条。
    max_radial_ratio: float = 0.12

    # Scoring weights; confidence = score / (sum of weights) -> full mark = 1.0.
    vertical_weight: float = 1.2
    center_weight: float = 0.8
    area_weight: float = 0.4
    # Vertical coverage that already counts as "full" for scoring.
    coverage_full_ratio: float = 0.20

    # If both colours are visible in one frame, treat it as red (conservative).
    # **只有当 `fork_light_competition` 关掉时这一条才生效**，见下面那个开关。
    red_priority: bool = True

    # 红绿同框（岔路口的"左红右绿"，赛题 4）时本模块**不接管**。
    #
    # 为什么必须是默认行为：岔路口红绿两盏灯同框，而本模块只认颜色不认位置，
    # `red_priority` 会把这一帧判成红灯 → 2 帧后接管 → 原地停到
    # `max_hold_seconds` 超时。而 `green_junction` 要 3 帧才确认岔路，
    # 所以它永远赢不了这场赛跑；灯模块接管之后，协调器的红灯否决权又会把岔路
    # 模块按停（否决时**不调用**它的 step）。实测后果：600 帧里 599 帧零指令，
    # 车钉死在第一岔路口，赛题 4 的 15 分丢失。
    #
    # "往绿灯那侧走"的判据本来就归 `green_junction`（它内置的 LampSpotter 会
    # 同时报出红绿两盏以及各自在画面哪一边）。自选地点的"红灯停绿灯行"
    # （赛题 6）按定义只有一盏灯，不受这条影响。
    fork_light_competition: bool = True

    # Scoring-image config: the final score counts saved images, e.g.
    # "Team 10 detects a red light and stops the robot". One screenshot is
    # requested when the red light is confirmed; it never gates the state
    # machine (a failed write must not turn a red light into a green).
    team_number: str = "10"
    max_evidence_attempts: int = 2

    # Confirmation policy. Asymmetric by design: stop fast (2 frames, ~0.1 s),
    # release slowly (5 frames, ~0.25 s) - never releasing too early is the
    # fail-safe direction, while stopping late is not.
    red_confirm_frames: int = 2
    green_confirm_frames: int = 5
    # Seconds a missing green is tolerated inside green confirmation. Dropped
    # frames / LED flicker inside this window do NOT reset the streak.
    green_gap_grace: float = 0.30

    # Total time the car may stay stopped at the light (any phase, counting
    # from the first hold). NOT reset by green flicker, so a stuck loop always
    # ends in FAILED instead of stopping forever.
    max_hold_seconds: float = 15.0


class TrafficLightDetector:
    """Pure colour detector; returns a VisualDetection, never a motion."""

    def __init__(self, settings: Optional[TrafficLightConfig] = None) -> None:
        self.settings = settings or TrafficLightConfig()

    @staticmethod
    def _kernel(size: int) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def _mask(self, hsv: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        s = self.settings
        red_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in s.red_hsv_ranges:
            red_mask = cv2.bitwise_or(
                red_mask,
                cv2.inRange(
                    hsv,
                    np.array(lower, dtype=np.uint8),
                    np.array(upper, dtype=np.uint8),
                ),
            )
        green_mask = cv2.inRange(
            hsv,
            np.array(s.green_hsv_lower, dtype=np.uint8),
            np.array(s.green_hsv_upper, dtype=np.uint8),
        )
        kernel_open = self._kernel(s.open_kernel)
        kernel_close = self._kernel(s.close_kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel_open)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_close)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel_open)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel_close)
        return red_mask, green_mask

    @staticmethod
    def _radial_ratio(contour: np.ndarray) -> float:
        """轮廓的径向一致性：``std(半径) / mean(半径)``（从质心量到轮廓各点）。

        真圆接近 0；正方形四角比边长大 √2 倍，所以天然偏大。见
        `max_radial_ratio` 的说明：这是把"方形红色标识"从"圆形红灯"里分出来的判据。
        取不到足够点时保守地返回 1.0（= 不合格），宁可漏报也不误停。
        """
        points = contour.reshape(-1, 2).astype(np.float64)
        if len(points) < 5:
            return 1.0
        center_x = points[:, 0].mean()
        center_y = points[:, 1].mean()
        radii = np.hypot(points[:, 0] - center_x, points[:, 1] - center_y)
        mean = float(radii.mean())
        if mean <= 1e-6:
            return 1.0
        return float(radii.std() / mean)

    def _score(
        self, mask: np.ndarray, width: int, height: int, require_round: bool = False
    ) -> List[Tuple[float, Tuple[int, int], Tuple[int, int, int, int]]]:
        """Score blobs by shape (roundness), size, coverage and centering.

        ``require_round=True`` 时额外要求径向一致性合格 —— **只给红灯用**，
        用来把方形红色标识挡在外面（见 `max_radial_ratio`）。绿灯不套这条：
        绿色误检不会接管，削弱它反而会削弱"绿灯放行"。
        """
        s = self.settings
        frame_area = max(width * height, 1)
        candidates = []
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if area < s.min_area or area > frame_area * s.max_area_ratio:
                continue
            coverage = h / max(height, 1)
            if coverage < s.min_vertical_coverage:
                continue
            # Shape gate: lamps are roughly round; long red/green objects
            # (poles, signs, clothes) must never be treated as lamps.
            aspect = min(w, h) / max(max(w, h), 1)
            if aspect < s.min_aspect_ratio:
                continue
            fill = area / max(w * h, 1)
            if fill < s.min_fill_ratio:
                continue
            # 灯是**圆**的。2026-09-18 实车教训：画着数字的**方形**红色标识
            # aspect/fill 都合格（0.81 / 0.77），被当成红灯把车停死 15 秒。
            # 这条把它挡掉：方形实测 0.179~0.291，真圆灯 0.012~0.070。
            # 只对红灯生效（require_round），绿灯不套 —— 见 max_radial_ratio。
            if require_round and self._radial_ratio(contour) > s.max_radial_ratio:
                continue
            center_x = x + w / 2.0
            center_y = y + h / 2.0
            center_distance = abs(center_x - width / 2.0) / max(width / 2.0, 1)
            score = (
                s.vertical_weight * min(coverage / s.coverage_full_ratio, 1.0)
                + s.center_weight * (1.0 - min(center_distance, 1.0))
                + s.area_weight * min((area / frame_area) / 0.10, 1.0)
            )
            candidates.append(
                (
                    float(score),
                    (int(round(center_x)), int(round(center_y))),
                    (x, y, x + w, y + h),
                )
            )
        return candidates

    def detect(self, frame: np.ndarray) -> VisualDetection:
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must be a non-empty BGR image")
        height, width = frame.shape[:2]
        s = self.settings
        left = int(width * s.roi_left)
        right = int(width * s.roi_right)
        top = int(height * s.roi_top)
        bottom = int(height * s.roi_bottom)
        if right <= left or bottom <= top:
            return VisualDetection.no_result("traffic_light")
        roi = frame[top:bottom, left:right]
        if roi.size == 0:
            return VisualDetection.no_result("traffic_light")

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        red_mask, green_mask = self._mask(hsv)
        roi_height, roi_width = roi.shape[:2]
        # 红灯必须"圆"（挡掉方形红色标识）；绿灯不套这条，见 max_radial_ratio。
        red_candidates = self._score(red_mask, roi_width, roi_height,
                                     require_round=True)
        green_candidates = self._score(green_mask, roi_width, roi_height)

        best_red = max(red_candidates, default=None)
        best_green = max(green_candidates, default=None)
        if best_red is not None and best_green is not None:
            # 红绿同框 = 岔路口的"左红右绿"（赛题 4），不是"自选地点的红灯停"
            # （赛题 6，那里只有一盏灯）。两盏同时可见时我们**不报**其中任何一盏：
            # 只认颜色不认位置的读数在岔路口必然把车按停（实测 599/600 帧零指令，
            # 见 tests/test_fork_light_competition.py 的 fork 用例），而"往绿灯那边走"的
            # 判据归 green_junction（它的 LampSpotter 会同时报出两盏和各自在哪边）。
            # 关掉 fork_light_competition 就退回旧的"红优先"行为。
            if s.fork_light_competition:
                return VisualDetection.no_result("traffic_light")
            if s.red_priority:
                best_green = None
            else:
                best_red = None

        if best_red is not None:
            return self._detection("red", best_red, left, top)
        if best_green is not None:
            return self._detection("green", best_green, left, top)
        return VisualDetection.no_result("traffic_light")

    def sees_both_colours(self, frame: np.ndarray) -> bool:
        """这一帧是否同时看到红和绿（= 岔路口"左红右绿"，赛题 4）。

        `detect()` 在红绿同框时按 `fork_light_competition` 返回"无结果"，
        所以任务层需要这个**独立**判据来区分两种"没结果"：
          * 真的没有灯 → 保持原来的行为；
          * 红绿同框 → 这是岔路口，本模块不该接管（见 Task 里的闸门）。
        只允许在"本来就没报出颜色"的帧上调用；它只回答"两色是不是都在"。
        """
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            return False
        height, width = frame.shape[:2]
        s = self.settings
        left = int(width * s.roi_left)
        right = int(width * s.roi_right)
        top = int(height * s.roi_top)
        bottom = int(height * s.roi_bottom)
        if right <= left or bottom <= top:
            return False
        roi = frame[top:bottom, left:right]
        if roi.size == 0:
            return False
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        red_mask, green_mask = self._mask(hsv)
        roi_height, roi_width = roi.shape[:2]
        red = self._score(red_mask, roi_width, roi_height)
        green = self._score(green_mask, roi_width, roi_height)
        return bool(red) and bool(green)

    @staticmethod
    def _detection(
        color: str,
        candidate: Tuple[float, Tuple[int, int], Tuple[int, int, int, int]],
        left: int,
        top: int,
    ) -> VisualDetection:
        score, center, box = candidate
        full_center = (center[0] + left, center[1] + top)
        full_box = (box[0] + left, box[1] + top, box[2] + left, box[3] + top)
        # Confidence is normalised by the sum of weights so a perfect lamp
        # scores 1.0 (previous score/4.0 could never exceed 0.6).
        weights = (
            TrafficLightConfig.vertical_weight
            + TrafficLightConfig.center_weight
            + TrafficLightConfig.area_weight
        )
        return VisualDetection(
            valid=True,
            kind="traffic_light",
            center=full_center,
            color=color,
            confidence=min(1.0, max(0.0, score / weights)),
            box=full_box,
        )


class TrafficLightTask:
    """Stateful light task; returns TaskUpdate once per frame, non-blocking.

    Status flow (rule: once RUNNING, stay RUNNING until COMPLETED/FAILED):
      idle -> holding_red (red confirmed)        -> RUNNING, zero motion
      holding_red -> confirm_green (green seen)  -> RUNNING, zero motion
      confirm_green (green confirmed)            -> COMPLETED (release)
      holding_red / confirm_green -> FAILED      -> after max_hold_seconds of
                                                    TOTAL stop time
      idle + green                               -> NOT_TRIGGERED (never
                                                    takes over while driving)
    """

    name = KIND

    def __init__(
        self,
        settings: Optional[TrafficLightConfig] = None,
        detector: Optional[TrafficLightDetector] = None,
    ) -> None:
        self.settings = settings or TrafficLightConfig()
        self.detector = detector or TrafficLightDetector(self.settings)
        self.reset()

    def reset(self) -> None:
        self._state = IDLE
        self._red_streak = 0
        self._green_streak = 0
        self._stopped_at = 0.0      # total-stop clock, never reset by flicker
        self._last_green_at = 0.0   # last frame that actually saw green
        self._queued_evidence = None
        self._active_evidence = None
        #: 这一轮已经排过照片的事件（"traffic_light:red" / "traffic_light:green"）。
        self._evidence_queued_kinds: set = set()
        self._evidence_outcome = None

    def _enter_hold(self, now: float) -> None:
        self._state = HOLDING_RED
        self._red_streak = 0
        self._green_streak = 0
        if not self._stopped_at:
            self._stopped_at = now

    def _back_to_idle(self) -> None:
        self._state = IDLE
        self._red_streak = 0
        self._green_streak = 0
        self._stopped_at = 0.0
        self._last_green_at = 0.0

    # ------------------------------------------------------------------
    # Evidence protocol (main.service_task_evidence polls these every frame)
    # ------------------------------------------------------------------

    @property
    def pending_evidence_request(self) -> Optional[EvidenceRequest]:
        """Inspect the queued request without consuming it."""
        return self._queued_evidence

    def take_evidence_request(self) -> Optional[EvidenceRequest]:
        """Transfer one request to the integration-owned evidence writer."""
        request = self._queued_evidence
        self._queued_evidence = None
        return request

    def acknowledge_evidence(self, request_id: str, saved: bool) -> bool:
        """Record the real writer result; retry a bounded number of times.

        The state machine never waits for this: whether the screenshot is
        saved or not, the car keeps holding at red and releases on green.
        """
        if (
            self._active_evidence is None
            or self._active_evidence.request_id != request_id
        ):
            return False
        if saved:
            self._evidence_outcome = True
            self._active_evidence = None
            return True
        if self._active_evidence.attempt >= self.settings.max_evidence_attempts:
            self._evidence_outcome = False
            self._active_evidence = None
            return True
        retry = replace(
            self._active_evidence,
            attempt=self._active_evidence.attempt + 1,
        )
        self._active_evidence = retry
        self._queued_evidence = retry
        self._evidence_outcome = None
        return True

    def _queue_evidence(
        self, frame: FramePacket, detection: VisualDetection, kind: str = "traffic_light:red"
    ) -> None:
        """Queue one screenshot per **event** (red stop / green release).

        老师后来明确："红*T 和绿*T 是两个独立计分事件，应分别保存照片"。所以这里按
        `kind` 分事件去重（`evidence.saved_events` 在证据层再兜一层）：
        红灯确认存一张、绿灯放行再存一张，彼此不覆盖。
        """
        if kind in self._evidence_queued_kinds:
            return
        if self._evidence_outcome is False:
            return
        if self._queued_evidence is not None or self._active_evidence is not None:
            return
        request = self._make_evidence_request(frame, detection, kind)
        self._evidence_queued_kinds.add(kind)
        self._active_evidence = request
        self._queued_evidence = request

    def _make_evidence_request(
        self, frame: FramePacket, detection: VisualDetection, kind: str = "traffic_light:red"
    ):
        """红灯/绿灯都用**圆圈**标出灯（老师要求"用圆圈标出红灯/绿灯"）。"""
        return make_evidence_photo(
            kind,
            frame,
            detection=detection,
            shape="circle",
            team=self.settings.team_number,
            label=kind.replace(":", "_"),
        )

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        detection = self.detector.detect(frame.image)
        s = self.settings
        color = detection.color if detection.valid else None

        if self._state == IDLE:
            if (
                getattr(s, "fork_light_competition", True)
                and not detection.valid
                and self.detector.sees_both_colours(frame.image)
            ):
                # 岔路口（红绿同框）：选道判据归 green_junction。
                # 本模块从 IDLE 状态**绝不接管**，否则它会只认颜色不认位置、
                # 在第 2 帧就把车按在岔路口（见 TrafficLightConfig 里的说明）。
                self._red_streak = 0
                self._green_streak = 0
                return TaskUpdate(
                    TaskStatus.NOT_TRIGGERED,
                    detection=detection,
                    message="red+green in one frame; a fork decides by light colour",
                )
            if color == RED:
                self._red_streak += 1
                if self._red_streak >= s.red_confirm_frames:
                    self._enter_hold(now)
                    self._queue_evidence(frame, detection)
                    return TaskUpdate(
                        TaskStatus.RUNNING,
                        motion=STOP_COMMAND,
                        detection=detection,
                        message="red confirmed; holding",
                    )
                return TaskUpdate(
                    TaskStatus.NOT_TRIGGERED,
                    detection=detection,
                    message="red seen; confirming",
                )
            # Green from idle never takes over: the car is already driving and
            # the light only authorises release of a stop (4.2, plan A).
            self._red_streak = 0
            self._green_streak = 0
            return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)

        # Stopped: total-stop clock runs from the first hold and is NOT reset
        # by green flicker, so a stuck loop always ends in FAILED (4.3).
        if now - self._stopped_at > s.max_hold_seconds:
            self._back_to_idle()
            return TaskUpdate(
                TaskStatus.FAILED,
                detection=detection,
                message="stop timeout; failed",
            )

        if self._state == HOLDING_RED:
            if color == RED:
                return TaskUpdate(
                    TaskStatus.RUNNING,
                    motion=STOP_COMMAND,
                    detection=detection,
                    message="holding at red",
                )
            if color == GREEN:
                self._state = CONFIRM_GREEN
                self._green_streak = 1
                self._last_green_at = now
                return TaskUpdate(
                    TaskStatus.RUNNING,
                    motion=STOP_COMMAND,
                    detection=detection,
                    message="green seen; confirming",
                )
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=STOP_COMMAND,
                detection=detection,
                message="holding (light lost)",
            )

        # CONFIRM_GREEN (only reachable from a stop)
        if color == GREEN:
            self._green_streak += 1
            self._last_green_at = now
            if self._green_streak >= s.green_confirm_frames:
                # 老师："红*T 和绿*T 是两个独立计分事件，应分别保存照片"。
                # 绿灯确认这一刻再存一张（红灯那张在确认停车时已经存了）。
                # 照片是**这一帧的画面**，不阻塞状态机：是否写盘失败都不影响放行。
                self._queue_evidence(frame, detection, "traffic_light:green")
                self._back_to_idle()
                return TaskUpdate(
                    TaskStatus.COMPLETED,
                    detection=detection,
                    message="green confirmed; release",
                )
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=STOP_COMMAND,
                detection=detection,
                message="confirming green",
            )
        if color == RED:
            # Red reappeared: back to holding; the total-stop clock keeps
            # running, so a red/green flicker loop still times out (4.3).
            self._state = HOLDING_RED
            self._red_streak = 0
            self._green_streak = 0
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=STOP_COMMAND,
                detection=detection,
                message="back to red; holding",
            )
        # No lamp: tolerate short gaps (dropped frames / flicker) inside the
        # grace window; beyond it, go back to holding and wait for red or the
        # total-stop timeout (4.1).
        if now - self._last_green_at <= s.green_gap_grace:
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=STOP_COMMAND,
                detection=detection,
                message="confirming green (gap)",
            )
        self._state = HOLDING_RED
        self._green_streak = 0
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=STOP_COMMAND,
            detection=detection,
            message="green lost; holding",
        )
