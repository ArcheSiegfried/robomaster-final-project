"""障碍检测与绕行（WP5 / Issue #5）。

状态：**已实现**（只用合成帧离线验证；未上真车，未见过真道具）。

职责边界（违反会被 tests/test_task_contract.py 直接拦下）：
  * 只接收主流程给的 FramePacket，绝不自己开相机或视频流。
  * 只返回 VisualDetection / TaskUpdate / MotionCommand，绝不直接调用 SDK，
    也不接触底盘的唯一运动出口。
  * step() 必须立刻返回：绕行动作被拆成"每次一小步"的状态机，
    每次只在一小块 ROI 里做一次颜色筛选，不做阻塞动作。

必须实现的接口（签名已冻结，不要改）：
    class ObstacleTask:
        name = "obstacle"
        def step(self, frame: FramePacket, now: float) -> TaskUpdate

返回约定：
  * 没有障碍   -> TaskUpdate(TaskStatus.NOT_TRIGGERED)
  * 绕行中     -> TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(...))
  * 绕行完成   -> TaskUpdate(TaskStatus.COMPLETED)
  * 失败/超时  -> TaskUpdate(TaskStatus.FAILED)
  一旦返回 RUNNING，就必须持续返回 RUNNING，直到 COMPLETED 或 FAILED。

交给骨架处理、本文件**不重复实现**的事：
  * 命令限幅、nan/inf 置零、20 秒硬超时、视频中断、人工 SPACE/R 打断；
  * 绕完之后"硬停车 -> 交回巡线 -> 清历史 -> 等一张新鲜有效线 -> 自动恢复"。
    所以这里绕完直接返回 COMPLETED，不自己判断"线回来了没有"。

我自己拍板的假设（不知道真场地规则，先按这组来；全部是下面一节的常量，随时能改）
  假设 1：障碍是一个立在路线上、挡住去路的东西，颜色明显区别于蓝色跑道和浅色地面。
          默认按"橙色系 + 暗色系"两段 HSV 区间找；**真道具什么颜色必须现场调参**。
  假设 2：赛道够宽，车往一侧平移大约半个车身就能绕过去。默认往**左**绕
          （MotionCommand 约定 lateral 正值向右，所以往左是负值）。
  假设 3：绕行用三段式够用——先侧移让开、再直行越过、再侧移回中线。
  假设 4：绕完不需要本模块判断线是否找回，交回流程由协调器负责（见上）。

单独自测：python scripts/check_module.py obstacle
"""

from typing import Optional

from models import (
    FramePacket,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)

try:  # 让本文件在没装 OpenCV 的机器上也能被 import
    import cv2
    import numpy as np
except Exception:  # pragma: no cover
    cv2 = None
    np = None

KIND = "obstacle"


# =====================================================================
# 1. 可调参数（假设全在这里；改这一节就够了）
# =====================================================================

# --- 绕行方向 ---
DODGE_SIDE = "left"          # "left" 往左绕；"right" 往右绕

# --- 速度（m/s），骨架还会再裁一次，这里先给更保守的值 ---
SIDE_SPEED = 0.18            # 横移让开的速度
FWD_SPEED = 0.16             # 越过障碍时的前进速度
BACK_SPEED = 0.14            # 绕完往回收的速度

# --- 每一段的最长时间（秒），到点就进入下一段，绝不无限做下去 ---
HOLD_BEFORE_GO = 0.20        # 决定绕之前，先原地停一下看清楚
T_OUT_TIME = 0.90            # 第 1 段：往侧面让开
T_PASS_TIME = 1.20           # 第 2 段：贴着障碍往前越过去
T_BACK_TIME = 0.90           # 第 3 段：往回收、回到线的附近

# --- 硬性保护 ---
MAX_TOTAL_TIME = 6.00        # 整段绕行最长 6 秒，超了立刻停车报 FAILED
CONFIRM_FRAMES = 3           # 连续 3 帧都看到障碍，才算"真有障碍"
REARM_SECONDS = 1.50         # 一次绕行结束后，这段时间内不再重新接管

# --- 障碍检测：ROI（画面比例 x1, y1, x2, y2），只看画面下半部分中间这条带 ---
OBSTACLE_ROI = (0.20, 0.45, 0.80, 0.95)

# 色块要多大才算障碍（比例都相对 ROI）
MIN_OBSTACLE_AREA = 0.03     # 面积至少占 ROI 的 3%，更小的当噪声
MIN_OBSTACLE_WIDTH = 0.10    # 宽度至少占 ROI 宽度的 10%
MAX_OBSTACLE_AREA = 0.90     # 占到 90% 以上的"色块"多半是整片同色背景
MIN_CONFIDENCE = 0.35        # 候选打分低于这个值就不认（归一化后的 0~1）

# 去噪用的形态学核（函数内部会自动取奇数）
OPEN_KERNEL = 3              # 开运算：去掉零散噪点
CLOSE_KERNEL = 5             # 闭运算：把障碍内部的小洞补上

# 多个候选一起出现时，谁更像"该绕的那一个"（打分权重）
SCORE_AREA_W = 1.0           # 大块优先
SCORE_LOWER_W = 0.8          # 位置越靠下（离车越近）优先
SCORE_CENTER_W = 0.6         # 越在正前方优先
SCORE_WIDTH_W = 0.6          # 越宽（越挡路）优先
SCORE_TOTAL_W = SCORE_AREA_W + SCORE_LOWER_W + SCORE_CENTER_W + SCORE_WIDTH_W

# --- 障碍颜色（HSV，OpenCV 的 H 是 0~179）---
# 默认取"橙色系"和"暗色系（偏黑）"两组。真道具换成什么颜色，改这里即可。
OBSTACLE_HSV_RANGES = (
    ((5, 90, 80), (25, 255, 255)),      # 橙色系
    ((0, 0, 0), (180, 255, 70)),        # 暗色系
)


def _clamp(value, low, high):
    return max(low, min(high, value))


def _odd_kernel(size):
    size = max(1, int(size))
    if size % 2 == 0:
        size += 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))


class ObstacleDetector:
    """只回答一个问题：这一帧里有没有障碍、在哪儿。

    写法借用竞速工程 blue_line_detector.py 的轮廓筛选框架，把"找蓝线"换成"找障碍色"：
        ROI -> 转 HSV -> 按颜色区间出掩膜 -> 开闭运算去噪 -> 找轮廓 -> 多个候选打分挑最好的
    全程只在自己那一小块 ROI 里算，不会拖着整幅画面跑。
    """

    def __init__(self) -> None:
        self.last_candidates = 0

    def detect(self, image) -> VisualDetection:
        """返回整幅图像坐标的 VisualDetection；没有障碍返回 no_result。"""
        self.last_candidates = 0
        if cv2 is None or np is None or image is None:
            return VisualDetection.no_result(KIND)

        height, width = image.shape[:2]
        if height <= 0 or width <= 0:
            return VisualDetection.no_result(KIND)

        left = max(0, min(int(width * OBSTACLE_ROI[0]), width - 1))
        top = max(0, min(int(height * OBSTACLE_ROI[1]), height - 1))
        right = max(left + 1, min(int(width * OBSTACLE_ROI[2]), width))
        bottom = max(top + 1, min(int(height * OBSTACLE_ROI[3]), height))
        roi = image[top:bottom, left:right]
        roi_height, roi_width = roi.shape[:2]
        roi_area = float(roi_height * roi_width)
        if roi_area <= 0:
            return VisualDetection.no_result(KIND)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = np.zeros((roi_height, roi_width), dtype=np.uint8)
        for lower, upper in OBSTACLE_HSV_RANGES:
            mask |= cv2.inRange(
                hsv,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _odd_kernel(OPEN_KERNEL))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _odd_kernel(CLOSE_KERNEL))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_score = -1.0
        for contour in contours:
            area_ratio = float(cv2.contourArea(contour)) / roi_area
            if area_ratio < MIN_OBSTACLE_AREA or area_ratio > MAX_OBSTACLE_AREA:
                continue
            box_left, box_top, box_width, box_height = cv2.boundingRect(contour)
            width_ratio = box_width / float(roi_width)
            if width_ratio < MIN_OBSTACLE_WIDTH:
                continue

            self.last_candidates += 1
            center_x = box_left + box_width / 2.0
            center_y = box_top + box_height / 2.0
            area_score = _clamp(area_ratio / 0.15, 0.0, 1.0)
            lower_score = _clamp(center_y / float(roi_height), 0.0, 1.0)
            center_score = 1.0 - _clamp(
                abs(center_x - roi_width / 2.0) / (roi_width / 2.0), 0.0, 1.0
            )
            width_score = _clamp(width_ratio / 0.50, 0.0, 1.0)
            score = (
                area_score * SCORE_AREA_W
                + lower_score * SCORE_LOWER_W
                + center_score * SCORE_CENTER_W
                + width_score * SCORE_WIDTH_W
            )
            if score > best_score:
                best_score = score
                best = (box_left, box_top, box_width, box_height, center_x, center_y)

        if best is None:
            return VisualDetection.no_result(KIND)

        confidence = _clamp(best_score / SCORE_TOTAL_W, 0.0, 1.0)
        if confidence < MIN_CONFIDENCE:
            return VisualDetection.no_result(KIND)

        box_left, box_top, box_width, box_height, center_x, center_y = best
        return VisualDetection(
            valid=True,
            kind=KIND,
            center=(int(round(left + center_x)), int(round(top + center_y))),
            confidence=confidence,
            box=(
                int(left + box_left),
                int(top + box_top),
                int(left + box_left + box_width),
                int(top + box_top + box_height),
            ),
        )


class ObstacleTask:
    """看到障碍就绕过去；绕行被拆成"每帧只走一小步"的状态机。

    阶段：IDLE -> HOLD（停下来看清） -> OUT（侧移让开） -> PASS（前进越过）
          -> BACK（侧移回中线） -> COMPLETED
    """

    name = "obstacle"

    def __init__(self, settings: Optional[object] = None) -> None:
        # settings 保留是为了和骨架的约定一致；本模块的阈值全部是文件顶部的常量。
        self.settings = settings
        self.detector = ObstacleDetector()
        self.last_detection = VisualDetection.no_result(KIND)
        self.started_at = None
        self.stage = "IDLE"
        self.segment_started_at = None
        self.hit_frames = 0
        self.finished_at = None

    # ---------------- 对外 ----------------

    def detect(self, image) -> VisualDetection:
        """给一帧图像，回答有没有障碍（整幅图像坐标）。"""
        return self.detector.detect(image)

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        """主循环每帧调用一次，必须立刻返回。"""
        detection = self.detect(frame.image)
        self.last_detection = detection

        if self.stage == "IDLE":
            return self._step_idle(detection, now)
        return self._step_active(now)

    # ---------------- 还没接管：判断要不要管 ----------------

    def _step_idle(self, detection: VisualDetection, now: float) -> TaskUpdate:
        if self.finished_at is not None and now - self.finished_at < REARM_SECONDS:
            # 刚绕完，别对着同一个（可能还在画面里的）障碍再来一次
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED, detection=detection, message="re-arm cooldown"
            )

        if not detection.valid:
            self.hit_frames = 0
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED, detection=detection, message="no obstacle"
            )

        self.hit_frames += 1
        if self.hit_frames < CONFIRM_FRAMES:
            # 连确认帧数都不够，还不能接管
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED,
                detection=detection,
                message="candidate %d/%d" % (self.hit_frames, CONFIRM_FRAMES),
            )

        self.stage = "HOLD"
        self.started_at = now
        self.segment_started_at = now
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(),
            detection=detection,
            message="obstacle confirmed; holding before the dodge",
        )

    # ---------------- 已经接管：一步一步绕 ----------------

    def _step_active(self, now: float) -> TaskUpdate:
        if now - self.started_at > MAX_TOTAL_TIME:
            return self._finish(
                TaskStatus.FAILED,
                "dodge exceeded %.2fs; stopping" % MAX_TOTAL_TIME,
                now,
            )

        elapsed = now - self.segment_started_at
        if self.stage == "HOLD" and elapsed >= HOLD_BEFORE_GO:
            self._enter("OUT", now)
        elif self.stage == "OUT" and elapsed >= T_OUT_TIME:
            self._enter("PASS", now)
        elif self.stage == "PASS" and elapsed >= T_PASS_TIME:
            self._enter("BACK", now)
        elif self.stage == "BACK" and elapsed >= T_BACK_TIME:
            return self._finish(TaskStatus.COMPLETED, "dodge finished; handing back", now)

        if self.stage == "HOLD":
            motion = MotionCommand()
            message = "holding before the dodge"
        elif self.stage == "OUT":
            motion = MotionCommand(lateral=self._side() * SIDE_SPEED)
            message = "stepping aside"
        elif self.stage == "PASS":
            motion = MotionCommand(forward=FWD_SPEED)
            message = "passing the obstacle"
        else:
            motion = MotionCommand(lateral=-self._side() * BACK_SPEED)
            message = "returning to the line"

        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=motion,
            detection=self.last_detection,
            message=message,
        )

    # ---------------- 小工具 ----------------

    def _enter(self, stage: str, now: float) -> None:
        self.stage = stage
        self.segment_started_at = now

    def _finish(self, status: TaskStatus, message: str, now: float) -> TaskUpdate:
        self.stage = "IDLE"
        self.hit_frames = 0
        self.started_at = None
        self.segment_started_at = None
        self.finished_at = now
        return TaskUpdate(
            status,
            motion=MotionCommand(),
            detection=self.last_detection,
            message=message,
        )

    def _side(self) -> float:
        return 1.0 if DODGE_SIDE == "right" else -1.0
