"""6 号任务模块（``green_junction.py``）的离线测试。

只用合成画面、假时钟和假巡线结果：
**不连相机、不连机器人、不跑 main.py、不发任何真实运动命令。**

运行：

    python -m unittest tests.test_green_junction -v
    python -m unittest discover -s tests -v
"""

import pathlib
import re
import sys
import unittest

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from green_junction import (  # noqa: E402
    LIGHT_OWNER,
    STOP,
    Branch,
    GreenJunctionTask,
    JunctionConfig,
    JunctionDetector,
    JunctionState,
    LightColor,
    LightReading,
    blue_branch_mask,
    detections_for_log,
    evaluate_branches,
    LampSpotter,
    LIGHT_OWNER,
    LIGHT_OWNER_LEGACY,
    line_is_centered,
    make_light_probe,
    make_two_lamp_probe,
    near_field_line,
    near_line_is_centered,
)
from examples.green_junction_demo import run_demo  # noqa: E402
from tests.task_harness import TaskHarness  # noqa: E402
from models import (  # noqa: E402
    FramePacket,
    LineDetection,
    MotionCommand,
    TaskStatus,
    VisualDetection,
)


# --------------------------------------------------------------------------
# 合成画面
# --------------------------------------------------------------------------

WIDTH = 640
HEIGHT = 360
GROUND = (200, 200, 200)
TAPE_BGR = (255, 0, 0)  # OpenCV 里 BGR=(255,0,0) 就是蓝色
TAPE_HALF = 12
SPLIT_ROW = 220

#: 假时钟每帧走多少秒（10 FPS，接近低分辨率视频流的实际帧率）。
FRAME_DT = 0.10


def blank_frame():
    """只有地面、没有蓝线：模块必须不接管。"""
    return np.full((HEIGHT, WIDTH, 3), GROUND, np.uint8)


def line_frame(x=320, top=180, bottom=350):
    """普通直道。"""
    image = blank_frame()
    cv2.line(image, (x, bottom), (x, top), TAPE_BGR, TAPE_HALF * 2)
    return image


def junction_frame(split_y=SPLIT_ROW, spread=0.30):
    """Y 字岔路：上方一条垂直带，``split_y`` 以下分成左右两条。"""
    image = blank_frame()
    cv2.line(image, (WIDTH // 2, split_y), (WIDTH // 2, split_y - 90), TAPE_BGR, TAPE_HALF * 2)
    dx = int(WIDTH * spread / 2.0)
    for sign in (-1, 1):
        cv2.line(
            image,
            (WIDTH // 2, split_y),
            (WIDTH // 2 + sign * dx, HEIGHT - 2),
            TAPE_BGR,
            TAPE_HALF * 2,
        )
    return image


def approach_frame(fork_y=290, far_y=120, reach=110):
    """实车那次报告里的朝向：分叉点在画面**下方**，两条分支向上张开，下面一条腿。

    和 :func:`junction_frame` 的唯一区别就是"张开的方向反过来"：老代码的
    ``_opens_upward`` 只认 junction_frame 那种朝向，实车恰好是这一种，
    于是把真岔路判成了 ``branches too short``（见 2026-09-15 实车测试报告）。
    """
    image = blank_frame()
    cv2.line(image, (WIDTH // 2, fork_y), (WIDTH // 2, HEIGHT - 2), TAPE_BGR, TAPE_HALF * 2)
    for sign in (-1, 1):
        cv2.line(
            image,
            (WIDTH // 2, fork_y),
            (WIDTH // 2 + sign * reach, far_y),
            TAPE_BGR,
            TAPE_HALF * 2,
        )
    return image


def green_lamp_frame(center=(500, 100), radius=22):
    """实车朝向的岔路 + 画面右上方一盏绿灯（给 3 号的检测器认）。"""
    image = approach_frame()
    cv2.circle(image, center, radius, (0, 255, 0), -1)  # BGR 纯绿 ≈ HSV H=60
    return image


def two_lamp_frame(green_on_left=True, y=100, radius=22):
    """**本关卡的真实场景**：岔路口两边各放一盏灯，一边红一边绿（灯立在路边）。

    和 :func:`green_lamp_frame` 的区别就是"同时有两盏"——这正是 3 号的
    ``TrafficLightDetector`` 会翻车的场景：它整帧只挑**一盏**得分最高的，
    而且 ``red_priority=True``，两盏同时可见时只会报红。
    """
    image = approach_frame()
    red_x, green_x = (500, 100) if green_on_left else (100, 500)
    cv2.circle(image, (red_x, y), radius, (0, 0, 255), -1)    # 红（BGR 纯红）
    cv2.circle(image, (green_x, y), radius, (0, 255, 0), -1)   # 绿
    return image


def fake_line(error=0.0, valid=True, confidence=0.9):
    """假的巡线结果，只有本模块用到的字段。"""
    mask = np.zeros((HEIGHT, WIDTH), np.uint8)
    return LineDetection(
        valid,
        error,
        0.0,
        confidence,
        (int(WIDTH / 2), HEIGHT - 10),
        (int(WIDTH / 2), 200),
        (0, 150, WIDTH, HEIGHT),
        mask,
    )


def packet(image, sequence, captured_at):
    return FramePacket(image=image, sequence=sequence, captured_at=captured_at)


def green(side=None, confidence=1.0):
    return LightReading(color=LightColor.GREEN, branch=side, confidence=confidence)


def green_without_side(frame=None, now=None):
    """"看到绿灯，但不知道是哪一边"。

    配合 ``JunctionConfig(fallback_rule="none")`` 用：模块会接管（因为这一帧真的
    有读数，见 A14），但永远选不出分支，于是停在 ``DECIDE`` 里等着。
    """
    return LightReading(color=LightColor.GREEN, branch=None, confidence=1.0)


# --------------------------------------------------------------------------
# 必过的两条：合规检查 + 不许误触发
# --------------------------------------------------------------------------


class ComplianceTests(unittest.TestCase):
    """第一条：我的文件里没有违规代码。"""

    def test_source_has_no_forbidden_code(self):
        text = (ROOT / "green_junction.py").read_text(encoding="utf-8")
        lowered = text.lower()
        for token in (
            "robomaster",
            "cv2.videocapture",
            "drive_wheels",
            "drive_speed",
            "chassis.",
            "ep_robot",
            "time.sleep(",
            "socket",
            "requests",
            "subprocess",
        ):
            self.assertNotIn(
                token,
                lowered,
                "green_junction.py 不允许出现 %r（红线 3、4、5）" % token,
            )
        self.assertIsNone(
            re.search(r"[A-Za-z]:[\\/]", text),
            "不允许出现个人绝对路径（红线 6）",
        )
        self.assertIn("from models import", text, "只使用公共接口 models.py")

    def test_light_rule_now_belongs_to_this_module(self):
        """灯的判据归属：2026-09-16 团队删掉了 ``traffic_light.py``（``6dc2c1f``）。

        理由：赛题里没有"独立的红绿灯停车"这一项，而它在实车上反复把红色物体
        判成红灯、原地锁停。但第一个岔路口两边各有一盏灯、车要往绿灯那边走，
        所以"绿灯在哪一边"这个判据由本模块提供（A17）；注入式探针仍然保留。
        """
        self.assertEqual(LIGHT_OWNER, "green_junction.py")
        self.assertEqual(LIGHT_OWNER_LEGACY, "traffic_light.py")


class NoTriggerTests(unittest.TestCase):
    """第二条：只给蓝线、没有岔路的画面时，不许接管。"""

    def test_straight_line_never_takes_over(self):
        task = GreenJunctionTask()
        now = 10.0
        for index in range(40):
            now += 0.05
            update = task.step(packet(line_frame(), index + 1, now), now, fake_line())
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
            self.assertFalse(task.active)
        self.assertEqual(task.state, JunctionState.IDLE)

    def test_blank_and_empty_frames_never_take_over(self):
        task = GreenJunctionTask()
        for index in range(30):
            now = 20.0 + index * 0.05
            for image in (blank_frame(), line_frame(x=120), line_frame(x=520)):
                update = task.step(packet(image, index + 1, now), now, fake_line())
                self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertEqual(task.state, JunctionState.IDLE)

    def test_off_centre_blob_is_not_a_junction(self):
        """画面一侧的一团蓝色不算岔路（A3：两段 + 间距 + 连续行）。"""
        image = blank_frame()
        cv2.rectangle(image, (60, 200), (200, 340), TAPE_BGR, -1)
        cv2.rectangle(image, (440, 200), (580, 340), TAPE_BGR, -1)
        detection = JunctionDetector(JunctionConfig()).detect(image)
        self.assertFalse(detection.valid)

    def test_no_junction_frame_yields_no_coordinates(self):
        detection = JunctionDetector(JunctionConfig()).detect(line_frame())
        self.assertFalse(detection.valid)
        self.assertIsNone(detection.split_row)
        self.assertIsNone(detection.split_center_x)
        self.assertEqual(detection.branches, ())


# --------------------------------------------------------------------------
# 检测
# --------------------------------------------------------------------------


class DetectorTests(unittest.TestCase):
    def test_y_junction_is_detected_with_two_branches(self):
        settings = JunctionConfig()
        detection = JunctionDetector(settings).detect(junction_frame())
        self.assertTrue(detection.valid, detection.message)
        self.assertIsNotNone(detection.split_row)
        self.assertIsNotNone(detection.box)
        self.assertEqual(len(detection.branches), 2)
        left = detection.branch(Branch.LEFT)
        right = detection.branch(Branch.RIGHT)
        self.assertIsNotNone(left)
        self.assertIsNotNone(right)
        self.assertLess(left.bearing_deg, 0.0)
        self.assertGreater(right.bearing_deg, 0.0)
        self.assertLess(left.center[0], right.center[0])
        left_px, top_px, right_px, bottom_px = detection.box
        self.assertTrue(top_px <= detection.split_row < bottom_px)
        self.assertTrue(left_px < detection.split_center_x < right_px)
        self.assertGreater(detection.confidence, settings.min_junction_confidence)

    def test_tight_split_is_rejected(self):
        """两条分支几乎贴在一起（就是一条粗线）不算岔路。"""
        detection = JunctionDetector(JunctionConfig()).detect(junction_frame(spread=0.03))
        self.assertFalse(detection.valid)

    def test_junction_below_the_band_is_rejected(self):
        """分叉点已经贴到画面底部（车已经开过去了）不算要处理的岔路。"""
        detection = JunctionDetector(JunctionConfig()).detect(junction_frame(split_y=340))
        self.assertFalse(detection.valid)

    def test_mask_is_full_frame_and_non_empty(self):
        image = junction_frame()
        mask = blue_branch_mask(image, JunctionConfig())
        self.assertEqual(mask.shape, image.shape[:2])
        self.assertGreater(int(np.count_nonzero(mask)), 0)

    def test_bad_frame_raises(self):
        with self.assertRaises(ValueError):
            JunctionDetector(JunctionConfig()).detect(np.zeros((10, 10), np.uint8))

    def test_log_summary_has_no_big_arrays(self):
        detection = JunctionDetector(JunctionConfig()).detect(junction_frame())
        summary = detections_for_log(detection)
        self.assertTrue(summary["valid"])
        self.assertEqual(len(summary["branches"]), 2)
        self.assertNotIn("mask", summary)
        # 实车排障要用的新字段：分叉带的下沿、行数、间距趋势、有没有"腿"。
        self.assertIsNotNone(summary["fork_bottom_row"])
        self.assertGreater(summary["band_rows"], 0)
        self.assertIn("trend_ratio", summary)
        self.assertIn("has_stem", summary)
        self.assertIn("band_bottom_ratio", summary)
        self.assertFalse(detections_for_log(None)["valid"])

    # -- 2026-09-15 实车测试报告里的三个问题 -----------------------------

    def test_fork_opening_toward_the_camera_is_detected(self):
        """分叉点在画面下方、两条分支向上张开（实车那次）也必须检出。

        老代码只认反方向（``_opens_upward``），实车正是在这里被判成
        ``branches too short``。
        """
        settings = JunctionConfig()
        detection = JunctionDetector(settings).detect(approach_frame())
        self.assertTrue(detection.valid, detection.message)
        self.assertEqual(len(detection.branches), 2)
        left = detection.branch(Branch.LEFT)
        right = detection.branch(Branch.RIGHT)
        self.assertLess(left.bearing_deg, 0.0)
        self.assertGreater(right.bearing_deg, 0.0)
        # 张开也好、收拢也好，趋势判据都认；两条平行色块才会被它挡掉。
        self.assertIsNotNone(detection.trend_ratio)
        self.assertGreaterEqual(detection.trend_ratio, settings.min_branch_opening_ratio)
        # 这种朝向里分叉带在画面中上部：不该被当成"车已经开过岔路口"（A12）。
        self.assertLess(detection.band_bottom_ratio, settings.drove_past_fork_row_ratio)

    def test_a_row_split_into_three_runs_still_counts_as_two_branches(self):
        """一条带子被噪点切成三段时，取最左最右两段（老代码要求"恰好两段"）。"""
        detector = JunctionDetector(JunctionConfig())
        row = np.zeros(200, np.uint8)
        row[10:20] = 255
        row[40:50] = 255      # 中间这段是噪点，不该把这条带子判成"没分叉"
        row[120:140] = 255
        pair = detector._two_runs(row, 200)
        self.assertIsNotNone(pair)
        self.assertEqual(pair[0], (10, 19))
        self.assertEqual(pair[1], (120, 139))

    def test_parallel_bands_without_a_stem_are_rejected(self):
        """两条平行色块：间距不变、两头都不接带子 → 不是岔路。"""
        image = blank_frame()
        cv2.rectangle(image, (60, 200), (200, 340), TAPE_BGR, -1)
        cv2.rectangle(image, (440, 200), (580, 340), TAPE_BGR, -1)
        detection = JunctionDetector(JunctionConfig()).detect(image)
        self.assertFalse(detection.valid)
        self.assertIn("parallel", detection.message)

    def test_trend_ratio_never_divides_by_zero(self):
        """P1：老的 ``_opens_upward`` 在有效行数 0/1/2 时 ``span=0`` 会除零。

        实车日志里出现过 34 条 ``float division by zero``。
        """
        detector = JunctionDetector(JunctionConfig())
        for count in (0, 1, 2, 3, 5):
            band = [(200 + index, ((0, 10), (100 + index, 110 + index))) for index in range(count)]
            self.assertIsNone(
                detector._band_trend_ratio(band),
                "行数不够时应该返回 None，而不是猜一个数",
            )
        # 行数够了就给出比值；9 行里间距从 20 涨到 40 → 2.0 左右。
        band = [(200 + index, ((0, 10), (20 + index * 5, 30 + index * 5))) for index in range(9)]
        ratio = detector._band_trend_ratio(band)
        self.assertIsNotNone(ratio)
        self.assertGreater(ratio, 1.0)

    def test_short_band_needs_a_stem(self):
        """行数太少时趋势不可信，这时必须靠"腿"：孤零零两条短色块不算岔路。"""
        image = blank_frame()
        cv2.rectangle(image, (250, 250), (290, 252), TAPE_BGR, -1)
        cv2.rectangle(image, (350, 250), (390, 252), TAPE_BGR, -1)
        detection = JunctionDetector(JunctionConfig()).detect(image)
        self.assertFalse(detection.valid)


class NearLineTests(unittest.TestCase):
    """A11：coordinator 只传 (frame, now)，"线回中央了没有"只能自己从画面算。"""

    def test_straight_line_is_centred(self):
        error, reason = near_field_line(line_frame())
        self.assertIsNotNone(error, reason)
        self.assertLess(abs(error), 0.05)
        self.assertTrue(near_line_is_centered(line_frame())[0])

    def test_off_centre_line_is_not_centred(self):
        centered, _ = near_line_is_centered(line_frame(x=120))
        self.assertFalse(centered)

    def test_blank_frame_has_no_line(self):
        error, reason = near_field_line(blank_frame())
        self.assertIsNone(error)
        self.assertIn("no tape", reason)
        self.assertFalse(near_line_is_centered(blank_frame())[0])

    def test_two_branches_in_the_near_band_are_not_centred(self):
        """岔路的两条分支伸到车头前 → 不是"一条居中的线"。"""
        error, reason = near_field_line(junction_frame())
        self.assertIsNone(error)
        self.assertIn("split", reason)
        self.assertFalse(near_line_is_centered(junction_frame())[0])

    def test_bad_image_is_not_centred(self):
        self.assertIsNone(near_field_line(None)[0])
        self.assertIsNone(near_field_line(np.zeros((10, 10), np.uint8))[0])


class LineCentredTests(unittest.TestCase):
    def test_centred_rules(self):
        self.assertFalse(line_is_centered(None))
        self.assertFalse(line_is_centered(fake_line(valid=False)))
        self.assertFalse(line_is_centered(fake_line(error=0.0, confidence=0.05)))
        self.assertFalse(line_is_centered(fake_line(error=0.9)))
        self.assertTrue(line_is_centered(fake_line(error=0.05)))


# --------------------------------------------------------------------------
# 判据
# --------------------------------------------------------------------------


class RuleTests(unittest.TestCase):
    def setUp(self):
        detection = JunctionDetector(JunctionConfig()).detect(junction_frame())
        self.assertTrue(detection.valid)
        self.branches = detection.branches
        self.settings = JunctionConfig()

    def test_green_on_a_branch_selects_it(self):
        chosen, reason = evaluate_branches(
            self.branches, green(Branch.RIGHT), self.settings
        )
        self.assertIs(chosen.side, Branch.RIGHT)
        chosen, _ = evaluate_branches(
            self.branches, green(Branch.LEFT), self.settings
        )
        self.assertIs(chosen.side, Branch.LEFT)

    def test_red_on_a_branch_selects_nothing(self):
        chosen, reason = evaluate_branches(
            self.branches,
            LightReading(color=LightColor.RED, branch=Branch.LEFT),
            self.settings,
        )
        self.assertIsNone(chosen)
        self.assertIn("red", reason)

    def test_unknown_light_never_takes_a_branch(self):
        chosen, reason = evaluate_branches(
            self.branches, LightReading(color=LightColor.UNKNOWN), self.settings
        )
        self.assertIsNone(chosen)
        self.assertIn("unknown", reason)

    def test_no_probe_and_no_fallback_rule_selects_nothing(self):
        chosen, reason = evaluate_branches(self.branches, None, self.settings)
        self.assertIsNone(chosen)
        self.assertIn("fallback_color", reason)

    def test_fallback_straightest_works_when_explicitly_enabled(self):
        settings = JunctionConfig(fallback_color="green", fallback_rule="straightest")
        chosen, reason = evaluate_branches(self.branches, None, settings)
        self.assertIsNotNone(chosen)
        self.assertIn("straightest", reason)

    def test_fallback_none_rule_selects_nothing(self):
        settings = JunctionConfig(fallback_color="green", fallback_rule="none")
        chosen, _ = evaluate_branches(self.branches, None, settings)
        self.assertIsNone(chosen)

    def test_probe_pointing_at_a_missing_branch_selects_nothing(self):
        one_branch = (self.branches[0],)
        chosen, reason = evaluate_branches(one_branch, green(Branch.RIGHT), self.settings)
        self.assertIsNone(chosen)
        self.assertIn("missing branch", reason)


# --------------------------------------------------------------------------
# 状态机
# --------------------------------------------------------------------------


class StateMachineTests(unittest.TestCase):
    def test_green_probe_drives_the_full_sequence(self):
        task = GreenJunctionTask(light_probe=lambda frame, now: green(Branch.RIGHT))
        statuses = []
        now = 100.0
        for index in range(30):
            now += FRAME_DT
            line = fake_line(error=0.0 if index >= 10 else 0.9)
            update = task.step(
                packet(junction_frame(), index + 1, now), now, line
            )
            statuses.append(update.status)
        self.assertEqual(task.state, JunctionState.COMPLETED)
        self.assertEqual(statuses[-1], TaskStatus.COMPLETED)
        self.assertEqual(update.detection.target_id, "right")
        self.assertEqual(update.motion, STOP)
        # 一旦接管过，就没有任何一帧改口成 NOT_TRIGGERED（红线 8）
        first_running = statuses.index(TaskStatus.RUNNING)
        self.assertNotIn(TaskStatus.NOT_TRIGGERED, statuses[first_running:])

    def test_turn_command_is_bounded_and_points_at_the_branch(self):
        task = GreenJunctionTask(light_probe=lambda frame, now: green(Branch.RIGHT))
        now = self._advance_until(task, JunctionState.TURN)
        update = task.step(packet(junction_frame(), 20, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.RUNNING)
        command = update.motion
        settings = task.settings
        self.assertGreater(command.yaw, 0.0, "右分支应该右转（yaw 为正）")
        self.assertLessEqual(abs(command.yaw), settings.max_turn_yaw)
        self.assertLessEqual(command.forward, 0.30)
        self.assertEqual(command.lateral, 0.0)

    def test_left_branch_turns_left(self):
        task = GreenJunctionTask(light_probe=lambda frame, now: green(Branch.LEFT))
        now = self._advance_until(task, JunctionState.TURN)
        update = task.step(packet(junction_frame(), 20, now), now, fake_line(error=0.9))
        self.assertLess(update.motion.yaw, 0.0, "左分支应该左转（yaw 为负）")

    def test_turn_yaw_is_limited_for_a_wide_branch(self):
        settings = JunctionConfig(yaw_gain=50.0, max_turn_yaw=30.0)
        task = GreenJunctionTask(
            settings=settings, light_probe=lambda frame, now: green(Branch.RIGHT)
        )
        now = self._advance_until(task, JunctionState.TURN)
        update = task.step(packet(junction_frame(), 20, now), now, fake_line(error=0.9))
        self.assertLessEqual(abs(update.motion.yaw), 30.0)
        self.assertGreater(abs(update.motion.yaw), 0.0)

    def test_waiting_for_a_rule_keeps_the_car_stopped(self):
        """还没拿到判据时只能保持停车，不能自己往前冲。"""
        settings = JunctionConfig(decision_timeout=5.0, fallback_rule="none")
        task = GreenJunctionTask(settings=settings, light_probe=green_without_side)
        now = self._advance_until(task, JunctionState.DECIDE)
        update = task.step(packet(junction_frame(), 20, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertEqual(update.motion, STOP)
        self.assertEqual(update.motion.forward, 0.0)
        self.assertIn("waiting for rule", update.message)

    def test_probe_error_does_not_crash_or_move(self):
        """别人的模块炸了：既不能让车乱走，也不能让本模块崩。"""
        settings = JunctionConfig(decision_timeout=5.0, fallback_rule="none")

        def broken_probe(frame, now):
            raise RuntimeError("别人的模块炸了")

        # 还没接管时探针就炸 → 这一帧没有可用读数 → 根本不接管（A14）。
        idle = GreenJunctionTask(settings=settings, light_probe=broken_probe)
        now = 400.0
        for index in range(6):
            now += FRAME_DT
            update = idle.step(packet(junction_frame(), index + 1, now), now, fake_line(error=0.9))
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
            self.assertEqual(update.motion, STOP)

        # 接管之后探针才炸（灯被挡住）→ 保持 RUNNING + 停车，不许乱走。
        state = {"broken": False}

        def sometimes_broken(frame, now):
            if state["broken"]:
                raise RuntimeError("灯被挡住了")
            return green_without_side()

        task = GreenJunctionTask(settings=settings, light_probe=sometimes_broken)
        now = self._advance_until(task, JunctionState.DECIDE)
        state["broken"] = True
        now += FRAME_DT
        update = task.step(packet(junction_frame(), 20, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertEqual(update.motion, STOP)

    def test_red_only_never_takes_over(self):
        """**绝不为红灯锁停**（A14/A17）：只有红灯时本模块不接管。

        团队删掉 ``traffic_light.py`` 的理由就是"它会为了红灯原地锁停、
        浪费跑圈时间"（``6dc2c1f``）。本模块只在**真的有绿灯证据**时才接管；
        只有红灯时岔路交给后面的模块，车不会停在这里。
        """
        settings = JunctionConfig(decision_timeout=2.0)
        task = GreenJunctionTask(
            settings=settings,
            light_probe=lambda frame, now: LightReading(color=LightColor.RED),
        )
        now = 50.0
        for index in range(20):
            now += FRAME_DT
            update = task.step(packet(junction_frame(), index + 1, now), now, fake_line(error=0.9))
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
            self.assertEqual(update.motion, STOP)
        self.assertEqual(task.state, JunctionState.IDLE)
        self.assertIn("no usable light reading", update.message)

    def test_no_rule_source_at_all_never_takes_over(self):
        """连内置检测器都关掉、又没有探针 → 同样不接管（消息也不一样）。"""
        settings = JunctionConfig(decision_timeout=0.4, builtin_lamp_detection=False)
        task = GreenJunctionTask(settings=settings)
        now = 300.0
        for index in range(20):
            now += FRAME_DT
            update = task.step(packet(junction_frame(), index + 1, now), now, fake_line(error=0.9))
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
            self.assertEqual(update.motion, STOP)
        self.assertEqual(task.state, JunctionState.IDLE)
        self.assertIn("no light rule source", update.message)

    def test_probe_without_a_reading_never_takes_over(self):
        """A14（v3 报告第 4.2 条）："接了探针"不等于"有判据"。

        探针这一帧没看到灯（None / UNKNOWN）时不许接管，否则它会在**任何**岔路
        都先抢下来，白等 decision_timeout 再失败交回。
        """
        for probe in (
            lambda frame, now: None,
            lambda frame, now: LightReading(color=LightColor.UNKNOWN),
        ):
            task = GreenJunctionTask(
                settings=JunctionConfig(decision_timeout=0.4), light_probe=probe
            )
            now = 600.0
            for index in range(20):
                now += FRAME_DT
                update = task.step(
                    packet(junction_frame(), index + 1, now), now, fake_line(error=0.9)
                )
                self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
                self.assertEqual(update.motion, STOP)
            self.assertEqual(task.state, JunctionState.IDLE)
            self.assertIn("no usable light reading", update.message)

    def test_green_then_red_still_stops_safely(self):
        """已经接管之后灯变红 → 原地停 + 超时 FAILED（安全方向，不许硬闯）。"""
        settings = JunctionConfig(decision_timeout=0.4, fallback_rule="none")
        state = {"red": False}

        def probe(frame, now):
            if state["red"]:
                return LightReading(color=LightColor.RED)
            return green_without_side()

        task = GreenJunctionTask(settings=settings, light_probe=probe)
        now = self._advance_until(task, JunctionState.DECIDE)
        state["red"] = True
        now += settings.decision_timeout + 0.5
        update = task.step(packet(junction_frame(), 7, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertIn("red", update.message)
        self.assertEqual(update.motion, STOP)

    def test_the_fail_safe_path_is_still_available_on_request(self):
        """想要老行为（接管 → 停住 → 超时 FAILED）时，一个参数切回去。"""
        settings = JunctionConfig(decision_timeout=0.4, require_rule_source=False)
        task = GreenJunctionTask(settings=settings)
        now = self._advance_until(task, JunctionState.DECIDE)
        now += settings.decision_timeout + 0.5
        update = task.step(packet(junction_frame(), 7, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertIn("fallback_color", update.message)
        self.assertEqual(update.motion, STOP)

    def test_explicit_fallback_color_green_can_complete_without_a_probe(self):
        settings = JunctionConfig(fallback_color="green", fallback_rule="straightest")
        task = GreenJunctionTask(settings=settings)
        statuses = []
        now = 500.0
        for index in range(30):
            now += FRAME_DT
            line = fake_line(error=0.0 if index >= 10 else 0.9)
            update = task.step(packet(junction_frame(), index + 1, now), now, line)
            statuses.append(update.status)
        self.assertEqual(task.state, JunctionState.COMPLETED)
        self.assertIn(TaskStatus.RUNNING, statuses)

    def test_turn_timeout_fails_and_stops(self):
        settings = JunctionConfig(turn_min_duration=10.0, turn_timeout=0.4)
        task = GreenJunctionTask(
            settings=settings, light_probe=lambda frame, now: green(Branch.RIGHT)
        )
        now = self._advance_until(task, JunctionState.TURN)
        now += 1.0
        update = task.step(packet(junction_frame(), 8, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion, STOP)
        self.assertIn("turn_timeout", update.message)

    def test_settle_timeout_fails_and_stops(self):
        settings = JunctionConfig(settle_timeout=0.4)
        task = GreenJunctionTask(
            settings=settings, light_probe=lambda frame, now: green(Branch.RIGHT)
        )
        # 转向结束、线也回到中央 → 进入 SETTLE；随后线又丢了 → 超时失败。
        now = self._advance_until(task, JunctionState.SETTLE, line=fake_line(error=0.0))
        now += 1.0
        update = task.step(packet(junction_frame(), 9, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion, STOP)
        self.assertIn("did not return", update.message)

    def test_losing_the_junction_before_a_rule_fails_and_stops(self):
        settings = JunctionConfig(decision_timeout=0.5, fallback_rule="none")
        task = GreenJunctionTask(settings=settings, light_probe=green_without_side)
        now = self._advance_until(task, JunctionState.DECIDE)
        now += 1.0
        update = task.step(packet(blank_frame(), 5, now), now, fake_line())
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion, STOP)

    def test_driving_past_the_junction_without_a_rule_fails(self):
        """线已经回到中央、岔路形态还在，连续几帧都这样 → 车其实开过了 → 失败停车。"""
        settings = JunctionConfig(decision_timeout=5.0, fallback_rule="none")
        task = GreenJunctionTask(settings=settings, light_probe=green_without_side)
        now = self._advance_until(task, JunctionState.DECIDE)
        for index in range(settings.drove_past_frames):
            now += FRAME_DT
            update = task.step(packet(junction_frame(), 30 + index, now), now, fake_line(error=0.02))
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion, STOP)
        self.assertIn("past the junction", update.message)

    def test_normal_approach_is_not_mistaken_for_driving_past(self):
        """A12：正常进近时车头前的带子也是单条居中，不能因此判"开过了"。

        这里连 ``line`` 都不传——就是协调器的真实调用方式。
        """
        settings = JunctionConfig(decision_timeout=5.0, confirm_frames=2, fallback_rule="none")
        task = GreenJunctionTask(settings=settings, light_probe=green_without_side)
        now = 50.0
        statuses = []
        for index in range(12):
            now += FRAME_DT
            update = task.step(packet(approach_frame(), index + 1, now), now)
            statuses.append(update.status)
        self.assertIn(TaskStatus.RUNNING, statuses)
        self.assertNotIn(TaskStatus.FAILED, statuses)
        self.assertNotIn("drove past", task.last_message)

    def test_completes_without_the_line_argument(self):
        """P0 接口修复：协调器只调 ``step(frame, now)`` 也必须能走完整条流程。

        老代码的 TURN→SETTLE→COMPLETED 两道门全靠 ``line``，实车上恒为 False，
        只能超时失败。
        """
        task = GreenJunctionTask(light_probe=lambda frame, now: green(Branch.RIGHT))
        statuses = []
        now = 200.0
        for index in range(30):
            now += FRAME_DT
            update = task.step(packet(approach_frame(), index + 1, now), now)  # 只有两个参数
            statuses.append(update.status)
        self.assertIn(TaskStatus.RUNNING, statuses)
        self.assertEqual(task.state, JunctionState.COMPLETED)
        self.assertEqual(update.status, TaskStatus.COMPLETED)
        self.assertEqual(update.motion, STOP)
        self.assertEqual(update.detection.target_id, "right")

    def test_single_contradictory_frame_is_not_enough_to_fail(self):
        """单帧巧合不算：判定"开过了"要连续几帧都矛盾。"""
        settings = JunctionConfig(decision_timeout=5.0, fallback_rule="none")
        task = GreenJunctionTask(settings=settings, light_probe=green_without_side)
        now = self._advance_until(task, JunctionState.DECIDE)
        now += FRAME_DT
        update = task.step(packet(junction_frame(), 40, now), now, fake_line(error=0.02))
        self.assertEqual(update.status, TaskStatus.RUNNING)
        now += FRAME_DT
        update = task.step(packet(junction_frame(), 41, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.RUNNING)

    def test_losing_the_junction_while_turning_fails_after_the_grace(self):
        settings = JunctionConfig(junction_gap_grace=0.2, turn_min_duration=10.0)
        task = GreenJunctionTask(
            settings=settings, light_probe=lambda frame, now: green(Branch.RIGHT)
        )
        now = self._advance_until(task, JunctionState.TURN)
        now += 1.0
        update = task.step(packet(blank_frame(), 31, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion, STOP)
        self.assertIn("lost the junction", update.message)

    def test_short_junction_flicker_does_not_take_over(self):
        """只闪一帧的岔路形态不算岔路（连续确认）。"""
        task = GreenJunctionTask(
            settings=JunctionConfig(confirm_frames=3, confirm_gap_grace=0.05),
            light_probe=lambda frame, now: green(Branch.RIGHT),
        )
        now = 800.0
        updates = []
        for index in range(12):
            now += 0.05
            image = junction_frame() if index == 2 else line_frame()
            updates.append(task.step(packet(image, index + 1, now), now, fake_line()))
        self.assertEqual(task.state, JunctionState.IDLE)
        for update in updates:
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)

    def test_reset_clears_everything(self):
        task = GreenJunctionTask(light_probe=lambda frame, now: green(Branch.RIGHT))
        self._advance_until(task, JunctionState.TURN)
        self.assertTrue(task.active)
        task.reset()
        self.assertEqual(task.state, JunctionState.IDLE)
        self.assertFalse(task.active)
        self.assertIsNone(task.chosen_branch)
        self.assertIsNone(task.last_detection)
        update = task.step(packet(line_frame(), 1, 900.0), 900.0, fake_line())
        self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)

    def test_rearm_cooldown_blocks_an_immediate_retrigger(self):
        """刚走过一个岔路后，同一个岔路形状不会立刻被处理第二遍（A9）。"""
        settings = JunctionConfig(fallback_color="green", rearm_cooldown=2.0)
        task = GreenJunctionTask(settings=settings)
        now = 700.0
        for index in range(30):
            now += FRAME_DT
            line = fake_line(error=0.0 if index >= 10 else 0.9)
            update = task.step(packet(junction_frame(), index + 1, now), now, line)
        self.assertEqual(update.status, TaskStatus.COMPLETED)

        # coordinator 交出控制权后会复位模块；冷却期内不许重复触发。
        task.reset()
        task._rearm_ready_at = now + settings.rearm_cooldown
        for index in range(3):
            now += FRAME_DT
            blocked = task.step(packet(junction_frame(), 100 + index, now), now, fake_line())
            self.assertEqual(blocked.status, TaskStatus.NOT_TRIGGERED)
            self.assertIn("cooldown", blocked.message)

        # 冷却时间过去以后，新的岔路可以正常触发。
        now += settings.rearm_cooldown + 0.1
        for index in range(settings.confirm_frames):
            now += FRAME_DT
            armed = task.step(packet(junction_frame(), 200 + index, now), now, fake_line(error=0.9))
        self.assertTrue(task.active)
        self.assertEqual(armed.status, TaskStatus.RUNNING)

    def test_finished_state_keeps_reporting_the_terminal_status(self):
        """完成后重复调用必须继续报 COMPLETED，不能改口。"""
        task = GreenJunctionTask(settings=JunctionConfig(fallback_color="green"))
        now = 950.0
        update = None
        for index in range(30):
            now += FRAME_DT
            line = fake_line(error=0.0 if index >= 10 else 0.9)
            update = task.step(packet(junction_frame(), index + 1, now), now, line)
        self.assertEqual(update.status, TaskStatus.COMPLETED)
        for index in range(5):
            now += FRAME_DT
            again = task.step(packet(junction_frame(), 99, now), now, fake_line(error=0.9))
            self.assertEqual(again.status, TaskStatus.COMPLETED)
            self.assertEqual(again.motion, STOP)

    def test_missing_frame_image_while_owning_control_fails(self):
        task = GreenJunctionTask(light_probe=lambda frame, now: green(Branch.RIGHT))
        now = self._advance_until(task, JunctionState.TURN)
        now += 0.05
        update = task.step(FramePacket(image=None, sequence=2, captured_at=now), now)
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion, STOP)

    def test_chosen_bearing_is_exposed_for_logging(self):
        task = GreenJunctionTask(light_probe=lambda frame, now: green(Branch.RIGHT))
        self._advance_until(task, JunctionState.TURN)
        self.assertIsNotNone(task.chosen_bearing_deg)
        self.assertGreater(task.chosen_bearing_deg, 0.0)

    # -- helpers ---------------------------------------------------------

    @staticmethod
    def _advance_until(task, state, limit=40, line=None):
        """用假时钟把状态机推到指定状态，返回当时的时刻（秒）。"""
        now = 50.0
        for index in range(limit):
            now += FRAME_DT
            task.step(
                packet(junction_frame(), index + 1, now),
                now,
                fake_line(error=0.9) if line is None else line,
            )
            if task.state is state:
                return now
        raise AssertionError("module never reached %s" % state.value)


class LightProbeAdapterTests(unittest.TestCase):
    """v3 报告第三节那条断掉的链：3 号没有 ``reading()``，本模块给适配器（A13）。

    探针接口是 ``light_probe(frame, now) -> LightReading``；适配器负责把 3 号
    **现有**的两样东西（``TrafficLightDetector.detect`` / ``TrafficLightTask.step``）
    转成它，所以整合层不用等公共接口变更。
    """

    @staticmethod
    def _packet(image=None):
        return packet(blank_frame() if image is None else image, 1, 0.0)

    def test_detector_like_source(self):
        class Detector:
            def detect(self, image):
                return VisualDetection(
                    valid=True,
                    kind="traffic_light",
                    center=(500, 100),
                    color="green",
                    confidence=0.5,
                )

        probe = make_light_probe(Detector())
        reading = probe(self._packet(), 0.0)
        self.assertIsNotNone(reading)
        self.assertIs(reading.color, LightColor.GREEN)
        self.assertIs(reading.branch, Branch.RIGHT, "灯在画面右半边 → 右分支（A13）")
        self.assertAlmostEqual(reading.confidence, 0.5)

    def test_task_like_source(self):
        class Task:
            def step(self, frame, now):
                return type(
                    "Update",
                    (),
                    {
                        "detection": VisualDetection(
                            valid=True, kind="traffic_light", center=(100, 80), color="red"
                        )
                    },
                )()

        reading = make_light_probe(Task())(self._packet(), 0.0)
        self.assertIs(reading.color, LightColor.RED)
        self.assertIs(reading.branch, Branch.LEFT)

    def test_callable_readings_pass_through(self):
        probe = make_light_probe(lambda frame, now: green(Branch.LEFT))
        reading = probe(self._packet(), 0.0)
        self.assertIs(reading.color, LightColor.GREEN)
        self.assertIs(reading.branch, Branch.LEFT)

    def test_callable_that_only_takes_an_image(self):
        """有人直接把绑定方法 ``detector.detect`` 递进来时也不能静默失效。"""
        seen = {}

        def reader(image):
            seen["shape"] = image.shape
            return "green"

        probe = make_light_probe(reader)
        reading = probe(self._packet(), 0.0)
        self.assertEqual(seen["shape"], (HEIGHT, WIDTH, 3))
        self.assertIs(reading.color, LightColor.GREEN)
        self.assertIsNone(reading.branch, "没有坐标就没有左右信息")

    def test_branch_inference_can_be_disabled(self):
        class Detector:
            def detect(self, image):
                return VisualDetection(valid=True, kind="traffic_light", center=(500, 100), color="green")

        reading = make_light_probe(Detector(), infer_branch_from_position=False)(
            self._packet(), 0.0
        )
        self.assertIsNone(reading.branch)

    def test_unreadable_or_broken_sources_give_no_reading(self):
        # 完全读不出东西 → None（既不知道颜色，也不知道位置）。
        cases = (
            lambda frame, now: None,
            lambda frame, now: VisualDetection.no_result("traffic_light"),
            lambda frame, now: object(),
        )
        for reader in cases:
            self.assertIsNone(make_light_probe(reader)(self._packet(), 0.0))

        # 颜色看不懂 → 明确给 UNKNOWN，而不是 None：调用方两种情况都当"没有判据"，
        # 但 UNKNOWN 能进日志，排障时看得出"探针说话了，只是没听懂"。
        reading = make_light_probe(lambda frame, now: "purple")(self._packet(), 0.0)
        self.assertIsNotNone(reading)
        self.assertIs(reading.color, LightColor.UNKNOWN)

        def broken(frame, now):
            raise RuntimeError("3 号炸了")

        self.assertIsNone(make_light_probe(broken)(self._packet(), 0.0))

    def test_min_confidence_filter(self):
        class Detector:
            def detect(self, image):
                return VisualDetection(
                    valid=True, kind="traffic_light", center=(500, 100), color="green", confidence=0.10
                )

        self.assertIsNotNone(make_light_probe(Detector(), min_confidence=0.05)(self._packet(), 0.0))
        self.assertIsNone(make_light_probe(Detector(), min_confidence=0.50)(self._packet(), 0.0))

    def test_non_callable_source_raises(self):
        with self.assertRaises(TypeError):
            make_light_probe(42)

    def test_builtin_spotter_drives_the_whole_module(self):
        """用**内置**检测器（本模块自己的 LampSpotter）把整条链跑通。

        ``traffic_light.py`` 已在 ``6dc2c1f`` 被删除，所以这条链现在不依赖任何
        外部模块：绿灯 → 岔路 → 接管 → 转向 → 交回巡线。
        """
        spotter = LampSpotter()
        readings = spotter.readings(green_lamp_frame())
        self.assertTrue(readings, "内置检测器应该认出这盏绿灯")
        self.assertIs(readings[0].color, LightColor.GREEN)

        # 注入形态也支持（把 spotter 当别人给的检测器用）。
        task = GreenJunctionTask(light_probe=make_light_probe(spotter))
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        now = 1.05
        for _ in range(60):
            now += 0.05
            decision = harness.feed_image(now, green_lamp_frame())
            if task.finished and decision.owner == "line":
                break
        self.assertTrue(
            any(row.task_name == "green_junction" for row in harness.traces),
            "绿灯亮着时模块应该接管",
        )
        self.assertEqual(task.chosen_branch, Branch.RIGHT)
        self.assertEqual(task.state, JunctionState.COMPLETED)
        self.assertEqual(harness.owner, "line")


class TwoLampTests(unittest.TestCase):
    """A15/A16：岔路口两边各一盏灯（一边红一边绿，灯立在路边），往绿灯那边走。"""

    def setUp(self):
        detection = JunctionDetector(JunctionConfig()).detect(approach_frame())
        self.assertTrue(detection.valid, detection.message)
        self.branches = detection.branches
        self.settings = JunctionConfig()

    # -- 判据：绿灯在哪边就走哪边 ------------------------------------------

    def test_green_on_the_left_beats_red_on_the_right(self):
        chosen, reason = evaluate_branches(
            self.branches,
            [
                LightReading(color=LightColor.RED, branch=Branch.RIGHT),
                LightReading(color=LightColor.GREEN, branch=Branch.LEFT),
            ],
            self.settings,
        )
        self.assertIsNotNone(chosen)
        self.assertIs(chosen.side, Branch.LEFT)
        self.assertIn("green light on the left branch", reason)

    def test_green_on_the_right_beats_red_on_the_left(self):
        chosen, reason = evaluate_branches(
            self.branches,
            [
                LightReading(color=LightColor.GREEN, branch=Branch.RIGHT),
                LightReading(color=LightColor.RED, branch=Branch.LEFT),
            ],
            self.settings,
        )
        self.assertIsNotNone(chosen)
        self.assertIs(chosen.side, Branch.RIGHT)
        self.assertIn("green light on the right branch", reason)

    def test_both_green_falls_back_to_the_rule(self):
        chosen, reason = evaluate_branches(
            self.branches,
            [
                LightReading(color=LightColor.GREEN, branch=Branch.LEFT),
                LightReading(color=LightColor.GREEN, branch=Branch.RIGHT),
            ],
            JunctionConfig(fallback_rule="left"),
        )
        self.assertIsNotNone(chosen)
        self.assertIs(chosen.side, Branch.LEFT)
        self.assertIn("fallback_rule", reason)

    def test_two_reds_never_move(self):
        chosen, reason = evaluate_branches(
            self.branches,
            [
                LightReading(color=LightColor.RED, branch=Branch.LEFT),
                LightReading(color=LightColor.RED, branch=Branch.RIGHT),
            ],
            self.settings,
        )
        self.assertIsNone(chosen)
        self.assertIn("red", reason)

    def test_one_red_only_is_still_red(self):
        chosen, reason = evaluate_branches(
            self.branches,
            [LightReading(color=LightColor.RED, branch=Branch.LEFT)],
            self.settings,
        )
        self.assertIsNone(chosen)
        self.assertIn("left branch", reason)

    def test_green_without_a_side_plus_a_red_side_takes_the_other_side(self):
        """A16：只知道"看到绿灯"，但知道左边是红灯 → 走右边。"""
        chosen, reason = evaluate_branches(
            self.branches,
            [
                LightReading(color=LightColor.GREEN, branch=None),
                LightReading(color=LightColor.RED, branch=Branch.LEFT),
            ],
            self.settings,
        )
        self.assertIsNotNone(chosen)
        self.assertIs(chosen.side, Branch.RIGHT)
        self.assertIn("red", reason)

    def test_green_without_a_side_alone_uses_the_fallback_rule(self):
        chosen, _ = evaluate_branches(
            self.branches,
            [LightReading(color=LightColor.GREEN, branch=None)],
            JunctionConfig(fallback_rule="left"),
        )
        self.assertIsNotNone(chosen)
        self.assertIs(chosen.side, Branch.LEFT)

    # -- 适配器：把"整帧一盏"变成"左右各一盏" ----------------------------

    def test_adapter_splits_the_frame_into_left_and_right(self):
        class CropDetector:
            """按调用顺序返回：第一刀（左半边）红、第二刀（右半边）绿。

            注意 ``center`` 要按**传进来那张图**的坐标给（真检测器就是这样）：
            适配器用灯的整幅图坐标归边（见 ``overlap`` 那段注释）。
            """

            def __init__(self):
                self.calls = 0

            def detect(self, image):
                self.calls += 1
                return VisualDetection(
                    valid=True,
                    kind="traffic_light",
                    center=(image.shape[1] // 2, image.shape[0] // 2),
                    color="red" if self.calls == 1 else "green",
                )

        probe = make_two_lamp_probe(CropDetector())
        readings = probe(packet(two_lamp_frame(), 1, 0.0), 0.0)
        self.assertEqual(len(readings), 2)
        self.assertIs(readings[0].color, LightColor.RED)
        self.assertIs(readings[0].branch, Branch.LEFT)
        self.assertIs(readings[1].color, LightColor.GREEN)
        self.assertIs(readings[1].branch, Branch.RIGHT)

    def test_adapter_keeps_the_other_half_when_one_half_fails(self):
        class FlakyDetector:
            def __init__(self):
                self.calls = 0

            def detect(self, image):
                self.calls += 1
                if self.calls == 1:
                    return VisualDetection(
                        valid=True,
                        kind="traffic_light",
                        center=(image.shape[1] // 2, image.shape[0] // 2),
                        color="green",
                    )
                raise RuntimeError("右半边炸了")

        readings = make_two_lamp_probe(FlakyDetector())(packet(two_lamp_frame(), 1, 0.0), 0.0)
        self.assertEqual(len(readings), 1)
        self.assertIs(readings[0].color, LightColor.GREEN)
        self.assertIs(readings[0].branch, Branch.LEFT)

    def test_builtin_spotter_handles_a_lamp_on_the_split_line(self):
        """内置检测器扫整幅图，所以骑在画面中心线上的灯天然不会丢。"""
        image = approach_frame()
        cv2.circle(image, (340, 120), 40, (0, 255, 0), -1)
        readings = LampSpotter().readings(image)
        self.assertEqual(len(readings), 1, readings)
        self.assertIs(readings[0].color, LightColor.GREEN)
        self.assertIs(readings[0].branch, Branch.RIGHT, "灯中心在 320 右边 → 右分支")

    def test_adapter_places_a_straddling_lamp_by_its_centre(self):
        """给"只回报一盏、但带 center"的外部检测器用：骑线也要归到正确的一边。

        硬按 0.5 裁、不留重叠的话，这盏灯在半幅图里会被裁掉一半、形状判据就认不出来了。
        """
        class CentreDetector:
            """像真检测器那样：在传进来的图里找绿灯，回报它**在本图里**的 center。"""

            def detect(self, image):
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(
                    hsv,
                    np.array((40, 90, 60), np.uint8),
                    np.array((85, 255, 255), np.uint8),
                )
                ys, xs = np.nonzero(mask)
                if len(xs) == 0:
                    return VisualDetection.no_result("traffic_light")
                return VisualDetection(
                    valid=True,
                    kind="traffic_light",
                    center=(float(np.median(xs)), float(np.median(ys))),
                    color="green",
                    confidence=0.6,
                )

        image = approach_frame()
        cv2.circle(image, (340, 120), 40, (0, 255, 0), -1)  # 骑在 x=320 上
        probe = make_two_lamp_probe(CentreDetector(), min_confidence=0.40)
        readings = probe(packet(image, 1, 0.0), 0.0)
        self.assertEqual(len(readings), 1, "同一盏灯在重叠区被看到两次，要去重成一条")
        self.assertIs(readings[0].color, LightColor.GREEN)
        self.assertIs(readings[0].branch, Branch.RIGHT, "灯心 340 在切分线右边 → 右分支")

    def test_adapter_with_no_lamps_returns_nothing(self):
        readings = make_two_lamp_probe(lambda image: None)(packet(approach_frame(), 1, 0.0), 0.0)
        self.assertEqual(readings, [])

    # -- 内置检测器：一次找出两盏 ------------------------------------------

    def test_builtin_spotter_sees_both_lamps_at_once(self):
        """内置检测器一次就把两盏都找出来（不像被删掉的那个模块只报一盏、红优先）。"""
        for green_on_left in (True, False):
            readings = LampSpotter().readings(two_lamp_frame(green_on_left=green_on_left))
            self.assertEqual(len(readings), 2, readings)
            colours = {item.branch.value: item.color.value for item in readings}
            self.assertEqual(
                colours,
                {"left": "green", "right": "red"} if green_on_left
                else {"left": "red", "right": "green"},
            )

    def test_no_probe_at_all_drives_the_module_to_the_green_side(self):
        """**实车配置**：不注入任何探针，靠内置检测器；绿灯在哪边就走哪边。"""
        for green_on_left, expected in ((True, Branch.LEFT), (False, Branch.RIGHT)):
            task = GreenJunctionTask()  # 没有 light_probe
            harness = TaskHarness(task=task)
            harness.start_line(now=1.0)
            now = 1.05
            for _ in range(60):
                now += 0.05
                decision = harness.feed_image(now, two_lamp_frame(green_on_left=green_on_left))
                if task.finished and decision.owner == "line":
                    break
            self.assertTrue(
                any(row.task_name == "green_junction" for row in harness.traces),
                "绿灯亮着时模块应该接管",
            )
            self.assertEqual(
                task.chosen_branch,
                expected,
                "绿灯在%s边却走了 %s" % ("左" if green_on_left else "右", task.chosen_branch),
            )
            self.assertEqual(task.state, JunctionState.COMPLETED)
            self.assertEqual(harness.owner, "line")

    def test_two_lamp_readings_are_exposed_for_logging(self):
        task = GreenJunctionTask()
        now = 10.0
        for index in range(4):
            now += FRAME_DT
            task.step(packet(two_lamp_frame(green_on_left=False), index + 1, now), now)
        self.assertEqual(len(task.last_readings), 2)
        sides = sorted(item.branch.value for item in task.last_readings)
        self.assertEqual(sides, ["left", "right"])
        greens = [item for item in task.last_readings if item.color is LightColor.GREEN]
        self.assertEqual(len(greens), 1)
        self.assertEqual(greens[0].branch.value, "right")


class BuiltinLampTests(unittest.TestCase):
    """A17：内置认灯器（算法来自 3 号那版实车上调过的 vision_tasks.py）。"""

    def setUp(self):
        self.spotter = LampSpotter()

    def _frame_with(self, draw):
        image = approach_frame()
        draw(image)
        return image

    def test_round_green_lamp_is_found_on_the_left(self):
        image = self._frame_with(lambda img: cv2.circle(img, (100, 100), 22, (0, 255, 0), -1))
        readings = self.spotter.readings(image)
        self.assertEqual(len(readings), 1)
        self.assertIs(readings[0].color, LightColor.GREEN)
        self.assertIs(readings[0].branch, Branch.LEFT)

    def test_elongated_red_bar_is_not_a_lamp(self):
        """长条状红/绿物不是灯（圆度判据）——被删掉那个模块吃过这个亏。"""
        image = self._frame_with(
            lambda img: cv2.rectangle(img, (60, 60), (300, 90), (0, 0, 255), -1)
        )
        self.assertEqual(self.spotter.readings(image), [])

    def test_tiny_speck_is_not_a_lamp(self):
        """半径太小（< lamp_min_radius）的亮点不是灯。"""
        image = self._frame_with(lambda img: cv2.circle(img, (100, 100), 3, (0, 0, 255), -1))
        self.assertEqual(self.spotter.readings(image), [])

    def test_colour_competition_rejects_a_mismatched_colour(self):
        """候选灯圆内另一色像素更多时判掉（像素竞争）。"""
        image = approach_frame()
        # 半径 26 的"绿灯"里塞一个半径 20 的红块：绿轮廓仍然近似圆（外轮廓不外扩），
        # 但圆内红色像素比绿色多 → 绿色候选必须被竞争判据挡掉。
        cv2.circle(image, (300, 100), 26, (0, 255, 0), -1)
        cv2.circle(image, (300, 100), 20, (0, 0, 255), -1)
        readings = self.spotter.readings(image)
        self.assertEqual(
            [item for item in readings if item.color is LightColor.GREEN],
            [],
            "绿色候选应该被像素竞争挡掉",
        )
        # 关掉竞争判据后，绿色候选就会露出来（说明确实是这条判据挡的）。
        tolerant = LampSpotter(JunctionConfig(lamp_require_colour_dominance=False))
        greens = [item for item in tolerant.readings(image) if item.color is LightColor.GREEN]
        self.assertTrue(greens)

    def test_builtin_detection_can_be_switched_off(self):
        settings = JunctionConfig(builtin_lamp_detection=False)
        task = GreenJunctionTask(settings=settings)
        now = 20.0
        for index in range(10):
            now += FRAME_DT
            update = task.step(
                packet(two_lamp_frame(), index + 1, now), now, fake_line(error=0.9)
            )
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertIn("no light rule source", update.message)

    def test_light_side_must_repeat_before_turning(self):
        """A15：同一个"哪边是绿灯"要连续 light_confirm_frames 帧才算数。

        ``confirm_frames=1`` 时，IDLE 那一帧只把岔路记成候选，下一帧才进 DECIDE，
        所以前两帧是"确认岔路"，之后才开始数灯。
        """
        settings = JunctionConfig(light_confirm_frames=2, confirm_frames=1)
        task = GreenJunctionTask(settings=settings)
        now = 30.0
        for index in (1, 2):  # 确认岔路（第 2 帧进 DECIDE，还没数灯）
            now += FRAME_DT
            update = task.step(
                packet(two_lamp_frame(green_on_left=True), index, now), now, fake_line(error=0.9)
            )
        self.assertIsNone(task.chosen_branch)
        # 第 3 帧：绿灯在左 → 只数到 1，不许转身
        now += FRAME_DT
        update = task.step(
            packet(two_lamp_frame(green_on_left=True), 3, now), now, fake_line(error=0.9)
        )
        self.assertIsNone(task.chosen_branch)
        self.assertIn("confirming light side left (1/2", update.message)
        # 第 4 帧：绿灯换到右边 → 计数清零，仍然不许转身
        now += FRAME_DT
        update = task.step(
            packet(two_lamp_frame(green_on_left=False), 4, now), now, fake_line(error=0.9)
        )
        self.assertIsNone(task.chosen_branch)
        self.assertIn("confirming light side right (1/2", update.message)
        # 第 5 帧：同一侧再来一帧 → 走右边
        now += FRAME_DT
        update = task.step(
            packet(two_lamp_frame(green_on_left=False), 5, now), now, fake_line(error=0.9)
        )
        self.assertIs(task.chosen_branch, Branch.RIGHT)
        self.assertIn("take right branch", update.message)


class CoordinatorHarnessTests(unittest.TestCase):
    """把模块塞进**真正的** TaskCoordinator 跑一遍（协调器只调 ``step(frame, now)``）。

    这是 2026-09-15 报告里"44 条用例全都在传 line，没覆盖实车调用路径"的直接回应：
    这一条从头到尾都不传 ``line``，走的就是实车那条路。
    """

    def test_takeover_turn_and_release_through_the_coordinator(self):
        task = GreenJunctionTask(light_probe=lambda frame, now: green(Branch.RIGHT))
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        now = 1.05
        for _ in range(60):
            now += 0.05
            decision = harness.feed_image(now, approach_frame())
            if task.finished and decision.owner == "line":
                break
        self.assertTrue(
            any(row.task_name == "green_junction" for row in harness.traces),
            "模块在真正的协调器里没有接管",
        )
        self.assertEqual(task.state, JunctionState.COMPLETED)
        self.assertEqual(harness.owner, "line", "完成后必须把控制权交回巡线")
        # 转向请求必须落在骨架的护栏里（0.30 m/s、90 deg/s）。
        moved = harness.chassis.motion_calls
        self.assertTrue(moved)
        self.assertLessEqual(max(abs(item["z"]) for item in moved), 90.0)


class ConstantContractTests(unittest.TestCase):
    def test_stop_is_zero_motion(self):
        self.assertEqual(STOP, MotionCommand(0.0, 0.0, 0.0))
        self.assertEqual(STOP, MotionCommand())


class OfflineDemoTests(unittest.TestCase):
    """整条流程的离线回放：巡线 → 接管 → 完成 → 交回巡线。"""

    def test_demo_completes_and_returns_control(self):
        events = run_demo()
        self.assertEqual(
            events,
            [
                "line:TRACKING",
                "owner:external",
                "task:running",
                "branch:right",
                "motion:yaw=16.3",
                "task:completed",
                "owner:line",
                "line:TRACKING",
            ],
        )

    def test_demo_without_a_light_rule_never_completes(self):
        events = run_demo(light_probe=None)
        self.assertNotIn("task:completed", events)


if __name__ == "__main__":
    unittest.main()
