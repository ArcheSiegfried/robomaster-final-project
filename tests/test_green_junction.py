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
    line_is_centered,
)
from examples.green_junction_demo import run_demo  # noqa: E402
from models import (  # noqa: E402
    FramePacket,
    LineDetection,
    MotionCommand,
    TaskStatus,
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

    def test_light_rule_belongs_to_other_module(self):
        self.assertEqual(LIGHT_OWNER, "traffic_light.py")


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
        self.assertFalse(detections_for_log(None)["valid"])


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
        settings = JunctionConfig(decision_timeout=5.0)
        task = GreenJunctionTask(settings=settings)
        now = self._advance_until(task, JunctionState.DECIDE)
        update = task.step(packet(junction_frame(), 20, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertEqual(update.motion, STOP)
        self.assertEqual(update.motion.forward, 0.0)

    def test_probe_error_does_not_crash_or_move(self):
        def broken_probe(frame, now):
            raise RuntimeError("别人的模块炸了")

        task = GreenJunctionTask(light_probe=broken_probe)
        now = self._advance_until(task, JunctionState.DECIDE)
        update = task.step(packet(junction_frame(), 20, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertEqual(update.motion, STOP)

    def test_red_light_waits_then_fails_and_stops(self):
        settings = JunctionConfig(decision_timeout=2.0)
        task = GreenJunctionTask(
            settings=settings,
            light_probe=lambda frame, now: LightReading(color=LightColor.RED),
        )
        now = self._advance_until(task, JunctionState.DECIDE)
        update = task.step(packet(junction_frame(), 21, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertEqual(update.motion, STOP)
        now += settings.decision_timeout + 0.5
        update = task.step(packet(junction_frame(), 22, now), now, fake_line(error=0.9))
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion, STOP)
        self.assertEqual(task.state, JunctionState.FAILED)
        self.assertIn("red", update.message)

    def test_no_light_source_at_all_fails_instead_of_guessing(self):
        settings = JunctionConfig(decision_timeout=0.4)
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
        settings = JunctionConfig(decision_timeout=0.5)
        task = GreenJunctionTask(settings=settings)
        now = self._advance_until(task, JunctionState.DECIDE)
        now += 1.0
        update = task.step(packet(blank_frame(), 5, now), now, fake_line())
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion, STOP)

    def test_driving_past_the_junction_without_a_rule_fails(self):
        """线已经回到中央、岔路形态还在，连续几帧都这样 → 车其实开过了 → 失败停车。"""
        settings = JunctionConfig(decision_timeout=5.0)
        task = GreenJunctionTask(settings=settings)
        now = self._advance_until(task, JunctionState.DECIDE)
        for index in range(settings.drove_past_frames):
            now += FRAME_DT
            update = task.step(packet(junction_frame(), 30 + index, now), now, fake_line(error=0.02))
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion, STOP)
        self.assertIn("past the junction", update.message)

    def test_single_contradictory_frame_is_not_enough_to_fail(self):
        """单帧巧合不算：判定"开过了"要连续几帧都矛盾。"""
        settings = JunctionConfig(decision_timeout=5.0)
        task = GreenJunctionTask(settings=settings)
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
                "motion:yaw=16.9",
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
