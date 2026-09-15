"""障碍检测与绕行（WP5 / Issue #5）。

契约测试在前，成员用例区在后。骨架已经把本文件注册进 task_registry，
你只要替换上面的实现即可，不需要改 main.py。
"""

import pathlib
import sys
import unittest

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from models import FramePacket, TaskStatus  # noqa: E402
from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
    line_frame,
)

import obstacle  # noqa: E402
from obstacle import ObstacleTask  # noqa: E402

# 合成画面约定：蓝线是 (255, 0, 0)，灰度地面是 210，尺寸 640x360。
# 障碍都画在**新的 ROI**（x 160..480, y 108..270）里面。
OBSTACLE_BGR = (0, 165, 255)     # 橙色道具
SKIN_BGR = (140, 170, 220)       # 手掌/手臂的典型肤色（BGR）


def obstacle_frame(x=320, center=(320, 200), size=(80, 60)):
    """蓝线 + 一块橙色障碍。"""
    image = line_frame(x)
    cx, cy = center
    half_w, half_h = size[0] // 2, size[1] // 2
    cv2.rectangle(
        image, (cx - half_w, cy - half_h), (cx + half_w, cy + half_h), OBSTACLE_BGR, -1
    )
    return image


def car_frame(center=(330, 200)):
    """**另一台车**当障碍：灰白车身 + 深色轮子，一个橙色像素都没有。

    这是 2026-09-11 实车实测失败的场景。注意位置要落在新 ROI 里：
    ROI 下沿抬到 0.75 之后，太近（画面太低）的东西不在检测区里。
    """
    image = line_frame()
    cx, cy = center
    cv2.rectangle(image, (cx - 60, cy - 55), (cx + 60, cy + 45), (120, 120, 120), -1)
    cv2.rectangle(image, (cx - 55, cy + 25), (cx - 30, cy + 45), (35, 35, 35), -1)
    cv2.rectangle(image, (cx + 30, cy + 25), (cx + 55, cy + 45), (35, 35, 35), -1)
    return image


def skin_frame():
    """手掌色块，大小和橙色道具一样 —— 但它是皮肤，不许当障碍。

    实测：皮肤的 H 落在 5~25，和橙色道具完全重叠；426 帧现场画面里被判成
    障碍的 4 帧全是人的手。所以要有这条负样本。
    """
    image = line_frame()
    cv2.rectangle(image, (280, 170), (360, 230), SKIN_BGR, -1)
    return image


def robot_self_frame():
    """只有"车自己身上的橙色件"（画面底部正中），没有外部障碍。

    实测：镜头正下方中间常驻一块橙色件，每帧在旧 ROI 里贡献 900~1700 px。
    新 ROI 下沿抬到 0.75，把它关在外面；这里还故意让它有一点探进 ROI，
    确认探进来的那一点也因为高度不够而被拒。
    """
    image = line_frame()
    cv2.rectangle(image, (280, 260), (360, 360), OBSTACLE_BGR, -1)
    return image


def soft_shadow_frame():
    """地面上一大片**软边**的阴影：真实影子就长这样，不许当成障碍。"""
    image = line_frame()
    shadow = np.zeros(image.shape[:2], np.uint8)
    cv2.ellipse(shadow, (330, 230), (170, 55), 0, 0, 360, 110, -1)
    shadow = cv2.GaussianBlur(shadow, (61, 61), 0)
    return cv2.subtract(image, cv2.merge([shadow, shadow, shadow]))


def feed_image(harness, image, now):
    """把一张自己画的图喂进真实的协调器（夹具只提供蓝线帧和空白帧）。"""
    harness.sequence += 1
    packet = FramePacket(image, harness.sequence, now)
    decision = harness.coordinator.step(packet, now)
    harness.traces.append(decision)
    return decision


class ObstacleContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "obstacle.py")

    def test_does_not_take_over_on_a_plain_line_frame(self):
        """合成帧里只有一条蓝线，没有任何障碍。

        这条测试在你实现完之后**仍然必须通过**：没有障碍就不许接管，
        否则车会在空跑道上做出绕行动作。
        """
        assert_inert_through_harness(self, ObstacleTask())


# ============================================================================
# 成员用例区（4 号 / 高睿盈）
#
# 对应 2026-09-14 实车测试报告的三条根因，逐条钉死：
#   P0-1 车自己的橙色件 / 皮肤色         -> robot_self / skin 两条负样本
#   P0-2 确认后 0.35s 延时才侧移          -> first_takeover_frame 断言
#   P1-3 ROI 下沿太近                    -> 障碍都画在新 ROI 里，太近的不算
# 全部离线：合成画面 + 假底盘 + 假时钟，不连相机、不连车。
# ============================================================================


class ObstacleDetectorTests(unittest.TestCase):
    """只看识别本身，不经过协调器。"""

    def setUp(self):
        self.detector = obstacle.ObstacleDetector()

    # ---- 正样本 ----

    def test_detects_another_car_without_knowing_its_colour(self):
        """障碍是另一台车，灰白车身、没有橙色 —— 必须靠"结构"认出来。"""
        result = self.detector.detect(car_frame())
        self.assertTrue(result.valid, "另一台车没认出来")
        self.assertGreater(result.center[0], 250)
        self.assertLess(result.center[0], 420)

    def test_detects_a_large_orange_obstacle(self):
        result = self.detector.detect(obstacle_frame())
        self.assertTrue(result.valid)
        self.assertEqual(result.kind, obstacle.KIND)
        self.assertAlmostEqual(result.center[0], 320, delta=10)
        self.assertGreater(result.confidence, 0.0)

    # ---- 负样本（报告点名要的）----

    def test_ignores_the_robots_own_orange_part(self):
        """P0-1：车自己的橙色件不许触发（旧 ROI 下它每帧有 900~1700 px）。"""
        result = self.detector.detect(robot_self_frame())
        self.assertFalse(result.valid, "把车自己身上的橙色件当成障碍了")

    def test_ignores_the_measured_self_occlusion_strip(self):
        """报告实测：车自己那块橙色件在 ROI 里是 **72×20 的扁平条**（中位数）。

        426 帧原始数据里，这种尺寸的色块有 423 帧。新加的高度判据
        （>= 22% ROI 高 = 36 px）一关就把它全部挡掉 —— 这里把它放在 ROI 正中间，
        证明**即使位置落在 ROI 里面**也过不了。
        """
        image = line_frame()
        cv2.rectangle(image, (240, 200), (312, 220), OBSTACLE_BGR, -1)   # 72 x 20
        self.assertFalse(self.detector.detect(image).valid, "72x20 的扁条被当成障碍了")

    def test_ignores_a_skin_coloured_hand(self):
        """P0-1：手掌/手臂的肤色不许触发（实测 426 帧里 4 帧误触发全是手）。"""
        result = self.detector.detect(skin_frame())
        self.assertFalse(result.valid, "把皮肤色的东西当成障碍了")

    def test_ignores_a_soft_shadow(self):
        """审核风险 1：影子不能当障碍（真实影子是软边的）。"""
        self.assertFalse(self.detector.detect(soft_shadow_frame()).valid)

    def test_ignores_a_tiny_speck(self):
        """只有 12x12 的小色点：是噪声，不是障碍。"""
        result = self.detector.detect(obstacle_frame(center=(320, 200), size=(12, 12)))
        self.assertFalse(result.valid)

    def test_ignores_the_plain_line_and_blank_frames(self):
        self.assertFalse(self.detector.detect(line_frame()).valid)
        self.assertFalse(self.detector.detect(np.full((360, 640, 3), 210, np.uint8)).valid)

    def test_picks_the_bigger_nearer_candidate(self):
        """同时有两块时，选更大更靠下的那块。"""
        image = line_frame()
        cv2.rectangle(image, (200, 170), (240, 195), OBSTACLE_BGR, -1)   # 又小又远
        cv2.rectangle(image, (300, 180), (390, 240), OBSTACLE_BGR, -1)   # 又大又近
        result = self.detector.detect(image)
        self.assertTrue(result.valid)
        self.assertGreater(result.center[0], 300)
        self.assertGreater(result.center[1], 180)

    def test_returns_no_result_for_a_none_image(self):
        self.assertFalse(self.detector.detect(None).valid)


class ObstacleParameterTests(unittest.TestCase):
    """按官方信息核对参数：障碍是**一辆静止的小车**（另一台同型 RoboMaster）。

    同型车长约 30~32 cm、宽约 24 cm。下面这些距离是从这个尺寸算出来的，
    不是拍脑袋；改了速度或时长，这几条会立刻告诉你还够不够。
    """

    def test_dodge_distances_clear_a_car_sized_obstacle(self):
        lateral = obstacle.SIDE_SPEED * obstacle.T_OUT_TIME
        forward = obstacle.FWD_SPEED * obstacle.T_PASS_TIME
        self.assertGreaterEqual(
            lateral, 0.34, "横移只有 %.2fm，两辆车（各 24cm 宽）错不开" % lateral
        )
        self.assertGreaterEqual(
            forward, 0.41, "前进只有 %.2fm，两辆车（各约 31cm 长）越不过去" % forward
        )

    def test_total_timeout_is_long_enough_for_all_four_stages(self):
        """总超时如果比四段加起来还短，那这个动作永远走不完。"""
        needed = (
            obstacle.HOLD_BEFORE_GO
            + obstacle.T_OUT_TIME
            + obstacle.T_PASS_TIME
            + obstacle.T_BACK_TIME
            + obstacle.SEEK_TIME
        )
        self.assertGreater(
            obstacle.MAX_TOTAL_TIME, needed,
            "MAX_TOTAL_TIME=%.2fs 不够走完四段（需要 %.2fs）"
            % (obstacle.MAX_TOTAL_TIME, needed),
        )

    def test_task_envelope_respected_by_our_own_speeds(self):
        """我们自己给的速度必须还在骨架限幅之内（横移 0.25、前进 0.30）。"""
        for speed in (obstacle.SIDE_SPEED, obstacle.BACK_SPEED, obstacle.SEEK_SPEED):
            self.assertLessEqual(abs(speed), 0.25)
        self.assertLessEqual(abs(obstacle.FWD_SPEED), 0.30)


class ObstacleLineCheckTests(unittest.TestCase):
    """SEEK 段用的"线还在不在"判断。"""

    def test_sees_the_line_on_a_line_frame(self):
        self.assertTrue(obstacle.line_is_visible(line_frame()))

    def test_does_not_see_the_line_on_a_blank_frame(self):
        self.assertFalse(obstacle.line_is_visible(np.full((360, 640, 3), 210, np.uint8)))


class ObstacleTakeoverTests(unittest.TestCase):
    """把模块接进真实协调器，用假底盘跑轨迹。"""

    def setUp(self):
        self.harness = TaskHarness(task=ObstacleTask())
        self.harness.start_line(now=1.0)

    def _feed(self, image, start, frames):
        now = start
        decision = None
        for _ in range(frames):
            decision = feed_image(self.harness, image, now)
            now += 0.05
        return decision, now

    def _feed_obstacle(self, start, frames, image=None):
        return self._feed(obstacle_frame() if image is None else image, start, frames)

    # ---- P0-2：确认完当帧就要侧移 ----

    def test_first_takeover_frame_already_sidesteps(self):
        """起手不许再有零速停车段。

        报告实测：确认 3 帧 + HOLD 0.20s 的零速，会让车在侧移之前多前冲约 11 cm，
        操作者看到的就是"车停住了、没绕"。现在确认那一帧就必须在下发侧移。
        """
        decision, _ = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        self.assertEqual(self.harness.owner, "external", "没有接管")
        self.assertEqual(decision.task_update.status, TaskStatus.RUNNING)
        self.assertNotEqual(decision.command.lateral, 0.0, "接管第一帧居然还是零速度")
        self.assertAlmostEqual(
            abs(decision.command.lateral), obstacle.SIDE_SPEED, delta=1e-6
        )

    def test_never_commands_a_full_stop_while_dodging(self):
        """接管期间不许出现"前进和横移同时为零"的整帧——那就是停车段（P0-2）。

        注意：PASS 段本来就是只前进不横移，所以"横移为零"本身不算错；
        错的是**三轴全零**（车原地不动）。
        """
        _, now = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        for _ in range(200):
            decision = feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
            status = decision.task_update.status if decision.task_update else None
            if status is not TaskStatus.RUNNING:
                break
            command = decision.command
            self.assertFalse(
                command.forward == 0.0 and command.lateral == 0.0,
                "接管期间出现整帧零速度：又变成停车段了",
            )

    # ---- 正样本端到端 ----

    def test_takes_over_for_another_car(self):
        """实车实测场景的端到端回归：另一台车 -> 接管。"""
        self.assertEqual(self.harness.owner, "line")
        decision, _ = self._feed_obstacle(
            1.05, obstacle.CONFIRM_FRAMES, image=car_frame()
        )
        self.assertEqual(self.harness.owner, "external", "另一台车没有触发接管")
        self.assertEqual(self.harness.task_name, "obstacle")

    def test_does_not_take_over_before_the_confirm_frames(self):
        """只看到 1 帧疑似障碍，不许接管（防误触发）。"""
        for index in range(obstacle.CONFIRM_FRAMES - 1):
            feed_image(self.harness, obstacle_frame(), 1.05 + index * 0.05)
        self.assertEqual(self.harness.owner, "line")
        self.assertIsNone(self.harness.task_name)

    def test_dodge_goes_left_far_enough_and_stays_in_the_envelope(self):
        """方向往左；横移总距离要够绕开另一台车；所有命令都在骨架安全范围内。"""
        _, now = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        laterals = []
        forwards = []
        for _ in range(200):
            decision = feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
            command = decision.command
            self.assertLessEqual(abs(command.forward), 0.30)
            self.assertLessEqual(abs(command.lateral), 0.25)
            self.assertLessEqual(abs(command.yaw), 90.0)
            if command.lateral:
                laterals.append(command.lateral)
            if command.forward:
                forwards.append(command.forward)
            if decision.task_update and decision.task_update.status is not TaskStatus.RUNNING:
                break
        self.assertTrue(laterals, "整段绕行没有发出任何横移命令")
        self.assertTrue(
            forwards,
            "整段绕行没有发出任何前进命令 —— PASS 段被跳过了（曾经真的犯过这个错）",
        )
        self.assertLess(laterals[0], 0, "第一步应该往左让开（lateral 为负）")
        self.assertGreater(max(laterals), 0, "让开之后应该往回收")
        outward = [value for value in laterals if value < 0]
        self.assertGreaterEqual(
            abs(outward[0]) * obstacle.T_OUT_TIME, 0.30,
            "横移距离不到 30cm，绕不开另一台车",
        )

    def test_completes_then_hands_back_to_the_line(self):
        """绕完 -> 主动找到线 -> COMPLETED -> 交回巡线 -> 恢复。"""
        _, now = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        completed = False
        for _ in range(300):
            decision = feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
            if decision.task_update and decision.task_update.status is TaskStatus.COMPLETED:
                completed = True
                break
        self.assertTrue(completed, "绕行没有在预期时间内完成")
        self.assertLess(now - 1.05, obstacle.MAX_TOTAL_TIME + 1.0)

        for _ in range(10):
            feed_image(self.harness, line_frame(), now)
            now += 0.05
            if self.harness.owner == "line" and self.harness.state == "LINE_FOLLOWING":
                break
        self.assertEqual(self.harness.owner, "line")
        self.assertEqual(self.harness.state, "LINE_FOLLOWING")
        self.assertIsNone(self.harness.task_name)

    def test_total_timeout_fails_and_stops(self):
        """绕行拖太久 -> 立刻 FAILED 停车，绝不一直绕下去。

        这里用白盒方式把"开始时刻"往前拨 10 秒来触发总超时：
        直接跳时间会先命中"被外部踢掉"的断档检测（STALE_STEP_GAP），那是另一条路径。
        """
        task = ObstacleTask()
        now = 1.0
        seq = 0
        while task.stage == "IDLE":
            seq += 1
            task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
        task.started_at -= obstacle.MAX_TOTAL_TIME + 1.0
        update = task.step(FramePacket(obstacle_frame(), seq + 1, now), now)
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion.forward, 0.0)
        self.assertEqual(update.motion.lateral, 0.0)
        self.assertEqual(update.motion.yaw, 0.0)

    def test_restarts_cleanly_after_an_external_release(self):
        """协调器从外面把控制权拿走时**不会通知模块**，再被问到时不许从半路接着走。

        实车报告（2026-09-15）里"接管 0.1 秒后就 COMPLETED"就是这么造出来的：
        stage 停在半路、每段时间都早已过期，于是一帧跳一段把流程"走"完，
        看起来像绕过去了，其实一动没动。
        """
        task = ObstacleTask()
        now = 1.0
        seq = 0
        while task.stage != "PASS":
            seq += 1
            task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
        self.assertEqual(task.stage, "PASS")

        # 模拟：中途被踢掉（视频中断 / 人工按键），2.6 秒之后才重新被问到
        now += 2.6
        seq += 40
        update = task.step(FramePacket(obstacle_frame(), seq, now), now)
        self.assertIn(task.stage, ("IDLE", "OUT"), "被踢之后居然从半路接着走了")
        if update.status is TaskStatus.RUNNING:
            self.assertEqual(update.motion.forward, 0.0, "被踢之后居然直接往前开")

    def test_never_completes_before_the_dodge_has_actually_run(self):
        """完整动作最短 1.7 + 1.6 + 1.2 = 4.5 秒，绝不允许更早报 COMPLETED。

        这条是上面那个实车假象的直接回归测试。
        """
        task = ObstacleTask()
        now = 1.0
        seq = 0
        takeover_at = None
        update = None
        for _ in range(400):
            seq += 1
            update = task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
            if takeover_at is None and update.status is TaskStatus.RUNNING:
                takeover_at = now
            if update.status is TaskStatus.COMPLETED:
                break
        self.assertIsNotNone(takeover_at, "一直没有接管")
        self.assertEqual(update.status, TaskStatus.COMPLETED)
        shortest = obstacle.T_OUT_TIME + obstacle.T_PASS_TIME + obstacle.T_BACK_TIME
        self.assertGreaterEqual(
            now - takeover_at,
            shortest - 0.2,
            "完成得太快：动作根本没跑完（实车报告里那种假 COMPLETED）",
        )

    def test_stops_after_too_many_consecutive_dodges(self):
        """审核风险 2：障碍一直在，不许没完没了地绕。

        最多连绕 MAX_CONSECUTIVE_DODGES 次，再多就直接 FAILED 停车要人来看，
        并且**从此不再接管**（否则会变成每几秒停一下的走走停停）。
        这里故意把障碍放在线的右侧，让巡线全程有效，好把"反复触发"跑出来。
        """
        harness = TaskHarness(task=ObstacleTask())
        harness.start_line(now=1.0)
        now = 1.05
        image = obstacle_frame(center=(430, 200))
        dodges = 0
        failed = False
        takeovers = 0
        was_owner = False
        for _ in range(1200):                      # 模拟 60 秒
            decision = feed_image(harness, image, now)
            now += 0.05
            owned = decision.task_name is not None
            if owned and not was_owner:
                takeovers += 1
            was_owner = owned
            if decision.task_update is None:
                continue
            if decision.task_update.status is TaskStatus.COMPLETED:
                dodges += 1
            elif decision.task_update.status is TaskStatus.FAILED:
                failed = True

        self.assertTrue(failed, "连绕之后没有停下来，可能又在无限绕")
        self.assertLessEqual(
            dodges, obstacle.MAX_CONSECUTIVE_DODGES,
            "连绕次数超过了上限：%d 次" % dodges,
        )
        self.assertEqual(
            takeovers, obstacle.MAX_CONSECUTIVE_DODGES + 1,
            "接管次数应该正好是 %d 次绕行 + 1 次失败锁停，实际 %d 次"
            % (obstacle.MAX_CONSECUTIVE_DODGES, takeovers),
        )

        locked_takeovers = takeovers
        for _ in range(600):                       # 再喂 30 秒：锁住之后不许再接管
            decision = feed_image(harness, image, now)
            now += 0.05
            if decision.task_name is not None:
                locked_takeovers += 1
        self.assertEqual(locked_takeovers, takeovers, "锁住之后又接管了，锁没生效")

    def test_unlocks_once_the_obstacle_clears(self):
        """障碍消失够久之后，应该重新愿意干活（不是永久瘫掉）。"""
        task = ObstacleTask()
        image = obstacle_frame(center=(430, 200))
        now = 1.0
        seq = 0
        for _ in range(3000):                      # 最多模拟 150 秒
            seq += 1
            task.step(FramePacket(image, seq, now), now)
            now += 0.05
            if task.locked:
                break
        self.assertTrue(task.locked, "连绕到上限了却没锁住")
        self.assertEqual(task.dodge_count, obstacle.MAX_CONSECUTIVE_DODGES)

        while task.stage != "IDLE":
            seq += 1
            task.step(FramePacket(image, seq, now), now)
            now += 0.05

        now += obstacle.CLEAR_SECONDS + 0.1
        seq += 1
        task.step(FramePacket(line_frame(), seq, now), now)
        self.assertEqual(task.dodge_count, 0)
        self.assertFalse(task.locked)

    def test_rearm_cooldown_blocks_an_immediate_second_takeover(self):
        """刚绕完的这段时间里，同一个障碍不许把车再拉去绕一次。"""
        _, now = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        for _ in range(300):
            decision = feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
            if decision.task_update and decision.task_update.status is TaskStatus.COMPLETED:
                break
        for _ in range(10):
            feed_image(self.harness, line_frame(), now)
            now += 0.05
            if self.harness.owner == "line" and self.harness.state == "LINE_FOLLOWING":
                break
        self.assertEqual(self.harness.state, "LINE_FOLLOWING")

        for _ in range(obstacle.CONFIRM_FRAMES):
            feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
        self.assertEqual(self.harness.owner, "line")

        now += obstacle.REARM_SECONDS + 0.1
        for _ in range(obstacle.CONFIRM_FRAMES):
            feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
        self.assertEqual(self.harness.owner, "external")
        self.assertEqual(self.harness.task_name, "obstacle")


class ObstacleStateTests(unittest.TestCase):
    """不经过协调器，直接调 step()，测状态机本身。"""

    def test_no_obstacle_never_returns_running(self):
        task = ObstacleTask()
        for index in range(20):
            now = 1.0 + index * 0.05
            update = task.step(FramePacket(line_frame(), index + 1, now), now)
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
            self.assertIsNone(update.motion)

    def test_running_is_never_interrupted_by_a_missing_obstacle(self):
        """一旦 RUNNING，就算障碍这一帧没认出来（比如被挡住），也必须继续 RUNNING。"""
        task = ObstacleTask()
        now = 1.0
        for _ in range(obstacle.CONFIRM_FRAMES):
            update = task.step(FramePacket(obstacle_frame(), 1, now), now)
            now += 0.05
        self.assertEqual(update.status, TaskStatus.RUNNING)

        update = task.step(FramePacket(line_frame(), 2, now), now)
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertIsNotNone(update.motion)

    def test_seek_moves_back_towards_the_line_and_fails_without_it(self):
        """绕完线不见了：必须主动往回挪去找，找不到就停车报失败。"""
        task = ObstacleTask()
        now = 1.0
        seq = 0
        while task.stage != "SEEK":
            seq += 1
            task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
            self.assertLess(now - 1.0, obstacle.MAX_TOTAL_TIME)

        blank = np.full((360, 640, 3), 210, np.uint8)
        laterals = []
        update = None
        for _ in range(200):
            seq += 1
            update = task.step(FramePacket(blank, seq, now), now)
            now += 0.05
            if update.status is TaskStatus.RUNNING:
                laterals.append(update.motion.lateral)
            else:
                break
        self.assertEqual(update.status, TaskStatus.FAILED, "找不到线必须停车报失败")
        self.assertTrue(laterals)
        self.assertTrue(all(value > 0 for value in laterals),
                        "往左绕完应该往右（线的方向）挪回去，实际 %s" % laterals)

    def test_seek_completes_when_the_line_is_found(self):
        """绕完线还在：连续确认两帧就交回。"""
        task = ObstacleTask()
        now = 1.0
        seq = 0
        while task.stage != "SEEK":
            seq += 1
            task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
        update = None
        for _ in range(10):
            seq += 1
            update = task.step(FramePacket(line_frame(), seq, now), now)
            now += 0.05
            if update.status is not TaskStatus.RUNNING:
                break
        self.assertEqual(update.status, TaskStatus.COMPLETED)


if __name__ == "__main__":
    unittest.main()
