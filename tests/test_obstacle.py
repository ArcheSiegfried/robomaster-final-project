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

# 假设里的"橙色障碍"（BGR）。蓝线是 (255, 0, 0)，灰度地面是 210。
OBSTACLE_BGR = (0, 165, 255)


def obstacle_frame(x=320, center=(320, 310), size=(80, 60)):
    """蓝线 + 一块橙色障碍的合成帧（640x360，和夹具同尺寸）。"""
    image = line_frame(x)
    cx, cy = center
    half_w, half_h = size[0] // 2, size[1] // 2
    cv2.rectangle(
        image,
        (cx - half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        OBSTACLE_BGR,
        -1,
    )
    return image


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
# 覆盖：有障碍 / 无障碍 / 误检候选 / 噪声 / 限幅 / 超时 / 交回 / 冷却
# 全部离线：合成画面 + 假底盘 + 假时钟，不连相机、不连车。
# ============================================================================


class ObstacleDetectorTests(unittest.TestCase):
    """只看识别本身，不经过协调器。"""

    def setUp(self):
        self.detector = obstacle.ObstacleDetector()

    def test_detects_a_large_obstacle(self):
        result = self.detector.detect(obstacle_frame())
        self.assertTrue(result.valid)
        self.assertEqual(result.kind, obstacle.KIND)
        # 画的中心是 (320, 310)，认出来的中心应该就在附近
        self.assertAlmostEqual(result.center[0], 320, delta=6)
        self.assertAlmostEqual(result.center[1], 310, delta=6)
        left, top, right, bottom = result.box
        self.assertAlmostEqual(right - left, 80, delta=6)
        self.assertAlmostEqual(bottom - top, 60, delta=6)
        self.assertGreater(result.confidence, 0.0)

    def test_ignores_a_tiny_speck(self):
        """只有 12x12 的小色点：是噪声，不是障碍。"""
        result = self.detector.detect(obstacle_frame(center=(320, 310), size=(12, 12)))
        self.assertFalse(result.valid)

    def test_ignores_the_plain_line_and_blank_frames(self):
        self.assertFalse(self.detector.detect(line_frame()).valid)
        self.assertFalse(self.detector.detect(np.full((360, 640, 3), 210, np.uint8)).valid)

    def test_ignores_a_dark_grey_shadow_block(self):
        """审核风险 1：亮度 <=70 的灰块不许当障碍。

        v1 的暗色区间是 ((0,0,0),(180,255,70))，等于"任何够暗的像素"，
        车自己的影子 / 场地深色接缝 / 桌腿阴影全会命中。现在默认关掉了。
        """
        for level in (20, 40, 60, 70):
            image = line_frame()
            cv2.rectangle(image, (280, 280), (400, 340), (level, level, level), -1)
            self.assertFalse(
                self.detector.detect(image).valid,
                "亮度 %d 的灰块被误判成障碍了" % level,
            )

    def test_picks_the_bigger_nearer_candidate(self):
        """同时有两块时，选更大更靠下的那块。"""
        image = line_frame()
        cv2.rectangle(image, (200, 250), (240, 270), OBSTACLE_BGR, -1)   # 又小又远
        cv2.rectangle(image, (300, 300), (390, 350), OBSTACLE_BGR, -1)   # 又大又近
        result = self.detector.detect(image)
        self.assertTrue(result.valid)
        self.assertGreater(result.center[0], 300)
        self.assertGreater(result.center[1], 300)

    def test_returns_no_result_for_a_none_image(self):
        self.assertFalse(self.detector.detect(None).valid)


class ObstacleTakeoverTests(unittest.TestCase):
    """把模块接进真实协调器，用假底盘跑轨迹。"""

    def setUp(self):
        self.harness = TaskHarness(task=ObstacleTask())
        self.harness.start_line(now=1.0)

    def _feed_obstacle(self, start, frames):
        now = start
        decision = None
        for _ in range(frames):
            decision = feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
        return decision, now

    def test_takes_over_when_an_obstacle_appears(self):
        """连续看到障碍 -> 接管；接管的第一帧必须原地不动（先停车看清）。"""
        self.assertEqual(self.harness.owner, "line")
        decision, _ = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        self.assertEqual(self.harness.owner, "external")
        self.assertEqual(self.harness.task_name, "obstacle")
        self.assertEqual(decision.task_update.status, TaskStatus.RUNNING)
        self.assertEqual(decision.command.forward, 0.0)
        self.assertEqual(decision.command.lateral, 0.0)

    def test_does_not_take_over_before_the_confirm_frames(self):
        """只看到 1 帧疑似障碍，不许接管（防误触发）。"""
        for index in range(obstacle.CONFIRM_FRAMES - 1):
            feed_image(self.harness, obstacle_frame(), 1.05 + index * 0.05)
        self.assertEqual(self.harness.owner, "line")
        self.assertIsNone(self.harness.task_name)

    def test_dodge_goes_left_and_stays_inside_the_envelope(self):
        """方向：默认往左 = lateral 为负；所有命令都在骨架的安全范围内。"""
        decision, now = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        seen_lateral = []
        for _ in range(120):
            decision = feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
            command = decision.command
            self.assertLessEqual(abs(command.forward), 0.30)
            self.assertLessEqual(abs(command.lateral), 0.25)
            self.assertLessEqual(abs(command.yaw), 90.0)
            if command.lateral:
                seen_lateral.append(command.lateral)
            if decision.task_update and decision.task_update.status is not TaskStatus.RUNNING:
                break
        self.assertTrue(seen_lateral, "整段绕行没有发出任何横移命令")
        self.assertLess(seen_lateral[0], 0, "第一步应该往左让开（lateral 为负）")
        self.assertTrue(
            any(value > 0 for value in seen_lateral),
            "让开之后应该往回收（lateral 变正），实际是 %s" % seen_lateral,
        )

    def test_completes_then_hands_back_to_the_line(self):
        """绕完 -> COMPLETED -> 交回巡线 -> 再给一张新鲜蓝线就恢复。"""
        _, now = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        completed = False
        for _ in range(200):
            decision = feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
            if decision.task_update and decision.task_update.status is TaskStatus.COMPLETED:
                completed = True
                break
            self.assertLess(now - 1.05, obstacle.MAX_TOTAL_TIME + 1.0)
        self.assertTrue(completed, "绕行没有在预期时间内完成")
        self.assertLess(now - 1.05, obstacle.MAX_TOTAL_TIME + 1.0)

        # 交回后需要一张新鲜有效的线才恢复
        for _ in range(10):
            decision = feed_image(self.harness, line_frame(), now)
            now += 0.05
            if self.harness.owner == "line" and self.harness.state == "LINE_FOLLOWING":
                break
        self.assertEqual(self.harness.owner, "line")
        self.assertEqual(self.harness.state, "LINE_FOLLOWING")
        self.assertIsNone(self.harness.task_name)

    def test_total_timeout_fails_and_stops(self):
        """时间突然跳过头 -> 立刻 FAILED 停车，绝不一直绕下去。"""
        self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        decision = feed_image(
            self.harness, obstacle_frame(), 1.05 + obstacle.MAX_TOTAL_TIME + 1.0
        )
        self.assertEqual(decision.task_update.status, TaskStatus.FAILED)
        self.assertEqual(self.harness.owner, "line")
        self.assertEqual(decision.command.forward, 0.0)
        self.assertEqual(decision.command.lateral, 0.0)
        self.assertEqual(decision.command.yaw, 0.0)

    def test_stops_after_too_many_consecutive_dodges(self):
        """审核风险 2：障碍一直在，不许没完没了地绕。

        最多连绕 MAX_CONSECUTIVE_DODGES 次，再多就直接 FAILED 停车要人来看，
        并且**从此不再接管**（否则会变成每几秒停一下的走走停停）。
        这里故意把障碍放在线的右侧，让巡线全程有效，好把"反复触发"跑出来。
        """
        harness = TaskHarness(task=ObstacleTask())
        harness.start_line(now=1.0)
        now = 1.05
        image = obstacle_frame(center=(430, 310))
        dodges = 0
        failed = False
        takeovers = 0
        was_owner = False
        for _ in range(600):                       # 模拟 30 秒
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

        # 锁住之后：障碍还在，但必须不再接管
        locked_takeovers = takeovers
        for _ in range(400):                       # 再喂 20 秒
            decision = feed_image(harness, image, now)
            now += 0.05
            if decision.task_name is not None:
                locked_takeovers += 1
        self.assertEqual(
            locked_takeovers, takeovers, "锁住之后又接管了，锁没生效"
        )

    def test_unlocks_once_the_obstacle_clears(self):
        """障碍消失够久之后，应该重新愿意干活（不是永久瘫掉）。"""
        task = ObstacleTask()
        image = obstacle_frame(center=(430, 310))
        now = 1.0
        seq = 0
        for _ in range(2400):                      # 最多模拟 120 秒
            seq += 1
            task.step(FramePacket(image, seq, now), now)
            now += 0.05
            if task.locked:
                break
        self.assertTrue(task.locked, "连绕到上限了却没锁住")
        self.assertEqual(task.dodge_count, obstacle.MAX_CONSECUTIVE_DODGES)

        # 把已经开始的这次失败走完，回到 IDLE
        while task.stage != "IDLE":
            seq += 1
            task.step(FramePacket(image, seq, now), now)
            now += 0.05

        # 障碍消失足够久 -> 计数归零、解锁
        now += obstacle.CLEAR_SECONDS + 0.1
        seq += 1
        task.step(FramePacket(line_frame(), seq, now), now)
        self.assertEqual(task.dodge_count, 0)
        self.assertFalse(task.locked)

    def test_rearm_cooldown_blocks_an_immediate_second_takeover(self):
        """刚绕完的这段时间里，同一个障碍不许把车再拉去绕一次。"""
        _, now = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        for _ in range(200):
            decision = feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
            if decision.task_update and decision.task_update.status is TaskStatus.COMPLETED:
                break
        # 交回巡线
        for _ in range(10):
            feed_image(self.harness, line_frame(), now)
            now += 0.05
            if self.harness.owner == "line" and self.harness.state == "LINE_FOLLOWING":
                break
        self.assertEqual(self.harness.state, "LINE_FOLLOWING")

        # 冷却期内的障碍帧：不许接管
        for _ in range(obstacle.CONFIRM_FRAMES):
            decision = feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
        self.assertEqual(self.harness.owner, "line")

        # 冷却过去之后：同一个障碍应该重新能触发
        now += obstacle.REARM_SECONDS + 0.1
        for _ in range(obstacle.CONFIRM_FRAMES):
            decision = feed_image(self.harness, obstacle_frame(), now)
            now += 0.05
        self.assertEqual(self.harness.owner, "external")
        self.assertEqual(self.harness.task_name, "obstacle")


class ObstacleStateTests(unittest.TestCase):
    """不经过协调器，直接调 step()，测状态机本身。"""

    def test_no_obstacle_never_returns_running(self):
        task = ObstacleTask()
        for index in range(20):
            update = task.step(FramePacket(line_frame(), index + 1, 1.0 + index * 0.05), 1.0 + index * 0.05)
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


if __name__ == "__main__":
    unittest.main()
