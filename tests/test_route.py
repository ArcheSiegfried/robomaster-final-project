"""长断线巡回（WP6a / Issue #6）。

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

from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
)

from coordinator import LINE_FOLLOWING, RELEASING, TASK_ACTIVE  # noqa: E402
from models import TaskStatus  # noqa: E402
from route import (  # noqa: E402
    BRIDGE_FORWARD_SPEED,
    SEARCH_YAW_SPEED,
    TOTAL_RECOVERY_SECONDS,
    RouteTask,
)


def far_fragment_frame(x=400, height=360, width=640):
    image = np.full((height, width, 3), 210, np.uint8)
    cv2.line(image, (x, 155), (x, 105), (255, 0, 0), 18)
    return image


def start_and_trigger(x=400):
    task = RouteTask()
    harness = TaskHarness(task=task)
    harness.start_line(now=1.0, x=x)
    before_grace = harness.feed_blank(1.30)
    assert before_grace.state == LINE_FOLLOWING
    takeover = harness.feed_blank(1.36)
    assert takeover.state == TASK_ACTIVE
    return task, harness, takeover


class RouteContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "route.py")

    def test_does_not_take_over_on_a_clear_line_frame(self):
        """合成帧里有一条清晰的蓝线。

        这条测试在你实现完之后**仍然必须通过**：能看到线的时候归巡线管，
        长断线任务只处理线真的断了的情况，不许抢正常巡线的活。
        """
        assert_inert_through_harness(self, RouteTask())


class RouteRecoveryTests(unittest.TestCase):
    def test_waits_beyond_the_base_grace_then_raises_view_while_stopped(self):
        _, harness, takeover = start_and_trigger()
        self.assertEqual(takeover.command.forward, 0.0)
        self.assertEqual(takeover.command.yaw, 0.0)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -5.0)
        self.assertEqual(harness.owner, "external")

    def test_blank_search_is_stationary_and_alternates_from_last_direction(self):
        _, harness, _ = start_and_trigger(x=400)
        first = harness.feed_blank(1.82)
        opposite = harness.feed_blank(2.30)
        self.assertEqual(first.command.forward, 0.0)
        self.assertEqual(first.command.lateral, 0.0)
        self.assertAlmostEqual(first.command.yaw, SEARCH_YAW_SPEED)
        self.assertAlmostEqual(opposite.command.yaw, -SEARCH_YAW_SPEED)

    def test_far_fragment_needs_confirmation_before_low_speed_bridge(self):
        _, harness, _ = start_and_trigger()
        first = harness.feed_image(1.82, far_fragment_frame())
        second = harness.feed_image(1.87, far_fragment_frame())
        self.assertEqual(first.command.forward, 0.0)
        self.assertAlmostEqual(second.command.forward, BRIDGE_FORWARD_SPEED)
        self.assertGreater(second.command.yaw, 0.0)
        self.assertLessEqual(abs(second.command.yaw), 25.0)

    def test_reacquire_requires_three_fresh_frames_then_restores_and_resumes(self):
        _, harness, _ = start_and_trigger()
        harness.feed_blank(1.82)
        one = harness.feed_line(1.90, x=330)
        two = harness.feed_line(1.95, x=330)
        done = harness.feed_line(2.00, x=330)
        self.assertEqual(one.task_update.status, TaskStatus.RUNNING)
        self.assertEqual(two.task_update.status, TaskStatus.RUNNING)
        self.assertEqual(done.state, RELEASING)
        self.assertEqual(done.task_update.status, TaskStatus.COMPLETED)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -25.0)

        waiting = harness.feed_line(2.20, x=330)
        self.assertEqual(waiting.state, RELEASING)
        resumed = harness.feed_line(2.50, x=330)
        self.assertEqual(resumed.state, LINE_FOLLOWING)
        self.assertTrue(harness.follower.motion_enabled)

    def test_total_timeout_fails_stops_and_restores_line_view(self):
        _, harness, _ = start_and_trigger()
        harness.feed_blank(1.82)
        failed = harness.feed_blank(1.36 + TOTAL_RECOVERY_SECONDS + 0.01)
        self.assertEqual(failed.state, RELEASING)
        self.assertEqual(failed.task_update.status, TaskStatus.FAILED)
        self.assertTrue(failed.force_stop)
        self.assertEqual(failed.command.forward, 0.0)
        self.assertEqual(failed.command.yaw, 0.0)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -25.0)

    def test_human_stop_during_search_restores_view_and_needs_explicit_resume(self):
        _, harness, _ = start_and_trigger()
        harness.feed_blank(1.82)
        harness.coordinator.human_stop(1.90)
        self.assertEqual(harness.owner, "line")
        self.assertFalse(harness.follower.motion_enabled)
        self.assertEqual(harness.gimbal.last_move()["pitch"], -25.0)
        self.assertFalse(harness.coordinator.human_resume(2.00))


# ============================================================================
# 你的用例区（成员补充）。建议至少覆盖：
#   [ ] 触发条件明确：必须晚于基础底座的 lost_grace_seconds=0.28s，
#       不得靠延长短时容错冒充长断线
#   [ ] 巡逻动作有硬超时，假时钟下方向、限幅、左右扫描次序可断言
#   [ ] 重新找到线之后要连续确认若干帧才算重获（防止误抓邻近线）
#   [ ] 找不到线时停车并报告失败，不许无限扫描
#   [ ] 任务结束后归还控制权，需要新鲜有效路线才恢复
#
# 参考蓝本（这是全场最赚的一个名额，竞速工程有现成实现）：
#   F:\robomaster\blue_line_following\line_following_core.py:81-131 _search_decision()
#     —— 用最后看到的线路方向决定首扫方向、扫线期间前进强制为 0、
#        超时硬停、按 sweep_index 奇偶左右交替扫
#   F:\robomaster\blue_line_following\race_v32_config.py:186-201
#     —— MAX_SEARCH_TIME=1.80 / 首扫 0.28s / 全扫 0.56s / 扫线 340 deg/s /
#        REACQUIRE_STABLE_FRAMES=3 / REACQUIRE_MAX_ERROR_JUMP=0.45
#   F:\robomaster\blue_line_following\race_v32_config.py:58-63
#     —— 断口两侧真实线段的几何关联参数（GAP_FRAGMENT_*）
#
# 单独自测命令：
#   python scripts/check_module.py route
# ============================================================================


if __name__ == "__main__":
    unittest.main()
