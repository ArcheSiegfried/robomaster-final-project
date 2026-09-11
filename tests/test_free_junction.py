"""无拥堵岔路（WP6b / Issue #6）。

契约测试在前，成员用例区在后。骨架已经把这个文件登记进 task_registry，
你只要替换上面的实现即可，不需要改 main.py。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
    line_frame,
)

from free_junction import (  # noqa: E402
    Branch,
    FreeJunctionConfig,
    FreeJunctionDetector,
    FreeJunctionTask,
    JunctionState,
)
from models import FramePacket, TaskStatus  # noqa: E402


class FreeJunctionContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "free_junction.py")

    def test_does_not_take_over_on_a_single_line_frame(self):
        """合成帧里只有一条线，没有岔路。

        这条测试在你实现完之后**仍然必须通过**：不能把普通弯道当成岔路。
        """
        assert_inert_through_harness(self, FreeJunctionTask())


# ============================================================================
# 你的用例区（成员补充）
#
# 合成帧的坐标系与真机一致：360x640 BGR，蓝线在 HSV 上是 (255, 0, 0)。
# 默认 ROI 是 x 6%~94%、y 54%~96%，也就是 x 38~601、y 194~345。
#
# 覆盖的场景（对应 TASKS.md 的"三类场景各自触发/完成/失败可说明"）：
#   [x] 岔路判据，以及它与普通弯道、与"只有一条线"的区分
#   [x] 一侧被堵 → 走另一侧；两侧都通 / 两侧都堵 → 判据不成立 → 停车报失败
#   [x] 固定规则（decision_rule="fixed"）与兜底边（fallback_branch）
#   [x] 动作有硬超时和限幅，随时可取消；RUNNING 不会中途改口
#   [x] 完成后交回控制权，并且不会在原地反复触发
#   [x] 与绿灯岔路的分工：本模块只做拥堵判据，不认灯色（代码里没有灯色逻辑）
# ============================================================================

BLUE = (255, 0, 0)        # BGR 里的纯蓝，HSV 约为 (120, 255, 255)
FLOOR = 210
BLOCK = (35, 35, 35)      # 暗色障碍块：亮度 35 < floor_min_v(90) → 算"被占住"

# 在 ROI 里画一个岔路：下面一条主干，上面分成左右两条。
STEM_TOP = 300
BRANCH_TOP = 190
LEFT_TIP = 200
RIGHT_TIP = 440

# 分叉行上方那条判定带（y 201~269）里放障碍块；故意不盖住分叉行本身。
LEFT_BLOCKER = (60, 205, 300, 265)
RIGHT_BLOCKER = (345, 205, 585, 265)


def fork_frame(block_left=False, block_right=False):
    """合成岔路帧：可选在某一侧放一个障碍块。"""
    image = np.full((360, 640, 3), FLOOR, np.uint8)
    cv2.rectangle(image, (308, STEM_TOP), (332, 350), BLUE, -1)
    cv2.line(image, (320, STEM_TOP), (LEFT_TIP, BRANCH_TOP), BLUE, 18)
    cv2.line(image, (320, STEM_TOP), (RIGHT_TIP, BRANCH_TOP), BLUE, 18)
    if block_left:
        x0, y0, x1, y1 = LEFT_BLOCKER
        cv2.rectangle(image, (x0, y0), (x1, y1), BLOCK, -1)
    if block_right:
        x0, y0, x1, y1 = RIGHT_BLOCKER
        cv2.rectangle(image, (x0, y0), (x1, y1), BLOCK, -1)
    return image


def run_task(task, image, frames=240, dt=0.05, start=1.0):
    """按假时钟把同一张图喂进去，直到出现终态。返回 (更新列表, 结束时间)。"""
    records = run_with_states(task, image, frames=frames, dt=dt, start=start)
    updates = [update for _state, update, _now in records]
    return updates, (records[-1][2] if records else start)


def run_with_states(task, image, frames=240, dt=0.05, start=1.0):
    """同上，但把每帧之后的 (状态, 更新, 当前时间) 一起记下来。"""
    records = []
    now = start
    for index in range(frames):
        packet = FramePacket(image, index + 1, now)
        update = task.step(packet, now)
        records.append((task.state, update, now))
        now += dt
        if update.status in (TaskStatus.COMPLETED, TaskStatus.FAILED):
            break
    return records


def turn_yaws(records):
    """只取"转向阶段实际发出的" yaw，不含对准和直行阶段。"""
    return [
        update.motion.yaw
        for state, update, _now in records
        if state is JunctionState.TURN and update.status is TaskStatus.RUNNING and update.motion
    ]


class ForkDetectionTests(unittest.TestCase):
    def test_fork_is_detected_in_the_synthetic_frame(self):
        detection = FreeJunctionDetector().detect(fork_frame(block_left=True))
        self.assertTrue(detection.valid)
        self.assertGreater(detection.separation_px, 0)
        self.assertLess(detection.left_x, detection.right_x)
        # 坐标必须是整幅图像像素，落在画面里。
        self.assertGreater(detection.split_row, 0)
        self.assertLess(detection.split_row, 360)
        self.assertGreater(detection.split_x, 0)
        self.assertLess(detection.split_x, 640)

    def test_a_single_line_is_not_a_fork(self):
        detector = FreeJunctionDetector()
        self.assertFalse(detector.detect(line_frame()).valid)
        self.assertFalse(detector.detect(np.full((360, 640, 3), FLOOR, np.uint8)).valid)

    def test_bad_input_never_crashes(self):
        detector = FreeJunctionDetector()
        self.assertFalse(detector.detect(None).valid)
        self.assertFalse(detector.detect(np.zeros((3, 3), np.uint8)).valid)
        self.assertFalse(detector.detect(np.zeros((8, 8, 3), np.uint8)).valid)

    def test_detect_helper_reports_no_result_without_a_junction(self):
        task = FreeJunctionTask()
        result = task.detect(line_frame())
        self.assertFalse(result.valid)
        self.assertIsNone(result.center)
        self.assertEqual(result.kind, "free_junction")


class ChoosingBranchTests(unittest.TestCase):
    def test_left_blocked_takes_the_right_branch(self):
        task = FreeJunctionTask()
        records = run_with_states(task, fork_frame(block_left=True))
        updates = [update for _state, update, _now in records]
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.RIGHT)
        self.assertIn("blocked", task.last_reason)
        # 走右边 → 按项目约定 yaw 为正（正值右转）。
        yaws = turn_yaws(records)
        self.assertTrue(yaws, "应该进入转向阶段")
        self.assertTrue(all(yaw > 0 for yaw in yaws), yaws)

    def test_right_blocked_takes_the_left_branch(self):
        task = FreeJunctionTask()
        records = run_with_states(task, fork_frame(block_right=True))
        updates = [update for _state, update, _now in records]
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.LEFT)
        yaws = turn_yaws(records)
        self.assertTrue(yaws, "应该进入转向阶段")
        self.assertTrue(all(yaw < 0 for yaw in yaws), yaws)

    def test_both_open_fails_instead_of_guessing(self):
        """两侧都通、又没有明显差距 → 判据不成立 → 停车报失败，不瞎选。"""
        task = FreeJunctionTask()
        updates, _ = run_task(task, fork_frame())
        self.assertIs(updates[-1].status, TaskStatus.FAILED)
        self.assertIsNone(task.chosen_branch)
        self.assertIn("no clear winner", task.last_message)
        self.assertEqual(updates[-1].motion.forward, 0.0)
        self.assertEqual(updates[-1].motion.yaw, 0.0)

    def test_both_blocked_fails_instead_of_guessing(self):
        task = FreeJunctionTask()
        updates, _ = run_task(task, fork_frame(block_left=True, block_right=True))
        self.assertIs(updates[-1].status, TaskStatus.FAILED)
        self.assertIsNone(task.chosen_branch)
        self.assertIn("both sides blocked", task.last_message)
        self.assertEqual(updates[-1].motion.forward, 0.0)

    def test_fallback_branch_is_used_when_vision_cannot_tell(self):
        settings = FreeJunctionConfig(fallback_branch="left")
        task = FreeJunctionTask(settings=settings)
        updates, _ = run_task(task, fork_frame())
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.LEFT)
        self.assertIn("fallback", task.last_reason)

    def test_fixed_rule_ignores_vision(self):
        settings = FreeJunctionConfig(decision_rule="fixed", fixed_branch="right")
        task = FreeJunctionTask(settings=settings)
        updates, _ = run_task(task, fork_frame(block_left=True, block_right=True))
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.RIGHT)

    def test_fixed_rule_without_a_side_fails(self):
        settings = FreeJunctionConfig(decision_rule="fixed", fixed_branch=None)
        task = FreeJunctionTask(settings=settings)
        updates, _ = run_task(task, fork_frame(block_left=True))
        self.assertIs(updates[-1].status, TaskStatus.FAILED)
        self.assertIn("without a side", task.last_message)

    def test_unknown_decision_rule_fails_instead_of_guessing(self):
        settings = FreeJunctionConfig(decision_rule="vibes")
        task = FreeJunctionTask(settings=settings)
        updates, _ = run_task(task, fork_frame(block_left=True))
        self.assertIs(updates[-1].status, TaskStatus.FAILED)


class OwnershipAndTimingTests(unittest.TestCase):
    def test_running_never_reverts_to_not_triggered(self):
        """红线 8：一旦说要接管，就必须一路 RUNNING 到 COMPLETED / FAILED。"""
        task = FreeJunctionTask()
        updates, _ = run_task(task, fork_frame(block_left=True), frames=30)
        started = False
        for update in updates:
            if update.status is TaskStatus.RUNNING:
                started = True
                continue
            if started:
                self.assertIn(update.status, (TaskStatus.COMPLETED, TaskStatus.FAILED))
        self.assertTrue(started)

    def test_waiting_for_the_criterion_stands_still(self):
        """等判据期间必须是零速度（A10）：不猜、也不往前冲。"""
        task = FreeJunctionTask()
        updates, _ = run_task(task, fork_frame(), frames=8)
        running = [u for u in updates if u.status is TaskStatus.RUNNING]
        self.assertTrue(running, "两端都通时应该进入等判据阶段")
        for update in running:
            self.assertEqual(update.motion.forward, 0.0)
            self.assertEqual(update.motion.yaw, 0.0)

    def test_commands_stay_inside_limits(self):
        task = FreeJunctionTask()
        updates, _ = run_task(task, fork_frame(block_left=True))
        for update in updates:
            if update.motion is None:
                continue
            self.assertLessEqual(abs(update.motion.forward), 0.30)
            self.assertLessEqual(abs(update.motion.lateral), 0.25)
            self.assertLessEqual(abs(update.motion.yaw), 90.0)
            for value in (update.motion.forward, update.motion.lateral, update.motion.yaw):
                self.assertTrue(np.isfinite(value))

    def test_task_time_budget_is_enforced(self):
        """总时长有硬上限：一直不结束就报失败，不会无限接管。"""
        settings = FreeJunctionConfig(max_task_seconds=0.5, approach_timeout=10.0)
        task = FreeJunctionTask(settings=settings)
        updates, _ = run_task(task, fork_frame(block_left=True), frames=60)
        self.assertIs(updates[-1].status, TaskStatus.FAILED)
        self.assertIn("time budget", task.last_message)
        self.assertEqual(updates[-1].motion.forward, 0.0)

    def test_lost_line_while_owning_control_stops_the_car(self):
        task = FreeJunctionTask()
        blank = np.full((360, 640, 3), FLOOR, np.uint8)
        now = 1.0
        for index in range(6):
            task.step(FramePacket(fork_frame(block_left=True), index + 1, now), now)
            now += 0.05
        self.assertTrue(task.active, "前几帧应该已经接管")
        last = None
        for index in range(10):
            last = task.step(FramePacket(blank, 100 + index, now), now)
            now += 0.05
            if last.status is TaskStatus.FAILED:
                break
        self.assertIs(last.status, TaskStatus.FAILED)
        self.assertIn("line lost", task.last_message)
        self.assertEqual(last.motion.forward, 0.0)

    def test_missing_frame_while_owning_control_fails(self):
        task = FreeJunctionTask()
        image = fork_frame(block_left=True)
        now = 1.0
        for index in range(6):
            task.step(FramePacket(image, index + 1, now), now)
            now += 0.05
        self.assertTrue(task.active)
        update = task.step(None, now)
        self.assertIs(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion.forward, 0.0)

    def test_step_returns_immediately(self):
        """step() 必须立刻返回（骨架 max_step_seconds = 0.02s）。"""
        task = FreeJunctionTask()
        image = fork_frame(block_left=True)
        import time as _time

        now = 1.0
        task.step(FramePacket(image, 1, now), now)      # 预热
        started = _time.perf_counter()
        for index in range(60):
            now += 0.05
            task.step(FramePacket(image, index + 2, now), now)
        elapsed = (_time.perf_counter() - started) / 60.0
        self.assertLess(elapsed, 0.02, "平均每帧 %.4fs，太慢了" % elapsed)


class ReArmTests(unittest.TestCase):
    def test_does_not_retrigger_while_the_junction_is_still_in_view(self):
        task = FreeJunctionTask()
        image = fork_frame(block_left=True)
        updates, now = run_task(task, image)
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        after = [task.step(FramePacket(image, 500 + i, now + i * 0.05), now + i * 0.05)
                 for i in range(30)]
        self.assertTrue(all(u.status is TaskStatus.NOT_TRIGGERED for u in after))

    def test_can_handle_another_junction_after_this_one_clears(self):
        task = FreeJunctionTask()
        image = fork_frame(block_left=True)
        updates, now = run_task(task, image)
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        plain = line_frame()
        for index in range(60):                          # 60 帧 × 0.05s = 3s > 冷却 2s
            now += 0.05
            task.step(FramePacket(plain, 600 + index, now), now)
        again = []
        for index in range(8):
            now += 0.05
            again.append(task.step(FramePacket(image, 700 + index, now), now))
        self.assertIn(TaskStatus.RUNNING, [u.status for u in again], "下一个岔路应该还能接管")

    def test_reset_returns_to_idle(self):
        task = FreeJunctionTask()
        run_task(task, fork_frame(block_left=True))
        task.reset()
        self.assertIs(task.state, JunctionState.IDLE)
        self.assertIsNone(task.chosen_branch)
        self.assertIsNone(task.last_detection)


class HarnessIntegrationTests(unittest.TestCase):
    """把模块接进真实的 LineFollower + TaskCoordinator + MotionOutput 跑一遍。"""

    def _feed(self, harness, image, now):
        harness.sequence += 1
        packet = FramePacket(image, harness.sequence, now)
        return harness.coordinator.step(packet, now)

    def test_takeover_completes_and_hands_back_to_the_line(self):
        harness = TaskHarness(task=FreeJunctionTask())
        harness.start_line(now=1.0)
        self.assertEqual(harness.owner, "line")

        now = 1.05
        took_over = False
        finished = None
        for _ in range(200):
            decision = self._feed(harness, fork_frame(block_left=True), now)
            now += 0.05
            if harness.task_name == "free_junction":
                took_over = True
            if decision.task_update is not None and decision.task_update.status in (
                TaskStatus.COMPLETED,
                TaskStatus.FAILED,
            ):
                finished = decision.task_update.status
                break
        self.assertTrue(took_over, "岔路帧应该让模块接管")
        self.assertIs(finished, TaskStatus.COMPLETED)

        for _ in range(12):                      # 交回巡线
            self._feed(harness, line_frame(), now)
            now += 0.05
            if harness.owner == "line":
                break
        self.assertEqual(harness.owner, "line")
        self.assertIsNone(harness.task_name)

    def test_no_takeover_on_plain_line_frames(self):
        harness = TaskHarness(task=FreeJunctionTask())
        harness.start_line(now=1.0)
        now = 1.05
        for index in range(20):
            self._feed(harness, line_frame(x=320 + (index % 5) * 5), now)
            now += 0.05
        self.assertEqual(harness.owner, "line")
        self.assertIsNone(harness.task_name)


if __name__ == "__main__":
    unittest.main()
