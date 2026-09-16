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
    BLOCKAGE_BOTH,
    BLOCKAGE_LEFT,
    BLOCKAGE_NONE,
    Branch,
    FreeJunctionConfig,
    FreeJunctionDetector,
    FreeJunctionTask,
    JunctionState,
    VehicleDetector,
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
# 合成帧与真机同坐标系：360x640 BGR，蓝色胶带 HSV 约 (120,255,255)。
# 默认 ROI 是 x 6%~94%、y 54%~96% → x 38~601、y 194~345。
#
# v2/v3 的判据（见 free_junction.py 的 A5/A6）：
#   "拥堵" = 那条分支的走廊里停着一辆**大疆 RoboMaster S1 / EP 小车**。
#   判据就是这辆车的样貌：**深色车体 + 高饱和彩色装甲/灯**（真车实测：
#   车 深色 0.61~0.70 / 彩色 0.25~0.34，空地彩色 0.000）。
# 所以下面的"车"都画成**深色车体 + 两侧彩色装甲**，而不是 v1 那种一片暗色块。
#
# 覆盖（对应 TASKS.md 的"三类场景各自触发/完成/失败可说明"）：
#   [x] 岔路判据：与"只有一条线"、与"被竖直噪声切开的粗带"区分；远的/偏的岔路也要能认出来
#   [x] 一侧有车 → 走另一侧；两侧都有车 → 停车报失败；两侧都没车 → 不接管
#   [x] 固定规则 / 兜底边
#   [x] 动作有限、有超时、有限幅；RUNNING 不会中途改口
#   [x] 完成后交回控制权，不原地反复触发；**被迫结束后也不许立刻重来**（实车报告的 bug）
#   [x] 与绿灯岔路的分工：本模块只认"车堵路"，代码里没有任何灯色逻辑
# ============================================================================

BLUE = (255, 0, 0)          # BGR 里的纯蓝
FLOOR = 210                 # 浅色地面
CAR_BODY = (55, 55, 55)     # 车体：深色、硬边缘
CAR_TOP = (150, 150, 150)   # 顶盖亮条
CAR_ARMOR_RED = (0, 0, 255)    # 装甲/灯：高饱和彩色（S1/EP 的特征之一）
CAR_ARMOR_GREEN = (0, 255, 0)  # 另一种装甲色（故意不用蓝：蓝是胶带的颜色）

#: 停在左/右分支上的"车"（画面坐标）。尺寸**按真车比例**来：
#: 2026-09-16 真车画面里那辆车在检测区域里约占 宽0.39 / 高0.36，这里取 120x100
#: （检测区域约 282x273）→ 比例接近，才测得出闸门该不该过。
CAR_LEFT_BOX = (150, 150, 270, 250)
CAR_RIGHT_BOX = (370, 150, 490, 250)


def draw_car(image, box):
    """画一辆"停着的同型小车"：**深色车体 + 两侧高饱和彩色装甲**。

    这两条正是 2026-09-16 从真车画面上量出来的 S1/EP 特征
    （车：深色 0.61~0.70、高饱和彩色 0.25~0.34；空地：0.000）。
    装甲故意用**红/绿**（不用蓝）：蓝色是胶带的颜色，会被判据当成胶带抠掉。
    """
    x0, y0, x1, y1 = box
    cv2.rectangle(image, (x0, y0), (x1, y1), CAR_BODY, -1)
    cv2.rectangle(image, (x0 + 2, y0 + 2), (x0 + 14, y1 - 2), CAR_ARMOR_RED, -1)      # 左装甲
    cv2.rectangle(image, (x1 - 14, y0 + 2), (x1 - 2, y1 - 2), CAR_ARMOR_GREEN, -1)    # 右装甲


def fork_frame(car_left=False, car_right=False, stem_top=300, tips=(200, 440), x=320):
    """合成岔路帧：主干 + 两条向上张开的分支，可选在某一侧停一辆车。"""
    image = np.full((360, 640, 3), FLOOR, np.uint8)
    cv2.rectangle(image, (x - 12, stem_top), (x + 12, 350), BLUE, -1)
    cv2.line(image, (x, stem_top), (tips[0], 190), BLUE, 18)
    cv2.line(image, (x, stem_top), (tips[1], 190), BLUE, 18)
    if car_left:
        draw_car(image, CAR_LEFT_BOX)
    if car_right:
        draw_car(image, CAR_RIGHT_BOX)
    return image


def noise_split_frame():
    """一条很宽的带子被"竖直噪声"等宽切开：上下间距完全一样，不是岔路。"""
    image = np.full((360, 640, 3), FLOOR, np.uint8)
    cv2.rectangle(image, (290, 194), (350, 350), BLUE, -1)
    cv2.rectangle(image, (305, 194), (335, 350), FLOOR, -1)
    return image


def run_with_states(task, image, frames=400, dt=0.05, start=1.0):
    """按假时钟把同一张图喂进去，记下 (状态, 更新, 时间)，直到出现终态。"""
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


def run_task(task, image, frames=400, dt=0.05, start=1.0):
    records = run_with_states(task, image, frames=frames, dt=dt, start=start)
    updates = [update for _state, update, _now in records]
    return updates, (records[-1][2] if records else start)


def drive_sequence(task, phases, dt=0.05, start=1.0):
    """按顺序喂不同的画面：phases = [(图, 帧数), ...]，记下 (状态, 更新, 时间)。"""
    records = []
    now = start
    sequence = 1
    for image, count in phases:
        for _ in range(count):
            update = task.step(FramePacket(image, sequence, now), now)
            records.append((task.state, update, now))
            sequence += 1
            now += dt
            if update.status in (TaskStatus.COMPLETED, TaskStatus.FAILED):
                return records
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
        detection = FreeJunctionDetector().detect(fork_frame(car_left=True))
        self.assertTrue(detection.valid)
        self.assertGreater(detection.separation_px, 0)
        self.assertGreater(detection.divergence, 0.0)
        self.assertLess(detection.left_x, detection.right_x)
        self.assertGreater(detection.split_row, 194)
        self.assertLess(detection.split_row, 346)

    def test_a_single_line_is_not_a_fork(self):
        detector = FreeJunctionDetector()
        self.assertFalse(detector.detect(line_frame()).valid)
        self.assertFalse(detector.detect(np.full((360, 640, 3), FLOOR, np.uint8)).valid)

    def test_a_wide_tape_cut_by_noise_is_not_a_fork(self):
        """2026-09 实车画面里一条带子常被噪点切成几段（6 号报告里也提到）。

        这里刻意造成"最左段 + 最右段"够开、空隙也够大，但**上下等宽**：
        没有张开趋势，所以必须判成不是岔路。
        """
        self.assertFalse(FreeJunctionDetector().detect(noise_split_frame()).valid)

    def test_a_far_fork_is_still_detected(self):
        """v1 的漏洞：只看 ROI 50% 处那 7 行 → 岔路远一点就整个漏检。

        这里分叉点靠近 ROI 顶部，50% 处那一行已经只剩主干，v2 靠整列扫描仍要认出来。
        """
        far = fork_frame(stem_top=250, tips=(200, 440))
        self.assertTrue(FreeJunctionDetector().detect(far).valid)

    def test_an_off_center_fork_is_still_detected(self):
        """v1 的另一个漏洞：顶部检查按 ROI 中线分左右 → 车没对正就漏检。

        这里整条岔路偏在画面左侧，v2 必须照样认出来。
        """
        off = fork_frame(tips=(60, 300), x=180)
        detection = FreeJunctionDetector().detect(off)
        self.assertTrue(detection.valid)
        self.assertLess(detection.split_x, 320)

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


class VehicleDetectorTests(unittest.TestCase):
    def test_a_parked_car_is_found_in_its_corridor(self):
        task = FreeJunctionTask()
        image = fork_frame(car_left=True)
        task.step(FramePacket(image, 1, 1.0), 1.0)
        self.assertEqual(task.last_blockage.reading, BLOCKAGE_LEFT)
        self.assertGreater(task.last_blockage.left_evidence, 0.0)

    def test_empty_corridor_reports_no_vehicle(self):
        detector = VehicleDetector()
        empty = np.full((120, 200, 3), FLOOR, np.uint8)
        blocked, score, box = detector.detect(empty, None)
        self.assertFalse(blocked)
        self.assertEqual(score, 0.0)
        self.assertIsNone(box)

    def test_a_far_away_blob_is_rejected_by_the_distance_gate(self):
        """走廊最上沿那一小块不算"不远处"（A5）。"""
        region = np.full((200, 200, 3), FLOOR, np.uint8)
        cv2.rectangle(region, (60, 5), (140, 40), CAR_BODY, -1)
        blocked, _score, _box = VehicleDetector().detect(region, None)
        self.assertFalse(blocked)

    def test_a_clean_tape_is_not_a_vehicle(self):
        """**2026-09-16 真车 bug 的回归测试**：干净的蓝带不许被当成一辆车。

        当时的现象：左侧分支停着一辆车（真堵），右侧空着；模块却报"右侧有车"，
        于是选了左边那条堵的。根因是胶带边缘的抗锯齿像素不在 HSV 掩码里，
        抹不干净 → Canny 把胶带自己的轮廓当成"硬边物体" → 外接框过了形状判据。
        修法：找车之前把蓝带按 `tape_clear_px` 膨胀后再抹平（见 structure_mask）。
        """
        region = np.full((150, 280, 3), FLOOR, np.uint8)
        cv2.line(region, (40, 149), (200, 0), BLUE, 16)          # 斜穿走廊的一条胶带
        region = cv2.GaussianBlur(region, (5, 5), 0)             # 真实相机感：边缘是软的
        detector = FreeJunctionDetector()
        line = detector.blue_mask(region)
        self.assertGreater(float(line.mean()), 0.0, "掩码应该有东西")
        blocked, score, box = VehicleDetector().detect(region, line)
        self.assertFalse(
            blocked,
            "干净的蓝带自己就被判成车了（真车上会因此选错边）：证据=%.3f 框=%s"
            % (score, box),
        )

    def test_a_plain_dark_block_is_not_a_robot(self):
        """**要求 2 的回归测试**：光"深色一大块"不算堵，必须是 S1/EP 那种
        "深色车体 + 高饱和彩色装甲/灯"。

        真车实测：空地的高饱和像素占比是 **0.000**，花岗岩地砖、影子、暗墙都进不来。
        """
        settings = FreeJunctionConfig()
        detector = FreeJunctionDetector(settings)
        vehicle = VehicleDetector(settings)
        region = np.full((150, 280, 3), FLOOR, np.uint8)
        cv2.rectangle(region, (70, 40), (210, 120), CAR_BODY, -1)     # 只有深色，没有彩色
        blocked, score, box = vehicle.detect(region, detector.blue_mask(region))
        self.assertFalse(blocked, "只有深色、没有彩色装甲，不该判成车: %s %s" % (score, box))

        # 同一块地方，加上两条高饱和彩色装甲 → 应该判成车
        draw_car(region, (70, 40, 210, 120))
        blocked2, score2, box2 = vehicle.detect(region, detector.blue_mask(region))
        self.assertTrue(blocked2, "深色车体 + 彩色装甲应该判成车，实际 %s %s" % (score2, box2))

    def test_skin_like_blob_is_rejected(self):
        """手/皮肤色占比过半的候选丢掉（同 obstacle.py 的现场教训）。"""
        region = np.full((200, 200, 3), FLOOR, np.uint8)
        cv2.rectangle(region, (60, 120), (150, 190), (120, 150, 200), -1)  # 肤色 BGR
        blocked, _score, _box = VehicleDetector().detect(region, None)
        self.assertFalse(blocked)


class ChoosingBranchTests(unittest.TestCase):
    def test_car_on_the_left_takes_the_right_branch(self):
        task = FreeJunctionTask()
        records = run_with_states(task, fork_frame(car_left=True))
        updates = [update for _state, update, _now in records]
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.RIGHT)
        self.assertIn("left branch blocked", task.last_reason)
        yaws = turn_yaws(records)
        self.assertTrue(yaws, "应该进入转向阶段")
        self.assertTrue(all(yaw > 0 for yaw in yaws), yaws)

    def test_car_on_the_right_takes_the_left_branch(self):
        task = FreeJunctionTask()
        records = run_with_states(task, fork_frame(car_right=True))
        updates = [update for _state, update, _now in records]
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.LEFT)
        yaws = turn_yaws(records)
        self.assertTrue(yaws, "应该进入转向阶段")
        self.assertTrue(all(yaw < 0 for yaw in yaws), yaws)

    def test_no_vehicle_means_this_is_not_our_scenario(self):
        """两条分支都没车 → 不接管，把控制权留给巡线/6 号（A6）。

        这条同时是"不误触发"的加强版：v1 在这种情况下会接管然后停车。
        """
        task = FreeJunctionTask()
        updates, _now = run_task(task, fork_frame(), frames=60)
        self.assertTrue(all(u.status is TaskStatus.NOT_TRIGGERED for u in updates))
        self.assertEqual(task.last_blockage.reading, BLOCKAGE_NONE)
        self.assertIn("no vehicle", task.last_message)

    def test_both_blocked_fails_instead_of_guessing(self):
        task = FreeJunctionTask()
        updates, _now = run_task(task, fork_frame(car_left=True, car_right=True))
        self.assertIs(updates[-1].status, TaskStatus.FAILED)
        self.assertEqual(task.last_blockage.reading, BLOCKAGE_BOTH)
        self.assertIn("both branches blocked", task.last_message)
        self.assertEqual(updates[-1].motion.forward, 0.0)
        self.assertEqual(updates[-1].motion.yaw, 0.0)

    def test_fallback_branch_is_used_when_both_are_blocked(self):
        settings = FreeJunctionConfig(fallback_branch="left")
        task = FreeJunctionTask(settings=settings)
        updates, _now = run_task(task, fork_frame(car_left=True, car_right=True))
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.LEFT)
        self.assertIn("fallback", task.last_reason)

    def test_fixed_rule_ignores_the_vehicle_reading(self):
        settings = FreeJunctionConfig(decision_rule="fixed", fixed_branch="right")
        task = FreeJunctionTask(settings=settings)
        updates, _now = run_task(task, fork_frame(car_left=True))
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.RIGHT)

    def test_fixed_rule_without_a_side_fails(self):
        settings = FreeJunctionConfig(decision_rule="fixed", fixed_branch=None)
        task = FreeJunctionTask(settings=settings)
        updates, _now = run_task(task, fork_frame(car_left=True))
        self.assertIs(updates[-1].status, TaskStatus.FAILED)
        self.assertIn("without a side", task.last_message)

    def test_unknown_decision_rule_fails_instead_of_guessing(self):
        settings = FreeJunctionConfig(decision_rule="vibes")
        task = FreeJunctionTask(settings=settings)
        updates, _now = run_task(task, fork_frame(car_left=True))
        self.assertIs(updates[-1].status, TaskStatus.FAILED)

    def test_messages_carry_the_reading_for_the_run_log(self):
        """实车报告提到运行记录里拿不到模块给的原因；v2 把读数写进 message。"""
        task = FreeJunctionTask()
        records = run_with_states(task, fork_frame(car_left=True))
        messages = [update.message for _state, update, _now in records]
        self.assertTrue(any("vehicle L=" in message for message in messages), messages[:3])
        self.assertTrue(any("left branch blocked" in message for message in messages))


class OwnershipAndTimingTests(unittest.TestCase):
    def test_running_never_reverts_to_not_triggered(self):
        """红线 8：一旦说要接管，就必须一路 RUNNING 到 COMPLETED / FAILED。"""
        task = FreeJunctionTask()
        updates, _now = run_task(task, fork_frame(car_left=True), frames=30)
        started = False
        for update in updates:
            if update.status is TaskStatus.RUNNING:
                started = True
                continue
            if started:
                self.assertIn(update.status, (TaskStatus.COMPLETED, TaskStatus.FAILED))
        self.assertTrue(started)

    def test_waiting_for_the_criterion_stands_still(self):
        """等判据期间必须是零速度（A7）：不猜、也不往前冲。"""
        settings = FreeJunctionConfig(decide_timeout=0.4, max_task_seconds=2.0)
        task = FreeJunctionTask(settings=settings)
        records = run_with_states(task, fork_frame(car_left=True, car_right=True), frames=6)
        running = [u for _s, u, _n in records if u.status is TaskStatus.RUNNING]
        self.assertTrue(running, "两条都有车时应该先接管再判")
        for update in running:
            self.assertEqual(update.motion.forward, 0.0)
            self.assertEqual(update.motion.yaw, 0.0)

    def test_commands_stay_inside_limits(self):
        task = FreeJunctionTask()
        updates, _now = run_task(task, fork_frame(car_left=True))
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
        settings = FreeJunctionConfig(max_task_seconds=0.5, approach_seconds_max=10.0)
        task = FreeJunctionTask(settings=settings)
        updates, _now = run_task(task, fork_frame(car_left=True), frames=80)
        self.assertIs(updates[-1].status, TaskStatus.FAILED)
        self.assertIn("time budget", task.last_message)
        self.assertEqual(updates[-1].motion.forward, 0.0)

    def test_lost_line_while_deciding_stops_the_car(self):
        """原地等判据的时候整条线不见了 → 停车认输（这一段必须看得见线）。"""
        settings = FreeJunctionConfig(decide_timeout=2.0)
        task = FreeJunctionTask(settings=settings)
        blank = np.full((360, 640, 3), FLOOR, np.uint8)
        records = drive_sequence(
            task, [(fork_frame(car_left=True, car_right=True), 10), (blank, 20)]
        )
        updates = [update for _state, update, _now in records]
        self.assertIs(updates[-1].status, TaskStatus.FAILED)
        self.assertIn("line lost", task.last_message)
        self.assertEqual(updates[-1].motion.forward, 0.0)

    def test_lost_line_during_the_turn_does_not_stop_the_car(self):
        """**2026-09-16 真车 bug 的回归测试**：转弯/出岔路时胶带跑出 ROI 不算丢线。

        当时的现象：两次都在 TURN/EXIT 里走了 0.8 秒就 FAILED 停在岔路口中间。
        车头一转，胶带本来就可能不在画面里了 —— 那不是"线没了"，不该停车。
        """
        task = FreeJunctionTask()
        blank = np.full((360, 640, 3), FLOOR, np.uint8)
        records = drive_sequence(task, [(fork_frame(car_left=True), 30), (blank, 80)])
        updates = [update for _state, update, _now in records]
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertNotIn("line lost", task.last_message)

    def test_turn_ends_early_once_the_tape_is_back_in_the_centre(self):
        """线已经回到车头正前方 → 转向提前收工，不在原地多拧几十度。"""
        task = FreeJunctionTask()
        records = drive_sequence(
            task, [(fork_frame(car_left=True), 30), (line_frame(), 40)]
        )
        turn_at = exit_at = None
        for state, _update, now in records:
            if state is JunctionState.TURN and turn_at is None:
                turn_at = now
            if state is JunctionState.EXIT and exit_at is None:
                exit_at = now
        self.assertIsNotNone(turn_at, "应该进入转向阶段")
        self.assertIsNotNone(exit_at, "应该进入出岔路阶段")
        self.assertLess(
            exit_at - turn_at,
            task.settings.turn_seconds,
            "线已经回中央了，还在按时间把角度转满",
        )

    def test_missing_frame_while_owning_control_fails(self):
        task = FreeJunctionTask()
        image = fork_frame(car_left=True)
        now = 1.0
        for index in range(8):
            task.step(FramePacket(image, index + 1, now), now)
            now += 0.05
        self.assertTrue(task.active)
        update = task.step(None, now)
        self.assertIs(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion.forward, 0.0)

    def test_step_returns_immediately(self):
        """step() 必须立刻返回（骨架 max_step_seconds = 0.02s）。"""
        import time as _time

        task = FreeJunctionTask()
        image = fork_frame(car_left=True)
        now = 1.0
        task.step(FramePacket(image, 1, now), now)      # 预热
        started = _time.perf_counter()
        for index in range(60):
            now += 0.05
            task.step(FramePacket(image, index + 2, now), now)
        elapsed = (_time.perf_counter() - started) / 60.0
        self.assertLess(elapsed, 0.02, "平均每帧 %.4fs，太慢了" % elapsed)


class ForcedEndTests(unittest.TestCase):
    """2026-09-15 实车报告：被迫结束后模块又立刻接管，同一次运行 20 次接管、0 次收尾。

    协调器在被迫结束（人工 SPACE / 视频中断 / 协调器超时）时会调用 `reset()`，
    正常完成不调用。所以 `reset()` 之后**不许**马上又能触发。
    """

    def test_reset_arms_the_rearm_gate(self):
        task = FreeJunctionTask()
        image = fork_frame(car_left=True)
        now = 1.0
        statuses = []
        for index in range(10):
            statuses.append(task.step(FramePacket(image, index + 1, now), now).status)
            now += 0.05
        self.assertIn(TaskStatus.RUNNING, statuses, "前几帧应该已经接管")

        task.reset()                     # 协调器被迫结束时就是这么调的
        self.assertIs(task.state, JunctionState.IDLE)
        self.assertEqual(task.interruptions, 1)

        after = []
        for index in range(20):          # 车还停在同一个岔路口，画面没变
            after.append(task.step(FramePacket(image, 200 + index, now), now).status)
            now += 0.05
        self.assertTrue(
            all(status is TaskStatus.NOT_TRIGGERED for status in after),
            "被迫结束之后立刻又接管 = 实车报告里的反复触发",
        )

    def test_reset_then_cleared_junction_can_trigger_again(self):
        """封锁不是永久的：车开过岔路、画面里没有岔路了，之后还能正常干活。"""
        task = FreeJunctionTask()
        image = fork_frame(car_left=True)
        plain = line_frame()
        now = 1.0
        for index in range(10):
            task.step(FramePacket(image, index + 1, now), now)
            now += 0.05
        task.reset()
        for index in range(60):          # 60 帧 × 0.05s = 3s > 冷却 2s
            task.step(FramePacket(plain, 100 + index, now), now)
            now += 0.05
        again = []
        for index in range(8):
            again.append(task.step(FramePacket(image, 300 + index, now), now).status)
            now += 0.05
        self.assertIn(TaskStatus.RUNNING, again)

    def test_reset_clears_motion_and_state(self):
        task = FreeJunctionTask()
        run_task(task, fork_frame(car_left=True))
        task.reset()
        self.assertIs(task.state, JunctionState.IDLE)
        self.assertIsNone(task.chosen_branch)
        self.assertIsNone(task.last_detection)


class ReArmTests(unittest.TestCase):
    def test_does_not_retrigger_while_the_junction_is_still_in_view(self):
        task = FreeJunctionTask()
        image = fork_frame(car_left=True)
        updates, now = run_task(task, image)
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        after = [task.step(FramePacket(image, 500 + i, now + i * 0.05), now + i * 0.05)
                 for i in range(30)]
        self.assertTrue(all(u.status is TaskStatus.NOT_TRIGGERED for u in after))

    def test_can_handle_another_junction_after_this_one_clears(self):
        task = FreeJunctionTask()
        image = fork_frame(car_left=True)
        updates, now = run_task(task, image)
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        plain = line_frame()
        for index in range(60):                          # 60 帧 × 0.05s = 3s > 冷却 2s
            now += 0.05
            task.step(FramePacket(plain, 600 + index, now), now)
        again = []
        for index in range(8):
            now += 0.05
            again.append(task.step(FramePacket(image, 700 + index, now), now).status)
        self.assertIn(TaskStatus.RUNNING, again, "下一个岔路应该还能接管")


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
        for _ in range(300):
            decision = self._feed(harness, fork_frame(car_left=True), now)
            now += 0.05
            if harness.task_name == "free_junction":
                took_over = True
            if decision.task_update is not None and decision.task_update.status in (
                TaskStatus.COMPLETED,
                TaskStatus.FAILED,
            ):
                finished = decision.task_update.status
                break
        self.assertTrue(took_over, "有车的岔路帧应该让模块接管")
        self.assertIs(finished, TaskStatus.COMPLETED)

        for _ in range(20):                      # 交回巡线
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

    def test_no_takeover_on_a_fork_without_a_vehicle(self):
        """岔路本身不是接管理由：两条分支都没车就不插手（A6）。"""
        harness = TaskHarness(task=FreeJunctionTask())
        harness.start_line(now=1.0)
        now = 1.05
        for _ in range(40):
            self._feed(harness, fork_frame(), now)
            now += 0.05
        self.assertEqual(harness.owner, "line")
        self.assertIsNone(harness.task_name)


if __name__ == "__main__":
    unittest.main()
