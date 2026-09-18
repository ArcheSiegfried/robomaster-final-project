"""无拥堵岔路（WP6b / Issue #6）。

契约测试在前，成员用例区在后。骨架已经把这个文件登记进 task_registry，
你只要替换上面的实现即可，不需要改 main.py。
"""

import dataclasses
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
    BLOCKAGE_RIGHT,
    Branch,
    ForkDetection,
    FreeJunctionConfig,
    FreeJunctionDetector,
    FreeJunctionTask,
    JunctionState,
    VehicleDetector,
    _fork_rows,
    _separated_runs,
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


def fork_frame(car_left=False, car_right=False, stem_top=300, tips=(200, 440), x=320,
               floor=None):
    """合成岔路帧：主干 + 两条向上张开的分支，可选在某一侧停一辆车。

    `floor` 可以换地面灰度（默认 `FLOOR`）—— 用来验证判据**与场地无关**：
    考试场地是浅灰地面，练习场地是深色水磨石，两者都要成立。
    """
    image = np.full((360, 640, 3), FLOOR if floor is None else floor, np.uint8)
    cv2.rectangle(image, (x - 12, stem_top), (x + 12, 350), BLUE, -1)
    cv2.line(image, (x, stem_top), (tips[0], 190), BLUE, 18)
    cv2.line(image, (x, stem_top), (tips[1], 190), BLUE, 18)
    if car_left:
        draw_car(image, CAR_LEFT_BOX)
    if car_right:
        draw_car(image, CAR_RIGHT_BOX)
    return image


#: **没有彩色装甲**的深色车（2026-09-18 新场地那辆车的合成版）。框压在支路胶带上，
#: 而且底边别太靠下（太靠下会被当成我们自己的车头/影子）。
DARK_CAR_LEFT_BOX = (185, 185, 275, 245)
DARK_CAR_RIGHT_BOX = (355, 185, 445, 245)
#: 一块"没有胶带通向它"的深色块（模拟墙裙/家具）：位置远离那条支路的胶带。
DARK_BLOB_NO_TAPE_BOX = (30, 185, 120, 245)


def draw_dark_car(image, box):
    """画一辆**只有深色车体、没有任何高饱和彩色**的车。

    这正是 2026-09-18 新场地实车的样子：车框内 S>=100 只占 **0.004**（旧场地那辆
    是 0.162），于是"深色 + 彩色"的 S1/EP 判据整车找不到 → 不接管 → 巡线自己把车
    开进那条堵着的支路。这里用它来锁住新的"不看颜色"的兜底判据。
    """
    x0, y0, x1, y1 = box
    cv2.rectangle(image, (x0, y0), (x1, y1), CAR_BODY, -1)


class ExamObstacleTests(unittest.TestCase):
    """**考试摆法**回归：关机/只剩底盘的 EP 停在岔路蓝线上、离岔路口约 1 米。

    用户 2026-09-18 更正：不是"车身上有蓝线"，而是**车停在蓝线上**；
    而且障碍车**不一定开机、不一定有视觉标签**，甚至可能只剩底盘（云台顶缺失）。
    所以官方 SDK 可能一次都认不出来（练习场地 `robots_in_snapshot 0` 就是这么回事），
    判据必须能只靠"一个压在蓝线上的深色物体"成立 —— 这一组用例锁住这件事，
    并且**不许**依赖练习场地量出来的固定亮度/颜色。
    """

    def reading(self, image, settings=None):
        task = FreeJunctionTask(settings) if settings is not None else FreeJunctionTask()
        fork, _roi, _line, rect = task.detector.analyze(image)
        self.assertTrue(fork.valid, "这一帧应该能判出岔路")
        return task._read_blockage(fork, image, rect, 1.0)

    def test_powered_off_chassis_on_the_tape_is_seen_on_any_floor(self):
        """同一个障碍物、三种地面亮度（很亮的浅灰 / 浅灰 / 深色）都必须认出来。"""
        for floor in (235, 210, 120):
            image = fork_frame(floor=floor)
            draw_powered_off_obstacle(image, EXAM_OBSTACLE_LEFT)
            reading = self.reading(image)
            self.assertEqual(reading.reading, BLOCKAGE_LEFT,
                             "地面亮度 %d 时漏检：%s" % (floor, reading.describe()))
            self.assertIsNotNone(reading.left_box)

    def test_a_bare_chassis_without_visible_tracks_is_still_seen(self):
        """连履带都不明显、只有一块深色底盘 → 也要认（顶部缺失/低视角的情形）。"""
        image = fork_frame(floor=235)
        draw_powered_off_obstacle(image, EXAM_OBSTACLE_LEFT, wheels=False)
        self.assertEqual(self.reading(image).reading, BLOCKAGE_LEFT)

    def test_the_obstacle_is_seen_across_the_exam_distance_range(self):
        """考试距离 0.9~1.1 米，判据的有效范围实测到约 1.3 米（更远就超出设计范围）。"""
        for label, box in (
            ("约 1.0 米", (190, 188, 280, 233)),
            ("约 1.1 米", (195, 190, 275, 230)),
            ("约 1.3 米", EXAM_OBSTACLE_FAR_LEFT),
        ):
            image = fork_frame(floor=235)
            draw_powered_off_obstacle(image, box)
            self.assertEqual(self.reading(image).reading, BLOCKAGE_LEFT,
                             "%s 的车漏检了" % label)

    def test_the_side_is_reported_correctly_and_empty_junctions_are_quiet(self):
        right = fork_frame(floor=235)
        draw_powered_off_obstacle(right, EXAM_OBSTACLE_RIGHT)
        self.assertEqual(self.reading(right).reading, BLOCKAGE_RIGHT)
        empty = fork_frame(floor=235)
        self.assertEqual(self.reading(empty).reading, BLOCKAGE_NONE,
                         "空岔路不该报拥堵")

    def test_the_official_sdk_reading_still_wins_when_it_exists(self):
        """障碍车万一被官方识别认出来（开机/带视觉标签），官方读数优先、且不看尺寸。"""
        task = FreeJunctionTask()
        image = fork_frame(floor=235)
        task.update_robot_observations([(0.22, 0.45, 0.10, 0.06)], observed_at=1.0)
        fork, _roi, _line, rect = task.detector.analyze(image)
        reading = task._read_blockage(fork, image, rect, 1.0)
        self.assertEqual(reading.reading, BLOCKAGE_LEFT)
        self.assertEqual(reading.source, "sdk")


class ColorlessCarTests(unittest.TestCase):
    """2026-09-18 新场地：停着的车**完全没有彩色装甲**，只能靠"深色块挡在胶带前"认出来。

    那两次实车（112922 / 113011）都是：岔路判出来了（`valid=True`），但"深色 + 彩色"
    的 S1/EP 判据整车找不到（掩码 0 像素）→ `reading=none` → 不接管 → 巡线自己把车
    开进左边那条堵着的支路。这里的合成帧就按那个样子造。
    """

    def reading(self, image, settings=None):
        task = FreeJunctionTask(settings) if settings is not None else FreeJunctionTask()
        fork, _roi, _line, rect = task.detector.analyze(image)
        self.assertTrue(fork.valid, "这一帧应该能判出岔路")
        return task._read_blockage(fork, image, rect, 1.0)

    def test_a_colorless_car_on_the_left_branch_is_seen(self):
        image = fork_frame()
        draw_dark_car(image, DARK_CAR_LEFT_BOX)
        reading = self.reading(image)
        self.assertEqual(reading.reading, BLOCKAGE_LEFT,
                         "没有彩色装甲的车也必须认出来：%s" % reading.describe())
        self.assertIsNotNone(reading.left_box, "要给出车的框（得分快照要画它）")
        self.assertGreater(reading.left_evidence, 0.0)

    def test_a_colorless_car_on_the_right_branch_is_seen(self):
        image = fork_frame()
        draw_dark_car(image, DARK_CAR_RIGHT_BOX)
        reading = self.reading(image)
        self.assertEqual(reading.reading, BLOCKAGE_RIGHT, reading.describe())
        self.assertIsNotNone(reading.right_box)

    def test_a_dark_blob_without_tape_under_it_is_not_a_car(self):
        """没有胶带通向它 → 不是"堵在支路上的车"（墙裙、家具那类深色块）。"""
        image = fork_frame()
        draw_dark_car(image, DARK_BLOB_NO_TAPE_BOX)
        reading = self.reading(image)
        self.assertEqual(reading.reading, BLOCKAGE_NONE,
                         "离胶带很远的深色块不该算拥堵：%s" % reading.describe())

    def test_our_own_dark_body_at_the_bottom_is_not_a_car(self):
        """判据带最下沿那一块是我们自己的车头/影子（2026-09-18 实测就是这么误报的）。"""
        image = fork_frame()
        cv2.rectangle(image, (200, 265), (290, 305), CAR_BODY, -1)
        reading = self.reading(image)
        self.assertEqual(reading.reading, BLOCKAGE_NONE, reading.describe())

    def test_a_one_frame_criterion_flicker_does_not_kill_the_decision(self):
        """判据闪断一两帧不能把整个岔路判死（2026-09-18 run_121100 的教训）。

        那次接管后 0.2~0.3 s 读数从"有车"闪成"没车"，旧版 0.25 s 超时立刻 FAILED，
        整条岔路白跑；现在 `decide_hold_seconds` 内还能用最近一条可用读数继续判。
        """
        task = FreeJunctionTask()
        good = fork_frame()
        draw_dark_car(good, DARK_CAR_LEFT_BOX)
        blank = fork_frame()
        now = 1.0
        for index in range(8):                       # 先让判据成立、接管
            task.step(FramePacket(good, index + 1, now), now)
            now += 0.05
        self.assertTrue(task.active, "应该已经接管")
        # 闪断：两帧读不到车，然后恢复
        for index, image in enumerate([blank, blank, good, blank, good]):
            update = task.step(FramePacket(image, 100 + index, now), now)
            now += 0.05
            self.assertIsNot(update.status, TaskStatus.FAILED,
                             "闪断不该判失败：%s" % update.message)
        # 继续走完（APPROACH/TURN/EXIT 需要时间）
        for index in range(120):
            update = task.step(FramePacket(good, 200 + index, now), now)
            now += 0.05
            if update.status in (TaskStatus.COMPLETED, TaskStatus.FAILED):
                break
        self.assertIs(update.status, TaskStatus.COMPLETED,
                      "闪断之后应该照常完成：%s" % update.message)
        self.assertIs(task.chosen_branch, Branch.RIGHT)

    def test_a_vanishing_criterion_does_not_fail_the_task(self):
        """接管后判据立刻消失：**不能 FAILED**，要用刚才那条可用读数继续走。

        这就是 run_121100 的现场：接管后 0.2~0.3 s 读数从"有车"闪成"没车"，
        旧版 0.25 s 超时立刻 FAILED（报告里 `criterion unavailable: no vehicle on
        either branch`），整个岔路白跑。现在 `decide_hold_seconds` 内的可用读数
        会直接把决策推进到 APPROACH，所以任务继续往下走。
        """
        task = FreeJunctionTask()
        good = fork_frame()
        draw_dark_car(good, DARK_CAR_LEFT_BOX)
        blank = fork_frame()
        now = 1.0
        for index in range(8):
            task.step(FramePacket(good, index + 1, now), now)
            now += 0.05
        self.assertTrue(task.active, "应该已经接管")
        for index in range(12):                      # 判据一直读不到
            update = task.step(FramePacket(blank, 100 + index, now), now)
            now += 0.05
            self.assertIsNot(update.status, TaskStatus.FAILED,
                             "判据闪断不该判失败：%s" % update.message)
        self.assertIsNot(task.state, JunctionState.FAILED)
        # 软失败的兜底（判据在窗口内始终读不到才走）也要留着：冷却比硬失败短
        settings = FreeJunctionConfig()
        self.assertLess(settings.soft_rearm_cooldown, settings.rearm_cooldown)

    def test_the_colorless_path_can_be_switched_off(self):
        """`occluder_enabled=False` 时行为回到"只认颜色"（旧场地口径，便于对比）。"""
        settings = dataclasses.replace(FreeJunctionConfig(), occluder_enabled=False)
        image = fork_frame()
        draw_dark_car(image, DARK_CAR_LEFT_BOX)
        self.assertEqual(self.reading(image, settings).reading, BLOCKAGE_NONE)
        # 有彩色装甲的车照旧认得出（颜色判据没被动过）
        colored = fork_frame(car_left=True)
        self.assertEqual(self.reading(colored, settings).reading, BLOCKAGE_LEFT)

    def test_the_criterion_does_not_depend_on_the_venue(self):
        """**换场地也要成立**：浅灰地面 + 光照偏亮时车身不再是"绝对深色"。

        用户明确：考试场地地面是浅灰、和练习场地不一样；障碍车是 EP 小车、
        停在岔路上、车下方有蓝线、离岔路口约 0.9~1.1 米。
        所以判据不能靠"练习场地量出来的固定亮度 90"：
        这里地面用 235（很亮的浅灰），车身用 135（光照下的中灰），
        固定阈值（V<=90）会**整辆漏掉**，而"比自己邻域暗"的自适应取块照样认得出。
        """
        box = (175, 170, 265, 242)          # 约 1 米外的车，压在左支胶带上
        image = fork_frame(floor=235)
        cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), (135, 135, 135), -1)
        # 1) 默认（自适应局部阈值）→ 必须认出来
        reading = self.reading(image)
        self.assertEqual(reading.reading, BLOCKAGE_LEFT,
                         "浅灰地面上的车必须认出来：%s" % reading.describe())
        # 2) 对照：切回"固定亮度 90"的旧口径 → 这一帧必然漏（说明自适应那条真的在起作用）
        fixed = dataclasses.replace(FreeJunctionConfig(), occluder_mask_mode="fixed",
                                    occluder_dark_mode="fixed")
        self.assertEqual(self.reading(image, fixed).reading, BLOCKAGE_NONE,
                         "固定阈值口径在这一帧应该漏掉（这正是换场地会失败的原因）")

    def test_a_car_one_metre_away_on_a_light_floor_is_seen(self):
        """**考试规范的距离**：车停在岔路口外约 1 米（画面里只有近处那辆的一半大）。

        合成帧的地面是浅灰（V=210），和用户说的考试场地（浅灰地面）同量级 ——
        固定亮度阈值（V<=90）在浅灰地面上比在深色地面上更稳，因为车比地面暗得多。
        """
        image = fork_frame()
        draw_far_car(image, FAR_CAR_LEFT_BOX)
        reading = self.reading(image)
        self.assertEqual(reading.reading, BLOCKAGE_LEFT,
                         "1 米外的车也必须认出来：%s" % reading.describe())
        self.assertIsNotNone(reading.left_box)

    def test_the_official_reading_is_independent_of_distance(self):
        """官方 SDK 读数**不看尺寸**：1 米外的车框很小也照样算数（`sdk_or_vision` 默认）。"""
        task = FreeJunctionTask()
        image = fork_frame()
        # 归一化坐标：中心 (0.22, 0.45)，宽高只有 0.10 x 0.06 —— 相当于远处的小车
        task.update_robot_observations([(0.22, 0.45, 0.10, 0.06)], observed_at=1.0)
        fork, _roi, _line, rect = task.detector.analyze(image)
        reading = task._read_blockage(fork, image, rect, 1.0)
        self.assertEqual(reading.reading, BLOCKAGE_LEFT,
                         "官方说左边有车，框小也必须采信：%s" % reading.describe())
        self.assertEqual(reading.source, "sdk")

    def test_the_darkness_rule_and_its_measured_boundary(self):
        """亮度口径的**实测结论**锁在这里（换场地时照这条改）。

        * 绝对阈值 `V<=90`（``occluder_dark_mode="fixed"``）是 126 帧 A/B 里唯一
          三条目标帧都不误判空侧的口径 —— 浅灰地面（考试场地）下车比地面暗得多，
          固定阈值反而最稳；
        * 试过的两种"自适应/相对"口径都**更差**：相对口径（块内亮度 vs 判据带 p60）
          因为判据带里暗背景多、p60 被拉低，车和背景连片 → 三条目标帧**全漏**；
          自适应口径（p60 - 40）在深色地面上会误判空侧。
        """
        settings = FreeJunctionConfig()
        self.assertEqual(settings.occluder_dark_mode, "fixed",
                         "相对/自适应口径实测更差，默认保持固定阈值")
        self.assertEqual(settings.blockage_source, "sdk_or_vision",
                         "默认必须官方优先 —— 换场地时靠它兜底")
        self.assertTrue(settings.occluder_enabled)
        # 1 米外实测深色占比 0.349，门槛不能高过它（否则 1 米外的车整辆被挡掉）
        self.assertGreaterEqual(settings.occluder_dark_min_ratio, 0.25)
        self.assertLessEqual(settings.occluder_dark_min_ratio, 0.40)
        # 连拍两帧一样的读数才算数（压住单帧误报；A/B 里那 1 帧危险就靠它兜）
        self.assertGreaterEqual(settings.blockage_confirm_frames, 2)
        # 判据一时读不到 → 软失败 + 短冷却，别把整个岔路错过
        self.assertLess(settings.soft_rearm_cooldown, settings.rearm_cooldown)
        self.assertGreater(settings.decide_hold_seconds, 0.0)


#: **1 米以外**的车（考试规范：障碍车停在岔路口外约 1 米）。按几何推算，
#: 1 米外 EP 车约 90x72 像素；合成帧地面是浅灰（V=210，和考试场地的浅灰地面同量级）。
FAR_CAR_LEFT_BOX = (175, 170, 265, 242)
FAR_CAR_RIGHT_BOX = (375, 170, 465, 242)


def draw_far_car(image, box):
    """画一辆"停在约 1 米外、没有彩色装甲"的车（比近处那辆小一半左右）。"""
    x0, y0, x1, y1 = box
    cv2.rectangle(image, (x0, y0), (x1, y1), CAR_BODY, -1)


#: 考试摆法（用户 2026-09-18 明确）：障碍物是 RoboMaster EP，**关机也可能**、
#: 甚至**只剩底盘**（云台顶缺失），**停在岔路的蓝线上**、离岔路口 **0.9~1.1 米**。
#: 尺寸按 1 米外 EP 底盘（约 30cm 宽）折算：约 90x45 像素。
EXAM_OBSTACLE_LEFT = (190, 188, 280, 233)
EXAM_OBSTACLE_RIGHT = (360, 188, 450, 233)
#: 更远一点的同一辆车（约 1.3 米）—— 用来记录判据的有效距离范围。
EXAM_OBSTACLE_FAR_LEFT = (200, 192, 270, 227)


def draw_powered_off_obstacle(image, box, wheels=True):
    """画一辆**关机、没有灯、没有云台顶**的 EP：只有深灰底盘（+ 两条更黑的履带）。

    这就是考试现场可能的样子：没有高饱和彩色装甲（灯没亮）、顶部缺失、可能没通电。
    所以判据只能靠"**一个压在蓝线上的深色物体**"这个与外观无关的特征。
    """
    x0, y0, x1, y1 = box
    cv2.rectangle(image, (x0, y0), (x1, y1), (72, 72, 72), -1)          # 深灰底盘
    height = y1 - y0
    width = x1 - x0
    if wheels:
        wheel_w = max(4, width // 6)
        cv2.rectangle(image, (x0 + 2, y0 + height // 2), (x0 + 2 + wheel_w, y1 - 2),
                      (28, 28, 28), -1)
        cv2.rectangle(image, (x1 - 2 - wheel_w, y0 + height // 2), (x1 - 2, y1 - 2),
                      (28, 28, 28), -1)


def noise_split_frame():
    """一条很宽的带子被"竖直噪声"等宽切开：上下间距完全一样，不是岔路。

    注意 `cv2.rectangle` 的颜色必须写成**三元组**：给标量 `FLOOR`（一个 int）时，
    OpenCV 只往第一个通道写，填出来是 `[210,0,0]` —— 那**仍然是蓝色**（HSV 在蓝线
    区间内），于是"被切开的带子"根本没被切开，这条测试就成了空转（2026-09-17 发现）。
    """
    image = np.full((360, 640, 3), FLOOR, np.uint8)
    cv2.rectangle(image, (290, 194), (350, 350), BLUE, -1)
    cv2.rectangle(image, (305, 194), (335, 350), (FLOOR, FLOOR, FLOOR), -1)
    return image


def edge_fork_frame():
    """右分支一直伸到 ROI 右边缘之外。

    加速版的整列扫描最初就是漏了这种"某一段贴着边缘"的行（数段数时少算一段），
    真车画面里踩到过：同一帧旧版认得出岔路、新版认不出。这个帧专门守这个 bug。
    """
    image = np.full((360, 640, 3), FLOOR, np.uint8)
    cv2.rectangle(image, (548, 300), (572, 350), BLUE, -1)
    cv2.line(image, (560, 300), (200, 190), BLUE, 18)
    cv2.line(image, (560, 300), (700, 170), BLUE, 18)
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


    def test_a_wall_band_with_a_bit_of_colour_is_not_a_vehicle(self):
        """**换场地后的误报**（2026-09-17 19:05/19:06）：墙 + 墙脚阴影带 + 木门。

        那条暗带会横跨整条走廊，木门的彩色把它"点亮" → 原来过了闸门，
        于是空的那一侧也被判成"有车"，日志里出现 `both branches blocked`。
        实测：真车候选面积占比 ≤ 0.66，这条误报 0.84~0.85 —— 用面积上限挡掉。
        """
        settings = FreeJunctionConfig()
        detector = FreeJunctionDetector(settings)
        vehicle = VehicleDetector(settings)
        region = np.full((136, 300, 3), FLOOR, np.uint8)
        # 暗带占走廊的大部分（宽 250 × 高 130 / 300×136 ≈ 0.80），只有一小块彩色
        cv2.rectangle(region, (10, 5), (260, 135), CAR_BODY, -1)
        cv2.rectangle(region, (200, 40), (250, 90), CAR_ARMOR_RED, -1)   # 木门那点彩色
        blocked, score, box = vehicle.detect(region, detector.blue_mask(region))
        self.assertFalse(
            blocked,
            "墙裙那条大暗带被当成车了（真车面积上限 0.75）：证据=%.3f 框=%s" % (score, box),
        )

        # 同一块地方，把它缩成"车那么大"（占走廊约 0.3）→ 应该判成车
        small = np.full((136, 300, 3), FLOOR, np.uint8)
        cv2.rectangle(small, (60, 40), (170, 110), CAR_BODY, -1)
        cv2.rectangle(small, (64, 44), (76, 106), CAR_ARMOR_RED, -1)
        cv2.rectangle(small, (154, 44), (166, 106), CAR_ARMOR_GREEN, -1)
        blocked2, score2, box2 = vehicle.detect(small, detector.blue_mask(small))
        self.assertTrue(blocked2, "车那么大的深色块+装甲应该判成车：%s %s" % (score2, box2))

    def test_a_large_blob_is_a_vehicle_only_with_enough_colour_on_it(self):
        """大块候选（面积 ≥ 0.5）要"彩色够多"才算车 —— 车和墙连成一块时也要认得出。

        这条规则是为了同时满足两头（2026-09-17 换场地后实测）：
          * 墙裙那条大暗块：彩色 0.074 → **挡掉**；
          * 车和墙连成一块的真检测：彩色 0.22~0.27、面积 0.7~0.8 → **照旧认出**。

        注意"彩色"用的是**细灯条**：真车（S1/EP）的装甲灯就是细条/小块，
        掩码只把离深色车体一个核半径以内的彩色像素并进来，所以大面积纯色块的
        内部不会被计入 —— 这是照着实物标定的，不是缺陷。
        """
        settings = FreeJunctionConfig()
        detector = FreeJunctionDetector(settings)
        vehicle = VehicleDetector(settings)

        def bars(image, y0, y1, colour, thickness=6):
            for y in range(y0, y1, 22):
                cv2.rectangle(image, (20, y), (250, y + thickness), colour, -1)

        # 大块 + 细灯条够多（≈车和墙连成一块）→ 判成车
        merged = np.full((136, 300, 3), FLOOR, np.uint8)
        cv2.rectangle(merged, (5, 5), (265, 130), CAR_BODY, -1)
        bars(merged, 14, 128, CAR_ARMOR_RED)
        bars(merged, 25, 128, CAR_ARMOR_GREEN)
        blocked, score, box = vehicle.detect(merged, detector.blue_mask(merged))
        self.assertTrue(
            blocked,
            "车和墙连成一块的大候选（彩色够多）不该被挡：证据=%.3f 框=%s" % (score, box),
        )

        # 同样大的块，但彩色只有一小条（≈墙裙 + 木门）→ 不是车
        wall = np.full((136, 300, 3), FLOOR, np.uint8)
        cv2.rectangle(wall, (5, 5), (265, 130), CAR_BODY, -1)
        cv2.rectangle(wall, (232, 40), (256, 46), CAR_ARMOR_RED, -1)
        blocked2, score2, box2 = vehicle.detect(wall, detector.blue_mask(wall))
        self.assertFalse(
            blocked2,
            "大块但彩色很少（墙裙+门）不该判成车：证据=%.3f 框=%s" % (score2, box2),
        )


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


class _FakeCandidate(object):
    """模仿 `number_marker.MarkerCandidate` 的鸭子类型对象（只带本模块要用的字段）。"""

    def __init__(self, center, width, height, observed_at=None, target_id=""):
        self.center = center
        self.width = width
        self.height = height
        self.observed_at = observed_at
        self.target_id = target_id


class OfficialSdkCriterionTests(unittest.TestCase):
    """**官方 SDK 识别结果当判据**（`blockage_source="sdk"`）。

    官方读数走的是 `main.py` 的 `feed_robot_observations()`（`robot_source.py` 订阅
    SDK 的**机器人识别**）→ 任务上的 `update_robot_observations()`
    （和 5 号 `obstacle.py` 同一条通路），
    本模块只消费纯数据、不碰 SDK。所以这里全部用假读数离线验证。

    ⚠️ 别把这条通路接回 `update_robot_observations()`：那是 `number_marker` 的**视觉标签**
    通道，标签不是车（接线回归在 `tests/test_free_junction_observation_wiring.py`）。

    重点证明一件事：**这条路上判据只有官方读数** ——
    画面里没有车（`fork_frame()`）也能判对，画面里画了车也不作数。
    """

    def _task(self, **overrides):
        settings = FreeJunctionConfig(blockage_source="sdk", **overrides)
        return FreeJunctionTask(settings)

    def _replay(self, task, image, sightings, frames=200, dt=0.05, start=1.0):
        """和 main.py 同序：**每帧先把官方读数推给任务，再 step**。"""
        records = []
        now = start
        for index in range(frames):
            task.update_robot_observations(sightings, now=now)
            update = task.step(FramePacket(image, index + 1, now), now)
            records.append((task.state, update, now))
            now += dt
            if update.status in (TaskStatus.COMPLETED, TaskStatus.FAILED):
                break
        return records

    def test_official_reading_on_the_left_takes_the_right_branch(self):
        """官方在左分支报出一辆车 —— 画面里**没有**画车，照样走右边。"""
        task = self._task()
        records = self._replay(task, fork_frame(), [(0.20, 0.40, 0.18, 0.26)])
        updates = [update for _state, update, _now in records]
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.RIGHT)
        self.assertEqual(task.last_blockage.source, "sdk")
        self.assertIn("official detector", task.last_reason)
        self.assertTrue(all(yaw > 0 for yaw in turn_yaws(records)), turn_yaws(records))

    def test_official_reading_on_the_right_takes_the_left_branch(self):
        task = self._task()
        records = self._replay(task, fork_frame(), [(320.0, 180.0, 120.0, 90.0)])
        updates = [update for _state, update, _now in records]
        self.assertIs(updates[-1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.LEFT)
        self.assertTrue(all(yaw < 0 for yaw in turn_yaws(records)), turn_yaws(records))

    def test_the_picture_is_not_a_criterion_in_this_mode(self):
        """画面上有车（画面判据会判"堵"），但官方没读数 → **不接管**。

        这就是要求里"不要拿长宽高/颜色/形状当判据"的回归测试。
        """
        task = self._task()
        for _state, update, _now in self._replay(task, fork_frame(car_left=True), (), frames=60):
            self.assertIs(update.status, TaskStatus.NOT_TRIGGERED)

    def test_snapshot_expiry_does_not_revive_the_criterion(self):
        """过期的官方读数不能被当成判据（宁可没有判据，也不拿旧读数选路）。"""
        task = self._task()
        now = 1.0
        stale = [(0.20, 0.40, 0.18, 0.26)]
        for index in range(40):
            task.update_robot_observations(stale, now=now - 1.0)      # 时间戳整整旧了 1 秒
            update = task.step(FramePacket(fork_frame(), index + 1, now), now)
            self.assertIs(update.status, TaskStatus.NOT_TRIGGERED)
            now += 0.05
        # 换成新鲜读数 → 立刻恢复正常
        records = self._replay(task, fork_frame(), stale, start=now, frames=200)
        self.assertIs(records[-1][1].status, TaskStatus.COMPLETED)

    def test_reading_outside_the_corridor_band_is_ignored(self):
        """带外（画面最下方的自家车头 / 最上方的背景 / ROI 之外）的读数不算数。"""
        outside = [
            (0.20, 0.95, 0.18, 0.10),     # 太靠下：自家车头那一带
            (0.20, 0.02, 0.18, 0.06),     # 太靠上：背景
            (0.01, 0.40, 0.02, 0.10),     # 落在岔路 ROI 横向范围之外
        ]
        for sighting in outside:
            task = self._task()
            records = self._replay(task, fork_frame(), [sighting], frames=40)
            self.assertIs(
                records[-1][1].status, TaskStatus.NOT_TRIGGERED,
                "带外的读数不该成为判据: %s" % (sighting,),
            )

    def test_readings_on_both_sides_fail_instead_of_guessing(self):
        task = self._task()
        records = self._replay(
            task, fork_frame(), [(0.20, 0.40, 0.18, 0.26), (0.80, 0.40, 0.18, 0.26)]
        )
        self.assertIs(records[-1][1].status, TaskStatus.FAILED)
        self.assertIn("both branches", task.last_reason)

    def test_normalized_and_pixel_readings_agree(self):
        """归一化坐标和像素坐标说的是同一件事（SDK 文档没写用哪种，两种都要认）。"""
        pixel = self._task()
        pixel_records = self._replay(pixel, fork_frame(), [(128.0, 144.0, 115.0, 94.0)])
        normalized = self._task()
        normalized_records = self._replay(
            normalized, fork_frame(), [(0.20, 0.40, 0.18, 0.26)]
        )
        for records in (pixel_records, normalized_records):
            self.assertIs(records[-1][1].status, TaskStatus.COMPLETED)
        self.assertIs(pixel.chosen_branch, Branch.RIGHT)
        self.assertIs(normalized.chosen_branch, Branch.RIGHT)
        self.assertEqual(pixel.last_sdk_mode, "pixels")
        self.assertEqual(normalized.last_sdk_mode, "normalized")

    def test_pushed_shapes_are_all_understood(self):
        """三种推送写法都认：`(x,y,w,h)`、`(x,y,w,h,标签)`、`MarkerCandidate` 那样的对象。"""
        for label, sighting in (
            ("4 元组（机器人识别）", (0.20, 0.40, 0.18, 0.26)),
            ("5 元组（视觉标签）", (0.20, 0.40, 0.18, 0.26, "3")),
            ("对象（number_marker 的候选）", _FakeCandidate((0.20, 0.40), 0.18, 0.26, 1.0, "3")),
        ):
            task = self._task()
            records = self._replay(task, fork_frame(), [sighting])
            self.assertIs(
                records[-1][1].status, TaskStatus.COMPLETED,
                "这种推送写法没被认出来: %s" % label,
            )
            self.assertIs(task.chosen_branch, Branch.RIGHT, label)

    def test_the_default_source_prefers_the_official_reading(self):
        """**默认配置下官方 SDK 的读数说了算**（要求：判据用官方机器人识别）。

        画面里"左支有车"、官方读数却说右边有车 —— 默认必须听**官方**的
        （走左边），而且读数来源标成 `sdk`。
        """
        task = FreeJunctionTask()                    # 默认 blockage_source="sdk_or_vision"
        self.assertEqual(task.settings.blockage_source, "sdk_or_vision")
        records = self._replay(task, fork_frame(car_left=True), [(0.80, 0.40, 0.18, 0.26)])
        self.assertIs(records[-1][1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.LEFT)
        self.assertEqual(task.last_blockage.source, "sdk")
        self.assertGreater(task.official_sightings, 0)

    def test_the_default_falls_back_and_says_so_when_the_sdk_is_silent(self):
        """官方读数一直没到（集成层还没订阅 robot 识别）时：退回画面判据，并**写在 message 里**。

        实车上就靠这句话判断"集成层到底接没接"：
        看到 `official robot detection unavailable` = 现在用的是画面判据兜底。
        """
        for source in ("sdk_or_vision", "sdk"):
            task = FreeJunctionTask(FreeJunctionConfig(blockage_source=source))
            self.assertIn("official robot detection unavailable", task._official_note())
            self.assertEqual(task.official_sightings, 0)

        # "sdk_or_vision"：官方没读数 → 退回画面判据，并把这件事写进接管那一刻的 message
        fallback = FreeJunctionTask(FreeJunctionConfig(blockage_source="sdk_or_vision"))
        records = self._replay(fallback, fork_frame(car_left=True), ())
        messages = [update.message for _state, update, _now in records]
        self.assertTrue(
            any("official robot detection unavailable" in m for m in messages),
            "官方没读数时必须写明在用画面判据兜底：%s" % messages[:3],
        )
        self.assertIs(records[-1][1].status, TaskStatus.COMPLETED)
        self.assertEqual(fallback.last_blockage.source, "vision")

        # "sdk"：官方没读数就是"没有判据" —— 不接管（只用于确认订阅有没有通）
        strict = FreeJunctionTask(FreeJunctionConfig(blockage_source="sdk"))
        records = self._replay(strict, fork_frame(car_left=True), (), frames=40)
        for _state, update, _now in records:
            self.assertIs(update.status, TaskStatus.NOT_TRIGGERED)

        # 纯画面判据那条路上不该出现这句提示
        picture = FreeJunctionTask(FreeJunctionConfig(blockage_source="vision"))
        self.assertEqual(picture._official_note(), "")

    def test_or_vision_mode_falls_back_when_the_official_source_is_silent(self):
        """"sdk_or_vision"：官方没读数时退回画面判据，不会因此错过岔路。"""
        task = FreeJunctionTask(FreeJunctionConfig(blockage_source="sdk_or_vision"))
        records = self._replay(task, fork_frame(car_left=True), ())
        self.assertIs(records[-1][1].status, TaskStatus.COMPLETED)
        self.assertIs(task.chosen_branch, Branch.RIGHT)
        self.assertEqual(task.last_blockage.source, "vision")

        # 官方有读数时以官方为准（即使画面里根本没车）
        official = FreeJunctionTask(FreeJunctionConfig(blockage_source="sdk_or_vision"))
        records = self._replay(official, fork_frame(), [(0.20, 0.40, 0.18, 0.26)])
        self.assertIs(records[-1][1].status, TaskStatus.COMPLETED)
        self.assertEqual(official.last_blockage.source, "sdk")

    def test_snapshot_is_replaced_not_merged(self):
        """快照是"整体替换"：官方改口说"什么也没看到"，读数就得跟着变。"""
        task = self._task()
        image = fork_frame()
        task.update_robot_observations([(0.20, 0.40, 0.18, 0.26)], now=1.0)
        task.step(FramePacket(image, 1, 1.0), 1.0)
        self.assertEqual(task.last_blockage.reading, BLOCKAGE_LEFT)

        task.update_robot_observations([], now=1.05)          # 官方这一帧什么也没看到
        task.step(FramePacket(image, 2, 1.05), 1.05)
        self.assertEqual(task.last_blockage.reading, BLOCKAGE_NONE)

        task.update_robot_observations([(0.80, 0.40, 0.18, 0.26)], now=1.10)   # 换成右边
        task.step(FramePacket(image, 3, 1.10), 1.10)
        self.assertEqual(task.last_blockage.reading, BLOCKAGE_RIGHT)


class SpeedRegressionTests(unittest.TestCase):
    """**提速不许改判**（2026-09-16 19:58 实车运行里 step() 超预算的修法）。

    那一次运行成功但日志一直报 `free_junction step was slow: 0.031s (limit 0.020s)`，
    连模块结束后的帧也在报。三处改动：整列扫描向量化预筛、用不到判据的阶段不算判据、
    判据在缩小的检测带上算。这一组测试把"结果必须和老实算完全一样"钉住。
    """

    def test_fork_rows_equals_the_naive_full_scan(self):
        """加速后的整列扫描 == 逐行老实扫（含"某一段贴着 ROI 边缘"的行）。"""
        settings = FreeJunctionConfig()
        for image in (fork_frame(), fork_frame(car_left=True), fork_frame(tips=(200, 440)),
                      noise_split_frame(), edge_fork_frame(), line_frame()):
            _fork, _roi, mask, _rect = FreeJunctionDetector(settings).analyze(image)
            naive = []
            for row in range(mask.shape[0]):
                found = _separated_runs(
                    mask[row], settings.merge_gap_px, settings.min_run_px,
                    settings.min_branch_separation_px, settings.min_gap_over_tape)
                if found is not None:
                    naive.append((row, found[0], found[1], found[2]))
            fast = _fork_rows(
                mask, settings.merge_gap_px, settings.min_run_px,
                settings.min_branch_separation_px, settings.min_gap_over_tape)
            self.assertEqual(naive, fast, "整列扫描的加速版和逐行扫描结果不一致")

    def test_a_fork_touching_the_roi_edge_is_still_found(self):
        """贴着 ROI 右边缘的分支也要认得出（加速版最初漏掉的就是这种行）。"""
        self.assertTrue(FreeJunctionDetector().detect(edge_fork_frame()).valid)

    def test_downscaling_does_not_change_the_reading(self):
        """检测带缩小一半只提速、不改判。"""
        for image, expected in (
            (fork_frame(car_left=True), BLOCKAGE_LEFT),
            (fork_frame(car_right=True), BLOCKAGE_RIGHT),
            (fork_frame(), BLOCKAGE_NONE),
        ):
            readings = []
            for scale in (1.0, 0.5):
                task = FreeJunctionTask(FreeJunctionConfig(vehicle_downscale=scale))
                fork, _roi, _line, rect = task.detector.analyze(image)
                readings.append(task._read_blockage(fork, image, rect, 1.0).reading)
            self.assertEqual(
                readings, [expected, expected],
                "缩放检测带改变了读数：1.0 -> %s，0.5 -> %s（期望 %s）"
                % (readings[0], readings[1], expected),
            )

    def test_the_blockage_is_not_recomputed_after_the_branch_is_chosen(self):
        """APPROACH / TURN / EXIT 用的是已定分支，不该每帧再算一遍判据。"""
        task = FreeJunctionTask()
        seen = []
        original = task._read_blockage

        def counting(*args, **kwargs):
            seen.append(task.state)
            return original(*args, **kwargs)

        task._read_blockage = counting
        records = run_with_states(task, fork_frame(car_left=True))
        self.assertIs(records[-1][1].status, TaskStatus.COMPLETED)
        self.assertTrue(seen, "IDLE / DECIDE 阶段必须算判据")
        for state in (JunctionState.APPROACH, JunctionState.TURN, JunctionState.EXIT):
            self.assertNotIn(state, seen, "%s 阶段不该再算判据" % state)


class AimDirectionTests(unittest.TestCase):
    """选完支之后**方向必须打对**（2026-09-17 实车翻车的那一条）。

    那次日志里明明写着 `aligning with the right branch`，车却开进了左边那条
    "有车"的分支，最后交回巡线时直接 `LINE_LOST`。两个原因，现在都改掉了：

    1. `left_x` / `right_x` 原来取**分叉行**（两条分支刚分开、几乎重合）的中心，
       所以"瞄准右支"实际瞄的是几乎正前方（实测 yaw 只有 +2.5，等于直行）；
       现在取**张开最大那一行**的中心 —— 那才是"分支往哪走"。
    2. 瞄准的参考点原来是**画面中心**，于是整条岔路偏在画面左边时，"右支"也落在
       中心左边 → yaw 变成负的、往左打。现在参考点是**车头正下方那根线**
       （车实际压在哪），右支永远在它右边。
    """

    def _aim(self, image, fixed_branch="right"):
        settings = FreeJunctionConfig(decision_rule="fixed", fixed_branch=fixed_branch)
        task = FreeJunctionTask(settings)
        fork, _roi, _line, _rect = task.detector.analyze(image)
        self.assertTrue(fork.valid, "这一帧应该认得出岔路")
        task.last_detection = fork
        task.chosen_branch = task._as_branch(fixed_branch)
        return task, fork, task._approach_yaw()

    def test_branch_targets_point_along_the_branches(self):
        """瞄准点要指向"分支张开后"的方向，而不是分叉点上几乎重合的两点。"""
        fork, _roi, _line, _rect = FreeJunctionDetector().analyze(fork_frame(car_left=True))
        self.assertLess(fork.left_x, fork.split_x)
        self.assertGreater(fork.right_x, fork.split_x)
        self.assertGreater(
            fork.right_x - fork.left_x, fork.separation_px * 1.5,
            "两个瞄准点分得太开不够 —— 说明又回去用分叉行了",
        )

    def test_the_aim_is_strong_when_the_right_branch_is_chosen(self):
        """选右支时不能只给一个"几乎直行"的 yaw（实车那次只有 +2.5）。

        注：这张合成图里"车"是画在左分支上的，挡住了左分支的上半段，
        所以可见的分支方向偏弱 —— 这里只要求"明显往右"，比例关系由
        `test_the_aim_follows_the_offset_proportionally` 精确守住。
        """
        _task, _fork, yaw = self._aim(fork_frame(car_left=True))
        self.assertGreater(yaw, 3.0, "选右支时的 yaw 太弱：%+.1f" % yaw)

    def test_handback_needs_one_clean_line_not_just_some_blue(self):
        """交回巡线要求"脚下只有**一段**胶带"（单根清晰的线），不只是"有蓝线像素"。

        车还压在岔路口上时脚下常常同时有主干和分支两段，底层巡线拿到这种画面会
        自己判丢线 —— 交回也没用。所以两段时必须再等，等到只有一段（或到时间上限）。
        """

        def run_into_exit(task, image, now, limit=200):
            for index in range(limit):
                update = task.step(FramePacket(image, index + 1, now), now)
                now += 0.05
                if task.state is JunctionState.EXIT or update.status in (
                    TaskStatus.COMPLETED, TaskStatus.FAILED,
                ):
                    return now, update
            return now, update

        # ① 干净的单根线 → 走"条件满足"这条路交回
        clean = FreeJunctionTask()
        now, _update = run_into_exit(clean, fork_frame(car_left=True), 1.0)
        for index in range(20):
            update = clean.step(FramePacket(line_frame(), 500 + index, now), now)
            now += 0.05
            if update.status is TaskStatus.COMPLETED:
                break
        self.assertIs(update.status, TaskStatus.COMPLETED)
        self.assertIn("the line is back", clean.last_message)

        # ② 脚下是两段（还压在岔路口上）→ 不能靠"有线"交回，只能到时间上限
        split = FreeJunctionTask()
        now, _update = run_into_exit(split, fork_frame(car_left=True), 1.0)
        for index in range(30):
            update = split.step(FramePacket(noise_split_frame(), 900 + index, now), now)
            now += 0.05
            if update.status is TaskStatus.COMPLETED:
                break
        self.assertIs(update.status, TaskStatus.COMPLETED)
        self.assertIn("junction cleared", split.last_message,
                      "两段胶带时不该按'线回来了'交回：%s" % split.last_message)

    def test_the_aim_is_an_angle_not_a_pixel_ratio(self):
        """瞄准量的是**角度**：偏 51 px ≈ 15°，车就该转 15 deg/s（再限幅到 12）。

        这是 2026-09-17 19:48 / 19:49 两次实车的回归用例：那条岔路很"浅"，
        右支只比画面中心偏 ~51 px。旧的"像素/半宽"口径算出 0.16 →
        yaw 只有 3.5（再乘远处折扣只剩 ~1），车几乎直着开进了左边拥堵支。
        """
        settings = FreeJunctionConfig()          # hfov 120° → 焦距 185 px
        task = FreeJunctionTask(settings)
        task.chosen_branch = Branch.RIGHT

        def aim(right_x):
            task.last_detection = ForkDetection(
                valid=True, split_x=300, left_x=200, right_x=right_x, frame_width=640
            )
            return task._approach_yaw()

        offset_px = 51
        degrees = task._pixels_to_degrees(offset_px, 640)
        self.assertAlmostEqual(degrees, 15.4, delta=1.0, msg="角度换算不对")
        yaw = aim(320 + offset_px)
        self.assertAlmostEqual(yaw, degrees, delta=0.1, msg="增益 1.0 时应等于偏角")
        self.assertGreater(yaw, 8.0, "偏 15° 只给出 %+.1f deg/s —— 车根本转不过来" % yaw)

        # 偏角越大 yaw 越大（单调），很大时超过限幅（由 _motion 裁到 12）
        self.assertGreater(aim(320 + 100), yaw)
        self.assertGreater(aim(320 + 300), settings.max_approach_yaw)

        # 左支：同样的几何下必须往反方向（负）
        task.chosen_branch = Branch.LEFT
        task.last_detection = ForkDetection(
            valid=True, split_x=300, left_x=200, right_x=371, frame_width=640
        )
        self.assertLess(task._approach_yaw(), 0.0)

        # 真正发出去的命令必须被裁到限幅内
        task.chosen_branch = Branch.RIGHT
        task.state = JunctionState.APPROACH
        task.last_detection = ForkDetection(
            valid=True, split_x=300, left_x=200, right_x=600, frame_width=640
        )
        self.assertAlmostEqual(
            task._motion(1.0).yaw, settings.max_approach_yaw, delta=1e-6
        )

    def test_choosing_right_always_aims_further_right_than_choosing_left(self):
        """不管岔路偏在哪，选右支瞄得一定比选左支更靠右（这才是"选对边"的含义）。

        整条岔路偏在画面左侧时，"右支"也可能仍在车头方向左边 —— 那时**应该**
        往左打（车得先开到岔路口），但**永远比选左支时更靠右**；
        进入分支这件事由每帧重新瞄准的闭环完成。
        （2026-09-17 曾经把"脚下那根线"当参考点，结果浅岔路下 yaw 只有 ±1、车不转。）
        """
        for image in (fork_frame(car_left=True), fork_frame(x=180, tips=(60, 300))):
            _task, _fork, right_yaw = self._aim(image, "right")
            _task2, _fork2, left_yaw = self._aim(image, "left")
            self.assertGreater(
                right_yaw, left_yaw,
                "选右支必须瞄得更靠右：右 %+.1f vs 左 %+.1f" % (right_yaw, left_yaw),
            )
            self.assertGreater(right_yaw - left_yaw, 5.0, "两者的差太小，等于没区分")

    def test_the_turn_offset_uses_the_same_reference(self):
        """转弯阶段和瞄准阶段必须用同一个参考点，否则两段会互相打架。"""
        task, fork, yaw = self._aim(fork_frame(car_left=True))
        offset = task._turn_offset(fork)
        self.assertIsNotNone(offset)
        self.assertGreater(offset, 0.0)
        self.assertGreater(yaw * offset, 0.0, "对准和转弯的方向符号必须一致")

    def test_the_turn_is_gentler_while_the_junction_is_still_far(self):
        """岔路还在画面远处时**先别急着转**（摄像头比车头早半米看到岔路）。

        2026-09-17 17:24 / 17:26 / 17:27 三次实车都是"看到就满舵转"，
        车头还没到路口方向就转过去了；17:24 那次交回巡线后直接 LINE_LOST。
        """
        settings = FreeJunctionConfig()
        task = FreeJunctionTask(settings)

        def turn_yaw(split_row):
            task.state = JunctionState.TURN
            task.chosen_branch = Branch.RIGHT
            task.last_detection = ForkDetection(
                valid=True, split_row=split_row, split_x=300, left_x=200,
                right_x=420, frame_width=640, frame_height=360,
            )
            task._last_line_mask = None
            return task._motion(1.0).yaw

        far = turn_yaw(200)      # 岔路还在 ROI 上半部（车头离路口还远）
        near = turn_yaw(330)     # 岔路已经到画面下部（车到了）
        self.assertGreater(near, 0.0)
        self.assertGreater(far, 0.0, "远处也要转一点，不能完全不转")
        self.assertLess(far, near, "远处应该转得更轻")
        # 折扣本身：远处不小于 turn_far_yaw_scale，到跟前基本不折扣
        forked = lambda row: ForkDetection(  # noqa: E731
            valid=True, split_row=row, split_x=300, left_x=200, right_x=420,
            frame_width=640, frame_height=360,
        )
        self.assertGreaterEqual(
            task._turn_rate_scale(forked(200)), settings.turn_far_yaw_scale - 1e-6,
            "远处最多只能打到 turn_far_yaw_scale 这个折扣",
        )
        self.assertAlmostEqual(task._turn_rate_scale(forked(345)), 1.0, delta=1e-6)
        # 折扣随"岔路越来越近"单调上升（不是一刀切）
        self.assertLess(turn_yaw(220), turn_yaw(260))
        self.assertLess(turn_yaw(260), turn_yaw(300))

    def test_the_turn_keeps_moving_forward(self):
        """转弯时要真的往前挪：0.04 太小，车几乎原地转身（实车就是这么拐早的）。"""
        settings = FreeJunctionConfig()
        self.assertGreaterEqual(
            settings.turn_forward, settings.approach_forward * 0.9,
            "转弯时的前进速度不该比对准阶段还慢一大截",
        )
        task = FreeJunctionTask(settings)
        task.state = JunctionState.TURN
        task.chosen_branch = Branch.RIGHT
        task.last_detection = ForkDetection(
            valid=True, split_row=330, split_x=300, left_x=200, right_x=420,
            frame_width=640, frame_height=360,
        )
        task._last_line_mask = None
        self.assertAlmostEqual(
            task._motion(1.0).forward, settings.turn_forward, delta=1e-9
        )


    def test_no_detection_while_the_rearm_gate_is_cooling(self):
        """封锁冷却期内**连画面都不看**：这些帧一定是"不接管"，却白花最贵的一步。

        实车日志里 `free_junction step was slow` 有相当一部分就落在"刚走完岔路、
        还在冷却"的那些帧上（2026-09-16 的 14.7/15.8s、2026-09-17 的 18.9s）。
        """
        task = FreeJunctionTask()
        calls = []
        original = task.detector.analyze

        def counting(image):
            calls.append(1)
            return original(image)

        task.detector.analyze = counting
        task.reset()                      # 拉起封锁闸门（冷却 rearm_cooldown 秒）
        now = 1.0
        for index in range(10):
            update = task.step(FramePacket(fork_frame(car_left=True), index + 1, now), now)
            self.assertIs(update.status, TaskStatus.NOT_TRIGGERED)
            now += 0.05
        self.assertEqual(calls, [], "冷却期内不该跑岔路检测")

        # 冷却结束后必须重新看画面，不能一直瞎着
        task.step(FramePacket(fork_frame(car_left=True), 100, 3.0), 3.0)
        self.assertTrue(calls, "冷却结束后必须重新检测岔路")


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
