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
# 障碍都画在 ROI（x 160..480, y 108..270）里面。
#
# 官方信息：障碍是**一辆静止的同型小车**（深色、有轮子/云台）。
# 所以合成障碍也照小车画 —— 结构判据既要对比度、也要框内有结构（过闸六那一关）。
BODY_BGR = (105, 105, 105)       # 灰车身
WHEEL_BGR = (25, 25, 25)         # 深色轮子
TOP_BGR = (230, 230, 230)        # 车身顶上的亮色件（云台）
SKIN_BGR = (140, 170, 220)       # 手掌/手臂的典型肤色（BGR）

# v7 之前的合成障碍是"亮橙色扁平块"。v8 把 Canny 前的模糊调大之后，
# 那种低对比度的扁平块结构判据抓不到（亮橙块灰色约 173，浅色地面 210，只差 37）。
# 这是 v8 的已知代价，写在模块顶部；真障碍是深色小车，对比度足够。
OBSTACLE_BGR = (0, 165, 255)     # 只留给"颜色判据"相关的用例


def _draw_car(image, center=(320, 200), size=(120, 90)):
    """在一张图上画一辆停着的同型小车（车身 + 两个轮子 + 云台）。"""
    cx, cy = center
    half_w, half_h = size[0] // 2, size[1] // 2
    left, top = cx - half_w, cy - half_h
    cv2.rectangle(image, (left, top), (cx + half_w, cy + half_h), BODY_BGR, -1)
    # 轮子放在框内部（不贴下缘），保证"框内有结构"那一关能过
    cv2.rectangle(image, (left + 8, top + 12), (left + 28, cy + half_h - 8), WHEEL_BGR, -1)
    cv2.rectangle(image, (cx + half_w - 28, top + 12), (cx + half_w - 8, cy + half_h - 8),
                  WHEEL_BGR, -1)
    cv2.rectangle(image, (cx - 18, top + 4), (cx + 18, top + 26), TOP_BGR, -1)


def obstacle_frame(x=320, center=(320, 220), size=(170, 160)):
    """**已经贴到车前的障碍**：它把近处的蓝线挡住了。

    v10 的触发条件是"视野里有车 **且** 蓝线消失"，所以合成障碍必须真的挡住
    `LINE_CHECK_ROI`（y 162..288）那一段的蓝线 —— 这正是车开到障碍跟前的样子。
    """
    image = line_frame(x)
    _draw_car(image, center=center, size=size)
    return image


def car_visible_frame(x=320, center=(320, 200), size=(120, 90)):
    """**远处就看到的车**：蓝线还看得见（不该触发）。

    用户要求：不要一看见车就开始位移。这一帧就是"看到了但还没到跟前"。
    """
    image = line_frame(x)
    _draw_car(image, center=center, size=size)
    return image


def car_frame(center=(330, 200)):
    """**另一台车**当障碍：灰车身 + 深色轮子，一个橙色像素都没有。

    2026-09-11 实车实测失败的场景，也是官方口径里障碍的样子。
    """
    image = line_frame()
    _draw_car(image, center=center, size=(120, 100))
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


def run_one_dodge(task, now, seq, limit=400):
    """触发一次绕行并跑完：先用"挡住线的障碍帧"触发，之后喂干净的线帧。

    v10 之后绕行途中不再看画面，所以触发完就可以喂线帧 ——
    这正是实车"绕过去、线又出现了"的样子。
    返回 (最后一次 update, now, seq)。
    """
    triggered = False
    update = None
    for _ in range(limit):
        seq += 1
        image = line_frame() if triggered else obstacle_frame()
        update = task.step(FramePacket(image, seq, now), now)
        now += 0.05
        if update.status is TaskStatus.RUNNING:
            triggered = True
        elif update.status in (TaskStatus.COMPLETED, TaskStatus.FAILED):
            break
    return update, now, seq


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

    def test_detects_a_car_like_obstacle(self):
        """合成的小车（车身 + 轮子 + 云台）要能被认出来。"""
        result = self.detector.detect(obstacle_frame())
        self.assertTrue(result.valid)
        self.assertEqual(result.kind, obstacle.KIND)
        self.assertAlmostEqual(result.center[0], 320, delta=15)
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

    def test_ignores_candidates_touching_the_roi_edge(self):
        """闸一（贴边淘汰）：实测 11 个误判里 9 个压在 ROI 下/左/右沿上。

        真障碍是"挡在路中间"的东西，应该完整待在 ROI 里面；被 ROI 边界切开的
        多半是背景、场地边界或车体自身。**上沿不算**——远处的东西本来就从上面露出来。
        """
        # ROI 是 x 160..480, y 108..270（640x360 画面）
        cases = {
            "压住下沿": (280, 200, 360, 268),
            "压住左沿": (162, 180, 250, 250),
            "压住右沿": (400, 180, 478, 250),
        }
        for name, (x1, y1, x2, y2) in cases.items():
            image = line_frame()
            cv2.rectangle(image, (x1, y1), (x2, y2), (120, 120, 120), -1)
            self.assertFalse(
                self.detector.detect(image).valid,
                "%s 的候选没有被淘汰" % name,
            )

    def test_ignores_a_light_flat_structure(self):
        """闸三：一块"和地面一样浅"的结构不算障碍。

        关掉颜色判据之后只剩"硬边 + 连成块"，对高对比背景没有区分力；
        但背景/场地边界的内部几乎全是浅色低饱和像素，这个比例会很低。
        """
        image = line_frame()
        # 只有一圈深一点的边，里面还是地面色（210）—— 像场地边界、背景结构
        cv2.rectangle(image, (240, 170), (360, 240), (170, 170, 170), 4)
        self.assertFalse(self.detector.detect(image).valid, "浅色结构被当成障碍了")

    def test_ignores_background_at_the_top_of_the_roi(self):
        """v5 反馈的"一启动就避障"：两个误判框的 y 正好等于 ROI 顶边。

        框内是"中灰 70%+亮白 22%"和"暗区 85%"——墙、白板、暗处，全是背景。
        v5 时我故意豁免了上沿，这次不再豁免。框尺寸直接抄自报告。
        """
        cases = {
            "中灰背景": (306, 108, 439, 161, (128, 128, 128)),
            "暗区背景": (180, 108, 316, 157, (60, 60, 60)),
        }
        for name, (x1, y1, x2, y2, color) in cases.items():
            image = line_frame()
            cv2.rectangle(image, (x1, y1), (x2, y2), color, -1)
            self.assertFalse(
                self.detector.detect(image).valid, "ROI 顶边的%s没被淘汰" % name
            )

    def test_ignores_a_candidate_off_to_the_side(self):
        """v5 反馈第三个误判：62x63 的块在 ROI 左下角，横向偏了 65%。

        真障碍挡在线上、就在车正前方，横向不该偏出 ROI 中间一半。框抄自报告。
        """
        image = line_frame()
        cv2.rectangle(image, (185, 201), (247, 264), (120, 120, 120), -1)
        self.assertFalse(self.detector.detect(image).valid, "侧前方的块被当成障碍了")

    def test_ignores_a_candidate_in_the_upper_part_of_the_roi(self):
        """框中心落在 ROI 上半部（太远）不算挡路的障碍。"""
        image = line_frame()
        cv2.rectangle(image, (280, 118), (360, 172), (120, 120, 120), -1)
        self.assertFalse(self.detector.detect(image).valid, "ROI 上半部的块被当成障碍了")

    def test_ignores_a_flat_neutral_region(self):
        """闸六：一块"平的、中性色"的区域不是障碍。

        审阅人给的 589 帧记录里，闸四/闸五之后漏网的 6 帧全是这个特征：
        `145456` 中灰 90.6% / meanV=104；`131607`、`131842` 那几帧亮白、
        meanV 211~226。它们既没有内部结构、也没有颜色。
        框坐标直接抄自那份记录。
        """
        cases = {
            "中灰平板": (247, 193, 322, 254, (104, 104, 104)),
            "亮白平板": (230, 179, 300, 261, (225, 225, 225)),
        }
        for name, (x1, y1, x2, y2, color) in cases.items():
            image = line_frame()
            cv2.rectangle(image, (x1, y1), (x2, y2), color, -1)
            self.assertFalse(self.detector.detect(image).valid, "%s 被当成障碍了" % name)

    def test_a_grey_object_with_inner_structure_is_still_an_obstacle(self):
        """闸六不能把"有结构的灰色物体"一起拒掉 —— 另一台车就是这样的。"""
        image = line_frame()
        cv2.rectangle(image, (280, 180), (400, 250), (120, 120, 120), -1)
        cv2.rectangle(image, (300, 220), (330, 248), (35, 35, 35), -1)   # 内部深色结构
        cv2.rectangle(image, (355, 220), (385, 248), (35, 35, 35), -1)
        self.assertTrue(self.detector.detect(image).valid, "有结构的灰色物体被拒了")

    def test_picks_the_bigger_nearer_candidate(self):
        """同时有两辆时，选更大更靠下的那辆。

        注意两辆车要**分开**，贴在一起会合并成一个大块、反而被宽度上限拒掉。
        """
        image = line_frame()
        _draw_car(image, center=(215, 180), size=(70, 52))     # 又小又远
        _draw_car(image, center=(345, 205), size=(130, 100))   # 又大又近
        result = self.detector.detect(image)
        self.assertTrue(result.valid)
        self.assertGreater(result.center[0], 280)
        self.assertGreater(result.center[1], 190)

    def test_returns_no_result_for_a_none_image(self):
        self.assertFalse(self.detector.detect(None).valid)


class ObstacleParameterTests(unittest.TestCase):
    """按官方信息核对参数：障碍是**一辆静止的小车**（另一台同型 RoboMaster）。

    同型车长约 30~32 cm、宽约 24 cm。下面这些距离是从这个尺寸算出来的，
    不是拍脑袋；改了速度或时长，这几条会立刻告诉你还够不够。
    """

    def test_official_dimensions_are_the_ones_we_use(self):
        """尺寸必须照 DJI 官方技术参数来，不能估。

        来源：https://www.dji.com/cn/robomaster-ep/specs 「机器人 - 尺寸」
              步兵机器人 320×240×270 mm（长×宽×高）；障碍是同型号小车。
        """
        self.assertAlmostEqual(obstacle.OUR_LENGTH_M, 0.32, places=3)
        self.assertAlmostEqual(obstacle.OUR_WIDTH_M, 0.24, places=3)
        self.assertAlmostEqual(obstacle.OBSTACLE_LENGTH_M, 0.32, places=3)
        self.assertAlmostEqual(obstacle.OBSTACLE_WIDTH_M, 0.24, places=3)

    def test_pass_distance_clears_the_whole_car_not_just_the_nose(self):
        """前进距离要按"**我们车尾也过去**"算：障碍车长 + 我们车长 + 余量。

        只算"车头过了"是不够的 —— 车头刚过障碍时车尾还在它旁边，
        一收回来就蹭上（用户实测就是这么蹭的）。
        """
        needed = obstacle.OBSTACLE_LENGTH_M + obstacle.OUR_LENGTH_M
        forward = obstacle.FWD_SPEED * obstacle.T_PASS_TIME
        self.assertGreaterEqual(
            forward,
            needed + 0.05,
            "前进只有 %.2fm，而两车全长加起来要 %.2fm —— 车尾过不去" % (forward, needed),
        )

    def test_side_distance_clears_both_widths(self):
        """横移要错开两车的**宽度**：(我们宽 + 障碍宽) / 2 + 余量。"""
        needed = (obstacle.OUR_WIDTH_M + obstacle.OBSTACLE_WIDTH_M) / 2.0
        lateral = obstacle.SIDE_SPEED * obstacle.T_OUT_TIME
        self.assertGreaterEqual(
            lateral,
            needed + 0.05,
            "横移只有 %.2fm，而两车半宽和是 %.2fm —— 错不开" % (lateral, needed),
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


class ObstacleRealSceneTests(unittest.TestCase):
    """用审阅人那次实车留下的**真实画面裁片**做回归。

    这是 2026-09-16 那次"半路没障碍却莫名开始避障"的直接证据：
      * `samples/real_floor_roi.png`    —— 被误判那一帧的 ROI 区域（只有地板和胶带）
      * `samples/real_obstacle_car.png` —— 同一批画面里那台真车
    素材缺了就跳过，不让测试因为少一张图就红。
    """

    ROI = (160, 108, 480, 270)

    def _asset(self, name):
        path = pathlib.Path(__file__).resolve().parent / "samples" / name
        if not path.exists():
            self.skipTest("缺少样图 %s" % name)
        data = np.fromfile(str(path), dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            self.skipTest("读不出样图 %s" % name)
        return image

    @staticmethod
    def _place(piece, at):
        """把真实裁片贴进一张 640x360 的合成帧里（蓝线 + 浅色地面）。"""
        frame = np.full((360, 640, 3), 210, np.uint8)
        cv2.line(frame, (320, 350), (320, 190), (255, 0, 0), 24)
        px, py = at
        height, width = piece.shape[:2]
        frame[py:py + height, px:px + width] = piece
        return frame

    def test_real_floor_does_not_trigger(self):
        """真实地板（误判那一帧的 ROI 区域）不许触发 —— 本轮的核心回归。

        v7 及以前：这块地板会被判成障碍（边缘密度 4.28%，真车才 14.28%，
        只差 1.8 倍）。v8 把 Canny 前的模糊调到 9、阈值 60/160 之后降到 0.00%。
        """
        floor = self._asset("real_floor_roi.png")
        frame = self._place(floor, (self.ROI[0], self.ROI[1]))
        result = obstacle.ObstacleDetector().detect(frame)
        self.assertFalse(result.valid, "真实地板又被当成障碍了")

    def test_real_obstacle_car_is_detected(self):
        """真实那台小车必须认出来 —— 认不出就没法绕（召回侧的断言）。"""
        car = self._asset("real_obstacle_car.png")
        for at in [(250, 150), (220, 160), (280, 140)]:
            frame = self._place(car, at)
            result = obstacle.ObstacleDetector().detect(frame)
            self.assertTrue(result.valid, "真车贴在 %s 时没认出来" % (at,))
            self.assertGreater(result.confidence, 0.5)


class ObstacleTriggerTests(unittest.TestCase):
    """v10 触发条件：**视野里有车 且 蓝线消失** 才动手。

    用户实测：原来"一看到车就横移"，车会在很远的地方就早早开始绕，
    结果每次都**正好停在障碍机器人前面**。所以必须等蓝线被障碍挡住再动。
    """

    def test_a_distant_car_does_not_trigger(self):
        """看到车、但蓝线还看得见 -> 不许接管（"不要一看见车就开始位移"）。"""
        task = ObstacleTask()
        now = 1.0
        seq = 0
        update = None
        for _ in range(40):
            seq += 1
            update = task.step(FramePacket(car_visible_frame(), seq, now), now)
            now += 0.05
        self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertEqual(task.stage, "IDLE")

    def test_a_blocking_car_triggers(self):
        """车把近处蓝线挡住了 -> 这才是该绕的距离，接管。"""
        task = ObstacleTask()
        now = 1.0
        seq = 0
        update = None
        for _ in range(20):
            seq += 1
            update = task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
            if update.status is TaskStatus.RUNNING:
                break
        self.assertEqual(update.status, TaskStatus.RUNNING, "车挡住了线却没接管")

    def test_line_loss_alone_does_not_trigger(self):
        """只有线没了（比如胶带缺口）、看不到车 -> 不许接管。"""
        task = ObstacleTask()
        blank = np.full((360, 640, 3), 210, np.uint8)
        for index in range(30):
            now = 1.0 + index * 0.05
            update = task.step(FramePacket(blank, index + 1, now), now)
            self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertEqual(task.stage, "IDLE")

    def test_the_dodge_ignores_the_obstacle_box_once_started(self):
        """一旦开始绕，就不再管障碍框了 —— 中间几帧把车挪走也照样绕完。"""
        task = ObstacleTask()
        now = 1.0
        seq = 0
        for _ in range(20):
            seq += 1
            update = task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
            if update.status is TaskStatus.RUNNING:
                break
        self.assertEqual(update.status, TaskStatus.RUNNING)

        # 障碍"消失"了（画面里只剩线），动作必须照走不误
        stages = []
        for _ in range(200):
            seq += 1
            update = task.step(FramePacket(line_frame(), seq, now), now)
            now += 0.05
            stages.append(task.stage)
            if update.status is not TaskStatus.RUNNING:
                break
        self.assertIn("PASS", stages, "前进段没跑")
        self.assertIn("SEEK", stages, "没有回到线上的收尾段")
        self.assertEqual(update.status, TaskStatus.COMPLETED)


class ObstacleRobotObservationTests(unittest.TestCase):
    """【v9 主路径】DJI SDK 的机器人识别 —— **完全不看颜色**。

    集成层订阅 `vision.sub_detect_info(name="robot")`，每帧把回调给的
    `(x, y, w, h)`（中心点 + 宽高）推给模块；模块只用、不订（契约不允许碰 SDK）。
    """

    def test_sdk_robot_box_is_used_directly(self):
        """画面里只有蓝线，但 SDK 说"那里有一台机器人" -> 就用 SDK 的框。"""
        task = ObstacleTask()
        task.update_robot_observations([(320, 200, 200, 150)], observed_at=1.0)
        result = task.detect(line_frame(), now=1.05)
        self.assertTrue(result.valid, "SDK 给了框却没认出来")
        self.assertAlmostEqual(result.center[0], 320, delta=2)
        self.assertAlmostEqual(result.center[1], 200, delta=2)
        self.assertGreater(result.confidence, 0.9)
        self.assertEqual(result.kind, obstacle.KIND)

    def test_stale_observation_is_ignored(self):
        """SDK 回调掉了一会儿了，不能拿旧框当新鲜目标。"""
        task = ObstacleTask()
        task.update_robot_observations([(320, 200, 200, 150)], observed_at=1.0)
        late = 1.0 + obstacle.ROBOT_OBSERVATION_MAX_AGE + 0.10
        self.assertFalse(task.detect(line_frame(), now=late).valid)

    def test_empty_observation_clears_the_box(self):
        """这一帧 SDK 没识别到机器人 -> 必须清掉旧框。"""
        task = ObstacleTask()
        task.update_robot_observations([(320, 200, 200, 150)], observed_at=1.0)
        task.update_robot_observations([], observed_at=1.05)
        self.assertFalse(task.detect(line_frame(), now=1.06).valid)

    def test_too_far_robot_is_ignored(self):
        """SDK 框只占画面宽 3%（太远）-> 还不到要绕的距离。"""
        task = ObstacleTask()
        task.update_robot_observations([(320, 200, 19, 14)], observed_at=1.0)
        self.assertFalse(task.detect(line_frame(), now=1.05).valid)

    def test_robot_off_to_the_side_is_ignored(self):
        """识别到的机器人偏到画面边上（不挡路）-> 不接管。"""
        task = ObstacleTask()
        task.update_robot_observations([(40, 200, 200, 150)], observed_at=1.0)
        self.assertFalse(task.detect(line_frame(), now=1.05).valid)

    def test_nan_observation_is_dropped(self):
        task = ObstacleTask()
        task.update_robot_observations([(float("nan"), 200, 200, 150)], observed_at=1.0)
        self.assertEqual(task.detector._robot_rows, ())

    def test_sdk_robot_triggers_a_full_takeover(self):
        """端到端：SDK 说前方有车 + 蓝线已被挡住 -> 接管并侧移。"""
        task = ObstacleTask()
        now = 1.0
        seq = 0
        blank = np.full((360, 640, 3), 210, np.uint8)   # 线被挡住了（看不见）
        update = None
        for _ in range(30):
            seq += 1
            task.update_robot_observations([(320, 200, 200, 150)], observed_at=now)
            update = task.step(FramePacket(blank, seq, now), now)
            now += 0.05
            if update.status is TaskStatus.RUNNING:
                break
        self.assertEqual(update.status, TaskStatus.RUNNING, "SDK 报了车却没接管")
        self.assertNotEqual(update.motion.lateral, 0.0)
        self.assertIn(task.last_side, ("left", "right"))


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
        """实车实测场景的端到端回归：另一台车挡住线 -> 接管。"""
        self.assertEqual(self.harness.owner, "line")
        decision, _ = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
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
        """绕完 -> 主动找到线 -> COMPLETED -> 交回巡线 -> 恢复。

        v10 之后绕行途中不看画面，所以触发之后要喂**干净的线帧**
        （实车就是"绕过去、线又出现了"）。
        """
        _, now = self._feed_obstacle(1.05, obstacle.CONFIRM_FRAMES)
        completed = False
        for _ in range(300):
            decision = feed_image(self.harness, line_frame(), now)
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
        for _ in range(300):
            if task.stage != "IDLE":
                break
            seq += 1
            task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
        self.assertNotEqual(task.stage, "IDLE", "一直没接管")
        task.started_at -= obstacle.MAX_TOTAL_TIME + 1.0
        update = task.step(FramePacket(obstacle_frame(), seq + 1, now), now)
        self.assertEqual(update.status, TaskStatus.FAILED)
        self.assertEqual(update.motion.forward, 0.0)
        self.assertEqual(update.motion.lateral, 0.0)
        self.assertEqual(update.motion.yaw, 0.0)

    def test_restarts_cleanly_after_an_external_release(self):
        """协调器从外面把控制权拿走时**不会通知模块**，再被问到时不许从半路接着走。

        实车记录（2026-09-15）里"接管 0.1 秒后就 COMPLETED"就是这么造出来的：
        stage 停在半路、每段时间都早已过期，于是一帧跳一段把流程"走"完。

        【v8 修法】断档时**保留控制权**（RUNNING + 零速度，进入 RECONFIRM），
        下一帧重新判断；**绝不能返回 NOT_TRIGGERED** —— 实车记录里一共 12 次
        `obstacle returned NOT_TRIGGERED while owning motion` 就是从那条路径来的。
        """
        task = ObstacleTask()
        now = 1.0
        seq = 0
        for _ in range(300):
            if task.stage == "PASS":
                break
            seq += 1
            task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
        self.assertEqual(task.stage, "PASS", "没走到 PASS 段")

        # 模拟：中途被踢掉（视频中断 / 人工按键），2.6 秒之后才重新被问到
        now += 2.6
        seq += 40
        update = task.step(FramePacket(obstacle_frame(), seq, now), now)
        self.assertEqual(task.stage, "RECONFIRM")
        self.assertIsNot(
            update.status, TaskStatus.NOT_TRIGGERED,
            "断档之后返回了 NOT_TRIGGERED —— 协调器会判违约",
        )
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertEqual(update.motion.forward, 0.0)
        self.assertEqual(update.motion.lateral, 0.0)

        # 下一帧：障碍还在 -> 重新开始一轮完整的绕行
        now += obstacle.RECONFIRM_HOLD + 0.05
        seq += 1
        update = task.step(FramePacket(obstacle_frame(), seq, now), now)
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertEqual(task.stage, "OUT")

    def test_chooses_the_far_side_when_the_obstacle_is_off_centre(self):
        """PDF 要求"看清后自己选左或右"，所以不能永远往左。

        障碍明显偏左 -> 从右边绕（lateral 为正）；基本在正中间 -> 按默认往左。
        注意障碍要够大，才能既挡住近处的线、又偏在一侧。
        """
        # 障碍偏左：横向偏移 -0.31（还在居中门槛 0.5 内），并且挡住了线
        task = ObstacleTask()
        now = 1.0
        seq = 0
        update = None
        for _ in range(200):
            seq += 1
            update = task.step(
                FramePacket(obstacle_frame(center=(270, 220), size=(190, 160)), seq, now),
                now,
            )
            now += 0.05
            if update.status is TaskStatus.RUNNING:
                break
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertEqual(task.last_side, "right", "障碍偏左，应该从右边绕")
        self.assertGreater(update.motion.lateral, 0.0, "往右绕应该是正的 lateral")

        # 障碍在正中间 -> 默认往左
        task2 = ObstacleTask()
        now = 1.0
        seq = 0
        for _ in range(200):
            seq += 1
            update = task2.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
            if update.status is TaskStatus.RUNNING:
                break
        self.assertEqual(task2.last_side, "left", "正中间的障碍应该按默认往左")
        self.assertLess(update.motion.lateral, 0.0, "往左绕应该是负的 lateral")

    def test_a_stale_gap_hands_back_cleanly_when_the_obstacle_is_gone(self):
        """断档之后画面里已经没有障碍了：正常交回（COMPLETED），不是违约。"""
        task = ObstacleTask()
        now = 1.0
        seq = 0
        for _ in range(300):
            if task.stage == "PASS":
                break
            seq += 1
            task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
        self.assertEqual(task.stage, "PASS")

        now += 2.6
        seq += 40
        update = task.step(FramePacket(line_frame(), seq, now), now)
        self.assertIsNot(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertEqual(update.status, TaskStatus.RUNNING, "先停一脚")
        # 过了"停一脚"的时长再判断：画面里没车了，正常交回
        now += obstacle.RECONFIRM_HOLD + 0.05
        seq += 1
        update = task.step(FramePacket(line_frame(), seq, now), now)
        self.assertIsNot(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertEqual(update.status, TaskStatus.COMPLETED)
        self.assertEqual(task.stage, "IDLE")

    def test_never_completes_before_the_dodge_has_actually_run(self):
        """完整动作最短 1.7 + 1.6 + 1.2 = 4.5 秒，绝不允许更早报 COMPLETED。

        这条是上面那个实车假象的直接回归测试。
        """
        task = ObstacleTask()
        now = 1.0
        seq = 0
        takeover_at = None
        update = None
        triggered = False
        for _ in range(400):
            seq += 1
            image = line_frame() if triggered else obstacle_frame()
            update = task.step(FramePacket(image, seq, now), now)
            now += 0.05
            if takeover_at is None and update.status is TaskStatus.RUNNING:
                takeover_at = now
                triggered = True
            elif update.status is TaskStatus.COMPLETED:
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
        v10 之后每一轮都是"障碍挡住线 -> 绕 -> 线又出现"的完整循环。
        """
        task = ObstacleTask()
        now = 1.0
        seq = 0
        dodges = 0
        failed = False
        for _ in range(10):
            update, now, seq = run_one_dodge(task, now, seq)
            if update.status is TaskStatus.COMPLETED:
                dodges += 1
            elif update.status is TaskStatus.FAILED:
                failed = True
                break
            else:
                break

        self.assertTrue(failed, "连绕之后没有停下来，可能又在无限绕")
        self.assertLessEqual(
            dodges, obstacle.MAX_CONSECUTIVE_DODGES,
            "连绕次数超过了上限：%d 次" % dodges,
        )

        # 锁住之后：障碍还在，但必须不再接管
        locked_takeovers = 0
        for _ in range(600):                       # 再喂 30 秒
            seq += 1
            update = task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
            if update.status is TaskStatus.RUNNING:
                locked_takeovers += 1
        self.assertEqual(locked_takeovers, 0, "锁住之后又接管了，锁没生效")

    def test_unlocks_once_the_obstacle_clears(self):
        """障碍消失够久之后，应该重新愿意干活（不是永久瘫掉）。"""
        task = ObstacleTask()
        now = 1.0
        seq = 0
        for _ in range(10):
            update, now, seq = run_one_dodge(task, now, seq)
            if task.locked:
                break
        self.assertTrue(task.locked, "连绕到上限了却没锁住")
        self.assertEqual(task.dodge_count, obstacle.MAX_CONSECUTIVE_DODGES)

        for _ in range(50):
            if task.stage == "IDLE":
                break
            seq += 1
            task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
        self.assertEqual(task.stage, "IDLE", "锁停那一轮没有收尾")

        now += obstacle.CLEAR_SECONDS + 0.1
        seq += 1
        task.step(FramePacket(line_frame(), seq, now), now)
        self.assertEqual(task.dodge_count, 0)
        self.assertFalse(task.locked)

    def test_rearm_cooldown_blocks_an_immediate_second_takeover(self):
        """刚绕完的这段时间里，同一个障碍不许把车再拉去绕一次。"""
        task = ObstacleTask()
        now = 1.0
        seq = 0
        update, now, seq = run_one_dodge(task, now, seq)
        self.assertEqual(update.status, TaskStatus.COMPLETED)
        finished_at = task.finished_at
        self.assertIsNotNone(finished_at)

        # 冷却期内的障碍帧：不许接管
        for _ in range(obstacle.CONFIRM_FRAMES + 2):
            seq += 1
            update = task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
        self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertEqual(task.stage, "IDLE")

        # 冷却过去之后：同一个障碍应该重新能触发
        now = finished_at + obstacle.REARM_SECONDS + 0.1
        for _ in range(obstacle.CONFIRM_FRAMES):
            seq += 1
            update = task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertEqual(task.stage, "OUT")


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

    def test_startup_grace_can_suppress_takeover(self):
        """启动预热是可用的旋钮：设成 3 秒时，刚开始这段时间不许接管。

        默认是 0（关），因为闸四/闸五已经把实测那三帧挡掉了；
        要压制"一启动就避障"时把它设成 3.0 即可。
        """
        original = obstacle.STARTUP_GRACE_SECONDS
        obstacle.STARTUP_GRACE_SECONDS = 3.0
        try:
            task = ObstacleTask()
            seq = 0
            now = 1.0
            for _ in range(obstacle.CONFIRM_FRAMES + 2):
                seq += 1
                update = task.step(FramePacket(obstacle_frame(), seq, now), now)
                now += 0.05
                self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
            # 过了预热期就恢复正常
            now = 1.0 + 3.0 + 0.10
            for _ in range(obstacle.CONFIRM_FRAMES):
                seq += 1
                update = task.step(FramePacket(obstacle_frame(), seq, now), now)
                now += 0.05
            self.assertEqual(update.status, TaskStatus.RUNNING)
        finally:
            obstacle.STARTUP_GRACE_SECONDS = original

    def test_seek_moves_back_towards_the_line_and_fails_without_it(self):
        """绕完线不见了：必须主动往回挪去找，找不到就停车报失败。"""
        task = ObstacleTask()
        now = 1.0
        seq = 0
        for _ in range(300):
            if task.stage == "SEEK":
                break
            seq += 1
            task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
            self.assertLess(now - 1.0, obstacle.MAX_TOTAL_TIME)
        self.assertEqual(task.stage, "SEEK", "没走到 SEEK 段")

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
        for _ in range(300):
            if task.stage == "SEEK":
                break
            seq += 1
            task.step(FramePacket(obstacle_frame(), seq, now), now)
            now += 0.05
        self.assertEqual(task.stage, "SEEK", "没走到 SEEK 段")
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
