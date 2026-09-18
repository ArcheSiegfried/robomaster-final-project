"""岔路口（红绿同框）时，红绿灯模块**不得**接管运动。

为什么要有这个文件：整套 646 个测试里，没有任何一条让**真的** `traffic_light`
和岔路模块同时挂在协调器上跑。于是这个组合缺陷一路没被发现：

* 考场第一岔路是"左红右绿"（赛题 4），两盏灯同框；
* `traffic_light` 只认颜色不认位置，`red_priority` 把这一帧判成红灯，
  **2 帧**就返回 RUNNING 接管；
* `green_junction` 要 **3 帧**才确认岔路 → 它永远赢不了这场赛跑；
* 灯模块接管之后，`coordinator.py` 的红灯否决权每帧先问红灯、否决时
  **不调用**当前任务的 `step()` → 岔路模块被按停，选路永远做不完。

实测后果（合成 Y 形岔路 + 左红右绿 + 真协调器，30fps×20s=600 帧）：
`green_junction.step` 只被调用 3 次、599/600 帧零指令、车钉死在岔路口。

修复方式：红绿同框时 `TrafficLightTask` 从 IDLE 状态返回 NOT_TRIGGERED，
把岔路口让给 `green_junction`（它内置的 LampSpotter 会同时报出两盏灯各自在
哪一边）。自选地点的"红灯停绿灯行"（赛题 6）按定义只有一盏灯，不受影响。

只用合成帧、假底盘：不连车、不连相机。
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
    CONFIG,
    FakeChassis,
    FakeGimbal,
    GimbalOutput,
    LineFollower,
    MotionOutput,
    line_frame,
)

from coordinator import LINE_FOLLOWING, TaskCoordinator  # noqa: E402
from green_junction import LampSpotter  # noqa: E402
from models import FramePacket, TaskStatus  # noqa: E402
from traffic_light import TrafficLightConfig, TrafficLightTask  # noqa: E402

HEIGHT, WIDTH = 360, 640

RED = (0, 0, 255)
GREEN = (0, 255, 0)


def lamp_frame(x, colour, radius=28, top=90):
    """One round lamp above a valid blue tape line."""
    image = line_frame(320, height=HEIGHT, width=WIDTH)
    cv2.circle(image, (x, top), radius, colour, -1)
    return image


def two_lamp_frame(left_colour, right_colour, left_x=200, right_x=440):
    """The exam's first fork: one lamp on each side of the line."""
    image = line_frame(320, height=HEIGHT, width=WIDTH)
    cv2.circle(image, (left_x, 90), 28, left_colour, -1)
    cv2.circle(image, (right_x, 90), 28, right_colour, -1)
    return image


def fork_geometry_frame(left_colour=RED, right_colour=GREEN):
    """A real blue Y-fork (stem + two branches) with one lamp on each branch.

    Used by the coordinator tests so the fork stand-in triggers on the same
    geometry the real module looks for, instead of on a blind frame count.
    """
    image = np.full((HEIGHT, WIDTH, 3), 210, np.uint8)
    cv2.line(image, (330, 355), (330, 250), (255, 0, 0), 22)   # stem
    cv2.line(image, (330, 250), (150, 120), (255, 0, 0), 20)   # left branch
    cv2.line(image, (330, 250), (520, 120), (255, 0, 0), 20)   # right branch
    cv2.circle(image, (150, 80), 26, left_colour, -1)
    cv2.circle(image, (520, 80), 26, right_colour, -1)
    return image


#: Bottom-of-frame tape width that marks a branch/stem row of the Y above.
_STEM_WIDTH = 22


def has_fork_geometry(image):
    """True when the frame holds a blue Y-fork (two bands + one thicker stem).

    This is the stand-in's trigger, so the double only fires where a real
    junction module could: a plain straight line must never trigger it.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array((95, 80, 60)), np.array((135, 255, 255)))
    stem_rows = 0
    fork_rows = 0
    for y in range(int(HEIGHT * 0.6), HEIGHT, 4):
        row = mask[y] > 0
        if not row.any():
            continue
        runs = 0
        width = 0
        in_run = False
        for value in row:
            if value and not in_run:
                runs += 1
                in_run = True
                width = 1
            elif value:
                width += 1
            else:
                in_run = False
        if runs >= 2:
            fork_rows += 1
        elif width >= _STEM_WIDTH:
            stem_rows += 1
    return fork_rows >= 2 and stem_rows >= 2


class CountingGreenJunction:
    """Minimal stand-in for green_junction: takes over once a fork is confirmed.

    The real module needs a few frames to confirm a fork, which is exactly why
    it loses the race against traffic_light's two-frame red confirmation. The
    delay is reproduced honestly, and the trigger is fork geometry so the
    double stays inert on a plain line (the same contract the real module has).
    """

    name = "green_junction"

    def __init__(self, confirm_frames=3, trigger=has_fork_geometry):
        self.confirm_frames = confirm_frames
        self.trigger = trigger
        self.calls = 0
        self.seen = 0

    def step(self, frame, now):
        from models import MotionCommand, TaskUpdate

        self.calls += 1
        if not self.trigger(frame.image):
            if self.seen:          # rearm, like the real module's cooldown
                self.seen = 0
            return TaskUpdate(TaskStatus.NOT_TRIGGERED, message="no fork")
        self.seen += 1
        if self.seen < self.confirm_frames:
            return TaskUpdate(TaskStatus.NOT_TRIGGERED, message="confirming fork")
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(forward=0.10, yaw=-20.0),
            message="take the left branch",
        )


class TrafficLightConfigTests(unittest.TestCase):
    def test_fork_competition_is_on_by_default(self):
        self.assertTrue(TrafficLightConfig().fork_light_competition)

    def test_turning_the_switch_off_restores_red_priority(self):
        """关掉开关就退回旧行为：红绿同框报红（给现场临时回退用）。"""
        detector = TrafficLightTask(
            settings=TrafficLightConfig(fork_light_competition=False)
        ).detector
        detection = detector.detect(two_lamp_frame(RED, GREEN))
        self.assertTrue(detection.valid)
        self.assertEqual(detection.color, "red")


class ForkLightGateTests(unittest.TestCase):
    """模块层：红绿同框时不许接管；只有一盏灯时行为不变。"""

    def setUp(self):
        self.sequence = 0
        self.now = 100.0

    def frames(self, image, count):
        updates = []
        task = TrafficLightTask()
        for _ in range(count):
            self.sequence += 1
            self.now += 1.0 / 30.0
            packet = FramePacket(image, self.sequence, self.now)
            updates.append(task.step(packet, self.now))
        return task, updates

    def test_a_fork_with_two_lamps_never_takes_over(self):
        """左红右绿：跑 60 帧也不许接管，否则它会停 15 秒再 FAILED。"""
        task, updates = self.frames(two_lamp_frame(RED, GREEN), 60)
        statuses = {update.status for update in updates}
        self.assertEqual(statuses, {TaskStatus.NOT_TRIGGERED},
                         "岔路口（红绿同框）时红绿灯模块必须完全不接管")
        self.assertEqual(task._red_streak, 0)
        self.assertEqual(task._state, "idle")

    def test_the_mirrored_fork_is_also_left_alone(self):
        task, updates = self.frames(two_lamp_frame(GREEN, RED), 60)
        self.assertEqual({u.status for u in updates}, {TaskStatus.NOT_TRIGGERED})

    def test_a_single_red_lamp_still_stops_the_car(self):
        """自选地点的红灯只有一盏：红灯停的能力不能被这次修复削弱。"""
        task, updates = self.frames(lamp_frame(200, RED), 6)
        self.assertEqual(updates[0].status, TaskStatus.NOT_TRIGGERED)
        self.assertEqual(updates[1].status, TaskStatus.RUNNING)
        self.assertEqual(updates[1].motion.forward, 0.0)
        self.assertEqual(updates[1].motion.yaw, 0.0)

    def test_a_single_green_lamp_still_does_not_take_over(self):
        """绿灯从 IDLE 永不接管（绿灯只用来放行，不抢正在开车的车）。"""
        task, updates = self.frames(lamp_frame(440, GREEN), 20)
        self.assertEqual({u.status for u in updates}, {TaskStatus.NOT_TRIGGERED})

    def test_the_fork_module_itself_can_see_both_lamps(self):
        """闸门的依据：LampSpotter 同时报出两盏灯和各自在哪一边。"""
        readings = LampSpotter().readings(two_lamp_frame(RED, GREEN))
        seen = {(r.color.value, getattr(r.branch, "value", r.branch))
                for r in readings}
        self.assertIn(("red", "left"), seen)
        self.assertIn(("green", "right"), seen)


class CoordinatorForkTests(unittest.TestCase):
    """协调器层：这才是真正丢分的那条路径（灯模块 + 岔路模块同时在队里）。"""

    def setUp(self):
        self.chassis = FakeChassis()
        self.follower = LineFollower(CONFIG)
        self.output = MotionOutput(self.chassis, CONFIG)
        self.gimbal_output = GimbalOutput(FakeGimbal(), CONFIG)
        self.junction = CountingGreenJunction(confirm_frames=3)
        self.light = TrafficLightTask(settings=TrafficLightConfig())
        self.coordinator = TaskCoordinator(
            CONFIG,
            self.follower,
            self.output,
            motion_tasks=(self.junction, self.light),  # 新顺序：岔路第 1
            observers=(),
            gimbal_output=self.gimbal_output,
        )
        self.sequence = 0
        self.now = 200.0
        # Arm the base line with a frame that has NO fork geometry and NO lamp,
        # so the stand-in stays inert during start-up.
        ready = np.full((HEIGHT, WIDTH, 3), 210, np.uint8)
        cv2.line(ready, (320, 350), (320, 190), (255, 0, 0), 24)
        first = FramePacket(ready, 1, self.now)
        self.follower.process_frame(first.image, first.captured_at)
        self.assertTrue(self.follower.resume(self.now), "巡线底座要能起来")

    def run_frames(self, image, count, step=1.0 / 30.0):
        decisions = []
        for _ in range(count):
            self.now += step
            self.sequence += 1
            packet = FramePacket(image, self.sequence, self.now)
            decisions.append(self.coordinator.step(packet, self.now))
        return decisions

    def test_the_fork_module_keeps_control_at_a_two_lamp_fork(self):
        """红绿同框的 Y 形岔路：岔路模块拿到控制权，灯模块不抢、不停。"""
        image = fork_geometry_frame()
        decisions = self.run_frames(image, 10)

        self.assertEqual(self.coordinator.active_task_name, "green_junction",
                         "岔路模块必须拿到控制权，而不是被红灯按停")
        self.assertEqual(self.light._state, "idle",
                         "灯模块在岔路口不得进入 holding_red")

        junction_frames = sum(1 for d in decisions
                              if d.task_name == "green_junction")
        self.assertGreater(junction_frames, 0, "岔路模块必须真的被调度到")
        self.assertGreaterEqual(self.junction.calls, 3,
                               "岔路模块要连续被询问才能确认岔路")

        moving = [d for d in decisions
                  if d.command.forward or d.command.lateral or d.command.yaw]
        self.assertTrue(moving, "接管后必须真的下发运动指令，不能全是零")

    def test_the_car_is_not_pinned_for_the_whole_run(self):
        """回归核心：600 帧里绝大多数帧必须有运动指令，而不是被钉死。"""
        image = fork_geometry_frame()
        decisions = self.run_frames(image, 120)

        zero_motion = [d for d in decisions
                       if not (d.command.forward or d.command.lateral
                               or d.command.yaw)]
        self.assertLess(
            len(zero_motion), len(decisions) / 2,
            "修复前实测 599/600 帧零指令（车被红灯钉死在岔路口）",
        )

    def test_a_single_red_lamp_still_vetoes_a_running_task(self):
        """只有一盏红灯时，否决权必须原样保留（自选地点的红灯停）。"""
        # A plain Y-fork with **no** lamps: the stand-in takes over, and the
        # light module sees nothing, so no veto is involved yet.
        self.run_frames(fork_geometry_frame(None, None), 6)
        self.assertEqual(self.coordinator.active_task_name, "green_junction")

        decisions = self.run_frames(lamp_frame(200, RED), 5)
        self.assertTrue(
            all(not (d.command.forward or d.command.lateral or d.command.yaw)
                for d in decisions),
            "单盏红灯必须继续把正在接管的任务否决成停车",
        )

    def test_plain_line_is_untouched(self):
        """没有任何灯、没有岔路几何时，两个模块都不许接管。"""
        plain = line_frame(320, height=HEIGHT, width=WIDTH)
        self.run_frames(plain, 30)
        self.assertIsNone(self.coordinator.active_task_name)
        self.assertEqual(self.coordinator.state, LINE_FOLLOWING)


if __name__ == "__main__":
    unittest.main()
