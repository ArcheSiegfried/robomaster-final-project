"""free_junction 的官方观测**接线回归**：机器人识别进得来、视觉标签进不来。

为什么单独钉这一组：王炜嘉 final3.1 把"官方 SDK 读数"接在了 `update_candidates()`
上（`main.py` 的 `feed_marker_observations()` 会给任何实现了该名字的任务推数据）。
但那条通道在本仓库里送的是 SDK 的**视觉标签(marker)** —— 墙上的标签会被当成
"这条分支堵着一辆车"，正好是这一轮要修的那类误报。机器人识别走的是另一条通道
（`robot_source.py` → `feed_robot_observations()` → `update_robot_observations()`，
和 5 号 `obstacle.py` 同一条），2026-09-17 的接口适配把它改接对了。

这组测试保证：
  1. **机器人识别到得了它**：`feed_robot_observations()` 推的框进快照，并且判据据此选对分支；
  2. **视觉标签到不了它**：`feed_marker_observations()` 推的 marker 候选不进快照；
  3. **接口名字不能退化**：模块必须实现 `update_robot_observations`，且**不再**实现
     `update_candidates`（否则标签又会漏进来）。
"""

import pathlib
import sys
import time
import unittest
from types import SimpleNamespace

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import main  # noqa: E402
from free_junction import FreeJunctionConfig, FreeJunctionTask  # noqa: E402
from models import FramePacket, TaskStatus  # noqa: E402
from number_marker import MarkerCandidate  # noqa: E402
from tests.test_free_junction import fork_frame  # noqa: E402


class FakeMarkerSource:
    """冒充 SDK marker 订阅：给出一个落在左分支走廊里的**视觉标签**。"""

    def candidates(self, frame, now):
        return (
            MarkerCandidate(
                target_id="5",
                center=(0.20 * 640.0, 0.40 * 360.0),
                width=0.18 * 640.0,
                height=0.26 * 360.0,
                observed_at=now,
            ),
        )


class FakeRobotSource:
    """冒充 SDK 机器人识别订阅：给一个左分支走廊里的**车框**（归一化 x,y,w,h）。"""

    def __init__(self, rows):
        self._rows = rows

    def observations(self, frame):
        return self._rows, time.monotonic()


class WiringTests(unittest.TestCase):
    def test_module_exposes_the_robot_entry_and_not_the_marker_one(self):
        task = FreeJunctionTask()
        self.assertTrue(
            callable(getattr(task, "update_robot_observations", None)),
            "free_junction 必须实现 update_robot_observations（机器人识别通道）",
        )
        self.assertFalse(
            hasattr(task, "update_candidates"),
            "不许再实现 update_candidates：那是视觉标签通道，标签会被当成车",
        )

    def test_robot_observations_reach_the_module(self):
        task = FreeJunctionTask()
        coordinator = SimpleNamespace(motion_tasks=(task,))
        source = FakeRobotSource(((0.20, 0.40, 0.18, 0.26),))
        fed = main.feed_robot_observations(
            coordinator, source, FramePacket(fork_frame(), 1, 1.0), 1.0
        )
        self.assertEqual(fed, 1, "机器人识别没有喂到 free_junction")
        self.assertEqual(len(task._sdk_rows), 1)

    def test_marker_tags_do_not_reach_the_module(self):
        task = FreeJunctionTask()
        coordinator = SimpleNamespace(motion_tasks=(task,))
        fed = main.feed_marker_observations(
            coordinator, FakeMarkerSource(), FramePacket(fork_frame(), 1, 1.0), 1.0
        )
        self.assertEqual(fed, 0, "视觉标签不该被喂给 free_junction")
        self.assertEqual(task._sdk_rows, (), "视觉标签进了快照 —— 会被当成车")

    def test_official_robot_reading_chooses_the_other_branch(self):
        """端到端：机器人识别在左分支报出一辆车，画面里没画车 -> 走右边。

        注意时间基：`robot_source` 打的是**单调钟**时间戳，主循环的 `now` 也是单调钟
        （`main.py` 里 `now = time.monotonic()`），所以这里必须用同一个钟 —— 否则观测会被
        判成"过期"（age < 0）而丢掉，跟真实运行不是一回事。
        """
        task = FreeJunctionTask(
            FreeJunctionConfig(blockage_source="sdk_or_vision")
        )
        coordinator = SimpleNamespace(motion_tasks=(task,))
        source = FakeRobotSource(((0.20, 0.40, 0.18, 0.26),))
        image = fork_frame()
        now = time.monotonic()
        status = None
        for index in range(200):
            packet = FramePacket(image, index + 1, now)
            main.feed_robot_observations(coordinator, source, packet, now)
            update = task.step(packet, now)
            status = update.status
            now += 0.05
            if status in (TaskStatus.COMPLETED, TaskStatus.FAILED):
                break
        self.assertIs(status, TaskStatus.COMPLETED)
        self.assertEqual(getattr(task.chosen_branch, "value", None), "right")

    def test_marker_tags_alone_never_take_over(self):
        """只有视觉标签、没有机器人识别时：不该接管（标签不是车）。"""
        task = FreeJunctionTask()
        coordinator = SimpleNamespace(motion_tasks=(task,))
        image = fork_frame()
        now = 1.0
        for index in range(60):
            packet = FramePacket(image, index + 1, now)
            main.feed_marker_observations(coordinator, FakeMarkerSource(), packet, now)
            update = task.step(packet, now)
            self.assertIs(update.status, TaskStatus.NOT_TRIGGERED)
            now += 0.05


if __name__ == "__main__":
    unittest.main()
