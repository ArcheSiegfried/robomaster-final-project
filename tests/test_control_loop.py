"""控制循环时序的两个回归锁（都是实车踩出来的）。

1. **正常的一帧间隔不能被当成"视频中断"把正在执行的任务踢掉。**
   实车教训：main 只要这一轮没等到新帧就会调 `coordinator.video_gap()`，
   而它原来**无条件**结束任务。实测（tools/probe_video_gap.py）：等帧超时 12ms、
   相机 30fps 时 **75% 的循环**都会走到那里 → 每次任务接管都在 0.1 秒内被踢掉，
   没有一个模块能跑完（traffic_light 接管 7 次、free_junction 接管 20 次，全部如此）。

2. `consumer_wait_timeout` 必须 >= 相机一帧的时长，否则第 1 条会持续触发。

不是实车结论，离线即可复现。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import CONFIG  # noqa: E402
from coordinator import TASK_ACTIVE  # noqa: E402
from models import MotionCommand, TaskStatus, TaskUpdate  # noqa: E402
from tests.task_harness import TaskHarness  # noqa: E402

#: 30fps 相机一帧的时长。
CAMERA_FRAME_SECONDS = 1.0 / 30.0


class _AlwaysRunning:
    """接管后就一直 RUNNING 的任务（模拟一次正常执行中的模块）。"""

    name = "always_running"

    def step(self, frame, now):
        return TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand())


class ControlLoopTimingTests(unittest.TestCase):
    def _take_over(self):
        harness = TaskHarness(task=_AlwaysRunning())
        harness.start_line(now=1.0)
        decision = harness.coordinator.step(harness.frame(1.01), 1.01)
        self.assertEqual(decision.state, TASK_ACTIVE)
        self.assertEqual(harness.owner, "external")
        return harness

    def test_a_short_video_gap_does_not_end_the_task(self):
        harness = self._take_over()
        short = harness.coordinator.video_gap(CAMERA_FRAME_SECONDS / 3.0, 1.02)
        self.assertEqual(harness.state, TASK_ACTIVE, "几十毫秒的间隔不该结束任务")
        self.assertEqual(harness.owner, "external", "控制权不该被交回去")
        self.assertNotIn("ended by video gap", short.message)

    def test_a_real_video_loss_still_ends_the_task(self):
        harness = self._take_over()
        real = harness.coordinator.video_gap(
            CONFIG.video_gap_stop_seconds + 0.01, 1.30)
        self.assertEqual(harness.owner, "line", "视频真的断了必须结束任务")
        self.assertEqual(harness.state, "LINE_FOLLOWING")
        self.assertIn("ended by video gap", real.message)

    def test_consumer_wait_timeout_covers_a_camera_frame(self):
        self.assertGreaterEqual(
            CONFIG.consumer_wait_timeout, CAMERA_FRAME_SECONDS,
            "等新帧的时间不能短于相机一帧（30fps → 33ms），"
            "否则正常帧间隔会被当成视频中断，任何任务都活不过一帧",
        )


if __name__ == "__main__":
    unittest.main()
