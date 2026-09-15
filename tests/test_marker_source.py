"""marker 观测来源（集成层）的离线测试。

全部离线：假 vision 对象 + 合成帧，不连 SDK、不连相机、不连车。
验证的是"SDK 回调 → 带接收时间的快照 → 任务要的候选"这条链，
以及失败时的安全降级（订阅失败 = 模块不触发，绝不影响巡线）。
"""

import pathlib
import sys
import time
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from models import FramePacket  # noqa: E402
from tests.task_harness import blank_frame  # noqa: E402

import main  # noqa: E402
from marker_source import MIN_USEFUL_HZ, MarkerObservationSource  # noqa: E402

WIDTH, HEIGHT = 640, 360


def packet(sequence=1, now=1.0):
    return FramePacket(blank_frame(HEIGHT, WIDTH), sequence, now)


class FakeVision:
    """只记录订阅行为，不碰任何硬件。"""

    def __init__(self, result=True, raises=False):
        self.result = result
        self.raises = raises
        self.subscriptions = []
        self.unsubscribed = []
        self.callback = None

    def sub_detect_info(self, name=None, color=None, callback=None, **kwargs):
        if self.raises:
            raise RuntimeError("synthetic vision failure")
        self.subscriptions.append({"name": name, "color": color})
        self.callback = callback
        return self.result

    def unsub_detect_info(self, name=None):
        self.unsubscribed.append(name)
        return True

    def emit(self, marker_info):
        self.callback(marker_info)


class _PushTask:
    """只实现 update_candidates 的假任务。"""

    def __init__(self):
        self.received = []

    def update_candidates(self, candidates):
        self.received.append(tuple(candidates))


class _Coordinator:
    def __init__(self, *tasks):
        self.motion_tasks = tasks


class MarkerObservationSourceTests(unittest.TestCase):
    def test_no_vision_is_a_safe_no_op(self):
        source = MarkerObservationSource(None)
        self.assertFalse(source.start())
        self.assertFalse(source.enabled)
        self.assertEqual(source.candidates(packet(), 1.0), ())

    def test_subscribes_to_marker_and_unsubscribes(self):
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        self.assertTrue(source.start())
        self.assertTrue(source.enabled)
        self.assertEqual(vision.subscriptions[0]["name"], "marker")
        self.assertIsNone(vision.subscriptions[0]["color"])
        source.stop()
        self.assertFalse(source.enabled)
        self.assertEqual(vision.unsubscribed, ["marker"])

    def test_configured_color_is_passed_to_the_sdk(self):
        vision = FakeVision()
        source = MarkerObservationSource(vision, color="green")
        source.start()
        self.assertEqual(vision.subscriptions[0]["color"], "green")

    def test_subscribe_failure_degrades_to_inert(self):
        vision = FakeVision(result=False)
        source = MarkerObservationSource(vision)
        self.assertFalse(source.start())
        self.assertFalse(source.enabled)
        self.assertEqual(source.candidates(packet(), 1.0), ())

    def test_subscribe_exception_never_propagates(self):
        vision = FakeVision(raises=True)
        source = MarkerObservationSource(vision)
        self.assertFalse(source.start())
        self.assertEqual(source.candidates(packet(), 1.0), ())

    def test_normalized_callback_becomes_full_frame_pixels(self):
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        source.start()
        vision.emit([(0.25, 0.50, 0.30, 0.20, "3")])

        candidates = source.candidates(packet(), 42.0)
        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].target_id, "3")
        self.assertEqual(candidates[0].center, (160.0, 180.0))
        self.assertAlmostEqual(candidates[0].width, 192.0)
        self.assertEqual(source.stats()["coordinate_mode"], "normalized")

    def test_pixel_callback_is_auto_detected(self):
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        source.start()
        vision.emit([(320.0, 180.0, 160.0, 100.0, "2")])

        candidates = source.candidates(packet(), 42.0)
        self.assertEqual(candidates[0].center, (320.0, 180.0))
        self.assertEqual(candidates[0].width, 160.0)
        self.assertEqual(source.stats()["coordinate_mode"], "pixels")

    def test_explicit_coordinate_mode_overrides_autodetect(self):
        vision = FakeVision()
        source = MarkerObservationSource(vision, coordinate_mode="pixels")
        source.start()
        vision.emit([(0.25, 0.50, 0.30, 0.20, "1")])
        candidates = source.candidates(packet(), 42.0)
        self.assertEqual(candidates[0].center, (0.25, 0.50))

    def test_observation_keeps_the_callback_time_not_the_frame_time(self):
        """模块靠 observed_at 判过期，所以这里绝不能刷成这一帧的时刻。"""
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        source.start()
        before = time.monotonic()
        vision.emit([(0.5, 0.5, 0.3, 0.2, "1")])
        candidates = source.candidates(packet(now=999.0), 999.0)
        self.assertGreaterEqual(candidates[0].observed_at, before)
        self.assertNotEqual(candidates[0].observed_at, 999.0)
        self.assertLess(
            abs(candidates[0].observed_at - time.monotonic()), 1.0,
            "observed_at 应该是回调的接收时刻（同一时钟），不是这一帧的 now",
        )

    def test_empty_callback_clears_the_snapshot(self):
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        source.start()
        vision.emit([(0.5, 0.5, 0.3, 0.2, "1")])
        self.assertEqual(len(source.candidates(packet(), 1.0)), 1)
        vision.emit([])
        self.assertEqual(source.candidates(packet(), 1.0), ())
        self.assertEqual(source.stats()["empty_callbacks"], 1)

    def test_other_ids_are_passed_through_untouched(self):
        """筛选是 number_marker 的职责，这里不许自作主张。"""
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        source.start()
        vision.emit([(0.5, 0.5, 0.3, 0.2, "9")])
        self.assertEqual(source.candidates(packet(), 1.0)[0].target_id, "9")

    def test_broken_rows_are_dropped_without_raising(self):
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        source.start()
        vision.emit([(0.5, 0.5), (float("nan"), 0.5, 0.3, 0.2, "1"), ("a", "b", "c", "d", "2")])
        self.assertEqual(source.candidates(packet(), 1.0), ())

    def test_candidates_need_a_frame_with_an_image(self):
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        source.start()
        vision.emit([(0.5, 0.5, 0.3, 0.2, "1")])
        self.assertEqual(source.candidates(object(), 1.0), ())
        self.assertEqual(source.candidates(FramePacket(None, 1, 1.0), 1.0), ())

    def test_stats_and_rate_warning(self):
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        source.start()
        self.assertIn("还没有收到", source.rate_warning())
        for _ in range(6):
            vision.emit([(0.5, 0.5, 0.3, 0.2, "1")])
            time.sleep(0.01)
        stats = source.stats()
        self.assertEqual(stats["callbacks"], 6)
        self.assertIsNotNone(stats["callback_hz"])
        self.assertEqual(stats["markers_in_snapshot"], 1)
        self.assertLess(MIN_USEFUL_HZ, 100.0)


class FeedMarkerObservationsTests(unittest.TestCase):
    """主循环 -> 任务 的推送通道。"""

    def _source(self, marker_info=(0.25, 0.50, 0.30, 0.20, "3")):
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        source.start()
        vision.emit([marker_info])
        return source

    def test_pushes_fresh_candidates_to_the_task(self):
        task = _PushTask()
        fed = main.feed_marker_observations(
            _Coordinator(task), self._source(), packet(), 1.0
        )
        self.assertEqual(fed, 1)
        self.assertEqual(len(task.received), 1)
        self.assertEqual(task.received[0][0].target_id, "3")
        self.assertEqual(task.received[0][0].center, (160.0, 180.0))

    def test_pushes_an_empty_tuple_when_there_is_no_observation(self):
        """没有观测时必须推空，否则任务会拿着上一帧的旧目标。"""
        task = _PushTask()
        vision = FakeVision()
        source = MarkerObservationSource(vision)
        source.start()
        main.feed_marker_observations(_Coordinator(task), source, packet(), 1.0)
        self.assertEqual(task.received, [()])

    def test_tasks_without_the_hook_are_skipped(self):
        class _Plain:
            name = "plain"

        fed = main.feed_marker_observations(
            _Coordinator(_Plain()), self._source(), packet(), 1.0
        )
        self.assertEqual(fed, 0)

    def test_no_source_is_a_no_op(self):
        task = _PushTask()
        self.assertEqual(
            main.feed_marker_observations(_Coordinator(task), None, packet(), 1.0), 0
        )
        self.assertEqual(task.received, [])


if __name__ == "__main__":
    unittest.main()
