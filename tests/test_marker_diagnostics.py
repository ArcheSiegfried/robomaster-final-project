"""marker 候选宽度诊断：回答"为什么看到了标识却不接管"。

背景（2026-09-16 实车）：SDK marker 订阅完全正常（回调 23Hz、快照里有 1 个 marker），
但 `number_marker` 一次接管都没有 —— 它要求"标记宽度 > 画面宽 20%"，不满足就
`TARGET_TOO_SMALL` 保持不接管（不拍照、不停车）。原来的诊断只记"快照里有几个 marker"，
**不记它到底有多宽**，所以无法区分"车太远"还是"SDK 报的宽度不是卡片宽度"。

这里锁死：诊断要记录本次运行看到过的最宽候选（宽度比 + 目标号），并且当它一直小于
模块门槛时给出人话提醒。

全部离线：假 vision 对象 + 合成帧，不连 SDK、不连相机、不连车。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from models import FramePacket  # noqa: E402
from tests.task_harness import blank_frame  # noqa: E402

from marker_source import MarkerObservationSource  # noqa: E402

WIDTH, HEIGHT = 640, 360


def packet(sequence=1, now=1.0):
    return FramePacket(blank_frame(HEIGHT, WIDTH), sequence, now)


class FakeVision:
    def __init__(self):
        self.callback = None

    def sub_detect_info(self, name=None, color=None, callback=None, **kwargs):
        self.callback = callback
        return True

    def unsub_detect_info(self, name=None):
        return True

    def emit(self, marker_info):
        self.callback(marker_info)


def source_with(rows):
    """建一个已订阅、且刚收到过一条回调的观测源。"""
    vision = FakeVision()
    source = MarkerObservationSource(vision, "", "auto")
    source.start()
    vision.emit(rows)
    return source


class WidthDiagnosticsTests(unittest.TestCase):
    def test_stats_report_the_widest_candidate_seen(self):
        source = source_with([(0.5, 0.5, 0.10, 0.10, 5)])
        source.candidates(packet(), 1.0)
        stats = source.stats()
        self.assertAlmostEqual(stats["max_width_ratio"], 0.10, places=3)
        self.assertIn("5", str(stats["widest_target"]))

    def test_stats_keep_the_widest_over_the_run_not_the_last(self):
        source = source_with([(0.5, 0.5, 0.30, 0.20, 2)])
        source.candidates(packet(), 1.0)
        vision = FakeVision()
        source._vision = vision  # 换一个假 vision，模拟"后面又收到一条更小的"
        vision.callback = None
        source.subscribe_result = True
        source._marker_info = [(0.5, 0.5, 0.05, 0.05, 3)]
        source._received_at = 2.0
        source.candidates(packet(2, 2.0), 2.0)
        self.assertAlmostEqual(source.stats()["max_width_ratio"], 0.30, places=3,
                               msg="要记本次运行看到过的最宽候选，不是最后一条")

    def test_warning_points_at_the_width_gate_when_too_small(self):
        source = source_with([(0.5, 0.5, 0.10, 0.10, 5)])
        source.candidates(packet(), 1.0)
        warning = source.rate_warning()
        self.assertTrue(warning, "宽度一直不够时必须给出提醒")
        self.assertIn("20", warning, "要写出模块要求的是画面宽的 20%")
        self.assertIn("TARGET_TOO_SMALL", warning)

    def test_no_width_warning_when_a_big_enough_candidate_was_seen(self):
        source = source_with([(0.5, 0.5, 0.35, 0.25, 5)])
        source.candidates(packet(), 1.0)
        self.assertNotIn("TARGET_TOO_SMALL", source.rate_warning())

    def test_stats_are_safe_before_any_callback(self):
        vision = FakeVision()
        source = MarkerObservationSource(vision, "", "auto")
        source.start()
        stats = source.stats()
        self.assertIsNone(stats["max_width_ratio"])
        self.assertEqual(stats["observed_candidates"], 0)


if __name__ == "__main__":
    unittest.main()
