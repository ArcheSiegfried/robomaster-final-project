"""截图标注与任务证据（基础设施，不占功能模块名额）。

这是观察型模块：每帧都会被调用，但**永远不能接管运动**。
契约测试在前，下面是功能测试。全部离线：临时目录 + 合成帧，不连相机、不连车。
"""

import csv
import json
import pathlib
import sys
import tempfile
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from models import FramePacket  # noqa: E402
from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
    line_frame,
)

import evidence  # noqa: E402
from evidence import EvidenceRecorder  # noqa: E402


class _BrokenWriter:
    """一个写就报错的 writer，用来验证"写盘失败不影响控制循环"。"""

    def writerow(self, row):
        raise OSError("disk full")

    def writeheader(self):
        raise OSError("disk full")


class EvidenceContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "evidence.py")

    def test_recorder_never_takes_over(self):
        """记录模块没有运动权限，任何帧都不许接管。"""
        assert_inert_through_harness(self, EvidenceRecorder(), observer=True)

    def test_recorder_never_takes_over_even_when_it_writes(self):
        """即使真的在写盘，也绝不允许接管运动。"""
        with tempfile.TemporaryDirectory() as tmp:
            recorder = EvidenceRecorder(directory=tmp)
            try:
                assert_inert_through_harness(self, recorder, observer=True)
            finally:
                recorder.close()

    def test_recorder_is_called_on_every_frame(self):
        """骨架每帧都会调 observe()，包括别的任务正在接管时。"""
        recorder = EvidenceRecorder()
        harness = TaskHarness(observer=recorder)
        harness.start_line(now=1.0)
        for step in range(4):
            harness.feed_line(1.10 + step * 0.05, x=320 + step * 3)
        self.assertEqual(harness.owner, "line")


class EvidenceDisabledTests(unittest.TestCase):
    """不带目录 = 安全的空操作，不能有任何副作用。"""

    def test_disabled_by_default(self):
        recorder = EvidenceRecorder()
        self.assertFalse(recorder.enabled)
        self.assertIsNone(recorder.run_directory)

    def test_disabled_recorder_writes_nothing(self):
        before = set(pathlib.Path(ROOT).rglob("run_*"))
        recorder = EvidenceRecorder()
        for index in range(20):
            recorder.observe(
                FramePacket(line_frame(), index + 1, 1.0 + index * 0.05),
                1.0 + index * 0.05,
            )
        recorder.close()
        after = set(pathlib.Path(ROOT).rglob("run_*"))
        self.assertEqual(before, after, "关闭状态下不应产生任何运行目录")
        self.assertEqual(recorder.snapshots, 0)
        self.assertEqual(recorder.rows_written, 0)

    def test_bad_directory_disables_the_recorder_without_raising(self):
        """目录建不出来时，模块把自己关掉，而不是让程序起不来。"""
        with tempfile.TemporaryDirectory() as tmp:
            blocker = pathlib.Path(tmp) / "not_a_directory"
            blocker.write_text("x", encoding="utf-8")
            recorder = EvidenceRecorder(directory=str(blocker / "sub"))
            self.assertFalse(recorder.enabled)
            self.assertGreaterEqual(recorder.write_failures, 1)
            # 关闭状态下调用 observe() 也不许抛
            recorder.observe(FramePacket(line_frame(), 1, 1.0), 1.0)


class EvidenceWritingTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = self._tmp.name
        self._recorders = []

    def tearDown(self):
        # 无论如何都要关掉文件，否则 Windows 上临时目录删不掉。
        for recorder in self._recorders:
            try:
                recorder.close()
            except Exception:
                pass
        try:
            self._tmp.cleanup()
        except Exception:
            pass

    def _recorder(self, **kwargs):
        recorder = EvidenceRecorder(directory=self.tmp, **kwargs)
        self._recorders.append(recorder)
        return recorder

    def _feed(self, recorder, count, start=1.0, step=0.05, blank_every=None):
        now = start
        for index in range(count):
            image = line_frame() if not (blank_every and index % blank_every == 0) else None
            if image is None:
                image = np.full((360, 640, 3), 210, np.uint8)
            recorder.observe(FramePacket(image, index + 1, now), now)
            now += step
        return now

    def test_creates_a_run_directory_with_log(self):
        recorder = self._recorder()
        self.assertTrue(recorder.enabled)
        self.assertTrue(recorder.run_directory.is_dir())
        self.assertTrue(recorder.log_path.is_file())
        recorder.close()

    def test_log_is_written_to_a_temp_directory_not_the_repo(self):
        recorder = self._recorder()
        self._feed(recorder, 20)
        recorder.close()
        log = recorder.log_path
        self.assertTrue(str(log).startswith(self.tmp), "日志必须写在给定目录里")
        self.assertNotIn(str(pathlib.Path(ROOT)), str(log))

    def test_log_rows_have_the_expected_columns(self):
        recorder = self._recorder(flush_interval=0.0)
        self._feed(recorder, 5)
        recorder.close()
        with recorder.log_path.open("r", encoding="utf-8-sig", newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 5)
        self.assertEqual(
            list(rows[0].keys()),
            ["sequence", "captured_at", "loop_time", "elapsed_s",
             "width", "height", "mean_v", "note"],
        )
        self.assertEqual(rows[0]["sequence"], "1")
        self.assertEqual(rows[0]["width"], "640")
        self.assertEqual(rows[0]["height"], "360")

    def test_flush_is_throttled_not_every_frame(self):
        """节流：第一帧建立文件并落一次盘，之后紧跟的帧只入队、不落盘。"""
        recorder = self._recorder(flush_interval=10.0)
        self._feed(recorder, 5, start=1.0, step=0.05)
        self.assertEqual(recorder.rows_written, 1, "第一帧落一次盘即可")
        self.assertEqual(len(recorder.pending), 4, "剩下 4 帧还应该在内存队列里")
        recorder.close()
        self.assertEqual(recorder.rows_written, 5, "close() 必须把队列写完")

    def test_queue_limit_forces_a_flush(self):
        recorder = self._recorder(flush_interval=999.0, queue_limit=4)
        self._feed(recorder, 40, start=1.0, step=0.001)
        self.assertGreater(recorder.rows_written, 0, "队列满时必须落盘，不能无限涨")
        recorder.close()

    def test_duplicate_sequences_are_recorded_once(self):
        """同一个帧号重复喂进来，只记一次。"""
        recorder = self._recorder(flush_interval=0.0)
        frame = FramePacket(line_frame(), 7, 1.0)
        for _ in range(5):
            recorder.observe(frame, 1.0)
        recorder.close()
        with recorder.log_path.open("r", encoding="utf-8-sig", newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 1)
        self.assertEqual(recorder.duplicate_frames_skipped, 4)

    def test_snapshots_are_throttled(self):
        recorder = self._recorder(snapshot_interval=1.0)
        # 10 帧 × 0.05 秒 = 0.5 秒，只该存第一张
        self._feed(recorder, 10, start=1.0, step=0.05)
        self.assertEqual(recorder.snapshots, 1, "0.5 秒内不该存第二张图")
        # 再走 1.1 秒，应该存第二张
        self._feed(recorder, 22, start=1.6, step=0.05)
        self.assertGreaterEqual(recorder.snapshots, 2)
        recorder.close()

    def test_snapshot_files_land_in_the_run_directory(self):
        recorder = self._recorder(snapshot_interval=0.0)
        self._feed(recorder, 3)
        recorder.close()
        images = sorted(recorder.run_directory.glob("frame_*.jpg"))
        self.assertGreaterEqual(len(images), 1)
        self.assertTrue(images[0].stat().st_size > 0)

    def test_snapshot_survives_a_non_ascii_directory(self):
        """目录名里有中文时截图也必须成功。

        OpenCV 自带的写文件接口在 Windows 上走窄字符路径，遇到中文目录会静默
        失败（只返回 False，不抛异常）。本项目的工作目录就带中文，所以必须用
        "先编码到内存、再用 Python 写盘"的方式，并且专门测一条防回归。
        """
        non_ascii = pathlib.Path(self.tmp) / "机器人期末" / "captures"
        recorder = EvidenceRecorder(directory=str(non_ascii), snapshot_interval=0.0)
        self._recorders.append(recorder)
        self._feed(recorder, 3)
        recorder.close()
        self.assertEqual(recorder.write_failures, 0, "中文目录下写截图不该失败")
        images = sorted(recorder.run_directory.glob("frame_*.jpg"))
        self.assertGreaterEqual(len(images), 1)
        self.assertGreater(images[0].stat().st_size, 0)

    def test_close_writes_a_summary_and_is_idempotent(self):
        recorder = self._recorder(flush_interval=0.0)
        self._feed(recorder, 8)
        recorder.close()
        summary_path = recorder.run_directory / "summary.json"
        self.assertTrue(summary_path.is_file())
        payload = json.loads(summary_path.read_text(encoding="utf-8"))
        self.assertEqual(payload["rows_written"], 8)
        self.assertGreaterEqual(payload["snapshots"], 1)
        self.assertEqual(payload["write_failures"], 0)
        recorder.close()  # 再关一次不许抛
        self.assertFalse((recorder.run_directory / "summary.json.tmp").exists())

    def test_write_failure_is_counted_and_never_raised(self):
        """写盘炸了只计数，绝不向外抛——不能影响安全停车。"""
        recorder = self._recorder(flush_interval=0.0)
        recorder._writer = _BrokenWriter()
        for index in range(3):
            recorder.observe(
                FramePacket(line_frame(), index + 1, 1.0 + index * 0.05),
                1.0 + index * 0.05,
            )
        self.assertGreaterEqual(recorder.write_failures, 1)
        recorder.close()

    def test_observe_is_fast(self):
        """observe() 不能阻塞控制循环。"""
        import time

        recorder = self._recorder(flush_interval=999.0, snapshot_interval=999.0)
        frame = FramePacket(line_frame(), 1, 1.0)
        recorder.observe(frame, 1.0)
        started = time.monotonic()
        for index in range(200):
            recorder.observe(line_frame_frame(index), 1.0 + index * 0.01)
        elapsed = time.monotonic() - started
        recorder.close()
        self.assertLess(elapsed, 1.0, "200 帧的入队不该超过 1 秒（实际 %.3fs）" % elapsed)


def line_frame_frame(index):
    return FramePacket(line_frame(320 + (index % 5)), index + 2, 1.0 + index * 0.01)


class EvidenceHarnessTests(unittest.TestCase):
    def test_records_while_another_module_drives(self):
        """别的任务接管时，记录模块仍然每帧都在记。"""
        with tempfile.TemporaryDirectory() as tmp:
            recorder = EvidenceRecorder(directory=tmp, flush_interval=0.0)
            try:
                harness = TaskHarness(observer=recorder)
                harness.start_line(now=1.0)  # 这一帧也会被记录
                for step in range(6):
                    harness.feed_line(1.10 + step * 0.05, x=320 + step * 4)
            finally:
                recorder.close()
            self.assertEqual(harness.owner, "line")
            # start_line 内部喂了 1 帧，加上循环里的 6 帧，一共 7 帧
            self.assertEqual(recorder.rows_written, 7)


class GitignoreTests(unittest.TestCase):
    def test_captures_directory_is_gitignored(self):
        """运行产物不能被提交进仓库。"""
        text = (pathlib.Path(ROOT) / ".gitignore").read_text(encoding="utf-8")
        entries = [line.strip() for line in text.splitlines()]
        self.assertIn(
            evidence.DEFAULT_CAPTURE_DIRECTORY + "/",
            entries,
            "captures/ 必须在 .gitignore 里",
        )


if __name__ == "__main__":
    unittest.main()
