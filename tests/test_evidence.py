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

from models import FramePacket, VisualDetection  # noqa: E402
from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
    line_frame,
)

import evidence  # noqa: E402
from evidence import EvidenceRecorder, make_evidence_photo  # noqa: E402


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

    def test_keyframes_are_capped(self):
        """关键帧只是调试记录，必须封顶；不然交作业时图多到发不完。"""
        recorder = EvidenceRecorder(
            directory=self.tmp, snapshot_interval=0.0, max_keyframes=3
        )
        self._recorders.append(recorder)
        self._feed(recorder, 20)
        recorder.close()
        images = sorted(recorder.run_directory.glob("frame_*.jpg"))
        self.assertEqual(len(images), 3, "关键帧必须停在 max_keyframes 张：%s" % images)
        self.assertEqual(recorder.snapshots, 3)

    def test_the_cap_does_not_touch_scoring_screenshots(self):
        """得分截图不能受关键帧上限影响 —— 老师按它们的张数算分。"""
        recorder = EvidenceRecorder(
            directory=self.tmp, snapshot_interval=0.0, max_keyframes=1
        )
        self._recorders.append(recorder)
        self._feed(recorder, 5)
        # 两个**不同事件**的得分截图（真实请求对象，带各自的 event_key）。
        first = make_evidence_photo(
            "obstacle", FramePacket(line_frame(320), 7, 3.5),
            detection=VisualDetection(
                valid=True, kind="obstacle", box=(10, 10, 90, 90)), side="left")
        second = make_evidence_photo(
            "traffic_light:red", FramePacket(line_frame(320), 9, 4.5), shape="circle",
            detection=VisualDetection(
                valid=True, kind="traffic_light", color="red",
                box=(20, 20, 99, 99)))
        self.assertTrue(recorder.save_task_evidence(first))
        self.assertTrue(recorder.save_task_evidence(second))
        recorder.close()
        self.assertEqual(recorder.snapshots, 1, "关键帧封顶在 1 张")
        self.assertEqual(recorder.task_snapshots, 2,
                         "得分截图不受关键帧上限影响")
        self.assertEqual(len(list(recorder.scoring_directory.glob("task_*.jpg"))), 2)

    def test_console_log_captures_terminal_lines(self):
        """终端状态行要留一份在磁盘上，关掉窗口/崩了之后还能查。"""
        recorder = EvidenceRecorder(directory=self.tmp)
        self._recorders.append(recorder)
        self.assertIsNotNone(recorder.console_log_path, "应该建好 console.log")
        recorder.write_console_log("[   1.2s] 巡线 TRACKING\n")
        recorder.write_console_log("[   2.4s] >> green_junction took over\n")
        recorder.close_console_log()          # 模拟 main 收尾时的显式关闭
        text = recorder.console_log_path.read_text(encoding="utf-8")
        self.assertIn("巡线 TRACKING", text)
        self.assertIn("green_junction took over", text)

    def test_console_log_write_failure_never_raises(self):
        """写日志失败绝不能把车搞崩：关掉句柄后再写只计数。"""
        recorder = EvidenceRecorder(directory=self.tmp)
        self._recorders.append(recorder)
        recorder.close_console_log()
        recorder.write_console_log("这行应该被安静地丢掉\n")   # 不该抛异常
        self.assertIsNone(recorder._console_log)

    def test_summary_names_the_new_artifacts(self):
        recorder = EvidenceRecorder(directory=self.tmp)
        self._recorders.append(recorder)
        recorder.close()
        summary = json.loads(
            (recorder.run_directory / "summary.json").read_text(encoding="utf-8")
        )
        self.assertEqual(summary["console_log"], "console.log")
        self.assertEqual(summary["scoring_directory"], "scoring")
        self.assertEqual(summary["max_keyframes"], 20)

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


class _FakeRequest:
    """一个任务模块交出来的得分截图请求（字段与 EvidenceRequest 同形）。"""

    def __init__(
        self,
        image,
        annotation="Team 10 detects a marker with ID of 2",
        request_id="marker:2:frame:7:attempt:1",
        marker_id="2",
        sequence=7,
        captured_at=1.5,
        box=(240, 130, 400, 230),
    ):
        self.request_id = request_id
        self.marker_id = marker_id
        self.frame_sequence = sequence
        self.captured_at = captured_at
        self.annotation = annotation
        self.text_anchor = (320, 180)
        self.image = image
        self.detection = VisualDetection(valid=True, kind="number_marker", box=box)


class _FakeEvidenceTask:
    """最小假任务：只实现"交请求 / 收回执"这一对协议。"""

    name = "fake_evidence_task"

    def __init__(self, request):
        self._request = request
        self.acks = []

    def take_evidence_request(self):
        request, self._request = self._request, None
        return request

    def acknowledge_evidence(self, request_id, saved):
        self.acks.append((request_id, saved))
        return True


class _FakeCoordinator:
    def __init__(self, task, recorder=None):
        self.motion_tasks = () if task is None else (task,)
        self.observers = () if recorder is None else (recorder,)


class TaskEvidenceTests(unittest.TestCase):
    """Final 的得分截图链：任务交请求 -> 证据层画框写字存盘 -> 回传真实结果。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.directory = self._tmp.name

    def _request(self, **kwargs):
        return _FakeRequest(line_frame(320), **kwargs)

    def test_render_draws_on_a_copy_and_keeps_the_full_scene(self):
        request = self._request()
        before = request.image.copy()
        shown = evidence.render_task_evidence(request)
        self.assertEqual(shown.shape, before.shape)
        self.assertTrue(np.array_equal(request.image, before), "原图不许被改")
        self.assertFalse(np.array_equal(shown, before), "框和字应该画上去了")

    def test_request_without_an_image_is_reported_as_not_saved(self):
        recorder = EvidenceRecorder(directory=self.directory)
        self.addCleanup(recorder.close)
        request = self._request()
        request.image = None
        self.assertFalse(recorder.save_task_evidence(request))
        self.assertEqual(recorder.task_snapshots, 0)
        self.assertEqual(recorder.write_failures, 1)

    def test_saved_snapshot_lands_on_disk_and_in_the_log(self):
        recorder = EvidenceRecorder(directory=self.directory)
        self.addCleanup(recorder.close)
        self.assertTrue(recorder.save_task_evidence(self._request()))
        self.assertEqual(recorder.task_snapshots, 1)

        # 得分截图现在住在 scoring/ 子目录里（交作业只交那个目录）。
        pictures = [path.name for path in recorder.run_directory.rglob("task_*.jpg")]
        self.assertEqual(len(pictures), 1, f"应该正好存一张，实际 {pictures}")
        self.assertIn("2", pictures[0], "文件名里要能看出是哪个标识")
        self.assertEqual(
            len(list(recorder.scoring_directory.glob("task_*.jpg"))), 1,
            "得分截图必须落在 scoring/ 里",
        )

        recorder.close()
        with recorder.log_path.open(encoding="utf-8-sig") as handle:
            notes = [row["note"] for row in csv.DictReader(handle)]
        self.assertTrue(
            any("Team 10 detects a marker with ID of 2" in note for note in notes),
            f"说明文字要进日志，实际日志 {notes}",
        )
        summary = json.loads(
            (recorder.run_directory / "summary.json").read_text(encoding="utf-8")
        )
        self.assertEqual(summary["task_snapshots"], 1)

    def test_disabled_recorder_reports_failure_instead_of_pretending(self):
        """没开记录时必须如实说"没存成"，否则任务会虚报得分。"""
        recorder = EvidenceRecorder()
        self.assertFalse(recorder.save_task_evidence(self._request()))
        self.assertEqual(recorder.task_snapshots, 0)

    def test_service_passes_the_real_result_back_to_the_task(self):
        import main

        recorder = EvidenceRecorder(directory=self.directory)
        self.addCleanup(recorder.close)
        request = self._request()
        task = _FakeEvidenceTask(request)

        saved = main.service_task_evidence(_FakeCoordinator(task, recorder))

        self.assertEqual(saved, 1)
        self.assertEqual(task.acks, [(request.request_id, True)])
        self.assertEqual(recorder.task_snapshots, 1)

    def test_service_reports_failure_when_recording_is_off(self):
        """没有证据写入器时必须马上回 False，让任务立刻交回控制权——
        绝不能把请求悬着，那样车会停在原地等到超时。"""
        import main

        request = self._request()
        task = _FakeEvidenceTask(request)

        saved = main.service_task_evidence(_FakeCoordinator(task))

        self.assertEqual(saved, 0)
        self.assertEqual(task.acks, [(request.request_id, False)])


class RunReportTests(unittest.TestCase):
    """每次运行的独立记录：结束时写入 report.md，可以直接贴进报告发给别人。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.directory = self._tmp.name

    def _request(self, **kwargs):
        return _FakeRequest(line_frame(320), **kwargs)

    def _decision(self, state="LINE_FOLLOWING", task_name=None, status=None,
                  message="", errors=()):
        from coordinator import CoordinatorDecision
        from models import TaskStatus, TaskUpdate

        update = None if status is None else TaskUpdate(TaskStatus[status])
        return CoordinatorDecision(
            state=state,
            owner="line",
            task_name=task_name,
            task_update=update,
            message=message,
            errors=tuple(errors),
        )

    def test_report_lists_the_scoring_snapshot_and_the_task_timeline(self):
        recorder = EvidenceRecorder(directory=self.directory)
        self.addCleanup(recorder.close)
        recorder.observe(FramePacket(line_frame(320), 1, 1.0), 1.0)
        recorder.record_decision(self._decision(), 1.0)
        recorder.record_decision(
            self._decision("TASK_ACTIVE", "number_marker", "RUNNING", "task took over"),
            1.1,
        )
        recorder.record_decision(
            self._decision(
                "RELEASING", "number_marker", "COMPLETED", "task completed",
                errors=("number_marker step was slow: 0.030s",),
            ),
            1.2,
        )
        self.assertTrue(recorder.save_task_evidence(self._request()))
        recorder.close()

        text = (recorder.run_directory / "report.md").read_text(encoding="utf-8")
        for expected in (
            "得分截图",
            "task_2_000007",
            "Team 10 detects a marker with ID of 2",
            "number_marker",
            "COMPLETED",
            "step was slow",
        ):
            self.assertIn(expected, text, f"运行记录里缺少 {expected!r}")

    def test_scoring_screenshots_live_in_their_own_folder(self):
        """得分截图必须在 scoring/ 子目录里 —— 交作业只交那个目录。

        这条同时钉住三件事：文件真在 scoring/ 下、报告里写的是带子目录的路径、
        关键帧不会被误放进 scoring/（放进去就等于把调试图当成分数图交上去）。
        """
        recorder = EvidenceRecorder(directory=self.directory, snapshot_interval=0.0)
        self.addCleanup(recorder.close)
        recorder.observe(FramePacket(line_frame(320), 1, 1.0), 1.0)
        self.assertTrue(recorder.save_task_evidence(self._request()))
        recorder.close()

        run = recorder.run_directory
        scoring = sorted(p.name for p in (run / "scoring").glob("task_*.jpg"))
        self.assertEqual(len(scoring), 1, f"scoring/ 里应该有且只有一张：{scoring}")
        self.assertEqual(
            [], sorted(p.name for p in (run / "scoring").glob("frame_*.jpg")),
            "关键帧不许进 scoring/",
        )
        report = (run / "report.md").read_text(encoding="utf-8")
        self.assertIn("scoring/task_2_000007", report,
                      "报告里要写带子目录的真实相对路径")

    def test_documented_capture_layout_matches_reality(self):
        """模块头写的目录结构必须和真实落盘一致（文档不能骗下一个人）。"""
        recorder = EvidenceRecorder(directory=self.directory)
        self.addCleanup(recorder.close)
        doc = evidence.__doc__ or ""
        for expected in ("log.csv", "console.log", "scoring/", "report.md"):
            self.assertIn(expected, doc, f"模块头没写 {expected}")
        self.assertTrue((recorder.run_directory / "log.csv").exists())
        self.assertTrue(recorder.console_log_path.exists())
        self.assertTrue(recorder.scoring_directory.is_dir())

    def test_run_record_keeps_the_module_own_failure_reason(self):
        """协调器只说"task failed"，模块自己给的原因也必须进记录。

        实车教训：obstacle 有一次 FAILED，记录里只剩 "task failed"，
        为什么失败查不出来了。
        """
        from coordinator import CoordinatorDecision
        from models import TaskStatus, TaskUpdate

        recorder = EvidenceRecorder(directory=self.directory)
        self.addCleanup(recorder.close)
        recorder.observe(FramePacket(line_frame(320), 1, 1.0), 1.0)
        recorder.record_decision(
            CoordinatorDecision(
                state="RELEASING",
                owner="line",
                task_name="obstacle",
                task_update=TaskUpdate(
                    TaskStatus.FAILED, message="no safe way around; giving up"),
                message="task failed",
                errors=(),
            ),
            1.1,
        )
        recorder.close()

        text = (recorder.run_directory / "report.md").read_text(encoding="utf-8")
        self.assertIn("task failed", text)
        self.assertIn("no safe way around; giving up", text,
                      "模块自身的失败原因必须出现在运行记录里")

    def test_record_decision_keeps_transitions_only(self):
        """逐帧调用不能刷屏：只有状态变化或出现错误才记一行。"""
        recorder = EvidenceRecorder(directory=self.directory)
        self.addCleanup(recorder.close)
        recorder.observe(FramePacket(line_frame(320), 1, 1.0), 1.0)
        plain = self._decision()
        recorder.record_decision(plain, 1.00)
        recorder.record_decision(plain, 1.01)
        recorder.record_decision(self._decision(message="still following"), 1.02)
        self.assertEqual(len(recorder.events), 1, "同样的状态不该重复记")

        recorder.record_decision(self._decision(errors=("clamped",)), 1.03)
        self.assertEqual(len(recorder.events), 2, "出现错误必须记下来")

    def test_report_says_so_when_nothing_happened(self):
        recorder = EvidenceRecorder(directory=self.directory)
        self.addCleanup(recorder.close)
        recorder.observe(FramePacket(line_frame(320), 1, 1.0), 1.0)
        recorder.close()
        text = (recorder.run_directory / "report.md").read_text(encoding="utf-8")
        self.assertIn("没有产生得分截图", text)
        self.assertIn("没有任何模块接管", text)

    def test_report_can_carry_runtime_diagnostics(self):
        """运行结束的记录里要能带上"接线层自检结果"。

        为什么要有这个测试：`run_20260915_161540` 里数字标识一次都没接管，
        而记录里查不到"SDK 的 marker 订阅到底成没成功、回调多少 Hz、坐标是
        像素还是归一化"，于是只能靠猜。把自检结果写进 report.md，下一次跑完
        就有答案。
        """
        recorder = EvidenceRecorder(directory=self.directory)
        self.addCleanup(recorder.close)
        recorder.observe(FramePacket(line_frame(320), 1, 1.0), 1.0)
        recorder.record_diagnostics(
            "数字标识观测（SDK marker 订阅）",
            {"subscribed": True, "callback_hz": 9.5},
        )
        recorder.close()

        text = (recorder.run_directory / "report.md").read_text(encoding="utf-8")
        self.assertIn("## 数字标识观测（SDK marker 订阅）", text)
        self.assertIn("| callback_hz | 9.5 |", text)


if __name__ == "__main__":
    unittest.main()
