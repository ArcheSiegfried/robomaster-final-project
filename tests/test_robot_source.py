"""集成侧的 SDK"机器人识别"观测通道（`robot_source.py` + `main.py` 的喂数据函数）。

存在的原因：障碍模块 v10 起把 "SDK 机器人识别 + 蓝线消失" 当主路径，而任务模块按契约
不得自己订阅 SDK，所以订阅与喂数据都在集成层。这一组测试同时钉住三件事：
  1. `RobotObservationSource` 的订阅/回调/坐标换算/诊断是安全的；
  2. `main.feed_robot_observations()` 真的把 `(rows, observed_at)` 喂给了任务，
     而且**不会**把观测时刻刷新成 now（否则过期的框会被当新鲜）；
  3. **端到端**：SDK 观测 + 蓝线消失 → 障碍模块真的接管；没有观测时同一帧不接管
     （证明主路径是这条通道给出来的，不是灰度兜底）。
"""

import pathlib
import sys
import time
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import main  # noqa: E402
import obstacle  # noqa: E402
import robot_source  # noqa: E402
from models import FramePacket, TaskStatus  # noqa: E402
from obstacle import ObstacleTask  # noqa: E402
from robot_source import RobotObservationSource  # noqa: E402

WIDTH, HEIGHT = 640, 360


class FakeVision:
    """假的 SDK vision：记住订阅名与回调，供测试手动触发回调。"""

    def __init__(self, subscribe_result=True, explode=False):
        self.subscribe_result = subscribe_result
        self.explode = explode
        self.subscribed = []
        self.unsubscribed = []
        self.callback = None

    def sub_detect_info(self, name, callback=None, **kwargs):
        if self.explode:
            raise RuntimeError("sdk boom")
        self.subscribed.append(name)
        self.callback = callback
        return self.subscribe_result

    def unsub_detect_info(self, name):
        self.unsubscribed.append(name)
        return True


def frame(line=True, sequence=1, captured_at=0.0):
    image = np.full((HEIGHT, WIDTH, 3), 210, np.uint8)
    if line:
        cv2.line(image, (320, 350), (320, 160), (255, 0, 0), 24)
    return FramePacket(image=image, sequence=sequence, captured_at=captured_at)


class SourceTests(unittest.TestCase):
    def test_start_subscribes_robot_and_reports_it(self):
        vision = FakeVision()
        source = RobotObservationSource(vision)
        self.assertTrue(source.start())
        self.assertEqual(vision.subscribed, ["robot"])
        self.assertTrue(source.enabled)
        self.assertTrue(source.stats()["subscribed"])

    def test_start_without_vision_is_a_safe_noop(self):
        source = RobotObservationSource(None)
        self.assertFalse(source.start())
        self.assertFalse(source.enabled)
        rows, observed_at = source.observations(frame())
        self.assertEqual(rows, ())
        self.assertIsNone(observed_at)
        source.stop()  # 不许抛异常

    def test_start_survives_a_broken_sdk(self):
        source = RobotObservationSource(FakeVision(explode=True))
        self.assertFalse(source.start())
        self.assertFalse(source.enabled)

    def test_start_reports_a_refused_subscription(self):
        source = RobotObservationSource(FakeVision(subscribe_result=False))
        self.assertFalse(source.start())
        self.assertFalse(source.enabled)

    def test_stop_unsubscribes(self):
        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        source.stop()
        self.assertEqual(vision.unsubscribed, ["robot"])
        self.assertFalse(source.enabled)

    def test_pixel_rows_pass_through_as_centre_plus_size(self):
        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        vision.callback([[320, 200, 180, 120]])
        rows, observed_at = source.observations(frame())
        self.assertEqual(rows, ((320.0, 200.0, 180.0, 120.0),))
        self.assertIsNotNone(observed_at)

    def test_normalized_rows_are_converted_to_pixels(self):
        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        vision.callback([[0.5, 0.55, 0.3, 0.33]])
        rows, _ = source.observations(frame())
        self.assertEqual(len(rows), 1)
        x, y, width, height = rows[0]
        self.assertAlmostEqual(x, 320.0, delta=0.5)
        self.assertAlmostEqual(y, 198.0, delta=0.5)
        self.assertAlmostEqual(width, 192.0, delta=0.5)
        self.assertAlmostEqual(height, 118.8, delta=0.5)
        self.assertEqual(source.stats()["coordinate_mode"], "normalized")

    def test_empty_callback_clears_the_snapshot(self):
        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        vision.callback([[320, 200, 180, 120]])
        self.assertEqual(len(source.observations(frame())[0]), 1)
        vision.callback([])
        rows, observed_at = source.observations(frame())
        self.assertEqual(rows, ())
        self.assertIsNone(observed_at)
        stats = source.stats()
        self.assertEqual(stats["callbacks"], 2)
        self.assertEqual(stats["empty_callbacks"], 1)
        self.assertEqual(stats["robots_in_snapshot"], 0)

    def test_broken_and_sizeless_rows_are_dropped(self):
        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        vision.callback([
            [float("nan"), 200, 180, 120],
            [320, 200, 0, 120],
            [320, 200, 180, float("inf")],
            [320, 200, 180, 120],
        ])
        rows, _ = source.observations(frame())
        self.assertEqual(rows, ((320.0, 200.0, 180.0, 120.0),))

    def test_observations_need_a_real_image(self):
        source = RobotObservationSource(FakeVision())
        source.start()
        source._on_detect_info([[320, 200, 180, 120]])
        self.assertEqual(source.observations(None), ((), None))
        self.assertEqual(
            source.observations(SimpleNamespace(image=None)), ((), None)
        )

    def test_stats_and_warnings_explain_why_the_module_stays_idle(self):
        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        self.assertIn("还没有收到", source.rate_warning())
        # 一次很窄的框 -> 提醒"太远"（宽度分支优先于频率分支）
        vision.callback([[320, 200, 30, 20]])
        source.observations(frame())  # 宽度统计发生在 observations() 里
        stats = source.stats()
        self.assertEqual(stats["callbacks"], 1)
        self.assertIsNone(stats["callback_hz"], "只收到 1 次回调时不该报 0 Hz")
        self.assertAlmostEqual(stats["max_width_ratio"], round(30 / 640.0, 3))
        self.assertIn("低于模块要求的", source.rate_warning())
        # 再补一次回调，频率就测得出来了；框仍然太窄 -> 还是宽度提醒
        vision.callback([[320, 200, 30, 20]])
        source.observations(frame())
        with source._lock:
            source.first_callback_at = 100.0
            source.last_callback_at = 100.9
        self.assertAlmostEqual(source.stats()["callback_hz"], round(1 / 0.9, 2))
        self.assertIn("低于模块要求的", source.rate_warning())

    def test_a_slow_callback_rate_is_called_out(self):
        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        vision.callback([[320, 200, 260, 150]])
        vision.callback([[320, 200, 260, 150]])
        with source._lock:
            source.first_callback_at = 100.0
            source.last_callback_at = 100.9
        self.assertAlmostEqual(source.stats()["callback_hz"], round(1 / 0.9, 2))
        self.assertIn("低于模块要求的", source.rate_warning())

    def test_width_threshold_matches_the_module(self):
        """集成侧的提醒门槛必须与障碍模块的要求一致，否则提示会骗人。"""
        self.assertEqual(
            robot_source.MIN_USEFUL_WIDTH_RATIO, obstacle.ROBOT_MIN_WIDTH_RATIO
        )
        self.assertLessEqual(
            1.0 / robot_source.MIN_USEFUL_HZ, obstacle.ROBOT_OBSERVATION_MAX_AGE
        )

    def test_no_warning_when_everything_is_healthy(self):
        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        vision.callback([[320, 200, 260, 150]])
        vision.callback([[320, 200, 260, 150]])
        with source._lock:
            source.first_callback_at = 100.0
            source.last_callback_at = 100.3
        self.assertAlmostEqual(source.stats()["callback_hz"], round(1 / 0.3, 2))
        self.assertEqual(source.rate_warning(), "")


class FakeTask:
    def __init__(self, accept=True):
        self.calls = []
        self.accept = accept

    def update_robot_observations(self, rows, observed_at=None):
        if not self.accept:
            raise RuntimeError("this task hates observations")
        self.calls.append((rows, observed_at))


class FeedTests(unittest.TestCase):
    def _source_with(self, rows):
        source = RobotObservationSource(FakeVision())
        source.start()
        source._on_detect_info(rows)
        return source

    def test_feed_pushes_rows_and_the_original_observation_time(self):
        source = self._source_with([[320, 200, 180, 120]])
        task = FakeTask()
        coordinator = SimpleNamespace(motion_tasks=(task, SimpleNamespace()))
        fed = main.feed_robot_observations(coordinator, source, frame(), 5.0)
        self.assertEqual(fed, 1, "只有一个任务有这条入口")
        rows, observed_at = task.calls[0]
        self.assertEqual(rows, ((320.0, 200.0, 180.0, 120.0),))
        # 绝不能刷新成 now(5.0)，否则过期观测会被当成新鲜
        self.assertNotEqual(observed_at, 5.0)
        self.assertIsNotNone(observed_at)

    def test_feed_is_a_noop_without_a_source(self):
        task = FakeTask()
        coordinator = SimpleNamespace(motion_tasks=(task,))
        self.assertEqual(main.feed_robot_observations(coordinator, None, frame(), 5.0), 0)
        self.assertEqual(task.calls, [])

    def test_feed_never_raises_when_the_source_or_task_breaks(self):
        broken_source = SimpleNamespace(
            observations=lambda image: (_ for _ in ()).throw(RuntimeError("boom"))
        )
        task = FakeTask()
        coordinator = SimpleNamespace(motion_tasks=(task,))
        # 观测层炸了：仍然要把"这帧没有观测"喂下去（清掉任务的旧框），但绝不抛异常。
        self.assertEqual(
            main.feed_robot_observations(coordinator, broken_source, frame(), 5.0), 1
        )
        self.assertEqual(task.calls[0][0], ())
        self.assertIsNone(task.calls[0][1])

        good_source = self._source_with([[320, 200, 180, 120]])
        angry = FakeTask(accept=False)
        coordinator = SimpleNamespace(motion_tasks=(angry,))
        # 任务自己炸了：喂进去失败，不计入成功数，也不许把主循环带崩。
        self.assertEqual(
            main.feed_robot_observations(coordinator, good_source, frame(), 5.0), 0
        )


class EndToEndTests(unittest.TestCase):
    """SDK 观测 + 蓝线消失 -> 障碍模块真的接管（这就是这轮适配要证明的事）。"""

    BOX = [[320, 200, 260, 150]]

    def _takeover(self, with_observation, line_lost=True, frames=4):
        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        task = ObstacleTask()
        coordinator = SimpleNamespace(motion_tasks=(task,))
        now = time.monotonic()
        last = None
        for index in range(frames):
            if with_observation:
                vision.callback(self.BOX)
            packet = frame(line=not line_lost, sequence=index + 1, captured_at=now)
            main.feed_robot_observations(coordinator, source, packet, now)
            last = task.step(packet, now)
            now += 0.05
        return task, last

    def test_sdk_robot_plus_lost_line_takes_over_and_steps_aside(self):
        task, update = self._takeover(with_observation=True)
        self.assertEqual(update.status, TaskStatus.RUNNING)
        self.assertIsNotNone(update.motion)
        self.assertNotEqual(update.motion.lateral, 0.0, "接管当帧就该横移让开")
        self.assertEqual(task.detector.last_source, "sdk_robot")

    def test_line_lost_without_any_robot_observation_does_not_take_over(self):
        task, update = self._takeover(with_observation=False)
        self.assertEqual(update.status, TaskStatus.NOT_TRIGGERED)
        self.assertNotEqual(
            task.detector.last_source, "sdk_robot", "没有观测却走了 SDK 主路径"
        )

    def test_a_stale_observation_does_not_take_over(self):
        """SDK 回调停了（车开走了）：旧框不能继续当新鲜观测。"""
        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        vision.callback(self.BOX)
        task = ObstacleTask()
        coordinator = SimpleNamespace(motion_tasks=(task,))
        now = time.monotonic()
        update = None
        for index in range(4):
            now += obstacle.ROBOT_OBSERVATION_MAX_AGE + 0.05
            packet = frame(line=False, sequence=index + 1, captured_at=now)
            main.feed_robot_observations(coordinator, source, packet, now)
            update = task.step(packet, now)
        self.assertNotEqual(update.status, TaskStatus.RUNNING)


class DiagnosticsTests(unittest.TestCase):
    class Sink:
        def __init__(self):
            self.sections = []

        def record_diagnostics(self, title, values):
            self.sections.append((title, values))

        def save_task_evidence(self, request):  # 证据写入器的标志方法（被 main 用来找 sink）
            return False

    def test_both_subscriptions_land_in_the_run_record(self):
        marker = SimpleNamespace(
            stats=lambda: {"subscribed": True},
            rate_warning=lambda: "",
        )
        robot = SimpleNamespace(
            stats=lambda: {"subscribed": True, "callbacks": 12},
            rate_warning=lambda: "回调太慢",
        )
        sink = self.Sink()
        coordinator = SimpleNamespace(observers=(sink,))
        main.record_runtime_diagnostics(coordinator, marker, robot)
        titles = [title for title, _ in sink.sections]
        self.assertEqual(len(titles), 2)
        self.assertIn("marker", titles[0])
        self.assertIn("机器人识别", titles[1])
        self.assertEqual(sink.sections[1][1]["提醒"], "回调太慢")

    def test_missing_subscriptions_are_explained_not_silent(self):
        sink = self.Sink()
        coordinator = SimpleNamespace(observers=(sink,))
        main.record_runtime_diagnostics(coordinator, None, None)
        self.assertEqual(len(sink.sections), 2)
        for _title, values in sink.sections:
            self.assertIn("状态", values)


class CoordinatorEndToEndTests(unittest.TestCase):
    """最接近实车的一条：真协调器 + 假底盘 + 我喂的 SDK 观测。"""

    def test_real_coordinator_hands_the_chassis_to_obstacle(self):
        from tests.task_harness import TaskHarness, line_frame

        vision = FakeVision()
        source = RobotObservationSource(vision)
        source.start()
        task = ObstacleTask()
        harness = TaskHarness(task=task)
        now = time.monotonic()
        harness.start_line(now)
        self.assertEqual(harness.state, "LINE_FOLLOWING")

        # SDK 报"正前方有一台车"，紧接着蓝线被它挡住（画面里没有蓝线了）
        vision.callback([[320, 200, 260, 150]])
        blocked = np.full((HEIGHT, WIDTH, 3), 210, np.uint8)
        decision = None
        for index in range(6):
            now += 0.05
            packet = FramePacket(blocked, 200 + index, now)
            main.feed_robot_observations(harness.coordinator, source, packet, now)
            decision = harness.coordinator.step(packet, now)

        self.assertEqual(
            harness.task_name, "obstacle", "协调器没把控制权判给障碍模块"
        )
        self.assertEqual(harness.owner, "external", "唯一运动出口没交给任务")
        # 底盘真的收到了横移指令（不是只有 status）
        moving = harness.chassis.motion_calls
        self.assertTrue(moving, "接管了却没有发出任何会动的指令")
        self.assertTrue(
            any(abs(call.get("y", 0.0)) > 0.0 for call in moving),
            "接管后没有横移让开：%s" % (moving[-1],),
        )
        self.assertEqual(task.detector.last_source, "sdk_robot")

        # 对照组：同样的帧，但没有 SDK 观测 -> 不许接管
        silent = ObstacleTask()
        quiet = TaskHarness(task=silent)
        now2 = time.monotonic()
        quiet.start_line(now2)
        for index in range(6):
            now2 += 0.05
            quiet.coordinator.step(FramePacket(blocked, 300 + index, now2), now2)
        self.assertNotEqual(quiet.task_name, "obstacle")


class ReportSectionTests(unittest.TestCase):
    """运行记录里必须真的出现"障碍物观测"这一小节（现场排查靠它）。"""

    def test_report_carries_the_robot_subscription_section(self):
        import tempfile

        from evidence import EvidenceRecorder
        from tests.task_harness import line_frame

        with tempfile.TemporaryDirectory() as directory:
            recorder = EvidenceRecorder(directory=directory)
            self.addCleanup(recorder.close)
            recorder.observe(FramePacket(line_frame(320), 1, 1.0), 1.0)
            source = RobotObservationSource(FakeVision())
            source.start()
            source._on_detect_info([[320, 200, 260, 150]])
            source.observations(frame())
            coordinator = SimpleNamespace(observers=(recorder,))
            main.record_runtime_diagnostics(coordinator, None, source)
            recorder.close()

            text = (recorder.run_directory / "report.md").read_text(encoding="utf-8")
        self.assertIn("## 障碍物观测（SDK 机器人识别）", text)
        self.assertIn("| callbacks | 1 |", text)
        self.assertIn("| max_width_ratio | 0.406 |", text)


if __name__ == "__main__":
    unittest.main()
