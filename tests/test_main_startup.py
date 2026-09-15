"""main.py 的启动提示：连不上车时绝不能一言不发。

为什么要有这个测试：RoboMaster SDK 的 `initialize()` 没有超时，机器人在
192.168.2.1 但本机不在那个网段时会**静默阻塞**；而 main.py 的第一条正常输出
原本排在"连接 + 云台回中 + 开视频流"之后，于是现场看到的就是一片空白，
分不清是卡住了还是根本没跑起来。这条测试锁死"碰硬件之前先出声"。

离线实现方式：注入一个假的 robomaster 模块，让连接立刻失败，
断言异常抛出**之前**提示已经打印出来。不连车、不连相机。
"""

import contextlib
import io
import pathlib
import sys
import tempfile
import types
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


class _FakeRobot:
    """连不上车的假机器人：initialize 立刻抛错。"""

    def __init__(self):
        self.closed = False
        self.chassis = object()
        self.gimbal = object()
        self.vision = object()
        self.camera = types.SimpleNamespace()

    def initialize(self, **kwargs):
        raise RuntimeError("synthetic: no robot on 192.168.2.1")

    def set_robot_mode(self, mode=None):
        return True

    def close(self):
        self.closed = True


def _install_fake_sdk(robot_instance):
    module = types.ModuleType("robomaster")
    module.robot = types.SimpleNamespace(
        Robot=lambda: robot_instance, FREE=0, CHASSIS_LEAD=1
    )
    module.camera = types.SimpleNamespace()
    sys.modules["robomaster"] = module
    return module


class MainStartupTests(unittest.TestCase):
    def test_banner_is_printed_before_touching_hardware(self):
        import main

        robot_instance = _FakeRobot()
        previous = sys.modules.get("robomaster")
        _install_fake_sdk(robot_instance)
        buffer = io.StringIO()
        try:
            with contextlib.redirect_stdout(buffer):
                with self.assertRaises(RuntimeError):
                    main.main()
        finally:
            if previous is not None:
                sys.modules["robomaster"] = previous
            else:
                sys.modules.pop("robomaster", None)

        output = buffer.getvalue()
        self.assertIn("正在连接机器人", output, "连接之前必须先打出提示")
        self.assertIn("192.168.2.1", output, "提示里要写清机器人的固定地址")
        self.assertIn("RMEP", output, "提示里要写清该连哪个热点")
        self.assertTrue(robot_instance.closed, "失败路径也要关掉连接")

    def test_banner_text_appears_before_the_initialize_call_in_source(self):
        """静态兜底：提示必须排在 initialize 之前，不能被挪到后面去。"""
        source = (ROOT / "main.py").read_text(encoding="utf-8")
        banner = source.find("正在连接机器人")
        initialize = source.find('ep_robot.initialize(conn_type="ap"')
        self.assertNotEqual(banner, -1, "启动提示不见了")
        self.assertNotEqual(initialize, -1, "找不到 initialize 调用")
        self.assertLess(banner, initialize, "启动提示必须排在连接硬件之前")


class RuntimeDiagnosticsTests(unittest.TestCase):
    """运行结束时把接线层自检结果写进运行记录。

    为什么要有这个测试：`run_20260915_161540` 里数字标识一次都没接管，而
    `report.md` 里查不到"SDK 的 marker 订阅到底成没成功、回调多少 Hz"，
    于是只能猜是模块的问题还是现场的问题。这条测试锁死"跑完就有答案"。
    """

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)

    def _recorder(self):
        from evidence import EvidenceRecorder

        recorder = EvidenceRecorder(directory=self._tmp.name)
        self.addCleanup(recorder.close)
        return recorder

    def _report(self, recorder):
        return (recorder.run_directory / "report.md").read_text(encoding="utf-8")

    def test_marker_subscription_status_lands_in_the_run_record(self):
        import main

        recorder = self._recorder()
        coordinator = types.SimpleNamespace(observers=[recorder])

        class _Source:
            def stats(self):
                return {
                    "subscribed": True,
                    "callback_hz": 9.5,
                    "coordinate_mode": "pixels",
                    "markers_in_snapshot": 1,
                }

            def rate_warning(self):
                return ""

        main.record_runtime_diagnostics(coordinator, _Source())
        recorder.close()

        text = self._report(recorder)
        self.assertIn("数字标识", text)
        self.assertIn("| callback_hz | 9.5 |", text)
        self.assertIn("| coordinate_mode | pixels |", text)

    def test_silent_subscription_is_reported_with_its_reason(self):
        """一个 marker 回调都没收到时，记录里要写清这一点。"""
        import main

        recorder = self._recorder()
        coordinator = types.SimpleNamespace(observers=[recorder])

        class _Source:
            def stats(self):
                return {"subscribed": False, "callbacks": 0, "callback_hz": 0.0}

            def rate_warning(self):
                return "还没有收到任何 marker 回调。"

        main.record_runtime_diagnostics(coordinator, _Source())
        recorder.close()

        text = self._report(recorder)
        self.assertIn("还没有收到任何 marker 回调。", text)


    def test_missing_marker_subscription_is_recorded_too(self):
        """连订阅都没建起来时（marker_source 为空），记录里不能是一片空白。"""
        import main

        recorder = self._recorder()
        coordinator = types.SimpleNamespace(observers=[recorder])

        main.record_runtime_diagnostics(coordinator, None)
        recorder.close()

        text = self._report(recorder)
        self.assertIn("数字标识", text)
        self.assertIn("没有建立", text)


if __name__ == "__main__":
    unittest.main()
