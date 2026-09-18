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
        # 让网络预检通过，这样才会真的走到 SDK 的 initialize（本测试的意图是
        # "提示打在碰硬件之前"，不是测网络）。真实机器上连不上时预检会先给诊断，
        # 那条路径由 ConnectPreflightTests 单独覆盖。
        original_local = main._local_ap_address
        original_probe = main._robot_reachable
        main._local_ap_address = lambda *a, **k: "192.168.2.23"
        main._robot_reachable = lambda *a, **k: True
        buffer = io.StringIO()
        try:
            with contextlib.redirect_stdout(buffer):
                with self.assertRaises(RuntimeError):
                    main.main()
        finally:
            main._local_ap_address = original_local
            main._robot_reachable = original_probe
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


class ConnectPreflightTests(unittest.TestCase):
    """连不上车时必须给**能照着做的**诊断，而不是 SDK 内部的 traceback。

    2026-09-18 现场被误导过一次：真正原因是没连上车的热点，屏幕上却只有
    `Exception ignored in: <function Client.__del__ ...>
     AttributeError: 'NoneType' object has no attribute 'is_alive'`
    —— 那条来自 SDK 的 `Client.__del__ → stop()`（建连接失败时 `_thread` 仍是
    None），它会**盖住真正的原因**。所以现在在调 SDK 之前先探一次。

    ⚠️ 另一条教训：探针**必须走 UDP 20020**。我第一版探 TCP 80/20001，
    结果"已经连上车了也报连不上"—— 车的控制通道是 UDP，TCP 必然超时。
    `test_the_probe_uses_udp_not_tcp` 就是钉住这一点的。
    """

    def _patch(self, local="192.168.2.23", reachable=True):
        import main

        original_local = main._local_ap_address
        original_probe = main._robot_reachable
        main._local_ap_address = lambda *a, **k: local
        main._robot_reachable = lambda *a, **k: reachable
        self.addCleanup(setattr, main, "_local_ap_address", original_local)
        self.addCleanup(setattr, main, "_robot_reachable", original_probe)

    def test_unreachable_robot_does_not_block_but_warns(self):
        """**回归（我自己的错）**：探测探不通**不许拦住连接**。

        我第一版把探测做成了硬闸门（探不通就 SystemExit）。结果探针自己写错三次
        （先探 TCP、再写死绑 .23、又用错 send_self），每次都把"网络本来是好的"
        报成"连不上"，白白让现场去查网络。教训：**诊断不能有能力阻止一次
        本来能成功的连接**。所以现在探不通也要继续往下走，让 SDK 自己判。
        """
        import main

        self._patch(local="192.168.2.24", reachable=False)
        built = object()
        module = types.SimpleNamespace(Robot=lambda: built)

        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            robot = main._connect_robot(module)

        self.assertIs(robot, built, "探测失败也必须继续建立机器人对象")
        text = printed.getvalue()
        self.assertIn("192.168.2.24", text, "要把本机实际地址打出来")
        self.assertIn("没有应答", text, "要如实说明探测没收到应答")

    def test_missing_local_ap_address_warns_but_still_connects(self):
        """本机没有 192.168.2.x 时也只提示、不拦路。"""
        import main

        self._patch(local=None, reachable=False)
        built = object()
        module = types.SimpleNamespace(Robot=lambda: built)

        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            robot = main._connect_robot(module)

        self.assertIs(robot, built)
        text = printed.getvalue()
        self.assertIn("没有 192.168.2.x", text)
        self.assertIn("仍然继续尝试连接", text)

    def test_reachable_robot_builds_the_object(self):
        import main

        self._patch(local="192.168.2.23", reachable=True)
        built = object()
        module = types.SimpleNamespace(Robot=lambda: built)
        self.assertIs(main._connect_robot(module), built)

    def test_the_probe_never_raises(self):
        """探测本身绝不能抛异常（没网、SDK 不在、绑不上端口都只该返回 False）。"""
        import main

        self.assertIn(main._robot_reachable(timeout=0.05), (True, False))
        self.assertIsInstance(main._local_ap_address(), (str, type(None)))

    def test_the_probe_uses_udp_not_tcp(self):
        """**回归**：探针必须走 UDP 20020，不能退回 TCP。

        我第一版用的是 `socket.create_connection`（TCP 80/20001），实车"已经
        连上热点"也报连不上。车的控制通道是 UDP，这条钉住不再犯。

        **只看代码，不看注释/docstring** —— 函数说明里正当地解释了
        "为什么不能用 send_self"，用整段源码做断言会把自己讲过的坑当成违规。
        """
        import ast
        import inspect
        import re

        import main

        source = inspect.getsource(main._robot_reachable)
        # 去掉 docstring 与注释再看代码：函数说明里正当地写了
        # "为什么不能用 send_self"，用整段源码断言会把自己讲过的坑当成违规。
        code = re.sub(r'"""(?:.|\n)*?"""', "", source)
        code = re.sub(r"'''(?:.|\n)*?'''", "", code)
        code = "\n".join(line.split("#", 1)[0] for line in code.splitlines())
        self.assertIn("udp", code.lower(), "探针必须显式用 UDP")
        self.assertNotIn("create_connection", code,
                         "create_connection 是 TCP —— 会误报连不上车")
        self.assertNotIn("send_self", code,
                         "send_self 发给本机自己，机器人收不到")
        self.assertEqual(main.ROBOT_AP_PORT, 20020)

    def test_alignment_fixes_the_hardcoded_local_address(self):
        """**回归**：SDK 把本机地址写死成 192.168.2.23，实际是别的值时连不上。

        现场是 `.24`：`Connection.create()` 的 `bind(('192.168.2.23', 10100))`
        直接抛 OSError → `_conn=None` → 连不上，最后只剩一条
        `AttributeError: 'NoneType' object has no attribute 'is_alive'`。
        这里验证会把它对齐到实际地址。
        """
        import main
        import robomaster.config as sdk_config

        original_setting = sdk_config.LOCAL_IP_STR
        original_local = main._local_ap_address
        main._local_ap_address = lambda *a, **k: "192.168.2.24"
        sdk_config.LOCAL_IP_STR = None
        self.addCleanup(setattr, main, "_local_ap_address", original_local)
        self.addCleanup(setattr, sdk_config, "LOCAL_IP_STR", original_setting)

        self.assertEqual(main._align_local_address(), "192.168.2.24")
        self.assertEqual(sdk_config.LOCAL_IP_STR, "192.168.2.24")

    def test_alignment_respects_an_explicit_setting(self):
        """用户显式设过 LOCAL_IP_STR 就不许被覆盖。"""
        import main
        import robomaster.config as sdk_config

        original_setting = sdk_config.LOCAL_IP_STR
        original_local = main._local_ap_address
        main._local_ap_address = lambda *a, **k: "192.168.2.99"
        sdk_config.LOCAL_IP_STR = "192.168.2.50"
        self.addCleanup(setattr, main, "_local_ap_address", original_local)
        self.addCleanup(setattr, sdk_config, "LOCAL_IP_STR", original_setting)

        self.assertEqual(main._align_local_address(), "192.168.2.50")
        self.assertEqual(sdk_config.LOCAL_IP_STR, "192.168.2.50")


class ConsoleTeeTests(unittest.TestCase):
    """终端状态行同时落盘：关掉窗口/程序崩了之后还能查当时发生了什么。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)

    def test_console_status_lines_also_land_in_console_log(self):
        import main
        from evidence import EvidenceRecorder

        recorder = EvidenceRecorder(directory=self._tmp.name)
        self.addCleanup(recorder.close)
        terminal = io.StringIO()
        console = main.ConsoleStatus(
            stream=main._TeeStream(terminal, recorder.write_console_log),
            heartbeat_interval=2.0,
        )
        console.note("启动提示：测试用")
        console.note("第二条状态行")
        recorder.close_console_log()

        on_screen = terminal.getvalue()
        on_disk = recorder.console_log_path.read_text(encoding="utf-8")
        self.assertIn("启动提示：测试用", on_screen, "终端照旧要看得到")
        self.assertIn("启动提示：测试用", on_disk, "磁盘上也要有一份")
        self.assertIn("第二条状态行", on_disk)

    def test_a_broken_sink_never_breaks_the_terminal(self):
        """日志写坏了也不能影响打印，更不能影响控制循环。"""
        import main

        def broken(text):
            raise RuntimeError("synthetic: disk on fire")

        terminal = io.StringIO()
        stream = main._TeeStream(terminal, broken)
        stream.write("还是要打印出来\n")
        stream.flush()
        self.assertIn("还是要打印出来", terminal.getvalue())

    def test_tee_is_skipped_when_the_sink_cannot_log(self):
        """观察者没有 write_console_log 时不该包 tee（否则启动就报错）。"""
        import main

        sink = types.SimpleNamespace(run_directory=None)
        self.assertFalse(callable(getattr(sink, "write_console_log", None)))
        # 真实判断条件：run_directory 存在 **且** 能写日志，才包 tee。
        self.assertIsNone(getattr(sink, "run_directory", None))


if __name__ == "__main__":
    unittest.main()
