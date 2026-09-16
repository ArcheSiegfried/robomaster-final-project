"""终端实时显示"现在在跑哪个模块"。

为什么要有这个：调优先级的时候，最想知道的是"此刻谁在开车、它在干什么、谁没被轮到"。
原来的心跳每 2 秒一条、且不带"已接管多久"，看不出模块在跑什么。

这里锁死三件事：
1. **有模块接管时心跳更快**（默认 0.5s），纯巡线时保持低频（默认 2s），不刷屏；
2. 接管中的心跳行要写清：模块名 + 已接管多久 + 实际命令 + 模块自己的说明；
3. 接管那一刻要写出**这一帧排在它后面、根本没被问到的模块**——那就是优先级被截断的位置，
   而且这个信息不需要额外调用任何模块（协调器本来就是按顺序问、遇到第一个 RUNNING 就停）。

只用假 decision 和内存流：不连车、不连相机。
"""

import io
import pathlib
import sys
import types
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from coordinator import LINE_FOLLOWING, RELEASING, TASK_ACTIVE  # noqa: E402
from models import MotionCommand, TaskStatus, TaskUpdate  # noqa: E402
import main as main_module  # noqa: E402


class FakeDecision:
    """只带 ConsoleStatus 真正读到的那几个字段。"""

    def __init__(self, state=LINE_FOLLOWING, task_name=None, message="",
                 command=None, line_state="TRACKING", errors=(),
                 task_status=None):
        self.state = state
        self.task_name = task_name
        self.message = message
        self.command = command if command is not None else MotionCommand()
        self.errors = tuple(errors)
        self.line = types.SimpleNamespace(state=line_state)
        self.task_update = (
            None if task_status is None else TaskUpdate(task_status, message=message)
        )


ORDER = ("traffic_light", "number_marker", "green_junction",
         "free_junction", "route", "obstacle")


def build(task_order=ORDER, heartbeat=2.0, task_heartbeat=0.5):
    stream = io.StringIO()
    status = main_module.ConsoleStatus(
        stream=stream, heartbeat_interval=heartbeat,
        task_heartbeat_interval=task_heartbeat, task_order=task_order,
    )
    return status, stream


class TaskHeartbeatTests(unittest.TestCase):
    def test_task_heartbeat_comes_faster_than_the_idle_one(self):
        status, stream = build()
        status.update(FakeDecision(), 0.0)
        status.update(FakeDecision(), 0.6)          # 纯巡线：0.6s 不该有心跳
        self.assertNotIn("-- ", stream.getvalue())

        status.update(FakeDecision(TASK_ACTIVE, "obstacle", "stepping aside",
                                   MotionCommand(0.2, -0.2, 0.0)), 1.0)
        status.update(FakeDecision(TASK_ACTIVE, "obstacle", "stepping aside",
                                   MotionCommand(0.2, -0.2, 0.0)), 1.6)
        self.assertIn("-- ", stream.getvalue(), "接管中心跳要变成 0.5s 一档")

    def test_task_heartbeat_names_the_module_and_how_long_it_has_run(self):
        status, stream = build()
        status.update(FakeDecision(TASK_ACTIVE, "obstacle", "stepping aside",
                                   MotionCommand(0.2, -0.2, 0.0)), 10.0)
        status.update(FakeDecision(TASK_ACTIVE, "obstacle", "stepping aside",
                                   MotionCommand(0.2, -0.2, 0.0)), 11.5)
        text = stream.getvalue()
        self.assertIn("obstacle", text)
        self.assertIn("已 1.5s", text, "要能看出这个模块已经跑了多久")
        self.assertIn("stepping aside", text, "要带上模块自己给的说明")
        self.assertIn("x=0.20", text)

    def test_idle_heartbeat_is_still_low_frequency(self):
        status, stream = build()
        for step in range(1, 13):
            status.update(FakeDecision(), step * 0.5)   # 6 秒纯巡线
        self.assertLessEqual(stream.getvalue().count("-- "), 3,
                             "纯巡线不该被刷屏")

    def test_heartbeat_does_not_flood_during_a_long_takeover(self):
        status, stream = build(task_heartbeat=0.5)
        for index in range(60):                          # 20 秒接管
            status.update(FakeDecision(TASK_ACTIVE, "route", "searching",
                                       MotionCommand(0.1, 0.0, 20.0)),
                          100.0 + index / 3.0)
        heartbeat_lines = stream.getvalue().count("-- ")
        self.assertGreater(heartbeat_lines, 20, "20 秒接管要有足够多的可见行")
        self.assertLess(heartbeat_lines, 70, "但也不能逐帧刷屏")


class PriorityCutTests(unittest.TestCase):
    def test_takeover_line_lists_the_modules_that_were_not_asked(self):
        status, stream = build()
        status.update(FakeDecision(TASK_ACTIVE, "traffic_light", "holding red"), 1.0)
        text = stream.getvalue()
        self.assertIn("模块开始运行：traffic_light", text)
        for name in ("number_marker", "green_junction", "free_junction",
                     "route", "obstacle"):
            self.assertIn(name, text, "被截断的模块要列出来：%s" % name)

    def test_last_module_has_nothing_after_it(self):
        status, stream = build()
        status.update(FakeDecision(TASK_ACTIVE, "obstacle", "stepping aside"), 1.0)
        text = stream.getvalue()
        self.assertIn("模块开始运行：obstacle", text)
        self.assertNotIn("number_marker", text, "障碍物是最后一个，后面没人")

    def test_missing_task_order_is_harmless(self):
        status, stream = build(task_order=())
        status.update(FakeDecision(TASK_ACTIVE, "obstacle", "stepping aside"), 1.0)
        self.assertIn("模块开始运行：obstacle", stream.getvalue())

    def test_release_still_reports_the_end(self):
        status, stream = build()
        status.update(FakeDecision(TASK_ACTIVE, "obstacle", "stepping aside"), 1.0)
        status.update(FakeDecision(RELEASING, "obstacle", "task completed",
                                   task_status=TaskStatus.COMPLETED), 2.0)
        text = stream.getvalue()
        self.assertIn("模块结束运行：obstacle", text)
        self.assertIn("COMPLETED", text)


if __name__ == "__main__":
    unittest.main()
