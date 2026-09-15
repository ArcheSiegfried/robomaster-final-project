"""终端状态反馈（ConsoleStatus）的离线测试。

验的是操作员在 VS Code 终端里能不能看到关键信息：丢线、任务接管/结束、
限幅与异常、得分截图、以及低频心跳；而且绝不逐帧刷屏、绝不影响控制循环。
"""

import io
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from coordinator import (  # noqa: E402
    LINE_FOLLOWING,
    RELEASING,
    TASK_ACTIVE,
    CoordinatorDecision,
)
from models import TaskStatus, TaskUpdate  # noqa: E402

from main import ConsoleStatus  # noqa: E402


class _Line:
    """只带 state 的巡线结果替身（真实 RuntimeDecision 的同名字段）。"""

    def __init__(self, state):
        self.state = state


def decision(
    line_state="TRACKING",
    state=LINE_FOLLOWING,
    task_name=None,
    status=None,
    message="",
    errors=(),
):
    update = None if status is None else TaskUpdate(TaskStatus[status])
    return CoordinatorDecision(
        state=state,
        owner="line",
        line=_Line(line_state),
        task_name=task_name,
        task_update=update,
        message=message,
        errors=tuple(errors),
    )


class _Boom:
    """写就报错的流：打印失败不能影响控制循环。"""

    def write(self, text):
        raise OSError("terminal gone")

    def flush(self):
        raise OSError("terminal gone")


class ConsoleStatusTests(unittest.TestCase):
    def setUp(self):
        self.buffer = io.StringIO()
        self.console = ConsoleStatus(stream=self.buffer, heartbeat_interval=2.0)

    @property
    def text(self):
        return self.buffer.getvalue()

    def test_line_loss_is_shown(self):
        self.console.update(decision("TRACKING"), 1.0)
        self.console.update(decision("LINE_LOST"), 1.4)
        self.assertIn("LINE_LOST", self.text)
        self.assertIn("丢线", self.text)

    def test_unchanged_state_is_not_repeated(self):
        for index in range(50):
            self.console.update(decision("TRACKING"), 1.0 + index * 0.01)
        self.assertEqual(
            self.text.count("正常跟线"), 1, "同一个状态只能打一次"
        )

    def test_video_loss_is_shown(self):
        self.console.update(decision("VIDEO_LOST"), 1.0)
        self.assertIn("VIDEO_LOST", self.text)
        self.assertIn("视频失效", self.text)

    def test_task_takeover_and_release_are_shown(self):
        self.console.update(
            decision("TRACKING", TASK_ACTIVE, "number_marker", "RUNNING",
                     "STOPPING:TARGET_FOUND"),
            1.0,
        )
        self.console.update(
            decision("TRACKING", RELEASING, "number_marker", "COMPLETED",
                     "task completed"),
            1.5,
        )
        self.assertIn(">> number_marker 接管", self.text)
        self.assertIn("<< number_marker 结束（COMPLETED）", self.text)

    def test_errors_and_clamping_are_shown(self):
        self.console.update(
            decision(errors=("task command clamped from (1, 0, 0) to (0.3, 0, 0)",)),
            1.0,
        )
        self.assertIn("!!", self.text)
        self.assertIn("clamped", self.text)

    def test_saved_scoring_snapshot_is_announced(self):
        self.console.update(decision(), 1.0, saved_evidence=1)
        self.console.update(decision(), 1.1, saved_evidence=1)
        self.assertIn("得分截图已保存", self.text)
        self.assertIn("第 2 张", self.text)

    def test_heartbeat_is_throttled_but_proves_liveness(self):
        for index in range(30):
            self.console.update(decision(), 1.0 + index * 0.01)
        self.assertEqual(
            len(self.text.splitlines()), 1,
            "0.3 秒里只该有状态变化那一行，不能逐帧刷屏",
        )
        self.console.update(decision(), 3.5)
        heartbeats = [
            line for line in self.text.splitlines() if "-- 运行" in line
        ]
        self.assertEqual(len(heartbeats), 1, "过了心跳间隔才该出现心跳行")
        self.assertIn("帧 31", heartbeats[0])

    def test_heartbeat_shows_how_long_the_line_has_been_lost(self):
        self.console.update(decision("LINE_LOST"), 1.0)
        self.console.update(decision("LINE_LOST"), 3.5)
        self.assertIn("已丢线", self.text)

    def test_a_broken_terminal_never_raises(self):
        console = ConsoleStatus(stream=_Boom(), heartbeat_interval=2.0)
        console.update(decision("LINE_LOST"), 1.0)
        console.update(
            decision("TRACKING", TASK_ACTIVE, "route", "RUNNING", "x"), 1.2
        )
        console.update(decision(), 1.3, saved_evidence=1)
        console.note("人工停止")

    def test_disabled_console_prints_nothing(self):
        buffer = io.StringIO()
        console = ConsoleStatus(stream=buffer, enabled=False)
        console.update(decision("LINE_LOST"), 1.0, saved_evidence=1)
        console.note("启动信息")
        self.assertEqual(buffer.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
