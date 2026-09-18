"""控制权变更日志：`[ARB] owner changed: A -> B  reason=... priority=.. > ..`

仲裁层 v0.3 新增的可观测性：实车调试时看一眼终端就知道"谁把车交给了谁、为什么"。
之前终端只打协调器的 state 变化（接管/结束），`owner` 只出现在画面文字里。

只用假模块、假底盘、合成帧。
"""

import io
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from coordinator import CoordinatorDecision  # noqa: E402
from main import ConsoleStatus  # noqa: E402
from models import MotionCommand, STOP_COMMAND, TaskStatus, TaskUpdate  # noqa: E402
from tests.task_harness import TaskHarness  # noqa: E402


class OwnerChangeLogTests(unittest.TestCase):
    def test_owner_change_is_printed_once(self):
        stream = io.StringIO()
        console = ConsoleStatus(stream=stream, enabled=True)
        line = ("[ARB] owner changed: line -> obstacle  reason=go  priority=70 > 10")
        decision = CoordinatorDecision(
            state="TASK_ACTIVE",
            owner="external",
            task_name="obstacle",
            command=STOP_COMMAND,
            message="go",
            owner_change=line,
        )

        console.update(decision, 1.0)
        console.update(decision, 1.1)

        text = stream.getvalue()
        self.assertEqual(text.count("[ARB] owner changed"), 1, "同一行不许重复打")
        self.assertIn("line -> obstacle", text)
        self.assertIn("priority=70 > 10", text)

    def test_no_owner_change_prints_nothing_extra(self):
        stream = io.StringIO()
        console = ConsoleStatus(stream=stream, enabled=True)
        console.update(CoordinatorDecision(state="LINE_FOLLOWING", owner="line"), 1.0)
        self.assertNotIn("[ARB]", stream.getvalue())

    def test_a_real_takeover_prints_the_arb_line(self):
        class Stub:
            name = "obstacle"
            priority = 70

            def reset(self):
                pass

            def step(self, frame, now):
                return TaskUpdate(
                    status=TaskStatus.RUNNING,
                    motion=MotionCommand(forward=0.2),
                    message="stepping aside",
                )

        harness = TaskHarness(tasks=(Stub(),))
        packet = harness.frame(1.0, 320)
        harness.follower.process_frame(packet.image, packet.captured_at)
        self.assertTrue(harness.follower.resume(packet.captured_at))
        stream = io.StringIO()
        console = ConsoleStatus(stream=stream, enabled=True)

        decision = harness.feed_line(1.05)
        console.update(decision, 1.05)

        text = stream.getvalue()
        self.assertIn("[ARB] owner changed: line -> obstacle", text)
        self.assertIn("reason=stepping aside", text)
        self.assertIn("priority=70 > 10", text)


if __name__ == "__main__":
    unittest.main()
