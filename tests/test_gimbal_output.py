"""Offline tests for the centralized gimbal outlet."""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import CONFIG  # noqa: E402
from gimbal_output import GimbalOutput  # noqa: E402
from models import GimbalCommand  # noqa: E402
from tests.task_harness import FakeGimbal  # noqa: E402


class _FailedAction:
    state = "action_rejected"


class _RejectingGimbal(FakeGimbal):
    def moveto(self, **kwargs):
        self.calls.append(dict(kwargs))
        return _FailedAction()


class GimbalOutputTests(unittest.TestCase):
    def test_clamps_non_finite_values_and_deduplicates_requests(self):
        gimbal = FakeGimbal()
        output = GimbalOutput(gimbal, CONFIG)
        applied = output.send(GimbalCommand(pitch=999.0, yaw=float("nan")))
        self.assertEqual(applied.pitch, CONFIG.gimbal_pitch_max)
        self.assertEqual(applied.yaw, CONFIG.gimbal_yaw)
        output.send(GimbalCommand(pitch=999.0, yaw=float("nan")))
        self.assertEqual(len(gimbal.calls), 1)

    def test_restore_uses_configured_line_view(self):
        gimbal = FakeGimbal()
        output = GimbalOutput(gimbal, CONFIG)
        output.send(GimbalCommand(CONFIG.gimbal_search_pitch))
        output.restore_line_view()
        self.assertEqual(gimbal.last_move()["pitch"], CONFIG.gimbal_pitch)
        self.assertEqual(gimbal.last_move()["yaw"], CONFIG.gimbal_yaw)

    def test_rejected_action_is_reported_on_next_poll(self):
        output = GimbalOutput(_RejectingGimbal(), CONFIG)
        command = GimbalCommand(CONFIG.gimbal_search_pitch)
        output.send(command)
        self.assertEqual(output.last_action_state, "action_rejected")
        with self.assertRaisesRegex(RuntimeError, "action_rejected"):
            output.send(command)


if __name__ == "__main__":
    unittest.main()
