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


class _ControllableAction:
    def __init__(self):
        self.is_completed = False
        self.has_failed = False
        self.state = "action_started"
        self.failure_reason = None

    def succeed(self):
        self.is_completed = True
        self.state = "action_succeeded"

    def fail(self, reason="motor rejected"):
        self.is_completed = True
        self.has_failed = True
        self.state = "action_failed"
        self.failure_reason = reason

    def wait_for_completed(self, timeout=None):
        raise AssertionError("the control loop must never wait for an action")


class _ScriptedGimbal(FakeGimbal):
    def __init__(self, actions=()):
        super().__init__()
        self.actions = list(actions)

    def moveto(self, **kwargs):
        self.calls.append(dict(kwargs))
        return self.actions.pop(0) if self.actions else _ControllableAction()


class _CollisionOnceGimbal(_ScriptedGimbal):
    def __init__(self):
        super().__init__()
        self.collisions = 1

    def moveto(self, **kwargs):
        if self.collisions:
            self.collisions -= 1
            raise RuntimeError("Robot is already performing 1 action(s)")
        return super().moveto(**kwargs)


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
        output.send(command)  # A failed action must not reset the owning task.
        self.assertIn("action_rejected", output.last_error)
        self.assertEqual(output.stats()["action_failures"], 1)

    def test_busy_queues_without_second_action(self):
        gimbal = _ScriptedGimbal()
        output = GimbalOutput(gimbal, CONFIG)
        first = GimbalCommand(-12)
        second = GimbalCommand(-10)
        output.send(first)
        self.assertEqual(output.send(second), second)
        self.assertEqual(len(gimbal.calls), 1)
        self.assertEqual(output.pending, second)
        self.assertTrue(output.busy)

    def test_latest_target_replaces_obsolete_pending(self):
        first_action = _ControllableAction()
        gimbal = _ScriptedGimbal((first_action,))
        output = GimbalOutput(gimbal, CONFIG)
        output.send(GimbalCommand(-12))
        output.send(GimbalCommand(-10))
        output.send(GimbalCommand(-8))
        self.assertEqual(len(gimbal.calls), 1)
        first_action.succeed()
        output.poll()
        self.assertEqual([call["pitch"] for call in gimbal.calls], [-12.0, -8.0])
        self.assertIsNone(output.pending)

    def test_latest_target_cancels_pending_when_it_matches_active(self):
        action = _ControllableAction()
        gimbal = _ScriptedGimbal((action,))
        output = GimbalOutput(gimbal, CONFIG)
        first = GimbalCommand(-12)
        output.send(first)
        output.send(GimbalCommand(-10))
        output.send(first)
        self.assertIsNone(output.pending)
        action.succeed()
        output.poll()
        self.assertEqual(len(gimbal.calls), 1)

    def test_sdk_collision_is_queued_and_recovers_on_poll(self):
        gimbal = _CollisionOnceGimbal()
        output = GimbalOutput(gimbal, CONFIG)
        target = GimbalCommand(-12)
        output.send(target)
        self.assertEqual(output.pending, target)
        self.assertIn("Robot is already performing", output.last_error)
        self.assertEqual(output.stats()["rejected_by_inflight"], 1)
        self.assertFalse(output.busy)
        output.poll()
        self.assertTrue(output.busy)
        self.assertIsNone(output.pending)
        self.assertEqual(output.stats()["started"], 1)

    def test_new_target_overwrites_collision_pending_before_retry(self):
        gimbal = _CollisionOnceGimbal()
        output = GimbalOutput(gimbal, CONFIG)
        output.send(GimbalCommand(-12))
        output.send(GimbalCommand(-8))
        self.assertEqual([call["pitch"] for call in gimbal.calls], [-8.0])

    def test_failed_action_retries_only_twice(self):
        actions = [_ControllableAction() for _ in range(3)]
        gimbal = _ScriptedGimbal(actions)
        output = GimbalOutput(gimbal, CONFIG)
        target = GimbalCommand(-12)
        output.send(target)
        for index in range(3):
            actions[index].fail("motor rejected")
            output.poll()
        self.assertEqual(len(gimbal.calls), 3)
        self.assertEqual(output.stats()["action_failures"], 3)
        self.assertIn("motor rejected", output.last_error)
        self.assertIsNone(output.pending)
        for _ in range(5):
            output.poll()
            output.send(target)
        self.assertEqual(len(gimbal.calls), 3)

    def test_new_target_supersedes_failed_action_retry(self):
        action = _ControllableAction()
        gimbal = _ScriptedGimbal((action,))
        output = GimbalOutput(gimbal, CONFIG)
        output.send(GimbalCommand(-12))
        output.send(GimbalCommand(-10))
        action.fail()
        output.poll()
        self.assertEqual([call["pitch"] for call in gimbal.calls], [-12.0, -10.0])

    def test_busy_restore_queues_line_view_until_action_completes(self):
        aim = _ControllableAction()
        restore = _ControllableAction()
        gimbal = _ScriptedGimbal((aim, restore))
        output = GimbalOutput(gimbal, CONFIG)
        output.send(GimbalCommand(-12))
        output.send(GimbalCommand(-10))
        output.restore_line_view()
        self.assertEqual(output.pending, output.line_view)
        self.assertFalse(output.at_line_view)
        self.assertEqual(len(gimbal.calls), 1)
        aim.succeed()
        output.poll()
        self.assertEqual(gimbal.last_move()["pitch"], CONFIG.gimbal_pitch)
        self.assertFalse(output.at_line_view)
        restore.succeed()
        output.poll()
        self.assertTrue(output.at_line_view)

    def test_new_aim_invalidates_completed_line_view(self):
        restore = _ControllableAction()
        aim = _ControllableAction()
        output = GimbalOutput(_ScriptedGimbal((restore, aim)), CONFIG)
        output.restore_line_view()
        self.assertFalse(output.at_line_view)
        restore.succeed()
        output.poll()
        self.assertTrue(output.at_line_view)
        output.send(GimbalCommand(-12))
        self.assertFalse(output.at_line_view)
        aim.fail()
        output.poll()
        self.assertFalse(output.at_line_view)

    def test_poll_is_nonblocking_and_idempotent(self):
        action = _ControllableAction()
        gimbal = _ScriptedGimbal((action,))
        output = GimbalOutput(gimbal, CONFIG)
        output.send(GimbalCommand(-12))
        for _ in range(10):
            output.poll()  # wait_for_completed raises if ever called.
        self.assertEqual(output.stats()["started"], 1)
        self.assertEqual(output.stats()["action_failures"], 0)
        action.succeed()
        output.poll()
        for _ in range(10):
            output.poll()
        self.assertEqual(len(gimbal.calls), 1)
        self.assertEqual(output.stats()["started"], 1)


if __name__ == "__main__":
    unittest.main()
