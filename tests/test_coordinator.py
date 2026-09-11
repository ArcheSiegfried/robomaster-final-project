"""Coordinator state machine tests: who owns motion, and how it is returned.

These tests use the real LineFollower, the real MotionOutput and a recording
fake chassis. No hardware, no camera, no SDK.
"""

import time
import unittest
from dataclasses import replace

from tests.task_harness import (
    CONFIG,
    TaskHarness,
    blank_frame,
    line_frame,
)

from config import RuntimeConfig
from coordinator import (
    LINE_FOLLOWING,
    RELEASING,
    TASK_ACTIVE,
    TaskCoordinator,
)
from models import (
    FramePacket,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)
from motion_output import MotionOutput
from runtime import LINE_LOST, STOPPED, TRACKING, VIDEO_LOST, LineFollower


def config_with(**task_overrides) -> RuntimeConfig:
    return replace(CONFIG, tasks=replace(CONFIG.tasks, **task_overrides))


class ScriptedTask:
    """Returns a scripted list of TaskUpdate; the last entry repeats."""

    def __init__(self, updates, name="scripted"):
        self.name = name
        self.calls = 0
        self._updates = list(updates)

    def step(self, frame, now):
        self.calls += 1
        index = min(self.calls - 1, len(self._updates) - 1)
        return self._updates[index]


class NeverTask:
    def __init__(self, name="never"):
        self.name = name
        self.calls = 0

    def step(self, frame, now):
        self.calls += 1
        return TaskUpdate(TaskStatus.NOT_TRIGGERED)


class ExplodingTask:
    name = "exploding"

    def step(self, frame, now):
        raise RuntimeError("task blew up")


class ExplodingAfterTakeover:
    """Takes over on the first call, then raises on every later call."""

    name = "exploding_after_takeover"

    def __init__(self):
        self.calls = 0

    def step(self, frame, now):
        self.calls += 1
        if self.calls == 1:
            return TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(forward=0.1))
        raise RuntimeError("task blew up while owning motion")


class SlowTask:
    name = "slow"

    def __init__(self, delay=0.05):
        self.delay = delay

    def step(self, frame, now):
        time.sleep(self.delay)
        return TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(forward=0.1))


class RecordingObserver:
    def __init__(self, name="observer"):
        self.name = name
        self.seen = []

    def observe(self, frame, now):
        self.seen.append((frame.sequence, now))


class ExplodingObserver:
    name = "exploding_observer"

    def observe(self, frame, now):
        raise RuntimeError("observer blew up")


def running(motion=None):
    return TaskUpdate(TaskStatus.RUNNING, motion=motion)


class CoordinatorTests(unittest.TestCase):
    # -- normal line following -----------------------------------------
    def test_line_following_keeps_line_owner(self):
        harness = TaskHarness()
        harness.start_line(now=1.0)
        decision = harness.feed_line(1.10, x=330)
        self.assertEqual(decision.owner, "line")
        self.assertEqual(decision.state, LINE_FOLLOWING)
        self.assertTrue(harness.chassis.motion_calls)
        self.assertEqual(harness.chassis.last_speed()["x"] > 0, True)

    def test_not_triggered_task_never_takes_over(self):
        task = NeverTask()
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        for step in range(4):
            harness.feed_line(1.10 + step * 0.05)
        self.assertEqual(harness.owner, "line")
        self.assertIsNone(harness.task_name)
        self.assertGreaterEqual(task.calls, 4)

    # -- takeover -------------------------------------------------------
    def test_running_task_takes_over_and_pauses_the_line(self):
        task = ScriptedTask([running(MotionCommand(forward=0.1))])
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        decision = harness.feed_line(1.10)
        self.assertEqual(decision.state, TASK_ACTIVE)
        self.assertEqual(harness.owner, "external")
        self.assertEqual(harness.task_name, "scripted")
        self.assertFalse(harness.follower.motion_enabled)
        self.assertEqual(harness.follower.state, STOPPED)

    def test_takeover_hard_stops_before_handing_over_control(self):
        task = ScriptedTask(
            [TaskUpdate(TaskStatus.NOT_TRIGGERED), running(MotionCommand(forward=0.1))]
        )
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        self.assertEqual(harness.owner, "line")
        before = len(harness.chassis.hard_stops)
        harness.feed_line(1.10)
        self.assertEqual(harness.owner, "external")
        self.assertGreater(len(harness.chassis.hard_stops), before)

    def test_running_task_motion_is_forwarded(self):
        task = ScriptedTask([running(MotionCommand(forward=0.12, yaw=8.0))])
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        last = harness.chassis.last_speed()
        self.assertAlmostEqual(last["x"], 0.12)
        self.assertAlmostEqual(last["z"], 8.0)

    def test_running_without_motion_stops_but_holds_ownership(self):
        task = ScriptedTask([running(None)])
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        decision = harness.feed_line(1.10)
        self.assertEqual(harness.owner, "external")
        self.assertEqual(decision.command, MotionCommand())
        self.assertAlmostEqual(harness.chassis.last_speed()["x"], 0.0)

    def test_only_the_first_running_task_takes_over(self):
        first = ScriptedTask([running(MotionCommand(forward=0.1))], name="first")
        second = ScriptedTask([running(MotionCommand(forward=0.1))], name="second")
        harness = TaskHarness()
        harness.coordinator = TaskCoordinator(
            CONFIG,
            harness.follower,
            harness.output,
            motion_tasks=(first, second),
            observers=(),
        )
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        self.assertEqual(harness.task_name, "first")
        self.assertEqual(second.calls, 0)

    def test_other_tasks_are_not_asked_while_one_is_active(self):
        first = ScriptedTask([running(MotionCommand(forward=0.1))], name="first")
        second = NeverTask(name="second")
        harness = TaskHarness()
        harness.coordinator = TaskCoordinator(
            CONFIG,
            harness.follower,
            harness.output,
            motion_tasks=(first, second),
            observers=(),
        )
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        harness.feed_line(1.15)
        harness.feed_line(1.20)
        self.assertEqual(second.calls, 0)

    # -- releasing ------------------------------------------------------
    def test_completed_task_stops_and_returns_ownership(self):
        task = ScriptedTask(
            [running(MotionCommand(forward=0.1)), TaskUpdate(TaskStatus.COMPLETED)]
        )
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        decision = harness.feed_line(1.10)
        self.assertEqual(decision.state, RELEASING)
        self.assertEqual(harness.owner, "line")
        self.assertIsNone(harness.task_name)
        self.assertTrue(decision.force_stop)

    def test_failed_task_is_released_the_same_way(self):
        task = ScriptedTask(
            [running(MotionCommand(forward=0.1)), TaskUpdate(TaskStatus.FAILED)]
        )
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        decision = harness.feed_line(1.10)
        self.assertEqual(decision.state, RELEASING)
        self.assertEqual(harness.owner, "line")
        self.assertTrue(decision.force_stop)

    def test_not_triggered_while_active_is_treated_as_failure(self):
        task = ScriptedTask(
            [running(MotionCommand(forward=0.1)), TaskUpdate(TaskStatus.NOT_TRIGGERED)]
        )
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        decision = harness.feed_line(1.10)
        self.assertEqual(decision.state, RELEASING)
        self.assertEqual(harness.owner, "line")
        self.assertTrue(any("NOT_TRIGGERED" in error for error in decision.errors))

    def test_task_exception_before_takeover_is_recorded_and_ignored(self):
        """A broken module must not be able to stop normal line following."""
        harness = TaskHarness(task=ExplodingTask())
        harness.start_line(now=1.0)
        decision = harness.feed_line(1.10)
        self.assertEqual(harness.owner, "line")
        self.assertFalse(decision.force_stop)
        self.assertTrue(any("exception" in error for error in decision.errors))
        self.assertTrue(harness.chassis.motion_calls)

    def test_task_exception_while_owning_releases_and_stops(self):
        harness = TaskHarness(task=ExplodingAfterTakeover())
        harness.start_line(now=1.0)
        self.assertEqual(harness.owner, "external")
        decision = harness.feed_line(1.10)
        self.assertEqual(harness.owner, "line")
        self.assertIsNone(harness.task_name)
        self.assertTrue(decision.force_stop)
        self.assertTrue(any("exception" in error for error in decision.errors))

    def test_task_running_too_long_is_released(self):
        task = ScriptedTask([running(MotionCommand(forward=0.1))])
        harness = TaskHarness(task=task, config=config_with(max_task_seconds=0.05))
        harness.start_line(now=1.0)
        decision = harness.feed_line(1.10)
        self.assertEqual(harness.owner, "line")
        self.assertTrue(decision.force_stop)
        self.assertTrue(any("max_task_seconds" in error for error in decision.errors))

    # -- release to resume ---------------------------------------------
    def test_release_waits_for_a_fresh_valid_frame_before_resuming(self):
        task = ScriptedTask(
            [running(MotionCommand(forward=0.1)), TaskUpdate(TaskStatus.COMPLETED)]
        )
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        release = harness.feed_line(1.10)
        self.assertEqual(release.state, RELEASING)
        self.assertEqual(harness.owner, "line")

        harness.feed_blank(1.15)
        self.assertEqual(harness.state, RELEASING)
        self.assertFalse(harness.follower.motion_enabled)

        harness.feed_line(1.20)
        self.assertEqual(harness.state, LINE_FOLLOWING)
        self.assertEqual(harness.follower.state, TRACKING)

        before = len(harness.chassis.motion_calls)
        harness.feed_line(1.25)
        self.assertGreater(len(harness.chassis.motion_calls), before)
        self.assertEqual(harness.owner, "line")

    def test_release_timeout_leaves_the_robot_stopped(self):
        task = ScriptedTask(
            [running(MotionCommand(forward=0.1)), TaskUpdate(TaskStatus.COMPLETED)]
        )
        harness = TaskHarness(task=task, config=config_with(release_resume_timeout=0.1))
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        before = len(harness.chassis.motion_calls)
        decision = harness.feed_blank(1.50)
        self.assertEqual(harness.state, LINE_FOLLOWING)
        self.assertTrue(decision.force_stop)
        self.assertFalse(harness.follower.motion_enabled)
        harness.feed_blank(1.60)
        self.assertEqual(len(harness.chassis.motion_calls), before)

    # -- command safety envelope ---------------------------------------
    def test_task_command_is_clamped_to_config_limits(self):
        task = ScriptedTask(
            [running(MotionCommand(forward=5.0, lateral=9.0, yaw=9999.0))]
        )
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        last = harness.chassis.last_speed()
        self.assertAlmostEqual(last["x"], CONFIG.tasks.task_max_forward)
        self.assertAlmostEqual(last["y"], CONFIG.tasks.task_max_lateral)
        self.assertAlmostEqual(last["z"], CONFIG.tasks.task_max_yaw)

    def test_non_finite_task_command_is_zeroed(self):
        task = ScriptedTask(
            [
                running(
                    MotionCommand(
                        forward=float("nan"),
                        lateral=float("inf"),
                        yaw=float("-inf"),
                    )
                )
            ]
        )
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        last = harness.chassis.last_speed()
        self.assertEqual((last["x"], last["y"], last["z"]), (0.0, 0.0, 0.0))

    # -- observers ------------------------------------------------------
    def test_observer_sees_every_frame_without_owning_motion(self):
        observer = RecordingObserver()
        harness = TaskHarness(observer=observer)
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        harness.feed_blank(1.15)
        harness.feed_line(1.20)
        self.assertEqual(harness.owner, "line")
        self.assertEqual([entry[0] for entry in observer.seen], [2, 3, 4, 5])

    def test_observer_exception_is_recorded_and_ignored(self):
        harness = TaskHarness(observer=ExplodingObserver())
        harness.start_line(now=1.0)
        decision = harness.feed_line(1.10)
        self.assertTrue(any("observer" in error for error in decision.errors))
        self.assertTrue(harness.chassis.motion_calls)
        self.assertEqual(harness.owner, "line")

    def test_slow_step_is_recorded(self):
        harness = TaskHarness(
            task=SlowTask(delay=0.05), config=config_with(max_step_seconds=0.01)
        )
        harness.start_line(now=1.0)
        decision = harness.feed_line(1.10)
        self.assertTrue(any("slow" in error for error in decision.errors))

    # -- takeover is gated on the base line being armed -----------------
    def test_task_cannot_take_over_before_the_line_is_started(self):
        """A task must never start the robot by itself.

        Before the operator resumes the line follower the base is STOPPED, and
        no module may claim motion from that state.
        """
        task = ScriptedTask([running(MotionCommand(forward=0.1))])
        harness = TaskHarness(task=task)
        harness.feed_line(1.0)
        harness.feed_line(1.05)
        self.assertEqual(harness.owner, "line")
        self.assertIsNone(harness.task_name)
        self.assertFalse(harness.chassis.motion_calls)

    def test_task_can_take_over_after_a_line_lost_lock(self):
        """WP6 needs to take over a line that the base already gave up on."""
        task = NeverTask()
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        harness.feed_blank(1.6)
        self.assertEqual(harness.follower.state, LINE_LOST)
        self.assertTrue(harness.coordinator.takeover_allowed)

    # -- line faults during a takeover ---------------------------------
    def test_line_fault_does_not_interrupt_an_active_task(self):
        task = ScriptedTask([running(MotionCommand(forward=0.1))])
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        harness.feed_blank(1.15)
        harness.feed_blank(1.60)
        self.assertEqual(harness.owner, "external")
        self.assertEqual(harness.follower.state, STOPPED)

    # -- human override -------------------------------------------------
    def test_human_stop_ends_takeover_and_requires_explicit_resume(self):
        task = ScriptedTask([running(MotionCommand(forward=0.1))])
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        self.assertEqual(harness.owner, "external")

        harness.coordinator.human_stop(1.15)
        self.assertIsNone(harness.task_name)
        self.assertEqual(harness.owner, "line")
        self.assertFalse(harness.follower.motion_enabled)

        before = len(harness.chassis.motion_calls)
        harness.feed_line(1.20)
        self.assertEqual(len(harness.chassis.motion_calls), before)
        self.assertFalse(harness.follower.motion_enabled)

        self.assertTrue(harness.coordinator.human_resume(1.25))
        self.assertTrue(harness.follower.motion_enabled)

    def test_human_reset_ends_takeover(self):
        task = ScriptedTask([running(MotionCommand(forward=0.1))])
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        harness.coordinator.human_reset(1.15)
        self.assertIsNone(harness.task_name)
        self.assertEqual(harness.owner, "line")
        self.assertEqual(harness.follower.state, STOPPED)

    def test_video_gap_ends_takeover_and_latches_video_lost(self):
        task = ScriptedTask([running(MotionCommand(forward=0.1))])
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        harness.feed_line(1.10)
        decision = harness.coordinator.video_gap(
            CONFIG.video_gap_stop_seconds, 1.30
        )
        self.assertIsNone(harness.task_name)
        self.assertEqual(harness.owner, "line")
        self.assertEqual(harness.follower.state, VIDEO_LOST)
        self.assertTrue(decision.force_stop)

        after = harness.coordinator.step(
            FramePacket(line_frame(), 99, 1.40), 1.40
        )
        self.assertEqual(after.state, LINE_FOLLOWING)
        self.assertFalse(harness.follower.motion_enabled)


if __name__ == "__main__":
    unittest.main()
