"""Small, hardware-free checks for integrated scoring-image handoff."""

import unittest
from types import SimpleNamespace

import numpy as np

from evidence import IntegratedScoreEvidence, render_task_evidence
from models import FramePacket, TaskStatus, TaskUpdate, VisualDetection


class Recorder:
    def __init__(self, saved=True):
        self.saved = saved
        self.requests = []

    def save_task_evidence(self, request):
        self.requests.append(request)
        return self.saved


def decision(name, status, detection=None, message=""):
    return SimpleNamespace(task_name=name,
        task_update=TaskUpdate(status, detection=detection, message=message))


class IntegratedScoreEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.frame = FramePacket(np.zeros((90, 160, 3), np.uint8), 7, 1.0)
        self.visual = VisualDetection(True, "target", box=(10, 12, 50, 52))

    def exercise_task(self, name, task, expected_shape="rectangle"):
        tracker = IntegratedScoreEvidence()
        recorder = Recorder()
        coordinator = SimpleNamespace(motion_tasks=(task,))
        running = decision(name, TaskStatus.RUNNING, self.visual)
        complete = decision(name, TaskStatus.COMPLETED, self.visual)
        self.assertEqual(tracker.process(self.frame, running, coordinator, recorder), 0)
        self.assertEqual(recorder.requests, [])
        self.assertEqual(tracker.process(self.frame, complete, coordinator, recorder), 1)
        self.assertEqual(len(recorder.requests), 1)
        request = recorder.requests[0]
        self.assertIn("Team 10", request.annotation)
        self.assertEqual(getattr(request, "shape", "rectangle"), expected_shape)
        self.assertEqual(render_task_evidence(request).shape, self.frame.image.shape)
        self.assertEqual(tracker.process(self.frame, complete, coordinator, recorder), 0)

    def test_obstacle_photo_waits_until_dodge_completed(self):
        self.exercise_task("obstacle", SimpleNamespace(name="obstacle", last_side="left"))

    def test_free_junction_marks_the_blocking_robot(self):
        branch = SimpleNamespace(value="right")
        reading = SimpleNamespace(reading="left", left_box=(10, 12, 50, 52))
        self.exercise_task("free_junction", SimpleNamespace(
            name="free_junction", chosen_branch=branch, last_blockage=reading))

    def test_green_fork_marks_the_deciding_lamp(self):
        branch = SimpleNamespace(value="left")
        lamp = SimpleNamespace(branch=branch, color=SimpleNamespace(value="green"),
                               box=(10, 12, 50, 52))
        self.exercise_task("green_junction", SimpleNamespace(
            name="green_junction", chosen_branch=branch, last_readings=(lamp,)),
            "circle")

    def test_ordinary_green_is_independent_of_red_photo(self):
        visual = VisualDetection(True, "traffic_light", color="green",
                                 box=(10, 12, 50, 52))
        tracker = IntegratedScoreEvidence()
        recorder = Recorder()
        coordinator = SimpleNamespace(motion_tasks=())
        self.assertEqual(tracker.process(self.frame,
            decision("traffic_light", TaskStatus.COMPLETED, visual),
            coordinator, recorder), 1)
        self.assertEqual(recorder.requests[0].shape, "circle")
        self.assertIn("continues", recorder.requests[0].annotation)

    def test_missing_or_failed_write_has_bounded_failure_result(self):
        tracker = IntegratedScoreEvidence()
        coordinator = SimpleNamespace(motion_tasks=())
        visual = VisualDetection(True, "traffic_light", color="green",
                                 box=(10, 12, 50, 52))
        complete = decision("traffic_light", TaskStatus.COMPLETED, visual)
        self.assertEqual(tracker.process(self.frame, complete, coordinator,
                                         Recorder(saved=False)), 0)
        self.assertEqual(len(tracker.failures), 1)

    def test_route_requires_a_valid_new_line(self):
        tracker = IntegratedScoreEvidence()
        recorder = Recorder()
        coordinator = SimpleNamespace(motion_tasks=())
        visual = VisualDetection(True, "route", box=(10, 12, 50, 52))
        self.assertEqual(tracker.process(self.frame,
            decision("route", TaskStatus.COMPLETED, visual,
                     "base line detector confirmed centered new route"),
            coordinator, recorder), 1)
        self.assertIn("correct line", recorder.requests[0].annotation)

    def test_connected_corner_does_not_create_false_gap_evidence(self):
        tracker = IntegratedScoreEvidence()
        recorder = Recorder()
        coordinator = SimpleNamespace(motion_tasks=())
        visual = VisualDetection(True, "route", box=(10, 12, 50, 52))
        self.assertEqual(tracker.process(self.frame,
            decision("route", TaskStatus.COMPLETED, visual,
                     "connected corner returned to the base line detector"),
            coordinator, recorder), 0)
        self.assertEqual(recorder.requests, [])


if __name__ == "__main__":
    unittest.main()
