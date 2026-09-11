"""Offline harness that drives one task module through the real skeleton.

A module author uses this to test their own file against the real
LineFollower + TaskCoordinator + MotionOutput chain using synthetic frames and
a fake chassis. Nothing here opens a camera, imports the SDK or touches the
network.

Typical use inside tests/test_<module>.py:

    from tests.task_harness import TaskHarness, assert_module_is_clean

    class MyTests(unittest.TestCase):
        def test_inert(self):
            harness = TaskHarness(task=MyTask())
            ...
"""

import ast
import pathlib
import re
import sys
import unittest

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import CONFIG  # noqa: E402
from coordinator import TaskCoordinator  # noqa: E402
from models import FramePacket  # noqa: E402
from motion_output import MotionOutput  # noqa: E402
from runtime import LineFollower  # noqa: E402

# Root-level .py files that belong to the skeleton, not to a task module.
INFRASTRUCTURE_FILES = {
    "__init__.py",
    "config.py",
    "models.py",
    "camera_source.py",
    "line_detector.py",
    "controller.py",
    "runtime.py",
    "motion_output.py",
    "coordinator.py",
    "task_registry.py",
    "main.py",
}

# Patterns a task module must never contain. A module returns MotionCommand;
# only motion_output.py is allowed to talk to the chassis.
FORBIDDEN_PATTERNS = (
    (r"\brobomaster\b", "must not import or reference the RoboMaster SDK"),
    (r"\bdrive_speed\b", "must not call the chassis directly"),
    (r"\bdrive_wheels\b", "must not call the chassis directly"),
    (r"\bmotion_output\b", "must not touch the motion outlet; return MotionCommand"),
    (r"\bLatestFrameSource\b", "must not create a second camera entry"),
    (r"\bcamera_source\b", "must not create a second camera entry"),
    (r"cv2\.VideoCapture", "must not open a video capture of its own"),
    (r"\btime\.sleep\b", "step() must not block"),
    (r"while\s+True", "step() must not contain an unbounded loop"),
    (r"[A-Za-z]:[\\/]", "must not contain an absolute local path"),
)


class FakeChassis:
    """Records drive calls in memory. It has no SDK and no hardware."""

    def __init__(self):
        self.calls = []

    def drive_speed(self, **kwargs):
        self.calls.append(("speed", dict(kwargs)))

    def drive_wheels(self, **kwargs):
        self.calls.append(("wheels", dict(kwargs)))

    @property
    def motion_calls(self):
        """Only the drive_speed calls that would actually move the robot."""
        moving = []
        for kind, kwargs in self.calls:
            if kind != "speed":
                continue
            if kwargs.get("x") or kwargs.get("y") or kwargs.get("z"):
                moving.append(kwargs)
        return moving

    @property
    def hard_stops(self):
        return [kwargs for kind, kwargs in self.calls if kind == "wheels"]

    def last_speed(self):
        for kind, kwargs in reversed(self.calls):
            if kind == "speed":
                return kwargs
        return None


def line_frame(x=320, height=360, width=640):
    """Synthetic frame holding one vertical blue tape line."""
    image = np.full((height, width, 3), 210, np.uint8)
    cv2.line(image, (x, 350), (x, 190), (255, 0, 0), 24)
    return image


def blank_frame(height=360, width=640):
    """Synthetic frame with no line at all."""
    return np.full((height, width, 3), 210, np.uint8)


class TaskHarness:
    """Wire one module into the real coordinator with fake hardware."""

    def __init__(self, task=None, observer=None, config=CONFIG):
        self.config = config
        self.chassis = FakeChassis()
        self.follower = LineFollower(config)
        self.output = MotionOutput(self.chassis, config)
        self.coordinator = TaskCoordinator(
            config,
            self.follower,
            self.output,
            motion_tasks=() if task is None else (task,),
            observers=() if observer is None else (observer,),
        )
        self.sequence = 0
        self.traces = []

    # -- frame helpers -------------------------------------------------
    def frame(self, now, x=320, blank=False):
        self.sequence += 1
        image = blank_frame() if blank else line_frame(x)
        return FramePacket(image, self.sequence, now)

    def feed(self, now, x=320, blank=False):
        decision = self.coordinator.step(self.frame(now, x, blank=blank), now)
        self.traces.append(decision)
        return decision

    def feed_line(self, now, x=320):
        return self.feed(now, x=x)

    def feed_blank(self, now):
        return self.feed(now, blank=True)

    # -- base line startup ---------------------------------------------
    def start_line(self, now=1.0, x=320):
        """Bring the base up to TRACKING so a takeover has something to take."""
        packet = self.frame(now, x)
        self.follower.process_frame(packet.image, packet.captured_at)
        if not self.follower.resume(packet.captured_at):
            raise RuntimeError("harness could not start the line follower")
        return self.feed_line(now + 0.01, x)

    # -- observability -------------------------------------------------
    @property
    def owner(self):
        return self.output.owner

    @property
    def state(self):
        return self.coordinator.state

    @property
    def task_name(self):
        return self.coordinator.active_task_name


def scan_module_files(root=ROOT):
    """Find root-level module files that define a task or observer class.

    A file counts as a module when it has a class with a method whose first
    three parameters are (self, frame, now) named step() or observe(). Those
    files must appear in task_registry, otherwise the skeleton would silently
    ignore them.
    """
    found = {}
    for path in sorted(pathlib.Path(root).glob("*.py")):
        if path.name in INFRASTRUCTURE_FILES:
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except SyntaxError:
            continue
        kinds = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.ClassDef):
                continue
            for item in node.body:
                if not isinstance(item, ast.FunctionDef):
                    continue
                if item.name not in ("step", "observe"):
                    continue
                names = [arg.arg for arg in item.args.args][:3]
                if names == ["self", "frame", "now"]:
                    kinds.add(item.name)
        if kinds:
            found[path.name] = kinds
    return found


def _strip_comments_and_docstrings(text):
    """Scan code only.

    A module author is allowed (encouraged) to write "never call drive_speed()"
    in a comment or docstring. Only real code may trip the contract check.
    """
    text = re.sub(r'"""(?:.|\n)*?"""', '""', text)
    text = re.sub(r"'''(?:.|\n)*?'''", "''", text)
    text = re.sub(r"#[^\n]*", "", text)
    return text


def assert_module_source_is_clean(case, module_file):
    """Shared static check: a module file must obey the safety rules."""
    path = pathlib.Path(ROOT) / module_file
    case.assertTrue(path.exists(), f"{module_file} is missing")
    code = _strip_comments_and_docstrings(path.read_text(encoding="utf-8"))
    for pattern, reason in FORBIDDEN_PATTERNS:
        match = re.search(pattern, code)
        detail = f" (matched {match.group(0)!r})" if match else ""
        case.assertIsNone(
            match,
            f"{module_file} violates the module contract: {reason}{detail}",
        )
    return code


def assert_inert_through_harness(case, module, observer=False):
    """A stub module must never move the robot or take ownership."""
    harness = TaskHarness(observer=module) if observer else TaskHarness(task=module)
    harness.start_line(now=1.0)
    case.assertEqual(harness.owner, "line")
    for step in range(5):
        harness.feed_line(1.1 + step * 0.05, x=320 + step * 4)
    case.assertEqual(harness.owner, "line", "an unimplemented module took control")
    case.assertIsNone(harness.task_name)
    case.assertEqual(harness.state, "LINE_FOLLOWING")
    return harness
