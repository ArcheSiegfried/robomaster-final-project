"""Cross-module contract tests.

Every registered module is checked against the same contract, so replacing a
stub with a real implementation cannot silently break the skeleton. The last
group of tests also makes sure that adding a brand new module file without
registering it is a loud failure instead of a silent no-op.
"""

import inspect
import pathlib
import time
import unittest

from tests.task_harness import (
    CONFIG,
    FakeChassis,
    ROOT,
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
    line_frame,
    scan_module_files,
)

import main
import task_registry
from models import FramePacket, TaskUpdate
from motion_output import MotionOutput
from runtime import LineFollower

# One file per person, one person per file. "step" means the module may take
# over motion; "observe" means it never may.
# evidence.py is infrastructure owned by the integration person, not a slot.
EXPECTED_MODULE_FILES = {
    "number_marker.py": "step",
    "traffic_light.py": "step",
    "obstacle.py": "step",
    "route.py": "step",
    "green_junction.py": "step",
    "free_junction.py": "step",
    "evidence.py": "observe",
}


def synthetic_frame():
    return FramePacket(line_frame(), 1, 1.0)


class RegistryTests(unittest.TestCase):
    def test_expected_module_files_exist(self):
        for filename in EXPECTED_MODULE_FILES:
            self.assertTrue(
                (pathlib.Path(ROOT) / filename).exists(),
                f"module slot file {filename} is missing",
            )

    def test_registry_covers_every_module_file(self):
        registered = set()
        for cls in task_registry.MOTION_TASK_CLASSES + task_registry.OBSERVER_CLASSES:
            source = inspect.getsourcefile(cls)
            self.assertIsNotNone(source, f"{cls.__name__} has no source file")
            registered.add(pathlib.Path(source).name)

        for filename, kinds in scan_module_files().items():
            if kinds == {"observe"}:
                self.assertIn(
                    filename,
                    registered,
                    f"{filename} defines an observer but is not listed in "
                    f"task_registry.OBSERVER_CLASSES",
                )
            else:
                self.assertIn(
                    filename,
                    registered,
                    f"{filename} defines a task class but is not listed in "
                    f"task_registry.MOTION_TASK_CLASSES",
                )

    def test_every_expected_file_is_registered(self):
        registered = set()
        for cls in task_registry.MOTION_TASK_CLASSES + task_registry.OBSERVER_CLASSES:
            registered.add(pathlib.Path(inspect.getsourcefile(cls)).name)
        for filename in EXPECTED_MODULE_FILES:
            self.assertIn(filename, registered, f"{filename} is not registered")

    def test_module_names_are_unique_and_non_empty(self):
        names = []
        for cls in task_registry.MOTION_TASK_CLASSES + task_registry.OBSERVER_CLASSES:
            instance = cls()
            self.assertTrue(
                isinstance(instance.name, str) and instance.name.strip(),
                f"{cls.__name__}.name must be a non-empty string",
            )
            names.append(instance.name)
        self.assertEqual(len(names), len(set(names)), f"duplicate names: {names}")

    def test_registered_classes_are_constructible_without_arguments(self):
        for cls in task_registry.MOTION_TASK_CLASSES + task_registry.OBSERVER_CLASSES:
            try:
                cls()
            except TypeError as error:
                self.fail(f"{cls.__name__} must support a no-argument constructor: {error}")

    def test_builders_return_one_instance_per_registered_class(self):
        tasks = task_registry.build_motion_tasks()
        observers = task_registry.build_observers()
        self.assertEqual(len(tasks), len(task_registry.MOTION_TASK_CLASSES))
        self.assertEqual(len(observers), len(task_registry.OBSERVER_CLASSES))


class ModuleContractTests(unittest.TestCase):
    def test_motion_tasks_return_a_task_update(self):
        frame = synthetic_frame()
        for task in task_registry.build_motion_tasks():
            with self.subTest(task=task.name):
                result = task.step(frame, 1.0)
                self.assertIsInstance(
                    result, TaskUpdate, f"{task.name}.step must return TaskUpdate"
                )

    def test_observers_return_none(self):
        frame = synthetic_frame()
        for observer in task_registry.build_observers():
            with self.subTest(observer=observer.name):
                self.assertIsNone(
                    observer.observe(frame, 1.0),
                    f"{observer.name}.observe must return None",
                )

    def test_registered_steps_are_fast(self):
        frame = synthetic_frame()
        budget = CONFIG.tasks.max_step_seconds * 10
        for task in task_registry.build_motion_tasks():
            with self.subTest(task=task.name):
                started = time.monotonic()
                task.step(frame, 1.0)
                elapsed = time.monotonic() - started
                self.assertLess(
                    elapsed,
                    budget,
                    f"{task.name}.step took {elapsed:.3f}s; it must return immediately",
                )
        for observer in task_registry.build_observers():
            with self.subTest(observer=observer.name):
                started = time.monotonic()
                observer.observe(frame, 1.0)
                elapsed = time.monotonic() - started
                self.assertLess(
                    elapsed,
                    budget,
                    f"{observer.name}.observe took {elapsed:.3f}s; it must return immediately",
                )

    def test_module_sources_obey_the_safety_rules(self):
        for filename in EXPECTED_MODULE_FILES:
            with self.subTest(module=filename):
                assert_module_source_is_clean(self, filename)

    def test_stub_modules_never_take_over(self):
        for task in task_registry.build_motion_tasks():
            with self.subTest(task=task.name):
                assert_inert_through_harness(self, task)
        for observer in task_registry.build_observers():
            with self.subTest(observer=observer.name):
                assert_inert_through_harness(self, observer, observer=True)


class MainWiringTests(unittest.TestCase):
    def test_build_coordinator_wires_every_registered_module(self):
        follower = LineFollower(CONFIG)
        output = MotionOutput(FakeChassis(), CONFIG)
        # capture_directory=None：测试不许在仓库里留 captures/。
        coordinator = main.build_coordinator(follower, output, capture_directory=None)
        self.assertEqual(
            len(coordinator.motion_tasks), len(task_registry.MOTION_TASK_CLASSES)
        )
        self.assertEqual(
            len(coordinator.observers), len(task_registry.OBSERVER_CLASSES)
        )
        self.assertIs(coordinator.follower, follower)
        self.assertIs(coordinator.output, output)
        coordinator.close()

    def test_built_coordinator_keeps_the_robot_stopped_on_a_blank_frame(self):
        chassis = FakeChassis()
        follower = LineFollower(CONFIG)
        output = MotionOutput(chassis, CONFIG)
        coordinator = main.build_coordinator(follower, output, capture_directory=None)
        coordinator.step(FramePacket(line_frame(), 1, 1.0), 1.0)
        self.assertFalse(chassis.motion_calls)
        self.assertEqual(coordinator.step(FramePacket(line_frame(), 2, 1.05), 1.05).owner, "line")
        coordinator.close()

    def test_evidence_recording_is_off_unless_a_directory_is_given(self):
        """默认不写盘：observer 构造时零副作用，测试不会污染仓库。"""
        for observer in task_registry.build_observers():
            with self.subTest(observer=observer.name):
                self.assertFalse(getattr(observer, "enabled", False))


if __name__ == "__main__":
    unittest.main()
