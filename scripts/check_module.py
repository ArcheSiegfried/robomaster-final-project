"""Run one task module's own checks plus a synthetic smoke trace.

Usage:
    python scripts/check_module.py number_marker
    python scripts/check_module.py evidence --frames 24
    python scripts/check_module.py --all

What it does, for one module only:

  1. static contract check  -- the module file must not contain SDK calls, a
     second camera, the motion outlet, blocking calls or absolute paths;
  2. module test file       -- runs tests/test_<module>.py;
  3. smoke trace            -- drives the module through the real skeleton
     (LineFollower + TaskCoordinator + MotionOutput) with synthetic frames and
     a fake chassis, and prints who owned motion on every frame.

It never opens a camera, never imports the SDK and never touches the network.
Exit code 0 means the module is still compatible with the skeleton.
"""

import argparse
import inspect
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import task_registry  # noqa: E402
from config import CONFIG  # noqa: E402
from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_module_source_is_clean,
    scan_module_files,
)


class _Checker(unittest.TestCase):
    """Borrows unittest's assertion methods for the standalone static check."""

    def runTest(self):
        pass


def resolve(module_name):
    """Return (class, is_observer) for a registered module.

    Accepts either the registered `name` ("route") or the file stem
    ("route_task"), because members will type whichever one they remember.
    """
    for cls in task_registry.MOTION_TASK_CLASSES:
        if module_name in (cls.name, pathlib.Path(module_file_for(cls)).stem):
            return cls, False
    for cls in task_registry.OBSERVER_CLASSES:
        if module_name in (cls.name, pathlib.Path(module_file_for(cls)).stem):
            return cls, True
    return None, None


def module_file_for(cls):
    """The real source file of a registered class.

    A module's `name` and its filename need not match (obstacle -> obstacle_task.py),
    so never guess the filename from the name.
    """
    return pathlib.Path(inspect.getsourcefile(cls)).name


def registered_names():
    return [cls.name for cls in task_registry.MOTION_TASK_CLASSES] + [
        cls.name for cls in task_registry.OBSERVER_CLASSES
    ]


def run_static_check(module_file):
    assert_module_source_is_clean(_Checker(), module_file)
    return f"{module_file} static contract OK"


def run_module_tests(module_file):
    """Run tests/test_<module_file stem>.py, derived from the file, not the name."""
    stem = pathlib.Path(module_file).stem
    suite = unittest.defaultTestLoader.loadTestsFromName(f"tests.test_{stem}")
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=1).run(suite)
    return result.wasSuccessful(), result.testsRun


def smoke(module_name, cls, observer, frames=16):
    harness = TaskHarness(observer=cls()) if observer else TaskHarness(task=cls())
    harness.start_line(now=1.0)
    print()
    print(f"{'seq':>4} {'now':>7} {'frame':>6} {'state':<15} {'owner':<9} "
          f"{'task':<10} {'v':>6} {'yaw':>7}  errors")
    print("-" * 96)

    now = 1.05
    for index in range(frames):
        blank = index % 5 == 4  # every fifth frame loses the line
        decision = harness.feed(now, x=320 + (index % 5) * 5, blank=blank)
        trace = harness.traces[-1]
        print(
            f"{harness.sequence:>4} {now:>7.2f} "
            f"{'blank' if blank else 'line':>6} "
            f"{decision.state:<15} {decision.owner:<9} "
            f"{decision.task_name or '-':<10} "
            f"{decision.command.forward:>6.2f} {decision.command.yaw:>7.1f}  "
            f"{'; '.join(decision.errors) if decision.errors else ''}"
        )
        now += 0.05

    took_over = any(row.task_name for row in harness.traces)
    print("-" * 96)
    print(f"frames fed        : {frames}")
    print(f"frames taken over : {sum(1 for r in harness.traces if r.task_name)}")
    print(f"final owner       : {harness.owner}")
    print(f"final state       : {harness.state}")
    print(f"line state        : {harness.follower.state}")
    if not took_over:
        print(
            "note: this module never took over. A stub is expected to stay inert; "
            "a finished implementation should take over when its target is present."
        )
    return harness


def check_one(module_name, module_file, frames, show_trace=True):
    print("=" * 96)
    print(f"module: {module_name}   file: {module_file}")
    print("=" * 96)

    cls, observer = resolve(module_name)
    if cls is None:
        print(f"FAIL: {module_name} is not registered in task_registry.py")
        return False

    try:
        print(run_static_check(module_file))
    except AssertionError as error:
        print(f"FAIL: {error}")
        return False

    tests_ok, count = run_module_tests(module_file)
    stem = pathlib.Path(module_file).stem
    if not tests_ok:
        print(f"FAIL: tests/test_{stem}.py did not pass")
        return False
    print(f"tests/test_{stem}.py OK ({count} tests)")

    if show_trace:
        harness = smoke(module_name, cls, observer, frames=frames)
        if harness.owner == "external":
            print(
                "WARNING: the module still owned motion when the trace ended. "
                "Every takeover must end with COMPLETED or FAILED."
            )
            return False

    print(f"RESULT: {module_name} OK")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("module", nargs="?", help="registered module name")
    parser.add_argument("--all", action="store_true", help="check every module")
    parser.add_argument("--frames", type=int, default=16, help="smoke frames")
    parser.add_argument("--quiet", action="store_true", help="skip the trace")
    args = parser.parse_args()

    if args.all:
        names = registered_names()
    elif args.module:
        names = [args.module]
    else:
        parser.error("give a module name or --all")

    scanned = scan_module_files()
    ok = True
    for name in names:
        cls, _observer = resolve(name)
        if cls is None:
            print(f"FAIL: {name} is not registered in task_registry.py")
            ok = False
            continue
        module_file = module_file_for(cls)
        if module_file not in scanned:
            print(f"FAIL: {module_file} does not define a task or observer class")
            ok = False
            continue
        if not check_one(name, module_file, args.frames, show_trace=not args.quiet):
            ok = False

    print()
    print("MODULE_CHECK_OK" if ok else "MODULE_CHECK_FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
