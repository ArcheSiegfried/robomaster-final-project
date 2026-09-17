"""Only the selected single-green fork layout is enabled in production."""

import unittest

from green_junction import (
    Branch, BranchGeometry, GreenJunctionTask, JunctionDetection,
    LightColor, LightReading, evaluate_branches,
)


class SingleGreenForkTests(unittest.TestCase):
    def setUp(self):
        self.task = GreenJunctionTask()
        self.branches = (
            BranchGeometry(Branch.LEFT, (120.0, 220.0), -35.0, 8),
            BranchGeometry(Branch.RIGHT, (520.0, 220.0), 35.0, 8),
        )

    def test_one_green_selects_its_side_and_requests_matching_yaw(self):
        for side, sign in ((Branch.LEFT, -1), (Branch.RIGHT, 1)):
            with self.subTest(side=side):
                reading = LightReading(LightColor.GREEN, side, 0.9,
                                       (10, 10, 40, 40))
                self.assertIs(self.task._single_green_reading([reading]), reading)
                chosen, _ = evaluate_branches(self.branches, [reading],
                                              self.task.settings)
                self.assertIs(chosen.side, side)
                self.task.last_detection = JunctionDetection(
                    True, branches=self.branches)
                self.task.chosen_branch = side
                command = self.task._turn_command(0.0)
                self.assertGreater(command.yaw * sign, 0.0)
                self.assertGreater(command.forward, 0.0)

    def test_unknown_or_conflicting_lamps_cannot_select(self):
        green = LightReading(LightColor.GREEN, Branch.LEFT, 0.9)
        red = LightReading(LightColor.RED, Branch.RIGHT, 0.9)
        self.assertIsNone(self.task._single_green_reading([]))
        self.assertIsNone(self.task._single_green_reading([red]))
        self.assertIsNone(self.task._single_green_reading([green, red]))
        self.assertIsNone(self.task._single_green_reading([green, green]))
        self.assertIsNone(self.task._single_green_reading(
            [LightReading(LightColor.GREEN, None, 0.9)]))


if __name__ == "__main__":
    unittest.main()
