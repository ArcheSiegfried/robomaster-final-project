"""Small offline checks for one-axis route distance calibration."""

import unittest

from route_distance import DistanceSample, RouteDistanceCalibration
from route import RouteTask


class RouteDistanceCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.calibration = RouteDistanceCalibration.from_samples(
            -12.0,
            (
                DistanceSample(0.55, 0.40),
                DistanceSample(0.75, 0.25),
                DistanceSample(0.95, 0.15),
            ),
        )

    def test_interpolates_only_inside_measured_range(self):
        self.assertAlmostEqual(
            self.calibration.estimate_remaining(0.65), 0.325
        )
        self.assertAlmostEqual(
            self.calibration.estimate_remaining(0.95), 0.15
        )
        self.assertIsNone(self.calibration.estimate_remaining(0.50))
        self.assertIsNone(self.calibration.estimate_remaining(0.98))

    def test_rejects_nonmonotonic_samples(self):
        with self.assertRaises(ValueError):
            RouteDistanceCalibration.from_samples(
                -12.0,
                (
                    DistanceSample(0.55, 0.40),
                    DistanceSample(0.75, 0.30),
                    DistanceSample(0.95, 0.35),
                ),
            )

    def test_live_measurements_update_the_bounded_turn_point(self):
        task = RouteTask()
        task._pose_forward = 0.20
        task._update_bridge_metric_target(0.18)
        self.assertAlmostEqual(task._bridge_target_forward, 0.38)
        task._pose_forward = 0.25
        task._update_bridge_metric_target(0.13)
        self.assertAlmostEqual(task._bridge_target_forward, 0.38)
        self.assertEqual(task._bridge_approach_speed(0.20), 0.15)
        self.assertEqual(task._bridge_approach_speed(0.10), 0.12)
        self.assertEqual(task._bridge_approach_speed(0.03), 0.08)


if __name__ == "__main__":
    unittest.main()
