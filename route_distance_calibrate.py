"""Build route_distance_calibration.json without connecting to a robot.

Examples:
  python route_distance_calibrate.py --point 0.15 0.96 --point 0.25 0.78 \
      --point 0.40 0.55

  python route_distance_calibrate.py --image 0.15 images/15cm.jpg \
      --image 0.25 images/25cm.jpg --image 0.40 images/40cm.jpg

Distances are measured from the chassis rotation centre to the desired turn
point along the current travel axis. Images must come from the robot camera at
CONFIG.gimbal_search_pitch. This script never initializes the SDK or camera.
"""

import argparse
from pathlib import Path
from typing import List

import cv2

from config import CONFIG
from route_detector import RouteVision
from route_distance import (
    DistanceSample,
    RouteDistanceCalibration,
    default_calibration_path,
)


def _sample_from_image(distance_m: float, path: Path) -> DistanceSample:
    frame = cv2.imread(str(path))
    if frame is None:
        raise ValueError("cannot read image: %s" % path)
    candidates = RouteVision(CONFIG.vision).candidates(frame)
    candidates = [
        item
        for item in candidates
        if item.entry_endpoint is not None and item.entry_endpoint.internal
    ]
    if not candidates:
        raise ValueError("no internal route endpoint found in %s" % path)
    candidate = max(
        candidates,
        key=lambda item: (
            item.entry_endpoint.branch_length,
            item.score,
        ),
    )
    ratio = candidate.entry_endpoint.point[1] / float(frame.shape[0])
    print(
        "%s: endpoint_y_ratio=%.6f, remaining_distance_m=%.3f"
        % (path, ratio, distance_m)
    )
    return DistanceSample(ratio, distance_m)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create the one-dimensional route distance calibration."
    )
    parser.add_argument(
        "--point",
        nargs=2,
        action="append",
        metavar=("DISTANCE_M", "ENDPOINT_Y_RATIO"),
        default=[],
        help="add one manually measured distance/vertical-ratio pair",
    )
    parser.add_argument(
        "--image",
        nargs=2,
        action="append",
        metavar=("DISTANCE_M", "IMAGE"),
        default=[],
        help="detect the endpoint ratio from a static robot-camera image",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_calibration_path(),
        help="output JSON path",
    )
    return parser


def main() -> None:
    args = _parser().parse_args()
    samples: List[DistanceSample] = [
        DistanceSample(float(ratio), float(distance))
        for distance, ratio in args.point
    ]
    samples.extend(
        _sample_from_image(float(distance), Path(image_path))
        for distance, image_path in args.image
    )
    calibration = RouteDistanceCalibration.from_samples(
        CONFIG.gimbal_search_pitch,
        samples,
    )
    calibration.save(args.output)
    print("saved %d samples to %s" % (len(calibration.samples), args.output))
    print("This file is local calibration data and is ignored by Git.")


if __name__ == "__main__":
    main()
