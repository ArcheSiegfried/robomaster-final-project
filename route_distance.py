"""One-dimensional ground-distance calibration for route recovery.

The disconnected route is on a flat floor and the search gimbal pitch is
fixed.  Under those constraints, the vertical image coordinate of the
gap-facing endpoint can be mapped to the remaining forward travel needed
before the chassis starts its turn.  The values are deliberately supplied by
real measurements rather than pretending that command speed is odometry.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Iterable, Optional, Tuple


CALIBRATION_FILENAME = "route_distance_calibration.json"
CALIBRATION_VERSION = 1
MIN_DISTANCE_M = 0.10
MAX_DISTANCE_M = 0.45


@dataclass(frozen=True)
class DistanceSample:
    endpoint_y_ratio: float
    remaining_distance_m: float


@dataclass(frozen=True)
class RouteDistanceCalibration:
    search_pitch_deg: float
    samples: Tuple[DistanceSample, ...]

    @classmethod
    def from_samples(
        cls,
        search_pitch_deg: float,
        samples: Iterable[DistanceSample],
    ) -> "RouteDistanceCalibration":
        pitch = float(search_pitch_deg)
        if not math.isfinite(pitch):
            raise ValueError("search_pitch_deg must be finite")
        ordered = tuple(sorted(samples, key=lambda item: item.endpoint_y_ratio))
        if len(ordered) < 3:
            raise ValueError("distance calibration needs at least three samples")
        previous_ratio = -1.0
        previous_distance = float("inf")
        for sample in ordered:
            ratio = float(sample.endpoint_y_ratio)
            distance = float(sample.remaining_distance_m)
            if not math.isfinite(ratio) or not 0.0 < ratio < 1.0:
                raise ValueError("endpoint_y_ratio must be between 0 and 1")
            if not math.isfinite(distance) or not (
                MIN_DISTANCE_M <= distance <= MAX_DISTANCE_M
            ):
                raise ValueError(
                    "remaining_distance_m must be between %.2f and %.2f"
                    % (MIN_DISTANCE_M, MAX_DISTANCE_M)
                )
            if ratio <= previous_ratio:
                raise ValueError("endpoint_y_ratio samples must be unique")
            if distance >= previous_distance:
                raise ValueError(
                    "remaining distance must decrease as endpoint_y_ratio increases"
                )
            previous_ratio = ratio
            previous_distance = distance
        return cls(pitch, ordered)

    @classmethod
    def load(cls, path: Path) -> "RouteDistanceCalibration":
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("version") != CALIBRATION_VERSION:
            raise ValueError("unsupported route distance calibration version")
        samples = (
            DistanceSample(
                float(item["endpoint_y_ratio"]),
                float(item["remaining_distance_m"]),
            )
            for item in payload["samples"]
        )
        return cls.from_samples(float(payload["search_pitch_deg"]), samples)

    def save(self, path: Path) -> None:
        payload = {
            "version": CALIBRATION_VERSION,
            "search_pitch_deg": self.search_pitch_deg,
            "samples": [
                {
                    "endpoint_y_ratio": sample.endpoint_y_ratio,
                    "remaining_distance_m": sample.remaining_distance_m,
                }
                for sample in self.samples
            ],
        }
        path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    def estimate_remaining(self, endpoint_y_ratio: float) -> Optional[float]:
        """Interpolate only inside the measured image range.

        Returning ``None`` outside the table prevents unsafe extrapolation.
        RouteTask may continue a short, already measured remainder after the
        endpoint leaves the lower image edge, but it never invents a new
        metric observation there.
        """
        ratio = float(endpoint_y_ratio)
        if not math.isfinite(ratio):
            return None
        if ratio < self.samples[0].endpoint_y_ratio:
            return None
        if ratio > self.samples[-1].endpoint_y_ratio:
            return None
        for left, right in zip(self.samples, self.samples[1:]):
            if left.endpoint_y_ratio <= ratio <= right.endpoint_y_ratio:
                width = right.endpoint_y_ratio - left.endpoint_y_ratio
                fraction = (ratio - left.endpoint_y_ratio) / width
                return left.remaining_distance_m + fraction * (
                    right.remaining_distance_m - left.remaining_distance_m
                )
        return self.samples[-1].remaining_distance_m


def default_calibration_path() -> Path:
    return Path(__file__).resolve().with_name(CALIBRATION_FILENAME)


def load_optional_calibration(
    search_pitch_deg: float,
) -> Tuple[Optional[RouteDistanceCalibration], str]:
    path = default_calibration_path()
    if not path.exists():
        return None, "distance calibration file is missing"
    try:
        calibration = RouteDistanceCalibration.load(path)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        return None, "invalid distance calibration: %s" % error
    if abs(calibration.search_pitch_deg - float(search_pitch_deg)) > 0.5:
        return None, (
            "distance calibration pitch %.1f does not match search pitch %.1f"
            % (calibration.search_pitch_deg, float(search_pitch_deg))
        )
    return calibration, "distance calibration loaded"
