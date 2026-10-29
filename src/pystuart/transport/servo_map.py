"""
Servo calibration: IK horn angles -> STS3215 encoder counts.

The 12-bit magnetic encoder reports 4096 counts over a full shaft
turn (0.0879 deg/count), and the mapping is linear per rig config:

    count = center_count + dir * counts_per_degree * (alpha_deg - center_alpha_deg)

dir flips per axis for mounting direction. Neutral IK angles map to
center_count; the desktop rig rests at alpha = 0 (the rest height was
chosen for exactly that), so the shipped defaults are honest starting
points to be refined by measuring the real horn on the assembled rig.
"""
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence, Tuple

import yaml

COUNTS_PER_TURN = 4096.0
COUNT_MIN = 0
COUNT_MAX = 4095


@dataclass(frozen=True)
class ServoCalibration:
    center_count: int = 2048
    center_alpha_deg: float = 0.0
    counts_per_degree: float = COUNTS_PER_TURN / 360.0
    invert: Tuple[bool, ...] = (False,) * 6

    def validate(self) -> None:
        if len(self.invert) != 6:
            raise ValueError("invert must hold exactly 6 flags")
        if not COUNT_MIN <= self.center_count <= COUNT_MAX:
            raise ValueError("center_count must be within 0..4095")
        if self.counts_per_degree <= 0:
            raise ValueError("counts_per_degree must be positive")

    def alpha_to_counts(self, alpha_rad: Sequence[float]) -> Tuple[int, ...]:
        if len(alpha_rad) != 6:
            raise ValueError("six servo targets are required")
        if any(math.isnan(a) for a in alpha_rad):
            raise ValueError("unreachable pose: alpha contains NaN")
        self.validate()
        counts = []
        for alpha, flip in zip(alpha_rad, self.invert):
            alpha_deg = math.degrees(alpha) - self.center_alpha_deg
            direction = -1.0 if flip else 1.0
            count = self.center_count + direction * self.counts_per_degree * alpha_deg
            counts.append(int(round(count)))
        return tuple(min(COUNT_MAX, max(COUNT_MIN, c)) for c in counts)

    def counts_to_alpha(self, counts: Sequence[int]) -> Tuple[float, ...]:
        """Inverse mapping for SRV_FB feedback display; returns radians."""
        if len(counts) != 6:
            raise ValueError("six servo feedback values are required")
        self.validate()
        alphas = []
        for count, flip in zip(counts, self.invert):
            direction = -1.0 if flip else 1.0
            alpha_deg = direction * (count - self.center_count) / self.counts_per_degree
            alphas.append(math.radians(alpha_deg + self.center_alpha_deg))
        return tuple(alphas)


def load_servo_calibration(path) -> ServoCalibration:
    """Build a ServoCalibration from the 'servos' mapping of a rig config."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or "servos" not in raw:
        raise ValueError("config file must contain a top-level 'servos' mapping")
    section = raw["servos"]
    try:
        calibration = ServoCalibration(
            center_count=int(section["center_count"]),
            center_alpha_deg=float(section["center_alpha_deg"]),
            counts_per_degree=float(section["counts_per_degree"]),
            invert=tuple(bool(v) for v in section["invert"]),
        )
    except KeyError as e:
        raise ValueError("servos mapping is missing key: %s" % e) from e
    calibration.validate()
    return calibration
