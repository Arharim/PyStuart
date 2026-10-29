"""
Platform geometry as data, loaded from YAML config.

The rig moves from the desktop prototype to full size by swapping a
config file, not by editing code (design document, section 11). The
desktop numbers below are the historical v1 constants, kept as the
default so a missing config behaves exactly like v1.
"""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import yaml


@dataclass(frozen=True)
class Geometry:
    """Rigid geometry of the hexapod, all angles in radians."""

    base_radius: float
    platform_radius: float
    horn_length: float
    rod_length: float
    base_angles: np.ndarray
    platform_angles: np.ndarray
    beta: np.ndarray
    initial_height: float

    def __eq__(self, other):
        if not isinstance(other, Geometry):
            return NotImplemented
        return (
            self.base_radius == other.base_radius
            and self.platform_radius == other.platform_radius
            and self.horn_length == other.horn_length
            and self.rod_length == other.rod_length
            and self.initial_height == other.initial_height
            and np.array_equal(self.base_angles, other.base_angles)
            and np.array_equal(self.platform_angles, other.platform_angles)
            and np.array_equal(self.beta, other.beta)
        )

    @staticmethod
    def desktop() -> "Geometry":
        """The v1 desktop servo rig: 76/60/40/130 mm, 120.28 mm rest height."""
        return Geometry(
            base_radius=76.0,
            platform_radius=60.0,
            horn_length=40.0,
            rod_length=130.0,
            base_angles=np.radians([-50, -70, -170, -190, -290, -310]),
            platform_angles=np.radians([-54, -66, -174, -186, -294, -306]),
            beta=np.radians([30, -150, -90, 90, 150, -30]),
            initial_height=120.28183632,
        )

    def validate(self) -> None:
        if len(self.base_angles) != 6 or len(self.platform_angles) != 6 or len(self.beta) != 6:
            raise ValueError("geometry needs exactly 6 base angles, platform angles and beta values")
        if self.rod_length <= self.horn_length:
            raise ValueError("rod_length must exceed horn_length")
        if min(self.base_radius, self.platform_radius, self.horn_length, self.rod_length) <= 0:
            raise ValueError("geometry dimensions must be positive")
        if self.initial_height <= 0:
            raise ValueError("initial_height must be positive")


def load_geometry(path) -> Geometry:
    """Build a Geometry from a YAML file with a 'geometry' mapping."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or "geometry" not in raw:
        raise ValueError("config file must contain a top-level 'geometry' mapping")
    g = raw["geometry"]
    try:
        geometry = Geometry(
            base_radius=float(g["base_radius_mm"]),
            platform_radius=float(g["platform_radius_mm"]),
            horn_length=float(g["horn_length_mm"]),
            rod_length=float(g["rod_length_mm"]),
            base_angles=np.radians(np.asarray(g["base_angles_deg"], dtype=float)),
            platform_angles=np.radians(np.asarray(g["platform_angles_deg"], dtype=float)),
            beta=np.radians(np.asarray(g["beta_deg"], dtype=float)),
            initial_height=float(g["initial_height_mm"]),
        )
    except KeyError as e:
        raise ValueError("geometry mapping is missing key: %s" % e) from e
    geometry.validate()
    return geometry
