"""
Classical washout filter chain (Reid-Nahon style).

Three channels turn body-frame forces and rates into a platform pose:

  translational  high-passed specific force, scaled and integrated
                 twice, gives the platform translation in mm
  rotational     high-passed angular rates, integrated, give the
                 dynamic roll/pitch/yaw on top of the tilt
  tilt           low-passed surge/sway force tilts the platform so
                 gravity fakes sustained acceleration

Defaults are starting points for the desktop rig, not tuned values;
coefficients live in the rig config and exist to be retuned on the
hardware (design document, section 11: live config, hot reload).

Conventions: +X forward, +Y left, +Z up; specific force in m/s^2 with
gravity removed (what a game telemetry feed reports); rates in rad/s.
Sustained forward acceleration pitches the nose up, sustained left
acceleration banks left. If a direction feels inverted on the rig,
flip the tilt gain sign in config - no code change.

Filters are discretized with the plain bilinear transform; at cueing
sample rates the cutoff warping is negligible.
"""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Tuple

import numpy as np
import yaml


class Biquad:
    """Stateful direct-form-1 biquad section."""

    __slots__ = ("_b0", "_b1", "_b2", "_a1", "_a2", "_x1", "_x2", "_y1", "_y2")

    def __init__(self, b0: float, b1: float, b2: float, a1: float, a2: float):
        self._b0, self._b1, self._b2 = b0, b1, b2
        self._a1, self._a2 = a1, a2
        self._x1 = self._x2 = self._y1 = self._y2 = 0.0

    def step(self, x: float) -> float:
        y = (self._b0 * x + self._b1 * self._x1 + self._b2 * self._x2
             - self._a1 * self._y1 - self._a2 * self._y2)
        self._x2, self._x1 = self._x1, x
        self._y2, self._y1 = self._y1, y
        return y

    def reset(self) -> None:
        self._x1 = self._x2 = self._y1 = self._y2 = 0.0


def _analog_to_bilinear(num: Tuple[float, float, float], den: Tuple[float, float, float], dt: float) -> Biquad:
    """Discretize an analog biquad H(s)=num/den with the bilinear transform."""
    k = 2.0 / dt
    n0, n1, n2 = num
    d0, d1, d2 = den
    nd = [
        d0 * k * k + d1 * k + d2,
        2 * (d2 - d0 * k * k),
        d0 * k * k - d1 * k + d2,
    ]
    b = [
        (n0 * k * k + n1 * k + n2) / nd[0],
        (2 * (n2 - n0 * k * k)) / nd[0],
        (n0 * k * k - n1 * k + n2) / nd[0],
    ]
    a1, a2 = nd[1] / nd[0], nd[2] / nd[0]
    return Biquad(b[0], b[1], b[2], a1, a2)


def high_pass(dt: float, cutoff_hz: float, zeta: float = 1.0) -> Biquad:
    """Second-order high-pass H(s) = s^2 / (s^2 + 2*zeta*w*s + w^2)."""
    w = 2 * np.pi * cutoff_hz
    return _analog_to_bilinear((1.0, 0.0, 0.0), (1.0, 2 * zeta * w, w * w), dt)


def low_pass(dt: float, cutoff_hz: float, zeta: float = 1.0) -> Biquad:
    """Second-order low-pass H(s) = w^2 / (s^2 + 2*zeta*w*s + w^2)."""
    w = 2 * np.pi * cutoff_hz
    return _analog_to_bilinear((0.0, 0.0, w * w), (1.0, 2 * zeta * w, w * w), dt)


@dataclass(frozen=True)
class WashoutConfig:
    dt: float
    accel_gain: Tuple[float, float, float] = (0.4, 0.4, 0.5)
    trans_hp_hz: float = 0.3
    trans_zeta: float = 1.0
    rot_hp_hz: float = 1.0
    rot_zeta: float = 1.0
    tilt_lp_hz: float = 0.15
    tilt_zeta: float = 1.0
    tilt_rate_limit_deg_s: float = 8.0
    tilt_roll_gain: float = 1.0
    tilt_pitch_gain: float = 1.0
    max_translation_mm: Tuple[float, float, float] = (60.0, 60.0, 50.0)
    max_rotation_deg: Tuple[float, float, float] = (25.0, 25.0, 40.0)
    gravity: float = 9.81

    def validate(self) -> None:
        if self.dt <= 0:
            raise ValueError("dt must be positive")
        if min(self.max_translation_mm) < 0 or min(self.max_rotation_deg) < 0:
            raise ValueError("limits must be non-negative")
        if min(self.trans_hp_hz, self.rot_hp_hz, self.tilt_lp_hz) <= 0:
            raise ValueError("cutoff frequencies must be positive")
        if self.tilt_rate_limit_deg_s <= 0:
            raise ValueError("tilt_rate_limit_deg_s must be positive")


@dataclass(frozen=True)
class CueingPose:
    translation_mm: np.ndarray
    rotation_rad: np.ndarray


def load_washout(path) -> WashoutConfig:
    """Build a WashoutConfig from the 'washout' mapping of a rig config."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or "washout" not in raw:
        raise ValueError("config file must contain a top-level 'washout' mapping")
    section = raw["washout"]
    try:
        config = WashoutConfig(
            dt=float(section["dt_s"]),
            accel_gain=tuple(float(v) for v in section["accel_gain"]),
            trans_hp_hz=float(section["trans_hp_hz"]),
            trans_zeta=float(section["trans_zeta"]),
            rot_hp_hz=float(section["rot_hp_hz"]),
            rot_zeta=float(section["rot_zeta"]),
            tilt_lp_hz=float(section["tilt_lp_hz"]),
            tilt_zeta=float(section["tilt_zeta"]),
            tilt_rate_limit_deg_s=float(section["tilt_rate_limit_deg_s"]),
            tilt_roll_gain=float(section["tilt_roll_gain"]),
            tilt_pitch_gain=float(section["tilt_pitch_gain"]),
            max_translation_mm=tuple(float(v) for v in section["max_translation_mm"]),
            max_rotation_deg=tuple(float(v) for v in section["max_rotation_deg"]),
        )
    except KeyError as e:
        raise ValueError("washout mapping is missing key: %s" % e) from e
    config.validate()
    return config


class WashoutFilter:
    """Stateful classical washout; one instance per platform, step per frame."""

    def __init__(self, config: WashoutConfig):
        config.validate()
        self.config = config
        dt = config.dt
        self._force_hp = [high_pass(dt, config.trans_hp_hz, config.trans_zeta) for _ in range(3)]
        self._rate_hp = [high_pass(dt, config.rot_hp_hz, config.rot_zeta) for _ in range(3)]
        self._tilt_lp = [low_pass(dt, config.tilt_lp_hz, config.tilt_zeta) for _ in range(2)]
        self._gain = np.asarray(config.accel_gain, dtype=float)
        self._max_trans = np.asarray(config.max_translation_mm, dtype=float)
        self._max_rot = np.radians(np.asarray(config.max_rotation_deg, dtype=float))
        self._vel = np.zeros(3)
        self._pos = np.zeros(3)
        self._rot_dynamic = np.zeros(3)
        self._tilt = np.zeros(2)

    def reset(self) -> None:
        for filters in (self._force_hp, self._rate_hp, self._tilt_lp):
            for f in filters:
                f.reset()
        self._vel[:] = 0.0
        self._pos[:] = 0.0
        self._rot_dynamic[:] = 0.0
        self._tilt[:] = 0.0

    def step(self, force, rate) -> CueingPose:
        cfg = self.config
        f_hp = np.array([self._force_hp[i].step(force[i]) for i in range(3)])
        w_hp = np.array([self._rate_hp[i].step(rate[i]) for i in range(3)])
        surge_lp = self._tilt_lp[0].step(force[0])
        sway_lp = self._tilt_lp[1].step(force[1])

        self._vel += self._gain * f_hp * cfg.dt
        self._pos += self._vel * cfg.dt
        self._rot_dynamic += w_hp * cfg.dt

        max_tilt_step = np.radians(cfg.tilt_rate_limit_deg_s) * cfg.dt
        target_roll = cfg.tilt_roll_gain * np.arctan2(sway_lp, cfg.gravity)
        target_pitch = cfg.tilt_pitch_gain * (-np.arctan2(surge_lp, cfg.gravity))
        self._tilt[0] += np.clip(target_roll - self._tilt[0], -max_tilt_step, max_tilt_step)
        self._tilt[1] += np.clip(target_pitch - self._tilt[1], -max_tilt_step, max_tilt_step)

        rotation = np.array([
            self._tilt[0] + self._rot_dynamic[0],
            self._tilt[1] + self._rot_dynamic[1],
            self._rot_dynamic[2],
        ])
        translation_mm = np.clip(self._pos * 1000.0, -self._max_trans, self._max_trans)
        rotation = np.clip(rotation, -self._max_rot, self._max_rot)
        return CueingPose(translation_mm, rotation)
