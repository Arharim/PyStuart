"""
Property tests for the washout chain.

These check DSP physics, not implementation echoes: DC rejection of the
high-pass channels, tilt convergence to the gravity-equivalent angle,
rate limiting, stability at rest, amplitude response at the filter
corner and hard limits.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from pystuart.cueing.washout import (
    WashoutConfig, WashoutFilter, high_pass, load_washout, low_pass,
)

DT = 0.005
REPO_CONFIG = None


def make_filter(**overrides) -> WashoutFilter:
    config = WashoutConfig(dt=DT, **overrides)
    return WashoutFilter(config)


def run(filter, seconds, force=(0.0, 0.0, 0.0), rate=(0.0, 0.0, 0.0)):
    poses = [
        filter.step(force, rate)
        for _ in range(int(round(seconds / DT)))
    ]
    return poses


class TestBiquad:
    def test_corner_amplitude_near_half_power(self):
        # |H(j*w_n)| = 1/(2*zeta) exactly; for zeta=1 the -3 dB point sits
        # higher, at ~0.64*w_n, so 0.5 is the honest expectation here.
        for factory in (high_pass, low_pass):
            biquad = factory(DT, 1.0, zeta=1.0)
            t = np.arange(0, 8, DT)
            signal = np.sin(2 * np.pi * 1.0 * t)
            out = np.array([biquad.step(x) for x in signal])
            ratio = np.ptp(out[-int(2 / DT):]) / np.ptp(signal[-int(2 / DT):])
            assert ratio == pytest.approx(0.5, abs=0.05), \
                "%s gain at corner: %s" % (factory, ratio)

    def test_high_pass_passes_fast_blocks_slow(self):
        fast = high_pass(DT, 1.0)
        t = np.arange(0, 6, DT)
        quick = np.array([fast.step(x) for x in np.sin(2 * np.pi * 10.0 * t)])
        assert np.ptp(quick[-int(1 / DT):]) > 1.7

        slow = high_pass(DT, 1.0)
        crawl = np.array([slow.step(x) for x in np.sin(2 * np.pi * 0.1 * t)])
        assert np.ptp(crawl[-int(1 / DT):]) < 0.5

    def test_low_pass_dc_gain_is_unity(self):
        biquad = low_pass(DT, 1.0)
        out = [biquad.step(1.0) for _ in range(20000)]
        assert out[-1] == pytest.approx(1.0, abs=1e-6)


class TestTranslationalChannel:
    def test_pulse_washes_out(self):
        washout = make_filter()
        poses = run(washout, 1.0, force=(2.0, 0.0, 0.0)) + run(washout, 8.0)
        tx = [p.translation_mm[0] for p in poses]
        peak = max(abs(v) for v in tx)
        assert peak > 10.0, "pulse should move the platform, peak=%s" % peak
        assert abs(tx[-1]) < 0.15 * peak, "high-pass must wash the offset out"

    def test_surge_stays_in_x(self):
        washout = make_filter()
        poses = run(washout, 1.0, force=(3.0, 0.0, 0.0))
        assert max(abs(p.translation_mm[1]) for p in poses) < 1e-6
        assert max(abs(p.translation_mm[2]) for p in poses) < 1e-6


class TestTiltChannel:
    def test_sustained_accel_converges_to_gravity_angle(self):
        washout = make_filter()
        poses = run(washout, 30.0, force=(3.0, 0.0, 0.0))
        target = -math.atan2(3.0, WashoutConfig(dt=DT).gravity)
        assert poses[-1].rotation_rad[1] == pytest.approx(target, rel=0.05)

    def test_tilt_rate_limited(self):
        washout = make_filter()
        limit_step = math.radians(WashoutConfig(dt=DT).tilt_rate_limit_deg_s) * DT
        prev = 0.0
        for _ in range(int(0.5 / DT)):
            pose = washout.step((50.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            delta = abs(pose.rotation_rad[1] - prev)
            assert delta <= limit_step * 1.5
            prev = pose.rotation_rad[1]

    def test_lateral_accel_banks_not_pitches(self):
        washout = make_filter()
        poses = run(washout, 30.0, force=(0.0, 3.0, 0.0))
        target = math.atan2(3.0, WashoutConfig(dt=DT).gravity)
        assert poses[-1].rotation_rad[0] == pytest.approx(target, rel=0.05)
        assert abs(poses[-1].rotation_rad[1]) < 0.02


class TestStability:
    def test_rest_stays_at_zero(self):
        washout = make_filter()
        poses = run(washout, 2.0)
        for pose in poses:
            assert np.allclose(pose.translation_mm, 0.0, atol=1e-12)
            assert np.allclose(pose.rotation_rad, 0.0, atol=1e-12)

    def test_reset_clears_state(self):
        washout = make_filter()
        run(washout, 2.0, force=(5.0, 5.0, 5.0), rate=(1.0, 1.0, 1.0))
        washout.reset()
        poses = run(washout, 0.5)
        for pose in poses:
            assert np.allclose(pose.translation_mm, 0.0, atol=1e-12)

    def test_outputs_respect_limits(self):
        washout = make_filter(max_translation_mm=(20.0, 20.0, 20.0),
                              max_rotation_deg=(5.0, 5.0, 5.0))
        poses = run(washout, 30.0, force=(30.0, 30.0, 30.0), rate=(3.0, 3.0, 3.0))
        for pose in poses:
            assert np.all(np.abs(pose.translation_mm) <= 20.0 + 1e-9)
            assert np.all(np.abs(pose.rotation_rad) <= math.radians(5.0) + 1e-9)


class TestConfig:
    def test_shipped_config_loads(self):
        repo_config = Path(__file__).parent.parent / "configs" / "desktop_servo.yaml"
        config = load_washout(repo_config)
        assert config.dt == 0.005
        assert config == WashoutConfig(dt=0.005)

    def test_rejects_bad_values(self):
        with pytest.raises(ValueError):
            WashoutConfig(dt=-1.0).validate()
        with pytest.raises(ValueError):
            WashoutConfig(dt=0.005, trans_hp_hz=0.0).validate()
        with pytest.raises(ValueError):
            WashoutConfig(dt=0.005, tilt_rate_limit_deg_s=0.0).validate()

    def test_rejects_missing_section(self, tmp_path):
        bad = tmp_path / "bad.yaml"
        bad.write_text("geometry: {}\n", encoding="utf-8")
        with pytest.raises(ValueError):
            load_washout(bad)
