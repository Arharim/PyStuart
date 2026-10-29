"""
Tests for the servo calibration mapping.

The mapping is six identical linear channels with a sign flip; the
tests pin the arithmetic, the encoder bounds, NaN rejection and the
YAML roundtrip against the shipped rig config.
"""
import math
from pathlib import Path

import pytest

from pystuart.transport.servo_map import ServoCalibration, load_servo_calibration


def make_calibration(**overrides):
    defaults = dict(center_count=2048, center_alpha_deg=0.0, counts_per_degree=10.0,
                    invert=(False,) * 6)
    defaults.update(overrides)
    return ServoCalibration(**defaults)


class TestAlphaToCounts:
    def test_neutral_maps_to_center(self):
        assert make_calibration().alpha_to_counts([0.0] * 6) == (2048,) * 6

    def test_linear_scaling(self):
        calibration = make_calibration()
        assert calibration.alpha_to_counts([math.radians(10.0)] * 6) == (2148,) * 6
        assert calibration.alpha_to_counts([math.radians(-10.0)] * 6) == (1948,) * 6

    def test_center_alpha_shifts_reference(self):
        calibration = make_calibration(center_alpha_deg=5.0)
        assert calibration.alpha_to_counts([math.radians(5.0)] * 6) == (2048,) * 6

    def test_invert_flips_direction(self):
        calibration = make_calibration(invert=(True, False, True, False, True, False))
        counts = calibration.alpha_to_counts([math.radians(10.0)] * 6)
        assert counts == (1948, 2148, 1948, 2148, 1948, 2148)

    def test_clamps_to_encoder_bounds(self):
        calibration = make_calibration()
        assert calibration.alpha_to_counts([math.radians(500.0)] * 6) == (4095,) * 6
        assert calibration.alpha_to_counts([math.radians(-500.0)] * 6) == (0,) * 6

    def test_rejects_nan_and_wrong_length(self):
        calibration = make_calibration()
        with pytest.raises(ValueError, match="NaN"):
            calibration.alpha_to_counts([0.0] * 5 + [float("nan")])
        with pytest.raises(ValueError):
            calibration.alpha_to_counts([0.0] * 5)


class TestCountsToAlpha:
    def test_roundtrip_within_quantization(self):
        calibration = make_calibration()
        for alpha_deg in (-15.0, -3.5, 0.0, 7.25, 18.0):
            counts = calibration.alpha_to_counts([math.radians(alpha_deg)] * 6)
            alphas = calibration.counts_to_alpha(counts)
            assert math.degrees(alphas[0]) == pytest.approx(alpha_deg, abs=0.1)

    def test_invert_roundtrip(self):
        calibration = make_calibration(invert=(True,) * 6)
        counts = calibration.alpha_to_counts([math.radians(12.0)] * 6)
        alphas = calibration.counts_to_alpha(counts)
        assert math.degrees(alphas[0]) == pytest.approx(12.0, abs=0.1)


class TestValidation:
    def test_bad_center_count(self):
        with pytest.raises(ValueError):
            make_calibration(center_count=5000).validate()

    def test_bad_invert_length(self):
        with pytest.raises(ValueError):
            make_calibration(invert=(True,) * 5).validate()


class TestConfigLoad:
    def test_shipped_config_loads_to_defaults(self):
        repo_config = Path(__file__).parent.parent / "configs" / "desktop_servo.yaml"
        calibration = load_servo_calibration(repo_config)
        assert calibration.center_count == 2048
        assert calibration.center_alpha_deg == 0.0
        assert calibration.counts_per_degree == pytest.approx(4096.0 / 360.0, rel=1e-3)
        assert calibration.invert == (False,) * 6

    def test_missing_section_raises(self, tmp_path):
        bad = tmp_path / "bad.yaml"
        bad.write_text("geometry: {}\n", encoding="utf-8")
        with pytest.raises(ValueError, match="servos"):
            load_servo_calibration(bad)
