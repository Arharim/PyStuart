"""
Tests for the headless runner.

Sources are tiny stand-ins feeding the real washout chain, sinks
collect poses: the runner logic (iteration, pacing, miss handling,
stride) is what's under test here, the DSP is covered in its own file.
"""
import math

import numpy as np
import pytest

from pystuart.apps.headless import main, run_ac, run_csv
from pystuart.cueing.telemetry import TelemetrySample
from pystuart.cueing.washout import WashoutConfig, WashoutFilter
from pystuart.core.config import Geometry
from pystuart.core.platform import StewartPlatform
from pystuart.transport.actuator import ActuatorWriter
from pystuart.transport.protocol import TYPE_CMD_ACT, decode_cmd_act, decode_frame
from pystuart.transport.serial_link import SerialLink
from pystuart.transport.servo_map import ServoCalibration


def make_washout():
    return WashoutFilter(WashoutConfig(dt=0.005))


class TestRunCsv:
    def test_replays_every_sample(self):
        samples = [TelemetrySample(t=i * 0.1, force=(0.0, 0.0, 0.0), rate=(0.0, 0.0, 0.0))
                   for i in range(10)]

        class Source:
            def __iter__(self):
                return iter(samples)

        poses = []
        stats = run_csv(Source(), make_washout(), lambda p, s: poses.append(p), pacing=False)
        assert stats["samples"] == 10
        assert len(poses) == 10
        for pose in poses:
            assert np.allclose(pose.translation_mm, 0.0, atol=1e-12)

    def test_stride_thins_the_sink(self):
        samples = [TelemetrySample(t=i * 0.1, force=(0.0, 0.0, 0.0), rate=(0.0, 0.0, 0.0))
                   for i in range(10)]

        class Source:
            def __iter__(self):
                return iter(samples)

        printed = []
        stats = run_csv(Source(), make_washout(), lambda p, s: printed.append(p),
                        pacing=False, stride=3)
        assert stats["samples"] == 10
        assert len(printed) == 4  # n = 0, 3, 6, 9

    def test_pulse_integrates_through_the_chain(self):
        """A half-second surge pulse must surface as washed-out translation."""
        class Source:
            def __init__(self):
                self.samples = [
                    TelemetrySample(t=i * 0.005,
                                    force=(3.0, 0.0, 0.0) if i < 100 else (0.0, 0.0, 0.0),
                                    rate=(0.0, 0.0, 0.0))
                    for i in range(2000)
                ]

            def __iter__(self):
                return iter(self.samples)

        poses = []
        run_csv(Source(), make_washout(), lambda p, s: poses.append(p), pacing=False)
        tx = np.array([p.translation_mm[0] for p in poses])
        peak = np.abs(tx).max()
        assert peak > 10.0
        assert abs(tx[-1]) < 0.15 * peak


class TestRunAc:
    def test_stops_after_max_misses(self):
        samples = [TelemetrySample(t=i, force=(1.0, 0.0, 0.0), rate=(0.0, 0.0, 0.0))
                   for i in range(3)]

        class Source:
            def __init__(self):
                self.it = iter(samples)

            def next_sample(self):
                try:
                    return next(self.it)
                except StopIteration:
                    return None

        poses = []
        stats = run_ac(Source(), make_washout(), lambda p, s: poses.append(p), max_misses=4)
        assert stats == {"samples": 3, "servos_sent": 0, "invalid_poses": 0,
                         "timed_out": True}
        assert len(poses) == 3

    def test_miss_counter_resets_on_data(self):
        class Source:
            def __init__(self):
                self.calls = 0

            def next_sample(self):
                self.calls += 1
                if self.calls > 20 or self.calls % 3 == 0:
                    return None
                return TelemetrySample(t=self.calls, force=(0.0, 0.0, 0.0), rate=(0.0, 0.0, 0.0))

        poses = []
        stats = run_ac(Source(), make_washout(), lambda p, s: poses.append(p), max_misses=5)
        assert stats["timed_out"] is True
        assert stats["samples"] == 14  # every call that is not a multiple of 3, up to 20


class TestServoOutput:
    def _zeros_source(self, count):
        samples = [TelemetrySample(t=i * 0.1, force=(0.0, 0.0, 0.0), rate=(0.0, 0.0, 0.0))
                   for i in range(count)]

        class Source:
            def __iter__(self):
                return iter(samples)

        return Source()

    def test_rest_pose_sends_center_counts(self):
        with ActuatorWriter(SerialLink("loop://", baudrate=921600),
                            ServoCalibration()) as actuator:
            ik = StewartPlatform(Geometry.desktop())
            stats = run_csv(self._zeros_source(5), make_washout(), lambda p, s: None,
                            pacing=False, actuator=actuator, ik_model=ik)
            actuator._link._serial.timeout = 0.2
            wire = actuator._link._serial.read(4096)
        assert stats["servos_sent"] == 5
        assert stats["invalid_poses"] == 0
        assert len(wire) == 5 * 24  # five CMD_ACT frames, 24 bytes each
        for i in range(5):
            frame = decode_frame(wire[i * 24:(i + 1) * 24])
            assert frame.type == TYPE_CMD_ACT
            counts, flags = decode_cmd_act(frame.payload)
            assert counts == (2048,) * 6

    def test_unreachable_poses_are_skipped(self):
        class BrokenIK:
            def update_pose(self, translation, rotation):
                pass

            def is_valid_pose(self):
                return False

            def get_alpha_angles(self):
                raise AssertionError("never reached on invalid pose")

        with ActuatorWriter(SerialLink("loop://", baudrate=921600),
                            ServoCalibration()) as actuator:
            stats = run_csv(self._zeros_source(3), make_washout(), lambda p, s: None,
                            pacing=False, actuator=actuator, ik_model=BrokenIK())
            actuator._link._serial.timeout = 0.05
            wire = actuator._link._serial.read(4096)
        assert stats == {"samples": 3, "servos_sent": 0, "invalid_poses": 3}
        assert wire == b""


class TestCli:
    def test_missing_source_is_a_usage_error(self, capsys):
        with pytest.raises(SystemExit) as e:
            main([])
        assert e.value.code == 2

    def test_unknown_source_is_a_usage_error(self, capsys):
        with pytest.raises(SystemExit) as e:
            main(["banana"])
        assert e.value.code == 2

    def test_csv_run_smoke(self, tmp_path, capsys):
        path = tmp_path / "rec.csv"
        lines = ["t_s,fx_mps2,fy_mps2,fz_mps2,p_radps,q_radps,r_radps"]
        lines += ["%.3f,0.0,0.0,0.0,0.0,0.0,0.0" % (i * 0.01) for i in range(5)]
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

        code = main(["csv", str(path), "--no-pacing"])
        assert code == 0
        err = capsys.readouterr().err
        assert "processed 5 samples" in err

    def test_csv_with_serial_smoke(self, tmp_path, capsys):
        path = tmp_path / "rec.csv"
        lines = ["t_s,fx_mps2,fy_mps2,fz_mps2,p_radps,q_radps,r_radps"]
        lines += ["%.3f,0.0,0.0,0.0,0.0,0.0,0.0" % (i * 0.01) for i in range(5)]
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

        code = main(["csv", str(path), "--no-pacing", "--serial", "loop://",
                     "--rate", "1000000"])
        assert code == 0
        err = capsys.readouterr().err
        assert "5 servo frames" in err
