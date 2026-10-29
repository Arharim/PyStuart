"""
Smoke tests for the pystuart.core package.

Fast, headless checks of the invariants that keep the simulator honest:
IK reachability, rod-length conservation, config-driven geometry,
trajectory interpolation and preset persistence. Golden-vector IK tests
land with the v2 cueing work.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from pystuart.core.config import Geometry, load_geometry
from pystuart.core.math_utils import clamp, rotation_matrix, safe_arcsin
from pystuart.core.platform import StewartPlatform
from pystuart.core.pose_presets import PosePresets
from pystuart.core.trajectory import Trajectory
from pystuart.core.workspace import WorkspaceAnalyzer


class TestMathUtils:
    def test_rotation_matrix_identity(self):
        assert np.allclose(rotation_matrix([0.0, 0.0, 0.0]), np.eye(3))

    def test_rotation_matrix_orthonormal(self):
        rng = np.random.default_rng(42)
        for _ in range(8):
            r = rotation_matrix(rng.uniform(-np.pi, np.pi, 3))
            assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)

    def test_clamp(self):
        assert clamp(5, 0, 10) == 5
        assert clamp(-1, 0, 10) == 0
        assert clamp(11, 0, 10) == 10

    def test_safe_arcsin(self):
        assert safe_arcsin(0.0) == 0.0
        assert math.isnan(safe_arcsin(1.5))
        assert math.isnan(safe_arcsin(-1.5))


class TestPlatformIK:
    def test_neutral_pose_valid(self):
        platform = StewartPlatform()
        assert platform.is_valid_pose()
        assert platform.get_alpha_angles().shape == (6,)
        assert not np.any(np.isnan(platform.get_alpha_angles()))

    def test_rod_length_conserved(self):
        """Horn tip to platform attach point always equals ROD_LENGTH."""
        platform = StewartPlatform()
        checked = 0
        for pose in (
            ([10.0, -5.0, 5.0], [0.087, -0.052, 0.14]),
            ([-20.0, 15.0, -10.0], [-0.1, 0.2, -0.3]),
            ([0.0, 0.0, 0.0], [0.0, 0.0, 0.5]),
        ):
            platform.update_pose(*pose)
            if not platform.is_valid_pose():
                continue
            checked += 1
            rods = np.linalg.norm(platform.q - platform.a, axis=1)
            assert np.allclose(rods, platform.geometry.rod_length, atol=1e-6)
        assert checked >= 2, "validity filter skipped every pose - nothing was tested"

    def test_geometry_tables(self):
        geometry = Geometry.desktop()
        assert len(geometry.base_angles) == 6
        assert len(geometry.platform_angles) == 6
        assert len(geometry.beta) == 6
        assert geometry.rod_length > geometry.horn_length > 0


class TestTrajectory:
    def _write_csv(self, path, rows):
        lines = ["pos_x,pos_y,pos_z,rot_x,rot_y,rot_z"]
        lines += [",".join(str(v) for v in row) for row in rows]
        path.write_text("\n".join(lines), encoding="utf-8")

    def test_load_and_interpolate(self, tmp_path):
        csv_file = tmp_path / "traj.csv"
        self._write_csv(csv_file, [(0, 0, 0, 0, 0, 0), (10, 0, 5, 0, 10, 0), (20, 0, 0, 0, 0, 0)])
        traj = Trajectory()
        count, err = traj.load_csv(str(csv_file))
        assert err is None
        assert count == 3
        assert traj.total_frames() == 2 * 50 + 1

    def test_rejects_single_waypoint(self, tmp_path):
        csv_file = tmp_path / "traj.csv"
        self._write_csv(csv_file, [(0, 0, 0, 0, 0, 0)])
        traj = Trajectory()
        count, err = traj.load_csv(str(csv_file))
        assert count == 0
        assert err is not None

    def test_frame_bounds(self, tmp_path):
        csv_file = tmp_path / "traj.csv"
        self._write_csv(csv_file, [(0, 0, 0, 0, 0, 0), (10, 0, 0, 0, 0, 0)])
        traj = Trajectory()
        traj.load_csv(str(csv_file))
        assert traj.get_frame(-1) is None
        assert traj.get_frame(traj.total_frames()) is None
        assert traj.get_frame(0) is not None


class TestPosePresets:
    def test_save_load_roundtrip(self, tmp_path):
        presets = PosePresets(config_path=tmp_path / "presets.json")
        pose = {
            "pos_x": 1.5, "pos_y": -2.0, "pos_z": 3.0,
            "rot_x": math.radians(10), "rot_y": 0.0, "rot_z": math.radians(-5),
        }
        assert presets.save_preset("cruise", pose)
        assert presets.preset_exists("cruise")

        loaded = presets.load_preset("cruise")
        assert loaded["pos_x"] == pytest.approx(1.5)
        assert loaded["rot_x"] == pytest.approx(math.radians(10))

    def test_rejects_blank_name(self, tmp_path):
        presets = PosePresets(config_path=tmp_path / "presets.json")
        assert not presets.save_preset("   ", {"pos_x": 0})

    def test_delete(self, tmp_path):
        presets = PosePresets(config_path=tmp_path / "presets.json")
        presets.save_preset("x", {"pos_x": 0})
        assert presets.delete_preset("x")
        assert not presets.preset_exists("x")
        assert not presets.delete_preset("x")


class TestWorkspace:
    def test_quick_bounds(self):
        analyzer = WorkspaceAnalyzer()
        bounds = analyzer.compute_bounds(
            t_range=(-10, 10), t_step=10.0, r_range=(-10, 10), r_step=10.0
        )
        assert analyzer.is_computed()
        for axis in ("pos_x", "pos_y", "pos_z"):
            lo, hi = bounds[axis]
            assert lo <= hi
        # symmetric rig: translation reach must be symmetric
        assert bounds["pos_x"][0] == pytest.approx(-bounds["pos_x"][1])


class TestGeometryConfig:
    def test_shipped_config_matches_desktop_defaults(self, tmp_path):
        repo_config = Path(__file__).parent.parent / "configs" / "desktop_servo.yaml"
        assert load_geometry(repo_config) == Geometry.desktop()

    def test_custom_geometry_roundtrip(self, tmp_path):
        config = tmp_path / "custom.yaml"
        config.write_text(
            "geometry:\n"
            "  base_radius_mm: 100.0\n"
            "  platform_radius_mm: 80.0\n"
            "  horn_length_mm: 50.0\n"
            "  rod_length_mm: 160.0\n"
            "  base_angles_deg: [-50, -70, -170, -190, -290, -310]\n"
            "  platform_angles_deg: [-54, -66, -174, -186, -294, -306]\n"
            "  beta_deg: [30, -150, -90, 90, 150, -30]\n"
            "  initial_height_mm: 150.0\n",
            encoding="utf-8",
        )
        geometry = load_geometry(config)
        assert geometry.base_radius == 100.0
        assert geometry.rod_length == 160.0

        platform = StewartPlatform(geometry)
        assert np.allclose(platform.b[0], [100.0 * np.cos(np.radians(-50)),
                                           100.0 * np.sin(np.radians(-50)), 0.0])
        assert platform.is_valid_pose()

    def test_rejects_broken_geometry(self):
        broken = Geometry(
            base_radius=76.0, platform_radius=60.0, horn_length=140.0,
            rod_length=130.0, base_angles=np.zeros(6),
            platform_angles=np.zeros(6), beta=np.zeros(6),
            initial_height=120.0,
        )
        with pytest.raises(ValueError):
            broken.validate()

    def test_rejects_missing_key(self, tmp_path):
        bad = tmp_path / "bad.yaml"
        bad.write_text("geometry:\n  base_radius_mm: 76.0\n", encoding="utf-8")
        with pytest.raises(ValueError):
            load_geometry(bad)

    def test_rejects_missing_section(self, tmp_path):
        bad = tmp_path / "bad.yaml"
        bad.write_text("something: else\n", encoding="utf-8")
        with pytest.raises(ValueError):
            load_geometry(bad)
