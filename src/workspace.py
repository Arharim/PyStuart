import numpy as np
from typing import List, Tuple, Optional
from .stewart_platform import StewartPlatform


class WorkspaceAnalyzer:
    def __init__(self):
        self._platform = StewartPlatform()
        self.bounds: Optional[dict] = None
        self.boundary_points: List[Tuple[float, float, float]] = []
        self.slice_points: Optional[dict] = None

    def compute_bounds(self, rot_deg=(0, 0, 0),
                       t_range=(-40, 40), t_step=2.0,
                       r_range=(-35, 35), r_step=2.0) -> dict:
        rot = np.radians(rot_deg)
        bounds = {
            'pos_x': [float('inf'), float('-inf')],
            'pos_y': [float('inf'), float('-inf')],
            'pos_z': [float('inf'), float('-inf')],
            'rot_x': [float('inf'), float('-inf')],
            'rot_y': [float('inf'), float('-inf')],
            'rot_z': [float('inf'), float('-inf')],
        }

        t_vals = np.arange(t_range[0], t_range[1] + t_step * 0.5, t_step)
        r_vals = np.arange(r_range[0], r_range[1] + r_step * 0.5, r_step)

        for tx in t_vals:
            for ty in t_vals:
                for tz in t_vals:
                    self._platform.update_pose([tx, ty, tz], rot)
                    if self._platform.is_valid_pose():
                        for key, val in [('pos_x', tx), ('pos_y', ty), ('pos_z', tz)]:
                            bounds[key][0] = min(bounds[key][0], val)
                            bounds[key][1] = max(bounds[key][1], val)

        for rx in r_vals:
            for ry in r_vals:
                for rz in r_vals:
                    self._platform.update_pose([0, 0, 0], np.radians([rx, ry, rz]))
                    if self._platform.is_valid_pose():
                        for key, val in [('rot_x', rx), ('rot_y', ry), ('rot_z', rz)]:
                            bounds[key][0] = min(bounds[key][0], val)
                            bounds[key][1] = max(bounds[key][1], val)

        for key in bounds:
            if bounds[key][0] == float('inf'):
                bounds[key] = [0, 0]

        self.bounds = bounds
        self._compute_boundary_slice(rot)
        return bounds

    def _compute_boundary_slice(self, rotation=(0, 0, 0)):
        self.boundary_points = []
        self.slice_points = {}

        if self.bounds is None:
            return

        x_min, x_max = self.bounds['pos_x']
        y_min, y_max = self.bounds['pos_y']
        z_min, z_max = self.bounds['pos_z']

        z_mid = (z_min + z_max) / 2
        step = 2.0
        self.slice_points['z_level'] = z_mid
        self.slice_points['valid'] = []
        self.slice_points['invalid'] = []

        for tx in np.arange(x_min - 2, x_max + 2 + step * 0.5, step):
            for ty in np.arange(y_min - 2, y_max + 2 + step * 0.5, step):
                self._platform.update_pose([tx, ty, z_mid], rotation)
                pt = (tx, ty, z_mid + self._platform.INITIAL_HEIGHT)
                if self._platform.is_valid_pose():
                    self.slice_points['valid'].append(pt)
                else:
                    self.slice_points['invalid'].append(pt)

        for tx in np.arange(x_min, x_max + step * 0.5, step):
            for ty in np.arange(y_min, y_max + step * 0.5, step):
                for tz in (z_min, z_max):
                    self._platform.update_pose([tx, ty, tz], rotation)
                    if self._platform.is_valid_pose():
                        self.boundary_points.append((tx, ty, tz + self._platform.INITIAL_HEIGHT))

        self._platform.update_pose([0, 0, 0], [0, 0, 0])

    def is_computed(self) -> bool:
        return self.bounds is not None
