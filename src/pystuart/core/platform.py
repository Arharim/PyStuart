"""
Stewart Platform model: geometry and inverse kinematics.

This module provides the StewartPlatform class for simulating
a 6-DOF parallel manipulator (Stewart platform / hexapod).
Pure numpy: no rendering or GUI dependencies live here.
"""
import numpy as np

from .config import Geometry
from .math_utils import rotation_matrix, safe_arcsin


class StewartPlatform:
    """
    Stewart Platform (hexapod) model.

    A 6-DOF parallel manipulator with 6 linear actuators connecting
    a base plate to a movable platform.

    Attributes:
        geometry: rigid dimensions and angles of the rig (see core.config).
    """

    def __init__(self, geometry: Geometry = None):
        """Initialize the platform; defaults to the desktop rig geometry."""
        self.geometry = geometry if geometry is not None else Geometry.desktop()
        self.geometry.validate()
        self.translation = np.array([0.0, 0.0, 0.0])
        self.rotation = np.array([0.0, 0.0, 0.0])
        self.initial_height = np.array([0.0, 0.0, self.geometry.initial_height])
        self.valid_pose = True

        self.b = np.zeros((6, 3))
        self.p = np.zeros((6, 3))
        self.q = np.zeros((6, 3))
        self.l = np.zeros((6, 3))
        self.a = np.zeros((6, 3))
        self.alpha = np.zeros(6)

        self._init_attachment_points()
        self._calculate_vectors()

    def _init_attachment_points(self):
        """Initialize base and platform attachment point positions."""
        g = self.geometry
        for i in range(6):
            self.b[i] = [
                g.base_radius * np.cos(g.base_angles[i]),
                g.base_radius * np.sin(g.base_angles[i]),
                0.0
            ]
            self.p[i] = [
                g.platform_radius * np.cos(g.platform_angles[i]),
                g.platform_radius * np.sin(g.platform_angles[i]),
                0.0
            ]
    
    def _calculate_vectors(self):
        """Calculate all leg vectors and servo angles."""
        R_matrix = rotation_matrix(self.rotation)
        
        for i in range(6):
            self.q[i] = R_matrix @ self.p[i] + self.initial_height + self.translation
            self.l[i] = self.q[i] - self.b[i]
        
        self._calculate_angles()
    
    def _calculate_angles(self):
        """
        Calculate servo angles using inverse kinematics.
        
        Sets valid_pose to False if pose is unreachable.
        """
        self.valid_pose = True

        g = self.geometry
        for i in range(6):
            L = np.dot(self.l[i], self.l[i]) - (g.rod_length ** 2 - g.horn_length ** 2)
            M = 2 * g.horn_length * (self.q[i, 2] - self.b[i, 2])
            N = 2 * g.horn_length * (
                np.cos(g.beta[i]) * (self.q[i, 0] - self.b[i, 0]) +
                np.sin(g.beta[i]) * (self.q[i, 1] - self.b[i, 1])
            )

            denominator = np.sqrt(M * M + N * N)
            arcsin_arg = L / denominator if denominator > 0 else L

            self.alpha[i] = safe_arcsin(arcsin_arg)

            if np.isnan(self.alpha[i]):
                self.valid_pose = False
            else:
                self.alpha[i] -= np.arctan2(N, M)

            if not np.isnan(self.alpha[i]):
                self.a[i] = [
                    g.horn_length * np.cos(self.alpha[i]) * np.cos(g.beta[i]) + self.b[i, 0],
                    g.horn_length * np.cos(self.alpha[i]) * np.sin(g.beta[i]) + self.b[i, 1],
                    g.horn_length * np.sin(self.alpha[i]) + self.b[i, 2]
                ]
    
    def update_pose(self, translation, rotation):
        """
        Update platform pose (position and orientation).
        
        Args:
            translation: [x, y, z] translation in mm.
            rotation: [rx, ry, rz] rotation in radians.
        """
        self.translation = np.array(translation)
        self.rotation = np.array(rotation)
        self._calculate_vectors()
    
    def get_alpha_angles(self):
        """Return copy of servo angles in radians."""
        return self.alpha.copy()
    
    def get_translation(self):
        """Return copy of current translation."""
        return self.translation.copy()
    
    def get_rotation(self):
        """Return copy of current rotation in radians."""
        return self.rotation.copy()
    
    def is_valid_pose(self):
        """Check if current pose is reachable."""
        return self.valid_pose
