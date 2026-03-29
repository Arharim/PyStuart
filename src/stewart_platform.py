"""
Stewart Platform model with inverse kinematics.

This module provides the StewartPlatform class for simulating
a 6-DOF parallel manipulator (Stewart platform / hexapod).
"""
import numpy as np
from OpenGL.GL import *
from .math_utils import rotation_matrix, safe_arcsin


class StewartPlatform:
    """
    Stewart Platform (hexapod) model.
    
    A 6-DOF parallel manipulator with 6 linear actuators connecting
    a base plate to a movable platform.
    
    Attributes:
        BASE_RADIUS: Radius of the base plate in mm.
        PLATFORM_RADIUS: Radius of the platform plate in mm.
        HORN_LENGTH: Length of servo horns in mm.
        ROD_LENGTH: Length of connecting rods in mm.
        INITIAL_HEIGHT: Initial height of platform above base in mm.
    """
    
    BASE_ANGLES = np.array([-50, -70, -170, -190, -290, -310])
    PLATFORM_ANGLES = np.array([-54, -66, -174, -186, -294, -306])
    BETA = np.array([np.pi / 6, -5 * np.pi / 6, -np.pi / 2, np.pi / 2, 5 * np.pi / 6, -np.pi / 6])
    
    BASE_RADIUS = 76.0
    PLATFORM_RADIUS = 60.0
    HORN_LENGTH = 40.0
    ROD_LENGTH = 130.0
    INITIAL_HEIGHT = 120.28183632

    def __init__(self):
        """Initialize the Stewart platform with default pose."""
        self.translation = np.array([0.0, 0.0, 0.0])
        self.rotation = np.array([0.0, 0.0, 0.0])
        self.initial_height = np.array([0.0, 0.0, self.INITIAL_HEIGHT])
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
        for i in range(6):
            base_angle_rad = np.radians(self.BASE_ANGLES[i])
            self.b[i] = [
                self.BASE_RADIUS * np.cos(base_angle_rad),
                self.BASE_RADIUS * np.sin(base_angle_rad),
                0.0
            ]
            
            platform_angle_rad = np.radians(self.PLATFORM_ANGLES[i])
            self.p[i] = [
                self.PLATFORM_RADIUS * np.cos(platform_angle_rad),
                self.PLATFORM_RADIUS * np.sin(platform_angle_rad),
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
        
        for i in range(6):
            L = np.dot(self.l[i], self.l[i]) - (self.ROD_LENGTH ** 2 - self.HORN_LENGTH ** 2)
            M = 2 * self.HORN_LENGTH * (self.q[i, 2] - self.b[i, 2])
            N = 2 * self.HORN_LENGTH * (
                np.cos(self.BETA[i]) * (self.q[i, 0] - self.b[i, 0]) +
                np.sin(self.BETA[i]) * (self.q[i, 1] - self.b[i, 1])
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
                    self.HORN_LENGTH * np.cos(self.alpha[i]) * np.cos(self.BETA[i]) + self.b[i, 0],
                    self.HORN_LENGTH * np.cos(self.alpha[i]) * np.sin(self.BETA[i]) + self.b[i, 1],
                    self.HORN_LENGTH * np.sin(self.alpha[i]) + self.b[i, 2]
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
    
    def draw(self):
        """Render the platform using OpenGL."""
        self._draw_base()
        self._draw_platform()
        self._draw_legs()
    
    def _draw_base(self):
        """Draw the base plate."""
        glColor4f(0.3, 0.3, 0.35, 0.8)
        self._draw_circle(0, 0, 0, self.BASE_RADIUS)
        
        glColor4f(0.2, 0.2, 0.25, 0.8)
        self._draw_circle_border(0, 0, 0, self.BASE_RADIUS)
        
        glPointSize(10.0)
        glBegin(GL_POINTS)
        for i in range(6):
            glColor3f(0.1, 0.1, 0.1)
            glVertex3fv(self.b[i])
        glEnd()
    
    def _draw_platform(self):
        """Draw the platform plate."""
        center = self.initial_height + self.translation
        
        glPushMatrix()
        glTranslatef(center[0], center[1], center[2])
        glRotatef(np.degrees(self.rotation[2]), 0, 0, 1)
        glRotatef(np.degrees(self.rotation[1]), 0, 1, 0)
        glRotatef(np.degrees(self.rotation[0]), 1, 0, 0)
        
        if self.valid_pose:
            glColor4f(0.2, 0.6, 0.3, 0.8)
        else:
            glColor4f(0.8, 0.2, 0.2, 0.8)
        self._draw_circle(0, 0, 0, self.PLATFORM_RADIUS)
        
        glColor4f(0.15, 0.45, 0.2, 0.8) if self.valid_pose else glColor4f(0.6, 0.15, 0.15, 0.8)
        self._draw_circle_border(0, 0, 0, self.PLATFORM_RADIUS)
        
        glPopMatrix()
        
        glPointSize(10.0)
        glBegin(GL_POINTS)
        for i in range(6):
            glColor3f(0.3, 0.5, 0.7)
            glVertex3fv(self.q[i])
        glEnd()
    
    def _draw_legs(self):
        """Draw the 6 actuator legs."""
        glLineWidth(4.0)
        glBegin(GL_LINES)
        for i in range(6):
            glColor3f(0.4, 0.4, 0.45)
            glVertex3fv(self.b[i])
            glVertex3fv(self.a[i])
            
            glColor3f(0.6, 0.6, 0.65)
            glVertex3fv(self.a[i])
            glVertex3fv(self.q[i])
        glEnd()
        
        glPointSize(8.0)
        glBegin(GL_POINTS)
        for i in range(6):
            glColor3f(0.9, 0.5, 0.1)
            glVertex3fv(self.a[i])
        glEnd()
    
    def _draw_circle(self, x, y, z, radius, segments=64):
        """Draw a filled circle."""
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(x, y, z)
        for i in range(segments + 1):
            angle = 2 * np.pi * i / segments
            glVertex3f(
                x + radius * np.cos(angle),
                y + radius * np.sin(angle),
                z
            )
        glEnd()
    
    def _draw_circle_border(self, x, y, z, radius, segments=64):
        """Draw a circle outline."""
        glLineWidth(2.0)
        glBegin(GL_LINE_LOOP)
        for i in range(segments):
            angle = 2 * np.pi * i / segments
            glVertex3f(
                x + radius * np.cos(angle),
                y + radius * np.sin(angle),
                z
            )
        glEnd()
