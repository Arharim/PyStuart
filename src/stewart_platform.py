import numpy as np
from OpenGL.GL import *


class StewartPlatform:
    BASE_ANGLES = np.array([-50, -70, -170, -190, -290, -310])
    PLATFORM_ANGLES = np.array([-54, -66, -174, -186, -294, -306])
    BETA = np.array([np.pi / 6, -5 * np.pi / 6, -np.pi / 2, np.pi / 2, 5 * np.pi / 6, -np.pi / 6])
    
    BASE_RADIUS = 76.0
    PLATFORM_RADIUS = 60.0
    HORN_LENGTH = 40.0
    ROD_LENGTH = 130.0
    INITIAL_HEIGHT = 120.28183632

    def __init__(self):
        self.translation = np.array([0.0, 0.0, 0.0])
        self.rotation = np.array([0.0, 0.0, 0.0])
        self.initial_height = np.array([0.0, 0.0, self.INITIAL_HEIGHT])
        
        self.b = np.zeros((6, 3))
        self.p = np.zeros((6, 3))
        self.q = np.zeros((6, 3))
        self.l = np.zeros((6, 3))
        self.a = np.zeros((6, 3))
        self.alpha = np.zeros(6)
        
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
        
        self._calculate_vectors()
    
    def _rotation_matrix(self, rotation):
        rx, ry, rz = rotation
        
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        
        return Rz @ Ry @ Rx
    
    def _calculate_vectors(self):
        R_matrix = self._rotation_matrix(self.rotation)
        
        for i in range(6):
            self.q[i] = R_matrix @ self.p[i] + self.initial_height + self.translation
            self.l[i] = self.q[i] - self.b[i]
        
        self._calculate_angles()
    
    def _calculate_angles(self):
        for i in range(6):
            L = np.dot(self.l[i], self.l[i]) - (self.ROD_LENGTH ** 2 - self.HORN_LENGTH ** 2)
            M = 2 * self.HORN_LENGTH * (self.q[i, 2] - self.b[i, 2])
            N = 2 * self.HORN_LENGTH * (
                np.cos(self.BETA[i]) * (self.q[i, 0] - self.b[i, 0]) +
                np.sin(self.BETA[i]) * (self.q[i, 1] - self.b[i, 1])
            )
            
            self.alpha[i] = np.arcsin(L / np.sqrt(M * M + N * N)) - np.arctan2(N, M)
            
            self.a[i] = [
                self.HORN_LENGTH * np.cos(self.alpha[i]) * np.cos(self.BETA[i]) + self.b[i, 0],
                self.HORN_LENGTH * np.cos(self.alpha[i]) * np.sin(self.BETA[i]) + self.b[i, 1],
                self.HORN_LENGTH * np.sin(self.alpha[i]) + self.b[i, 2]
            ]
    
    def update_pose(self, translation, rotation):
        self.translation = np.array(translation)
        self.rotation = np.array(rotation)
        self._calculate_vectors()
    
    def get_alpha_angles(self):
        return self.alpha.copy()
    
    def get_translation(self):
        return self.translation.copy()
    
    def get_rotation(self):
        return self.rotation.copy()
    
    def draw(self):
        self._draw_base()
        self._draw_platform()
        self._draw_legs()
    
    def _draw_base(self):
        glColor4f(1.0, 0.9, 0.8, 0.5)
        self._draw_circle(0, 0, 0, self.BASE_RADIUS)
        
        glPointSize(8.0)
        glBegin(GL_POINTS)
        for i in range(6):
            glColor3f(0.2, 0.2, 0.2)
            glVertex3fv(self.b[i])
        glEnd()
    
    def _draw_platform(self):
        center = self.initial_height + self.translation
        
        glPushMatrix()
        glTranslatef(center[0], center[1], center[2])
        glRotatef(np.degrees(self.rotation[2]), 0, 0, 1)
        glRotatef(np.degrees(self.rotation[1]), 0, 1, 0)
        glRotatef(np.degrees(self.rotation[0]), 1, 0, 0)
        
        glColor4f(0.8, 1.0, 0.8, 0.5)
        self._draw_circle(0, 0, 0, self.PLATFORM_RADIUS)
        
        glPopMatrix()
        
        glPointSize(8.0)
        glBegin(GL_POINTS)
        for i in range(6):
            glColor3f(0.4, 0.4, 0.8)
            glVertex3fv(self.q[i])
        glEnd()
    
    def _draw_legs(self):
        glLineWidth(3.0)
        glBegin(GL_LINES)
        for i in range(6):
            glColor3f(0.3, 0.3, 0.3)
            glVertex3fv(self.b[i])
            glVertex3fv(self.a[i])
            
            glColor3f(0.5, 0.5, 0.6)
            glVertex3fv(self.a[i])
            glVertex3fv(self.q[i])
        glEnd()
        
        glPointSize(6.0)
        glBegin(GL_POINTS)
        for i in range(6):
            glColor3f(0.8, 0.4, 0.2)
            glVertex3fv(self.a[i])
        glEnd()
    
    def _draw_circle(self, x, y, z, radius, segments=64):
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
