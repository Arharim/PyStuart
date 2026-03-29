from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from PyQt6.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLU import *
import math
from .stewart_platform import StewartPlatform


class GLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.camera_distance = 400.0
        self.camera_azimuth = 45.0
        self.camera_elevation = 30.0
        self.last_mouse_pos = None
        self.platform = StewartPlatform()

    def initializeGL(self):
        glClearColor(0.1, 0.1, 0.15, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glLineWidth(2.0)

    def resizeGL(self, w, h):
        if h == 0:
            h = 1
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = w / h
        gluPerspective(45.0, aspect, 1.0, 2000.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        cam_x = self.camera_distance * math.cos(math.radians(self.camera_elevation)) * math.cos(math.radians(self.camera_azimuth))
        cam_y = self.camera_distance * math.cos(math.radians(self.camera_elevation)) * math.sin(math.radians(self.camera_azimuth))
        cam_z = self.camera_distance * math.sin(math.radians(self.camera_elevation))

        gluLookAt(cam_x, cam_y, cam_z, 0, 0, 60, 0, 0, 1)

        self.draw_axes()
        self.platform.draw()

    def draw_axes(self):
        axis_length = 40.0

        glLineWidth(2.0)
        glBegin(GL_LINES)
        
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(axis_length, 0.0, 0.0)

        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, axis_length, 0.0)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, axis_length)

        glEnd()

    def mousePressEvent(self, a0):
        self.last_mouse_pos = a0.pos()

    def mouseMoveEvent(self, a0):
        if self.last_mouse_pos is not None:
            dx = a0.pos().x() - self.last_mouse_pos.x()
            dy = a0.pos().y() - self.last_mouse_pos.y()

            if a0.buttons() & Qt.MouseButton.LeftButton:
                self.camera_azimuth += dx * 0.5
                self.camera_elevation += dy * 0.5
                self.camera_elevation = max(-89, min(89, self.camera_elevation))

            self.last_mouse_pos = a0.pos()
            self.update()

    def wheelEvent(self, a0):
        delta = a0.angleDelta().y()
        self.camera_distance -= delta * 0.5
        self.camera_distance = max(100.0, min(1000.0, self.camera_distance))
        self.update()
