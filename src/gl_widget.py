from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from PyQt6.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLU import *
import math


class GLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.camera_distance = 5.0
        self.camera_azimuth = 45.0
        self.camera_elevation = 30.0
        self.last_mouse_pos = None

    def initializeGL(self):
        glClearColor(0.1, 0.1, 0.15, 1.0)
        glEnable(GL_DEPTH_TEST)
        glLineWidth(2.0)

    def resizeGL(self, width, height):
        if height == 0:
            height = 1
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = width / height
        gluPerspective(45.0, aspect, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        cam_x = self.camera_distance * math.cos(math.radians(self.camera_elevation)) * math.cos(math.radians(self.camera_azimuth))
        cam_y = self.camera_distance * math.cos(math.radians(self.camera_elevation)) * math.sin(math.radians(self.camera_azimuth))
        cam_z = self.camera_distance * math.sin(math.radians(self.camera_elevation))

        gluLookAt(cam_x, cam_y, cam_z, 0, 0, 0, 0, 0, 1)

        self.draw_axes()

    def draw_axes(self):
        axis_length = 2.0

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

        glPointSize(8.0)
        glBegin(GL_POINTS)
        
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(axis_length, 0.0, 0.0)

        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, axis_length, 0.0)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, axis_length)

        glEnd()

    def mousePressEvent(self, event):
        self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is not None:
            dx = event.pos().x() - self.last_mouse_pos.x()
            dy = event.pos().y() - self.last_mouse_pos.y()

            if event.buttons() & Qt.MouseButton.LeftButton:
                self.camera_azimuth += dx * 0.5
                self.camera_elevation += dy * 0.5
                self.camera_elevation = max(-89, min(89, self.camera_elevation))

            self.last_mouse_pos = event.pos()
            self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        self.camera_distance -= delta * 0.005
        self.camera_distance = max(1.0, min(20.0, self.camera_distance))
        self.update()
