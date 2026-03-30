"""
OpenGL 3D widget for rendering the Stewart Platform.
"""
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from PyQt6.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLU import *
import math
from .stewart_platform import StewartPlatform


class GLWidget(QOpenGLWidget):
    """
    OpenGL widget for rendering the Stewart Platform simulation.
    
    Provides camera controls:
        - Left mouse: rotate camera
        - Right mouse: pan camera
        - Middle mouse: vertical offset
        - Mouse wheel: zoom
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.camera_distance = 400.0
        self.camera_azimuth = 45.0
        self.camera_elevation = 30.0
        self.camera_target = [0.0, 0.0, 60.0]
        self.last_mouse_pos = None
        self.platform = StewartPlatform()
        self.show_workspace = False
        self.workspace_data = None

    def initializeGL(self):
        """Initialize OpenGL settings."""
        glClearColor(0.15, 0.15, 0.18, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        glLightfv(GL_LIGHT0, GL_POSITION, [200.0, 200.0, 400.0, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])

    def resizeGL(self, w, h):
        """Handle widget resize."""
        if h == 0:
            h = 1
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = w / h
        gluPerspective(45.0, aspect, 1.0, 2000.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        """Render the scene."""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        cam_x = self.camera_distance * math.cos(math.radians(self.camera_elevation)) * math.cos(math.radians(self.camera_azimuth)) + self.camera_target[0]
        cam_y = self.camera_distance * math.cos(math.radians(self.camera_elevation)) * math.sin(math.radians(self.camera_azimuth)) + self.camera_target[1]
        cam_z = self.camera_distance * math.sin(math.radians(self.camera_elevation)) + self.camera_target[2]

        gluLookAt(
            cam_x, cam_y, cam_z,
            self.camera_target[0], self.camera_target[1], self.camera_target[2],
            0, 0, 1
        )

        glDisable(GL_LIGHTING)
        self.draw_axes()
        glEnable(GL_LIGHTING)
        
        self.platform.draw()

        if self.show_workspace and self.workspace_data is not None:
            glDisable(GL_LIGHTING)
            self._draw_workspace()
            glEnable(GL_LIGHTING)

    def draw_axes(self):
        """Draw coordinate system axes."""
        axis_length = 40.0

        glLineWidth(2.0)
        glBegin(GL_LINES)
        
        glColor3f(0.9, 0.2, 0.2)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(axis_length, 0.0, 0.0)

        glColor3f(0.2, 0.9, 0.2)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, axis_length, 0.0)

        glColor3f(0.2, 0.2, 0.9)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, axis_length)

        glEnd()

    def mousePressEvent(self, a0):
        """Handle mouse press."""
        self.last_mouse_pos = a0.pos()

    def mouseMoveEvent(self, a0):
        """Handle mouse movement for camera control."""
        if self.last_mouse_pos is not None:
            dx = a0.pos().x() - self.last_mouse_pos.x()
            dy = a0.pos().y() - self.last_mouse_pos.y()

            if a0.buttons() & Qt.MouseButton.LeftButton:
                self.camera_azimuth += dx * 0.5
                self.camera_elevation += dy * 0.5
                self.camera_elevation = max(-89, min(89, self.camera_elevation))

            elif a0.buttons() & Qt.MouseButton.RightButton:
                pan_speed = self.camera_distance * 0.002
                azimuth_rad = math.radians(self.camera_azimuth)
                
                self.camera_target[0] -= (dx * math.cos(azimuth_rad) + dy * math.sin(azimuth_rad)) * pan_speed
                self.camera_target[1] -= (-dx * math.sin(azimuth_rad) + dy * math.cos(azimuth_rad)) * pan_speed

            elif a0.buttons() & Qt.MouseButton.MiddleButton:
                self.camera_target[2] += dy * 0.5

            self.last_mouse_pos = a0.pos()
            self.update()

    def wheelEvent(self, a0):
        """Handle mouse wheel for zoom."""
        delta = a0.angleDelta().y()
        self.camera_distance -= delta * 0.5
        self.camera_distance = max(100.0, min(1000.0, self.camera_distance))
        self.update()

    def _draw_workspace(self):
        data = self.workspace_data
        if data is None:
            return

        glPointSize(4.0)
        glBegin(GL_POINTS)

        if 'slice_points' in data and data['slice_points'] is not None:
            sp = data['slice_points']
            glColor4f(0.2, 0.8, 0.2, 0.3)
            for pt in sp.get('valid', []):
                glVertex3f(*pt)
            glColor4f(0.8, 0.2, 0.2, 0.15)
            for pt in sp.get('invalid', []):
                glVertex3f(*pt)

        glEnd()

        if 'boundary_points' in data and data['boundary_points']:
            glPointSize(5.0)
            glBegin(GL_POINTS)
            glColor4f(0.3, 0.7, 1.0, 0.5)
            for pt in data['boundary_points']:
                glVertex3f(*pt)
            glEnd()

        if 'bounds' in data and data['bounds'] is not None:
            b = data['bounds']
            h = StewartPlatform.INITIAL_HEIGHT
            xn, xx = b['pos_x']
            yn, yx = b['pos_y']
            zn, zx = b['pos_z'][0] + h, b['pos_z'][1] + h
            glColor4f(1.0, 1.0, 0.3, 0.25)
            glLineWidth(1.5)
            self._draw_wireframe_box(xn, xx, yn, yx, zn, zx)

    def _draw_wireframe_box(self, xn, xx, yn, yx, zn, zx):
        edges = [
            (xn, yn, zn), (xx, yn, zn),
            (xx, yx, zn), (xn, yx, zn),
            (xn, yn, zx), (xx, yn, zx),
            (xx, yx, zx), (xn, yx, zx),
        ]
        lines = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        glBegin(GL_LINES)
        for a, b_idx in lines:
            glVertex3f(*edges[a])
            glVertex3f(*edges[b_idx])
        glEnd()

    def set_workspace_data(self, analyzer):
        if analyzer is None or not analyzer.is_computed():
            self.workspace_data = None
            return
        self.workspace_data = {
            'bounds': analyzer.bounds,
            'boundary_points': list(analyzer.boundary_points),
            'slice_points': analyzer.slice_points,
        }

    def reset_camera(self):
        """Reset camera to default position."""
        self.camera_distance = 400.0
        self.camera_azimuth = 45.0
        self.camera_elevation = 30.0
        self.camera_target = [0.0, 0.0, 60.0]
        self.update()
