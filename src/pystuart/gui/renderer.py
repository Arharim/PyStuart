"""
OpenGL rendering for the Stewart platform model.

Draws the state of a pystuart.core.platform.StewartPlatform instance.
All model knowledge (IK, geometry) stays in the core package; this
module only reads the computed state and issues GL calls.
"""
import numpy as np
from OpenGL.GL import *


class PlatformRenderer:
    """Renders a StewartPlatform instance using legacy OpenGL."""

    def draw(self, platform):
        """Render base plate, platform plate and actuator legs."""
        self._draw_base(platform)
        self._draw_platform(platform)
        self._draw_legs(platform)

    def _draw_base(self, platform):
        """Draw the base plate."""
        glColor4f(0.3, 0.3, 0.35, 0.8)
        self._draw_circle(0, 0, 0, platform.geometry.base_radius)

        glColor4f(0.2, 0.2, 0.25, 0.8)
        self._draw_circle_border(0, 0, 0, platform.geometry.base_radius)

        glPointSize(10.0)
        glBegin(GL_POINTS)
        for i in range(6):
            glColor3f(0.1, 0.1, 0.1)
            glVertex3fv(platform.b[i])
        glEnd()

    def _draw_platform(self, platform):
        """Draw the platform plate."""
        center = platform.initial_height + platform.translation

        glPushMatrix()
        glTranslatef(center[0], center[1], center[2])
        glRotatef(np.degrees(platform.rotation[2]), 0, 0, 1)
        glRotatef(np.degrees(platform.rotation[1]), 0, 1, 0)
        glRotatef(np.degrees(platform.rotation[0]), 1, 0, 0)

        if platform.valid_pose:
            glColor4f(0.2, 0.6, 0.3, 0.8)
        else:
            glColor4f(0.8, 0.2, 0.2, 0.8)
        self._draw_circle(0, 0, 0, platform.geometry.platform_radius)

        glColor4f(0.15, 0.45, 0.2, 0.8) if platform.valid_pose else glColor4f(0.6, 0.15, 0.15, 0.8)
        self._draw_circle_border(0, 0, 0, platform.geometry.platform_radius)

        glPopMatrix()

        glPointSize(10.0)
        glBegin(GL_POINTS)
        for i in range(6):
            glColor3f(0.3, 0.5, 0.7)
            glVertex3fv(platform.q[i])
        glEnd()

    def _draw_legs(self, platform):
        """Draw the 6 actuator legs."""
        glLineWidth(4.0)
        glBegin(GL_LINES)
        for i in range(6):
            glColor3f(0.4, 0.4, 0.45)
            glVertex3fv(platform.b[i])
            glVertex3fv(platform.a[i])

            glColor3f(0.6, 0.6, 0.65)
            glVertex3fv(platform.a[i])
            glVertex3fv(platform.q[i])
        glEnd()

        glPointSize(8.0)
        glBegin(GL_POINTS)
        for i in range(6):
            glColor3f(0.9, 0.5, 0.1)
            glVertex3fv(platform.a[i])
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
