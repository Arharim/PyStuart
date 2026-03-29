from PyQt6.QtWidgets import QMainWindow, QWidget, QHBoxLayout
from PyQt6.QtCore import Qt
from .gl_widget import GLWidget
from .control_panel import ControlPanel


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stewart Platform Simulator v0.6")
        self.setMinimumSize(1000, 600)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.gl_widget = GLWidget()
        layout.addWidget(self.gl_widget, stretch=1)

        self.control_panel = ControlPanel()
        self.control_panel.pose_changed.connect(self._on_pose_changed)
        self.control_panel.reset_requested.connect(self._on_reset)
        self.control_panel.reset_camera_requested.connect(self._on_reset_camera)
        layout.addWidget(self.control_panel)

    def _on_pose_changed(self):
        pose = self.control_panel.get_pose()
        translation = [pose['pos_x'], pose['pos_y'], pose['pos_z']]
        rotation = [pose['rot_x'], pose['rot_y'], pose['rot_z']]
        self.gl_widget.platform.update_pose(translation, rotation)
        self.gl_widget.update()
        self._update_feedback()

    def _on_reset(self):
        self.gl_widget.platform.update_pose([0, 0, 0], [0, 0, 0])
        self.gl_widget.update()
        self._update_feedback()

    def _on_reset_camera(self):
        self.gl_widget.reset_camera()

    def _update_feedback(self):
        alpha_angles = self.gl_widget.platform.get_alpha_angles()
        rotation = self.gl_widget.platform.get_rotation()
        self.control_panel.update_feedback(alpha_angles, rotation)
