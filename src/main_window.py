from PyQt6.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QMessageBox, QScrollArea
from PyQt6.QtCore import Qt, QTimer
from .gl_widget import GLWidget
from .control_panel import ControlPanel
from .trajectory import Trajectory


class MainWindow(QMainWindow):
    TIMER_INTERVAL_MS = 16

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stewart Platform Simulator v1.0")
        self.setMinimumSize(1000, 600)

        self.trajectory = Trajectory()
        self._current_frame = 0
        self._playing = False
        self._seeking = False

        self._timer = QTimer(self)
        self._timer.setInterval(self.TIMER_INTERVAL_MS)
        self._timer.timeout.connect(self._on_animation_tick)

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
        self.control_panel.trajectory_import_requested.connect(self._on_import_trajectory)
        self.control_panel.trajectory_play_requested.connect(self._on_trajectory_play)
        self.control_panel.trajectory_pause_requested.connect(self._on_trajectory_pause)
        self.control_panel.trajectory_stop_requested.connect(self._on_trajectory_stop)
        self.control_panel.trajectory_seek_requested.connect(self._on_trajectory_seek)

        scroll = QScrollArea()
        scroll.setWidget(self.control_panel)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(270)
        layout.addWidget(scroll)

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

    def _on_import_trajectory(self, filepath: str):
        count, err = self.trajectory.load_csv(filepath)
        if err:
            QMessageBox.warning(self, "Import Error", f"Failed to import CSV:\n{err}")
            return
        self._current_frame = 0
        self._playing = False
        self._timer.stop()
        self.control_panel.set_trajectory_info(count, self.trajectory.total_frames())
        self.control_panel.set_playing_state(False)
        pt = self.trajectory.get_frame(0)
        if pt:
            self._apply_trajectory_point(pt)

    def _on_trajectory_play(self):
        if self.trajectory.is_empty():
            return
        if self._current_frame >= self.trajectory.total_frames() - 1:
            self._current_frame = 0
        self._playing = True
        self.control_panel.set_playing_state(True)
        self._timer.start()

    def _on_trajectory_pause(self):
        self._playing = False
        self._timer.stop()
        self.control_panel.set_playing_state(False)

    def _on_trajectory_stop(self):
        self._playing = False
        self._timer.stop()
        self._current_frame = 0
        self.control_panel.set_playing_state(False)
        pt = self.trajectory.get_frame(0)
        if pt:
            self._apply_trajectory_point(pt)
        total = self.trajectory.total_frames()
        self.control_panel.update_trajectory_frame(0, total)

    def _on_trajectory_seek(self, frame: int):
        if self._seeking:
            return
        self._seeking = True
        pt = self.trajectory.get_frame(frame)
        if pt:
            self._current_frame = frame
            self._apply_trajectory_point(pt)
            total = self.trajectory.total_frames()
            self.control_panel.update_trajectory_frame(frame, total)
        self._seeking = False

    def _on_animation_tick(self):
        if not self._playing or self.trajectory.is_empty():
            self._timer.stop()
            return

        speed = self.control_panel.get_speed_multiplier()
        step = max(1, round(speed))
        self._current_frame += step
        total = self.trajectory.total_frames()

        if self._current_frame >= total:
            if self.control_panel.is_loop_enabled():
                self._current_frame = 0
            else:
                self._current_frame = total - 1
                self._playing = False
                self._timer.stop()
                self.control_panel.set_playing_state(False)

        pt = self.trajectory.get_frame(self._current_frame)
        if pt:
            self._apply_trajectory_point(pt)
        self.control_panel.update_trajectory_frame(self._current_frame, total)

    def _apply_trajectory_point(self, point):
        pose = point.to_pose_dict()
        self.control_panel.set_pose(pose)
        translation = [pose['pos_x'], pose['pos_y'], pose['pos_z']]
        rotation = [pose['rot_x'], pose['rot_y'], pose['rot_z']]
        self.gl_widget.platform.update_pose(translation, rotation)
        self.gl_widget.update()
        self._update_feedback()
