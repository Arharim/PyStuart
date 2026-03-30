from PyQt6.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QMessageBox, QScrollArea, QFileDialog, QProgressDialog
from PyQt6.QtCore import Qt, QTimer
from .gl_widget import GLWidget
from .control_panel import ControlPanel
from .trajectory import Trajectory
from .workspace import WorkspaceAnalyzer
import csv
import math


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
        self.control_panel.export_current_requested.connect(self._on_export_current)
        self.control_panel.export_trajectory_requested.connect(self._on_export_trajectory)
        self.control_panel.workspace_compute_requested.connect(self._on_compute_workspace)
        self.control_panel.workspace_toggle_requested.connect(self._on_workspace_toggle)

        self._workspace_analyzer = None

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

    def _on_export_current(self):
        alpha = self.gl_widget.platform.get_alpha_angles()
        translation = self.gl_widget.platform.get_translation()
        rotation = self.gl_widget.platform.get_rotation()
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export Servo Angles", "servo_angles.csv", "CSV Files (*.csv);;All Files (*)"
        )
        if not filepath:
            return
        try:
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'pos_x', 'pos_y', 'pos_z',
                    'rot_x_deg', 'rot_y_deg', 'rot_z_deg',
                    'alpha0_deg', 'alpha1_deg', 'alpha2_deg',
                    'alpha3_deg', 'alpha4_deg', 'alpha5_deg'
                ])
                row = [
                    f"{translation[0]:.4f}", f"{translation[1]:.4f}", f"{translation[2]:.4f}",
                    f"{math.degrees(rotation[0]):.4f}", f"{math.degrees(rotation[1]):.4f}", f"{math.degrees(rotation[2]):.4f}",
                ]
                for a in alpha:
                    row.append(f"{math.degrees(a):.4f}" if not math.isnan(a) else "N/A")
                writer.writerow(row)
        except IOError as e:
            QMessageBox.warning(self, "Export Error", f"Failed to write file:\n{e}")

    def _on_export_trajectory(self):
        if self.trajectory.is_empty():
            QMessageBox.information(self, "Export", "No trajectory loaded.")
            return
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export Trajectory Angles", "trajectory_angles.csv", "CSV Files (*.csv);;All Files (*)"
        )
        if not filepath:
            return
        try:
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'frame', 'pos_x', 'pos_y', 'pos_z',
                    'rot_x_deg', 'rot_y_deg', 'rot_z_deg',
                    'alpha0_deg', 'alpha1_deg', 'alpha2_deg',
                    'alpha3_deg', 'alpha4_deg', 'alpha5_deg'
                ])
                total = self.trajectory.total_frames()
                platform = self.gl_widget.platform
                for i in range(total):
                    pt = self.trajectory.get_frame(i)
                    translation = [pt.pos_x, pt.pos_y, pt.pos_z]
                    rotation = [math.radians(pt.rot_x), math.radians(pt.rot_y), math.radians(pt.rot_z)]
                    platform.update_pose(translation, rotation)
                    alpha = platform.get_alpha_angles()
                    row = [i,
                           f"{pt.pos_x:.4f}", f"{pt.pos_y:.4f}", f"{pt.pos_z:.4f}",
                           f"{pt.rot_x:.4f}", f"{pt.rot_y:.4f}", f"{pt.rot_z:.4f}"]
                    for a in alpha:
                        row.append(f"{math.degrees(a):.4f}" if not math.isnan(a) else "N/A")
                    writer.writerow(row)
                pose = self.control_panel.get_pose()
                platform.update_pose(
                    [pose['pos_x'], pose['pos_y'], pose['pos_z']],
                    [pose['rot_x'], pose['rot_y'], pose['rot_z']]
                )
                self.gl_widget.update()
                self._update_feedback()
        except IOError as e:
            QMessageBox.warning(self, "Export Error", f"Failed to write file:\n{e}")

    def _on_compute_workspace(self):
        progress = QProgressDialog("Computing workspace boundaries...", None, 0, 0, self)
        progress.setWindowTitle("Workspace Analysis")
        progress.setWindowModality(Qt.WindowModality.WindowModal)
        progress.setMinimumDuration(0)
        progress.setCancelButton(None)
        progress.show()

        from PyQt6.QtWidgets import QApplication
        QApplication.processEvents()

        analyzer = WorkspaceAnalyzer()
        bounds = analyzer.compute_bounds()

        progress.close()

        self._workspace_analyzer = analyzer
        self.control_panel.set_workspace_info(bounds)
        self.gl_widget.set_workspace_data(analyzer)
        self.gl_widget.update()

    def _on_workspace_toggle(self, show: bool):
        self.gl_widget.show_workspace = show
        self.gl_widget.update()
