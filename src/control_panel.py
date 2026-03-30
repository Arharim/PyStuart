from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QPushButton, QGroupBox, QFrame,
    QComboBox, QInputDialog, QMessageBox, QFileDialog
)
from PyQt6.QtCore import Qt, pyqtSignal
import math
import numpy as np
from .pose_presets import PosePresets


class ControlPanel(QWidget):
    pose_changed = pyqtSignal()
    reset_requested = pyqtSignal()
    reset_camera_requested = pyqtSignal()
    trajectory_import_requested = pyqtSignal(str)
    trajectory_play_requested = pyqtSignal()
    trajectory_pause_requested = pyqtSignal()
    trajectory_stop_requested = pyqtSignal()
    trajectory_seek_requested = pyqtSignal(int)
    export_current_requested = pyqtSignal()
    export_trajectory_requested = pyqtSignal()
    workspace_toggle_requested = pyqtSignal(bool)
    workspace_compute_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.sliders = {}
        self.alpha_labels = []
        self.rotation_labels = []
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        translation_group = QGroupBox("Translation")
        translation_layout = QVBoxLayout(translation_group)
        
        self.sliders['pos_x'] = self._create_slider_row(translation_layout, "X:", -30, 30, 0)
        self.sliders['pos_y'] = self._create_slider_row(translation_layout, "Y:", -30, 30, 0)
        self.sliders['pos_z'] = self._create_slider_row(translation_layout, "Z:", -30, 30, 0)

        layout.addWidget(translation_group)

        rotation_group = QGroupBox("Rotation")
        rotation_layout = QVBoxLayout(rotation_group)
        
        self.sliders['rot_x'] = self._create_slider_row(rotation_layout, "Roll (°):", -30, 30, 0)
        self.sliders['rot_y'] = self._create_slider_row(rotation_layout, "Pitch (°):", -30, 30, 0)
        self.sliders['rot_z'] = self._create_slider_row(rotation_layout, "Yaw (°):", -30, 30, 0)

        layout.addWidget(rotation_group)

        line1 = QFrame()
        line1.setFrameShape(QFrame.Shape.HLine)
        line1.setFrameShadow(QFrame.Shadow.Sunken)
        layout.addWidget(line1)

        feedback_group = QGroupBox("Feedback")
        feedback_layout = QVBoxLayout(feedback_group)

        alpha_group = QGroupBox("Servo Angles (°)")
        alpha_layout = QVBoxLayout(alpha_group)
        alpha_layout.setSpacing(2)
        
        for i in range(6):
            row_layout = QHBoxLayout()
            label = QLabel(f"α{i}:")
            label.setMinimumWidth(25)
            row_layout.addWidget(label)
            
            value_label = QLabel("0.00")
            value_label.setMinimumWidth(50)
            value_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            row_layout.addWidget(value_label)
            row_layout.addStretch()
            
            self.alpha_labels.append(value_label)
            alpha_layout.addLayout(row_layout)
        
        feedback_layout.addWidget(alpha_group)

        orientation_group = QGroupBox("Orientation (°)")
        orientation_layout = QVBoxLayout(orientation_group)
        orientation_layout.setSpacing(2)
        
        for axis in ["Roll", "Pitch", "Yaw"]:
            row_layout = QHBoxLayout()
            label = QLabel(f"{axis}:")
            label.setMinimumWidth(40)
            row_layout.addWidget(label)
            
            value_label = QLabel("0.00")
            value_label.setMinimumWidth(50)
            value_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            row_layout.addWidget(value_label)
            row_layout.addStretch()
            
            self.rotation_labels.append(value_label)
            orientation_layout.addLayout(row_layout)
        
        feedback_layout.addWidget(orientation_group)

        export_current_btn = QPushButton("Export Current Angles")
        export_current_btn.clicked.connect(self._on_export_current)
        feedback_layout.addWidget(export_current_btn)

        layout.addWidget(feedback_group)

        ws_group = QGroupBox("Workspace Boundaries")
        ws_layout = QVBoxLayout(ws_group)

        compute_ws_btn = QPushButton("Compute Workspace")
        compute_ws_btn.clicked.connect(self._on_compute_workspace)
        ws_layout.addWidget(compute_ws_btn)

        self.ws_toggle_btn = QPushButton("Show Workspace: OFF")
        self.ws_toggle_btn.setCheckable(True)
        self.ws_toggle_btn.setEnabled(False)
        self.ws_toggle_btn.toggled.connect(self._on_workspace_toggled)
        ws_layout.addWidget(self.ws_toggle_btn)

        self.ws_info_label = QLabel("Not computed")
        self.ws_info_label.setWordWrap(True)
        ws_layout.addWidget(self.ws_info_label)

        layout.addWidget(ws_group)

        line2 = QFrame()
        line2.setFrameShape(QFrame.Shape.HLine)
        line2.setFrameShadow(QFrame.Shadow.Sunken)
        layout.addWidget(line2)

        reset_btn = QPushButton("Reset Pose")
        reset_btn.clicked.connect(self._on_reset)
        layout.addWidget(reset_btn)

        reset_camera_btn = QPushButton("Reset Camera")
        reset_camera_btn.clicked.connect(self._on_reset_camera)
        layout.addWidget(reset_camera_btn)

        line3 = QFrame()
        line3.setFrameShape(QFrame.Shape.HLine)
        line3.setFrameShadow(QFrame.Shadow.Sunken)
        layout.addWidget(line3)

        presets_group = QGroupBox("Pose Presets")
        presets_layout = QVBoxLayout(presets_group)

        self.preset_combo = QComboBox()
        self.preset_combo.addItem("-- Select Preset --")
        self.preset_combo.currentIndexChanged.connect(self._on_preset_selected)
        presets_layout.addWidget(self.preset_combo)

        btn_layout = QHBoxLayout()
        save_btn = QPushButton("Save")
        save_btn.clicked.connect(self._on_save_preset)
        btn_layout.addWidget(save_btn)

        delete_btn = QPushButton("Delete")
        delete_btn.clicked.connect(self._on_delete_preset)
        btn_layout.addWidget(delete_btn)

        presets_layout.addLayout(btn_layout)
        layout.addWidget(presets_group)

        self.presets_manager = PosePresets()
        self._refresh_presets()

        line4 = QFrame()
        line4.setFrameShape(QFrame.Shape.HLine)
        line4.setFrameShadow(QFrame.Shadow.Sunken)
        layout.addWidget(line4)

        traj_group = QGroupBox("Trajectory Animation")
        traj_layout = QVBoxLayout(traj_group)

        import_btn = QPushButton("Import CSV")
        import_btn.clicked.connect(self._on_import_csv)
        traj_layout.addWidget(import_btn)

        self.traj_info_label = QLabel("No trajectory loaded")
        self.traj_info_label.setWordWrap(True)
        traj_layout.addWidget(self.traj_info_label)

        self.traj_progress = QSlider(Qt.Orientation.Horizontal)
        self.traj_progress.setMinimum(0)
        self.traj_progress.setMaximum(0)
        self.traj_progress.setValue(0)
        self.traj_progress.valueChanged.connect(self._on_traj_seek)
        traj_layout.addWidget(self.traj_progress)

        self.traj_frame_label = QLabel("Frame: -")
        traj_layout.addWidget(self.traj_frame_label)

        play_btns = QHBoxLayout()
        self.play_btn = QPushButton("▶ Play")
        self.play_btn.clicked.connect(self._on_play)
        play_btns.addWidget(self.play_btn)

        self.pause_btn = QPushButton("⏸ Pause")
        self.pause_btn.clicked.connect(self._on_pause)
        self.pause_btn.setEnabled(False)
        play_btns.addWidget(self.pause_btn)

        self.stop_btn = QPushButton("⏹ Stop")
        self.stop_btn.clicked.connect(self._on_stop)
        self.stop_btn.setEnabled(False)
        play_btns.addWidget(self.stop_btn)
        traj_layout.addLayout(play_btns)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(20)
        self.speed_slider.setValue(5)
        self.speed_label = QLabel("1.0x")
        self.speed_label.setMinimumWidth(35)
        self.speed_slider.valueChanged.connect(
            lambda v: self.speed_label.setText(f"{v / 5:.1f}x")
        )
        speed_row.addWidget(self.speed_slider)
        speed_row.addWidget(self.speed_label)
        traj_layout.addLayout(speed_row)

        self.loop_checkbox = QPushButton("Loop: OFF")
        self.loop_checkbox.setCheckable(True)
        self.loop_checkbox.toggled.connect(
            lambda c: self.loop_checkbox.setText(f"Loop: {'ON' if c else 'OFF'}")
        )
        traj_layout.addWidget(self.loop_checkbox)

        export_traj_btn = QPushButton("Export Trajectory Angles")
        export_traj_btn.clicked.connect(self._on_export_trajectory)
        traj_layout.addWidget(export_traj_btn)

        layout.addWidget(traj_group)

        layout.addStretch()

        self.setMinimumWidth(250)

    def _create_slider_row(self, parent_layout, label_text, min_val, max_val, default_val):
        row_widget = QWidget()
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(0, 0, 0, 0)

        label = QLabel(label_text)
        label.setMinimumWidth(60)
        row_layout.addWidget(label)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setMinimum(int(min_val * 10))
        slider.setMaximum(int(max_val * 10))
        slider.setValue(int(default_val * 10))
        slider.valueChanged.connect(self._on_slider_changed)
        row_layout.addWidget(slider)

        value_label = QLabel(f"{default_val:.1f}")
        value_label.setMinimumWidth(40)
        value_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        row_layout.addWidget(value_label)

        slider.valueChanged.connect(lambda v: value_label.setText(f"{v / 10:.1f}"))

        parent_layout.addWidget(row_widget)

        return slider

    def _on_slider_changed(self):
        self.pose_changed.emit()

    def _on_reset(self):
        for slider in self.sliders.values():
            slider.setValue(0)
        self.reset_requested.emit()

    def _on_reset_camera(self):
        self.reset_camera_requested.emit()

    def get_pose(self):
        return {
            'pos_x': self.sliders['pos_x'].value() / 10.0,
            'pos_y': self.sliders['pos_y'].value() / 10.0,
            'pos_z': self.sliders['pos_z'].value() / 10.0,
            'rot_x': math.radians(self.sliders['rot_x'].value() / 10.0),
            'rot_y': math.radians(self.sliders['rot_y'].value() / 10.0),
            'rot_z': math.radians(self.sliders['rot_z'].value() / 10.0),
        }

    def set_pose(self, pose):
        if 'pos_x' in pose:
            self.sliders['pos_x'].setValue(int(pose['pos_x'] * 10))
        if 'pos_y' in pose:
            self.sliders['pos_y'].setValue(int(pose['pos_y'] * 10))
        if 'pos_z' in pose:
            self.sliders['pos_z'].setValue(int(pose['pos_z'] * 10))
        if 'rot_x' in pose:
            self.sliders['rot_x'].setValue(int(math.degrees(pose['rot_x']) * 10))
        if 'rot_y' in pose:
            self.sliders['rot_y'].setValue(int(math.degrees(pose['rot_y']) * 10))
        if 'rot_z' in pose:
            self.sliders['rot_z'].setValue(int(math.degrees(pose['rot_z']) * 10))

    def update_feedback(self, alpha_angles, rotation):
        for i, alpha in enumerate(alpha_angles):
            if np.isnan(alpha):
                self.alpha_labels[i].setText("N/A")
            else:
                self.alpha_labels[i].setText(f"{math.degrees(alpha):.2f}")
        
        for i, rot in enumerate(rotation):
            if np.isnan(rot):
                self.rotation_labels[i].setText("N/A")
            else:
                self.rotation_labels[i].setText(f"{math.degrees(rot):.2f}")

    def _refresh_presets(self):
        self.preset_combo.blockSignals(True)
        self.preset_combo.clear()
        self.preset_combo.addItem("-- Select Preset --")
        for name in self.presets_manager.get_preset_names():
            self.preset_combo.addItem(name)
        self.preset_combo.blockSignals(False)

    def _on_preset_selected(self, index):
        if index <= 0:
            return
        preset_name = self.preset_combo.itemText(index)
        pose = self.presets_manager.load_preset(preset_name)
        if pose:
            self.set_pose(pose)

    def _on_save_preset(self):
        name, ok = QInputDialog.getText(self, "Save Preset", "Preset name:")
        if ok and name:
            name = name.strip()
            if self.presets_manager.preset_exists(name):
                reply = QMessageBox.question(
                    self, "Overwrite Preset",
                    f"Preset '{name}' already exists. Overwrite?",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
                )
                if reply != QMessageBox.StandardButton.Yes:
                    return
            pose = self.get_pose()
            self.presets_manager.save_preset(name, pose)
            self._refresh_presets()

    def _on_delete_preset(self):
        index = self.preset_combo.currentIndex()
        if index <= 0:
            QMessageBox.information(self, "Delete Preset", "Please select a preset to delete.")
            return
        preset_name = self.preset_combo.itemText(index)
        reply = QMessageBox.question(
            self, "Delete Preset",
            f"Delete preset '{preset_name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.presets_manager.delete_preset(preset_name)
            self._refresh_presets()

    def _on_import_csv(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Import Trajectory CSV", "", "CSV Files (*.csv);;All Files (*)"
        )
        if filepath:
            self.trajectory_import_requested.emit(filepath)

    def _on_play(self):
        self.trajectory_play_requested.emit()

    def _on_pause(self):
        self.trajectory_pause_requested.emit()

    def _on_stop(self):
        self.trajectory_stop_requested.emit()

    def _on_traj_seek(self, value):
        self.trajectory_seek_requested.emit(value)

    def set_trajectory_info(self, waypoints: int, total_frames: int):
        self.traj_info_label.setText(f"Waypoints: {waypoints} | Frames: {total_frames}")
        self.traj_progress.blockSignals(True)
        self.traj_progress.setMaximum(max(0, total_frames - 1))
        self.traj_progress.setValue(0)
        self.traj_progress.blockSignals(False)
        self.traj_frame_label.setText(f"Frame: 0/{total_frames}")

    def update_trajectory_frame(self, frame: int, total: int):
        self.traj_progress.blockSignals(True)
        self.traj_progress.setValue(frame)
        self.traj_progress.blockSignals(False)
        self.traj_frame_label.setText(f"Frame: {frame + 1}/{total}")

    def set_playing_state(self, playing: bool):
        self.play_btn.setEnabled(not playing)
        self.pause_btn.setEnabled(playing)
        self.stop_btn.setEnabled(playing or self.traj_progress.value() > 0)

    def get_speed_multiplier(self) -> float:
        return self.speed_slider.value() / 5.0

    def is_loop_enabled(self) -> bool:
        return self.loop_checkbox.isChecked()

    def _on_export_current(self):
        self.export_current_requested.emit()

    def _on_export_trajectory(self):
        self.export_trajectory_requested.emit()

    def _on_compute_workspace(self):
        self.workspace_compute_requested.emit()

    def _on_workspace_toggled(self, checked):
        self.ws_toggle_btn.setText(f"Show Workspace: {'ON' if checked else 'OFF'}")
        self.workspace_toggle_requested.emit(checked)

    def set_workspace_info(self, bounds):
        if bounds is None:
            self.ws_info_label.setText("Not computed")
            self.ws_toggle_btn.setEnabled(False)
            self.ws_toggle_btn.setChecked(False)
            return
        self.ws_info_label.setText(
            f"Translation (mm):\n"
            f"  X: [{bounds['pos_x'][0]:.0f}, {bounds['pos_x'][1]:.0f}]\n"
            f"  Y: [{bounds['pos_y'][0]:.0f}, {bounds['pos_y'][1]:.0f}]\n"
            f"  Z: [{bounds['pos_z'][0]:.0f}, {bounds['pos_z'][1]:.0f}]\n"
            f"Rotation (deg):\n"
            f"  R: [{bounds['rot_x'][0]:.0f}, {bounds['rot_x'][1]:.0f}]\n"
            f"  P: [{bounds['rot_y'][0]:.0f}, {bounds['rot_y'][1]:.0f}]\n"
            f"  W: [{bounds['rot_z'][0]:.0f}, {bounds['rot_z'][1]:.0f}]"
        )
        self.ws_toggle_btn.setEnabled(True)
