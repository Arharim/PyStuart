from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QPushButton, QGroupBox, QFrame
)
from PyQt6.QtCore import Qt, pyqtSignal
import math


class ControlPanel(QWidget):
    pose_changed = pyqtSignal()
    reset_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.sliders = {}
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

        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        layout.addWidget(line)

        reset_btn = QPushButton("Reset")
        reset_btn.clicked.connect(self._on_reset)
        layout.addWidget(reset_btn)

        layout.addStretch()

        self.setMinimumWidth(250)

    def _create_slider_row(self, parent_layout, label_text, min_val, max_val, default_val):
        row_widget = QWidget()
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(0, 0, 0, 0)

        label = QLabel(label_text)
        label.setMinimumWidth(50)
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
