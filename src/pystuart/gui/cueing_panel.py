"""
Cueing tab: telemetry source selection, washout config, live readouts.

The panel only collects settings and displays state; MainWindow owns
the pipeline timer and the source/filter objects. Both AC UDP and CSV
replay feed the same washout chain that drives the 3D view and - when
the hardware tab is connected and streaming - the servo bus.
"""
import math

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QButtonGroup, QFileDialog, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QRadioButton, QVBoxLayout, QWidget,
)

from ..cueing.washout import WashoutConfig


class CueingPanel(QWidget):
    start_requested = pyqtSignal()
    stop_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        source_group = QGroupBox("Telemetry Source")
        source_layout = QVBoxLayout(source_group)

        self.ac_radio = QRadioButton("Assetto Corsa UDP")
        self.ac_radio.setChecked(True)
        source_layout.addWidget(self.ac_radio)

        ac_row = QHBoxLayout()
        ac_row.addWidget(QLabel("Host:"))
        self.ac_host_edit = QLineEdit("127.0.0.1")
        ac_row.addWidget(self.ac_host_edit)
        ac_row.addWidget(QLabel("Port:"))
        self.ac_port_edit = QLineEdit("9996")
        self.ac_port_edit.setMaximumWidth(80)
        ac_row.addWidget(self.ac_port_edit)
        source_layout.addLayout(ac_row)

        self.csv_radio = QRadioButton("CSV replay")
        source_layout.addWidget(self.csv_radio)

        csv_row = QHBoxLayout()
        self.csv_path_edit = QLineEdit()
        csv_row.addWidget(self.csv_path_edit)
        csv_browse = QPushButton("...")
        csv_browse.setMaximumWidth(32)
        csv_browse.clicked.connect(self._browse_csv)
        csv_row.addWidget(csv_browse)
        source_layout.addLayout(csv_row)

        self._source_group = QButtonGroup(self)
        self._source_group.addButton(self.ac_radio)
        self._source_group.addButton(self.csv_radio)

        config_row = QHBoxLayout()
        config_row.addWidget(QLabel("Rig config:"))
        self.config_path_edit = QLineEdit()
        self.config_path_edit.setPlaceholderText("optional YAML (washout section)")
        config_row.addWidget(self.config_path_edit)
        config_browse = QPushButton("...")
        config_browse.setMaximumWidth(32)
        config_browse.clicked.connect(self._browse_config)
        config_row.addWidget(config_browse)
        source_layout.addLayout(config_row)

        buttons_row = QHBoxLayout()
        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self.start_requested.emit)
        buttons_row.addWidget(self.start_btn)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_requested.emit)
        buttons_row.addWidget(self.stop_btn)
        source_layout.addLayout(buttons_row)

        self.status_label = QLabel("Idle")
        source_layout.addWidget(self.status_label)
        layout.addWidget(source_group)

        live_group = QGroupBox("Live")
        live_layout = QVBoxLayout(live_group)
        self.force_label = QLabel("force: -")
        self.rate_label = QLabel("rate: -")
        self.pose_label = QLabel("pose: -")
        for label in (self.force_label, self.rate_label, self.pose_label):
            label.setWordWrap(True)
            live_layout.addWidget(label)
        layout.addWidget(live_group)

        layout.addStretch()

    def _browse_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "Telemetry CSV", "", "CSV (*.csv);;All (*)")
        if path:
            self.csv_path_edit.setText(path)
            self.csv_radio.setChecked(True)

    def _browse_config(self):
        path, _ = QFileDialog.getOpenFileName(self, "Rig config", "", "YAML (*.yaml *.yml);;All (*)")
        if path:
            self.config_path_edit.setText(path)

    def is_ac_selected(self) -> bool:
        return self.ac_radio.isChecked()

    def ac_host(self) -> str:
        return self.ac_host_edit.text().strip() or "127.0.0.1"

    def ac_port(self) -> int:
        try:
            return int(self.ac_port_edit.text())
        except ValueError:
            return 9996

    def csv_path(self) -> str:
        return self.csv_path_edit.text().strip()

    def config_path(self):
        path = self.config_path_edit.text().strip()
        return path or None

    def load_washout_config(self) -> WashoutConfig:
        path = self.config_path()
        if path:
            from ..cueing.washout import load_washout
            return load_washout(path)
        return WashoutConfig(dt=0.005)

    def set_running(self, running: bool):
        self.start_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running)

    def set_status(self, text: str):
        self.status_label.setText(text)

    def update_live(self, sample, pose):
        f, r = sample.force, sample.rate
        self.force_label.setText("force: %+.2f %+.2f %+.2f m/s^2" % f)
        self.rate_label.setText("rate:  %+.3f %+.3f %+.3f rad/s" % r)
        t, rot = pose.translation_mm, pose.rotation_rad
        self.pose_label.setText(
            "pose: %+.1f %+.1f %+.1f mm | %+.1f %+.1f %+.1f deg"
            % (t[0], t[1], t[2],
               math.degrees(rot[0]), math.degrees(rot[1]), math.degrees(rot[2])))
