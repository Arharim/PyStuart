"""
Hardware tab: serial connection to the Bridge, servo state, torque.

Owns the ActuatorWriter (and a PoseSender when pose streaming is
enabled) for the lifetime of a connection. MainWindow feeds it manual
or cueing poses via send_pose() and calls poll_feedback() on a timer;
SRV_FB and PWR_MON frames update the per-axis table and power status.
Disconnecting always releases torque first.
"""
import math

import serial.tools.list_ports
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QFileDialog, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMessageBox, QPushButton, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from ..core.config import Geometry, load_geometry
from ..core.platform import StewartPlatform
from ..transport.actuator import ActuatorWriter, PoseSender
from ..transport.serial_link import SerialLink
from ..transport.servo_map import ServoCalibration, load_servo_calibration

AXIS_NAMES = ["alpha 0", "alpha 1", "alpha 2", "alpha 3", "alpha 4", "alpha 5"]


class HardwarePanel(QWidget):
    connection_changed = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._actuator = None
        self._sender = None
        self._build_ui()
        self._refresh_ports()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        connection_group = QGroupBox("Bridge Connection")
        connection_layout = QVBoxLayout(connection_group)

        port_row = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(170)
        port_row.addWidget(self.port_combo)
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_ports)
        port_row.addWidget(refresh_btn)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        port_row.addWidget(self.connect_btn)
        connection_layout.addLayout(port_row)

        config_row = QHBoxLayout()
        config_row.addWidget(QLabel("Rig config:"))
        self.config_path_edit = QLineEdit()
        self.config_path_edit.setPlaceholderText("optional YAML (geometry + servos)")
        config_row.addWidget(self.config_path_edit)
        config_browse = QPushButton("...")
        config_browse.setMaximumWidth(32)
        config_browse.clicked.connect(self._browse_config)
        config_row.addWidget(config_browse)
        connection_layout.addLayout(config_row)
        layout.addWidget(connection_group)

        torque_group = QGroupBox("Actuators")
        torque_layout = QVBoxLayout(torque_group)
        self.torque_off_btn = QPushButton("Torque OFF")
        self.torque_off_btn.setEnabled(False)
        self.torque_off_btn.clicked.connect(self._on_torque_off)
        torque_layout.addWidget(self.torque_off_btn)
        self.stream_check = QCheckBox("Stream pose targets (50 Hz)")
        self.stream_check.setEnabled(False)
        torque_layout.addWidget(self.stream_check)
        self.stats_label = QLabel("sent: - | skipped: -")
        torque_layout.addWidget(self.stats_label)
        layout.addWidget(torque_group)

        feedback_group = QGroupBox("Servo Feedback")
        feedback_layout = QVBoxLayout(feedback_group)
        self.servo_table = QTableWidget(6, 5)
        self.servo_table.setHorizontalHeaderLabels(
            ["Axis", "Target", "FB counts", "Load", "Temp C"])
        self.servo_table.verticalHeader().setVisible(False)
        for row, axis in enumerate(AXIS_NAMES):
            self.servo_table.setItem(row, 0, QTableWidgetItem(axis))
            for col in range(1, 5):
                self.servo_table.setItem(row, col, QTableWidgetItem("-"))
        self.servo_table.resizeColumnsToContents()
        feedback_layout.addWidget(self.servo_table)
        layout.addWidget(feedback_group)

        power_group = QGroupBox("Power (PWR_MON)")
        power_layout = QVBoxLayout(power_group)
        self.power_label = QLabel("no data")
        self.power_label.setWordWrap(True)
        power_layout.addWidget(self.power_label)
        layout.addWidget(power_group)

        layout.addStretch()

    def _refresh_ports(self):
        self.port_combo.clear()
        for port in serial.tools.list_ports.comports():
            self.port_combo.addItem("%s (%s)" % (port.device, port.description))

    def _browse_config(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Rig config", "", "YAML (*.yaml *.yml);;All (*)")
        if path:
            self.config_path_edit.setText(path)

    def _load_calibration_and_geometry(self):
        path = self.config_path_edit.text().strip()
        if not path:
            return ServoCalibration(), Geometry.desktop()
        return load_servo_calibration(path), load_geometry(path)

    def toggle_connection(self):
        if self._actuator is not None:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_combo.currentText().split(" ")[0]
        if not port:
            QMessageBox.warning(self, "Connect", "No serial port selected.")
            return
        try:
            calibration, geometry = self._load_calibration_and_geometry()
            link = SerialLink(port, baudrate=921600)
        except Exception as e:
            QMessageBox.warning(self, "Connect", "Failed to open %s:\n%s" % (port, e))
            return
        self._actuator = ActuatorWriter(link, calibration)
        self._sender = PoseSender(self._actuator, StewartPlatform(geometry), rate_hz=50.0)
        self.connect_btn.setText("Disconnect")
        self.torque_off_btn.setEnabled(True)
        self.stream_check.setEnabled(True)
        self.connection_changed.emit(True)

    def disconnect_serial(self):
        actuator, self._actuator, self._sender = self._actuator, None, None
        if actuator is None:
            return
        try:
            actuator.torque_off()
        finally:
            actuator.close()
        self.connect_btn.setText("Connect")
        self.torque_off_btn.setEnabled(False)
        self.stream_check.setEnabled(False)
        self.stream_check.setChecked(False)
        self.connection_changed.emit(False)

    def is_connected(self) -> bool:
        return self._actuator is not None

    def is_streaming(self) -> bool:
        return self._actuator is not None and self.stream_check.isChecked()

    def send_pose(self, translation_mm, rotation_rad) -> bool:
        """Route a pose to the servo bus; returns True when a frame went out."""
        if not self.is_streaming():
            return False
        sent = self._sender.send(translation_mm, rotation_rad)
        if sent:
            counts = self._actuator.last_counts
            for row, count in enumerate(counts):
                self.servo_table.item(row, 1).setText(str(count))
        self.stats_label.setText("sent: %d | skipped: %d"
                                 % (self._sender.sent, self._sender.skipped))
        return sent

    def _on_torque_off(self):
        if self._actuator is not None:
            self._actuator.torque_off()

    def poll_feedback(self):
        if self._actuator is None:
            return
        for kind, data in self._actuator.poll():
            if kind == "srv_fb":
                for row, (pos, load, temp) in enumerate(data):
                    self.servo_table.item(row, 2).setText(str(pos))
                    self.servo_table.item(row, 3).setText(str(load))
                    self.servo_table.item(row, 4).setText(str(temp))
            elif kind == "pwr_mon":
                v_mv, i_ma, relay, estop = data
                self.power_label.setText(
                    "bus: %.2f V, %.0f mA | relay: %s | e-stop: %s"
                    % (v_mv / 1000.0, i_ma,
                       "closed" if relay else "open",
                       "PRESSED" if estop else "ok"))

    def alpha_feedback_deg(self):
        """Latest FB counts interpreted back through calibration, degrees."""
        if self._actuator is None:
            return None
        fb = [self.servo_table.item(row, 2).text() for row in range(6)]
        if any(value == "-" for value in fb):
            return None
        alphas = self._actuator._calibration.counts_to_alpha([int(v) for v in fb])
        return [math.degrees(a) for a in alphas]
