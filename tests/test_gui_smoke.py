"""
GUI smoke tests.

Builds the real MainWindow without showing or executing it: tab
structure, wiring objects and clean shutdown are what's under test;
the Qt event loop itself is not.
"""
import pytest

pytest.importorskip("PyQt6.QtWidgets")


@pytest.fixture(scope="module")
def qapp():
    from PyQt6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app


def test_main_window_tabs(qapp):
    from pystuart.gui.main_window import MainWindow
    window = MainWindow()
    assert window.tabs.count() == 3
    assert [window.tabs.tabText(i) for i in range(3)] == ["Manual", "Cueing", "Hardware"]
    assert not window.hardware_panel.is_connected()
    assert not window._cueing_timer.isActive()
    window.close()


def test_hardware_panel_port_list(qapp):
    from pystuart.gui.hardware_panel import HardwarePanel
    panel = HardwarePanel()
    panel._refresh_ports()
    assert panel.port_combo.count() >= 0  # list builds without a device attached
    assert not panel.is_streaming()


def test_cueing_panel_settings(qapp):
    from pystuart.gui.cueing_panel import CueingPanel
    panel = CueingPanel()
    assert panel.is_ac_selected()
    assert panel.ac_port() == 9996
    panel.csv_path_edit.setText("rec.csv")
    panel.csv_radio.setChecked(True)
    assert panel.csv_path() == "rec.csv"
    config = panel.load_washout_config()
    assert config.dt == 0.005
    panel.ac_port_edit.setText("not-a-number")
    assert panel.ac_port() == 9996
