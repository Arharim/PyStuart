"""
GUI entry point.

Launches the PyQt6 simulator window:
    python -m pystuart
"""
import sys

from PyQt6.QtWidgets import QApplication

from ..gui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
