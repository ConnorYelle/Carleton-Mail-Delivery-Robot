import sys
import csv
from pathlib import Path
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QHBoxLayout, QFrame, QPushButton
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np

def load_metric_log():
    csv_path = Path.home() / 'robot_output' / 'robot_logs.csv'
    timestamps = []
    m1_var = []
    m2_delays = []
    m4_successes = []
    m4_failures = []
    m5_corrections = []

    if csv_path.exists():
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    timestamps.append(int(row["timestamp"]))
                    m1_var.append(float(row["M1_wall_follow_variance"]))
                    m2_delays.append(float(row["M2_avg_intersection_delay"]))
                    m4_successes.append(int(row["M4_dock_successes"]))
                    m4_failures.append(int(row["M4_dock_failures"]))
                    m5_corrections.append(int(row["M5_path_corrections"]))
                except (KeyError, ValueError):
                    continue

    return timestamps, m1_var, m2_delays, m4_successes, m4_failures, m5_corrections

class DashboardWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Test Dashboard")
        self.setGeometry(100, 100, 1000, 600)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        self.build_dashboard()

    def build_dashboard(self):
        # Clear existing layout if refreshing
        while self.layout.count():
            item = self.layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        timestamps, m1_var, m2_delays, m4_successes, m4_failures, m5_corrections = load_metric_log()

        if not m1_var:
            timestamps = [0]
            m1_var = [0.0]
            m2_delays = [0.0]
            m4_successes = [0]
            m4_failures = [0]
            m5_corrections = [0]

        # Summary Metrics
        summary_frame = QFrame()
        summary_layout = QHBoxLayout()
        summary_frame.setLayout(summary_layout)

        metrics = {
            "M1: Avg Wall Variance": f"{np.mean(m1_var):.3f}",
            "M2: Avg Intersection Delay (s)": f"{np.mean(m2_delays):.3f}",
            "M4: Dock Successes": str(m4_successes[-1]),
            "M4: Dock Failures": str(m4_failures[-1]),
            "M5: U-Turns": str(m5_corrections[-1])
        }

        for label, value in metrics.items():
            box = QLabel(f"{label}\n{value}")
            box.setFrameStyle(QFrame.Panel | QFrame.Raised)
            box.setLineWidth(2)
            box.setMargin(10)
            summary_layout.addWidget(box)

        self.layout.addWidget(summary_frame)

        # Chart
        chart_frame = QFrame()
        chart_layout = QHBoxLayout()
        chart_frame.setLayout(chart_layout)

        fig1, ax1 = plt.subplots()
        ax1.plot(timestamps, m1_var, marker='o', color='blue')
        ax1.set_title("M1 - Wall Follow Variance Over Time")
        ax1.set_xlabel("Timestamp (s)")
        ax1.set_ylabel("Variance (m²)")
        ax1.grid(True)
        canvas1 = FigureCanvas(fig1)
        chart_layout.addWidget(canvas1)

        self.layout.addWidget(chart_frame)

        # Reset Button
        reset_button = QPushButton("Reset for Next Run")
        reset_button.clicked.connect(self.reset_log)
        self.layout.addWidget(reset_button)

    def reset_log(self):
        csv_path = Path.home() / 'robot_output' / 'robot_logs.csv'
        csv_path.parent.mkdir(parents=True, exist_ok=True)

        headers = [
            "timestamp",
            "M1_wall_follow_variance",
            "M2_avg_intersection_delay",
            "M4_dock_successes",
            "M4_dock_failures",
            "M5_path_corrections"
        ]

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()

        self.build_dashboard()  # Refresh GUI after reset

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = DashboardWindow()
    win.show()
    sys.exit(app.exec_())
