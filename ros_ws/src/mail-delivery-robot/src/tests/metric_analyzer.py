import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from irobot_create_msgs.msg import DockStatus
from collections import deque
from pathlib import Path
import numpy as np
import csv

from tools.csv_parser import loadConfig

class MetricAnalyzer(Node):
    def __init__(self):
        super().__init__('metric_analyzer')

        # Load wall-follow config
        self.config = loadConfig()
        self.target_wall_distance = self.config.get("WALL_FOLLOW_SET_POINT", 0.8)

        # Internal tracking for metrics
        self.wall_distances = deque(maxlen=1000)
        self.last_camera_trigger_time = None
        self.intersection_delays = []

        self.awaiting_undock = False
        self.awaiting_dock = False
        self.dock_successes = 0
        self.dock_failures = 0
        self.path_corrections = 0
        self.is_docked = False

        # Subscribers
        self.create_subscription(String, 'lidar_data', self.lidar_callback, 10)
        self.create_subscription(Bool, 'camera_data', self.camera_callback, 10)
        self.create_subscription(String, 'intersection_detection', self.intersection_callback, 10)
        self.create_subscription(String, 'navigation', self.navigation_callback, 10)
        self.create_subscription(DockStatus, 'dock_status', self.dock_status_callback, 10)

        # Timer for periodic logging
        self.create_timer(1.0, self.log_metrics)

    def lidar_callback(self, msg):
        try:
            parts = list(map(float, msg.data.split(":")))
            if len(parts) == 5:
                self.wall_distances.append(parts[2])  # Right-side distance
        except ValueError:
            pass

    def camera_callback(self, msg):
        if msg.data:
            self.last_camera_trigger_time = self.get_clock().now()

    def intersection_callback(self, msg):
        if msg.data == "TRUE" and self.last_camera_trigger_time:
            delay = (self.get_clock().now() - self.last_camera_trigger_time).nanoseconds / 1e9
            self.intersection_delays.append(delay)
            self.last_camera_trigger_time = None

    def navigation_callback(self, msg):
        if msg.data == "U_TURN":
            self.path_corrections += 1
        elif msg.data == "UNDOCK":
            self.awaiting_undock = True
        elif msg.data == "DOCK":
            self.awaiting_dock = True

    def dock_status_callback(self, msg):
        self.is_docked = msg.is_docked
        if self.awaiting_undock and not msg.is_docked:
            self.awaiting_undock = False
        if self.awaiting_dock and msg.is_docked:
            self.dock_successes += 1
            self.awaiting_dock = False
        elif self.awaiting_dock and not msg.is_docked:
            self.dock_failures += 1
            self.awaiting_dock = False

    def log_metrics(self):
        deltas = [d - self.target_wall_distance for d in self.wall_distances]
        m1_variance = float(np.var(deltas)) if deltas else 0.0
        m2_avg_delay = float(np.mean(self.intersection_delays)) if self.intersection_delays else 0.0

        entry = {
            "timestamp": self.get_clock().now().to_msg().sec,
            "M1_wall_follow_variance": round(m1_variance, 4),
            "M2_avg_intersection_delay": round(m2_avg_delay, 3),
            "M4_dock_successes": self.dock_successes,
            "M4_dock_failures": self.dock_failures,
            "M5_path_corrections": self.path_corrections
        }

        log_path = Path.home() / 'robot_output' / 'robot_logs.csv'
        log_path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not log_path.exists()

        with open(log_path, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=entry.keys())
            if write_header:
                writer.writeheader()
            writer.writerow(entry)

        self.get_logger().info("Logged metrics snapshot to CSV.")

def main():
    rclpy.init()
    node = MetricAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
