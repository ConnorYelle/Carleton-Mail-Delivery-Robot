import os
import time
import re
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.msg import DockStatus
import math
from statistics import mean
from sensor_msgs.msg import LaserScan
from tools.csv_parser import loadConfig

class Metric:
    topic_name = None
    topic_type = None
    listen_qos = 10
    def start(self): pass
    def update(self, msg): pass
    def end(self): pass
    def serialize(self): return {}

class BatteryMetric(Metric):
    topic_name = '/battery_state'
    topic_type = BatteryState
    listen_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
    def __init__(self):
        self.level = None
        self.voltage = None
        self.temperature = None
        self.start_level = None
        self.end_level = None
        self.used = None
    def update(self, msg: BatteryState):
        self.level = msg.percentage * 100
        self.voltage = msg.voltage
        self.temperature = msg.temperature
        if self.start_level is None: self.start_level = self.level
    def end(self):
        self.end_level = self.level
        if self.start_level is not None and self.end_level is not None:
            self.used = self.start_level - self.end_level
    def serialize(self):
        return {
            "battery_start": round(self.start_level, 2) if self.start_level else 0.0,
            "battery_end": round(self.end_level, 2) if self.end_level else 0.0,
            "battery_used": round(self.used, 2) if self.used is not None else 0.0,
            "voltage_level": round(self.voltage, 2) if self.voltage else 0.0,
            "temperature_level": round(self.temperature, 2) if self.temperature else 0.0
        }

class WallFollowMetric(Metric):
    def __init__(self, log_path):
        self.log_path = log_path
        self.wall_time = 0.0
    def end(self):
        if not os.path.exists(self.log_path): return
        with open(self.log_path, 'r') as f:
            for line in reversed(f.readlines()):
                match = re.search(r"Total wall-following time:\s*([\d.]+)s", line)
                if match:
                    self.wall_time = float(match.group(1))
                    break
    def serialize(self): return {"wall_follow_time": self.wall_time}

class DeliveryTimeMetric(Metric):
    def __init__(self):
        self.start_time = None
        self.start_timestamp = None
        self.end_timestamp = None
        self.elapsed = 0.0
    def start(self):
        self.start_time = time.perf_counter()
        self.start_timestamp = datetime.now()
    def end(self):
        self.end_timestamp = datetime.now()
        self.elapsed = round(time.perf_counter() - self.start_time, 2)
    def serialize(self):
        return {
            "delivery_time": self.elapsed,
            "trip_start_time": self.start_timestamp,
            "trip_end_time": self.end_timestamp
        }

class Dock(Metric):
    topic_name = '/dock_status'
    topic_type = DockStatus
    listen_qos = 10
    def __init__(self): self.docked = False
    def update(self, msg: DockStatus): self.docked = msg.is_docked
    def serialize(self): return {"docked": self.docked}

class LidarDistanceMetric(Metric):
    topic_name = "/scan"
    topic_type = LaserScan
    listen_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
    def __init__(self):
        self.config = loadConfig()
        self.front_distances = []
        self.wall_distances = []
    def update(self, scan: LaserScan):
        count = len(scan.ranges)
        min_front = self.config["LARGE_DEFAULT_DISTANCE"]
        min_wall = self.config["LARGE_DEFAULT_DISTANCE"]
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            cur = scan.ranges[i]
            if cur == math.inf or cur <= 0.0: continue
            if (degree <= self.config["FRONT_MIN_ANGLE"] or degree >= self.config["FRONT_MAX_ANGLE"]) and cur < min_front:
                min_front = cur
            if (self.config["WALL_FOLLOW_MIN_ANGLE"] <= degree <= self.config["WALL_FOLLOW_MAX_ANGLE"]) and cur < min_wall:
                min_wall = cur
        if min_front < self.config["LARGE_DEFAULT_DISTANCE"]: self.front_distances.append(min_front)
        if min_wall < self.config["LARGE_DEFAULT_DISTANCE"]: self.wall_distances.append(min_wall)
    def serialize(self):
        avg_f = round(mean(self.front_distances), 2) if self.front_distances else 0.0
        avg_w = round(mean(self.wall_distances), 2) if self.wall_distances else 0.0
        return {"lidar_front_avg": avg_f, "wall_distance_avg": avg_w}

class FileLogger:
    def __init__(self, log_dir):
        self.log_dir = log_dir
        self.runs_dir = os.path.join(self.log_dir, "runs")
        os.makedirs(self.runs_dir, exist_ok=True)
        self.wall_log_path = os.path.join(self.log_dir, "robot_log_wallFollowing.txt")
        self.wall_log_file = open(self.wall_log_path, "a")
    def write_log(self, tag, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.wall_log_file.write(f"[{timestamp}] [{tag}] {message}\n")
        self.wall_log_file.flush()
    def write_run_file(self, data):
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filepath = os.path.join(self.runs_dir, f"run_{timestamp}.txt")
        with open(filepath, "w") as f:
            for key, value in data.items():
                f.write(f"{key}={value}\n")
    def close(self):
        self.wall_log_file.close()
        open(self.wall_log_path, 'w').close()

class RobotGeneralLogger(Node):
    def __init__(self):
        super().__init__('dashboard_logger')
        self.declare_parameter('log_dir', './tools/logs')
        log_dir = os.path.abspath(self.get_parameter('log_dir').value)
        self.logger = FileLogger(log_dir)
        self.metrics = [
            BatteryMetric(),
            WallFollowMetric(self.logger.wall_log_path),
            DeliveryTimeMetric(),
            LidarDistanceMetric(),
            Dock()
        ]
        for m in self.metrics:
            m.start()
            if m.topic_name:
                self.create_subscription(m.topic_type, m.topic_name,
                    lambda msg, metric=m: metric.update(msg), m.listen_qos)
        self.should_shutdown = False
    def end_trip(self):
        data = {}
        for m in self.metrics:
            m.end()
            data.update(m.serialize())
        self.logger.write_run_file(data)
        self.logger.close()

def main(args=None):
    rclpy.init(args=args)
    node = RobotGeneralLogger()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            docked = any(isinstance(m, Dock) and m.docked for m in node.metrics)
            if docked or node.should_shutdown:
                node.end_trip()
                break
    except KeyboardInterrupt:
        node.end_trip()
    node.destroy_node()
    rclpy.shAutdown()

if __name__ == "__main__":
    main()