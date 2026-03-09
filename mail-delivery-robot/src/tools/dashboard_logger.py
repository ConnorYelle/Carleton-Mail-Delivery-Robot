import os
import time
import re
from datetime import datetime
from typing import Any
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.msg import DockStatus
from std_msgs.msg import String
from rcl_interfaces.msg import Log
import math
from statistics import mean
from sensor_msgs.msg import LaserScan
from tools.csv_parser import loadConfig

def resolve_default_log_dir():
    env_log_dir = os.getenv("DASHBOARD_LOG_DIR")
    if env_log_dir:
        return os.path.abspath(env_log_dir)
    current_dir = os.path.abspath(os.path.dirname(__file__))
    for candidate in [current_dir] + [os.path.dirname(current_dir)]:
        current_dir = candidate
        while True:
            if os.path.basename(current_dir) == "mail-delivery-robot":
                logs_dir = os.path.join(current_dir, "tools", "logs")
                if os.path.isdir(logs_dir):
                    return logs_dir
            parent = os.path.dirname(current_dir)
            if parent == current_dir:
                break
            current_dir = parent
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "tools", "logs"))

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

class DockSuccessMetric(Metric):
    topic_name = '/dock_status'
    topic_type = DockStatus
    listen_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

    def __init__(self):
        self.dock_command_received = False
        self.is_docked = False
        self.success = False

    def update(self, msg: DockStatus):
        self.is_docked = msg.is_docked
        if self.dock_command_received and self.is_docked:
            self.success = True

    def on_navigation_msg(self, msg: String):
        if msg.data == 'DOCK':
            self.dock_command_received = True

    def serialize(self):
        return {
            "dock_attempted": self.dock_command_received,
            "dock_final_status": self.is_docked,
            "dock_success": self.success
        }

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

class LidarActivityMetric(Metric):
    topic_name = "/scan"
    topic_type = LaserScan
    listen_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

    def __init__(self):
        self.scan_messages = 0
        self.scan_changed_messages = 0
        self._last_signature = None

    def _signature(self, scan: LaserScan):
        # Use a sparse, rounded signature to detect meaningful scan changes.
        total = len(scan.ranges)
        if total == 0:
            return ()
        step = max(total // 32, 1)
        sample = []
        for i in range(0, total, step):
            val = scan.ranges[i]
            if val == math.inf:
                sample.append("inf")
            elif val <= 0.0:
                sample.append("nan")
            else:
                sample.append(round(float(val), 3))
            if len(sample) >= 32:
                break
        return tuple(sample)

    def update(self, scan: LaserScan):
        self.scan_messages += 1
        current = self._signature(scan)
        if self._last_signature is not None and current != self._last_signature:
            self.scan_changed_messages += 1
        self._last_signature = current

    def serialize(self):
        is_stale = self.scan_messages > 1 and self.scan_changed_messages == 0
        return {
            "scan_messages": self.scan_messages,
            "scan_changed_messages": self.scan_changed_messages,
            "lidar_stream_stale": is_stale,
        }

class LidarAIFallbackMetric(Metric):
    def __init__(self, fallback_log_path):
        self.fallback_log_path = fallback_log_path
        self.fallback_count = 0

    def end(self):
        if not os.path.exists(self.fallback_log_path):
            return

        with open(self.fallback_log_path, "r") as f:
            self.fallback_count = sum(
                1 for line in f if "TIMEOUT" in line
            )

    def serialize(self):
        return {
            "ai_fallback_count": self.fallback_count
        }

class RosoutLLMResponseTimeMetric(Metric):
    topic_name = "/rosout"
    topic_type = Log
    listen_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=200)

    def __init__(self):
        self.samples = []
        self.samples_by_node = {}
        self.latency_pattern = re.compile(r"latency=([\d.]+)s")

    def update(self, msg: Log):
        match = self.latency_pattern.search(msg.msg)
        if not match:
            return
        try:
            latency = float(match.group(1))
        except (TypeError, ValueError):
            return

        node_name = (msg.name or "ai_node").strip()
        self.samples.append(latency)
        if node_name not in self.samples_by_node:
            self.samples_by_node[node_name] = []
        self.samples_by_node[node_name].append(latency)

    def serialize(self):
        payload = {
            "ai_llm_response_avg_s": round(mean(self.samples), 3) if self.samples else 0.0,
            "ai_llm_response_max_s": round(max(self.samples), 3) if self.samples else 0.0,
            "ai_llm_response_count": len(self.samples),
        }
        for node_name, samples in self.samples_by_node.items():
            key_prefix = re.sub(r"[^a-zA-Z0-9_]+", "_", str(node_name)).strip("_").lower() or "ai_node"
            payload[f"{key_prefix}_llm_response_avg_s"] = round(mean(samples), 3)
            payload[f"{key_prefix}_llm_response_max_s"] = round(max(samples), 3)
            payload[f"{key_prefix}_llm_response_count"] = len(samples)
        return payload

def make_llm_response_time_metric(ai_node: Any) -> Metric:
    class LLMResponseTimeMetric(Metric):
        def __init__(self, node_ref: Any):
            self.ai_node = node_ref
            self.samples = []

        def end(self):
            if hasattr(self.ai_node, "get_llm_response_latencies"):
                latencies = self.ai_node.get_llm_response_latencies()
            else:
                latencies = getattr(self.ai_node, "llm_response_latencies", [])
            self.samples = [float(x) for x in latencies if isinstance(x, (int, float))]

        def serialize(self):
            node_name = self.ai_node.get_name() if hasattr(self.ai_node, "get_name") else "ai_node"
            key_prefix = re.sub(r"[^a-zA-Z0-9_]+", "_", str(node_name)).strip("_").lower() or "ai_node"
            avg_latency = round(mean(self.samples), 3) if self.samples else 0.0
            max_latency = round(max(self.samples), 3) if self.samples else 0.0
            return {
                f"{key_prefix}_llm_response_avg_s": avg_latency,
                f"{key_prefix}_llm_response_max_s": max_latency,
                f"{key_prefix}_llm_response_count": len(self.samples),
            }

    return LLMResponseTimeMetric(ai_node)

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
    def __init__(self, ai_nodes=None):
        super().__init__('dashboard_logger')
        self.declare_parameter('log_dir', resolve_default_log_dir())
        self.declare_parameter('max_trip_seconds', 180.0)
        log_dir = os.path.abspath(self.get_parameter('log_dir').value)
        self.logger = FileLogger(log_dir)
        self.max_trip_seconds = float(self.get_parameter('max_trip_seconds').value)
        self.run_start_perf = time.perf_counter()
        self.end_reason = "unknown"
        fallback_log_path = os.path.join(log_dir, "ai_fallback_log.txt")
        self.metrics = [
            BatteryMetric(),
            WallFollowMetric(self.logger.wall_log_path),
            DeliveryTimeMetric(),
            LidarDistanceMetric(),
            LidarActivityMetric(),
            DockSuccessMetric(),
            LidarAIFallbackMetric(fallback_log_path),
            RosoutLLMResponseTimeMetric(),
        ]
        if ai_nodes:
            for ai_node in ai_nodes:
                self.metrics.append(make_llm_response_time_metric(ai_node))
        for m in self.metrics:
            m.start()
            if m.topic_name:
                self.create_subscription(m.topic_type, m.topic_name,
                    lambda msg, metric=m: metric.update(msg), m.listen_qos)
        self.should_shutdown = False
        self.dock_metric = next((m for m in self.metrics if isinstance(m, DockSuccessMetric)), None)
        self.create_subscription(
            String,
            'navigation',
            self.dock_metric.on_navigation_msg if self.dock_metric else (lambda msg: None),
            10)

    def end_trip(self, reason: str = "unknown"):
        self.end_reason = reason
        data = {}
        for m in self.metrics:
            m.end()
            data.update(m.serialize())
        data["trip_end_reason"] = self.end_reason
        self.logger.write_run_file(data)
        self.logger.close()

def main(args=None):
    rclpy.init(args=args)
    node = RobotGeneralLogger()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            docked = any(isinstance(m, DockSuccessMetric) and m.is_docked for m in node.metrics)
            elapsed_s = time.perf_counter() - node.run_start_perf
            if docked:
                node.end_trip("docked")
                break
            if elapsed_s >= node.max_trip_seconds:
                node.end_trip("failed_to_dock")
                break
            if node.should_shutdown:
                node.end_trip("shutdown")
                break
    except KeyboardInterrupt:
        node.end_trip("keyboard_interrupt")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
