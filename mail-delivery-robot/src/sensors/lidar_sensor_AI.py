import math
import os
import json
import datetime
from statistics import stdev

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import ollama

import ollama
from tools.csv_parser import loadConfig


class LidarSensor(Node):
    """
    Node responsible for processing LiDAR data.
    - Fast classical processing runs on every scan
    - AI reasoning runs periodically (rate-limited)
    - AI results override classical results when available
    """

    def __init__(self):
        super().__init__('lidar_sensor')

        # Load configuration
        self.config = loadConfig()

        # Publisher
        self.publisher_ = self.create_publisher(String, 'lidar_data', 10)

        # Subscriber
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        # AI timer (1 Hz)
        self.ai_timer = self.create_timer(1.0, self.ai_tick)

        # State
        self.latest_scan = None
        self.last_ai_result = None

        self.left_distances = []
        self.right_distances = []
        self.front_distances = []

        # Fallback log
        self.fallback_log_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '../../tools/logs/ai_fallback_log.txt'
        )
        os.makedirs(os.path.dirname(self.fallback_log_path), exist_ok=True)

        self.get_logger().info("LidarSensor node started (AI rate-limited)")

    def scan_callback(self, scan: LaserScan):
        """
        Runs on every LiDAR scan.
        NEVER blocks.
        """
        self.latest_scan = scan

        # Always compute classical logic
        dist, angle, right, left, front = self.calculate(scan)

        # Override with AI if available
        if self.last_ai_result is not None:
            dist, angle, right, left, front = self.last_ai_result

        msg = String()
        msg.data = f"{dist}:{angle}:{right}:{left}:{front}"
        self.publisher_.publish(msg)


    # AI TIMER CALLBACK
    def ai_tick(self):
        """
        Runs periodically (1 Hz).
        Can block without affecting real-time control.
        """
        if self.latest_scan is None:
            return

        try:
            self.last_ai_result = self.calculate_ai(self.latest_scan)
        except Exception as e:
            self.get_logger().warn(f"AI tick failed: {e}")
            self.last_ai_result = None

    # CLASSICAL LIDAR LOGIC
    def calculate(self, scan: LaserScan):
        count = len(scan.ranges)
        angle = 0

        min_left = self.config["LARGE_DEFAULT_DISTANCE"]
        min_right = self.config["LARGE_DEFAULT_DISTANCE"]
        min_front = self.config["LARGE_DEFAULT_DISTANCE"]
        min_distance = self.config["LARGE_DEFAULT_DISTANCE"]

        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            dist = scan.ranges[i]

            if dist == math.inf:
                continue

            if (
                self.config["WALL_FOLLOW_MIN_ANGLE"]
                <= degree
                <= self.config["WALL_FOLLOW_MAX_ANGLE"]
                and dist < min_distance
            ):
                min_distance = dist
                angle = degree

            if degree <= self.config["FRONT_MIN_ANGLE"] or degree >= self.config["FRONT_MAX_ANGLE"]:
                min_front = min(min_front, dist)
            elif self.config["RIGHT_MIN_ANGLE"] <= degree < self.config["RIGHT_MAX_ANGLE"]:
                min_right = min(min_right, dist)
            elif self.config["LEFT_MIN_ANGLE"] < degree <= self.config["LEFT_MAX_ANGLE"]:
                min_left = min(min_left, dist)

        self.left_distances.append(min_left)
        self.right_distances.append(min_right)
        self.front_distances.append(min_front)

        if len(self.left_distances) <= self.config["LIDAR_STACK_LENGTH"]:
            return -1, -1, -1, -1, -1

        self.left_distances.pop(0)
        self.right_distances.pop(0)
        self.front_distances.pop(0)

        if (
            min_front >= self.config["LOST_WALL_FRONT_DISTANCE"]
            or stdev(self.front_distances) > self.config["LOST_WALL_FRONT_STDEV"]
        ):
            min_front = -1

        if (
            min_right >= self.config["LOST_WALL_RIGHT_DISTANCE"]
            or stdev(self.right_distances) > self.config["LOST_WALL_RIGHT_STDEV"]
        ):
            min_right = -1

        if (
            min_left >= self.config["LOST_WALL_LEFT_DISTANCE"]
            or stdev(self.left_distances) > self.config["LOST_WALL_LEFT_STDEV"]
        ):
            min_left = -1

        return min_distance, angle - 90, min_right, min_left, min_front


    # AI-BASED LIDAR LOGIC
    def calculate_ai(self, scan: LaserScan):
        scan_pairs = []
        step = 5
        default_dist = self.config.get("LARGE_DEFAULT_DISTANCE", 10.0)

        for i in range(0, len(scan.ranges), step):
            dist = scan.ranges[i]
            if dist == math.inf or dist == 0.0:
                continue

            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            scan_pairs.append(f"{degree:.1f}:{dist:.2f}")

        data_str = ", ".join(scan_pairs)

        prompt = f"""
            Analyze these Lidar readings (format "angle:distance").
            Data: [{data_str}]

            Find minimum distances in these sectors.
            If a sector has no data, use {default_dist}.

            Return ONLY JSON with keys:
            wf_dist, wf_angle, right, left, front.

            Angles:
            Wall Follow: {self.config["WALL_FOLLOW_MIN_ANGLE"]} to {self.config["WALL_FOLLOW_MAX_ANGLE"]}
            Front: <= {self.config["FRONT_MIN_ANGLE"]} OR >= {self.config["FRONT_MAX_ANGLE"]}
            Right: {self.config["RIGHT_MIN_ANGLE"]} to {self.config["RIGHT_MAX_ANGLE"]}
            Left: {self.config["LEFT_MIN_ANGLE"]} to {self.config["LEFT_MAX_ANGLE"]}
            """

        try:
            response = ollama.chat(
                model='gemma2:2b-instruct-q4_0',
                messages=[{'role': 'user', 'content': prompt}],
                format='json'
            )

            content = json.loads(response['message']['content'])

            return (
                float(content.get('wf_dist', default_dist)),
                float(content.get('wf_angle', 0.0)) - 90,
                float(content.get('right', default_dist)),
                float(content.get('left', default_dist)),
                float(content.get('front', default_dist)),
            )

        except Exception as e:
            self.get_logger().warn(f"AI calculation failed: {e}")

            with open(self.fallback_log_path, "a") as f:
                timestamp = datetime.datetime.now().strftime("%H:%M:%S")
                f.write(f"[{timestamp}] FALLBACK\n")

        if thread.is_alive():
            self._log_fallback("TIMEOUT")
            return self.calculate(scan)


def main():
    rclpy.init()
    node = LidarSensor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
