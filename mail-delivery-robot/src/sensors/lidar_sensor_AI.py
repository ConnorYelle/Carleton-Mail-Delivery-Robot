import math
import os
import threading
import json
import datetime
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from statistics import stdev
from ament_index_python.packages import get_package_share_directory

import ollama
from tools.csv_parser import loadConfig


class LidarSensor(Node):
    '''
    Node that listens to the lidar sensor and publishes processed data.
    Uses AI (LLM) with robot pause during query. Falls back to classical logic on failure.
    '''

    def __init__(self):
        super().__init__('lidar_sensor_AI')

        # Load config
        self.config = loadConfig()

        # Publishers
        self.publisher_ = self.create_publisher(String, 'lidar_data', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        # Sliding windows
        self.right_distances = []
        self.left_distances = []
        self.front_distances = []

        # AI query cooldown tracking
        self.last_ai_query_time = 0.0
        self.ai_cooldown_seconds = 5.0
        self.is_querying = False

        # Fallback logging
        pkg_share = get_package_share_directory('mail-delivery-robot')
        log_dir = os.path.join(pkg_share, 'logs')
        os.makedirs(log_dir, exist_ok=True)
        self.fallback_log_path = os.path.join(log_dir, 'ai_fallback_log.txt')
        
        self.get_logger().info("LidarSensor AI node started with 5s cooldown")

    # ---------------------------------------------------------
    # ROBOT CONTROL
    # ---------------------------------------------------------
    def stop_robot(self):
        """Publish zero velocity to stop the robot"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info("Robot stopped for AI query")

    # ---------------------------------------------------------
    # ROS CALLBACK
    # ---------------------------------------------------------
    def scan_callback(self, scan):
        current_time = time.time()
        time_since_last_query = current_time - self.last_ai_query_time

        # Check if we're in cooldown period or already querying
        if time_since_last_query < self.ai_cooldown_seconds or self.is_querying:
            # Use classical fallback during cooldown
            wf, angle, right, left, front = self.calculate(scan)
            source = "fallback (cooldown)"
        else:
            # Stop robot before AI query
            self.stop_robot()
            self.is_querying = True
            
            # Perform AI query
            wf, angle, right, left, front = self.calculate_ai(scan)
            
            # Update last query time
            self.last_ai_query_time = time.time()
            self.is_querying = False
            
            source = "ai" if self.used_ai else "fallback (error)"
            self.get_logger().info(f"AI query complete. Next query in {self.ai_cooldown_seconds}s")

        # Publish processed lidar data
        msg = String()
        msg.data = f"{wf}:{angle}:{right}:{left}:{front}"
        self.publisher_.publish(msg)

    # ---------------------------------------------------------
    # CLASSICAL FALLBACK METHOD
    # ---------------------------------------------------------
    def calculate(self, scan):
        count = len(scan.ranges)
        angle = 0

        min_left = self.config["LARGE_DEFAULT_DISTANCE"]
        min_right = self.config["LARGE_DEFAULT_DISTANCE"]
        min_front = self.config["LARGE_DEFAULT_DISTANCE"]
        min_distance = self.config["LARGE_DEFAULT_DISTANCE"]

        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            dist = scan.ranges[i]

            if dist == math.inf or dist <= 0.0:
                continue

            if (self.config["WALL_FOLLOW_MIN_ANGLE"]
                <= degree
                <= self.config["WALL_FOLLOW_MAX_ANGLE"]
                and dist < min_distance):
                min_distance = dist
                angle = degree

            if ((degree <= self.config["FRONT_MIN_ANGLE"]
                 or degree >= self.config["FRONT_MAX_ANGLE"])
                and dist < min_front):
                min_front = dist

            elif (self.config["RIGHT_MIN_ANGLE"]
                  <= degree
                  < self.config["RIGHT_MAX_ANGLE"]
                  and dist < min_right):
                min_right = dist

            elif (self.config["LEFT_MIN_ANGLE"]
                  < degree
                  <= self.config["LEFT_MAX_ANGLE"]
                  and dist < min_left):
                min_left = dist

        self.left_distances.append(min_left)
        self.right_distances.append(min_right)
        self.front_distances.append(min_front)

        if len(self.left_distances) <= self.config["LIDAR_STACK_LENGTH"]:
            return -1, -1, -1, -1, -1

        self.left_distances.pop(0)
        self.right_distances.pop(0)
        self.front_distances.pop(0)

        if (min_front >= self.config["LOST_WALL_FRONT_DISTANCE"]
            or stdev(self.front_distances) > self.config["LOST_WALL_FRONT_STDEV"]):
            min_front = -1

        if (min_right >= self.config["LOST_WALL_RIGHT_DISTANCE"]
            or stdev(self.right_distances) > self.config["LOST_WALL_RIGHT_STDEV"]):
            min_right = -1

        if (min_left >= self.config["LOST_WALL_LEFT_DISTANCE"]
            or stdev(self.left_distances) > self.config["LOST_WALL_LEFT_STDEV"]):
            min_left = -1

        return min_distance, angle - 90, min_right, min_left, min_front

    # ---------------------------------------------------------
    # AI QUERY METHOD
    # ---------------------------------------------------------
    def _run_ollama(self, prompt, result_holder):
        try:
            self.get_logger().info("Starting Ollama API call...")
            result_holder["response"] = ollama.chat(
                model='qwen3:0.6b',
                messages=[{'role': 'user', 'content': prompt}],
                format='json',
            )
            self.get_logger().info("Ollama API call completed")
        except Exception as e:
            self.get_logger().error(f"Ollama exception: {e}")
            result_holder["error"] = e

    def calculate_ai(self, scan):
        self.used_ai = False
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

            Task: Find the minimum distance in the following sectors.
            If a sector has no data, use {default_dist}.

            Sectors:
            1. Wall Follow: Angle between {self.config["WALL_FOLLOW_MIN_ANGLE"]} and {self.config["WALL_FOLLOW_MAX_ANGLE"]}.
            2. Front: Angle <= {self.config["FRONT_MIN_ANGLE"]} OR Angle >= {self.config["FRONT_MAX_ANGLE"]}.
            3. Right: Angle >= {self.config["RIGHT_MIN_ANGLE"]} and < {self.config["RIGHT_MAX_ANGLE"]}.
            4. Left: Angle > {self.config["LEFT_MIN_ANGLE"]} and <= {self.config["LEFT_MAX_ANGLE"]}.

            Return ONLY a JSON object with keys: wf_dist, wf_angle, right, left, front.
        """

        # Print the query to console
        self.get_logger().info("=" * 80)
        self.get_logger().info("AI QUERY:")
        self.get_logger().info(prompt)
        self.get_logger().info("=" * 80)

        result = {}
        thread = threading.Thread(
            target=self._run_ollama,
            args=(prompt, result),
            daemon=True
        )

        thread.start()
        thread.join()  # Wait indefinitely for AI response

        if "error" in result:
            self._log_fallback(f"ERROR: {result['error']}")
            return self.calculate(scan)

        if "response" not in result:
            self._log_fallback("NO RESPONSE from Ollama")
            return self.calculate(scan)

        try:
            # Print the response to console
            response_content = result["response"]["message"]["content"]
            self.get_logger().info("=" * 80)
            self.get_logger().info("AI RESPONSE:")
            self.get_logger().info(response_content)
            self.get_logger().info("=" * 80)

            content = json.loads(response_content)

            wf = float(content.get("wf_dist", default_dist))
            angle = float(content.get("wf_angle", 0.0))
            right = float(content.get("right", default_dist))
            left = float(content.get("left", default_dist))
            front = float(content.get("front", default_dist))

            self.used_ai = True
            return wf, angle - 90, right, left, front

        except Exception as e:
            self._log_fallback(f"PARSE_ERROR: {e}")
            return self.calculate(scan)

    # ---------------------------------------------------------
    # FALLBACK LOGGER
    # ---------------------------------------------------------
    def _log_fallback(self, reason):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.get_logger().warn(f"AI fallback: {reason}")
        with open(self.fallback_log_path, "a") as f:
            f.write(f"[{timestamp}] {reason}\n")


def main():
    rclpy.init()
    node = LidarSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
