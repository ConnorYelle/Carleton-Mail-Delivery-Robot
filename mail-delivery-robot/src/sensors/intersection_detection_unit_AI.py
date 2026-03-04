import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
import json
import threading
import ollama

from tools.csv_parser import loadConfig

ollama_client = ollama.Client(timeout=60)


class IntersectionDetectionUnit(Node):
    '''
    The Node in charge of intersection detection.
    Uses Ollama LLM to decide if an intersection is detected.
    '''

    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('intersection_detection_unit')

        self.config = loadConfig()

        self.lidar_data_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)
        self.odometry_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        ))

        self.intersection_detection_publisher = self.create_publisher(String, 'intersection_detection', 10)
        self.intersection_detection_timer = self.create_timer(0.5, self.update_intersection_detection)

        self.true_msg = String()
        self.true_msg.data = 'TRUE'
        self.false_msg = String()
        self.false_msg.data = 'FALSE'

        self.lidar_indicates_intersection = False
        self.odom_x_snapshot = None
        self.odom_y_snapshot = None
        self.odom_x = None
        self.odom_y = None

        self.get_logger().info("IntersectionDetectionUnit with AI started.")

    def lidar_data_callback(self, data):
        lidar_data = str(data.data)
        self.lidar_data_raw = lidar_data

    def odometry_callback(self, data):
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y

    def _run_ollama(self, prompt, result):
        try:
            result["response"] = ollama_client.chat(
                model='gemma2:2b-instruct-q4_0',
                messages=[{'role': 'user', 'content': prompt}],
                format='json',
            )
        except Exception as e:
            result["error"] = e


    def detect_intersection_ai(self):
        if self.lidar_data_raw is None:
            return None

        prompt = f"""
You are a robotics perception module.

Determine whether the robot is at an intersection.

Lidar data format:
front:left:right:rear_left:rear_right

Data:
{self.lidar_data_raw}

Rules:
- A value of -1 means open space (no wall detected).
- If at least two side directions are open, it is likely an intersection.

Return ONLY a JSON object:
{{"intersection": true}}
or
{{"intersection": false}}
"""

        result = {}
        thread = threading.Thread(
            target=self._run_ollama,
            args=(prompt, result),
            daemon=True
        )

        thread.start()
        thread.join(timeout=20.0)

        if thread.is_alive():
            self.get_logger().warning("Ollama response timed out.")
            return None

        if "error" in result:
            self.get_logger().error(f"Ollama error: {result['error']}")
            return None

        try:
            content = json.loads(result["response"]["message"]["content"])
            return content.get("intersection", None)
        except Exception as e:
            self.get_logger().error(f"Error parsing Ollama response: {e}")
            return None


    def update_intersection_detection(self):
        #AI
        ai_result = self.detect_intersection_ai()

        #Fallback
        if ai_result is None and self.lidar_data_raw is not None:

            split_data = self.lidar_data_raw.split(":")

            if len(split_data) >= 5:
                if ((split_data[2] == "-1" and split_data[3] == "-1")
                        or (split_data[2] == "-1" and split_data[4] == "-1")
                        or (split_data[3] == "-1" and split_data[4] == "-1")):
                    ai_result = True
                else:
                    ai_result = False
            else:
                ai_result = False

        # Publish result
        if ai_result:
            self.get_logger().info("[AI RESULT] Intersection detected.")
            self.intersection_detection_publisher.publish(self.true_msg)
        else:
            self.intersection_detection_publisher.publish(self.false_msg)


def main():
    rclpy.init()
    node = IntersectionDetectionUnit()
    rclpy.spin(node)


if __name__ == '__main__':
    main()