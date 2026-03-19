import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TopicLogger(Node):
    def __init__(self):
        super().__init__("topic_logger")
        self.declare_parameter("log_path", "")
        self.log_path = self._resolve_log_path(self.get_parameter("log_path").value)
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self._log_file = open(self.log_path, "a", encoding="utf-8")

        self.create_subscription(String, "lidar_data", self._log_string("lidar_data"), 10)
        self.create_subscription(String, "actions", self._log_string("actions"), 10)
        self.create_subscription(String, "navigation", self._log_string("navigation"), 10)
        self.create_subscription(String, "beacon_data", self._log_string("beacon_data"), 10)
        self.create_subscription(String, "fake_beacon_data", self._log_string("fake_beacon_data"), 10)
        self.create_subscription(String, "destinations", self._log_string("destinations"), 10)
        self.create_subscription(Twist, "cmd_vel", self._log_twist, 10)
        self.create_subscription(Odometry, "odom", self._log_odom, 10)

        self.get_logger().info(f"TopicLogger writing to {self.log_path}")

    def _resolve_log_path(self, configured):
        configured = str(configured).strip()
        if configured:
            return os.path.abspath(configured)
        default_dir = os.getenv("DASHBOARD_LOG_DIR") or os.path.join(
            os.path.dirname(__file__), "..", "..", "tools", "logs"
        )
        return os.path.abspath(os.path.join(default_dir, "topic_debug.log"))

    def _write(self, topic, payload):
        timestamp = time.strftime("%H:%M:%S")
        self._log_file.write(f"[{timestamp}] {topic}: {payload}\n")
        self._log_file.flush()

    def _log_string(self, topic):
        def handler(msg: String):
            self._write(topic, msg.data)
        return handler

    def _log_twist(self, msg: Twist):
        self._write(
            "cmd_vel",
            f"linear=({msg.linear.x:.3f},{msg.linear.y:.3f},{msg.linear.z:.3f}) "
            f"angular=({msg.angular.x:.3f},{msg.angular.y:.3f},{msg.angular.z:.3f})"
        )

    def _log_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        self._write("odom", f"pos=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f})")


def main():
    rclpy.init()
    node = TopicLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
