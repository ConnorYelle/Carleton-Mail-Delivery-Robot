import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class MotionProbe(Node):
    """
    Lightweight probe that logs whether the robot is moving.
    """

    def __init__(self):
        super().__init__("motion_probe")

        self.declare_parameter("min_distance_m", 0.05)
        self.declare_parameter("min_cmd_vel", 0.01)
        self.declare_parameter("log_interval_s", 2.0)

        self.min_distance_m = float(self.get_parameter("min_distance_m").value)
        self.min_cmd_vel = float(self.get_parameter("min_cmd_vel").value)
        self.log_interval_s = float(self.get_parameter("log_interval_s").value)

        self.last_pos = None
        self.last_cmd_vel = None
        self.last_cmd_time = None
        self.moved_since_last_log = False

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_timer(self.log_interval_s, self.log_status)

        self.get_logger().info(
            f"MotionProbe ready (min_distance_m={self.min_distance_m}, "
            f"min_cmd_vel={self.min_cmd_vel}, log_interval_s={self.log_interval_s})"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        if self.last_pos is None:
            self.last_pos = (pos.x, pos.y)
            return

        dx = pos.x - self.last_pos[0]
        dy = pos.y - self.last_pos[1]
        dist = math.hypot(dx, dy)
        if dist >= self.min_distance_m:
            self.moved_since_last_log = True
            self.last_pos = (pos.x, pos.y)

    def log_status(self):
        cmd_active = False
        cmd_linear = 0.0
        cmd_angular = 0.0
        if self.last_cmd_vel is not None:
            cmd_linear = float(self.last_cmd_vel.linear.x)
            cmd_angular = float(self.last_cmd_vel.angular.z)
            cmd_active = abs(cmd_linear) >= self.min_cmd_vel or abs(cmd_angular) >= self.min_cmd_vel

        if self.moved_since_last_log:
            self.get_logger().info(
                f"Robot movement detected. cmd_vel=({cmd_linear:.3f}, {cmd_angular:.3f})"
            )
        else:
            self.get_logger().warn(
                f"No movement detected. cmd_vel=({cmd_linear:.3f}, {cmd_angular:.3f})"
            )

        self.moved_since_last_log = False


def main():
    rclpy.init()
    node = MotionProbe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
