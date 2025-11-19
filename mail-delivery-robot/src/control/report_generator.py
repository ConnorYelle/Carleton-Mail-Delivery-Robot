import os
import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from jinja2 import Environment, FileSystemLoader
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class ReportGenerator(Node):
    """
    Reads the logs from GeneralLogger and generates a single HTML report.
    No calculations or timers are done here.
    """

    def __init__(self, log_dir="./tools/logs", template_dir=None):
        super().__init__('report_generator')

        self.log_dir = log_dir

        # Battery info
        self.battery_level = 0.0
        self.voltage_level = 0.0
        self.temperature_level = 0.0

        # Subscribe to battery state to get latest info
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, qos)

        # Template setup
        if template_dir is None:
            template_dir = os.path.dirname(os.path.realpath(__file__))
        self.env = Environment(loader=FileSystemLoader(template_dir))
        try:
            self.template = self.env.get_template("template.html")
        except Exception as e:
            self.get_logger().error(f"Template not found in {template_dir}: {e}")
            raise e

    def battery_callback(self, msg):
        """Store latest battery values"""
        self.battery_level = msg.percentage * 100
        self.voltage_level = msg.voltage
        self.temperature_level = msg.temperature

    def read_wall_follow_time(self):
        """Read the last wall-following time from the wall log"""
        wall_log_path = os.path.join(self.log_dir, "robot_log_wallFollowing.txt")
        if not os.path.exists(wall_log_path):
            return "N/A"
        with open(wall_log_path, "r") as f:
            lines = f.readlines()
        for line in reversed(lines):
            match = re.search(r"Total wall-following time:\s*([\d.]+)s", line)
            if match:
                return f"{match.group(1)} s"
        return "N/A"

    def read_delivery_time(self):
        """Read the last trip/delivery time from the trip log"""
        trip_log_path = os.path.join(self.log_dir, "robot_log_tripTime.txt")
        if not os.path.exists(trip_log_path):
            return "N/A"
        with open(trip_log_path, "r") as f:
            lines = f.readlines()
        if lines:
            return lines[-1].strip()
        return "N/A"

    def generate_report(self):
        """Render HTML report using the latest battery info and logs"""
        wall_follow_time = self.read_wall_follow_time()
        delivery_time = self.read_delivery_time()

        html_content = self.template.render(
            battery_level=self.battery_level,
            voltage_level=self.voltage_level,
            temperature_level=self.temperature_level,
            wall_follow_time=wall_follow_time,
            delivery_time=delivery_time
        )

        output_path = os.path.join(self.log_dir, "robot_report.html")
        with open(output_path, "w") as f:
            f.write(html_content)

        self.get_logger().info(f"Report generated at {output_path}!")


def main(args=None):
    rclpy.init(args=args)
    node = ReportGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Generating final report...")

    # Generate the report once at shutdown
    node.generate_report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
