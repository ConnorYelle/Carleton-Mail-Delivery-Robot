import os
import time
import re
from datetime import datetime, timedelta

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.msg import DockStatus

from jinja2 import Environment, FileSystemLoader


class GeneralLogger(Node):
    def __init__(self):
        super().__init__("general_logger")

        # Directory
        self.declare_parameter("log_dir", "./tools/logs")
        self.log_dir = self.get_parameter("log_dir").value
        os.makedirs(self.log_dir, exist_ok=True)

        # Start of trip
        self.trip_start_time = time.perf_counter()
        self.trip_start_timestamp = datetime.now()

        # Record battery at start
        self.battery_start = self.get_battery_data()["level"]
        self.battery_end = None
        self.battery_used = None

        self.get_logger().info(f"Battery at trip start: {self.battery_start:.2f}%")

        # Subscribe to DOCK STATUS (DockStatus)
        self.create_subscription(
            DockStatus,
            "/dock_status",
            self.dock_status_callback,
            10
        )

        # Wall following log file
        self.wall_log_path = os.path.join(self.log_dir, "robot_log_wallFollowing.txt")

    # function to read battery status during run
    def get_battery_data(self):
        """ Get one BatteryState message. """
        battery = {}

        def callback(msg):
            battery["level"] = msg.percentage * 100
            battery["voltage"] = msg.voltage
            battery["temperature"] = msg.temperature

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        sub = self.create_subscription(BatteryState, "/battery_state", callback, qos)

        start = self.get_clock().now()
        timeout = 2.0

        while "level" not in battery:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().warn("No battery msg. Using defaults.")
                battery = {"level": 0, "voltage": 0, "temperature": 0}
                break

        self.destroy_subscription(sub)
        return battery

    # Function to calculate the time the robot walls follows during the trip
    def get_wall_follow_time(self):
        if not os.path.exists(self.wall_log_path):
            return "N/A"

        with open(self.wall_log_path, "r") as f:
            for line in reversed(f.readlines()):
                match = re.search(r"Total wall-following time:\s*([\d.]+)s", line)
                if match:
                    return f"{match.group(1)} s"

        return "N/A"

    # function to stop logger once robot docks
    def dock_status_callback(self, msg: DockStatus):
        if not msg.is_docked:
            return

        self.get_logger().info("DOCKED detected → ending trip…")

        # Trip end
        self.trip_end_timestamp = datetime.now()

        # Delivery time in seconds
        delivery_time_sec = time.perf_counter() - self.trip_start_time

        # Convert seconds to HH:MM:SS
        delivery_time_str = str(timedelta(seconds=int(delivery_time_sec)))

        # Battery at end
        battery = self.get_battery_data()
        self.battery_end = battery["level"]
        self.battery_used = self.battery_start - self.battery_end

        self.get_logger().info(
            f"Battery Start: {self.battery_start:.2f}% | "
            f"End: {self.battery_end:.2f}% | "
            f"Used: {self.battery_used:.2f}%"
        )

        # Calculate times
        wall_time = self.get_wall_follow_time()
        delivery_time = time.perf_counter() - self.trip_start_time

        # Write final outputs
        self.write_run_file(battery, wall_time, delivery_time)
        self.generate_html_report(battery, wall_time, delivery_time)

        # Clear wall log
        open(self.wall_log_path, "w").close()

        self.get_logger().info("Logger completed. Shutting down ROS node.")
        rclpy.shutdown()

    # Function to generate a run file, then can compile all runs and
    # make a graph to find trends
    def write_run_file(self, battery, wall_time, delivery_time):
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filepath = os.path.join(self.log_dir, f"run_{timestamp}.txt")

        with open(filepath, "w") as f:
            f.write(f"battery_start={self.battery_start}\n")
            f.write(f"battery_end={self.battery_end}\n")
            f.write(f"battery_used={self.battery_used}\n\n")
            f.write(f"delivery_time={delivery_time:.2f}\n")
            f.write(f"wall_follow_time={wall_time}\n")
            f.write(f"voltage_level={battery['voltage']}\n")
            f.write(f"temperature_level={battery['temperature']}\n")

        self.get_logger().info(f"Run file saved: {filepath}")


    # Function to generate HTML report
    def generate_html_report(self, battery, wall_time, delivery_time, delivery_time_str):
        template_dir = os.path.dirname(os.path.realpath(__file__))
        env = Environment(loader=FileSystemLoader(template_dir))
        template = env.get_template("template.html")

        html = template.render(
            battery_level=battery["level"],
            voltage_level=battery["voltage"],
            temperature_level=battery["temperature"],
            wall_follow_time=wall_time,
            delivery_time=delivery_time_str,
            battery_start=self.battery_start,
            battery_end=self.battery_end,
            battery_used=self.battery_used,
            trip_start_time=self.trip_start_timestamp,
            trip_end_time=self.trip_end_timestamp
        )

        html_path = os.path.join(self.log_dir, "robot_report.html")
        with open(html_path, "w") as f:
            f.write(html)

        self.get_logger().info(f"HTML report generated: {html_path}")

def main(args=None):
    rclpy.init(args=args)
    node = GeneralLogger()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
