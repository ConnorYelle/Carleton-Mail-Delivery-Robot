import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from jinja2 import Environment, FileSystemLoader
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os

class ReportGenerator(Node):
    def __init__(self):
        super().__init__('report_generator')

        self.battery_level = 0.0
        self.voltage_level = 0.0
        self.temperature_level = 0.0
        

        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                depth = 10
        )
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, qos_profile)

        # Jinja2 setup
        self.template_dir = "/home/capstone2526/cmdr_ws/src/carleton-mail-delivery-robot"
        self.env = Environment(loader=FileSystemLoader(self.template_dir))
        self.template = self.env.get_template("template.html")

    def battery_callback(self, msg):
        # Update battery and generate report
        self.battery_level = msg.percentage * 100
        self.voltage_level = msg.voltage
        self.temperature_level = msg.temperature

        self.generate_report()

    def generate_report(self):
        # Render template with current battery level
        html_content = self.template.render(battery_level=self.battery_level, 
                                            voltage_level=self.voltage_level, 
                                            temperature_level=self.temperature_level)
        output_path = "/home/capstone2526/robot_report.html"
        with open(output_path, "w") as f:
            f.write(html_content)

        self.get_logger().info(f"Report generated at {output_path}!")

def main(args=None):
    rclpy.init(args=args)
    node = ReportGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
