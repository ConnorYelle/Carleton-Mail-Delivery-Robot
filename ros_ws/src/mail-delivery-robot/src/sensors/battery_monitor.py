import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from tools.csv_parser import loadBatteryDockMapping
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class BatteryMonitor(Node):
    '''
    The battery monitor sensor node reroutes the robot to the closest dock
    when battery is low and resumes the original trip after recharging.

    @Subscribers:
    - Listens to /battery_status for battery percentage.
    - Listens to /beacon_data to track last known beacon location.
    - Listens to /destinations to track current trip destination.

    @Publishers:
    - Publishes new destinations to /destinations when redirection is needed.
    '''

    LOW_BATTERY_THRESHOLD = 15.0
    HIGH_BATTERY_THRESHOLD = 90.0

    def __init__(self):
        super().__init__('battery_monitor')

        self.beacon_to_dock = loadBatteryDockMapping()

        self.current_battery = None
        self.low_battery_triggered = False
        self.last_beacon = None
        self.current_destination = None
        self.saved_destination = None

        self.battery_sub = self.create_subscription(BatteryState, 'battery_state', self.battery_callback, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        ))
        self.beacon_sub = self.create_subscription(String, 'beacon_data', self.beacon_callback, 10)
        self.destination_sub = self.create_subscription(String, 'destinations', self.destination_callback, 10)

        self.destination_pub = self.create_publisher(String, 'destinations', 10)

    def beacon_callback(self, msg):
        self.last_beacon = msg.data.split(",")[0]  # Fixed since the beacon data is like beacon,rssi
        #self.get_logger().info(f"Last beacon is: {self.last_beacon}")

    def destination_callback(self, msg):
        try:
            self.current_destination = msg.data.split(":")[1] # Again fixed to match format of source:destination 
        except IndexError:
            self.current_destination = None

    def battery_callback(self, data):
        if data.percentage is None:
            self.get_logger().warn("ERROR: Battery percentage is None.")
            return

        self.current_battery = data.percentage * 100
        #self.get_logger().info(f"Battery level: {self.current_battery:.2f}%")

        if self.current_battery <= self.LOW_BATTERY_THRESHOLD and not self.low_battery_triggered:
            self.low_battery_triggered = True
            self.saved_destination = self.current_destination

            dock = self.closest_dock()
            if dock:
                self.publish_destination(dock)
                self.get_logger().info(f"Battery low, rerouting to: {dock}")
            else:
                self.get_logger().warn("No valid beacon, cannot find nearest dock.")

        elif self.low_battery_triggered and self.current_battery >= self.HIGH_BATTERY_THRESHOLD:
            if self.saved_destination:
                self.publish_destination(self.saved_destination)
                self.get_logger().info(f"Battery recharged, resuming delivery to: {self.saved_destination}")
                self.saved_destination = None
            self.low_battery_triggered = False

    def closest_dock(self):
        return self.beacon_to_dock[self.last_beacon]

    def publish_destination(self, destination):
        msg = String()
        msg.data = f"{self.last_beacon}:{destination}"  # source:destination format
        self.destination_pub.publish(msg)

def main():
    rclpy.init()
    node = BatteryMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
