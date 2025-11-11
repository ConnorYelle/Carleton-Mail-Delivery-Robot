from std_msgs.msg import String
import rclpy
from rclpy.node import Node

from tools.csv_parser import loadBeacons, loadConfig
#beacon sensor node
class BeaconSensor(Node):
    '''
    Node in charge of listening to beacon data published to /rf_signal
    in the format "Beacon=<mac>;SignalStrength=<rssi>"
    
    @Subscribers:
    - Subscribes to /rf_signal for simulated RSSI readings.

    @Publishers:
    - Publishes to /beacon_data with processed beacon data.
    '''
    def __init__(self):
        super().__init__('beacon_sensor')
        
        self.initBeacons()
        self.config = loadConfig()

        self.publisher_ = self.create_publisher(String, 'beacon_data', 10)
        self.subscriber_ = self.create_subscription(String, '/rf_signal', self.rf_callback, 10)

        self.scan_counter = 0
        self.scan = dict()

    def initBeacons(self):
        '''
        Initializes known beacon MAC-to-location mappings.
        '''
        self.beacons = loadBeacons()

    def rf_callback(self, msg: String):
        '''
        Callback for /rf_signal topic.
        Parses RSSI values from simulated beacon publisher.
        '''
        self.scan_counter += 1

        try:
            data = msg.data.strip()
            parts = dict(pair.split('=') for pair in data.split(';'))
            mac = parts["Beacon"].replace('_', ':')
            rssi = float(parts["SignalStrength"])

            if mac in self.beacons:
                key = self.beacons[mac]
                beacon_rssi = abs(int(rssi))

                if beacon_rssi < abs(self.config["BEACON_RSSI_THRESHOLD"]):
                    if key in self.scan:
                        self.scan[key].append(beacon_rssi)
                    else:
                        self.scan[key] = [beacon_rssi]
        except Exception as e:
            self.get_logger().warn(f"Failed to parse /rf_signal message: {msg.data} ({e})")

        if self.scan_counter >= self.config["BEACON_SCAN_COUNT"]:
            self.publish_strongest_beacon()

    def publish_strongest_beacon(self):
        '''
        Publishes the most relevant beacon based on recent scans.
        '''
        best_beacon = ""
        best_rssi = 100
        for beacon, readings in self.scan.items():
            if len(readings) < 2:
                continue
            if readings[-1] > readings[-2]:
                continue
            if readings[-1] < best_rssi:
                best_beacon = beacon
                best_rssi = readings[-1]

        if best_beacon:
            msg = String()
            msg.data = f"{best_beacon},{best_rssi}"
            self.publisher_.publish(msg)

        self.scan = dict()
        self.scan_counter = 0

def main():
    rclpy.init()
    node = BeaconSensor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
