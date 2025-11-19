from std_msgs.msg import String
import rclpy
from collections import deque
import statistics
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
        self.MAX_HISTORY = 5                # how many past readings to keep per beacon
        self.MIN_READINGS_TO_PUBLISH = 1    # require at least this many readings per beacon to consider it
        # ensure self.scan is a dict mapping beacon -> deque
        self.scan = {}                      # will hold beacon -> deque(maxlen=self.MAX_HISTORY)
        self.scan_counter = 0
        self.publisher_ = self.create_publisher(String, '/beacon_data', 10)
        self.subscriber_ = self.create_subscription(String, 'rf_signal', self.rf_callback, 10)

        self.scan_counter = 0
        self.scan = dict()

    def initBeacons(self):
        '''
        Initializes known beacon MAC-to-location mappings.
        '''
        self.beacons = loadBeacons()

    def rf_callback(self, msg: String):
        #works
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
        """
        Publish the strongest/most relevant beacon using a short history per beacon.
        Uses median of recent RSSI readings (smaller = better). Publishes the beacon with
        the lowest median RSSI. Keeps history across scans (with max length).
        """
        if not self.scan:
            # nothing to do
            return

        best_beacon = None
        best_score = None  # lower is better (assuming smaller RSSI value = stronger)

        for beacon, readings in list(self.scan.items()):
            if not isinstance(readings, deque):
                readings = deque(readings, maxlen=self.MAX_HISTORY)
                self.scan[beacon] = readings

            if len(readings) < self.MIN_READINGS_TO_PUBLISH:
                self.get_logger().debug(f"Skipping {beacon}: only {len(readings)} reading(s)")
                continue

            try:
                median_rssi = int(statistics.median(readings))
            except Exception as e:
                self.get_logger().warn(f"Failed to compute median for {beacon}: {e}")
                continue

            if best_score is None or median_rssi < best_score:
                best_score = median_rssi
                best_beacon = beacon

        if best_beacon is not None:
            msg = String()
            msg.data = f"{best_beacon},{best_score}"
            #self.get_logger().info(f"Publishing beacon data: {msg.data}")
            self.publisher_.publish(msg)
        else:
            self.get_logger().debug("No beacon qualified to publish this cycle")
def main():
    rclpy.init()
    node = BeaconSensor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
