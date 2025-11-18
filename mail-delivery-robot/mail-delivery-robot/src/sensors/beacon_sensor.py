from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from bluepy.btle import Scanner, DefaultDelegate

from tools.csv_parser import loadBeacons, loadConfig


class ScanDelegate(DefaultDelegate):
    '''
    A default delegate for the scanner class.
    This enables handleNotification and handleDiscovery debugging logs
    '''

    def __init__(self):
        DefaultDelegate.__init__(self)


class BeaconSensor(Node):
    '''
    The Node in charge of listening to beacons.

    @Subscribers:
    - Uses the Scanner to scan for Bluetooth devices.

    @Publishers:
    - Publishes to /beacon_data with new beacon data.
    '''

    def __init__(self):
        super().__init__('beacon_sensor')

        self.initBeacons()

        # Load the global config.
        self.config = loadConfig()

        # Publisher
        self.publisher_ = self.create_publisher(String, 'beacon_data', 10)

        # Scanner
        self.scanner = Scanner().withDelegate(ScanDelegate())

        # Timer: run scan periodically
        self.timer = self.create_timer(
            self.config["BEACON_SCAN_TIMER"],
            self.checkForBeacons
        )

        self.scan_counter = 0
        self.scan = dict()

        self.get_logger().info("BeaconSensor node started.")

    def initBeacons(self):
        '''Initializes all the beacons and their values.'''
        self.beacons = loadBeacons()
        self.get_logger().info(f"Loaded beacons: {self.beacons}")

    def checkForBeacons(self):
        '''Scan for BLE devices and process beacons.'''

        # Perform scan
        devices = self.scanner.scan(self.config["BEACON_SCAN_DURATION"])
        self.get_logger().info(f"Devices found this scan: {len(devices)}")

        beaconData = String()
        self.scan_counter += 1

        # Log all nearby BLE devices (optional but useful)
        # for dev in devices:
        #   self.get_logger().info(f"BLE device detected: {dev.addr} RSSI={dev.rssi}")

        # Check if any device matches a known beacon
        for dev in devices:
            for beacon_mac in self.beacons.keys():
                if beacon_mac == dev.addr:

                    beacon_name = self.beacons[beacon_mac]
                    self.get_logger().info(
                        f"[MATCH] Beacon detected: {beacon_name} ({beacon_mac}), RSSI={dev.rssi}"
                    )

                    beacon_rssi = abs(int(dev.rssi))

                    # Apply RSSI threshold
                    if beacon_rssi < abs(self.config["BEACON_RSSI_THRESHOLD"]):

                        if beacon_name not in self.scan:
                            self.scan[beacon_name] = []

                        self.scan[beacon_name].append(beacon_rssi)

                    break

        # After enough scans, pick the best beacon
        if self.scan_counter >= self.config["BEACON_SCAN_COUNT"]:
            best_beacon = ""
            best_rssi = 100

            for beacon_name, rssi_list in self.scan.items():
                if len(rssi_list) < 2:
                    continue

                # Only consider if signal is improving (getting closer)
                if rssi_list[-1] > rssi_list[-2]:
                    continue

                # Lower RSSI → stronger signal
                if rssi_list[-1] < best_rssi:
                    best_beacon = beacon_name
                    best_rssi = rssi_list[-1]

            # Publish and log result
            if best_beacon != "":
                beaconData.data = f"{best_beacon},{best_rssi}"
                self.publisher_.publish(beaconData)

                self.get_logger().info(
                    f"[BEST] Selected beacon: {best_beacon} with RSSI={best_rssi}"
                )

            # Reset for next cycle
            self.scan = dict()
            self.scan_counter = 0


def main():
    rclpy.init()
    beacon_sensor = BeaconSensor()
    rclpy.spin(beacon_sensor)


if __name__ == '__main__':
    main()
