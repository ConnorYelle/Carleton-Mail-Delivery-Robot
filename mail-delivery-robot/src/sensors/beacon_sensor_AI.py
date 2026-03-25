import json
import threading
import time

import ollama
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from bluepy.btle import Scanner, DefaultDelegate

from tools.csv_parser import loadBeacons, loadConfig

ollama_client = ollama.Client(timeout=60)

class ScanDelegate(DefaultDelegate):

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
        self.declare_parameter("use_fake_beacon_data", True)
        self.use_fake_beacon_data = bool(self.get_parameter("use_fake_beacon_data").value)

        # Publisher
        self.publisher_ = self.create_publisher(String, 'beacon_data', 10)

        self.audio_publisher = self.create_publisher(String, 'llm_audio_event', 10)

        # Scanner
        self.scanner = Scanner().withDelegate(ScanDelegate())

        # Timer: run scan periodically
        self.timer = self.create_timer(
            self.config["BEACON_SCAN_TIMER"],
            self.checkForBeacons
        )

        # Optional simulated data input
        self.fake_beacon_sub = self.create_subscription(
            String, 'fake_beacon_data', self.fake_beacon_callback, 10
        )

        self.scan_counter = 0
        self.scan = dict()

        self.get_logger().info("BeaconSensor node started.")
        self.get_logger().info(f"BEACON_SCAN_COUNT = {self.config['BEACON_SCAN_COUNT']}")
        self.get_logger().info(f"use_fake_beacon_data = {self.use_fake_beacon_data}")
        self.llm_response_latencies = []

    def initBeacons(self):
        '''Initializes all the beacons and their values.'''
        self.beacons = loadBeacons()
        allowed_names = {"Nicol", "Canal"}
        self.beacons = {
            mac: name for mac, name in self.beacons.items() if name in allowed_names
        }
        self.get_logger().info(f"Loaded beacons: {self.beacons}")

    def _finalize_scan(self):
        # After enough scans, pick the best beacon
        self.get_logger().info(">>> REACHED AI BLOCK IN checkForBeacons() <<<")
        best_beacon = self.pick_beacon_ai()

        if best_beacon is None or best_beacon not in self.scan:
            self.get_logger().info("AI did not return a valid beacon, falling back to traditional method.")
            best_beacon = ""
            best_rssi = 100

            for beacon_name, rssi_list in self.scan.items():
                self.get_logger().info(
                    f"[SCAN RESULT] Beacon: {beacon_name}, RSSI readings: {rssi_list}"
                )

                # Use the most recent RSSI reading
                last_rssi = rssi_list[-1]

                # Lower RSSI -> stronger signal
                if last_rssi < best_rssi:
                    best_beacon = beacon_name
                    best_rssi = last_rssi

        else:
            best_rssi = self.scan[best_beacon][-1]
            self.get_logger().info(f"[AI RESULT] Selected beacon: {best_beacon} with RSSI={best_rssi}")

        # Publish and log result
        if best_beacon != "":
            beaconData = String()
            beaconData.data = f"{best_beacon},{best_rssi}"
            self.publisher_.publish(beaconData)

        # Reset for next cycle
        self.scan = dict()
        self.scan_counter = 0

    def fake_beacon_callback(self, msg: String):
        if not self.use_fake_beacon_data:
            return
        parts = msg.data.split(",")
        if len(parts) < 2:
            return
        beacon_name = parts[0].strip()
        try:
            beacon_rssi = abs(int(float(parts[1].strip())))
        except Exception:
            return
        if beacon_name not in self.scan:
            self.scan[beacon_name] = []
        self.scan[beacon_name].append(beacon_rssi)
        self.scan_counter += 1
        if self.scan_counter >= self.config["BEACON_SCAN_COUNT"]:
            self._finalize_scan()

    def checkForBeacons(self):
        self.get_logger().info(f"Scan count: {self.scan_counter} / {self.config['BEACON_SCAN_COUNT']}")

        if self.use_fake_beacon_data:
            return

        try:
            devices = self.scanner.scan(self.config["BEACON_SCAN_DURATION"])
        except Exception as e:
            self.get_logger().error(f"Bluetooth scan failed: {e}")
            return

        self.scan_counter += 1

        # Check if any device matches a known beacon
        for dev in devices:
            for beacon_mac in self.beacons.keys():
                if beacon_mac == dev.addr:

                    beacon_name = self.beacons[beacon_mac]

                    beacon_rssi = abs(int(dev.rssi))

                    # Apply RSSI threshold
                    if beacon_rssi < abs(self.config["BEACON_RSSI_THRESHOLD"]):

                        if beacon_name not in self.scan:
                            self.scan[beacon_name] = []

                        self.scan[beacon_name].append(beacon_rssi)
                    break

        # After enough scans, pick the best beacon
        if self.scan_counter >= self.config["BEACON_SCAN_COUNT"]:
            self._finalize_scan()

    def _run_ollama(self, prompt, result):
        try:
            result["response"] = ollama_client.chat(
                model='qwen2:0.5b',
                messages=[{'role': 'user', 'content': prompt}],
                format='json',
            )
        except Exception as e:
            result["error"] = e

    def pick_beacon_ai(self):
        self.get_logger().info(">>> pick_beacon_ai CALLED <<<")
        start = time.perf_counter()
        prompt = "Choose the best (strongest) beacon based on RSSI.\n" \
        "Lower RSSI values indicate stronger signals.\n" \
        "Beacon data:\n"

        for name, rssi_list in self.scan.items():
            prompt += f"- {name}: RSSI readings: {rssi_list}\n"
        
        prompt += """Return only a JSON object with the following structure: {"best_beacon": "BEACON_NAME"}"""

        result = {}
        thread = threading.Thread(target=self._run_ollama, args=(prompt, result), daemon=True)

        thread.start()
        self.get_logger().info("Waiting for Ollama response...")
        thread.join(timeout=20.0)

        if thread.is_alive():
            elapsed = time.perf_counter() - start
            self.record_llm_latency(elapsed, context="pick_beacon_ai_timeout")
            self.get_logger().warning("NO RESPONSE: Ollama response timed out.")
            return None
        
        if "error" in result:
            elapsed = time.perf_counter() - start
            self.record_llm_latency(elapsed, context="pick_beacon_ai_error")
            self.get_logger().error(f"OLLAMA ERROR: {result['error']}")
            return None
        
        try:
            elapsed = time.perf_counter() - start
            self.record_llm_latency(elapsed, context="pick_beacon_ai")
            content = json.loads(result["response"]["message"]["content"])
            return content.get("best_beacon", None)
        except Exception as e:
            elapsed = time.perf_counter() - start
            self.record_llm_latency(elapsed, context="pick_beacon_ai_parse_error")
            self.get_logger().error(f"AI_ERROR: Error parsing Ollama response: {e}")
            return None

    def record_llm_latency(self, elapsed_s: float, context: str = "llm_call"):
        self.llm_response_latencies.append(elapsed_s)
        # Keep `latency=<number>s` format so dashboard_logger can parse from /rosout.
        self.get_logger().info(f"{context}: latency={elapsed_s:.3f}s")

    def get_llm_response_latencies(self):
        return list(self.llm_response_latencies)

def main():
    rclpy.init()
    beacon_sensor = BeaconSensor()
    rclpy.spin(beacon_sensor)


if __name__ == '__main__':
    main()
