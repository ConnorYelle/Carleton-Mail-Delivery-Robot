import math
import os
import xml.etree.ElementTree as ET
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from tools.nav_parser import loadConnections
from tools.csv_parser import loadBeacons


class FakeBeaconPublisher(Node):
    '''
    Publishes simulated beacon_data so navigation can progress in sim
    without Bluetooth hardware.
    '''
    def __init__(self):
        super().__init__('fake_beacon_publisher')

        self.publisher = self.create_publisher(String, 'fake_beacon_data', 10)
        dest_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.destinations_sub = self.create_subscription(
            String, 'destinations', self.destinations_callback, dest_qos
        )
        self.destinations_pub = self.create_publisher(String, 'destinations', dest_qos)
        self.odom_sub = None
        self.gt_pose_sub = None

        self.publish_interval_s = 3.0
        self.fake_rssi = -40
        self.timer = None
        self.path = []
        self.path_index = 0
        self.robot_xy = None
        self.current_destination = None
        self.prev_beacon = None
        self.beacon_reach_distance = 0.9
        self.last_beacon = None
        self.last_odom_time = None
        self.last_pose_time = None
        self.last_beacon_publish_time = None
        self.pose_publish_timeout_s = 8.0
        self.odom_stale_timeout_s = 2.0
        self.declare_parameter("default_destination", "Nicol:Canal")
        self.default_destination = str(self.get_parameter("default_destination").value)
        self.declare_parameter("publish_default_destination", True)
        self.publish_default_destination = bool(self.get_parameter("publish_default_destination").value)
        self.declare_parameter("allowed_beacons", "")
        self.allowed_beacons = self._parse_allowed_beacons(self.get_parameter("allowed_beacons").value)
        self.declare_parameter("force_path_beacons", False)
        self.force_path_beacons = bool(self.get_parameter("force_path_beacons").value)
        self.default_dest_timer = self.create_timer(0.5, self._publish_default_destination_once)
        self._default_destination_sent = False

        self.connections = loadConnections()
        self.graph = self._build_graph(self.connections)
        self.beacons = loadBeacons()
        if self.allowed_beacons:
            self.beacons = {mac: name for mac, name in self.beacons.items() if name in self.allowed_beacons}
        self.beacon_positions = self._load_beacon_positions()
        if self.beacon_positions:
            self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
            self.gt_pose_sub = self.create_subscription(Odometry, 'sim_ground_truth_pose', self.gt_pose_callback, 10)

        self.get_logger().info("FakeBeaconPublisher ready.")

    @staticmethod
    def _parse_allowed_beacons(value) -> set:
        if value is None:
            return set()
        if isinstance(value, (list, tuple, set)):
            return {str(v).strip() for v in value if str(v).strip()}
        raw = str(value).strip()
        if not raw:
            return set()
        return {part.strip() for part in raw.split(",") if part.strip()}

    def _publish_default_destination_once(self):
        if self._default_destination_sent or not self.publish_default_destination:
            return
        if self.current_destination is not None or self.prev_beacon is not None:
            return
        msg = String()
        msg.data = self.default_destination
        self.destinations_pub.publish(msg)
        self._default_destination_sent = True
        self.default_dest_timer.cancel()
        self.get_logger().info(f"Published default destination: {msg.data}")

    def _build_graph(self, connections: dict) -> dict:
        graph = {}
        for current, prev_map in connections.items():
            graph.setdefault(current, set())
            for prev, orientation in prev_map.items():
                if prev == current:
                    continue
                if orientation == "-":
                    continue
                graph.setdefault(prev, set())
                graph[current].add(prev)
                graph[prev].add(current)
        return graph

    def _find_path(self, source: str, destination: str) -> list:
        if source == destination:
            return [source]
        if source not in self.graph or destination not in self.graph:
            return [destination]

        visited = set([source])
        queue = deque([(source, [source])])
        while queue:
            node, path = queue.popleft()
            if node == destination:
                return path
            for neighbor in sorted(self.graph.get(node, [])):
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                queue.append((neighbor, path + [neighbor]))
        return [destination]

    def _resolve_world_path(self) -> str | None:
        env_path = os.getenv("MAIL_ROBOT_WORLD_PATH")
        if env_path and os.path.exists(env_path):
            return env_path

        candidates = [
            "/tmp/demo_video_ci.world",
            "/ros2_ws/src/carleton_mail_robot/external_files/demo_video.world",
        ]
        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
        return None

    def _load_beacon_positions(self) -> dict:
        world_path = self._resolve_world_path()
        if not world_path:
            self.get_logger().warning("No world file found; falling back to timed beacons.")
            return {}

        model_to_beacon = {mac.replace(":", "_"): name for mac, name in self.beacons.items()}
        positions = {}
        try:
            tree = ET.parse(world_path)
            root = tree.getroot()
            for model in root.iter("model"):
                model_name = model.attrib.get("name")
                if not model_name or model_name not in model_to_beacon:
                    continue
                if model_name in positions:
                    continue
                pose_el = model.find("pose")
                if pose_el is None or not pose_el.text:
                    continue
                parts = pose_el.text.split()
                if len(parts) < 2:
                    continue
                x, y = float(parts[0]), float(parts[1])
                positions[model_to_beacon[model_name]] = (x, y)
        except Exception as exc:
            self.get_logger().warning(f"Failed to parse world file for beacons: {exc}")
            return {}

        if self.allowed_beacons:
            positions = {name: pos for name, pos in positions.items() if name in self.allowed_beacons}
        if not positions:
            self.get_logger().warning("No beacon poses found in world file; falling back to timed beacons.")
        else:
            self.get_logger().info(f"Loaded {len(positions)} beacon poses from world file.")
        return positions

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        self.robot_xy = (position.x, position.y)
        self.last_odom_time = self.get_clock().now()
        self.last_pose_time = self.last_odom_time

    def gt_pose_callback(self, msg: Odometry):
        # Use ground-truth pose when /odom is missing in sim.
        position = msg.pose.pose.position
        self.robot_xy = (position.x, position.y)
        self.last_pose_time = self.get_clock().now()

    def destinations_callback(self, msg: String):
        parts = msg.data.split(':')
        if len(parts) < 2:
            self.get_logger().warning(f"Invalid destination payload: {msg.data}")
            return
        source = parts[0].strip()
        destination = parts[1].strip()

        self.path = self._find_path(source, destination)
        self.path_index = 0
        self.get_logger().info(f"Simulated beacon path: {self.path}")

        if self.timer is not None:
            self.timer.cancel()
        if self.beacon_positions and not self.force_path_beacons:
            self.timer = self.create_timer(0.5, self.publish_next)
        else:
            self.timer = self.create_timer(self.publish_interval_s, self.publish_next)

    def publish_next(self):
        if self.beacon_positions and not self.force_path_beacons:
            if self.robot_xy is None:
                return
            now = self.get_clock().now()
            if self.last_pose_time is None:
                return
            pose_age = (now - self.last_pose_time).nanoseconds / 1e9
            if pose_age > self.odom_stale_timeout_s:
                self._publish_path_beacon()
                return

            closest_beacon = None
            closest_distance = None
            for name, pos in self.beacon_positions.items():
                dx = self.robot_xy[0] - pos[0]
                dy = self.robot_xy[1] - pos[1]
                distance = math.hypot(dx, dy)
                if closest_distance is None or distance < closest_distance:
                    closest_distance = distance
                    closest_beacon = name
            if closest_beacon is None or closest_distance is None:
                return
            if closest_distance > self.beacon_reach_distance:
                # If we're not close to any beacon for too long, fall back.
                if self.last_beacon_publish_time is not None:
                    idle_s = (now - self.last_beacon_publish_time).nanoseconds / 1e9
                    if idle_s > self.pose_publish_timeout_s:
                        self._publish_path_beacon()
                return
            if closest_beacon == self.last_beacon:
                return
            beacon = closest_beacon
        else:
            beacon = self._next_path_beacon()
            if beacon is None:
                return

        msg = String()
        msg.data = f"{beacon},{self.fake_rssi}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published fake beacon: {msg.data}")
        self.last_beacon = beacon
        self.last_beacon_publish_time = self.get_clock().now()

        if not self.beacon_positions:
            self.path_index += 1
            if self.path_index >= len(self.path) and self.timer is not None:
                self.timer.cancel()

    def _next_path_beacon(self):
        if not self.path:
            return None
        if self.path_index >= len(self.path):
            return None

        # Skip the source beacon; navigation already knows it as prev_beacon.
        if self.path_index == 0:
            self.path_index += 1
            if self.path_index >= len(self.path):
                return None
        while self.path_index < len(self.path):
            candidate = self.path[self.path_index]
            if not self.allowed_beacons or candidate in self.allowed_beacons:
                return candidate
            self.path_index += 1
        return None

    def _publish_path_beacon(self):
        beacon = self._next_path_beacon()
        if beacon is None:
            return
        msg = String()
        msg.data = f"{beacon},{self.fake_rssi}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published fallback beacon: {msg.data}")
        self.last_beacon = beacon
        self.last_beacon_publish_time = self.get_clock().now()
        self.path_index += 1
        if self.path_index >= len(self.path) and self.timer is not None:
            self.timer.cancel()


def main():
    rclpy.init()
    node = FakeBeaconPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
