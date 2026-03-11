import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque

from tools.nav_parser import loadConnections


class FakeBeaconPublisher(Node):
    '''
    Publishes simulated beacon_data so navigation can progress in sim
    without Bluetooth hardware.
    '''
    def __init__(self):
        super().__init__('fake_beacon_publisher')

        self.publisher = self.create_publisher(String, 'beacon_data', 10)
        self.destinations_sub = self.create_subscription(
            String, 'destinations', self.destinations_callback, 10
        )

        self.publish_interval_s = 3.0
        self.fake_rssi = -40
        self.timer = None
        self.path = []
        self.path_index = 0

        self.connections = loadConnections()
        self.graph = self._build_graph(self.connections)

        self.get_logger().info("FakeBeaconPublisher ready.")

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
        self.timer = self.create_timer(self.publish_interval_s, self.publish_next)

    def publish_next(self):
        if not self.path:
            return
        if self.path_index >= len(self.path):
            return

        # Skip the source beacon; navigation already knows it as prev_beacon.
        if self.path_index == 0:
            self.path_index += 1
            if self.path_index >= len(self.path):
                return

        beacon = self.path[self.path_index]
        msg = String()
        msg.data = f"{beacon},{self.fake_rssi}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published fake beacon: {msg.data}")

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
