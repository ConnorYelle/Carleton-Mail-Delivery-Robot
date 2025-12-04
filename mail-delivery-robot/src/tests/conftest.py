import sys
from unittest.mock import MagicMock

# -----------------------------
# Mock rclpy
# -----------------------------
rclpy_mock = MagicMock()
rclpy_mock.init = MagicMock()
rclpy_mock.shutdown = MagicMock()


class MockPublisher:
    def __init__(self):
        self.publish = MagicMock()


class MockSubscription:
    def __init__(self):
        pass


class MockSubscription:
    def __init__(self, topic):
        self.topic_name = topic


class MockNode:
    def __init__(self, *args, **kwargs):
        self._publishers = []
        self._subscriptions = []

    def create_publisher(self, msg_type, topic, qos):
        pub = MockPublisher()
        self._publishers.append((msg_type, topic, qos))
        return pub

    def create_subscription(self, msg_type, topic, callback, *args, **kwargs):
        sub = MockSubscription(topic)
        self._subscriptions.append((msg_type, topic, callback, args, kwargs))
        return sub

    def get_logger(self):
        logger = MagicMock()
        logger.info = MagicMock()
        logger.warn = MagicMock()
        logger.error = MagicMock()
        return logger



# Expose Node under rclpy.node.Node
rclpy_mock.node = MagicMock()
rclpy_mock.node.Node = MockNode

# Register rclpy mock
sys.modules["rclpy"] = rclpy_mock
sys.modules["rclpy.node"] = rclpy_mock.node


# -----------------------------
# Mock std_msgs.msg
# -----------------------------
std_msgs_mock = MagicMock()

class MockString:
    def __init__(self, data=None):
        self.data = data

std_msgs_mock.msg = MagicMock()
std_msgs_mock.msg.String = MockString

sys.modules["std_msgs"] = std_msgs_mock
sys.modules["std_msgs.msg"] = std_msgs_mock.msg


# -----------------------------
# Mock sensor_msgs.msg (LaserScan)
# -----------------------------
sensor_msgs_mock = MagicMock()

class MockLaserScan:
    def __init__(self):
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

sensor_msgs_mock.msg = MagicMock()
sensor_msgs_mock.msg.LaserScan = MockLaserScan

sys.modules["sensor_msgs"] = sensor_msgs_mock
sys.modules["sensor_msgs.msg"] = sensor_msgs_mock.msg
