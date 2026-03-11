import shutil
import sys
import tempfile
import types
from pathlib import Path

import pytest

SRC_DIR = Path(__file__).resolve().parents[1]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

# Mock
# ROS dependencies
fake_rclpy = types.ModuleType("rclpy")
fake_rclpy.node = types.ModuleType("rclpy.node")


class FakeNode:
    def __init__(self, *args, **kwargs):
        pass

    def create_subscription(self, *args, **kwargs):
        pass

    def get_logger(self):
        class Logger:
            def info(self, *args):
                pass

            def error(self, *args):
                pass

        return Logger()


fake_rclpy.node.Node = FakeNode
sys.modules["rclpy"] = fake_rclpy
sys.modules["rclpy.node"] = fake_rclpy.node

# sensor_msgs.msg.BatteryState
fake_sensor_msgs = types.ModuleType("sensor_msgs")
fake_sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
fake_sensor_msgs.msg.BatteryState = object
sys.modules["sensor_msgs"] = fake_sensor_msgs
sys.modules["sensor_msgs.msg"] = fake_sensor_msgs.msg

# rclpy.qos
fake_qos = types.ModuleType("rclpy.qos")


class FakeQoSReliabilityPolicy:
    BEST_EFFORT = 1


class FakeQoSProfile:
    def __init__(self, *args, **kwargs):
        pass


fake_qos.QoSProfile = FakeQoSProfile
fake_qos.QoSReliabilityPolicy = FakeQoSReliabilityPolicy

sys.modules["rclpy.qos"] = fake_qos

# jinja2 Environment
fake_jinja2 = types.ModuleType("jinja2")


class FakeTemplate:
    def render(self, **kwargs):
        return "<html>fake</html>"


class FakeEnvironment:
    def __init__(self, *args, **kwargs):
        pass

    def get_template(self, name):
        return FakeTemplate()


fake_jinja2.Environment = FakeEnvironment
fake_jinja2.FileSystemLoader = lambda *args, **kwargs: None

sys.modules["jinja2"] = fake_jinja2
sys.modules["jinja2.environment"] = fake_jinja2
sys.modules["jinja2.loaders"] = fake_jinja2

# -------------------------------------------------

from control.report_generator import ReportGenerator


@pytest.fixture
def temp_log_dir():
    temp_dir = tempfile.mkdtemp()
    yield temp_dir
    shutil.rmtree(temp_dir)


@pytest.fixture
def report_generator(temp_log_dir):
    return ReportGenerator(log_dir=temp_log_dir, template_dir=temp_log_dir)


def test_read_wall_follow_time_valid(report_generator, temp_log_dir):
    log_path = Path(temp_log_dir) / "robot_log_wallFollowing.txt"
    log_path.write_text("Some log\n" "Total wall-following time: 12.34s\n")

    result = report_generator.read_wall_follow_time()
    assert result == "12.34 s"


def test_read_wall_follow_time_missing_file(report_generator):
    result = report_generator.read_wall_follow_time()
    assert result == "N/A"


def test_read_wall_follow_time_no_match(report_generator, temp_log_dir):
    log_path = Path(temp_log_dir) / "robot_log_wallFollowing.txt"
    log_path.write_text("No useful data here\n")

    result = report_generator.read_wall_follow_time()
    assert result == "N/A"


def test_read_delivery_time_valid(report_generator, temp_log_dir):
    log_path = Path(temp_log_dir) / "robot_log_tripTime.txt"
    log_path.write_text("Trip 1: 10s\n" "Trip 2: 25s\n")

    result = report_generator.read_delivery_time()
    assert result == "Trip 2: 25s"


def test_read_delivery_time_empty_file(report_generator, temp_log_dir):
    log_path = Path(temp_log_dir) / "robot_log_tripTime.txt"
    log_path.write_text("")

    result = report_generator.read_delivery_time()
    assert result == "N/A"


def test_read_delivery_time_missing_file(report_generator):
    result = report_generator.read_delivery_time()
    assert result == "N/A"
