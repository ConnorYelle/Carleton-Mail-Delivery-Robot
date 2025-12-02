import rclpy
from rclpy.node import Node
from unittest.mock import MagicMock, patch
import pytest
import sensors.bumper_sensor as bumper_sensor


MOCK_BUMPER_CONFIG = {
    "MAX_BUMP_COUNT": 100 
}

@pytest.fixture(scope="session")  #scope session means that we run this once per test session
def rclpy_init_shutdown():
    rclpy.init()  #initialize ROS2 library
    yield         #run tests   
    rclpy.shutdown()  #shutdown ROS2 library


#fixture to initialize a BumperSensor node with mocked config
@pytest.fixture
def bumper_node():
    #instead of loading actual config file, we patch the loadConfig function to return our mock_config
    with patch("sensors.bumper_sensor.loadConfig", return_value=MOCK_BUMPER_CONFIG):
        node = bumper_sensor.BumperSensor()
        return node
    
def test_bumper_sensor_initialization(rclpy_init_shutdown, bumper_node):

    node = bumper_node
    assert node.counter == 0
    assert node.lastState == ""
    assert node.config == MOCK_BUMPER_CONFIG
    assert node.bumperSubscriber.topic_name == "/hazard_detection"

def test_read_bump(bumper_node):
    node = bumper_node
    node.publisher_ = MagicMock()

    class MockHazard:
        def __init__(self, hazard_type):
            self.type = hazard_type

    class MockHazardVector:
        def __init__(self, hazards):
            self.detections = hazards

    # Case 1: No hazards -> UNPRESSED
    node.lastState = ""
    node.counter = 0
    no_hazard_msg = MockHazardVector(hazards=[])
    node.read_bump(no_hazard_msg)
    node.publisher_.publish.assert_called_once()
    assert node.lastState == bumper_sensor.Bump_Event.UNPRESSED.value
    node.publisher_.publish.reset_mock()

    # Case 2: Hazard type 1 -> PRESSED
    node.lastState = ""
    node.counter = 0
    hazard_msg = MockHazardVector(hazards=[MockHazard(1)])
    node.read_bump(hazard_msg)
    node.publisher_.publish.assert_called_once()
    assert node.lastState == bumper_sensor.Bump_Event.PRESSED.value
    node.publisher_.publish.reset_mock()

    # Case 3: Hazard type 2 -> PRESSED
    node.lastState = ""
    node.counter = 0
    hazard_msg2 = MockHazardVector(hazards=[MockHazard(2)])
    node.read_bump(hazard_msg2)
    node.publisher_.publish.assert_called_once()
    assert node.lastState == bumper_sensor.Bump_Event.PRESSED.value
    node.publisher_.publish.reset_mock()

    # Case 4: Hazard type not 1 or 2 -> UNPRESSED
    node.lastState = ""
    node.counter = 0
    hazard_msg3 = MockHazardVector(hazards=[MockHazard(3)])
    node.read_bump(hazard_msg3)
    node.publisher_.publish.assert_called_once()
    assert node.lastState == bumper_sensor.Bump_Event.UNPRESSED.value