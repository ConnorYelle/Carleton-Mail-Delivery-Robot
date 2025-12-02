import rclpy
from rclpy.node import Node
from unittest.mock import patch
import pytest
import sensors.lidar_sensor as lidar_sensor

#mock configuration dictionary for testing, we don't want tests to depend on external config files
#these are the only parameters that LidarSensor uses from the config file
mock_config = {
    "LARGE_DEFAULT_DISTANCE": 10,
    "WALL_FOLLOW_MIN_ANGLE": 70,
    "WALL_FOLLOW_MAX_ANGLE": 150,
    "FRONT_MIN_ANGLE": -170,
    "FRONT_MAX_ANGLE": 170,
    "RIGHT_MIN_ANGLE": 110,
    "RIGHT_MAX_ANGLE": 115,
    "LEFT_MIN_ANGLE": -115,
    "LEFT_MAX_ANGLE": -110,
    "LOST_WALL_FRONT_DISTANCE": 4.0,
    "LOST_WALL_FRONT_STDEV": 0.2,
    "LOST_WALL_RIGHT_DISTANCE": 4.0,
    "LOST_WALL_RIGHT_STDEV": 0.2,
    "LOST_WALL_LEFT_DISTANCE": 5.0,
    "LOST_WALL_LEFT_STDEV": 0.5,
    "LIDAR_STACK_LENGTH": 10
}

@pytest.fixture(scope="session")  #scope session means that we run this once per test session
def rclpy_init_shutdown():
    rclpy.init()  #initialize ROS2 library
    yield         #run tests   
    rclpy.shutdown()  #shutdown ROS2 library


#test to verify that LidarSensor initializes correctly
def test_lidar_sensor_initialization(rclpy_init_shutdown):
    #instead of loading actual config file, we patch the loadConfig function to return our mock_config
    with patch("sensors.lidar_sensor.loadConfig", return_value=mock_config):
        node = lidar_sensor.LidarSensor()

    assert node is not None

    #check that all config parameters are set correctly
    for key, value in mock_config.items():
        assert node.config[key] == value


