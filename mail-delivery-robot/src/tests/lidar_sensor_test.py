import rclpy
from rclpy.node import Node
from unittest.mock import MagicMock, patch
import pytest
import sensors.lidar_sensor as lidar_sensor
from sensor_msgs.msg import LaserScan
import math

#mock configuration dictionary for testing, we don't want tests to depend on external config files
#these are the only parameters that LidarSensor uses from the config file
MOCK_CONFIG = {
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


#fixture to initialize a LidarSensor node with mocked config
@pytest.fixture
def lidar_node():
    with patch("sensors.lidar_sensor.loadConfig", return_value=MOCK_CONFIG):
        node = lidar_sensor.LidarSensor()
        return node


#helper function to create a mock LaserScan message
def make_mock_scan():
    scan = LaserScan()
    scan.ranges = [10.0] * 360  #initialize all distances to 10.0, no object in sight
    for i in range(175, 185):
        scan.ranges[i] = 0.5  #object 0.5m in front of the robot within angles 175 - 185
    scan.angle_min = 0.0
    scan.angle_increment = math.pi / 180  #1 degree in radians
    return scan


#test to verify that LidarSensor initializes correctly
def test_lidar_sensor_initialization(rclpy_init_shutdown, lidar_node):

    node = lidar_node

    assert node is not None

    #check that all config parameters are set correctly in the node
    for key, value in MOCK_CONFIG.items():
        assert node.config[key] == value

    #check that the node is subscribed to the correct topic
    assert node.lidar_info_sub.topic_name == "/scan"


#test to verify that calculate function works correctly
def test_calculate_function(lidar_node):
    node = lidar_node
    scan = make_mock_scan()

    #fill the distance stacks to avoid early return (-1 values)
    for _ in range(MOCK_CONFIG["LIDAR_STACK_LENGTH"]):
        node.calculate(scan)

    #call calculate once more to get real distances
    feedback, angle, right, left, front = node.calculate(scan)

    #object in front of sensor should not change the left and right distances
    assert left == -1 or left == 10
    assert right == -1 or right == 10

    #front distance should be approximately 0.5
    assert abs(front - 0.5) < 0.001

    #assert that the robot is following the wall under the default distance
    assert feedback <= MOCK_CONFIG["LARGE_DEFAULT_DISTANCE"]

    #angle should be within valid range
    assert -90 <= angle <= 90

#test to verify that scan_callback is sending messages correctly
def test_scan_callback_publish(lidar_node):
    node = lidar_node

    #mock the publisher to capture output instead of sending it
    node.publisher_.publish = MagicMock()

    scan = make_mock_scan()

    #fill the distance stacks to avoid early return
    for _ in range(MOCK_CONFIG["LIDAR_STACK_LENGTH"]):
        node.scan_callback(scan) #this also tests the calculate function in the lidar sensor node

    #get the published message
    published_msg = node.publisher_.publish.call_args[0][0]
    assert isinstance(published_msg.data, str)

    #split message fields from: feedback:angle:right:left:front
    fields = list(map(float, published_msg.data.split(":")))
    feedback, angle, right, left, front = fields
    assert len(fields) == 5

    
