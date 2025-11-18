import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
from tools.csv_parser import loadConfig

class IntersectionDetectionUnit(Node):
    '''
    The Node in charge of intersection detection.

    @Subscribers:
    - Subscribes to /camera_data for information about intersection markers.
    - Subscribes to /lidar_data for information about nearby walls.
    - Subscribes to /odom for information about the robot's current position.

    @Publishers:
    - Publishes intersection detection data to /intersection_detection.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('intersection_detection_unit')

        self.config = loadConfig()

        self.lidar_data_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)
        self.camera_data_sub = self.create_subscription(Bool, 'camera_data', self.camera_data_callback, 10)
        self.odometry_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        ))

        self.intersection_detection_publisher = self.create_publisher(String, 'intersection_detection', 10)
        self.intersection_detection_timer = self.create_timer(0.5, self.update_intersection_detection)

        self.true_msg = String()
        self.true_msg.data = 'TRUE'
        self.false_msg = String()
        self.false_msg.data = 'FALSE'

        self.lidar_indicates_intersection = False
        self.camera_indicates_intersection = False
        self.odom_x_snapshot = None
        self.odom_y_snapshot = None
        self.odom_x = None
        self.odom_y = None
    
    def lidar_data_callback(self, data):
        '''
        The callback for /lidar_data.
        Reads information from the lidar sensor.
        '''
        lidar_data = str(data.data)

        split_data = lidar_data.split(":")
        
        # waiting for the lidar to calibrate
        if split_data[0] == "-1" and split_data[1] == "-1":
            return
        
        if ((split_data[2] == "-1" and split_data[3] == "-1")
            or (split_data[2] == "-1" and split_data[4] == "-1")
            or (split_data[3] == "-1" and split_data[4] == "-1")):
            self.lidar_indicates_intersection = True
        else:
            self.lidar_indicates_intersection = False
    
    def camera_data_callback(self, data):
        '''
        The callback for /camera_data.
        Reads information about intersection markers.
        '''
        if data.data is True:
            self.camera_indicates_intersection = True
            self.odom_x_snapshot = self.odom_x
            self.odom_y_snapshot = self.odom_y

    def odometry_callback(self, data):
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y

    def update_intersection_detection(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /navigation when necessary
        '''
        if self.odom_x_snapshot is not None and self.odom_y_snapshot is not None:
            starting_location = [self.odom_x_snapshot, self.odom_y_snapshot]
            current_location = [self.odom_x, self.odom_y]
            #self.get_logger().info(str(math.dist(starting_location, current_location)))
            #TODO: if the distance is larger than some value, set self.camera_indicates_intersection to false
            if math.dist(starting_location, current_location) > self.config["INTERSECTION_MAX_DISTANCE_FROM_MARKER"]:
                self.camera_indicates_intersection = False


        if self.lidar_indicates_intersection and self.camera_indicates_intersection:
            self.intersection_detection_publisher.publish(self.true_msg)
        else:
            self.intersection_detection_publisher.publish(self.false_msg)

def main():
    rclpy.init()
    intersection_detection_unit = IntersectionDetectionUnit()
    rclpy.spin(intersection_detection_unit)

if __name__ == '__main__':
    main()