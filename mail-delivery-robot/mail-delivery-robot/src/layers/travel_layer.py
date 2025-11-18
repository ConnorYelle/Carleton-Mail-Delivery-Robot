import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tools.csv_parser import loadConfig

class TravelLayerStates(Enum):
    '''
    An enum for the internal states of the travel layer.
    '''
    NO_DEST = 'NO_DEST'
    HAS_DEST = 'HAS_DEST'

class TravelLayer(Node):
    '''
    The subsumption layer responsible for moving the robot forward,
    performing wall following, etc.

    @Subscribers:
    - Listens to /lidar_data for data about nearby walls.
    - Listens to /destinations for data about the robot's current destination.
    - Listens to /dock_status for data about the robot's dock status.

    @Publishers:
    - Publishes action messages to /actions.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('travel_layer')
        self.config = loadConfig()

        self.state = TravelLayerStates.NO_DEST
        self.current_destination = 'NONE'
        self.is_docked = False
        self.was_docked = False

        # Subscribe to lidar data (from the lidar sensor node).
        self.lidar_data_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)
        self.destinations_sub = self.create_subscription(String, 'destinations', self.destinations_callback, 10)
        self.dock_status_sub = self.create_subscription(DockStatus, 'dock_status', self.dock_status_callback, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        ))

        # Publisher for sending action messages.
        self.action_publisher = self.create_publisher(String, 'actions', 10)

        # Standard messages for basic commands.
        self.go_msg = String()
        self.go_msg.data = '3:GO'
        self.no_msg = String()
        self.no_msg.data = '3:NONE'

        # Latest parsed lidar data.
        # Expected keys: feedback, angle, right, left, front
        self.latest_lidar = None

        # Timer to update actions.
        self.timer = self.create_timer(0.2, self.update_actions)
        self.action_publisher.publish(self.no_msg)

    def lidar_data_callback(self, data):
        '''
        The callback for /lidar_data.
        Reads and parses information about nearby walls.
        Expected format from lidar sensor: "feedback:angle:right:left:front"
        '''
        try:
            parts = data.data.split(":")
            if len(parts) != 5:
                self.get_logger().warning("Lidar data format incorrect")
                return
            self.latest_lidar = {
                "feedback": float(parts[0]),
                "angle": float(parts[1]),
                "right": float(parts[2]),
                "left": float(parts[3]),
                "front": float(parts[4])
            }
        except Exception as e:
            self.get_logger().error(f"Error parsing lidar data: {e}")
            self.latest_lidar = None

    def destinations_callback(self, data):
        '''
        The callback for /destinations.
        Reads the robot's current destination.
        Expected format: "SOURCE:DESTINATION", where source is the starting location.
        '''
        try:
            # Assume destination is the part after the colon.
            self.current_destination = data.data.split(":")[1]
        except Exception as e:
            self.get_logger().error(f"Error parsing destination: {e}")
            self.current_destination = 'NONE'

    def dock_status_callback(self, data):
        '''
        The callback for /dock_status.
        Reads the robot's docked status.
        '''
        self.was_docked = self.is_docked
        self.is_docked = data.is_docked

    def compute_wall_follow(self):
        '''
        Computes the linear and angular speeds needed to follow a wall based
        on the most recent lidar data.
        
        Uses configuration parameters:
          - WALL_FOLLOW_SET_POINT: Desired distance from wall.
          - WALL_FOLLOW_AIM_ANGLE: Base aiming angle (degrees).
          - WALL_FOLLOW_SPEED: Base speed value.
          - WALL_FOLLOW_ANGLE_CHANGE_THRESHOLD: Threshold to adjust linear speed.
        
        Returns:
          Tuple (linear_speed, angular_speed) if computation is possible,
          otherwise None.
        '''
        if self.latest_lidar is None:
            return None

        # Use the lidar "feedback" and "angle" fields.
        cur_distance = self.latest_lidar["feedback"]
        cur_angle = self.latest_lidar["angle"]

        SET_POINT = self.config["WALL_FOLLOW_SET_POINT"]
        AIM_ANGLE = self.config["WALL_FOLLOW_AIM_ANGLE"]
        # Calculate an error range based on speed and desired aim.
        ERROR = self.config["WALL_FOLLOW_SPEED"] * math.sin(math.radians(AIM_ANGLE))

        # Normalize angle if necessary.
        if cur_angle > 180:
            cur_angle -= 360

        # Compute the required angle adjustment.
        if cur_distance > SET_POINT + ERROR:
            res_angle = -1 * AIM_ANGLE + cur_angle
        elif cur_distance < SET_POINT - ERROR:
            res_angle = AIM_ANGLE + cur_angle
        else:
            res_angle = cur_angle

        angular_speed = math.radians(res_angle)
        # Adjust linear speed: slow down if the angle change is large.
        if abs(angular_speed) > self.config["WALL_FOLLOW_ANGLE_CHANGE_THRESHOLD"]:
            linear_speed = self.config["WALL_FOLLOW_SPEED"] / 2
        else:
            linear_speed = self.config["WALL_FOLLOW_SPEED"]
        return linear_speed, angular_speed

    def update_actions(self):
        '''
        Timer callback.
        Updates internal state and sends action messages.
        Chooses wall-following movement if a destination is set and valid sensor
        data is available; otherwise, sends standard GO or NONE commands.
        '''
        # Update state based on destination.
        if self.state == TravelLayerStates.NO_DEST and self.current_destination != 'NONE':
            self.state = TravelLayerStates.HAS_DEST
        elif self.state == TravelLayerStates.HAS_DEST and self.current_destination == 'NONE':
            self.state = TravelLayerStates.NO_DEST

        # If the robot has a destination and is not docked, attempt wall following.
        if self.state == TravelLayerStates.HAS_DEST and not self.is_docked:
            speeds = self.compute_wall_follow()
            action_msg = String()
            if speeds is not None:
                linear_speed, angular_speed = speeds
                # Format: WALL_FOLLOW:<linear_speed>:<angular_speed>
                action_msg.data = f'3:WALL_FOLLOW,{linear_speed},{angular_speed}'
            else:
                # If no valid sensor data, fall back to a GO command.
                action_msg.data = '3:GO'
            self.action_publisher.publish(action_msg)
        # When docked, publish the NONE action.
        if self.is_docked and not self.was_docked:
            self.action_publisher.publish(self.no_msg)

def main():
    rclpy.init()
    travel_layer = TravelLayer()
    rclpy.spin(travel_layer)

if __name__ == '__main__':
    main()
