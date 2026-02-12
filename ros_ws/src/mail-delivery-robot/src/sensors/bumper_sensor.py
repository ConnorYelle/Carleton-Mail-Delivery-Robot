from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from enum import Enum
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tools.csv_parser import loadConfig

class Bump_Event(Enum):
    '''
    An enum for the various bump events for the robot.
    '''
    PRESSED = "PRESSED"
    UNPRESSED = "UNPRESSED"

class BumperSensor(Node):
    '''
    The Node in charge of listening to the bumper sensor.

    @Subscribers:
    - Listens to /hazard_detection to read the current state of the bumper sensor.

    @Publishers:
    - Publishes new  messages to /bumper_data.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('bumper_sensor')
        
        # Load the global config.
        self.config = loadConfig()

        # Sets the default values for the sensor.
        self.counter = 0
        self.lastState = ""

        # The publishers for the node.
        self.publisher_ = self.create_publisher(String, 'bumper_data', 10)
        
        self.bumperSubscriber = self.create_subscription(HazardDetectionVector, 'hazard_detection', self.read_bump, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10))
    
    def read_bump(self, data):
        '''
        The callback for /hazard_detection.
        Reads the bump data and acts accordingly.

        @param data: The new bumper data received.
        '''
        bumpEvent = String()

        # Updates the bumper state.
        hazards = data.detections
        got_hazard = False
        if len(hazards) != 0:
            for h in hazards:
                if h.type == 1 or h.type == 2:
                    got_hazard = True
                    break

        if got_hazard:
            bumpEvent.data = Bump_Event.PRESSED.value
        else:
            bumpEvent.data = Bump_Event.UNPRESSED.value

        # Slows the publishing of the messages to ensure the detection is smooth.
        if (self.lastState != bumpEvent.data or self.counter > self.config["MAX_BUMP_COUNT"]):
            self.lastState = bumpEvent.data
            self.publisher_.publish(bumpEvent)
            self.counter = 0
        self.counter += 1

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    bumper_sensor = BumperSensor()
    rclpy.spin(bumper_sensor)


if __name__ == '__main__':
    main()
