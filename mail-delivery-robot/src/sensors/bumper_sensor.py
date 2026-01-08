from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from enum import Enum
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector

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
        super().__init__("bumper_sensor")

        # Import QoS ONLY after rclpy.init() has happened
        from rclpy.qos import (
            QoSProfile,
            QoSReliabilityPolicy,
            QoSHistoryPolicy,
        )

        self.config = loadConfig()
        self.counter = 0
        self.lastState = ""

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.bumperSubscriber = self.create_subscription(
            HazardDetectionVector,
            "/hazard_detection",
            self.read_bump,
            qos_profile,
        )

        self.publisher_ = self.create_publisher(
            String,
            "/bumper_event",
            10,
        )

    def read_bump(self, msg: HazardDetectionVector):
        """
        Processes hazard detection messages and publishes
        PRESSED or UNPRESSED bumper events.
        """

        bump_detected = False

        for hazard in msg.detections:
            # Hazard types 1 and 2 are considered bumper presses
            if hazard.type in (1, 2):
                bump_detected = True
                break

        if bump_detected:
            new_state = Bump_Event.PRESSED.value
        else:
            new_state = Bump_Event.UNPRESSED.value

        # Only publish if state changes OR first run
        if new_state != self.lastState:
            self.lastState = new_state
            self.counter = min(
                self.counter + 1,
                self.config["MAX_BUMP_COUNT"]
            )

            msg_out = String()
            msg_out.data = new_state
            self.publisher_.publish(msg_out)

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    bumper_sensor = BumperSensor()
    rclpy.spin(bumper_sensor)


if __name__ == '__main__':
    main()
