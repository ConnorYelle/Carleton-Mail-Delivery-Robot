import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
from tools.csv_parser import loadConfig

class TurningLayer(Node):
    '''
    The subsumption layer responsible for moving the robot through intersections.

    @Subscribers:
    - Listens to /navigation for the next action the robot should take at an intersection
    - Listens to /intersection_detection for data about whether or not the robot is in an intersection

    @Publishers:
    - Publishes actions to /actions
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('turning_layer')

        self.config = loadConfig()

        self.last_nav_msg = None
        self.in_intersection = False
        self.nav_message_handled = False

        self.actions = {
            "U_TURN": String(data="2:U_TURN"),
            "LEFT_TURN": String(data="2:LEFT_TURN"),
            "RIGHT_TURN": String(data="2:RIGHT_TURN"),
            "STRAIGHT": String(data="2:GO")
        }
        self.no_msg = String(data="2:NONE")
        self.go_msg = String(data="2:GO")

        self.navigation_sub = self.create_subscription(String, 'navigation', self.navigation_callback, 10)
        self.intersection_detection_sub = self.create_subscription(String, 'intersection_detection', self.intersection_detection_callback, 10)
        
        self.action_publisher = self.create_publisher(String, 'actions', 10)
        self.turn_cycles = self.config["TURN_CYCLES"]
        self.u_turn_cycles = self.config["U_TURN_CYCLES"]
        self.go_cycles = self.config["TURNING_GO_CYCLES"]

        self.timer = self.create_timer(0.2, self.update_actions)


    def navigation_callback(self, data):
        '''
        The callback for /navigation.
        Reads information about navigation actions the robot should take.
        '''
        self.last_nav_msg = data.data.upper()
        self.nav_message_handled = False

    def intersection_detection_callback(self, data):
        '''
        The callback for /intersection_detection.
        Reads information about whether the robot is currently in an intersection.
        '''
        self.in_intersection = data.data.upper() == "TRUE"  

    def update_actions(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /actions when necessary
        '''
        if self.last_nav_msg == "U_TURN" and not self.nav_message_handled:
            if self.u_turn_cycles > 0:
                self.action_publisher.publish(self.actions["U_TURN"])
                self.u_turn_cycles -= 1
                return
            else:
                self.action_publisher.publish(self.no_msg)
                self.nav_message_handled = True
                self.u_turn_cycles = self.config["U_TURN_CYCLES"]
                return  

        if not self.in_intersection:
           self.action_publisher.publish(self.no_msg)
           return  

        if self.last_nav_msg is not None and not self.nav_message_handled:
            if self.turn_cycles > 0:
                if self.last_nav_msg in self.actions:
                    self.action_publisher.publish(self.actions[self.last_nav_msg])
                    self.turn_cycles -= 1
            else:
                if self.go_cycles > 0:
                    self.action_publisher.publish(self.go_msg)
                    self.go_cycles -= 1
                else:
                    self.action_publisher.publish(self.no_msg)
                    self.nav_message_handled = True  
                    self.turn_cycles = self.config["TURN_CYCLES"]
                    self.go_cycles = self.config["TURNING_GO_CYCLES"]

def main():
    rclpy.init()
    turning_layer = TurningLayer()
    rclpy.spin(turning_layer)

if __name__ == '__main__':
    main()