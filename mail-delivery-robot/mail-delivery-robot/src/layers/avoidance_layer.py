import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensors.bumper_sensor import Bump_Event
from enum import Enum
from tools.csv_parser import loadConfig

class AvoidanceLayerStates(Enum):
    '''
    An enum for the internal states of the avoidance layer.
    '''
    COLLISION = "COLLISION"
    NO_COLLISION = "NO_COLLISION"

class AvoidanceLayer(Node):
    '''
    The subsumption layer responsible for obstacle avoidance.

    @Subscribers:
    - Listens to /bumper_data for collision detection

    @Publishers:
    - Publishes actions to /actions
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('avoidance_layer')

        self.state = AvoidanceLayerStates.NO_COLLISION
        self.bump_data = False

        self.config = loadConfig()

        self.bumper_data_sub = self.create_subscription(String, 'bumper_data', self.bumper_data_callback, 10)
        
        self.action_publisher = self.create_publisher(String, 'actions', 10)
        
        self.wait_msg = String()
        self.wait_msg.data = '0:WAIT'
        self.no_msg = String()
        self.no_msg.data = '0:NONE'
        self.back_msg = String()
        self.back_msg.data = '0:BACK'
        self.left_turn_msg = String()
        self.left_turn_msg.data = '0:LEFT_TURN'
        self.right_turn_msg = String()
        self.right_turn_msg.data = '0:RIGHT_TURN'
        self.go_msg = String()
        self.go_msg.data = '0:GO'

        self.timer = self.create_timer(0.2, self.update_actions)
        self.bump_counter_reduce_timer = self.create_timer(self.config["BUMP_COUNTER_REDUCE_TIMER"], self.bump_counter_reduce)
        self.delay_counter = self.config["AVOIDANCE_DELAY"]
        self.bump_counter = 0
        self.pause_bump_counter = False

        self.action_publisher.publish(self.no_msg)

    def bumper_data_callback(self, data):
        '''
        The callback for /bumper_data.
        Reads and updates the information sent by the bumper sensor.

        @param data: The data sent by the bumper sensor.
        '''
        bumpData = str(data.data)
        if bumpData == Bump_Event.PRESSED.value:
            self.get_logger().info("GOT COLLISION")
            self.bump_data = True
        else:
            self.bump_data = False
    
    def bump_counter_reduce(self):
        '''
        The timer callback to reduce the bump counter by 1.
        '''
        if self.bump_counter > 0 and not self.pause_bump_counter:
            self.bump_counter -= 1

    def update_actions(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /actions when necessary
        '''
        if self.state == AvoidanceLayerStates.NO_COLLISION and self.bump_data:
            #Bumper sensor was triggered, transition from state NO_COLLISION to state COLLISION
            self.state = AvoidanceLayerStates.COLLISION
            self.delay_counter = self.config["AVOIDANCE_DELAY"]
            self.bump_counter += 1
        elif self.state == AvoidanceLayerStates.COLLISION and self.delay_counter:
            #Begin sending instructions to deal with the collision
            if self.bump_counter < self.config["MAX_BUMPS_BEFORE_AVOID"]:
                self.action_publisher.publish(self.wait_msg)
            else:
                self.pause_bump_counter = True
                if self.delay_counter == self.config["AVOIDANCE_DELAY"]:
                    self.action_publisher.publish(self.left_turn_msg)
                elif self.delay_counter == (self.config["AVOIDANCE_DELAY"] - self.config["AVOIDANCE_STEP"]):
                    self.action_publisher.publish(self.go_msg)
                elif self.delay_counter == (self.config["AVOIDANCE_DELAY"] - (self.config["AVOIDANCE_STEP"] * 2)):
                    self.action_publisher.publish(self.right_turn_msg)
                elif self.delay_counter == (self.config["AVOIDANCE_DELAY"] - (self.config["AVOIDANCE_STEP"] * 3)):
                    self.action_publisher.publish(self.go_msg)
                elif self.delay_counter == (self.config["AVOIDANCE_DELAY"] - (self.config["AVOIDANCE_STEP"] * 4)):
                    self.action_publisher.publish(self.right_turn_msg)
                elif self.delay_counter == (self.config["AVOIDANCE_DELAY"] - (self.config["AVOIDANCE_STEP"] * 5)):
                    self.action_publisher.publish(self.go_msg)
                elif self.delay_counter == (self.config["AVOIDANCE_DELAY"] - (self.config["AVOIDANCE_STEP"] * 6)):
                    self.action_publisher.publish(self.left_turn_msg)
                    self.bump_counter = 0
                    self.pause_bump_counter = False
            self.delay_counter -= 1
        elif self.state == AvoidanceLayerStates.COLLISION:
            #Collision has resolved, transition to state NO_COLLISION, and
            #IMPORTANT: send NONE action message when the subroutine resolves,
            #otherwise the captain would continue to execute the last instruction
            self.state = AvoidanceLayerStates.NO_COLLISION
            self.action_publisher.publish(self.no_msg)
            self.delay_counter = self.config["AVOIDANCE_DELAY"]

def main():
    rclpy.init()
    avoidance_layer = AvoidanceLayer()
    rclpy.spin(avoidance_layer)

if __name__ == '__main__':
    main()