from geometry_msgs.msg import Twist
from tools.csv_parser import loadConfig

class ActionTranslator():
    def __init__(self):
        self.msg = Twist()
        self.config = loadConfig()

    def translate_action(self, action):
        '''
        Translates an action message into a Twist command.
        This translator performs no computation; it simply maps the provided
        movement orders to the appropriate Twist fields.
        
        Expected action formats:
        - "GO"         : Standard forward motion.
        - "WAIT"       : Stop moving.
        - "BACK"       : Reverse.
        - "LEFT_TURN"  : Turn left.
        - "RIGHT_TURN" : Turn right.
        - "WALL_FOLLOW:<linear_speed>:<angular_speed>" : Use the provided speeds.
        
        Returns:
        A Twist message with the movement order.
        '''
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        match action:
            case 'GO':
                self.msg.linear.x = self.config["GO_MSG_LIN_SPEED"]
            case 'WAIT':
                self.msg.linear.x = 0.0
            case 'BACK':
                self.msg.linear.x = self.config["BACK_MSG_LIN_SPEED"]
            case 'LEFT_TURN':
                self.msg.angular.z = self.config["LEFT_TURN_ANG_SPEED"]
            case 'RIGHT_TURN':
                self.msg.angular.z = self.config["RIGHT_TURN_ANG_SPEED"]
            case 'U_TURN':
                self.msg.angular.z = self.config["U_TURN_ANG_SPEED"]
            case _:
                try:
                    parts = action.split(",")
                    # Expecting format: 3:WALL_FOLLOW:<linear_speed>:<angular_speed>
                    self.msg.linear.x = float(parts[1])
                    self.msg.angular.z = float(parts[2])
                except (IndexError, ValueError):
                    # If parsing fails, set speeds to zero.
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
        return self.msg

