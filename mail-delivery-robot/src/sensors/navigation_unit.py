import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tools.nav_parser import loadConnections
from tools.map import Map

class NavigationUnit(Node):
    '''
    The Node in charge of navigation.

    @Subscribers:
    - Subscribes to /destinations for data about the robot's current destination.
    - Subscribes to /beacon_data for information about where the robot is.

    @Publishers:
    - Publishes navigation data to /navigation
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('navigation_unit')

        self.destinations_sub = self.create_subscription(String, 'destinations', self.destinations_callback, 10)
        self.beacon_data_sub = self.create_subscription(String, 'beacon_data', self.beacon_data_callback, 10)

        self.navigation_publisher = self.create_publisher(String, 'navigation', 10)
        self.navigation_timer = self.create_timer(1, self.update_navigation)

        self.beacon_connections = loadConnections()
        self.map = Map()

        self.current_destination = None
        self.current_beacon = None
        self.prev_beacon = None
        self.direction = None
        self.can_send_direction = False

        self.left_msg = String()
        self.left_msg.data = 'LEFT_TURN'
        self.right_msg = String()
        self.right_msg.data = 'RIGHT_TURN'
        self.straight_msg = String()
        self.straight_msg.data = 'STRAIGHT'
        self.uturn_msg = String()
        self.uturn_msg.data = 'U_TURN'
        self.dock_msg = String()
        self.dock_msg.data = 'DOCK'
        self.no_msg = String()
        self.no_msg.data = 'NONE'
    
    def destinations_callback(self, data):
        #works
        '''
        The callback for /destinations.
        Reads the robot's current destination when one is published.
        '''
        self.prev_beacon = data.data.split(':')[0]
        self.current_destination = data.data.split(':')[1]

    
    def beacon_data_callback(self, data):
        '''
        The callback for /beacon_data.
        Reads information about nearby beacons.
        '''
        # No trip was defined
        #self.get_logger().info(f"Beacon Data Received: {data.data}")
        #self.get_logger().info(f"Current Destination: {self.current_destination}, Previous Beacon: {self.prev_beacon}")
        if self.current_destination is None or self.prev_beacon is None:
            return
        
        beacon_orientation = "0"

        self.current_beacon = data.data.split(',')[0]

        if self.current_beacon == self.prev_beacon:
            #self.get_logger().info("============Same beacon as before, no movement detected.=============")
            return
        else:
            beacon_orientation = self.beacon_connections[self.current_beacon][self.prev_beacon]
            if beacon_orientation == "-":
                self.get_logger().info("ERROR: ROBOT HAS BEEN MOVED")
                # Finds a valid orientation for the robot.
                for i in range(1, 5):
                    if self.map.exists(self.current_beacon + str(i)):
                        beacon_orientation = str(i)
                        break
            #self.get_logger().info(f"Current Beacon: {self.current_beacon}, Prev Beacon: {self.prev_beacon}, Destination: {self.current_destination}, Beacon Orientation: {beacon_orientation}")
            self.direction = self.map.getDirection(self.current_beacon + beacon_orientation, self.current_destination)
            self.get_logger().info(f"Determined Direction: {self.direction}")
            self.can_send_direction = True
        self.prev_beacon = self.current_beacon

    def update_navigation(self):
        '''
        The timer callback. Sends updates to /navigation when necessary.
        '''
        #wait for things to be initialized
        #this is broken
        #self.get_logger().info(f"Current Direction: {self.direction}, Can Send: {self.can_send_direction}")
        if self.direction is not None and self.can_send_direction:
            #don't send the message more than once
            self.can_send_direction = False
            #translate from old to new naming convention
            match self.direction:
                case 'NAV_LEFT':
                    self.navigation_publisher.publish(self.left_msg)
                case 'NAV_RIGHT':
                    self.navigation_publisher.publish(self.right_msg)
                case 'NAV_PASS':
                    self.navigation_publisher.publish(self.straight_msg)
                case 'NAV_U-TURN':
                    self.navigation_publisher.publish(self.uturn_msg)
                case 'NAV_DOCK':
                    self.navigation_publisher.publish(self.dock_msg)
                case _:
                    #error
                    self.navigation_publisher.publish(self.no_msg)
                    

            

def main():
    rclpy.init()
    navigation_unit = NavigationUnit()
    rclpy.spin(navigation_unit)

if __name__ == '__main__':
    main()