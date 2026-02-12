import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from control.action_translator import ActionTranslator
from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
import subprocess

class Captain(Node):
    '''
    The Node responsible for listening to actions from the
    subsumption layers and sending commands to the robot.

    @Subscribers:
    - Listens to /actions for new actions

    @Publishers:
    - Publishes commands to the robot to /cmd_vel
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('captain')

        self.current_actions = {
            '0' : 'NONE',
            '1' : 'NONE',
            '2' : 'NONE',
            '3' : 'NONE'
        }
        self.action_translator = ActionTranslator()

        self.actions_sub = self.create_subscription(String, 'actions', self.parse_action, 10)
        self.create_subscription(DockStatus, '/dock_status', self.dock_status_callback, 10)
        self.command_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.docking_client = ActionClient(self, Dock, 'dock')
        self.undocking_client = ActionClient(self, Undock, 'undock')
        self.dock_msg = Dock.Goal()
        self.undock_msg = Undock.Goal()
        self.dock_goal_future = None
        self.current_dock_state = False
        self.can_send_goal = True

        self.timer = self.create_timer(0.2, self.send_command)

    def parse_action(self, data):
        prio, action = data.data.split(':')
        self.current_actions[prio] = action

    def send_command(self):
        for action in self.current_actions.values():
            if action != 'NONE' and action != 'DOCK' and action != 'UNDOCK':
                command = self.action_translator.translate_action(action)
                self.command_publisher.publish(command)
                break
            elif action == 'DOCK':

                if self.current_dock_state:
                    self.can_send_goal = True
                    break

                if self.can_send_goal:
                    self.get_logger().info("Sending dock goal")
                    self.can_send_goal = False
                    self.dock_goal_future = self.docking_client.send_goal_async(
                        self.dock_msg,
                        feedback_callback=self.feedback_callback
                    )
                    self.dock_goal_future.add_done_callback(self.dock_goal_callback)
                    break
            elif action == 'UNDOCK':
                self.undocking_client.send_goal_async(self.undock_msg)
                break
        self.get_logger().info(str(self.current_actions))

    def dock_goal_callback(self, future):
        self.get_logger().info("got here")
        goal_handle = future.result()
        self.get_logger().info(f"Goal handle: {goal_handle}")
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(str(result))
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(str(feedback))

    def dock_status_callback(self, msg):
        self.current_dock_state = msg.is_docked
        if not msg.is_docked:
            self.can_send_goal = True

            
def main():
    rclpy.init()
    captain = Captain()
    rclpy.spin(captain)

if __name__ == '__main__':
    main()