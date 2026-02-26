import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import ollama

class DockingLayerStates(Enum):
    '''
    An enum for the internal states of the docking layer.
    '''
    NO_DEST = 'NO_DEST'
    HAS_DEST = 'HAS_DEST'

class DockingLayerAI(Node):
    '''
    The subsumption layer responsible for moving the robot on to and off of the dock.
    Uses LLM to make intelligent docking decisions.

    @Subscribers:
    - Listens to /navigation for data about the next actions the robot should take
    - Listens to /dock_status for data about nearby docking stations and the robot's dock status

    @Publishers:
    - Publishes actions to /actions
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('docking_layer')

        self.state = DockingLayerStates.NO_DEST
        self.last_navigation_message = 'NONE'
        self.dock_visible = False
        self.is_docked = False
        self.current_llm_decision = None  # Cached LLM decision


        self.navigation_sub = self.create_subscription(String, 'navigation', self.navigation_callback, 10)
        self.dock_status_sub = self.create_subscription(DockStatus, 'dock_status', self.dock_status_callback, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        ))
        
        self.action_publisher = self.create_publisher(String, 'actions', 10,)

        self.no_msg = String()
        self.no_msg.data = '1:NONE'
        self.dock_msg = String()
        self.dock_msg.data = '1:DOCK'
        self.undock_msg = String()
        self.undock_msg.data = '1:UNDOCK'

        self.timer = self.create_timer(0.1, self.update_actions)

        self.action_publisher.publish(self.no_msg)

    def navigation_callback(self, data):
        '''
        The callback for /navigation
        Reads navigation messages sent by the navigation unit.
        '''
        if data.data != self.last_navigation_message:
            # Navigation changed — invalidate cached LLM decision
            self.current_llm_decision = None
        self.last_navigation_message = data.data

    def dock_status_callback(self, data):
        '''
        The callback for /dock_status.
        Reads information about nearby docks, and the robot's current dock status.
        '''
        self.dock_visible = data.dock_visible
        self.is_docked = data.is_docked

    def ai_docking_query(self):
        '''
        Query the LLM to make an intelligent docking decision based on current state.
        Returns a decision string: DOCK, UNDOCK, or WAIT
        '''
        self.get_logger().info("Querying Ollama for docking decision...")
        try:
            # Build context for the LLM
            context = f"""
            Current robot state:
            - Navigation message: {self.last_navigation_message}
            - Dock visible: {self.dock_visible}
            - Currently docked: {self.is_docked}
            - Current state: {self.state.value}
            """

            response = ollama.chat(model='qwen2:0.5b', messages=[
                {
                    "role": "system",
                    "content": (
                        "You are a docking controller for a mail delivery robot. "
                        "Your job is to decide when to dock, undock, or wait based on the robot's state."
                    )
                },
                {
                    "role": "user",
                    "content": f"""
                    {context}

                    Rules:
                    - If the robot is docked and needs to travel (HAS_DEST), choose UNDOCK.
                    - If navigation says DOCK and the dock is visible, choose DOCK.
                    - If navigation says DOCK but the dock is not visible yet, choose WAIT.
                    - If the robot is already docked and navigation says DOCK, choose WAIT.
                    - Otherwise, choose WAIT.

                    Respond with ONLY ONE word:
                    DOCK, UNDOCK, or WAIT
                    """
                }
            ])

            decision = response['message']['content'].strip().upper()
            self.get_logger().info(f"Ollama docking decision: {decision}")
            return decision
        except Exception as e:
            self.get_logger().error(f"Ollama connection failed: {e}")
            return "WAIT"  # Safe fallback

    def update_actions(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /actions when necessary
        '''
        # State transitions
        if self.state == DockingLayerStates.NO_DEST and self.last_navigation_message != 'NONE':
            self.state = DockingLayerStates.HAS_DEST
            self.current_llm_decision = None  # Clear cache on state change
            
        elif self.state == DockingLayerStates.HAS_DEST and self.last_navigation_message == 'NONE':
            self.state = DockingLayerStates.NO_DEST
            self.current_llm_decision = None  # Clear cache on state change

        # Decision making with LLM
        if self.state == DockingLayerStates.HAS_DEST:
            # Query LLM once and cache the decision
            if self.current_llm_decision is None:
                self.action_publisher.publish(self.no_msg)  # Pause while deciding
                self.current_llm_decision = self.ai_docking_query()

            # Execute the decision
            if "UNDOCK" in self.current_llm_decision:
                if self.is_docked:
                    self.action_publisher.publish(self.undock_msg)
                else:
                    self.action_publisher.publish(self.no_msg)
            elif "DOCK" in self.current_llm_decision:
                if self.dock_visible and not self.is_docked:
                    self.action_publisher.publish(self.dock_msg)
                elif self.is_docked:
                    # Successfully docked, transition to NO_DEST
                    self.state = DockingLayerStates.NO_DEST
                    self.action_publisher.publish(self.no_msg)
                    self.current_llm_decision = None
                else:
                    self.action_publisher.publish(self.no_msg)
            else:  # WAIT or any other response
                self.action_publisher.publish(self.no_msg)
        else:
            self.action_publisher.publish(self.no_msg)
        

def main():
    rclpy.init()
    docking_layer = DockingLayerAI()
    rclpy.spin(docking_layer)

if __name__ == '__main__':
    main()
