import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tools.csv_parser import loadConfig
import ollama
import time

class DockingLayerStates(Enum):
    NO_DEST = 'NO_DEST'
    HAS_DEST = 'HAS_DEST'
    DOCKING = 'DOCKING'

class DockingLayerAI(Node):
    '''
    The subsumption layer is responsible for moving the robot onto and off the dock.
    Enhanced with LLM for intelligent docking decisions.

    @Subscribers:
    - Listens to /navigation for data about the next actions the robot should take
    - Listens to /dock_status for data about nearby docking stations and the robot's dock status
    - Listens to /lidar_data for environmental awareness during docking

    @Publishers:
    - Publishes actions to /actions
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('docking_layer_ai')

        self.state = DockingLayerStates.NO_DEST
        self.last_navigation_message = 'NONE'
        self.dock_visible = False
        self.is_docked = False
        self.latest_lidar = None
        self.current_llm_decision = None
        self.docking_attempts = 0
        self.max_docking_attempts = 3
        self.llm_response_latencies = []

        self.config = loadConfig()

        # Subscribers
        self.navigation_sub = self.create_subscription(String, 'navigation', self.navigation_callback, 10)
        self.dock_status_sub = self.create_subscription(DockStatus, 'dock_status', self.dock_status_callback, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        ))
        self.lidar_sensor_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)

        # Publisher
        self.action_publisher = self.create_publisher(String, 'actions', 10)

        self.audio_publisher = self.create_publisher(String, 'llm_audio_event', 10)

        # Action messages
        self.no_msg = String()
        self.no_msg.data = '1:NONE'
        self.dock_msg = String()
        self.dock_msg.data = '1:DOCK'
        self.undock_msg = String()
        self.undock_msg.data = '1:UNDOCK'
        self.wait_msg = String()
        self.wait_msg.data = '1:WAIT'
        self.left_turn_msg = String()
        self.left_turn_msg.data = '1:LEFT_TURN'
        self.right_turn_msg = String()
        self.right_turn_msg.data = '1:RIGHT_TURN'
        self.back_msg = String()
        self.back_msg.data = '1:BACK'

        self.timer = self.create_timer(0.1, self.update_actions)

        self.action_publisher.publish(self.no_msg)

    def navigation_callback(self, data):
        '''
        The callback for /navigation
        Reads navigation messages sent by the navigation unit.
        '''
        self.last_navigation_message = data.data

    def dock_status_callback(self, data):
        '''
        The callback for /dock_status.
        Reads information about nearby docks and the robot's current dock status.
        '''
        self.dock_visible = data.dock_visible
        self.is_docked = data.is_docked

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

    def get_docking_context(self):
        '''
        Creates a summary of the current docking situation for the LLM.
        '''
        lidar_summary = "Lidar data unavailable."
        if self.latest_lidar is not None:
            lidar_summary = f"""
            Lidar summary:
            - Front distance: {self.latest_lidar['front']:.2f}m
            - Left distance: {self.latest_lidar['left']:.2f}m
            - Right distance: {self.latest_lidar['right']:.2f}m
            """

        return f"""
        Docking Status:
        - Dock visible: {self.dock_visible}
        - Currently docked: {self.is_docked}
        - Docking attempts so far: {self.docking_attempts}
        - Navigation command: {self.last_navigation_message}

        {lidar_summary}
        """

    def ai_docking_query(self, situation: str):
        '''
        Query the LLM for docking decisions.

        @param situation: The specific situation requiring a decision
        @return: The LLM's decision
        '''
        self.get_logger().info(f"Querying Ollama for docking decision: {situation}")
        start = time.perf_counter()
        try:
            context = self.get_docking_context()

            response = ollama.chat(model='gemma2:2b-instruct-q4_0', messages=[
                {
                    "role": "system",
                    "content": (
                        "You are a docking controller for a mail delivery robot. "
                        "You help the robot dock and undock safely at charging stations. "
                        "You must make quick, safe decisions based on sensor data."
                    )
                },
                {
                    "role": "user",
                    "content": f"""
                    Situation: {situation}

                    {context}

                    Rules:
                    - If dock is visible and path is clear (front > 0.5m), proceed with DOCK
                    - If dock is not visible, suggest WAIT or ADJUST (LEFT/RIGHT) to find it
                    - If too many failed attempts, suggest BACK to reposition
                    - Safety is the priority

                    Respond with ONLY ONE word:
                    DOCK, UNDOCK, WAIT, LEFT, RIGHT, BACK, or NONE
                    """
                }
            ])

            elapsed = time.perf_counter() - start
            self.record_llm_latency(elapsed, context="ai_docking_query")
            decision = response['message']['content'].strip().upper()
            self.get_logger().info(f"Ollama docking decision: {decision}")
            return decision
        except Exception as e:
            elapsed = time.perf_counter() - start
            self.record_llm_latency(elapsed, context="ai_docking_query_error")
            self.get_logger().error(f"Ollama connection failed: {e}")
            return "FALLBACK"  # Will use original logic

    def record_llm_latency(self, elapsed_s: float, context: str = "llm_call"):
        self.llm_response_latencies.append(elapsed_s)
        # Keep `latency=<number>s` format so dashboard_logger can parse from /rosout.
        self.get_logger().info(f"{context}: latency={elapsed_s:.3f}s")

    def get_llm_response_latencies(self):
        return list(self.llm_response_latencies)

    def fallback_docking_logic(self):
        '''
        Original docking logic as a fallback when LLM is unavailable.
        '''
        if self.dock_visible:
            return "DOCK"
        elif self.is_docked:
            return "DONE"
        else:
            return "NONE"

    def update_actions(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /actions when necessary. Uses LLM for intelligent decisions.
        '''
        if self.state == DockingLayerStates.NO_DEST and self.last_navigation_message != 'NONE':
            self.state = DockingLayerStates.HAS_DEST
            self.docking_attempts = 0
            if self.is_docked:
                # Query LLM for undocking decision
                decision = self.ai_docking_query("Robot needs to undock to start delivery")
                if decision == "FALLBACK" or "UNDOCK" in decision:
                    self.action_publisher.publish(self.undock_msg)
                else:
                    self.action_publisher.publish(self.undock_msg)  # Default to undock

        elif self.state == DockingLayerStates.HAS_DEST and self.last_navigation_message == 'DOCK':
            # Use LLM to decide docking action
            if self.current_llm_decision is None:
                self.action_publisher.publish(self.wait_msg)  # Wait while querying
                self.current_llm_decision = self.ai_docking_query("Robot needs to dock at charging station")

            # Handle LLM decision or fallback
            if self.current_llm_decision == "FALLBACK":
                # Use original logic
                self.get_logger().warning("FALLBACK: Using rule-based docking logic.")
                decision = self.fallback_docking_logic()
            else:
                decision = self.current_llm_decision

            # Execute decision
            if "DOCK" in decision and self.dock_visible:
                self.action_publisher.publish(self.dock_msg)
                self.docking_attempts += 1
            elif "LEFT" in decision:
                self.action_publisher.publish(self.left_turn_msg)
            elif "RIGHT" in decision:
                self.action_publisher.publish(self.right_turn_msg)
            elif "BACK" in decision:
                self.action_publisher.publish(self.back_msg)
                self.docking_attempts += 1
            elif "WAIT" in decision:
                self.action_publisher.publish(self.wait_msg)
            elif self.is_docked:
                # Successfully docked
                self.state = DockingLayerStates.NO_DEST
                self.current_llm_decision = None
                self.docking_attempts = 0
            else:
                self.action_publisher.publish(self.no_msg)

            # Reset LLM decision for next cycle if we need to re-evaluate
            if self.docking_attempts >= self.max_docking_attempts:
                self.current_llm_decision = None  # Re-query LLM
                self.docking_attempts = 0

        else:
            self.action_publisher.publish(self.no_msg)
            self.current_llm_decision = None


def main():
    rclpy.init()
    docking_layer = DockingLayerAI()
    rclpy.spin(docking_layer)

if __name__ == '__main__':
    main()
