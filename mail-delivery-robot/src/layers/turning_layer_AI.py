import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tools.csv_parser import loadConfig
import ollama


class TurningLayerAI(Node):
    '''
    AI-enhanced subsumption layer responsible for moving the robot through intersections.

    @Subscribers:
    - /navigation: next action at an intersection
    - /intersection_detection: whether the robot is in an intersection
    - /scan: LIDAR data for situational awareness

    @Publishers:
    - /actions: turning actions
    '''

    def __init__(self):
        super().__init__('turning_layer_AI')

        self.config = loadConfig()

        self.last_nav_msg = None
        self.in_intersection = False
        self.nav_message_handled = False
        self.latest_lidar = None

        self.actions = {
            "U_TURN": String(data="2:U_TURN"),
            "LEFT_TURN": String(data="2:LEFT_TURN"),
            "RIGHT_TURN": String(data="2:RIGHT_TURN"),
            "STRAIGHT": String(data="2:GO")
        }
        self.no_msg = String(data="2:NONE")
        self.go_msg = String(data="2:GO")

        # Subscribers
        self.navigation_sub = self.create_subscription(
            String, 'navigation', self.navigation_callback, 10)
        self.intersection_detection_sub = self.create_subscription(
            String, 'intersection_detection', self.intersection_detection_callback, 10)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, qos)

        # Publisher
        self.action_publisher = self.create_publisher(String, 'actions', 10)

        # Cycle counters from config
        self.turn_cycles = self.config["TURN_CYCLES"]
        self.u_turn_cycles = self.config["U_TURN_CYCLES"]
        self.go_cycles = self.config["TURNING_GO_CYCLES"]

        self.current_ai_decision = None
        self.timer = self.create_timer(0.2, self.update_actions)

    def navigation_callback(self, data):
        self.last_nav_msg = data.data.upper()
        self.nav_message_handled = False
        self.current_ai_decision = None  # Reset AI decision for new nav command

    def intersection_detection_callback(self, data):
        self.in_intersection = data.data.upper() == "TRUE"

    def lidar_callback(self, data):
        self.latest_lidar = data

    def lidar_summary(self, scan):
        """Summarize LIDAR ranges into zones for the LLM."""
        if scan is None or len(scan.ranges) == 0:
            return "Lidar data unavailable."

        n = len(scan.ranges)
        front = min(scan.ranges[0:n // 8] + scan.ranges[7 * n // 8:n])
        left = min(scan.ranges[n // 8: 3 * n // 8])
        back = min(scan.ranges[3 * n // 8: 5 * n // 8])
        right = min(scan.ranges[5 * n // 8: 7 * n // 8])

        return (
            f"LIDAR distances (meters):\n"
            f"  Front: {front:.2f}\n"
            f"  Left:  {left:.2f}\n"
            f"  Right: {right:.2f}\n"
            f"  Back:  {back:.2f}"
        )

    def ai_turning_query(self, nav_command):
        """Ask the LLM whether to confirm or override the navigation command."""
        self.get_logger().info(f"Querying Ollama to validate turn: {nav_command}")
        try:
            lidar_text = (
                self.lidar_summary(self.latest_lidar)
                if self.latest_lidar is not None
                else "Lidar data unavailable."
            )

            response = ollama.chat(model='qwen2:0.5b', messages=[
                {
                    "role": "system",
                    "content": (
                        "You are a turning controller for a small mail-delivery robot "
                        "navigating hallway intersections. Your job is to validate or "
                        "override the planned turn based on LIDAR sensor data."
                    )
                },
                {
                    "role": "user",
                    "content": (
                        f"The navigation system has commanded: {nav_command}.\n\n"
                        f"{lidar_text}\n\n"
                        "Rules:\n"
                        "- If the commanded direction is safe (distance > 0.5m), CONFIRM it.\n"
                        "- If the commanded direction is blocked, choose the safest alternative.\n"
                        "- Valid responses: LEFT_TURN, RIGHT_TURN, U_TURN, STRAIGHT\n\n"
                        "Respond with ONLY ONE of: LEFT_TURN, RIGHT_TURN, U_TURN, STRAIGHT"
                    )
                }
            ])

            decision = response['message']['content'].strip().upper()
            self.get_logger().info(f"Ollama turning decision: {decision}")

            # Validate the LLM response
            if decision in ["LEFT_TURN", "RIGHT_TURN", "U_TURN", "STRAIGHT"]:
                return decision
            else:
                self.get_logger().warning(
                    f"Invalid LLM response: {decision}. Falling back to nav command.")
                return nav_command

        except Exception as e:
            self.get_logger().error(f"Ollama connection failed: {e}")
            return nav_command  # Safe fallback: trust the navigation system

    def update_actions(self):
        # Handle U-turns
        if self.last_nav_msg == "U_TURN" and not self.nav_message_handled:
            if self.current_ai_decision is None:
                self.current_ai_decision = self.ai_turning_query("U_TURN")

            action_key = self.current_ai_decision

            if action_key == "U_TURN":
                if self.u_turn_cycles > 0:
                    self.action_publisher.publish(self.actions["U_TURN"])
                    self.u_turn_cycles -= 1
                    return
                else:
                    self.action_publisher.publish(self.no_msg)
                    self.nav_message_handled = True
                    self.u_turn_cycles = self.config["U_TURN_CYCLES"]
                    return
            else:
                # AI overrode U_TURN to something else — treat as regular turn
                self.last_nav_msg = action_key

        if not self.in_intersection:
            self.action_publisher.publish(self.no_msg)
            return

        if self.last_nav_msg is not None and not self.nav_message_handled:
            # Query AI once per navigation command
            if self.current_ai_decision is None:
                self.current_ai_decision = self.ai_turning_query(self.last_nav_msg)

            action_key = self.current_ai_decision

            if self.turn_cycles > 0:
                if action_key in self.actions:
                    self.action_publisher.publish(self.actions[action_key])
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
    turning_layer = TurningLayerAI()
    rclpy.spin(turning_layer)


if __name__ == '__main__':
    main()