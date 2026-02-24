import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tools.csv_parser import loadConfig
import ollama


class TravelLayerStates(Enum):
    NO_DEST = 'NO_DEST'
    HAS_DEST = 'HAS_DEST'


class TravelLayerAI(Node):
    '''
    The subsumption layer responsible for moving the robot forward,
    performing wall following, etc.

    @Subscribers:
    - /lidar_data: nearby wall data
    - /destinations: current destination
    - /dock_status: docked status
    - /navigation: navigation commands

    @Publishers:
    - /actions: action messages
    '''
    def __init__(self):
        super().__init__('travel_layer')
        self.config = loadConfig()

        self.state = TravelLayerStates.NO_DEST
        self.current_destination = 'NONE'
        self.is_docked = False
        self.was_docked = False
        self.latest_lidar = None
        self.current_llm_decision = None   # Cached LLM travel decision

        self.lidar_data_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)
        self.destinations_sub = self.create_subscription(String, 'destinations', self.destinations_callback, 10)
        self.navigation_sub = self.create_subscription(String, 'navigation', self.navigation_callback, 10)

        self.dock_status_sub = self.create_subscription(
            DockStatus, 'dock_status', self.dock_status_callback,
            qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        )

        self.action_publisher = self.create_publisher(String, 'actions', 10)

        self.go_msg = String()
        self.go_msg.data = '3:GO'
        self.no_msg = String()
        self.no_msg.data = '3:NONE'

        self.timer = self.create_timer(0.2, self.update_actions)
        self.action_publisher.publish(self.no_msg)

    def lidar_data_callback(self, data):
        try:
            parts = data.data.split(":")
            if len(parts) != 5:
                self.get_logger().warning("Lidar data format incorrect")
                return
            self.latest_lidar = {
                "feedback": float(parts[0]),
                "angle":    float(parts[1]),
                "right":    float(parts[2]),
                "left":     float(parts[3]),
                "front":    float(parts[4])
            }
        except Exception as e:
            self.get_logger().error(f"Error parsing lidar data: {e}")
            self.latest_lidar = None

    def destinations_callback(self, data):
        try:
            new_dest = data.data.split(":")[1]
            if new_dest != self.current_destination:
                # Destination changed — invalidate cached LLM decision
                self.current_llm_decision = None
                self.get_logger().info(f"Destination changed to: {new_dest}, clearing LLM cache")
            self.current_destination = new_dest
        except Exception as e:
            self.get_logger().error(f"Error parsing destination: {e}")
            self.current_destination = 'NONE'

    def dock_status_callback(self, data):
        self.was_docked = self.is_docked
        self.is_docked = data.is_docked

    def navigation_callback(self, msg: String):
        if msg.data == 'DOCK':
            dock_msg = String()
            dock_msg.data = '3:DOCK'
            self.action_publisher.publish(dock_msg)

    def lidar_summary(self, lidar):
        closest = min({"LEFT": lidar["left"], "RIGHT": lidar["right"], "FRONT": lidar["front"]},
                      key=lambda k: lidar[k.lower()] if lidar[k.lower()] > 0 else float('inf'))
        most_open = max({"LEFT": lidar["left"], "RIGHT": lidar["right"], "FRONT": lidar["front"]},
                        key=lambda k: lidar[k.lower()])
        return (
            f"Lidar summary:\n"
            f"  - Front distance: {lidar['front']:.2f}\n"
            f"  - Left distance:  {lidar['left']:.2f}\n"
            f"  - Right distance: {lidar['right']:.2f}\n"
            f"  Closest obstacle: {closest} direction.\n"
            f"  Most open area:   {most_open} direction."
        )

    def ai_travel_query(self):
        self.get_logger().info("Querying Ollama for travel direction...")
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
                        "You are a navigation controller for a small mail-delivery robot "
                        "navigating narrow hallways. Choose the safest forward movement."
                    )
                },
                {
                    "role": "user",
                    "content": (
                        f"The robot is navigating toward destination: {self.current_destination}.\n\n"
                        f"{lidar_text}\n\n"
                        "Rules:\n"
                        "- Prefer WALL_FOLLOW if the path ahead is reasonably clear.\n"
                        "- Choose GO if the front is wide open with no walls close by.\n"
                        "- Choose LEFT_TURN or RIGHT_TURN only if the front is blocked.\n"
                        "- Do NOT choose a direction with the closest obstacle.\n\n"
                        "Respond with ONLY ONE word: WALL_FOLLOW, GO, LEFT_TURN, or RIGHT_TURN"
                    )
                }
            ])

            decision = response['message']['content'].strip().upper()
            self.get_logger().info(f"Ollama travel decision: {decision}")
            return decision
        except Exception as e:
            self.get_logger().error(f"Ollama connection failed: {e}")
            return "WALL_FOLLOW"  # Safe fallback to existing behaviour


    def compute_wall_follow(self):
        if self.latest_lidar is None:
            return None

        cur_distance = self.latest_lidar["feedback"]
        cur_angle = self.latest_lidar["angle"]

        SET_POINT = self.config["WALL_FOLLOW_SET_POINT"]
        AIM_ANGLE = self.config["WALL_FOLLOW_AIM_ANGLE"]
        ERROR = self.config["WALL_FOLLOW_SPEED"] * math.sin(math.radians(AIM_ANGLE))

        if cur_angle > 180:
            cur_angle -= 360

        if cur_distance > SET_POINT + ERROR:
            res_angle = -1 * AIM_ANGLE + cur_angle
        elif cur_distance < SET_POINT - ERROR:
            res_angle = AIM_ANGLE + cur_angle
        else:
            res_angle = cur_angle

        angular_speed = math.radians(res_angle)
        if abs(angular_speed) > self.config["WALL_FOLLOW_ANGLE_CHANGE_THRESHOLD"]:
            linear_speed = self.config["WALL_FOLLOW_SPEED"] / 2
        else:
            linear_speed = self.config["WALL_FOLLOW_SPEED"]

        return linear_speed, angular_speed

    def update_actions(self):
        # State transitions
        if self.state == TravelLayerStates.NO_DEST and self.current_destination != 'NONE':
            self.state = TravelLayerStates.HAS_DEST
        elif self.state == TravelLayerStates.HAS_DEST and self.current_destination == 'NONE':
            self.state = TravelLayerStates.NO_DEST
            self.current_llm_decision = None 

        if self.state == TravelLayerStates.HAS_DEST and not self.is_docked:
            # Query LLM once per destination (cached after first call)
            if self.current_llm_decision is None:
                self.action_publisher.publish(self.no_msg)  # Pause while deciding
                self.current_llm_decision = self.ai_travel_query()

            action_msg = String()

            if "WALL_FOLLOW" in self.current_llm_decision:
                speeds = self.compute_wall_follow()
                if speeds is not None:
                    linear_speed, angular_speed = speeds
                    action_msg.data = f'3:WALL_FOLLOW,{linear_speed},{angular_speed}'
                else:
                    action_msg.data = '3:GO'
            elif "LEFT_TURN" in self.current_llm_decision:
                action_msg.data = '3:LEFT_TURN'
            elif "RIGHT_TURN" in self.current_llm_decision:
                action_msg.data = '3:RIGHT_TURN'
            elif "GO" in self.current_llm_decision:
                action_msg.data = '3:GO'
            else:
                action_msg.data = '3:GO'

            self.action_publisher.publish(action_msg)

        if self.is_docked and not self.was_docked:
            self.action_publisher.publish(self.no_msg)


def main():
    rclpy.init()
    travel_layer = TravelLayerAI()
    rclpy.spin(travel_layer)


if __name__ == '__main__':
    main()
