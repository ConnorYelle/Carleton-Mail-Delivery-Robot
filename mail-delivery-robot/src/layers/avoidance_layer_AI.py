import time
from enum import Enum

import ollama
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sensors.bumper_sensor import Bump_Event
from tools.csv_parser import loadConfig


class AvoidanceLayerStates(Enum):
    COLLISION = "COLLISION"
    NO_COLLISION = "NO_COLLISION"


class AvoidanceLayerAI(Node):

    def __init__(self):
        super().__init__("avoidance_layer_AI")

        self.state = AvoidanceLayerStates.NO_COLLISION
        self.bump_data = False
        self.latest_lidar = None
        self.current_llm_decision = None
        self.llm_response_latencies = []

        self.config = loadConfig()

        # Subscribers
        self.bumper_data_sub = self.create_subscription(
            String, "bumper_data", self.bumper_data_callback, 10
        )

        self.lidar_sensor_sub = self.create_subscription(
            String, "lidar_data", self.lidar_data_callback, 10
        )

        # Publisher
        self.action_publisher = self.create_publisher(String, "actions", 10)

        # Predefined action messages
        self.wait_msg = String()
        self.wait_msg.data = "0:WAIT"

        self.no_msg = String()
        self.no_msg.data = "0:NONE"

        self.back_msg = String()
        self.back_msg.data = "0:BACK"

        self.left_turn_msg = String()
        self.left_turn_msg.data = "0:LEFT_TURN"

        self.right_turn_msg = String()
        self.right_turn_msg.data = "0:RIGHT_TURN"

        self.go_msg = String()
        self.go_msg.data = "0:GO"

        self.timer = self.create_timer(0.2, self.update_actions)

        self.delay_counter = self.config["AVOIDANCE_DELAY"]

        self.action_publisher.publish(self.no_msg)

    def bumper_data_callback(self, data):
        bumpData = str(data.data)

        if bumpData == Bump_Event.PRESSED.value:
            self.get_logger().info("GOT COLLISION")
            self.bump_data = True
        else:
            self.bump_data = False

    def lidar_data_callback(self, data):
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
                "front": float(parts[4]),
            }

        except Exception as e:
            self.get_logger().error(f"Error parsing lidar data: {e}")
            self.latest_lidar = None

    def get_closest_obstacle(self, lidarData):
        distances = {
            "LEFT": lidarData["left"],
            "RIGHT": lidarData["right"],
            "FRONT": lidarData["front"],
        }

        valid = {k: v for k, v in distances.items() if v > 0}
        if not valid:
            return "UNKNOWN"

        return min(valid, key=valid.get)

    def get_most_space(self, lidarData):
        distances = {
            "LEFT": lidarData["left"],
            "RIGHT": lidarData["right"],
            "FRONT": lidarData["front"],
        }

        valid = {k: v for k, v in distances.items() if v > 0}
        if not valid:
            return "UNKNOWN"

        return max(valid, key=valid.get)

    def lidar_summary(self, lidarData):
        return f"""
Lidar summary:

Front distance: {lidarData['front']:.2f}
Left distance: {lidarData['left']:.2f}
Right distance: {lidarData['right']:.2f}

Closest obstacle: {self.get_closest_obstacle(lidarData)}
Most open direction: {self.get_most_space(lidarData)}
"""

    def ai_avoidance_query(self):

        self.get_logger().info("Querying Ollama for collision resolution...")
        start = time.perf_counter()

        try:
            lidar_text = (
                self.lidar_summary(self.latest_lidar)
                if self.latest_lidar is not None
                else "Lidar data unavailable."
            )

            background = """
            You are an obstacle-avoidance controller
            for a small robot navigating narrow tunnels.
            Choose the safest movement."
            """

            prompt = f"""
The robot has collided with an obstacle.

{lidar_text}

Rules:
- Do NOT choose the closest obstacle direction.
- Prefer the most open direction.
- If FRONT is most open, choose GO.

Respond with ONLY ONE word:
BACK, LEFT, RIGHT, or GO
"""

            response = ollama.chat(
                model="qwen2:0.5b",
                messages=[
                    {"role": "system", "content": background},
                    {"role": "user", "content": prompt},
                ],
            )
            elapsed = time.perf_counter() - start
            self.record_llm_latency(elapsed, context="ai_avoidance_query")
            self.get_logger().info(f"Background: {background}")
            self.get_logger().info(f"Prompt: {prompt}")
            decision = response["message"]["content"].strip().upper()
            self.get_logger().info(f"Raw LLM Response: {response}")

            if decision not in ["LEFT", "RIGHT", "BACK", "GO"]:
                self.get_logger().warning(f"Invalid LLM response: {decision}")
                return "FAILED"

            self.get_logger().info(f"Ollama decided: {decision}")
            return decision

        except Exception as e:
            elapsed = time.perf_counter() - start
            self.record_llm_latency(elapsed, context="ai_avoidance_query_error")
            self.get_logger().error(f"Ollama connection failed: {e}")
            return "FAILED"

    def record_llm_latency(self, elapsed_s: float, context: str = "llm_call"):
        self.llm_response_latencies.append(elapsed_s)
        # Keep `latency=<number>s` format so dashboard_logger can parse from /rosout.
        self.get_logger().info(f"{context}: latency={elapsed_s:.3f}s")

    def get_llm_response_latencies(self):
        return list(self.llm_response_latencies)

    def rule_based_avoidance(self):

        if self.latest_lidar is None:
            self.get_logger().warning("No lidar data. Defaulting to BACK.")
            return "BACK"

        closest = self.get_closest_obstacle(self.latest_lidar)
        most_open = self.get_most_space(self.latest_lidar)

        if most_open == "FRONT":
            return "GO"

        if closest == "LEFT":
            return "RIGHT"
        elif closest == "RIGHT":
            return "LEFT"
        elif closest == "FRONT":
            return "BACK"

        return "BACK"

    # --------------------------------------------------
    # STATE MACHINE
    # --------------------------------------------------

    def update_actions(self):

        # Collision detected
        if self.state == AvoidanceLayerStates.NO_COLLISION and self.bump_data:

            self.get_logger().info("Collision detected. Stopping robot.")

            self.state = AvoidanceLayerStates.COLLISION
            self.delay_counter = self.config["AVOIDANCE_DELAY"]

            self.action_publisher.publish(self.wait_msg)

            decision = self.ai_avoidance_query()

            if decision == "FAILED":
                self.get_logger().warning("LLM failed. Using fallback.")
                decision = self.rule_based_avoidance()

            self.current_llm_decision = decision

        # Execute avoidance
        elif self.state == AvoidanceLayerStates.COLLISION and self.delay_counter > 0:

            decision = self.current_llm_decision or "BACK"

            if decision == "LEFT":
                self.action_publisher.publish(self.left_turn_msg)
            elif decision == "RIGHT":
                self.action_publisher.publish(self.right_turn_msg)
            elif decision == "BACK":
                self.action_publisher.publish(self.back_msg)
            elif decision == "GO":
                self.action_publisher.publish(self.go_msg)
            else:
                self.action_publisher.publish(self.back_msg)

            self.delay_counter -= 1

        # Reset state
        elif self.state == AvoidanceLayerStates.COLLISION:

            self.get_logger().info("Avoidance maneuver complete.")
            self.state = AvoidanceLayerStates.NO_COLLISION
            self.action_publisher.publish(self.no_msg)
            self.current_llm_decision = None


def main():
    rclpy.init()
    avoidance_layer = AvoidanceLayerAI()
    rclpy.spin(avoidance_layer)


if __name__ == "__main__":
    main()
