import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensors.bumper_sensor import Bump_Event
from enum import Enum
from tools.csv_parser import loadConfig
import ollama

class AvoidanceLayerStates(Enum):
    '''
    An enum for the internal states of the avoidance layer.
    '''
    COLLISION = "COLLISION"
    NO_COLLISION = "NO_COLLISION"

class AvoidanceLayerAI(Node):
    '''
    The subsumption layer responsible for obstacle avoidance.

    @Subscribers:
    - Listens to /bumper_data for collision detection
    - Listens to /lidar_data for collision avoidance

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
        self.lidar_sensor_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)

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
        self.current_llm_decision = None

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

    def get_closest_obstacle(self, lidarData):
        
        distances = {
            "LEFT": lidarData["left"],
            "RIGHT": lidarData["right"],
            "FRONT": lidarData["front"]
        }
        valid = {k: v for k, v in distances.items() if v > 0}
        if not valid:
            return "UNKNOWN"
        return min(valid, key=valid.get)
    
    def get_most_space(self, lidarData):

        distances = {
            "LEFT": lidarData["left"],
            "RIGHT": lidarData["right"],
            "FRONT": lidarData["front"]
        }

        valid = {k: v for k, v in distances.items() if v > 0}
        if not valid:
            return "UNKNOWN"
        return max(valid, key=valid.get)
    
    def lidar_summary(self, lidarData):
        return f"""
        Lidar summary:

        - Front distance: {lidarData['front']:.2f}
        - Left disatance: {lidarData['left']:.2f}
        - Right distance: {lidarData['right']:.2f}

        Closest obstacle is in the {self.get_closest_obstacle(lidarData)} direction.
        Most open area is in the {self.get_most_space(lidarData)} direction.
        """
        
    def bump_counter_reduce(self):
        '''
        The timer callback to reduce the bump counter by 1.
        '''
        if self.bump_counter > 0 and not self.pause_bump_counter:
            self.bump_counter -= 1

    def ai_avoidance_query(self):
        self.get_logger().info("Querying Ollama for collision resolution...")
        try:
            #Query can and will be tweaked over time, with potential addition of lidar data.

            lidar_text = (
                self.lidar_summary(self.latest_lidar)
                if self.latest_lidar is not None
                else "Lidar data unavailable."
            )

            response = ollama.chat(model='gemma2:2b-instruct-q4_0', messages=[
                {
                    "role": "system",
                    "content": (
                        "You are an obstacle-avoidance controller for a small robot "
                        "navigating narrow tunnels. You must choose the safest movement."
                    )
                },
                {
                    "role": "user",
                    "content": f"""
                    The robot has just collided with an obstacle.

                    {lidar_text}

                    Rules:
                    - Do NOT choose a direction with the closest obstacle.
                    - Prefer the most open direction.
                    - If FRONT is the most open direction, choose GO.

                    Respond with ONLY ONE word:
                    BACK, LEFT, RIGHT, or GO
                    """
                }
            ])

            decision = response['message']['content'].strip().upper()
            self.get_logger().info(f"Ollama decided: {decision}")
            return decision
        except Exception as e:
            self.get_logger().error(f"Ollama connection failed: {e}")
            return "FAILED" # Default fallback

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
                # --- LLM BLOCKING LOGIC START ---
                if self.current_llm_decision is None:
                    # 1. Force the robot to STOP immediately
                    self.action_publisher.publish(self.wait_msg)
                    
                    # 2. Call LLM (This blocks the thread; robot "waits" here)
                    self.current_llm_decision = self.ai_avoidance_query()

                # 3. Execute the decision (Repeated for the duration of delay_counter)
                if "LEFT" in self.current_llm_decision:
                    self.action_publisher.publish(self.left_turn_msg)
                elif "RIGHT" in self.current_llm_decision:
                    self.action_publisher.publish(self.right_turn_msg)
                elif "BACK" in self.current_llm_decision:
                    self.action_publisher.publish(self.back_msg)
                elif "GO" in self.current_llm_decision:
                    self.action_publisher.publish(self.go_msg)
                else:
                    self.action_publisher.publish(self.back_msg)
                # --- LLM BLOCKING LOGIC END ---

                # Reset logic if needed when the maneuver ends
                if self.delay_counter == 1:
                     self.pause_bump_counter = False
                     self.bump_counter = 0

            self.delay_counter -= 1
        elif self.state == AvoidanceLayerStates.COLLISION:
            #Collision has resolved, transition to state NO_COLLISION, and
            #IMPORTANT: send NONE action message when the subroutine resolves,
            #otherwise the captain would continue to execute the last instruction
            self.state = AvoidanceLayerStates.NO_COLLISION
            self.action_publisher.publish(self.no_msg)
            self.delay_counter = self.config["AVOIDANCE_DELAY"]
            self.current_llm_decision = None

def main():
    rclpy.init()
    avoidance_layer = AvoidanceLayerAI()
    rclpy.spin(avoidance_layer)

if __name__ == '__main__':
    main()