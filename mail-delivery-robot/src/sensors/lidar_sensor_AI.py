import math
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from statistics import  stdev
import ollama 
import json
import sys

from tools.csv_parser import loadConfig

class LidarSensor(Node):
    '''
    The Node in charge of listening to the lidar sensor.

    @Subscribers:
    - Listens to /scan for new lidar scans.

    @Publishers:
    - Publishes new updates to /lidar_data.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('lidar_sensor')

        # Load the global config.
        self.config = loadConfig()

        # The publishers for the node.
        self.publisher_ = self.create_publisher(String, 'lidar_data' , 10)
        
        # The subscribers for the node.
        self.lidar_info_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)

        self.right_distances = []
        self.left_distances = []
        self.front_distances = []

    def scan_callback(self, scan):
        '''
        The callback for /scan.
        Reads the lidar scan and acts accordingly.

        @param scan: The current lidar scan.
        '''
        calc = String()
        
        feedback, angle, right, left, front  = self.calculate_ai(scan)
        calc.data = str(feedback) + ":" + str(angle) + ":" + str(right) + ":" + str(left) + ":" + str(front)

        self.publisher_.publish(calc)

    def calculate(self, scan):
        '''
        Calculates the robot's distance with its surroundings.

        @param scan: The current lidar scan.
        ''' 
        count = len(scan.ranges)
        angle = 0
        min_left = self.config["LARGE_DEFAULT_DISTANCE"]
        min_right = self.config["LARGE_DEFAULT_DISTANCE"]
        min_front = self.config["LARGE_DEFAULT_DISTANCE"]
        min_distance = self.config["LARGE_DEFAULT_DISTANCE"]

        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            #self.get_logger().info(str(degree) + "  " + str(scan.ranges[i]))
            curDir = scan.ranges[i]
            if curDir == math.inf:
                continue

            #wall_following
            if degree >= self.config["WALL_FOLLOW_MIN_ANGLE"] and degree <= self.config["WALL_FOLLOW_MAX_ANGLE"] and curDir < min_distance:
                min_distance = curDir
                angle = degree
    
            if (degree <= self.config["FRONT_MIN_ANGLE"] or degree >= self.config["FRONT_MAX_ANGLE"] ) and curDir < min_front:
                min_front = curDir
            elif degree >= self.config["RIGHT_MIN_ANGLE"] and degree < self.config["RIGHT_MAX_ANGLE"] and curDir < min_right:
                min_right = curDir
            elif degree > self.config["LEFT_MIN_ANGLE"] and degree <= self.config["LEFT_MAX_ANGLE"] and curDir < min_left:
                min_left = curDir
        
        self.left_distances.append(min_left)

        self.right_distances.append(min_right)

        self.front_distances.append(min_front)

        if len(self.left_distances) > self.config["LIDAR_STACK_LENGTH"]:
            self.left_distances.pop(0)
        else:
            return -1, -1, -1, -1, -1

        if len(self.right_distances) > self.config["LIDAR_STACK_LENGTH"]:
            self.right_distances.pop(0)
        else:
            return -1, -1, -1, -1, -1

        if len(self.front_distances) > self.config["LIDAR_STACK_LENGTH"]:
            self.front_distances.pop(0)
        else:
            return -1, -1, -1, -1, -1

        if min_front >= self.config["LOST_WALL_FRONT_DISTANCE"] or stdev(self.front_distances) > self.config["LOST_WALL_FRONT_STDEV"]:
            min_front = -1
       
        #self.get_logger().info(str(min_right) + " " + str(stdev(self.right_distances)))
        if min_right >= self.config["LOST_WALL_RIGHT_DISTANCE"]  or stdev(self.right_distances) > self.config["LOST_WALL_RIGHT_STDEV"]:
            min_right = -1

        if min_left >= self.config["LOST_WALL_LEFT_DISTANCE"]  or stdev(self.left_distances) > self.config["LOST_WALL_LEFT_STDEV"]:
            min_left = -1

        return min_distance, angle - 90, min_right, min_left, min_front
    
    def calculate_ai(self, scan):
        '''
        Calculates the robot's distance with its surroundings for AI purposes.

        @param scan: The current lidar scan.
        ''' 
        scan_pairs = []
        step = 5 
        default_dist = self.config.get("LARGE_DEFAULT_DISTANCE", 10.0)

        for i in range(0, len(scan.ranges), step):
            dist = scan.ranges[i]
            # Filter invalid readings (infinite or zero)
            if dist == math.inf or dist == 0.0:
                continue
            
            # Match the degree calculation from your original calculate function
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            scan_pairs.append(f"{degree:.1f}:{dist:.2f}")
        
        data_str = ", ".join(scan_pairs)



        prompt = f"""
        Analyze these Lidar readings (format "angle:distance").
        Data: [{data_str}]

        Task: Find the minimum distance in the following sectors. 
        If a sector has no data, use {default_dist}.

        Sectors:
        1. Wall Follow: Angle between {self.config["WALL_FOLLOW_MIN_ANGLE"]} and {self.config["WALL_FOLLOW_MAX_ANGLE"]}.
           - Find the min_distance and the specific angle it occurred at.
        2. Front: Angle <= {self.config["FRONT_MIN_ANGLE"]} OR Angle >= {self.config["FRONT_MAX_ANGLE"]}.
        3. Right: Angle >= {self.config["RIGHT_MIN_ANGLE"]} and < {self.config["RIGHT_MAX_ANGLE"]}.
        4. Left: Angle > {self.config["LEFT_MIN_ANGLE"]} and <= {self.config["LEFT_MAX_ANGLE"]}.

        Return ONLY a JSON object with keys: wf_dist, wf_angle, right, left, front.
        """
        self.get_logger().info(f"Prompt to LLM: {prompt}")
        # 3. Query Ollama
        # Note: This is a blockinimport sys

        try:
            response = ollama.chat(
                model='gemma2:2b-instruct-q4_0',  # Ensure this model is pulled: `ollama pull qwen2`
                messages=[{'role': 'user', 'content': prompt}],
                format='json',  # Enforces valid JSON output
            )
            
            self.get_logger().info(f"Raw LLM Response: {response}")

            # 4. Parse & Return
            content = json.loads(response['message']['content'])
            
            min_distance = float(content.get('wf_dist', default_dist))
            angle = float(content.get('wf_angle', 0.0))
            min_right = float(content.get('right', default_dist))
            min_left = float(content.get('left', default_dist))
            min_front = float(content.get('front', default_dist))

            # Return exactly like the original function: (dist, angle-90, right, left, front)
            return min_distance, angle - 90, min_right, min_left, min_front

        except Exception as e:
            self.get_logger().info(f"AI Calculation failed: {e}")
            # Return to original if llm fails
            return self.calculate(scan)

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    lidar_sensor = LidarSensor()
    rclpy.spin(lidar_sensor)
    
if __name__ == '__main__':
    main()

