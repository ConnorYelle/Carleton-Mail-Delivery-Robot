import os
import rclpy
import time
from datetime import datetime, timedelta
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


UPDATE_ACTIONS_INTERVAL = 0.2 #this is the amount of time between each actions update

class GeneralLogger(Node):
    def __init__(self):
        super().__init__('general_logger')

        #Logs directory in src
        self.log_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "logs")
        os.makedirs(self.log_dir, exist_ok=True)

        #Time log
        self.time_log_path = os.path.join(self.log_dir, "robot_log_time.txt")
        with open(self.time_log_path, 'w') as file:
            file.truncate(0)
        self.time_log_file = open(self.time_log_path, "a")

        #Battery log
        self.battery_log_path = os.path.join(self.log_dir, "robot_log_battery.txt")
        with open(self.battery_log_path, 'w') as file:
            file.truncate(0)    
        self.battery_log_file = open(self.battery_log_path, "a")

        #Wall-follow log
        self.wall_log_path = os.path.join(self.log_dir, "robot_log_wallFollowing.txt")
        with open(self.wall_log_path, 'w') as file:
            file.truncate(0)  # reset file
        self.wall_log_file = open(self.wall_log_path, "a")

        self.get_logger().info(f"Logging battery to {self.battery_log_path}")
        self.get_logger().info(f"Logging wall-follow to {self.wall_log_path}")

        # Subscribe to battery updates
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.create_subscription(String, '/actions', self.log_wall_follow_callback, 10)

        #start the timer for metric collection
        self.start_time = self.start_timer()
        self.trip_start_timestamp = datetime.now()

        #variable for storing the amount of time spent wall following
        self.wall_following_time = 0.0


    def start_timer(self):
        start_time = time.perf_counter()
        return start_time

    def end_timer(self):
        end_time = time.perf_counter()  
        return end_time

    def battery_callback(self, msg):
        self.battery_log_file.write(
            f"[BATTERY] Percentage: {msg.percentage*100:.1f}%, "
            f"Voltage: {msg.voltage:.2f}V, Temp: {msg.temperature:.1f}C\n"
        )
        self.battery_log_file.flush()

    def log_wall_follow_callback(self, action):
        if "WALL_FOLLOW" in action.data:
            self.wall_log_file.write(action.data + "\n")
            self.wall_following_time += UPDATE_ACTIONS_INTERVAL
        self.wall_log_file.flush()

    

    def destroy_node_and_log_time(self):

        end_time = self.end_timer()
        trip_end_timestamp = datetime.now()
        elapsed_time = end_time - self.start_time
        self.time_log_file.write("Total Elapsed Time: "+str(elapsed_time)+"\n")
        self.time_log_file.write("Total Wall Following Time: "+str(self.wall_following_time)+"\n")
        self.time_log_file.write("Trip Start Timestamp: "+self.trip_start_timestamp.strftime("%Y-%m-%d %H:%M:%S")+"\n")
        self.time_log_file.write("Trip End Timestamp: "+trip_end_timestamp.strftime("%Y-%m-%d %H:%M:%S")+"\n")
        self.time_log_file.flush()
        self.time_log_file.close()
        self.battery_log_file.close()
        self.wall_log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GeneralLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node_and_log_time()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
