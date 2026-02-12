import rclpy
import numpy as np
import tkinter as tk
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque
from tools.csv_parser import loadConfig

class TravelAnalyzer(Node):
    def __init__(self):
        super().__init__('travel_layer_test')
        
        # Load configuration parameters
        self.config = loadConfig()
        self.target_wall_distance = self.config["WALL_FOLLOW_SET_POINT"]
        
        # Data tracking
        self.wall_distances = deque() 
        
        # Subscribers
        self.lidar_data_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)
        
        # Start test timer
        self.timer = self.create_timer(0.2, self.log_metrics)
    
    def lidar_data_callback(self, msg):
        """Processes LIDAR data and tracks right-side wall-following performance."""
        try:
            parts = msg.data.split(":")
            if len(parts) != 5:
                return
            _, _, right, _, _ = map(float, parts)
            self.wall_distances.append(right)
        except Exception as e:
            self.get_logger().error(f"Error parsing LIDAR data: {e}")
    
    def log_metrics(self):
        """Logs real-time metrics (optional for debugging)."""
        if self.wall_distances:
            avg_wall_distance = np.mean(self.wall_distances)
            std_dev_wall = np.std([abs(d - self.target_wall_distance) for d in self.wall_distances])
            self.get_logger().info(f"Wall Distance (Avg: {avg_wall_distance:.2f}, Std: {std_dev_wall:.2f})")
    
    def display_results(self):
        """Creates a simple Tkinter GUI to display final results."""
        avg_wall_distance = np.mean(self.wall_distances) if self.wall_distances else 0
        std_dev_wall = np.std([abs(d - self.target_wall_distance) for d in self.wall_distances]) if self.wall_distances else 0
        
        root = tk.Tk()
        root.title("Travel Layer Test Results")
        
        tk.Label(root, text="Wall Following Performance", font=("Arial", 14, "bold")).pack(pady=10)
        tk.Label(root, text=f"Average Distance from Wall: {avg_wall_distance:.2f} m").pack()
        tk.Label(root, text=f"Standard Deviation: {std_dev_wall:.2f}").pack()
        
        tk.Button(root, text="Close", command=root.destroy).pack(pady=10)
        
        root.mainloop()

def main():
    rclpy.init()
    test_node = TravelAnalyzer()
    rclpy.spin(test_node)
    test_node.display_results()
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
