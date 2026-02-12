import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests

class ai_command(Node):
    def __init__(self):
        super().__init__('ai_command')
        self.publisher = self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("AI Node started.")
        self.timer = self.create_timer(3.0, self.query_ollama)

    def query_ollama(self):
        prompt = "Suggest a movement command for the robot: 'forward', 'backward', 'left', 'right', or 'stop'. You should only send one word answers."
        try:
            response = requests.post(
                'http://localhost:11434/api/generate',    
                json={"model": "qwen2.5:0.5b", "prompt": prompt,"stream": False},
                timeout=60)
            if response.status_code == 200:
                text = response.json()['response'].strip().lower()
                self.get_logger().info(f"Ollama reply: {text}")
                self.publish_command(text)
            else:
                self.get_logger().warn(f"Failed to get response: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error querying Ollama: {e}")

    def publish_command(self, text):
        twist = Twist()
        if 'forward' in text:
            twist.linear.x = 0.3
        elif 'backward' in text:
            twist.linear.x = -0.3
        elif 'right' in text:
            twist.angular.z = -0.5
        elif 'left' in text:
            twist.angular.z = 0.5
        elif 'stop' in text:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.get_logger().info("No valid command found.")
            return
        self.publisher.publish(twist)
        self.get_logger().info(f"Published command: {text.strip()}")
        
def main(args=None):
    rclpy.init(args=args)
    node = ai_command()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
