import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import numpy
import cv2
from sensor_msgs.msg import Image
from tools.csv_parser import loadConfig
from cv_bridge import CvBridge

class CameraSensor(Node):
    """
    The Node in charge of taking pictures and deciding whether they contain an intersection marker.

    @Publishers:
    - Publishes camera data to /camera_data.
    """
    def __init__(self):
        super().__init__('camera_sensor')

        self.bridge = CvBridge()  # <-- THIS LINE is the fix

        self.config = loadConfig()

        self.camera_data_publisher = self.create_publisher(Bool, 'camera_data', 10)

        # Create the image subscriber to /camera/image_raw
        self.image_subscriber = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.timer = self.create_timer(0.2, self.get_yellow)

        self.true_msg = Bool()
        self.true_msg.data = True
        self.false_msg = Bool()
        self.false_msg.data = False

        self.lower_yellow = numpy.array([self.config["CAMERA_MIN_HUE"], self.config["CAMERA_MIN_SATURATION"], self.config["CAMERA_MIN_VALUE"]])
        self.upper_yellow = numpy.array([self.config["CAMERA_MAX_HUE"], self.config["CAMERA_MAX_SATURATION"], self.config["CAMERA_MAX_VALUE"]])

        self.image_width = int(self.config["CAMERA_IMG_WIDTH"])
        self.image_height = int(self.config["CAMERA_IMG_HEIGHT"])
        self.total_pixels = self.image_width * self.image_height

        self.latest_image = None  # Initialize variable to store the latest image

    def image_callback(self, msg: Image):
        """
        Callback to handle incoming images from /camera/image_raw.
        """
        try:
            # Convert ROS image message to OpenCV format
            self.latest_image = numpy.array(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def get_yellow(self):
        """
        Processes the image to detect yellow pixels and checks if the yellow threshold is met.
        """
        if self.latest_image is None:
            return

        # Save the image to disk as 'image.jpg'
        cv2.imwrite("image.jpg", self.latest_image)

        # Read the saved image for processing
        image = cv2.imread("image.jpg")
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv_image, self.lower_yellow, self.upper_yellow)
        yellow_pixels = numpy.sum(yellow_mask > 0)
        yellow_percentage = yellow_pixels / self.total_pixels

        if yellow_percentage > self.config["CAMERA_YELLOW_THRESHOLD"]:
            self.camera_data_publisher.publish(self.true_msg)
        else:
            self.camera_data_publisher.publish(self.false_msg)

def main():
    rclpy.init()
    camera_sensor = CameraSensor()
    rclpy.spin(camera_sensor)

if __name__ == '__main__':
    main()
