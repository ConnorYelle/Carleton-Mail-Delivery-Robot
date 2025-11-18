import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import subprocess
import numpy
import cv2
from tools.csv_parser import loadConfig

class CameraSensor(Node):
    """
    The Node in charge of taking pictures and deciding whether they contain an intersection marker.
    
    @Publishers:
    - Publishes camera data to /camera_data.
    """
    def __init__(self):
        super().__init__('camera_sensor')

        self.config = loadConfig()

        self.camera_data_publisher = self.create_publisher(Bool, 'camera_data', 10)

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

    def get_yellow(self):
        subprocess.run(["libcamera-still", "--output", "image.jpg", "--immediate", "--width", str(self.image_width), "--height", str(self.image_height), "-n", "--verbose", "0"])
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