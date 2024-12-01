import cv2 as cv

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag import apriltag

import numpy as np

class BridgeNode(Node):
    """
    Node that reads images and draws on them using open cv.

    Publishers
    ----------
    new_image (sensor_msgs/msg/Image): The image after post procesasing

    Subscribers
    -----------
    image (sensor_msgs/msg/Image): The image on which to do the processing
    """

    def __init__(self):
        super().__init__('bridge')
        self.bridge = CvBridge()
        self.create_subscription(Image, 'image', self.opencv_process, 10)
        self.pub = self.create_publisher(Image, 'new_image', 10)
        
        timer_period = 0.05 #secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('IN TIMERRRR')  # Log message

    def opencv_process(self, image):
        """Draw a circle on the subscribed image and republish it to new_image."""
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        # gray_image = cv.imread(cv_image, cv.IMREAD_GRAYSCALE)
        gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

        detector = apriltag("tagStandard41h12")

        detections = detector.detect(gray_image)
        if detections:
            self.get_logger().info('DETECTED!!!')  # Log message

        new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.pub.publish(new_msg)


def main():
    rclpy.init()
    n = BridgeNode()
    rclpy.spin(n)
