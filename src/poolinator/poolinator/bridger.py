import cv2 as cv

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag import apriltag

import numpy as np

def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

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
        self.create_subscription(Image, 'rgb_image', self.process_rgb_image, 10)
        # self.create_subscription(Image, 'depth_image', self.process_depth_image, 10)
        self.pub = self.create_publisher(Image, 'new_image', 10)
        
        timer_period = 0.05 #secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # self.get_logger().info('IN TIMERRRR')
        pass
    
    def process_depth_image(self, image):
        # cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        pass

    def process_rgb_image(self, image):
        """Draw a circle on the subscribed image and republish it to new_image."""
        # short side
        dist0to2meters = 0.3
        dist1to3meters = 0.3
        # long side
        dist0to1meters = 0.515
        dist2to3meters = 0.515

        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

        detector = apriltag("tagStandard41h12")

        centers = {}  # dict to store tag IDs and their centers

        detections = detector.detect(gray_image)
        if detections:
            for detection in detections:
                tag_id = detection['id']
                center = detection['center']
                center_x, center_y = int(center[0]), int(center[1])
                centers[tag_id] = (center_x, center_y)

                # draw a red dot at the center of each tag
                cv.circle(cv_image, (center_x, center_y), radius=10, color=(0, 0, 255), thickness=-1)

                # self.get_logger().info(f'Detection: ID={tag_id}, Center=({center_x}, {center_y})')

                # distances in pixels
                tag_pairs = [
                    (0, 2, 'dist0to2'),
                    (1, 3, 'dist1to3'),
                    (0, 1, 'dist0to1'),
                    (2, 3, 'dist2to3'),
                ]
                for tag1, tag2, label in tag_pairs:
                    if tag1 in centers and tag2 in centers:
                        point1 = centers[tag1]
                        point2 = centers[tag2]
                        distance = euclidean_distance(point1, point2)
                        self.get_logger().info(f'{label}: {distance:.2f} pixels')

        new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.pub.publish(new_msg)


def main():
    rclpy.init()
    n = BridgeNode()
    rclpy.spin(n)
