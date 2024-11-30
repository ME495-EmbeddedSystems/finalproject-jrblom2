import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np

class GetImageNode(Node):
    def __init__(self):
        """
        Initialize the GetImageNode.
        """
        super().__init__('get_image')

        timer_period = 0.05 #secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GetImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()