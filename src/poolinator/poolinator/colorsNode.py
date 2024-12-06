import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import cv2 as cv2
import numpy as np

import pyrealsense2 as rs

class ImageProcessorColors(Node):
    def __init__(self):
        super().__init__('image_processor_colors')
        self.bridge = CvBridge()
        self.create_subscription(Image, 'rgb_image', self.rgb_process, 10)
        self.create_subscription(Image, 'depth_image', self.depth_process, 10)
        self.pub = self.create_publisher(Image, 'new_image', 10)

        timer_period = 0.05 #secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Placeholder for depth image
        # self.depth_image = None
        self.cx = None
        self.cy = None
        self.depth_value = None

        self.rgb_image_height = None
        self.rgb_image_width = None

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()

    def rgb_process(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        self.rgb_image_height, self.rgb_image_width, _ = cv_image.shape

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_ball_mask = cv2.bitwise_or(mask1, mask2)
        red_ball = cv2.bitwise_and(hsv_image,hsv_image,mask = red_ball_mask)
        new_msg = self.bridge.cv2_to_imgmsg(red_ball, encoding='bgr8')

        # Find the center of mass (centroid) of the red ball
        cx, cy = self.find_center_of_mass(red_ball_mask)

        # Check if any red pixels detected
        if cx is None or cy is None:
            self.get_logger().info('No ball detected')
        else:
            # self.get_logger().info(f'Ball at {cx}, {cy}')
            self.cx = cx
            self.cy = cy
        
        if cx and cy and self.depth_value:
            x, y, z = self.pixel_to_world(cx, cy, self.depth_value)
            # x, y, z = cx, cy, self.depth_value
            self.get_logger().info(f'Ball in world at {x}, {y}, {z}')

        self.pub.publish(new_msg)

    def depth_process(self, image):
        """
        Callback to process the depth image.
        """
        depth_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='16UC1')
        if self.cx and self.cy:
            # Scale cx and cy to match the depth image resolution
            cx_scaled = int(self.cx * (depth_image.shape[1] / self.rgb_image_width))
            cy_scaled = int(self.cy * (depth_image.shape[0] / self.rgb_image_height))
            # self.get_logger().info(f'x, y: {self.cx}, {self.cy}')
            # self.get_logger().info(f'cx_scaled, cy_scaled: {cx_scaled}, {cy_scaled}')
            # self.get_logger().info(f'depth_image_x, depth_image_y: {len(depth_image)}, {len(depth_image[0])}')

            # cx = self.cx
            # cy = self.cy
            self.depth_value = depth_image[cy_scaled, cx_scaled]

        # self.get_logger().info(f'in depth process: {depth_image}')

        # new_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        # self.pub.publish(new_msg)


    def find_center_of_mass(self, mask):
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Get the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Calculate the moments of the largest contour
            M = cv2.moments(largest_contour)

            # Ensure the moment is not zero (to avoid division by zero)
            if M["m00"] != 0:
                # Calculate the center of mass (cx, cy)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return cx, cy
        return None, None # if no red ball is found 
    
    def pixel_to_world(self, u, v, depth_value):
        """
        Convert pixel coordinates (u, v) and depth to world coordinates.
        Parameters:
            u (int): Pixel x-coordinate.
            v (int): Pixel y-coordinate.
            depth_value (float): Depth value at (u, v).
        Returns:
            tuple: (x, y, z) world coordinates in meters.
        """
        intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy

        # Convert pixel (u, v) and depth to world coordinates
        z = depth_value * self.depth_scale  # Convert depth to meters
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return x, y, z


    def timer_callback(self):
        # self.get_logger().info('IN TIMERRRR')
        pass

def main():
    rclpy.init()
    n = ImageProcessorColors()
    rclpy.spin(n)
