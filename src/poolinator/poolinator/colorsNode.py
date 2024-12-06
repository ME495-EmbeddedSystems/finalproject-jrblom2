import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
import cv2 as cv2

from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2


"""
-y
|
|
|-x__ __ __ __ __+x
|
|
+y
"""

class ImageProcessorColors(Node):
    def __init__(self, depth_image_topic, depth_info_topic):
        super().__init__('image_processor_colors')
        self.bridge = CvBridge()
        self.create_subscription(msg_Image, 'rgb_image', self.rgb_process, 10)
        self.create_subscription(msg_Image, 'depth_image', self.depth_process, 10)
        self.pub = self.create_publisher(msg_Image, 'new_image', 10)

        timer_period = 0.05 #secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cx = None
        self.cy = None
        self.depth_value = None

        self.rgb_image_height = None
        self.rgb_image_width = None

        self.sub = self.create_subscription(msg_Image, depth_image_topic, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

        # TODO: find how to find this
        self.depth_scale = 0.001


    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # pick one pixel among all the pixels with the closest range:
            indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
            pix = (indices[1], indices[0])
            self.pix = pix
            line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])

            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            if (not self.pix_grade is None):
                line += ' Grade: %2d' % self.pix_grade
            line += '\r'
            sys.stdout.write(line)
            sys.stdout.flush()

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return


    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]

            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return

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
            self.pixel_to_world(cx, cy, self.depth_value)
            # self.pixel_to_world(cx, cy, self.depth_value)
            # coords = self.pixel_to_world(cx, cy, self.depth_value)
            # if coords:
            #     # self.get_logger().info(f'Ball in world at {x}, {y}, {z}')
            #     self.get_logger().info(f'Ball in world at {coords[0]}, {coords[1]}, {coords[2]}')

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
        self.get_logger().info(f'in pixel_to_world')
        if self.intrinsics:
            fx = self.intrinsics.fx
            fy = self.intrinsics.fy
            cx = self.intrinsics.ppx 
            cy = self.intrinsics.ppy 

            # self.get_logger().info(f'fx: {fx}')
            # self.get_logger().info(f'fy: {fy}')
            # self.get_logger().info(f'cx: {cx}')
            # self.get_logger().info(f'cy: {cy}')

            # Convert pixel (u, v) and depth to world coordinates
            z = depth_value * self.depth_scale  # Convert depth to meters
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            self.get_logger().info(f'x: {x}')
            self.get_logger().info(f'y: {y}')
            self.get_logger().info(f'z: {z}')

        #         return [x, y, z]
        # return None

    def timer_callback(self):
        # self.get_logger().info('IN TIMERRRR')
        pass

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main():
    depth_image_topic = '/camera/camera/depth/image_rect_raw'
    depth_info_topic = '/camera/camera/depth/camera_info'

    rclpy.init()
    n = ImageProcessorColors(depth_image_topic, depth_info_topic)
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()    

