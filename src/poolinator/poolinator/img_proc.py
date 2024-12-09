import rclpy
from rclpy.node import Node
import cv2 as cv

from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import pyrealsense2 as rs2

if not hasattr(rs2, 'intrinsics'):
    import pyrealsense2.pyrealsense2 as rs2

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from poolinator.bridger import quaternion_from_euler


"""
-y
|
|
|-x__ __ __ __ __+x
|
|
+y
"""


class ImageProcessNode(Node):
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
        super().__init__('image_processor_colors')
        self.bridge = CvBridge()
        self.create_subscription(
            msg_Image, 'rgb_image', self.rgb_process, 10)
        self.create_subscription(
            msg_Image, 'depth_image', self.depth_process, 10
        )
        self.create_subscription(
            CameraInfo, 'color_camera_info', self.imageDepthInfoCallback, 1
        )

        self.pub_redball = self.create_publisher(msg_Image, 'red_ball', 10)
        self.pub_blueball = self.create_publisher(msg_Image, 'blue_ball', 10)
        self.pub_circles = self.create_publisher(msg_Image, 'circles', 10)
        self.pub_table = self.create_publisher(msg_Image, 'table', 10)

        timer_period = 0.05  # secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cx = None
        self.cy = None
        self.depth_value = None
        self.depth_scale = 0.001

        self.rgb_image_height = None
        self.rgb_image_width = None

        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

        self.ball_x = None
        self.ball_y = None
        self.ball_z = None

        self.blue_ball_x = None
        self.blue_ball_y = None
        self.blue_ball_z = None

        self.x_bound_green = None
        self.y_bound_green = None
        self.w_bound_green = None
        self.h_bound_green = None

        self.circle_positions = None

        self.tf_broadcaster = TransformBroadcaster(self)

    def broadcast_camera_to_redball(self):
        """_summary_"""
        # Try until publish succeds, cant continue without this
        try:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_color_optical_frame'
            t.child_frame_id = 'red_ball'

            t.transform.translation.x = float(self.ball_x)
            t.transform.translation.y = float(self.ball_y)
            t.transform.translation.z = float(self.ball_z)

            q = quaternion_from_euler(0, 0, 0)
            t.transform.rotation = q

            self.tf_broadcaster.sendTransform(t)
            return
        except Exception as e:
            self.get_logger().error(f"Failed to publish transform: {e}")

    def broadcast_camera_to_blueball(self):
        """_summary_"""
        # Try until publish succeds, cant continue without this
        try:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_color_optical_frame'
            t.child_frame_id = 'blue_ball'

            t.transform.translation.x = float(self.blue_ball_x)
            t.transform.translation.y = float(self.blue_ball_y)
            t.transform.translation.z = float(self.blue_ball_z)

            q = quaternion_from_euler(0, 0, 0)
            t.transform.rotation = q

            self.tf_broadcaster.sendTransform(t)
            return
        except Exception as e:
            self.get_logger().error(f"Failed to publish transform: {e}")

    def broadcast_camera_to_otherballs(self):
        """_summary_"""
        try:
            if not self.circle_positions:
                # self.get_logger().info("No balls detected.")
                return

            for key, value in self.circle_positions.items():
                t = TransformStamped()

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_color_optical_frame'
                t.child_frame_id = key

                t.transform.translation.x = float(value['x'])
                t.transform.translation.y = float(value['y'])
                t.transform.translation.z = float(value['z'])

                q = quaternion_from_euler(0, 0, 0)
                t.transform.rotation = q

                self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Failed to publish transform: {e}")

    def imageDepthInfoCallback(self, cameraInfo):
        """_summary_

        Args:
            cameraInfo (_type_): _description_
        """
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
        except CvBridgeError as e:
            print(e)
            return

    def detect_table(self, image):
        """_summary_

        Args:
            image (_type_): _description_
        """
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        # isolate green surface
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        green_mask = cv.inRange(hsv_image, lower_green, upper_green)
        green_table = cv.bitwise_and(hsv_image, hsv_image, mask=green_mask)
        new_msg = self.bridge.cv2_to_imgmsg(green_table, encoding='bgr8')
        self.pub_table.publish(new_msg)

    def detect_circles(self, image):
        """_summary_

        Args:
            image (_type_): _description_
        """
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # Isolate green surface
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        green_mask = cv.inRange(hsv_image, lower_green, upper_green)

        # Find the largest green contour
        contours, _ = cv.findContours(
            green_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
        )
        if contours:
            largest_contour = max(contours, key=cv.contourArea)

            # Get the bounding rectangle
            x_bound, y_bound, w_bound, h_bound = cv.boundingRect(
                largest_contour
            )

            if x_bound:
                self.x_bound_green = x_bound
                self.y_bound_green = y_bound
                self.w_bound_green = w_bound
                self.h_bound_green = h_bound

            cv.rectangle(
                image,
                (x_bound, y_bound),
                (x_bound + w_bound, w_bound + h_bound),
                (255, 0, 0),
                2,
            )

            # Detect circles in the original image
            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            gray_blurred = cv.blur(gray, (3, 3))

            minradius_in_px = 12
            maxradius_in_px = 14
            circles = cv.HoughCircles(
                gray_blurred,
                cv.HOUGH_GRADIENT,
                dp=1.1,
                minDist=minradius_in_px / 2,
                param1=150,
                param2=17,
                minRadius=minradius_in_px,
                maxRadius=maxradius_in_px,
            )

            # Initialize the circle_positions dictionary
            self.circle_positions = {}

            # Filter circles within the bounding rectangle
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for idx, pt in enumerate(circles[0, :]):
                    a, b, r = pt[0], pt[1], pt[2]

                    # Check if the circle's center is within the bounding rectangle
                    if (
                        x_bound <= a <= x_bound + w_bound
                        and y_bound <= b <= y_bound + h_bound
                    ):
                        cv.circle(image, (a, b), r, (0, 255, 0), 2)
                        cv.circle(image, (a, b), 1, (0, 0, 255), 3)

                        # Get depth value at the circle center
                        if self.intrinsics and self.depth_value:
                            coords = self.pixel_to_world(
                                a, b, self.depth_value
                            )

                            if coords:
                                ball_name = 'ball' + str(idx)
                                self.circle_positions[ball_name] = {
                                    'x': coords[0],
                                    'y': coords[1],
                                    'z': coords[2],
                                }
            # self.get_logger().info(f"Number of balls detected: {len(self.circle_positions)}")

        new_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.pub_circles.publish(new_msg)

    def rgb_process(self, image):
        """_summary_

        Args:
            image (_type_): _description_
        """
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        self.rgb_image_height, self.rgb_image_width, _ = cv_image.shape

        self.detect_table(cv_image)
        self.detect_circles(cv_image)
        self.rgb_process_blue_ball(cv_image)

        hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv.inRange(hsv_image, lower_red2, upper_red2)
        red_ball_mask = cv.bitwise_or(mask1, mask2)
        red_ball = cv.bitwise_and(hsv_image, hsv_image, mask=red_ball_mask)
        new_msg = self.bridge.cv2_to_imgmsg(red_ball, encoding='bgr8')

        # Find the center of mass (centroid) of the red ball
        cx, cy = self.find_center_of_mass(red_ball_mask)

        # Check if any red pixels detected
        if cx is None or cy is None:
            self.get_logger().info('No ball detected')
        else:
            self.cx = cx
            self.cy = cy

        if cx and cy and self.depth_value:
            coords = self.pixel_to_world(cx, cy, self.depth_value)
            if coords:
                self.ball_x = coords[0]
                self.ball_y = coords[1]
                self.ball_z = coords[2]

        self.pub_redball.publish(new_msg)

    
    def rgb_process_blue_ball(self, image):
        """_summary_

        Args:
            image (_type_): _description_
        """
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        # lower_blue = np.array([0,0,200])
        # upper_blue = np.array([180, 30, 255])

        lower_blue = np.array([100,150,50])
        upper_blue = np.array([140, 255, 255])


        blue_ball_mask = cv.inRange(hsv_image, lower_blue, upper_blue)
    
        blue_ball = cv.bitwise_and(hsv_image, hsv_image, mask=blue_ball_mask)
        new_msg = self.bridge.cv2_to_imgmsg(blue_ball, encoding='bgr8')

        # Find the center of mass (centroid) of the ball
        cx, cy = self.find_center_of_mass(blue_ball_mask)

        if cx and cy and self.depth_value:
            coords = self.pixel_to_world(cx, cy, self.depth_value)
            if coords:
                self.blue_ball_x = coords[0]
                self.blue_ball_y = coords[1]
                self.blue_ball_z = coords[2]

        self.pub_blueball.publish(new_msg)

    def depth_process(self, image):
        """Callback to process the depth image.


        Args:
            image (_type_): _description_
        """
        depth_image = self.bridge.imgmsg_to_cv2(
            image, desired_encoding='16UC1'
        )
        if self.cx and self.cy:
            # Scale cx and cy to match the depth image resolution
            cx_scaled = int(
                self.cx * (depth_image.shape[1] / self.rgb_image_width)
            )
            cy_scaled = int(
                self.cy * (depth_image.shape[0] / self.rgb_image_height)
            )
            self.depth_value = depth_image[cy_scaled, cx_scaled]

    def find_center_of_mass(self, mask):
        """_summary_

        Args:
            mask (_type_): _description_

        Returns:
            _type_: _description_
        """
        # Find contours in the mask
        contours, _ = cv.findContours(
            mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
        )

        if contours:
            # Get the largest contour by area
            largest_contour = max(contours, key=cv.contourArea)

            # Calculate the moments of the largest contour
            M = cv.moments(largest_contour)

            # Ensure the moment is not zero (to avoid division by zero)
            if M["m00"] != 0:
                # Calculate the center of mass (cx, cy)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                return cx, cy
        return None, None  # if no red ball is found

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
        if self.intrinsics:
            fx = self.intrinsics.fx
            fy = self.intrinsics.fy
            cx = self.intrinsics.ppx
            cy = self.intrinsics.ppy

            # Convert pixel (u, v) and depth to world coordinates
            z = depth_value * self.depth_scale  # Convert depth to meters
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            return [x, y, z]
        return None

    def timer_callback(self):
        self.broadcast_camera_to_redball()
        self.broadcast_camera_to_blueball()
        self.broadcast_camera_to_otherballs()

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main():
    rclpy.init()
    n = ImageProcessNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
