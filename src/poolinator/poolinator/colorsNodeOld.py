import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point  # Import Point message for coordinates

class RealSenseCameraNode(Node):
    def __init__(self):
        super().__init__('realsense_camera_node')

        # Initialize pipeline and configure streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Set up the RealSense camera stream
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline.start(self.config)

        # Create align object -> aligns depth data to color data
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Publishers
        self.publisher_color = self.create_publisher(Image, '/camera/color/image_rawww', 10)
        self.publisher_mask = self.create_publisher(Image, '/camera/red_ball_mask', 10)
        self.publisher_coordinates = self.create_publisher(Point, '/camera/red_ball_coordinates', 10)

        # Camera intrinsics (example values; replace with actual camera intrinsics)
        self.camera_intrinsics = {
            'fx': 640.0,  # Focal length in x (in pixels)
            'fy': 480.0,  # Focal length in y (in pixels)
            'cx': 320.0,  # Principal point x-coordinate (in pixels)
            'cy': 240.0   # Principal point y-coordinate (in pixels)
        }

        # Create a timer to periodically get and publish frames
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        self.get_logger().info("In timer callback")
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if aligned_depth_frame and color_frame:
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            self.get_logger().info(f"Depth Image Shape: {depth_image.shape}")
            self.get_logger().info(f"Color Image Shape: {color_image.shape}")

            red_mask, red_ball = self.color_tracking(color_image, depth_image)

            # Publish images
            try:
                self.publisher_color.publish(self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8'))
                self.publisher_mask.publish(self.bridge.cv2_to_imgmsg(red_mask, encoding='mono8'))
            except Exception as e:
                self.get_logger().error(f"Image publishing error: {e}")
                self.get_logger().error(f"Image publishing error: {e}")

    def color_tracking(self, color_image, depth_image):
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Isolate the red ball
        red_ball_mask = red_mask

        # Calculate the center of the ball
        moments = cv2.moments(red_ball_mask)
        if moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])

            # Fetch the depth value at the center
            depth_value = depth_image[cy, cx]

            # Convert (cx, cy, depth) to real-world coordinates (x, y, z)
            x, y, z = self.pixel_to_world(cx, cy, depth_value)

            # Publish the coordinates
            self.publish_coordinates(x, y, z)

        return red_ball_mask, red_ball_mask

    def pixel_to_world(self, px, py, depth):
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        cx = self.camera_intrinsics['cx']
        cy = self.camera_intrinsics['cy']

        # Convert pixel coordinates and depth to world coordinates
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth / 1000.0  # Convert millimeters to meters

        return x, y, z

    def publish_coordinates(self, x, y, z):
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = z

        self.publisher_coordinates.publish(point_msg)
        self.get_logger().info(f"Published coordinates: x={x:.2f}, y={y:.2f}, z={z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
