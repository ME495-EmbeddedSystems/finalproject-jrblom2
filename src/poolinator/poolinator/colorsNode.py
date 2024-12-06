import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import cv2 as cv2
import numpy as np

class ImageProcessorColors(Node):
    def __init__(self):
        super().__init__('image_processor_colors')
        self.bridge = CvBridge()
        self.create_subscription(Image, 'image', self.opencv_process, 10)
        self.pub = self.create_publisher(Image, 'new_image', 10)

        timer_period = 0.05 #secs
        self.timer = self.create_timer(timer_period, self.timer_callback)


        # # Subscribing to the RGB and Depth image topics
        # self.rgb_subscription = self.create_subscription(
        #     Image,
        #     '/camera/color/image_raw',  # RGB topic
        #     self.process_rgb_image,
        #     10
        # )
        # self.depth_subscription = self.create_subscription(
        #     Image,
        #     '/camera/depth/image_rect_raw',  # Depth topic
        #     self.process_depth_image,
        #     10
        # )

        # # Placeholder for depth image
        # self.depth_image = None

        self.get_logger().info("Subscriptions created for RGB and Depth image topics.")


    def opencv_process(self, image):
        """Draw a circle on the subscribed image and republish it to new_image."""
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        res = cv2.bitwise_and(hsv_image,hsv_image,mask = red_mask)
        new_msg = self.bridge.cv2_to_imgmsg(res, encoding='bgr8')
        self.pub.publish(new_msg)

    def timer_callback(self):
        self.get_logger().info('IN TIMERRRR')
    
    # def process_rgb_image(self, msg):
    #     try:
    #         # Convert ROS2 Image message to OpenCV image
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    #         # Convert to HSV and apply a mask for red color
    #         hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    #         lower_red1 = np.array([0, 100, 100])
    #         upper_red1 = np.array([10, 255, 255])
    #         lower_red2 = np.array([160, 100, 100])
    #         upper_red2 = np.array([180, 255, 255])
    #         mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    #         mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    #         red_mask = cv2.bitwise_or(mask1, mask2)

    #         # Find contours
    #         contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #         if contours:
    #             # Find the largest contour
    #             largest_contour = max(contours, key=cv2.contourArea)
    #             # Get the center of mass
    #             M = cv2.moments(largest_contour)
    #             if M["m00"] > 0:
    #                 cx = int(M["m10"] / M["m00"])
    #                 cy = int(M["m01"] / M["m00"])
    #                 # Draw a circle at the center
    #                 cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
    #                 # Print the x, y, z distance
    #                 z_distance = self.get_depth_at_pixel(cx, cy)
    #                 self.get_logger().info(f"Red Ball Position: x={cx}, y={cy}, z={z_distance:.2f}m")

    #         # Display the image
    #         cv2.imshow("RGB Image", cv_image)
    #         cv2.waitKey(1)
    #         self.get_logger().info("Processing RGB image.")
    #     except Exception as e:
    #         self.get_logger().error(f"Error processing RGB image: {e}")

    # def process_depth_image(self, msg):
    #     try:
    #         # Convert ROS2 Image message to OpenCV image
    #         self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
    #     except Exception as e:
    #         self.get_logger().error(f"Error processing Depth image: {e}")

    # def get_depth_at_pixel(self, x, y):
    #     """Get the depth value at a specific pixel (x, y)."""
    #     if self.depth_image is not None:
    #         depth_value = self.depth_image[y, x]  # Depth value in millimeters
    #         return depth_value / 1000.0  # Convert to meters
    #     return float('nan')  # Return NaN if depth image is not available

# def main(args=None):
#     rclpy.init(args=args)
#     node = ImageProcessorColors()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Node stopped cleanly')
#     except Exception as e:
#         node.get_logger().error(f"Error in the node: {e}")
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

def main():
    rclpy.init()
    n = ImageProcessorColors()
    rclpy.spin(n)
