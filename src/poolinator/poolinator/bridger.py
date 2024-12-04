import numpy as np
import math
import cv2 as cv

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag import apriltag

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Pose


def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

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
        # self.create_subscription(Image, 'rgb_image', self.process_rgb_image, 10)
        # self.pub = self.create_publisher(Image, 'new_image', 10)
        
        timer_period = 0.05 #secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_to_cuetag_pose = Pose()

    def broadcast_ferhand_to_cuetag(self):
        try:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'fer_hand'
            t.child_frame_id = 'tagStandard41h12_4'

            t.transform.translation.x = 0.061
            t.transform.translation.y = 0.018
            t.transform.translation.z = 0.0

            q = quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)
            # self.get_logger().info(f"Published transform from {t.header.frame_id} to {t.child_frame_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish transform: {e}")

    def lookup_camera_to_cuetag(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'camera_link'
        to_frame_rel = 'tagStandard41h12:4'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        self.camera_to_cuetag_pose.position.x = t.transform.translation.y
        self.camera_to_cuetag_pose.position.y = t.transform.translation.x
        self.camera_to_cuetag_pose.position.z = t.transform.translation.x

        self.camera_to_cuetag_pose.orientation = t.transform.rotation

        self.get_logger().info(f"Transform from {from_frame_rel} to {to_frame_rel}: {t.transform}")

    def timer_callback(self):
        self.broadcast_ferhand_to_cuetag()
        self.lookup_camera_to_cuetag()

    
def main():
    rclpy.init()
    n = BridgeNode()
    rclpy.spin(n)
