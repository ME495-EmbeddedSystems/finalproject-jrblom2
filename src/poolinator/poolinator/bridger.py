import numpy as np
import math
import cv2 as cv

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag import apriltag

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import do_transform_pose

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
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

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

        timer_period = 5.0  # secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cuetag_to_camera_pose = Pose()
        self.hasCameraPosition = False

    def broadcast_ferhand_to_cuetag(self):
        # Try until publish succeds, cant continue without this
        while True:
            try:
                t = TransformStamped()

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'fer_hand_tcp'
                t.child_frame_id = 'que_tag'

                t.transform.translation.x = 0.061
                t.transform.translation.y = 0.0
                t.transform.translation.z = -0.018

                q = quaternion_from_euler(np.pi, 0, 0)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                self.static_broadcaster.sendTransform(t)
                self.get_logger().info(
                    f"Published transform from {t.header.frame_id} to {t.child_frame_id}"
                )
                return
            except Exception as e:
                self.get_logger().error(f"Failed to publish transform: {e}")

    def lookup_cuetag_to_camera(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_cam = 'tagStandard41h12:4'
        to_frame_cam = 'camera_link'

        from_frame_que = 'base'
        to_frame_que = 'que_tag'

        # Try until publish succeds, cant continue without this
        try:
            # Camera in the que tag frame
            tcam = self.tf_buffer.lookup_transform(
                from_frame_cam, to_frame_cam, rclpy.time.Time()
            )
            # Que tag in the base frame
            tque = self.tf_buffer.lookup_transform(
                from_frame_que, to_frame_que, rclpy.time.Time()
            )

            self.cuetag_to_camera_pose.position.x = (
                tcam.transform.translation.x
            )
            self.cuetag_to_camera_pose.position.y = (
                tcam.transform.translation.y
            )
            self.cuetag_to_camera_pose.position.z = (
                tcam.transform.translation.z
            )

            self.cuetag_to_camera_pose.orientation = tcam.transform.rotation

            # Combine transforms
            cameraInBase = do_transform_pose(self.cuetag_to_camera_pose, tque)

            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base'
            t.child_frame_id = 'camera_link'

            t.transform.translation.x = cameraInBase.position.x
            t.transform.translation.y = cameraInBase.position.y
            t.transform.translation.z = cameraInBase.position.z

            t.transform.rotation = cameraInBase.orientation

            # Publish camera in base frame
            self.static_broadcaster.sendTransform(t)
            self.hasCameraPosition = True
            return
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_cam} to {from_frame_cam}: {ex}'
            )

    def timer_callback(self):
        if not self.hasCameraPosition:
            self.broadcast_ferhand_to_cuetag()
            self.lookup_cuetag_to_camera()


def main():
    rclpy.init()
    n = BridgeNode()
    rclpy.spin(n)
