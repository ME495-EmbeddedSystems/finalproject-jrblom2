"""Keeps track of the world configuration."""

from geometry_msgs.msg import Pose, TransformStamped

import numpy as np

from poolinator.bridger import quaternion_from_euler

import rclpy
from rclpy.duration import Duration

from tf2_geometry_msgs import do_transform_pose

from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener

from tf_transformations import euler_from_quaternion

"""
    F
    |
    |
C1-----C2
|      |
C4-----C3
"""

"""
    F
    |
    |
P1--P2--P3
|      |
P6--P5--P4
"""


class World:
    """Representation of world to be used in control."""

    def __init__(self, node, cornerTagName, ballTagNames):
        """Set up the world.

        Args:
            node (Node): rclpy node object to do tf calls
            cornerTagName (String): Name of corner april tag frame
            ballTagNames (List[String]): List of ball tags to keep track of.
        """
        self.node = node

        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.static_broadcaster = StaticTransformBroadcaster(self.node)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.cornerTagName = cornerTagName
        self.ballNames = ballTagNames

        self.tableWidth = 0.31
        self.tableLength = 0.52
        self.tableHeight = 0.09

    def tableTagExists(self):
        """Check if tag is visable.

        Returns:
            Bool: Is the tag visable relative to the base?
        """
        try:
            self.tf_buffer.lookup_transform(
                'base', self.cornerTagName, rclpy.time.Time()
            )
            return True

        except TransformException:
            return False

    def tableExists(self):
        """Check if table has been built.

        Returns:
            Bool: Has the table been built?
        """
        try:
            self.tf_buffer.lookup_transform(
                'base', 'table_center', rclpy.time.Time()
            )
            return True

        except TransformException:
            return False

    def buildTable(self):
        """Construct the table relashonships to base."""
        try:
            baseToCornerTag = self.tf_buffer.lookup_transform(
                'base', self.cornerTagName, rclpy.time.Time()
            )
            centerPose = Pose()
            centerPose.position.x = 0.09 + self.tableLength / 2
            centerPose.position.y = -(0.08 + self.tableWidth / 2)
            centerPose.position.z = self.tableHeight - 0.035
            centerInBase = do_transform_pose(centerPose, baseToCornerTag)

            t = TransformStamped()

            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = 'base'
            t.child_frame_id = 'table_center'

            t.transform.translation.x = centerInBase.position.x
            t.transform.translation.y = centerInBase.position.y
            t.transform.translation.z = centerInBase.position.z

            q = centerInBase.orientation
            q_list = [q.x, q.y, q.z, q.w]
            _, _, yaw = euler_from_quaternion(q_list)
            tableOrientation = quaternion_from_euler(0.0, 0.0, yaw - np.pi / 2)
            t.transform.rotation = tableOrientation

            # Publish table in base frame
            self.static_broadcaster.sendTransform(t)

            p = TransformStamped()
            p.header.stamp = self.node.get_clock().now().to_msg()
            p.header.frame_id = 'table_center'

            p.child_frame_id = 'pocket1'
            p.transform.translation.x = -self.tableWidth / 2
            p.transform.translation.y = -self.tableLength / 2
            self.static_broadcaster.sendTransform(p)

            p.child_frame_id = 'pocket2'
            p.transform.translation.x = -self.tableWidth / 2
            p.transform.translation.y = 0.0
            self.static_broadcaster.sendTransform(p)

            p.child_frame_id = 'pocket3'
            p.transform.translation.x = -self.tableWidth / 2
            p.transform.translation.y = self.tableLength / 2
            self.static_broadcaster.sendTransform(p)

            p.child_frame_id = 'pocket4'
            p.transform.translation.x = self.tableWidth / 2
            p.transform.translation.y = self.tableLength / 2
            self.static_broadcaster.sendTransform(p)

            p.child_frame_id = 'pocket5'
            p.transform.translation.x = self.tableWidth / 2
            p.transform.translation.y = 0.0
            self.static_broadcaster.sendTransform(p)

            p.child_frame_id = 'pocket6'
            p.transform.translation.x = self.tableWidth / 2
            p.transform.translation.y = -self.tableLength / 2
            self.static_broadcaster.sendTransform(p)

        except TransformException:
            self.node.get_logger().error('Failed to build table')

    def pocketPositions(self):
        """Get list of pocket positions.

        Returns:
            List: List of points representing pockets in base frame
        """
        pocketPos = []
        try:
            p1 = self.tf_buffer.lookup_transform(
                'base', 'pocket1', rclpy.time.Time()
            )
            p1Translation = p1.transform.translation
            pocketPos.append(p1Translation)

            p2 = self.tf_buffer.lookup_transform(
                'base', 'pocket2', rclpy.time.Time()
            )
            p2Translation = p2.transform.translation
            pocketPos.append(p2Translation)

            p3 = self.tf_buffer.lookup_transform(
                'base', 'pocket3', rclpy.time.Time()
            )
            p3Translation = p3.transform.translation
            pocketPos.append(p3Translation)

            p4 = self.tf_buffer.lookup_transform(
                'base', 'pocket4', rclpy.time.Time()
            )
            p4Translation = p4.transform.translation
            pocketPos.append(p4Translation)

            p5 = self.tf_buffer.lookup_transform(
                'base', 'pocket5', rclpy.time.Time()
            )
            p5Translation = p5.transform.translation
            pocketPos.append(p5Translation)

            p6 = self.tf_buffer.lookup_transform(
                'base', 'pocket6', rclpy.time.Time()
            )
            p6Translation = p6.transform.translation
            pocketPos.append(p6Translation)

            self.lastPockets = pocketPos
            return pocketPos

        except TransformException:
            self.node.get_logger().error('Failed to get transform for pockets')
            return pocketPos

    def ballPositions(self):
        """Get current positions of tracked balls.

        Returns:
            Dict: Dict of ball names to points in base frame
        """
        ballDict = {}
        for ball in self.ballNames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base', ball, rclpy.time.Time()
                )

                if (
                    self.node.get_clock().now().to_msg().sec
                    - tf.header.stamp.sec
                    <= 1
                ):
                    ballDict[ball] = tf.transform.translation
            except TransformException:
                self.node.get_logger().debug(
                    'Failed to get transform for a ball'
                )

        return ballDict

    def center(self):
        """Get center of table.

        Returns:
            Transform: tf representing center of table
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'base', 'table_center', rclpy.time.Time()
            )
            return tf

        except TransformException:
            self.node.get_logger().error('Failed to get transform for center')

    def strikeTransform(self, eeMotion):
        """Get ee motion in base frame.

        Args:
            eeMotion (Pose): desired ee motion

        Returns:
            Pose: same Pose, but in base frame.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'base', 'fer_hand_tcp', rclpy.time.Time()
            )
            eeMotionInBase = do_transform_pose(eeMotion, tf)
            return eeMotionInBase

        except TransformException:
            self.node.get_logger().error(
                'Failed to get transform for eeMotion'
            )

    def cameraPosition(self):
        """Get position of camera in base frame.

        Returns:
            Pose: Pose of camera in base frame
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'base', 'camera_link', rclpy.time.Time()
            )
            return tf.transform.translation
        except TransformException:
            self.node.get_logger().error('Failed to get transform for balls')
            return None
