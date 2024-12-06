from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Vector3

import rclpy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def midpoint(point1, point2):
    mid = Vector3()
    mid.x = (point1.x + point2.x) / 2
    mid.y = (point1.y + point2.y) / 2
    mid.z = (point1.z + point2.z) / 2
    return mid


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
    def __init__(self, node, cornerTagName, ballTagNames):
        self.node = node

        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.cornerName = cornerTagName
        self.ballNames = ballTagNames

        self.tableWidth = 0.31
        self.tableLength = 0.51
        self.tableHeight = 0.09
        self.tableTagSize = 0.19

    def tableExists(self):
        try:
            self.tf_buffer.lookup_transform(
                'base', self.cornerName, rclpy.time.Time()
            )
            return True

        except TransformException:
            return False

    def pocketPositions(self):
        pocketPos = []
        try:
            tf = self.tf_buffer.lookup_transform(
                'base', self.cornerName, rclpy.time.Time()
            )
            tagTranslation = tf.transform.translation

            p1 = Vector3
            p1.x = tagTranslation.x + self.tableTagSize / 2
            p1.y = tagTranslation.y + self.tableTagSize / 2
            p1.z = tagTranslation.z + self.tableHeight
            pocketPos.append(p1)

            p2 = Vector3
            p2.x = tagTranslation.x + self.tableTagSize / 2
            p2.y = (
                tagTranslation.y + self.tableTagSize / 2 + self.tableLength / 2
            )
            p2.z = tagTranslation.z + self.tableHeight
            pocketPos.append(p2)

            p3 = Vector3
            p3.x = tagTranslation.x + self.tableTagSize / 2
            p3.y = tagTranslation.y + self.tableTagSize / 2 + self.tableLength
            p3.z = tagTranslation.z + self.tableHeight
            pocketPos.append(p3)

            p4 = Vector3
            p4.x = tagTranslation.x + self.tableTagSize / 2 + self.tableWidth
            p4.y = tagTranslation.y + self.tableTagSize / 2 + self.tableLength
            p4.z = tagTranslation.z + self.tableHeight
            pocketPos.append(p4)

            p5 = Vector3
            p5.x = tagTranslation.x + self.tableTagSize / 2 + self.tableWidth
            p5.y = (
                tagTranslation.y + self.tableTagSize / 2 + self.tableLength / 2
            )
            p5.z = tagTranslation.z + self.tableHeight
            pocketPos.append(p5)

            p6 = Vector3
            p6.x = tagTranslation.x + self.tableTagSize / 2 + self.tableWidth
            p6.y = tagTranslation.y + self.tableTagSize / 2
            p6.z = tagTranslation.z + self.tableHeight
            pocketPos.append(p6)

        except TransformException:
            self.node.get_logger().error("Failed to get transform for pockets")

        return pocketPos

    def ballPositions(self):
        ballPos = []
        try:
            for ball in self.ballNames:
                tf = self.tf_buffer.lookup_transform(
                    'base', ball, rclpy.time.Time()
                )
                ballPos.append(tf.transform.translation)
        except TransformException:
            self.node.get_logger().error("Failed to get transform for balls")

        return ballPos

    def cameraPosition(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'base', 'camera_link', rclpy.time.Time()
            )
            return tf.transform.translation
        except TransformException:
            self.node.get_logger().error("Failed to get transform for balls")
            return None
