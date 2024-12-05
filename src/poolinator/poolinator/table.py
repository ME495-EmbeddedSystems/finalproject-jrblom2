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


class PoolTable:
    def __init__(self, node, cornerTagNames, ballTagNames):
        self.node = node

        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.cornerNames = cornerTagNames
        self.ballNames = ballTagNames

    def tableExists(self):
        try:
            for corner in self.cornerNames:
                self.tf_buffer.lookup_transform(
                    'base', corner, rclpy.time.Time()
                )
            return True

        except TransformException:
            return False

    def pocketPositions(self):
        pocketPos = []
        for corner in self.cornerNames:
            tf = self.tf_buffer.lookup_transform(
                'base', corner, rclpy.time.Time()
            )
            pocketPos.append(tf.transform.translation)
        pocketPos.insert(1, midpoint(pocketPos[0], pocketPos[1]))
        pocketPos.insert(4, midpoint(pocketPos[2], pocketPos[3]))
        return pocketPos

    def ballPositions(self):
        ballPos = []
        for ball in self.ballNames:
            tf = self.tf_buffer.lookup_transform(
                'base', ball, rclpy.time.Time()
            )
            ballPos.append(tf.transform.translation)
        return ballPos
