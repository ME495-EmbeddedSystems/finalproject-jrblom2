from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import do_transform_pose

import rclpy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

"""
    F
    |
    |
C1-----C2
|      |
C4-----C3
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

    def cornerPositions(self):
        pass
