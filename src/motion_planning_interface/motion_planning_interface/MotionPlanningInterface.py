"""Combines the various pieces of the MoveIt wrapper classes."""

from motion_planning_interface.MotionPlanner import MotionPlanner
from motion_planning_interface.Gripper import Gripper


class MotionPlanningInterface:
    """Combines the various pieces of the MoveIt wrapper classes."""

    def __init__(self, node):
        """Initialize the classes needed for MoveIt.

        Args:
            node (Node): The parent node to use for interfacing with ROS
        """
        self.node = node
        self.mp = MotionPlanner(self.node, 'fer_arm')
        self.gripper = Gripper(self.node, 'hand')
