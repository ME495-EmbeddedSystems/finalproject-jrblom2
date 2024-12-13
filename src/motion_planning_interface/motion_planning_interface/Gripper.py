"""
Wrapper for simple motion planning with MoveIt.

Contains functions for opening and closing the gripper
"""

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
)

from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class Gripper:
    """A class for opening and closing the gripper."""

    def __init__(self, node, groupName):
        """Initialize the motion planner with the callers Node.

        Args:
            node (Node): The node to use for interacting with ROS systems.
            groupName (String): The name of the link group to plan for.

        Raises:
            Exception: Raises exception if move action server does not exist.
        """
        self.node = node
        self.groupName = groupName

        self._cbgroup = MutuallyExclusiveCallbackGroup()
        self._client = ActionClient(
            self.node, MoveGroup, 'move_action', callback_group=self._cbgroup)
        if not self._client.wait_for_server(timeout_sec=10):
            raise Exception('move_action action server not ready')

    async def openGripper(self):
            """Open the gripper.

            Returns:
                _type_: _description_
            """

            goalMessage = MotionPlanRequest()
            goalMessage.group_name = self.groupName

            # Format the goal joints
            goalConstraints = Constraints()
            goalJointConstraints = []
            jointConstraint = JointConstraint()
            jointConstraint.joint_name =  'fer_finger_joint1'
            jointConstraint.position = .035
            jointConstraint.tolerance_above = 0.0001
            jointConstraint.tolerance_below = 0.0001
            jointConstraint.weight = 1.0
            goalJointConstraints.append(jointConstraint)
            goalConstraints.joint_constraints = goalJointConstraints

            goalMessage.goal_constraints = [goalConstraints]
            goalMessage.max_velocity_scaling_factor = 0.1
            goalMessage.max_acceleration_scaling_factor = 0.1
            goalMessage.allowed_planning_time = 5.0
            goalMessage.num_planning_attempts = 10
            goalRequest = MoveGroup.Goal(request=goalMessage)

            response_goal_handle = await self._client.send_goal_async(goalRequest)
            resultFuture = response_goal_handle.get_result_async()
            return resultFuture
    
    async def closeGripper(self):
            """Close the gripper,

            Returns:
                _type_: _description_
            """

            goalMessage = MotionPlanRequest()
            goalMessage.group_name = self.groupName

            # Format the goal joints
            goalConstraints = Constraints()
            goalJointConstraints = []
            jointConstraint = JointConstraint()
            jointConstraint.joint_name =  'fer_finger_joint1'
            jointConstraint.position = 0.0
            jointConstraint.tolerance_above = 0.0001
            jointConstraint.tolerance_below = 0.0001
            jointConstraint.weight = 1.0
            goalJointConstraints.append(jointConstraint)
            goalConstraints.joint_constraints = goalJointConstraints

            goalMessage.goal_constraints = [goalConstraints]
            goalMessage.max_velocity_scaling_factor = 0.1
            goalMessage.max_acceleration_scaling_factor = 0.1
            goalMessage.allowed_planning_time = 5.0
            goalMessage.num_planning_attempts = 10
            goalRequest = MoveGroup.Goal(request=goalMessage)

            response_goal_handle = await self._client.send_goal_async(goalRequest)
            resultFuture = response_goal_handle.get_result_async()
            return resultFuture