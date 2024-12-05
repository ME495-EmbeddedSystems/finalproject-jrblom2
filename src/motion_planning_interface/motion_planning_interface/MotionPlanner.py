"""
Wrapper for simple motion planning with MoveIt.

Contains functions for planning a path based on goal joints, a goal pose,
a cartesian path goal pose, or planning to a saved configuration.
"""

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    RobotState,
    GenericTrajectory,
    PositionIKRequest,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK

from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import JointState


def robotStateFromJoints(joints):
    """Generate a RobotState from a dictionary of named joint values.

    Args:
        joints (dict): dictionary of string joint names to float radian values

    Returns:
        RobotState object with the joint_state field set
    """
    robotState = RobotState()
    jointState = JointState()
    startingJointNames = []
    startingJointValues = []
    for key, value in joints.items():
        startingJointNames.append(key)
        startingJointValues.append(value)
    jointState.name = startingJointNames
    jointState.position = startingJointValues
    robotState.joint_state = jointState
    return robotState


def goalJointsFromState(robotState):
    """Build a set of goal constraints from a robot state.

    Args:
        robotState (RobotState): the state containing the goal joints.

    Returns:
        [JointConstraint]: the desired joint states as a list of constraints.
    """
    names = robotState.joint_state.name
    positions = robotState.joint_state.position
    goalJointConstraints = []
    for i, name in enumerate(names):
        jointConstraint = JointConstraint()
        jointConstraint.joint_name = name
        jointConstraint.position = positions[i]
        jointConstraint.tolerance_above = 0.0001
        jointConstraint.tolerance_below = 0.0001
        jointConstraint.weight = 1.0
        goalJointConstraints.append(jointConstraint)
    return goalJointConstraints


class MotionPlanner:
    """A simple motion planner interface for MoveIt."""

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
        self._cartesianClient = self.node.create_client(
            GetCartesianPath,
            '/compute_cartesian_path',
            callback_group=self._cbgroup,
        )
        self._IKClient = self.node.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self._cbgroup,
        )
        self._client = ActionClient(
            self.node, MoveGroup, 'move_action', callback_group=self._cbgroup
        )
        if not self._client.wait_for_server(timeout_sec=10):
            raise Exception('move_action action server not ready')

    async def robotStateFromPose(self, startJoints, pose):
        """Call MoveIt IK service to get joints from Pose.

        Args:
            startJoints (dict): optional starting position
            pose (Pose): the goal Pose

        Returns:
            Robotstate: the robot state matching the pose
        """
        poseMessage = GetPositionIK.Request()
        IKRequest = PositionIKRequest()
        IKRequest.group_name = self.groupName
        if startJoints is not None:
            IKRequest.robot_state = robotStateFromJoints(startJoints)
        IKRequest.avoid_collisions = True

        # The stamped pose of the link, when the IK solver computes
        # the joint values for all the joints in a group.
        stampedPose = PoseStamped()
        stampedPose.pose = pose

        IKRequest.pose_stamped = stampedPose
        IKRequest.timeout.sec = 10

        poseMessage.ik_request = IKRequest
        robotState = await self._IKClient.call_async(poseMessage)
        return robotState.solution

    async def pathPlanJoints(self, goalJoints, startJoints=None):
        """Plan a path to a goal set of joints.

        If no start joints are passed
        the current robot configuration is used.

        Args:
            goalJoints (dict): Goal configuration.
            startJoints (dict, optional): Starting configuration.
                Defaults to None.

        Returns:
            _type_: _description_
        """

        goalMessage = MotionPlanRequest()
        goalMessage.group_name = self.groupName

        # If starting joints, include them in request
        if startJoints is not None:
            goalMessage.start_state = robotStateFromJoints(startJoints)

        # Format the goal joints
        goalConstraints = Constraints()
        goalJointConstraints = []
        for key, value in goalJoints.items():
            jointConstraint = JointConstraint()
            jointConstraint.joint_name = key
            jointConstraint.position = value
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

    async def pathPlanPose(self, goalPose=None, startJoints=None):
        """Plan a path to a specified pose (position and orientation)
           from any starting configuration.

        Args:
            goalPose (Pose): Goal configuration as a Pose
            (rotation and orientation).
            startJoints (dict, optional): Starting configuration.
                Defaults to None.

        Returns:
            _type_: _description_
        """
        goalState = await self.robotStateFromPose(startJoints, goalPose)

        # Put goal joints in the dict format that
        # is accepted by pathPlanJoints()
        goalJoints = {}
        for name, position in zip(
            goalState.joint_state.name, goalState.joint_state.position
        ):
            goalJoints[name] = position

        resultFuture = await self.pathPlanJoints(goalJoints, startJoints)
        return resultFuture

    async def cartesianPath(self, goalPose, startJoints=None):
        """Plan a path to a pose by a cartesian path.

        Args:
            goalPose (Pose): Goal configuration as a Pose.
            startJoints (dict, optional): Starting configuration.
                Defaults to None.

        Returns:
            _type_: _description_
        """
        # Call compute cartesian path
        cartesianRequest = GetCartesianPath.Request()
        cartesianRequest.group_name = self.groupName
        if startJoints is not None:
            cartesianRequest.start_state = robotStateFromJoints(startJoints)
        cartesianRequest.avoid_collisions = True
        cartesianRequest.max_step = 0.01
        cartesianRequest.waypoints = [goalPose]
        cartesianResponse = await self._cartesianClient.call_async(
            cartesianRequest
        )

        # Feed that result into the motion planner
        goalMessage = MotionPlanRequest()
        goalMessage.group_name = self.groupName

        # If starting joints, include them in request
        if startJoints is not None:
            goalMessage.start_state = robotStateFromJoints(startJoints)

        # Format the goal joints
        goalConstraints = Constraints()
        goalState = await self.robotStateFromPose(startJoints, goalPose)
        goalConstraints.joint_constraints = goalJointsFromState(goalState)
        goalMessage.goal_constraints = [goalConstraints]

        genericTrajectory = GenericTrajectory()
        genericTrajectory.joint_trajectory = [
            cartesianResponse.solution.joint_trajectory
        ]
        goalMessage.reference_trajectories = [genericTrajectory]
        goalMessage.max_velocity_scaling_factor = 0.1
        goalMessage.max_acceleration_scaling_factor = 0.1
        goalMessage.allowed_planning_time = 5.0
        goalMessage.num_planning_attempts = 10
        goalRequest = MoveGroup.Goal(request=goalMessage)

        response_goal_handle = await self._client.send_goal_async(goalRequest)
        resultFuture = response_goal_handle.get_result_async()
        return resultFuture
