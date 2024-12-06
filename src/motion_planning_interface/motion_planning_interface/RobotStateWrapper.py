"""
Wrapper for simple motion planning with MoveIt.

Contains functions for calculating the inverse kinematics of a robot given an
arbitrary end-effector pose and the forward kinematics given an arbitrary joint 
state, and retrieves the joint configuration or robot pose.
"""
from geometry_msgs.msg import Pose, PoseStamped

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MoveItErrorCodes,
    PositionIKRequest,
    RobotState,
)

from moveit_msgs.srv import GetPositionIK, GetPositionFK

from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import JointState

class RobotStateWrapper:
    """A simple motion planner interface for MoveIt."""

    def __init__(self, node, groupName):
        """
        Initialize the robot state with the callers Node.

        Args:
        ----
        node (Node): 
            The node to use for interacting with ROS systems.
        groupName (String): 
            The name of the link group to plan for.

        Raises
        ------
        Exception: 
            Raises exception if move action server does not exist.

        """
        self.node = node
        self.groupName = groupName
        self.jointStates = JointState()
        self.robotState = RobotState()

        # Create subscribers
        self.js_subscriber = self.node.create_subscription(JointState, 'joint_states', self.js_callback, 10)
        self.rs_subscriber = self.node.create_subscription(RobotState, 'robot_description', self.rs_callback, 10)

        # Initialize clients
        self._cbgroup = MutuallyExclusiveCallbackGroup()  # Set up client callback group
        self._IKClient = self.node.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self._cbgroup,
        )
        self._FKClient = self.node.create_client(
            GetPositionFK,
            '/compute_fk',
            callback_group=self._cbgroup,
        )
        self._client = ActionClient(self.node, MoveGroup, 'move_action')

        if not self._client.wait_for_server(timeout_sec=10):
            raise Exception('move_action action server not ready, robot state')

    def js_callback(self, msg):
        self.jointStates = msg
        
    async def IK(self, eePose=None):
        """
        Perform inverse kinematics on the robot from an arbitrary pose.

        If no pose is specified
        the current location is used

        Args
        ----
        eePose (Pose): 
            Arbitrary end pose.
        
        Returns
        -------
        RobotState(): 
            Robot state solution.
        MoveItErrorCodes(): 
            Errors from IK.

        """
        IKMessage = GetPositionIK.Request()
        IKRequest = PositionIKRequest()
        IKRequest.group_name = self.groupName
        IKRequest.avoid_collisions = True
        IKRequest.timeout.sec = 10
        stampedPose = PoseStamped()
        stampedPose.header.stamp = self.node.get_clock().now().to_msg()
        if eePose is not None:
            stampedPose.pose = eePose
        IKRequest.robot_state = self.robotState
        IKRequest.pose_stamped = stampedPose
        IKMessage.ik_request = IKRequest

        IKResponse = await self._IKClient.call_async(IKMessage)
        self.node.get_logger().info(f'{IKResponse.solution}')
        return IKResponse.solution

    async def FK(self, joint_states=None):
        """
        Perform forward kinematics on the robot from an arbitrary joint state.

        If no joint poses are specified the current location is used.

        Args:
        ----
        joint_states (JointState()): Arbitrary joint state

        Returns
        -------
        [PoseStamped]: 
            Stmaped poses of the requested links
        [fk_link_names]: 
            List of link names for the poses
        MoveItErrorCodes: 
            Errors from the calculation

        """
        FKRequest = GetPositionFK.Request()
        FKRequest.header.stamp = self.node.get_clock().now().to_msg()
        FKRequest.fk_link_names = self.jointStates.name
        self.node.get_logger().info(f'{FKRequest.fk_link_names}')
        FKRequest.robot_state = self.robotState
        
        if joint_states is not None:
            FKRequest.robot_state.joint_state = self.jointStates

        FKResponse = await self._FKClient.call_async(FKRequest)
        self.node.get_logger().info(f'Error: {FKResponse.error_code}')
        return FKResponse.pose_stamped


        
        