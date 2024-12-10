"""
Controls the arm to calculate and execute moves.

SERVICES:
  + strike_cue (std_srvs/srv/Empty) - Starts the game either from
    startup or from a cue ball reset.

"""

from enum import auto, Enum

from geometry_msgs.msg import Pose

from motion_planning_interface.MotionPlanningInterface import (
    MotionPlanningInterface,
)

import numpy as np

from poolinator.poolAlgorithm import PoolAlgorithm
from poolinator.world import World

import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty


class State(Enum):
    """Keep track of the robots current state."""

    SETUP = auto()
    LIVE = auto()
    STANDBY = auto()
    EXECUTING = auto()


class ControlNode(Node):
    """Controls the robot through the pool game."""

    def __init__(self):
        """Set up the state."""
        super().__init__('control')
        self.logger = self.get_logger()

        timer_period = 1.0  # secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.strike_cue = self.create_service(
            Empty, 'strike_cue', self.strike_cue_callback
        )

        self.mp_interface = MotionPlanningInterface(self)
        self.world = World(
            self,
            'table_tag',
            ['red_ball', 'blue0', 'blue1', 'blue2', 'blue3'],
        )

        self.state = State.SETUP

        self.ballDict = None
        self.pockets = None
        self.pool_algo = None

    async def timer_callback(self):
        """Check state of control loop for running or waiting."""
        # Stay in setup state until pool table frames exist from CV
        if self.state == State.SETUP:
            if self.world.tableTagExists():
                self.world.buildTable()
            if self.world.tableExists():
                self.setup_scene()
                self.update_world()
                self.pool_algo = PoolAlgorithm(self.ballDict, self.pockets)
                self.state = State.STANDBY
                return

        # Ready to act
        if self.state == State.STANDBY:
            self.update_world()

        # In the movement loop
        if self.state == State.LIVE:
            self.state = State.EXECUTING

            self.update_world()
            ball = None
            for key, value in self.ballDict.items():
                if key == 'red_ball':
                    ball = value

            # If no cue, standby
            if ball is None:
                self.state = State.STANDBY
                return

            # Get strike pose
            eePose = self.pool_algo.calc_strike_pose(
                ball, self.ballDict, self.pockets
            )

            await self.strike_ball(eePose)

            await self.stand_by()

            # Rebuild table in case it moved
            self.world.buildTable()
            self.update_world()

            # Go next
            self.state = State.LIVE

    def update_world(self):
        """Update the pockets and balls."""
        self.ballDict = self.world.ballPositions()
        self.pockets = self.world.pocketPositions()

    async def strike_cue_callback(self, request, response):
        """Service to init robot from standby.

        Args:
            request (Empty): Unused
            response (Empty): Unused

        Returns:
            Empty: unused
        """
        if self.state == State.STANDBY:
            self.state = State.LIVE

        return response

    async def strike_ball(self, que_pose):
        """Run motions to strike ball given pose."""
        eePose = que_pose

        # Standoff position
        eePose.position.z = 0.35
        resultFuture = await self.mp_interface.mp.pathPlanPose(
            eePose, startJoints=None, max_vel=0.2, max_accel=0.2
        )
        await resultFuture

        # Strike position
        eePose.position.z -= 0.09
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture

        # Hit through
        eeMotion = Pose()
        # Move along x axis of ee
        eeMotion.position.x = 0.07
        movement = self.world.strikeTransform(eeMotion)
        eePose.position.x = movement.position.x
        eePose.position.y = movement.position.y
        resultFuture = await self.mp_interface.mp.pathPlanPose(
            eePose, startJoints=None, max_vel=0.8, max_accel=0.8
        )
        await resultFuture

    async def stand_by(self):
        """Return to standby position."""
        joints = {}
        joints['fer_joint1'] = np.pi / 4
        joints['fer_joint2'] = -np.pi / 4
        joints['fer_joint3'] = 0.0
        joints['fer_joint4'] = -3 * np.pi / 4
        joints['fer_joint5'] = 0.0
        joints['fer_joint6'] = np.pi / 2
        joints['fer_joint7'] = np.pi / 4
        resultFuture = await self.mp_interface.mp.pathPlanJoints(
            joints, startJoints=None, max_vel=0.2, max_accel=0.2
        )
        await resultFuture

    def setup_scene(self):
        """Build the planning scene in Rviz."""
        tableWidth = 2.0
        tableLength = 2.4
        tableHeight = 0.1

        poseTable = Pose()
        poseTable.position.x = 0.0
        poseTable.position.y = 0.0
        poseTable.position.z = -tableHeight / 2 - 0.01
        self.mp_interface.ps.add_box(
            'table', (tableWidth, tableLength, tableHeight), poseTable
        )

        cameraWidth = 0.1
        cameraLength = 1.0
        cameraHeight = 0.05

        cameraPoint = self.world.cameraPosition()
        poseCamera = Pose()
        poseCamera.position.x = cameraPoint.x
        poseCamera.position.y = cameraPoint.y
        poseCamera.position.z = cameraPoint.z
        self.mp_interface.ps.add_box(
            'camera', (cameraWidth, cameraLength, cameraHeight), poseCamera
        )

        poolTableWidth = 0.31
        poolTableLength = 0.51
        # make it a little shorter than real to not be too restrictive
        poolTableHeight = 0.08

        posePoolTable = Pose()
        poolTableTf = self.world.center()
        posePoolTable.position.x = poolTableTf.transform.translation.x
        posePoolTable.position.y = poolTableTf.transform.translation.y
        posePoolTable.position.z = poolTableTf.transform.translation.z - 0.045
        posePoolTable.orientation = poolTableTf.transform.rotation
        self.mp_interface.ps.add_box(
            'poolTable',
            (poolTableWidth, poolTableLength, poolTableHeight),
            posePoolTable,
        )


def main():
    """Run main function."""
    rclpy.init()
    n = ControlNode()
    rclpy.spin(n)
