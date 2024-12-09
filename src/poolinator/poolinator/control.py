from enum import auto, Enum

import rclpy
from rclpy.node import Node

from poolinator.world import World
from poolinator.bridger import quaternion_from_euler

from motion_planning_interface.MotionPlanningInterface import (
    MotionPlanningInterface,
)

from std_srvs.srv import Empty

from geometry_msgs.msg import Point, Pose, Quaternion

import numpy as np

from poolinator.poolAlgorithm import PoolAlgorithm


class State(Enum):
    """Keep track of the robots current command."""

    SETUP = auto()
    LIVE = auto()
    STANDBY = auto()
    EXECUTING = auto()


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.logger = self.get_logger()

        timer_period = 1.0  # secs
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.move_c1 = self.create_service(
            Empty, 'move_c1', self.move_c1_callback
        )
        self.move_c2 = self.create_service(
            Empty, 'move_c2', self.move_c2_callback
        )
        self.move_c3 = self.create_service(
            Empty, 'move_c3', self.move_c3_callback
        )
        self.move_c4 = self.create_service(
            Empty, 'move_c4', self.move_c4_callback
        )
        self.center_strike = self.create_service(
            Empty, 'center_strike', self.center_strike_callback
        )

        self.strike_redball = self.create_service(
            Empty, 'strike_redball', self.strike_redball_callback
        )

        self.strike_cue = self.create_service(
            Empty, 'strike_cue', self.strike_cue_callback
        )

        self.mp_interface = MotionPlanningInterface(self)
        self.world = World(
            self,
            'table_tag',
            ['red_ball'],
        )

        self.state = State.SETUP

        self.ballDict = None
        self.pockets = None
        self.pool_algo = None

    async def timer_callback(self):
        # Stay in setup state until pool table frames exist from CV
        if self.state == State.SETUP:
            if self.world.tableTagExists():
                self.world.buildTable()
            if self.world.tableExists():
                self.setup_scene()
                self.state = State.STANDBY
                return

        if self.state == State.STANDBY:
            self.ballDict = self.world.ballPositions()
            self.pockets = self.world.pocketPositions()

        if self.state == State.LIVE:
            self.state = State.EXECUTING

            self.ballDict = self.world.ballPositions()
            self.pockets = self.world.pocketPositions()
            for key, value in self.ballDict.items():
                if key == 'red_ball':
                    ball = value
            pool_algo = PoolAlgorithm(self.ballDict, self.pockets)

            eePose = pool_algo.test_strike_pose(ball, self.pockets[0])
            await self.strike_ball(eePose)

            # await self.stand_by()

            self.state = State.STANDBY

    async def strike_cue_callback(self, request, response):
        if self.state == State.STANDBY:
            self.state = State.LIVE

        return response

    async def strike_redball_callback(self, request, response):
        if self.pockets:
            for key, value in self.ballDict.items():
                if key == 'red_ball':
                    ball = value
            pool_algo = PoolAlgorithm(self.ballDict, self.pockets)
            eePose = pool_algo.test_strike_pose(ball, self.pockets[2])
            self.get_logger().info(f'eePose: {eePose}')

            await self.strike_ball(eePose)

        return response

    async def move_c1_callback(self, request, response):
        pocket_pos = self.world.pocketPositions()
        c1 = pocket_pos[0]
        eePose = Pose()
        eePosition = Point()
        eePosition.x = c1.x
        eePosition.y = c1.y
        eePosition.z = c1.z + 0.2
        eePose.position = eePosition
        eeOrientation = quaternion_from_euler(np.pi, 0, 0)
        eePose.orientation = eeOrientation
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        self.logger.info('Move Done')
        return response

    async def move_c2_callback(self, request, response):
        pocket_pos = self.world.pocketPositions()
        c2 = pocket_pos[2]
        eePose = Pose()
        eePosition = Point()
        eePosition.x = c2.x
        eePosition.y = c2.y
        eePosition.z = c2.z + 0.2
        eePose.position = eePosition
        eeOrientation = quaternion_from_euler(np.pi, 0, 0)
        eePose.orientation = eeOrientation
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        self.logger.info('Move Done')
        return response

    async def move_c3_callback(self, request, response):
        pocket_pos = self.world.pocketPositions()
        c3 = pocket_pos[3]
        eePose = Pose()
        eePosition = Point()
        eePosition.x = c3.x
        eePosition.y = c3.y
        eePosition.z = c3.z + 0.2
        eePose.position = eePosition
        eeOrientation = quaternion_from_euler(np.pi, 0, 0)
        eePose.orientation = eeOrientation
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        self.logger.info('Move Done')
        return response

    async def move_c4_callback(self, request, response):
        pocket_pos = self.world.pocketPositions()
        c4 = pocket_pos[5]
        eePose = Pose()
        eePosition = Point()
        eePosition.x = c4.x
        eePosition.y = c4.y
        eePosition.z = c4.z + 0.2
        eePose.position = eePosition
        eeOrientation = quaternion_from_euler(np.pi, 0, 0)
        eePose.orientation = eeOrientation
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        self.logger.info('Move Done')
        return response

    async def center_strike_callback(self, request, response):
        center_pos = self.world.center()
        eePose = Pose()
        eePosition = Point()
        eePosition.x = center_pos.transform.translation.x - 0.15
        eePosition.y = center_pos.transform.translation.y
        eePosition.z = center_pos.transform.translation.z + 0.28
        eePose.position = eePosition
        # eePose.orientation = center_pos.transform.rotation
        eeOrientation = quaternion_from_euler(np.pi, 0, -np.pi / 4)
        eePose.orientation = eeOrientation

        self.get_logger().info(f'eePose center strike: {eePose}')

        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        eePose.position.z -= 0.11
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        eePose.position.x += 0.11
        resultFuture = await self.mp_interface.mp.pathPlanPose(
            eePose, startJoints=None, max_vel=0.4, max_accel=0.4
        )
        await resultFuture
        return response

    async def strike_ball(self, que_pose):
        # Caroline, this is where we woulc call you function
        # Or pass it into function
        eePose = que_pose

        # Standoff position
        eePose.position.z += 0.25
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture

        eeMotion = Pose()
        # Move along x axis of ee
        eeMotion.position.x = -0.17
        movement = self.world.strikeTransform(eeMotion)
        eePose.position.x = movement.position.x
        eePose.position.y = movement.position.y
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture

        # Strike position
        eePose.position.z -= 0.11
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture

        # Hit through
        eeMotion = Pose()
        # Move along x axis of ee
        eeMotion.position.x = 0.06
        movement = self.world.strikeTransform(eeMotion)
        eePose.position.x = movement.position.x
        eePose.position.y = movement.position.y
        resultFuture = await self.mp_interface.mp.pathPlanPose(
            eePose, startJoints=None, max_vel=0.5, max_accel=0.5
        )
        await resultFuture

    async def stand_by(self):
        readyPose = Pose()
        readyPose.position.x = 0.3
        readyPose.position.z = 0.48
        readyPose.orientation.x = 1.0
        resultFuture = await self.mp_interface.mp.pathPlanPose(readyPose)
        await resultFuture

    def setup_scene(self):
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
    rclpy.init()
    n = ControlNode()
    rclpy.spin(n)
