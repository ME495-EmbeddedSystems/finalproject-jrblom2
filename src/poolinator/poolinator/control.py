from enum import auto, Enum

import rclpy
from rclpy.node import Node

from poolinator.poolinator.world import World

from motion_planning_interface.motion_planning_interface.MotionPlanningInterface import (
    MotionPlanningInterface,
)

from std_srvs.srv import Empty

from geometry_msgs.msg import Point, Pose, Quaternion

import numpy as np

from tf_transformations import quaternion_from_euler


class State(Enum):
    """Keep track of the robots current command."""

    SETUP = auto()
    RUNNING = auto()


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')

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
        self.mp_interface = MotionPlanningInterface(self)
        self.world = World(
            self,
            [
                'tagStandard41h12:0',
                'tagStandard41h12:1',
                'tagStandard41h12:2',
                'tagStandard41h12:3',
            ],
            ['b1'],
        )

        self.state = State.SETUP

    def timer_callback(self):
        # Stay in setup state until pool table frames exist from CV
        # if self.state == State.SETUP:
        #     if self.world.tableExists():
        #         # Add planning scene objects here before running
        #         self.setup_scene()
        #         self.state = State.RUNNING
        #     return

        # if self.state == State.RUNNING:
        #     self.get_logger().info(f"{self.world.pocketPositions()}")
        pass

    async def move_c1_callback(self, request, response):
        pocket_pos = self.table.pocketPositions()
        c1 = pocket_pos[0]
        eePose = Pose()
        eePosition = Point()
        eePosition.x = c1.x
        eePosition.y = c1.y
        eePosition.z = c1.z + 0.2
        eePose.position = eePosition
        eeOrientation = Quaternion()
        eeOrientation.w = np.cos(np.pi / 2)
        eeOrientation.x = np.sin(np.pi / 2)
        eePose.orientation = eeOrientation
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        self.logger.info('Move Done')
        return response

    async def move_c2_callback(self, request, response):
        pocket_pos = self.table.pocketPositions()
        c2 = pocket_pos[2]
        eePose = Pose()
        eePosition = Point()
        eePosition.x = c2.x
        eePosition.y = c2.y
        eePosition.z = c2.z + 0.2
        eePose.position = eePosition
        eeOrientation = Quaternion()
        eeOrientation.w = np.cos(np.pi / 2)
        eeOrientation.x = np.sin(np.pi / 2)
        eePose.orientation = eeOrientation
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        self.logger.info('Move Done')
        return response

    async def move_c3_callback(self, request, response):
        pocket_pos = self.table.pocketPositions()
        c3 = pocket_pos[3]
        eePose = Pose()
        eePosition = Point()
        eePosition.x = c3.x
        eePosition.y = c3.y
        eePosition.z = c3.z + 0.2
        eePose.position = eePosition
        eeOrientation = Quaternion()
        eeOrientation.w = np.cos(np.pi / 2)
        eeOrientation.x = np.sin(np.pi / 2)
        eePose.orientation = eeOrientation
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        self.logger.info('Move Done')
        return response

    async def move_c4_callback(self, request, response):
        pocket_pos = self.table.pocketPositions()
        c4 = pocket_pos[5]
        eePose = Pose()
        eePosition = Point()
        eePosition.x = c4.x
        eePosition.y = c4.y
        eePosition.z = c4.z + 0.2
        eePose.position = eePosition
        eeOrientation = Quaternion()
        eeOrientation.w = np.cos(np.pi / 2)
        eeOrientation.x = np.sin(np.pi / 2)
        eePose.orientation = eeOrientation
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        self.logger.info('Move Done')
        return response

    def setup_scene(self):
        tableWidth = 2.0
        tableLength = 2.4
        tableHeight = 0.1

        poseTable = Pose()
        poseTable.position.x = 0.0
        poseTable.position.y = 0.0
        poseTable.position.z = -tableHeight / 2 - 0.01

        cameraWidth = 2.0
        cameraLength = 0.2
        cameraHeight = 0.1

        cameraPoint = self.world.cameraPosition()
        poseCamera = Pose()
        poseCamera.position.x = cameraPoint.x
        poseCamera.position.y = cameraPoint.y
        poseCamera.position.z = cameraPoint.z

        self.mp_interface.ps.add_box(
            'table', (tableWidth, tableLength, tableHeight), poseTable
        )
        self.mp_interface.ps.add_box(
            'camera', (cameraWidth, cameraLength, cameraHeight), poseCamera
        )


def main():
    rclpy.init()
    n = ControlNode()
    rclpy.spin(n)
