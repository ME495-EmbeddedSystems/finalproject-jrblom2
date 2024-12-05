from enum import auto, Enum

import rclpy
from rclpy.node import Node

from table import PoolTable

from motion_planning_interface.motion_planning_interface.MotionPlanningInterface import (
    MotionPlanningInterface,
)

from std_srvs.srv import Empty

from geometry_msgs.msg import Point, Pose


class State(Enum):
    """Keep track of the robots current command."""

    SETUP = auto()
    RUNNING = auto()


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')

        timer_period = 1.0  # secs
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.move_c1 = self.create_service(Empty, 'move_c1', self.move_c1_callback)
        self.move_c2 = self.create_service(Empty, 'move_c2', self.move_c2_callback)
        self.move_c3 = self.create_service(Empty, 'move_c3', self.move_c3_callback)
        self.move_c4 = self.create_service(Empty, 'move_c4', self.move_c4_callback)
        self.mp_interface = MotionPlanningInterface(self)
        self.table = PoolTable(
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
        if self.state == State.SETUP:
            if self.table.tableExists():
                # Add planning scene objects here before running
                self.state = State.RUNNING
            return

        if self.state == State.RUNNING:
            self.get_logger().info(f"{self.table.pocketPositions()}")

    async def move_c1_callback(self, request, response):
        pocket_pos = self.table.pocketPositions()
        c1 = pocket_pos[0]
        eePose = Pose()
        eePosition = Point()
        eePosition.x = c1.x
        eePosition.y = c1.y
        eePosition.z = c1.z + .2
        eePose.position = eePosition
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
        eePosition.z = c2.z + .2
        eePose.position = eePosition
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
        eePosition.z = c3.z + .2
        eePose.position = eePosition
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
        eePosition.z = c4.z + .2
        eePose.position = eePosition
        resultFuture = await self.mp_interface.mp.pathPlanPose(eePose)
        await resultFuture
        self.logger.info('Move Done')
        return response


def main():
    rclpy.init()
    n = ControlNode()
    rclpy.spin(n)
