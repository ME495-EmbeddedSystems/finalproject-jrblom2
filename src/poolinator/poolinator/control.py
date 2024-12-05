from enum import auto, Enum

import rclpy
from rclpy.node import Node

from table import PoolTable

from motion_planning_interface.motion_planning_interface.MotionPlanningInterface import (
    MotionPlanningInterface,
)

from std_srvs.srv import Empty


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

    def move_c1_callback(self, request, response):
        pass

def main():
    rclpy.init()
    n = ControlNode()
    rclpy.spin(n)
