from enum import auto, Enum

import rclpy
from rclpy.node import Node

from table import PoolTable

from motion_planning_interface.motion_planning_interface.MotionPlanningInterface import (
    MotionPlanningInterface,
)


class State(Enum):
    """Keep track of the robots current command."""

    SETUP = auto()
    RUNNING = auto()


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')

        timer_period = 1.0  # secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

<<<<<<< HEAD
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

=======
        # TODO need actual frame names
        self.table = PoolTable(self, ['c1, c2, c3, c4'], ['b1'])
        self.mp_interface = MotionPlanningInterface(self)
>>>>>>> a1792d5 (Added motion planning interface to control node)
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


def main():
    rclpy.init()
    n = ControlNode()
    rclpy.spin(n)
