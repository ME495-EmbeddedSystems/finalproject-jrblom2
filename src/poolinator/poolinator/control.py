from enum import auto, Enum

import rclpy
from rclpy.node import Node

from table import PoolTable


class State(Enum):
    """Keep track of the robots current command."""

    SETUP = auto()
    RUNNING = auto()


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')

        timer_period = 1.0  # secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

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


def main():
    rclpy.init()
    n = ControlNode()
    rclpy.spin(n)
