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

        # TODO need actual frame names
        self.table = PoolTable(self, ['c1, c2, c3, c4'], ['b1'])
        self.state = State.SETUP

    def timer_callback(self):
        # Stay in setup state until pool table frames exist from CV
        if self.state == State.SETUP:
            if self.table.tableExists():
                self.state = State.RUNNING
            return

        if self.state == State.RUNNING:
            pass


def main():
    rclpy.init()
    n = ControlNode()
    rclpy.spin(n)
