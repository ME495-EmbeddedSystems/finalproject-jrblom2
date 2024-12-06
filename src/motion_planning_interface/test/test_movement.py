import time
import unittest

import launch
import launch.actions
from launch.actions import TimerAction

from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

from geometry_msgs.msg import Pose

from launch_testing.actions import ReadyToTest
import launch_testing.markers
import launch_testing.asserts

from motion_planning_interface.MotionPlanningInterface import (
    MotionPlanningInterface,
)

from threading import Event
from threading import Thread

import rclpy

import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                [
                    FindPackageShare('franka_fer_moveit_config'),
                    '/launch/demo.launch.py',
                ],
                launch_arguments={'use_rviz': 'False'}.items(),
            ),
            TimerAction(period=5.0, actions=[ReadyToTest()]),
        ]
    )


class TestMoveItInterface(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_node')
        self.spinning = Event()
        self.mpInterface = MotionPlanningInterface(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_path_plan(self):
        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.1
        pose.position.z = 0.7
        executor = rclpy.get_global_executor()
        future = executor.create_task(self.mpInterface.mp.pathPlanPose, pose)
        rclpy.spin_until_future_complete(self.node, future)
        future = future.result()
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # an error code of 1 means the motion plan was successful
        assert response.result.error_code.val == 1

        # Sleeping in tests is bad, but MoveIt likes a little time to reset
        time.sleep(1)

    def test_path_plan_cartesian(self):
        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.1
        pose.position.z = 0.8
        executor = rclpy.get_global_executor()
        future = executor.create_task(self.mpInterface.mp.cartesianPath, pose)
        rclpy.spin_until_future_complete(self.node, future)
        future = future.result()
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        assert response.result.error_code.val == 1

        time.sleep(1)

    def test_path_plan_blocked(self):
        # Pose is inside the table, should fail
        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.1
        pose.position.z = -0.1

        poseTable = Pose()
        poseTable.position.x = 0.0
        poseTable.position.y = 0.0
        poseTable.position.z = -0.1 / 2 - 0.01

        self.mpInterface.ps.add_box('table', (2.0, 2.0, 0.1), poseTable)

        executor = rclpy.get_global_executor()
        future = executor.create_task(self.mpInterface.mp.pathPlanPose, pose)
        rclpy.spin_until_future_complete(self.node, future)
        future = future.result()
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        print(response.result.error_code)
        assert response.result.error_code.val != 1

        time.sleep(1)
