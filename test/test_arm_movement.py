#!/usr/bin/env python3
import rclpy
import rclpy.action
import unittest
from inspect import getmembers
from unittest import mock
import sys
import os.path
sys.path = [path for path in sys.path if 'install/dm_arm_control' not in path]
sys.path = [path for path in sys.path if 'ros2_ws/src/dm_arm_control' not in path]
# sys.path[0] = '/home/daniel/ros2_ws/install/dm_arm_control/lib/python3.8/site-packages'
sys.path.append('/home/daniel/ros2_ws/src/dm_arm_control')
import dm_arm_control
import pytest
NAME = 'arm_test'

from rclpy.action.server import ActionServer

# @mock.patch(rclpy.action.ActionServer, autospec=True)
# @mock.patch(rclpy.init)
# @mock.patch(rclpy.spin)
def test_arm_movement_feedback():
        rclpy.action.ActionServer = mock.Mock(ActionServer)
        arm_server = dm_arm_control.ArmMovementServer()
        # print(getmembers(dm_arm_control))
        print(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'dm_arm_control')))
        print(sys.path)
        assert(False)
        action_server_mock.assert_called_with('hello', 'hello', 'hello', 'hello')

# class ArmMovementTests(unittest.TestCase):

#     @mock.patch(rclpy.action.ActionServer, autospec=True)
#     @mock.patch(rclpy.init)
#     @mock.patch(rclpy.spin)
#     def test_arm_movement_feedback(action_server_mock, init_mock, spin_mock):
#         rclpy.action.ActionServer = mock.Mock(ActionServer)
#         arm_server = ArmMovementServer()
#         assert(False)
#         action_server_mock.assert_called_with('hello', 'hello', 'hello', 'hello')

# if __name__ == '__main__':
#     print("this is a test HELLO")
#     unittest.main()