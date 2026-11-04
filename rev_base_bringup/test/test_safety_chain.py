#!/usr/bin/env python3
# Copyright 2026 Safwan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Drive Rev D at the walls through the safety chain and check it stops short.

navigation_basic, spawned at (SPAWN) facing +X, in the open corner between
the interior wall and the north-east walls. Teleop commands go in on
/cmd_vel_teleop at more than the speed cap; Gazebo's true pose (tests may
read it) measures the gap between the hull and the wall. In order: the e-stop
lock holds the robot, then forward, sideways and diagonal runs, each followed
by backing away, which the keep-out zone must allow.
"""

import math
import time
import unittest

from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
from nav_msgs.msg import Odometry
import rclpy
from std_msgs.msg import Bool

SPAWN = (3.0, 1.0)
EAST_WALL_X = 5.0 - 0.075   # inner faces, navigation_basic.sdf
NORTH_WALL_Y = 4.0 - 0.075
HULL = ((0.15, 0.11), (0.15, -0.11), (-0.15, -0.11), (-0.15, 0.11))
COMMAND = 0.5               # above the 0.30 m/s cap
SPEED_CAP = 0.30
KEEP_OUT = 0.15             # config/safety.yaml keep_out minus the hull


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rev_base_bringup'), 'launch',
             'simulation.launch.py'])),
        launch_arguments={
            'world': 'navigation_basic',
            'x': str(SPAWN[0]),
            'y': str(SPAWN[1]),
            'gui': 'false',
            'rviz': 'false',
            'camera_windows': 'false',
            'tof_markers': 'false',
            'render_engine': 'ogre2',
        }.items(),
    )
    return LaunchDescription([simulation, launch_testing.actions.ReadyToTest()])


class TestSafetyChain(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('safety_chain_test')
        cls.pose = None
        cls.speed = 0.0
        cls.offset = None
        cls.node.create_subscription(
            Odometry, '/ground_truth/odom', cls.on_truth, 10)
        cls.teleop = cls.node.create_publisher(
            TwistStamped, '/cmd_vel_teleop', 10)
        cls.e_stop = cls.node.create_publisher(Bool, '/e_stop', 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_truth(cls, message):
        p = message.pose.pose.position
        if cls.offset is None:
            # World frame if it starts at the spawn point, else spawn-relative.
            near = math.dist((p.x, p.y), SPAWN) < 0.2
            cls.offset = (0.0, 0.0) if near else SPAWN
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        cls.pose = (p.x + cls.offset[0], p.y + cls.offset[1], yaw)
        v = message.twist.twist.linear
        cls.speed = math.hypot(v.x, v.y)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def hull(self):
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * hx - s * hy, y + s * hx + c * hy) for hx, hy in HULL]

    def gap(self):
        """Smallest distance from the hull to the east or north wall."""
        corners = self.hull()
        return min(EAST_WALL_X - max(cx for cx, _ in corners),
                   NORTH_WALL_Y - max(cy for _, cy in corners))

    def drive(self, vx, vy, seconds, until_stopped=False):
        """Publish teleop at 20 Hz; return the top speed seen."""
        top, still_since = 0.0, None
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            command = TwistStamped()
            command.header.stamp = self.node.get_clock().now().to_msg()
            command.header.frame_id = 'base_link'
            command.twist.linear.x, command.twist.linear.y = vx, vy
            self.teleop.publish(command)
            self.spin(0.05)
            top = max(top, self.speed)
            if until_stopped and top > 0.1:
                still_since = (still_since or time.monotonic()) \
                    if self.speed < 0.005 else None
                if still_since and time.monotonic() - still_since > 2.0:
                    break
        self.spin(1.0)  # let the 0.5 s reference timeout stop the wheels
        return top

    def run_at_wall(self, vx, vy, name):
        top = self.drive(vx, vy, 60.0, until_stopped=True)
        gap = self.gap()
        print(f'{name}: top speed {top:.3f} m/s, stopped {gap:.3f} m '
              'from the wall')
        self.assertLessEqual(top, SPEED_CAP + 0.02, name)
        self.assertGreater(gap, 0.05, f'{name}: too close to the wall')
        self.assertLess(gap, KEEP_OUT + 0.05, f'{name}: stopped far short')
        # The keep-out zone must let it drive away again.
        before = self.gap()
        self.drive(-vx, -vy, 2.0)
        self.assertGreater(self.gap() - before, 0.2, f'{name}: stuck')

    def test_safety_chain(self):
        deadline = time.monotonic() + 120.0
        while self.pose is None and time.monotonic() < deadline:
            self.spin(0.2)
        self.assertIsNotNone(self.pose, 'no ground truth')
        self.spin(10.0)  # controllers, EKF and collision_monitor come up

        self.e_stop.publish(Bool(data=True))
        self.spin(0.5)
        start = self.pose
        self.drive(COMMAND, 0.0, 3.0)
        self.assertLess(math.dist(start[:2], self.pose[:2]), 0.02,
                        'moved under e-stop')
        self.e_stop.publish(Bool(data=False))
        self.spin(0.5)

        self.run_at_wall(COMMAND, 0.0, 'forward')
        self.run_at_wall(0.0, COMMAND, 'sideways')
        diagonal = COMMAND / math.sqrt(2.0)
        self.run_at_wall(diagonal, diagonal, 'diagonal')
