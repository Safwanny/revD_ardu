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
Drive Rev D at a low box and off a table edge; the camera must stop it.

ledge_course: a 0.2 m high table with open edges, spawned at (SPAWN) facing
+X. Teleop above the speed cap: strafe +y at the 25 mm box (lower than the
ToF ring sees from a stopping distance), then forward at the east ledge.
Gazebo's true pose (tests may read it) measures the gaps. The robot must
stop short of both without touching the box or going over the edge.
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

SPAWN = (-1.0, 0.0)
LEDGE_X = 2.0           # table east edge, ledge_course.sdf
BOX_SOUTH_Y = 0.85      # low box (-1, 1), 0.3 m square
HALF = (0.15, 0.11)     # hull with wheels
COMMAND = 0.6
MAX_GAP = 0.08          # min_gap 3 cm plus the 2 cm hazard grid


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rev_base_bringup'), 'launch',
             'simulation.launch.py'])),
        launch_arguments={
            'world': 'ledge_course',
            'x': str(SPAWN[0]),
            'y': str(SPAWN[1]),
            'z': '0.3',
            'gui': 'false',
            'rviz': 'false',
            'camera_windows': 'false',
            'tof_markers': 'false',
            'render_engine': 'ogre2',
        }.items(),
    )
    return LaunchDescription([simulation, launch_testing.actions.ReadyToTest()])


class TestCameraFloor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('camera_floor_test')
        cls.pose = None
        cls.speed = 0.0
        cls.offset = None
        cls.node.create_subscription(
            Odometry, '/ground_truth/odom', cls.on_truth, 10)
        cls.teleop = cls.node.create_publisher(
            TwistStamped, '/cmd_vel_teleop', 10)

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
        cls.pose = (p.x + cls.offset[0], p.y + cls.offset[1])
        v = message.twist.twist.linear
        cls.speed = math.hypot(v.x, v.y)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def gaps(self):
        """Distances from the hull to the box's south face and the ledge."""
        x, y = self.pose
        return BOX_SOUTH_Y - (y + HALF[1]), LEDGE_X - (x + HALF[0])

    def drive(self, vx, vy, gap, seconds=60.0):
        """Teleop at 20 Hz until stopped; return the top speed, closest gap."""
        top, closest, still_since = 0.0, math.inf, None
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            command = TwistStamped()
            command.header.stamp = self.node.get_clock().now().to_msg()
            command.header.frame_id = 'base_link'
            command.twist.linear.x, command.twist.linear.y = vx, vy
            self.teleop.publish(command)
            self.spin(0.05)
            top = max(top, self.speed)
            closest = min(closest, self.gaps()[gap])
            if top > 0.1:
                still_since = (still_since or time.monotonic()) \
                    if self.speed < 0.005 else None
                if still_since and time.monotonic() - still_since > 2.0:
                    break
        self.spin(1.0)
        return top, closest

    def stopped_short(self, name, gap, top, closest):
        print(f'{name}: top speed {top:.3f} m/s, stopped {gap:.3f} m short, '
              f'closest {closest:.3f} m')
        self.assertGreater(closest, 0.0, f'{name}: went over')
        self.assertLess(gap, MAX_GAP, f'{name}: stopped far short')

    def test_camera_floor(self):
        deadline = time.monotonic() + 120.0
        while self.pose is None and time.monotonic() < deadline:
            self.spin(0.2)
        self.assertIsNotNone(self.pose, 'no ground truth')
        self.spin(10.0)  # controllers, EKF, camera and its hazard grid

        top, closest = self.drive(0.0, COMMAND, 0)
        self.stopped_short('low box', self.gaps()[0], top, closest)
        top, closest = self.drive(COMMAND, 0.0, 1)
        self.stopped_short('ledge', self.gaps()[1], top, closest)
        self.assertGreater(top, 0.3, 'ledge: crawled the whole way')
