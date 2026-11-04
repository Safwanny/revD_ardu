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
Send goals to the ToF-only bug navigator and check it gets there untouched.

failsafe_course, mode:=tof_only, spawned at (SPAWN) facing +X. Goals, in
order: behind the long wall, an open staging point, through the U-trap to
behind it, then inside the closed box, which must come back unreachable.
Gazebo's true pose (tests may read it) measures the hull's clearance to
every box in the world, read back from the installed SDF.
"""

import math
import os
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

SPAWN = (-3.5, -2.0)
GOALS = (            # world frame, expected end state
    ('behind the long wall', (-0.8, -2.0), 'reached'),
    ('staging', (0.5, 0.0), 'reached'),
    ('through the U-trap', (4.2, 0.0), 'reached'),
    ('inside the closed box', (3.0, 2.6), 'unreachable'),
)
TOLERANCE = 0.2
GOAL_TIMEOUT = 240.0
HALF = (0.15, 0.11)


def boxes():
    """(cx, cy, half_x, half_y) of every box in failsafe_course.sdf."""
    path = os.path.join(get_package_share_directory('rev_base_gazebo'),
                        'worlds', 'failsafe_course.sdf')
    out = []
    for model in ET.parse(path).getroot().iter('model'):
        size = model.find('.//collision/geometry/box/size')
        if size is not None:
            x, y = map(float, model.find('pose').text.split()[:2])
            sx, sy = map(float, size.text.split()[:2])
            out.append((x, y, sx / 2, sy / 2))
    return out


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rev_base_bringup'), 'launch',
             'simulation.launch.py'])),
        launch_arguments={
            'world': 'failsafe_course',
            'mode': 'tof_only',
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


class TestBugNav(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('bug_nav_test')
        cls.pose = None
        cls.offset = None
        cls.state = None
        cls.boxes = boxes()
        cls.node.create_subscription(
            Odometry, '/ground_truth/odom', cls.on_truth, 10)
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(
            String, '/bug_nav/state', cls.on_state, latched)
        cls.goal = cls.node.create_publisher(PoseStamped, '/goal_pose', 10)

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

    @classmethod
    def on_state(cls, message):
        cls.state = message.data

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def clearance(self):
        """Smallest gap from the hull outline to any box; < 0 is a contact."""
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = HALF
        outline = [(t * hx, hy) for t in (-1, -.5, 0, .5, 1)] \
            + [(t * hx, -hy) for t in (-1, -.5, 0, .5, 1)] \
            + [(hx, t * hy) for t in (-.5, 0, .5)] \
            + [(-hx, t * hy) for t in (-.5, 0, .5)]
        gap = math.inf
        for px, py in ((x + c * u - s * v, y + s * u + c * v)
                       for u, v in outline):
            for bx, by, bhx, bhy in self.boxes:
                dx, dy = abs(px - bx) - bhx, abs(py - by) - bhy
                gap = min(gap, math.hypot(max(dx, 0), max(dy, 0))
                          if dx > 0 or dy > 0 else max(dx, dy))
        return gap

    def send(self, world_goal):
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.node.get_clock().now().to_msg()
        # odom starts at the spawn pose, facing +X.
        goal.pose.position.x = world_goal[0] - SPAWN[0]
        goal.pose.position.y = world_goal[1] - SPAWN[1]
        goal.pose.orientation.w = 1.0
        type(self).state = None  # the subscriber writes the class's
        self.goal.publish(goal)

    def test_bug_nav(self):
        deadline = time.monotonic() + 120.0
        while (self.pose is None or self.state is None) \
                and time.monotonic() < deadline:
            self.spin(0.2)
        self.assertIsNotNone(self.pose, 'no ground truth')
        self.assertEqual(self.state, 'idle', 'bug_nav not up')
        self.spin(10.0)  # controllers, EKF and the ToF clouds come up

        for name, target, expected in GOALS:
            self.send(target)
            start, closest = time.monotonic(), math.inf
            while time.monotonic() - start < GOAL_TIMEOUT:
                self.spin(0.1)
                closest = min(closest, self.clearance())
                if self.state in ('reached', 'unreachable'):
                    break
            took = time.monotonic() - start
            miss = math.dist(self.pose[:2], target)
            print(f'{name}: {self.state} in {took:.0f} s, {miss:.2f} m from '
                  f'the goal, closest {closest:.3f} m to an obstacle')
            self.assertEqual(self.state, expected, name)
            self.assertGreater(closest, 0.0, f'{name}: touched an obstacle')
            if expected == 'reached':
                self.assertLess(miss, TOLERANCE, name)
