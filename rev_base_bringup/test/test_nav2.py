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
Send Nav2 several goals in one run and check it gets there untouched.

navigation_basic, spawned at the origin facing +X, so odom = world at the
start. Goals, in order: behind the interior wall, past the centre column to
the far corner, back to the start. Backing out is a penalised last resort: in all, under
BLIND_LIMIT of commanded travel may point beyond the turret camera's reach
(over 135 deg off the nose). Gazebo's true pose (tests may read it)
measures the hull's clearance to every box in the world, read back from
the installed SDF.
"""

import math
import os
import time
import unittest
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient

GOALS = (
    ('behind the interior wall', (3.0, 1.0)),
    ('past the column to the far corner', (-3.5, -2.5)),
    ('back to the start', (0.0, 0.0)),
)
# Nav2's goal checker is 0.15 m; odom drifts about 2 % of distance.
TOLERANCE = 0.3
GOAL_TIMEOUT = 180.0
HALF = (0.15, 0.11)
# turret_limit (properties.xacro): travel further off the nose is unseen.
CAMERA_REACH = math.radians(135)
BLIND_LIMIT = 0.1


def boxes():
    """(cx, cy, yaw, half_x, half_y) of every box in navigation_basic."""
    path = os.path.join(get_package_share_directory('rev_base_gazebo'),
                        'worlds', 'navigation_basic.sdf')
    out = []
    for model in ET.parse(path).getroot().iter('model'):
        size = model.find('.//collision/geometry/box/size')
        if size is not None:
            pose = [float(v) for v in model.find('pose').text.split()]
            sx, sy = map(float, size.text.split()[:2])
            out.append((pose[0], pose[1], pose[5], sx / 2, sy / 2))
    return out


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rev_base_bringup'), 'launch',
             'simulation.launch.py'])),
        launch_arguments={
            'world': 'navigation_basic',
            'gui': 'false',
            'rviz': 'false',
            'camera_windows': 'false',
            'tof_markers': 'false',
            'render_engine': 'ogre2',
        }.items(),
    )
    return LaunchDescription([simulation, launch_testing.actions.ReadyToTest()])


class TestNav2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('nav2_test')
        cls.pose = None
        cls.boxes = boxes()
        cls.node.create_subscription(
            Odometry, '/ground_truth/odom', cls.on_truth, 10)
        cls.nav = ActionClient(cls.node, NavigateToPose, 'navigate_to_pose')
        cls.blind = 0.0
        cls.last_command = None
        cls.node.create_subscription(
            TwistStamped, '/mobile_base_controller/reference',
            cls.on_command, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_truth(cls, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        cls.pose = (p.x, p.y, yaw)

    @classmethod
    def on_command(cls, message):
        """Add up the commanded travel that points where the camera can't."""
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        v = message.twist.linear
        if cls.last_command is not None \
                and abs(math.atan2(v.y, v.x)) > CAMERA_REACH:
            cls.blind += math.hypot(v.x, v.y) * min(
                stamp - cls.last_command, 0.5)
        cls.last_command = stamp

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
            for bx, by, byaw, bhx, bhy in self.boxes:
                # The point in the box's own frame.
                cb, sb = math.cos(byaw), math.sin(byaw)
                lx = cb * (px - bx) + sb * (py - by)
                ly = -sb * (px - bx) + cb * (py - by)
                dx, dy = abs(lx) - bhx, abs(ly) - bhy
                gap = min(gap, math.hypot(max(dx, 0), max(dy, 0))
                          if dx > 0 or dy > 0 else max(dx, dy))
        return gap

    def test_nav2(self):
        deadline = time.monotonic() + 120.0
        while self.pose is None and time.monotonic() < deadline:
            self.spin(0.2)
        self.assertIsNotNone(self.pose, 'no ground truth')
        self.assertTrue(self.nav.wait_for_server(timeout_sec=120.0),
                        'Nav2 not up')
        self.spin(10.0)  # controllers, EKF and the costmaps fill

        for name, target in GOALS:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'odom'
            goal.pose.pose.position.x, goal.pose.pose.position.y = target
            goal.pose.pose.orientation.w = 1.0
            sent = self.nav.send_goal_async(goal)
            while not sent.done():
                self.spin(0.1)
            handle = sent.result()
            self.assertTrue(handle.accepted, name)
            result = handle.get_result_async()
            start, closest = time.monotonic(), math.inf
            while not result.done() \
                    and time.monotonic() - start < GOAL_TIMEOUT:
                self.spin(0.1)
                closest = min(closest, self.clearance())
            took = time.monotonic() - start
            status = result.result().status if result.done() else None
            miss = math.dist(self.pose[:2], target)
            print(f'{name}: status {status} in {took:.0f} s, {miss:.2f} m '
                  f'from the goal, closest {closest:.3f} m to an obstacle')
            self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED, name)
            self.assertLess(miss, TOLERANCE, name)
            self.assertGreater(closest, 0.0, f'{name}: touched an obstacle')
        print(f'commanded travel beyond the camera: {self.blind:.3f} m')
        self.assertLess(self.blind, BLIND_LIMIT,
                        'backed where the camera cannot look')
