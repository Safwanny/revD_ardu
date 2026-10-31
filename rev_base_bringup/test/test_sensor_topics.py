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
Launch Gazebo headlessly and verify the Rev D sensor topic contract.

Runs in my_world. The checks are on frame ids, message shape and finiteness,
not world geometry. The one world-dependent requirement is that the LiDAR
returns something finite: the robot spawns in the hall with walls 0.7 m away
against a 4.0 m range.
"""

import math
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

FACES = ('front', 'rear', 'left', 'right')


def generate_test_description():
    launch_file = (
        get_package_share_directory('rev_base_bringup')
        + '/launch/simulation.launch.py'
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'world': 'my_world',
            'gui': 'false',
            'rviz': 'false',
            'camera_windows': 'false',
            'use_sim_time': 'true',
            # Software 'ogre' returns range_min on every LiDAR ray here, so
            # the self-hit check below would test nothing; ogre2 needs a GPU.
            'render_engine': 'ogre2',
        }.items(),
    )
    return LaunchDescription(
        [
            simulation,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSensorTopics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('mobile_base_sensor_contract_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_sensor_and_odometry_messages(self):
        topics = {
            '/scan': LaserScan,
            '/tof_camera/points': PointCloud2,
            '/tof_camera/depth_image': Image,
            '/mobile_base_controller/odometry': Odometry,
        }
        for face in FACES:
            topics[f'/tof/{face}/points'] = PointCloud2
            topics[f'/tof/{face}/rows'] = PointCloud2
        received = dict.fromkeys(topics)
        odom_tf_received = False

        def tf_callback(message):
            nonlocal odom_tf_received
            if any(
                transform.header.frame_id == 'odom'
                and transform.child_frame_id == 'base_footprint'
                for transform in message.transforms
            ):
                odom_tf_received = True

        subscriptions = [
            self.node.create_subscription(
                kind,
                topic,
                lambda message, topic=topic: received.__setitem__(
                    topic, message),
                qos_profile_sensor_data,
            )
            for topic, kind in topics.items()
        ]
        subscriptions.append(
            self.node.create_subscription(TFMessage, '/tf', tf_callback, 10))
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline and (
                None in received.values() or not odom_tf_received):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        missing = [topic for topic, message in received.items()
                   if message is None]
        self.assertEqual(missing, [], 'no message received')
        self.assertTrue(odom_tf_received, 'no odom -> base_footprint on /tf')

        scan = received['/scan']
        self.assertEqual(scan.header.frame_id, 'lidar_link')
        self.assertGreater(scan.header.stamp.sec + scan.header.stamp.nanosec, 0)
        self.assertGreater(len(scan.ranges), 100)
        self.assertAlmostEqual(scan.range_max, 4.0, delta=0.01)
        finite_ranges = [value for value in scan.ranges if math.isfinite(value)]
        self.assertTrue(finite_ranges)
        self.assertTrue(all(scan.range_min <= value <= scan.range_max
                            for value in finite_ranges))
        # The scan plane clears the hull, wheels and ToF camera: nothing of
        # the robot itself may show up, so the closest return is a wall.
        self.assertGreater(min(finite_ranges), 0.3)

        for face in FACES:
            grid = received[f'/tof/{face}/points']
            self.assertEqual(grid.header.frame_id, f'tof_{face}_link')
            self.assertEqual(grid.width * grid.height, 64)
            rows = received[f'/tof/{face}/rows']
            self.assertEqual(rows.header.frame_id, f'tof_{face}_link')
            self.assertEqual((rows.height, rows.width), (3, 8))

        cloud = received['/tof_camera/points']
        self.assertEqual(cloud.header.frame_id, 'tof_camera_lens_link')
        depth = received['/tof_camera/depth_image']
        self.assertEqual(depth.encoding, '32FC1')
        self.assertEqual((depth.width, depth.height), (240, 136))

        odometry = received['/mobile_base_controller/odometry']
        self.assertEqual(odometry.header.frame_id, 'odom')
        self.assertEqual(odometry.child_frame_id, 'base_footprint')

        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)
