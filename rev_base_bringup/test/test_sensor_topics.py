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

Runs in my_world. The checks are on frame ids, message shape and rates, not
world geometry, plus one actuation check: a turret command moves the pan
joint, and one isolation check: only evaluation reads the ground truth.
"""

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
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, JointState, PointCloud2
from std_msgs.msg import Float64MultiArray
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
            # Software 'ogre' returns wrong ranges; ogre2 needs a GPU.
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
            '/tof_camera/points': PointCloud2,
            '/tof_camera/depth_image': Image,
            '/mobile_base_controller/odometry': Odometry,
            '/ground_truth/odom': Odometry,
            '/odometry/filtered': Odometry,
            '/imu/data_unbiased': Imu,
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

        filtered = received['/odometry/filtered']
        self.assertEqual(filtered.header.frame_id, 'odom')
        self.assertEqual(filtered.child_frame_id, 'base_footprint')

        # The EKF owns odom -> base_footprint; the controller sends none.
        tf_writers = {
            info.node_name for info in
            self.node.get_publishers_info_by_topic('/tf')}
        self.assertIn('ekf_filter_node', tf_writers)
        self.assertNotIn('mobile_base_controller', tf_writers)

        truth = received['/ground_truth/odom']
        self.assertEqual(truth.header.frame_id, 'ground_truth')
        self.assertEqual(truth.child_frame_id, 'base_footprint')

        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)

    def test_imu_rate_and_frame(self):
        stamps = []

        def on_imu(message):
            self.assertEqual(message.header.frame_id, 'imu_link')
            stamps.append(Time.from_msg(message.header.stamp).nanoseconds)

        subscription = self.node.create_subscription(
            Imu, '/imu/data', on_imu, qos_profile_sensor_data)
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline and len(stamps) < 201:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(subscription)
        self.assertGreater(len(stamps), 200, 'IMU too slow or silent')
        # Sim-time rate, so a slow real-time factor does not fail it.
        rate = (len(stamps) - 1) / ((stamps[-1] - stamps[0]) * 1e-9)
        self.assertAlmostEqual(rate, 100.0, delta=5.0)

    def test_no_robot_node_reads_ground_truth(self):
        readers = [
            info.node_name for info in
            self.node.get_subscriptions_info_by_topic('/ground_truth/odom')
            if info.node_name != self.node.get_name()
        ]
        self.assertEqual(readers, [], 'robot nodes must not see truth')

    def test_turret_follows_command(self):
        target = 0.5
        pan = None

        def on_joints(message):
            nonlocal pan
            if 'turret_pan_joint' in message.name:
                pan = message.position[message.name.index('turret_pan_joint')]

        subscription = self.node.create_subscription(
            JointState, '/joint_states', on_joints, 10)
        publisher = self.node.create_publisher(
            Float64MultiArray, '/turret_controller/commands', 10)
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline and (
                pan is None or abs(pan - target) > 0.02):
            publisher.publish(Float64MultiArray(data=[target]))
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(subscription)
        self.node.destroy_publisher(publisher)
        self.assertIsNotNone(pan, 'turret_pan_joint not on /joint_states')
        self.assertAlmostEqual(pan, target, delta=0.02)
