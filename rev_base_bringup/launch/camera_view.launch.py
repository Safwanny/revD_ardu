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

"""Show the ToF camera's depth, colour graded by range, in its own window."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Colour the depth image and open one window on it.

    simulation.launch.py starts this itself; run it on its own to reattach
    to a stack that is already running. use_sim_time matters even though
    nothing is controlled: the images carry simulation stamps.
    """
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='True while any Gazebo stack is running.',
        ),
        # 32FC1 metres -> bgr8, near red to far blue, no return black.
        Node(
            package='rev_base_tools',
            executable='tof_depth_colour',
            name='tof_depth_colour',
            parameters=[use_sim_time],
            output='screen',
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='tof_depth_view',
            # Positional, so the window opens already on the topic.
            arguments=['/tof_camera/depth_colour'],
            parameters=[use_sim_time],
            output='screen',
        ),
    ])
