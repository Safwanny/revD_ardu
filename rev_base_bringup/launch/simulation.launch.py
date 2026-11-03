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

import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _prepend_paths(existing, additions):
    paths = additions + existing.split(os.pathsep)
    return os.pathsep.join(dict.fromkeys(path for path in paths if path))


def _without_path_prefix(existing, blocked_prefixes):
    paths = existing.split(os.pathsep)
    return os.pathsep.join(
        path for path in paths
        if path and not any(
            path == prefix or path.startswith(prefix + os.sep)
            for prefix in blocked_prefixes
        )
    )


def _workspace_controller_overlay_prefixes():
    prefix = get_package_prefix('mecanum_drive_controller')
    if prefix.startswith('/opt/ros/'):
        return []
    return [prefix]


def _gazebo_environment():
    ros_prefix = get_package_prefix('gz_sim_vendor')
    blocked_controller_prefixes = _workspace_controller_overlay_prefixes()
    vendor_names = (
        'gz_sim_vendor',
        'gz_sensors_vendor',
        'gz_physics_vendor',
        'gz_gui_vendor',
        'gz_transport_vendor',
        'gz_rendering_vendor',
        'gz_plugin_vendor',
        'gz_fuel_tools_vendor',
        'gz_msgs_vendor',
        'gz_common_vendor',
        'gz_ogre_next_vendor',
        'gz_dartsim_vendor',
        'sdformat_vendor',
        'gz_math_vendor',
        'gz_utils_vendor',
        'gz_tools_vendor',
        'gz_cmake_vendor',
    )

    library_paths = [
        os.path.join(ros_prefix, 'opt', name, 'lib') for name in vendor_names
    ]
    library_paths.extend(
        [
            os.path.join(ros_prefix, 'lib', 'x86_64-linux-gnu'),
            os.path.join(ros_prefix, 'lib'),
        ]
    )
    executable_paths = [
        os.path.join(ros_prefix, 'opt', name, 'bin') for name in vendor_names
    ]
    config_paths = [
        os.path.join(ros_prefix, 'opt', name, 'share', 'gz')
        for name in vendor_names
    ]

    return [
        SetEnvironmentVariable(
            'AMENT_PREFIX_PATH',
            _without_path_prefix(
                os.environ.get('AMENT_PREFIX_PATH', ''),
                blocked_controller_prefixes,
            ),
        ),
        SetEnvironmentVariable(
            'CMAKE_PREFIX_PATH',
            _without_path_prefix(
                os.environ.get('CMAKE_PREFIX_PATH', ''),
                blocked_controller_prefixes,
            ),
        ),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            _prepend_paths(
                _without_path_prefix(
                    os.environ.get('LD_LIBRARY_PATH', ''),
                    blocked_controller_prefixes,
                ),
                library_paths,
            ),
        ),
        SetEnvironmentVariable(
            'PATH',
            _prepend_paths(os.environ.get('PATH', ''), executable_paths),
        ),
        SetEnvironmentVariable(
            'GZ_CONFIG_PATH',
            _prepend_paths(os.environ.get('GZ_CONFIG_PATH', ''), config_paths),
        ),
    ]


def _resolve_world(context, worlds_directory):
    requested = LaunchConfiguration('world').perform(context)
    requested_path = Path(requested).expanduser()
    if requested_path.is_absolute():
        resolved = requested_path
    else:
        filename = requested if requested.endswith('.sdf') else requested + '.sdf'
        resolved = Path(worlds_directory) / filename
    if not resolved.is_file():
        raise RuntimeError(
            f"Gazebo world '{requested}' resolved to missing file: {resolved}"
        )
    render_engine = LaunchConfiguration('render_engine').perform(context)
    if render_engine not in ('ogre', 'ogre2'):
        raise RuntimeError("render_engine must be 'ogre' or 'ogre2'")
    max_step_text = LaunchConfiguration(
        'physics_max_step_size').perform(context)
    try:
        max_step_size = float(max_step_text)
    except ValueError as error:
        raise RuntimeError(
            'physics_max_step_size must be a positive finite number'
        ) from error
    if not math.isfinite(max_step_size) or max_step_size <= 0.0:
        raise RuntimeError(
            'physics_max_step_size must be a positive finite number')

    tree = ET.parse(resolved)
    physics = tree.findall('./world/physics')
    if len(physics) != 1:
        raise RuntimeError(
            f'Expected one physics element in {resolved}, found '
            f'{len(physics)}')
    max_step = physics[0].find('max_step_size')
    if max_step is None:
        max_step = ET.SubElement(physics[0], 'max_step_size')
    max_step.text = max_step_text

    if render_engine != 'ogre2':
        sensors_plugins = [
            plugin for plugin in tree.findall('.//plugin')
            if plugin.attrib.get('name') == 'gz::sim::systems::Sensors'
        ]
        if len(sensors_plugins) != 1:
            raise RuntimeError(
                f'Expected one Gazebo Sensors plugin in {resolved}, found '
                f'{len(sensors_plugins)}'
            )
        engine = sensors_plugins[0].find('render_engine')
        if engine is None:
            engine = ET.SubElement(sensors_plugins[0], 'render_engine')
        engine.text = render_engine
    generated = Path(
        f'/tmp/mobile_base_world_{os.getpid()}_{render_engine}.sdf'
    )
    tree.write(generated, encoding='utf-8', xml_declaration=True)
    resolved = generated
    context.launch_configurations['resolved_world'] = str(resolved.resolve())
    return []


def generate_launch_description():
    description_share = FindPackageShare('rev_base_description')
    bringup_share = FindPackageShare('rev_base_bringup')

    model = PathJoinSubstitution(
        [description_share, 'urdf', 'mobile_base.urdf.xacro']
    )
    controllers = PathJoinSubstitution(
        [bringup_share, 'config', 'controllers.yaml']
    )
    ekf_config = PathJoinSubstitution([bringup_share, 'config', 'ekf.yaml'])
    tof_rows_config = PathJoinSubstitution(
        [FindPackageShare('rev_base_tools'), 'config', 'tof_rows.yaml']
    )
    rviz_config = PathJoinSubstitution(
        [description_share, 'rviz', 'mobile_base_sim.rviz']
    )
    gazebo_worlds = os.path.join(
        get_package_share_directory('rev_base_gazebo'), 'worlds'
    )
    world = LaunchConfiguration('resolved_world')
    # A fixed RNG seed draws the same IMU bias and sensor noise every run.
    gz_seed = ['--seed ', LaunchConfiguration('seed'), ' ']
    simulation_sdf = '/tmp/mobile_base_sim_' + str(os.getpid()) + '.sdf'
    sdf_generator = PathJoinSubstitution(
        [FindPackagePrefix('rev_base_bringup'), 'lib', 'rev_base_bringup',
         'generate_sim_sdf.py']
    )
    gz_launch = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                model,
                ' use_gazebo:=true controllers_file:=',
                controllers,
            ]
        ),
        value_type=str,
    )
    generate_sdf = ExecuteProcess(
        cmd=[
            sdf_generator,
            '--xacro',
            model,
            '--controllers',
            controllers,
            '--output',
            simulation_sdf,
        ],
        output='screen',
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        output='screen',
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file',
            simulation_sdf,
            '-name',
            'mobile_base',
            '-allow_renaming',
            'true',
            '-x',
            LaunchConfiguration('x'),
            '-y',
            LaunchConfiguration('y'),
            '-z',
            LaunchConfiguration('z'),
            '-Y',
            LaunchConfiguration('yaw'),
        ],
        output='screen',
    )
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            'controller_manager',
            '--switch-timeout',
            '20.0',
            '--service-call-timeout',
            '20.0',
        ],
        output='screen',
    )
    base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mobile_base_controller',
            '--controller-manager',
            'controller_manager',
            '--switch-timeout',
            '20.0',
            '--service-call-timeout',
            '20.0',
        ],
        condition=IfCondition(LaunchConfiguration('start_controller')),
        output='screen',
    )
    turret_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'turret_controller',
            '--controller-manager',
            'controller_manager',
            '--switch-timeout',
            '20.0',
            '--service-call-timeout',
            '20.0',
        ],
        output='screen',
    )
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            ekf_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )
    imu_bias = Node(
        package='rev_base_tools',
        executable='imu_bias',
        name='imu_bias',
        parameters=[
            {
                'enabled': ParameterValue(
                    LaunchConfiguration('imu_bias'), value_type=bool),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    def after_robot_spawn(event, context):
        del context
        if event.returncode == 0:
            return [joint_state_spawner]
        return [
            EmitEvent(event=Shutdown(reason='robot spawn failed')),
        ]

    def after_sdf_generation(event, context):
        del context
        if event.returncode == 0:
            return [spawn_robot]
        return [
            EmitEvent(event=Shutdown(reason='simulation SDF generation failed')),
        ]

    def after_joint_state_spawner(event, context):
        del context
        if event.returncode == 0:
            return [
                base_controller_spawner,
                turret_controller_spawner,
                rviz,
            ]
        return [
            EmitEvent(event=Shutdown(reason='joint-state controller failed')),
        ]

    return LaunchDescription(
        [
            *_gazebo_environment(),
            DeclareLaunchArgument(
                'world',
                default_value='navigation_basic',
                description=(
                    'Installed world name, with or without .sdf, or an '
                    'absolute SDF path.'
                ),
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use the Gazebo simulation clock for ROS nodes.',
            ),
            DeclareLaunchArgument('gui', default_value='true'),
            # The ToF ray markers are a visualisation aid, not part of
            # the sensor pipeline, so they can be turned off without
            # affecting the clouds the sensors publish.
            DeclareLaunchArgument('tof_markers', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='true'),
            # The ToF camera's colour-graded depth in its own window.
            DeclareLaunchArgument('camera_windows', default_value='true'),
            DeclareLaunchArgument(
                'render_engine',
                default_value='ogre2',
                description=(
                    "Gazebo sensor render engine: 'ogre2' normally or "
                    "'ogre' for Mesa software-rendered validation."
                ),
            ),
            DeclareLaunchArgument(
                'physics_max_step_size',
                default_value='0.001',
                description='Gazebo physics integration step in seconds.',
            ),
            DeclareLaunchArgument(
                'start_controller',
                default_value='true',
                description='Start the mecanum ros2_control controller.',
            ),
            DeclareLaunchArgument(
                'seed',
                default_value='1',
                description='Gazebo RNG seed: sets the IMU bias draw.',
            ),
            DeclareLaunchArgument(
                'imu_bias',
                default_value='true',
                description=(
                    'Remove the gyro bias learnt at standstill; false '
                    'passes /imu/data through unchanged.'),
            ),
            DeclareLaunchArgument('x', default_value='0.0'),
            DeclareLaunchArgument('y', default_value='0.0'),
            DeclareLaunchArgument('z', default_value='0.10'),
            DeclareLaunchArgument('yaw', default_value='0.0'),
            OpaqueFunction(
                function=_resolve_world,
                args=[gazebo_worlds],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={'gz_args': ['-r ', *gz_seed, world]}.items(),
                condition=IfCondition(LaunchConfiguration('gui')),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={'gz_args': ['-r -s ', *gz_seed, world]}.items(),
                condition=UnlessCondition(LaunchConfiguration('gui')),
            ),
            robot_state_publisher,
            imu_bias,
            ekf,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=generate_sdf,
                    on_exit=after_sdf_generation,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=after_robot_spawn,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=after_joint_state_spawner,
                )
            ),
            generate_sdf,
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    # Evaluation only: no robot node may subscribe.
                    '/ground_truth/odom@nav_msgs/msg/Odometry'
                    '[gz.msgs.Odometry',
                    # ToF camera: IR amplitude stand-in, depth, cloud, intrinsics.
                    '/tof_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/tof_camera/depth_image@sensor_msgs/msg/Image'
                    '[gz.msgs.Image',
                    '/tof_camera/points@sensor_msgs/msg/PointCloud2'
                    '[gz.msgs.PointCloudPacked',
                    '/tof_camera/camera_info@sensor_msgs/msg/CameraInfo'
                    '[gz.msgs.CameraInfo',
                    # Each ToF sensor scans an 8x8 grid, so Gazebo publishes
                    # an organised cloud on <topic>/points beside the
                    # flattened scan. The cloud is the one that keeps the
                    # grid, so it is the one bridged.
                    '/tof/front/points@sensor_msgs/msg/PointCloud2'
                    '[gz.msgs.PointCloudPacked',
                    '/tof/rear/points@sensor_msgs/msg/PointCloud2'
                    '[gz.msgs.PointCloudPacked',
                    '/tof/left/points@sensor_msgs/msg/PointCloud2'
                    '[gz.msgs.PointCloudPacked',
                    '/tof/right/points@sensor_msgs/msg/PointCloud2'
                    '[gz.msgs.PointCloudPacked',
                ],
                remappings=[('/tof_camera/points', '/tof_camera/points_raw')],
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration(
                            'use_sim_time'
                        )
                    }
                ],
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [bringup_share, 'launch', 'camera_view.launch.py'])),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('camera_windows')),
            ),
            # Gazebo's camera cloud is x-forward but stamped optical; this
            # restamps it, /tof_camera/points_raw -> /tof_camera/points.
            Node(
                package='rev_base_tools',
                executable='tof_camera_points',
                name='tof_camera_points',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration(
                            'use_sim_time'
                        )
                    }
                ],
                output='screen',
            ),
            # Cuts each face's 8 x 8 grid down to the rows in tof_rows.yaml,
            # /tof/<face>/points -> /tof/<face>/rows.
            Node(
                package='rev_base_tools',
                executable='tof_rows',
                name='tof_rows',
                parameters=[
                    tof_rows_config,
                    {
                        'use_sim_time': LaunchConfiguration(
                            'use_sim_time'
                        )
                    }
                ],
                output='screen',
            ),
            Node(
                package='rev_base_tools',
                executable='tof_ray_markers',
                name='tof_ray_markers',
                condition=IfCondition(
                    LaunchConfiguration('tof_markers')
                ),
                parameters=[
                    tof_rows_config,
                    {
                        'use_sim_time': LaunchConfiguration(
                            'use_sim_time'
                        )
                    }
                ],
                output='screen',
            ),
        ]
    )
