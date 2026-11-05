from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rev_base_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Safwan',
    maintainer_email='safwan@example.com',
    description='ToF visualisation nodes and simulation cleanup for the Rev D base.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'tof_ray_markers = rev_base_tools.tof_ray_markers:main',
        'tof_rows = rev_base_tools.tof_rows:main',
        'tof_sim = rev_base_tools.tof_sim:main',
        'safety_governor = rev_base_tools.safety_governor:main',
        'bug_nav = rev_base_tools.bug_nav:main',
        'turret_gaze = rev_base_tools.turret_gaze:main',
        'tof_camera_points = rev_base_tools.tof_camera_points:main',
        'tof_depth_colour = rev_base_tools.tof_depth_colour:main',
        'stop_simulation = rev_base_tools.stop_simulation:main',
        'odom_eval = rev_base_tools.odom_eval:main',
        'odom_eval_plot = rev_base_tools.odom_eval_plot:main',
        'imu_bias = rev_base_tools.imu_bias:main',
    ]},
)
