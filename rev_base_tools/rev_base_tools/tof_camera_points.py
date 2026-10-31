#!/usr/bin/env python3
"""Restamp the simulated ToF camera cloud into the frame it is really in.

Gazebo's rgbd_camera fills /points x-forward from the lens but stamps it with
the optical frame (z-forward), so RViz draws it turned on its side. Moving the
stamp to tof_camera_lens_link, which has the x-forward axes at the lens, fixes
the picture without touching the data. Sim only: the real camera's driver
stamps its own cloud.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class ToFCameraPoints(Node):
    """Relay /tof_camera/points_raw to /tof_camera/points, restamped."""

    def __init__(self):
        super().__init__('tof_camera_points')
        self.declare_parameter('frame_id', 'tof_camera_lens_link')
        self.frame_id = self.get_parameter('frame_id').value
        # Reliable out, so both best-effort and reliable readers match.
        self.publisher = self.create_publisher(
            PointCloud2, '/tof_camera/points', 5)
        self.create_subscription(
            PointCloud2, '/tof_camera/points_raw', self.on_cloud,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_cloud(self, message):
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)


def main(args=None):
    """Run the ToF camera cloud relay."""
    rclpy.init(args=args)
    node = ToFCameraPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
