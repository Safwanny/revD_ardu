#!/usr/bin/env python3
"""Colour the ToF camera's depth image for viewing.

/tof_camera/depth_image is 32FC1 metres, which image viewers show as a flat
grey. This maps it onto a colour scale over the camera's range (near red, far
blue) and publishes bgr8 on /tof_camera/depth_colour. Pixels with no return
(NaN, inf, out of range) are black, so a gap reads as a gap rather than as far.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def colourise(depth, min_range, max_range):
    """Return an HxWx3 uint8 BGR image of ``depth`` (metres)."""
    valid = np.isfinite(depth) & (depth >= min_range) & (depth <= max_range)
    scaled = np.zeros(depth.shape, dtype=np.uint8)
    # Near is 255 so JET puts it at red.
    scaled[valid] = np.round(
        255.0 * (max_range - depth[valid]) / (max_range - min_range))
    colour = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
    colour[~valid] = 0
    return colour


class ToFDepthColour(Node):
    """Republish the depth image coloured by range."""

    def __init__(self):
        super().__init__('tof_depth_colour')
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('max_range', 4.0)
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.publisher = self.create_publisher(
            Image, '/tof_camera/depth_colour', 5)
        self.create_subscription(
            Image, '/tof_camera/depth_image', self.on_depth,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_depth(self, message):
        if message.encoding != '32FC1':
            self.get_logger().warn(
                'expected 32FC1 depth, got ' + message.encoding,
                throttle_duration_sec=5.0)
            return
        depth = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(
            message.height, message.step // 4)[:, :message.width]
        colour = colourise(depth, self.min_range, self.max_range)
        out = Image()
        out.header = message.header
        out.height, out.width = colour.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = colour.tobytes()
        self.publisher.publish(out)


def main(args=None):
    """Run the depth colouriser."""
    rclpy.init(args=args)
    node = ToFDepthColour()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
