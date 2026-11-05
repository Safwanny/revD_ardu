#!/usr/bin/env python3
"""Colour the ToF camera's depth image for viewing, with the drive path.

/tof_camera/depth_image is 32FC1 metres, which image viewers show as a flat
grey. This maps it onto a colour scale over the camera's range (near red, far
blue) and publishes bgr8 on /tof_camera/depth_colour. Pixels with no return
(NaN, inf, out of range) are black, so a gap reads as a gap rather than as far.

While the robot moves, the path it is commanded along
(/mobile_base_controller/reference, after the safety governor) is drawn on
the floor like a car's parking overlay: the hull's outline every
``mark_spacing`` m out to ``path_length`` m, and the track of each corner.
It is projected through TF, so it stays on the floor as the turret pans.
"""

import math

import cv2
from geometry_msgs.msg import TwistStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rev_base_tools.camera_floor import rotation_matrix
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

# Corners of the hull with wheels, in order round it.
CORNERS = ((1, 1), (1, -1), (-1, -1), (-1, 1))


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


def path(vx, vy, wz, length, spacing, steps_per_mark=8):
    """Return the poses (x, y, yaw) every ``spacing`` m along a constant twist.

    Starts at the robot (0, 0, 0) and runs ``length`` m of travel; poses come
    ``steps_per_mark`` integration steps apart, so arcs are drawn smooth.
    """
    speed = math.hypot(vx, vy)
    dt = spacing / speed / steps_per_mark
    x = y = yaw = 0.0
    poses = [(x, y, yaw)]
    for _ in range(int(round(length / spacing)) * steps_per_mark):
        c, s = math.cos(yaw + 0.5 * wz * dt), math.sin(yaw + 0.5 * wz * dt)
        x += (c * vx - s * vy) * dt
        y += (s * vx + c * vy) * dt
        yaw += wz * dt
        poses.append((x, y, yaw))
    return poses


def corner_tracks(poses, half):
    """Return 4 x N x 3 floor points: each hull corner at each pose."""
    p = np.asarray(poses)
    tracks = []
    for sx, sy in CORNERS:
        cx, cy = sx * half[0], sy * half[1]
        c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
        tracks.append(np.stack([p[:, 0] + c * cx - s * cy,
                                p[:, 1] + s * cx + c * cy,
                                np.zeros(len(p))], axis=1))
    return np.stack(tracks)


def project(points, rotation, origin, k, near=0.05):
    """Return (N x 2 pixels, in front mask) for base-frame ``points``.

    ``rotation`` and ``origin`` place the optical frame in the base frame;
    ``k`` is CameraInfo.k. Points closer than ``near`` in front are masked.
    """
    optical = (points - origin) @ rotation
    front = optical[:, 2] > near
    z = np.where(front, optical[:, 2], 1.0)
    pixels = np.stack([k[0] * optical[:, 0] / z + k[2],
                       k[4] * optical[:, 1] / z + k[5]], axis=1)
    return pixels, front


def draw_path(image, tracks, rotation, origin, k, steps_per_mark=8,
              colour=(255, 255, 255)):
    """Draw corner tracks and a hull outline every mark onto ``image``."""
    n_corners, n_poses = tracks.shape[:2]
    pixels, front = project(tracks.reshape(-1, 3), rotation, origin, k)
    pixels = pixels.reshape(n_corners, n_poses, 2)
    front = front.reshape(n_corners, n_poses)
    rect = (0, 0, image.shape[1], image.shape[0])

    def line(a, b, thickness):
        ok, p, q = cv2.clipLine(rect, tuple(int(v) for v in a),
                                tuple(int(v) for v in b))
        if ok:
            cv2.line(image, p, q, colour, thickness, cv2.LINE_AA)

    for corner in range(n_corners):
        for i in range(n_poses - 1):
            if front[corner, i] and front[corner, i + 1]:
                line(pixels[corner, i], pixels[corner, i + 1], 1)
    for i in range(steps_per_mark, n_poses, steps_per_mark):
        for corner in range(n_corners):
            j = (corner + 1) % n_corners
            if front[corner, i] and front[j, i]:
                line(pixels[corner, i], pixels[j, i], 2)
    return image


class ToFDepthColour(Node):
    """Republish the depth image coloured by range, with the drive path."""

    def __init__(self):
        super().__init__('tof_depth_colour')
        defaults = {
            'min_range': 0.2, 'max_range': 4.0, 'base_frame': 'base_footprint',
            'half_length': 0.15, 'half_width': 0.11, 'path_length': 1.0,
            'mark_spacing': 0.25, 'moving_speed': 0.01, 'command_timeout': 0.5}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.min_range, self.max_range = p['min_range'], p['max_range']
        self.p = p
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.k = None
        self.command, self.command_time = (0.0, 0.0, 0.0), -math.inf
        self.publisher = self.create_publisher(
            Image, '/tof_camera/depth_colour', 5)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image, '/tof_camera/depth_image', self.on_depth, sensor)
        self.create_subscription(
            CameraInfo, '/tof_camera/camera_info', self.on_info, sensor)
        self.create_subscription(
            TwistStamped, '/mobile_base_controller/reference', self.on_command,
            10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_info(self, message):
        self.k = list(message.k)

    def on_command(self, message):
        t = message.twist
        self.command = (t.linear.x, t.linear.y, t.angular.z)
        self.command_time = self.now()

    def overlay(self, colour, header):
        vx, vy, wz = self.command
        if self.k is None \
                or self.now() - self.command_time > self.p['command_timeout'] \
                or math.hypot(vx, vy) < self.p['moving_speed']:
            return
        try:
            try:
                t = self.tf.lookup_transform(self.p['base_frame'],
                                             header.frame_id,
                                             Time.from_msg(header.stamp))
            except TransformException:
                t = self.tf.lookup_transform(self.p['base_frame'],
                                             header.frame_id, Time())
            t = t.transform
        except TransformException as error:
            self.get_logger().warn(str(error), throttle_duration_sec=5.0)
            return
        o = t.translation
        tracks = corner_tracks(
            path(vx, vy, wz, self.p['path_length'], self.p['mark_spacing']),
            (self.p['half_length'], self.p['half_width']))
        draw_path(colour, tracks, rotation_matrix(t.rotation),
                  np.array([o.x, o.y, o.z]), self.k)

    def on_depth(self, message):
        if message.encoding != '32FC1':
            self.get_logger().warn(
                'expected 32FC1 depth, got ' + message.encoding,
                throttle_duration_sec=5.0)
            return
        depth = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(
            message.height, message.step // 4)[:, :message.width]
        colour = colourise(depth, self.min_range, self.max_range)
        self.overlay(colour, message.header)
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
