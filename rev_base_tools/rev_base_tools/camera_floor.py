#!/usr/bin/env python3
"""Find drops and low obstacles in the turret camera's depth.

The camera's height and tilt never change (it only pans), so every pixel has
a fixed expected floor: where its ray meets the floor plane. Each steady
depth frame (/tof_camera/depth_steady) is sorted by height above that floor:
floor within ``floor_tolerance``, a drop deeper than ``drop_depth``, an
obstacle from ``obstacle_min`` to ``obstacle_max``, anything higher an
overhang the hull passes under. A pixel with no return where floor was
expected within range is a drop too (a cliff into nothing).

Hazards are kept in odom on a ``cell`` grid: a drop at the expected floor
point (the floor that is missing), an obstacle where it was measured. A cell
stays until the camera sees floor there again, so a ledge the camera can no
longer see up close is still known; cells beyond ``memory_radius`` are
dropped. The whole grid goes out on /tof_camera/hazards (odom, one point per
cell) for every frame, and the distance from the lens to the nearest hazard
along the look direction within the hull's width on
/tof_camera/hazard_distance. Numbers: config/camera_floor.yaml.
"""

import math
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

FLOOR, DROP, OBSTACLE, OTHER = 0, 1, 2, 3


def rotation_matrix(q):
    """Return the 3 x 3 rotation of a geometry_msgs Quaternion."""
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rays(info):
    """Return H x W x 3 optical-frame rays (z = 1) from a CameraInfo."""
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    v, u = np.mgrid[0:info.height, 0:info.width].astype(float)
    return np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1)


def classify(depth, ray, rotation, origin, p):
    """Return (class per pixel, odom xyz per pixel) for one depth frame.

    ``depth`` H x W z-depth in metres, ``ray`` from ``rays``, ``rotation``
    and ``origin`` the optical frame in odom, whose z is height above the
    floor. Drops get the expected floor point, the rest the measured one.
    """
    direction = ray @ rotation.T
    with np.errstate(divide='ignore', invalid='ignore'):
        expected = np.where(direction[..., 2] < 0.0,
                            -origin[2] / direction[..., 2], np.inf)
        measured = origin + direction * depth[..., None]
    height = measured[..., 2]
    finite = np.isfinite(depth)
    valid = finite & (depth <= p.max_range)
    floor_ahead = expected <= p.max_range
    label = np.full(depth.shape, OTHER)
    label[valid & (np.abs(height) <= p.floor_tolerance)] = FLOOR
    label[valid & (height > p.obstacle_min) & (height <= p.obstacle_max)] \
        = OBSTACLE
    # Below the floor (however far the lower floor is), or no return where
    # the floor should have answered.
    drop = floor_ahead & ((finite & (height < -p.drop_depth))
                          | (~finite & (depth > 0.0)))
    label[drop] = DROP
    xyz = np.where((label == DROP)[..., None],
                   origin + direction * np.where(
                       floor_ahead, expected, 0.0)[..., None], measured)
    return label, xyz


def cell_keys(xy, cell):
    """Return one int64 key per row of ``xy`` for its ``cell`` grid square."""
    i = np.floor(np.asarray(xy) / cell).astype(np.int64)
    return (i[:, 0] << 32) + (i[:, 1] & 0xFFFFFFFF)


def cell_centres(keys, cell):
    """Return the N x 2 centres of the grid squares ``keys``."""
    ix = keys >> 32
    iy = keys & 0xFFFFFFFF
    iy = np.where(iy >= 2 ** 31, iy - 2 ** 32, iy)
    return (np.stack([ix, iy], axis=1) + 0.5) * cell


def remember(memory, label, xyz, cell):
    """Return the hazard keys after one frame: floor seen clears, hazards add.

    A square with both floor and hazard in it stays a hazard.
    """
    floor = cell_keys(xyz[label == FLOOR][:, :2], cell)
    hazard = cell_keys(xyz[(label == DROP) | (label == OBSTACLE)][:, :2], cell)
    return np.union1d(np.setdiff1d(memory, floor), hazard)


def distance_ahead(points, origin, heading, half_width):
    """Return how far along ``heading`` the nearest point within the width is.

    ``points`` N x 2, ``origin`` the lens (x, y); inf when none.
    """
    u = np.array([math.cos(heading), math.sin(heading)])
    offset = points - origin
    along = offset @ u
    across = np.abs(offset @ np.array([-u[1], u[0]]))
    ahead = along[(along > 0.0) & (across <= half_width)]
    return float(ahead.min()) if len(ahead) else math.inf


class CameraFloor(Node):
    """Classify steady depth frames into a remembered odom hazard grid."""

    def __init__(self):
        super().__init__('camera_floor')
        defaults = {
            'odom_frame': 'odom', 'floor_tolerance': 0.02, 'drop_depth': 0.03,
            'obstacle_min': 0.02, 'obstacle_max': 0.15, 'max_range': 2.0,
            'cell': 0.02, 'memory_radius': 3.0, 'half_width': 0.11}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = SimpleNamespace(**{name: self.get_parameter(name).value
                                    for name in defaults})
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.ray = None
        self.memory = np.empty(0, dtype=np.int64)
        self.hazards = self.create_publisher(
            PointCloud2, '/tof_camera/hazards', 5)
        self.distance = self.create_publisher(
            Float32, '/tof_camera/hazard_distance', 5)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            CameraInfo, '/tof_camera/camera_info', self.on_info, sensor)
        self.create_subscription(
            Image, '/tof_camera/depth_steady', self.on_depth, sensor)

    def on_info(self, message):
        if self.ray is None or self.ray.shape[:2] != (message.height,
                                                      message.width):
            self.ray = rays(message)

    def lookup(self, frame, stamp):
        try:
            return self.tf.lookup_transform(self.p.odom_frame, frame, stamp)
        except TransformException:
            return self.tf.lookup_transform(self.p.odom_frame, frame, Time())

    def on_depth(self, message):
        if self.ray is None or message.encoding != '32FC1':
            return
        try:
            t = self.lookup(message.header.frame_id,
                            Time.from_msg(message.header.stamp)).transform
        except TransformException as error:
            self.get_logger().warn(str(error), throttle_duration_sec=5.0)
            return
        depth = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(
            message.height, message.step // 4)[:, :message.width]
        rotation = rotation_matrix(t.rotation)
        o = t.translation
        origin = np.array([o.x, o.y, o.z])
        label, xyz = classify(depth, self.ray, rotation, origin, self.p)
        memory = remember(self.memory, label, xyz, self.p.cell)
        centres = cell_centres(memory, self.p.cell)
        near = np.hypot(*(centres - origin[:2]).T) <= self.p.memory_radius
        self.memory, centres = memory[near], centres[near]

        header = message.header
        header.frame_id = self.p.odom_frame
        self.hazards.publish(point_cloud2.create_cloud_xyz32(
            header, np.column_stack([centres, np.zeros(len(centres))])))
        # The optical +Z axis, flattened, is where the camera looks.
        heading = math.atan2(rotation[1, 2], rotation[0, 2])
        self.distance.publish(Float32(data=distance_ahead(
            centres, origin[:2], heading, self.p.half_width)))


def main(args=None):
    """Run the camera floor classifier."""
    rclpy.init(args=args)
    node = CameraFloor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
