#!/usr/bin/env python3
"""Point the turret camera where the robot is going.

While the robot moves (/cmd_vel_mux faster than ``deadband``) the turret
turns to the direction of travel, re-aimed only once that direction is
``hysteresis`` off the current aim. Past the pan limit (the wedge behind the
robot) it holds the limit on the side it is already on. Stopped for
``sweep_after`` it sweeps +-``sweep_angle`` to look around. The command
slews at ``pan_speed``, below the servo's 3.9 rad/s, so the sim turns like
the real turret instead of jumping.

Depth frames taken while the turret turns faster than ``steady_rate`` are
smeared on the real camera, so only the others are passed on, from
/tof_camera/depth_image to /tof_camera/depth_steady. Numbers:
config/turret_gaze.yaml.
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray


def aim(direction, target, limit, hysteresis):
    """Return the new pan target for travel along ``direction`` (rad).

    The target moves only once the travel direction is ``hysteresis`` off
    it. Beyond ``limit`` the turret holds the limit on the side of the
    current target, so straight back does not swing it from side to side.
    """
    if abs(direction) > limit:
        new = math.copysign(limit, target if target else direction)
    else:
        new = direction
    return new if abs(new - target) > hysteresis else target


def sweep(target, heading, angle, step):
    """Return (target, heading) one ``step`` further along a +-``angle`` sweep.

    ``heading`` is +1 or -1; it flips at either end.
    """
    target += heading * step
    if abs(target) >= angle:
        target = math.copysign(angle, target)
        heading = -math.copysign(1.0, target)
    return target, heading


def slew(current, target, step):
    """Return ``current`` moved toward ``target`` by at most ``step``."""
    return current + max(-step, min(step, target - current))


class TurretGaze(Node):
    """Aim the turret along /cmd_vel_mux; pass on only steady depth frames."""

    def __init__(self):
        super().__init__('turret_gaze')
        defaults = {
            'joint': 'turret_pan_joint', 'limit': math.radians(135.0),
            'rate': 50.0, 'deadband': 0.02, 'hysteresis': math.radians(10.0),
            'command_timeout': 0.5, 'pan_speed': 3.0, 'sweep_after': 1.0,
            'sweep_angle': math.radians(90.0), 'sweep_speed': 0.4,
            'steady_rate': 0.5}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.command = (0.0, 0.0)
        self.command_time = -math.inf
        self.stopped_since = None
        self.target = self.current = 0.0
        self.heading = 1.0
        self.pan_rate = math.inf  # unknown until /joint_states arrives
        self.output = self.create_publisher(
            Float64MultiArray, '/turret_controller/commands', 10)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.steady = self.create_publisher(
            Image, '/tof_camera/depth_steady', sensor)
        self.create_subscription(
            TwistStamped, '/cmd_vel_mux', self.on_command, 10)
        self.create_subscription(
            JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(
            Image, '/tof_camera/depth_image', self.on_depth, sensor)
        self.create_timer(1.0 / self.p['rate'], self.on_timer)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_command(self, message):
        self.command = (message.twist.linear.x, message.twist.linear.y)
        self.command_time = self.now()

    def on_joints(self, message):
        if self.p['joint'] in message.name:
            i = message.name.index(self.p['joint'])
            if i < len(message.velocity):
                self.pan_rate = abs(message.velocity[i])

    def on_depth(self, message):
        if self.pan_rate <= self.p['steady_rate']:
            self.steady.publish(message)

    def on_timer(self):
        p, now = self.p, self.now()
        vx, vy = self.command
        fresh = now - self.command_time <= p['command_timeout']
        dt = 1.0 / p['rate']
        if fresh and math.hypot(vx, vy) > p['deadband']:
            self.stopped_since = None
            self.target = aim(math.atan2(vy, vx), self.target, p['limit'],
                              p['hysteresis'])
        else:
            if self.stopped_since is None:
                self.stopped_since = now
                # Sweep on from where it looks, away from the nearer end.
                self.heading = -math.copysign(1.0, self.target or 1.0)
            if now - self.stopped_since >= p['sweep_after']:
                self.target, self.heading = sweep(
                    self.target, self.heading, p['sweep_angle'],
                    p['sweep_speed'] * dt)
        self.current = slew(self.current, self.target, p['pan_speed'] * dt)
        self.output.publish(Float64MultiArray(data=[self.current]))


def main(args=None):
    """Run the turret gaze."""
    rclpy.init(args=args)
    node = TurretGaze()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
