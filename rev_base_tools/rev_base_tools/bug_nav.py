#!/usr/bin/env python3
"""Drive to a goal with the four ToF faces only: DistBug, no map.

For when every other sensor is gone: a goal from RViz (2D Goal Pose, odom
frame) and the ToF ring are all it uses, and its commands go out on
/cmd_vel_nav through twist_mux and the safety governor (run it with
``tof_only``, so only motion along the sensing axes and clear spins pass).

DistBug (Kamon and Rivlin 1997):
- to_goal: spin to face the goal, drive at it. Something in the way within
  ``hit_distance`` makes the robot's position a hit point.
- follow: turn square to the obstacle (so its walls run along the
  sensing axes), then keep that heading and slide along it, axis by axis, on
  the side toward the goal, turning round corners by the hand rule.
  Leave when the free range F toward the goal, less the ``hit_distance``
  it stops short by, gives d(x, T) - F <= d_min - step, d_min being the closest it got to the goal
  since the hit, or when the goal itself is in the clear.
- Unreachable when it comes back to the hit point having turned a full turn
  round the obstacle, or hits where it hit before.
State on /bug_nav/state. Numbers: config/bug_nav.yaml.
"""

import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rev_base_tools.safety_governor import path_distance, spin_clear, \
    ToFMemory
from std_msgs.msg import String

# Base-frame axes, counter-clockwise: front, left, rear, right.
AXES = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))


def free(points, direction, half, margin):
    """How far the hull, grown by ``margin``, can move along ``direction``."""
    return path_distance(points, direction,
                         (half[0] + margin, half[1] + margin))


def leave(distance, free_range, closest, step, stop):
    """DistBug's leave condition.

    Leave if the goal is in the clear, or if driving at it gets closer than
    ``closest - step``. It stops ``stop`` short of whatever it meets, so
    that much of the free range does not count.
    """
    return free_range >= distance \
        or distance - (free_range - stop) <= closest - step


def square_up(points, reach):
    """Return the turn that puts the obstacle ahead square to the front.

    The wall direction is the principal axis of the points straight ahead
    within ``reach`` (a narrow strip, so a corner beside does not skew it);
    the turn is folded into +-45 deg. 0 with too few points.
    """
    ahead = points[(points[:, 0] > 0.0) & (points[:, 0] < reach)
                   & (np.abs(points[:, 1]) < 0.2)]
    if len(ahead) < 5:
        return 0.0
    _, vectors = np.linalg.eigh(np.cov(ahead.T))
    along = math.atan2(vectors[1, 1], vectors[0, 1])
    return (along + math.pi / 4.0) % (math.pi / 2.0) - math.pi / 4.0


def next_axis(points, travel, side, half, p, blocked=(), toward=True):
    """Pick the axis to slide along next, by the hand rule; None if boxed in.

    The obstacle is on axis ``travel + side``. In order: round the corner
    toward it once it is well clear (only if ``toward``), straight on, away
    from it, back.
    """
    for turn, needed in ((side, p.wall_lost), (0, p.hit_distance),
                         (-side, p.hit_distance), (2, p.hit_distance)):
        if turn == side and not toward:
            continue
        axis = (travel + turn) % 4
        if axis not in blocked and free(
                points, AXES[axis], half, p.margin) > needed:
            return axis
    return None


class BugNav(Node):
    """DistBug on the ToF ring: /goal_pose in, /cmd_vel_nav out."""

    def __init__(self):
        super().__init__('bug_nav')
        defaults = {
            'faces': ['front', 'rear', 'left', 'right'],
            'base_frame': 'base_footprint', 'odom_frame': 'odom',
            'half_length': 0.15, 'half_width': 0.11, 'min_height': 0.02,
            'max_height': 0.20, 'memory': 3.0, 'speed': 0.3,
            'spin_speed': 1.0, 'trim_angular': 0.2, 'heading_tolerance': 0.15,
            'hit_distance': 0.25, 'wall_lost': 0.5, 'margin': 0.03,
            'spin_margin': 0.02, 'step': 0.1, 'goal_tolerance': 0.1,
            'sensor_range': 1.8, 'view': 0.2, 'loop_radius': 0.4,
            'stall_time': 3.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = SimpleNamespace(**{name: self.get_parameter(name).value
                                    for name in defaults})
        self.half = (self.p.half_length, self.p.half_width)
        self.tof = ToFMemory(self, self.p)
        self.goal = None
        self.state = None
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_out = self.create_publisher(String, '/bug_nav/state',
                                               latched)
        self.command = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_timer(0.1, self.step)
        self.set_state('idle')

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.get_logger().info('state: ' + state)
            self.state_out.publish(String(data=state))

    def on_goal(self, message):
        if message.header.frame_id != self.p.odom_frame:
            self.get_logger().error('goal must be in {}, not {}'.format(
                self.p.odom_frame, message.header.frame_id))
            return
        self.goal = (message.pose.position.x, message.pose.position.y)
        self.hits = []
        self.progress = None
        self.set_state('to_goal')

    def drive(self, vx=0.0, vy=0.0, wz=0.0):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x, out.twist.linear.y = vx, vy
        out.twist.angular.z = wz
        self.command.publish(out)

    def stalled(self, here, now):
        """True if it has not moved 3 cm in ``stall_time``; then resets."""
        if self.progress is None or math.dist(self.progress[0], here) > 0.03:
            self.progress = (here, now)
        elif now - self.progress[1] > self.p.stall_time:
            self.progress = (here, now)
            return True
        return False

    def step(self):
        if self.state not in ('to_goal', 'follow'):
            return
        pose = self.tof.pose()
        if pose is None:
            return
        x, y, yaw = pose
        points = self.tof.points(pose)
        gx, gy = self.goal[0] - x, self.goal[1] - y
        distance = math.hypot(gx, gy)
        bearing = math.atan2(gy, gx) - yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        to_goal = (math.cos(bearing), math.sin(bearing))
        if distance < self.p.goal_tolerance:
            self.drive()
            self.set_state('reached')
            return
        stalled = self.stalled((x, y), self.tof.now())
        if self.state == 'to_goal':
            self.motion_to_goal(points, distance, bearing, to_goal, stalled,
                                (x, y), yaw)
        else:
            self.follow(points, distance, bearing, to_goal, stalled, (x, y),
                        yaw)

    def motion_to_goal(self, points, distance, bearing, to_goal, stalled,
                       here, yaw):
        if abs(bearing) > self.p.heading_tolerance:
            self.progress = None  # a spin is not a stall
            if spin_clear(points, self.half, self.p.spin_margin):
                self.drive(wz=math.copysign(
                    min(self.p.spin_speed, 2.0 * abs(bearing)), bearing))
            else:
                # Too close to something to spin: step out along the
                # clearest axis first.
                axis = max(range(4), key=lambda a: free(
                    points, AXES[a], self.half, self.p.margin))
                self.drive(*(self.p.speed * c for c in AXES[axis]))
            return
        ahead = free(points, AXES[0], self.half, self.p.margin)
        if stalled or ahead < min(self.p.hit_distance, distance):
            # DistBug's leave rule makes each hit at least ``step`` closer:
            # one this near an earlier hit is no progress.
            if any(math.dist(here, hit) < self.p.step for hit in self.hits):
                self.unreachable('hit where it hit before')
                return
            self.hits.append(here)
            self.closest = distance
            self.rounded = None
            self.turns = 0  # net quarter turns round the obstacle
            self.square = yaw + square_up(
                points, self.p.hit_distance + self.p.half_length + 0.3)
            # Slide on the side the goal is on; the obstacle is in front.
            self.travel = 1 if to_goal[1] >= 0.0 else 3
            self.side = -1 if self.travel == 1 else 1
            self.set_state('follow')
            self.get_logger().info('hit at ({:.2f}, {:.2f})'.format(*here))
            self.drive()
            return
        self.drive(vx=self.p.speed, wz=max(-self.p.trim_angular, min(
            self.p.trim_angular, bearing)))

    def follow(self, points, distance, bearing, to_goal, stalled, here,
               yaw):
        if self.square is not None:
            error = math.atan2(math.sin(self.square - yaw),
                               math.cos(self.square - yaw))
            if abs(error) > 0.03 and spin_clear(points, self.half,
                                                self.p.spin_margin):
                self.progress = None  # a spin is not a stall
                self.drive(wz=math.copysign(
                    min(self.p.spin_speed, 2.0 * abs(error)), error))
                return
            self.square = None
        self.closest = min(self.closest, distance)
        # Round once: a full turn of hand-rule turns, back at the hit point.
        if abs(self.turns) >= 4 and \
                math.dist(here, self.hits[-1]) < self.p.loop_radius:
            self.unreachable('went round the obstacle')
            return
        # F only where a face looks: off-axis bearings fall in the corner
        # gaps, where no range means nothing, not open floor.
        off_axis = abs(bearing) % (math.pi / 2.0)
        if min(off_axis, math.pi / 2.0 - off_axis) < self.p.view:
            free_range = min(self.p.sensor_range, free(
                points, to_goal, self.half, self.p.margin))
            if leave(distance, free_range, self.closest, self.p.step,
                     self.p.hit_distance):
                self.progress = None
                self.set_state('to_goal')
                return
        # Just round a corner, the obstacle is not beside it yet: go one
        # hull length before rounding again, or it turns in circles.
        toward = self.rounded is None or math.dist(here, self.rounded) \
            > 2.0 * self.p.half_length + self.p.margin
        axis = next_axis(points, self.travel, self.side, self.half, self.p,
                         blocked=(self.travel,) if stalled else (),
                         toward=toward)
        if axis is not None:
            turn = (axis - self.travel) % 4
            if turn == self.side % 4:
                self.rounded = here
                self.turns += 1
            elif turn == -self.side % 4:
                self.turns -= 1
            elif turn == 2:
                self.turns -= 2
        if axis is None:
            self.unreachable('boxed in')
            return
        if axis != self.travel:
            self.get_logger().info('follow: along the {} axis{}'.format(
                ('front', 'left', 'rear', 'right')[axis],
                ' (stalled)' if stalled else ''))
        self.travel = axis
        self.drive(*(self.p.speed * c for c in AXES[axis]))

    def unreachable(self, why):
        self.get_logger().warn('goal unreachable: ' + why)
        self.drive()
        self.set_state('unreachable')


def main(args=None):
    """Run the ToF-only bug navigator."""
    rclpy.init(args=args)
    node = BugNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
