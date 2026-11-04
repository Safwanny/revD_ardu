import math
from types import SimpleNamespace

import numpy as np
from rev_base_tools.safety_governor import (
    beside, govern, path_distance, stopping_speed, turn_angle)

P = SimpleNamespace(
    half_length=0.15, half_width=0.11, max_speed=0.45, max_angular=2.5,
    brake_decel=1.5, latency=0.18, min_gap=0.03, side_min_gap=0.01,
    creep_speed=0.03,
    side_gap=0.05, side_factor=0.6, slow_after=0.2, stale_after=0.75)
HALF = (0.15, 0.11)
FRESH = {face: 0.0 for face in ('front', 'rear', 'left', 'right')}
WALL_AHEAD = np.array([[0.40, y] for y in np.linspace(-0.3, 0.3, 13)])


def test_stopping_speed_stops_inside_the_distance():
    for distance in (0.05, 0.1, 0.2, 0.5):
        v = stopping_speed(distance, P)
        travel = v * P.latency + v * v / (2 * P.brake_decel)
        assert travel <= distance + 1e-9 or v == P.creep_speed
    assert stopping_speed(0.0, P) == 0.0
    assert stopping_speed(1e-4, P) == P.creep_speed
    assert stopping_speed(math.inf, P) == math.inf


def test_path_distance_by_direction():
    assert math.isclose(path_distance(WALL_AHEAD, (1, 0), HALF), 0.25)
    # Diagonal: the corner meets the wall after 0.25 / cos 45.
    d = path_distance(WALL_AHEAD, (math.sqrt(.5), math.sqrt(.5)), HALF)
    assert math.isclose(d, 0.25 * math.sqrt(2))
    # Moving away or along it never meets the wall.
    assert path_distance(WALL_AHEAD, (-1, 0), HALF) == math.inf
    assert path_distance(WALL_AHEAD, (0, 1), HALF) == math.inf


def test_points_inside_the_hull_are_ignored():
    assert path_distance(np.array([[0.1, 0.0]]), (1, 0), HALF) == math.inf


def test_beside_only_counts_the_sides():
    side_wall = np.array([[x, 0.14] for x in np.linspace(-0.3, 0.3, 13)])
    assert beside(side_wall, (1, 0), HALF, 0.05, 0.15)
    assert not beside(side_wall + (0, 0.1), (1, 0), HALF, 0.05, 0.15)
    assert not beside(WALL_AHEAD, (1, 0), HALF, 0.05, 0.15)


def test_turn_angle():
    assert turn_angle(np.array([[1.0, 0.0]]), HALF, 1.0) == math.inf
    # Just ahead of the front face's left end: turning right swings the
    # face into it at once, turning left swings it away.
    point = np.array([[0.16, 0.08]])
    left, right = turn_angle(point, HALF, 1.0), turn_angle(point, HALF, -1.0)
    assert right < math.radians(10) < left


def test_moving_away_is_never_limited():
    pressed = np.array([[0.16, y] for y in np.linspace(-0.1, 0.1, 9)])
    assert govern(0.3, 0.0, 0.0, pressed, FRESH, P)[0] == 0.0
    assert govern(-0.3, 0.0, 0.0, pressed, FRESH, P) == (-0.3, 0.0, 0.0)


def test_clearance_at_any_angle():
    # Sliding +y past a wall 2 cm off the side that closes in at 3 deg:
    # stopped where the side clearance would fall under side_min_gap.
    t = math.tan(math.radians(3))
    y = np.linspace(0.0, 2.0, 4001)
    slanted = np.stack([0.17 - y * t, y], axis=1)
    speed = govern(0.0, 0.6, 0.0, slanted, FRESH, P)[1]
    assert math.isclose(speed, stopping_speed(0.01 / t - 0.14, P), rel_tol=0.05)
    # A parallel wall 2 cm off the side: it slides on at the side cap.
    parallel = np.stack([np.full_like(y, 0.17), y - 1.0], axis=1)
    speed = govern(0.0, 0.6, 0.0, parallel, FRESH, P)[1]
    assert math.isclose(speed, P.side_factor * P.max_speed)
    # Facing a wall it stopped min_gap short of: no further.
    facing = np.stack([np.full_like(y, 0.175), y - 1.0], axis=1)
    assert govern(0.3, 0.0, 0.0, facing, FRESH, P)[0] == 0.0


def test_governed_speeds():
    vx, _, _ = govern(1.0, 0.0, 0.0, np.empty((0, 2)), FRESH, P)
    assert math.isclose(vx, P.max_speed)
    vx, _, _ = govern(1.0, 0.0, 0.0, WALL_AHEAD - (0.1, 0.0), FRESH, P)
    assert P.creep_speed < vx < P.max_speed
    side_wall = np.array([[x, 0.14] for x in np.linspace(-0.3, 0.3, 13)])
    vx, _, _ = govern(1.0, 0.0, 0.0, side_wall, FRESH, P)
    assert math.isclose(vx, P.side_factor * P.max_speed)


def test_stale_face_blocks_only_its_side():
    ages = dict(FRESH, front=1.0)
    empty = np.empty((0, 2))
    assert govern(0.3, 0.0, 0.0, empty, ages, P) == (0.0, 0.0, 0.0)
    assert govern(-0.3, 0.0, 0.0, empty, ages, P)[0] == -P.creep_speed
    assert govern(0.0, 0.0, 1.0, empty, ages, P)[2] == 0.0
    vx, _, _ = govern(0.3, 0.0, 0.0, empty, dict(FRESH, front=0.3), P)
    assert math.isclose(vx, P.creep_speed)
