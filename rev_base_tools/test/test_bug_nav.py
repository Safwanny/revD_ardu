import math
from types import SimpleNamespace

import numpy as np
from rev_base_tools.bug_nav import leave, next_axis, square_up

P = SimpleNamespace(hit_distance=0.25, wall_lost=0.5, margin=0.03)
HALF = (0.15, 0.11)
LEFT, RIGHT = 1, 3


def wall(x0, x1, y0, y1, n=60):
    """Points along the segment (x0, y0)-(x1, y1)."""
    return np.stack([np.linspace(x0, x1, n), np.linspace(y0, y1, n)], axis=1)


def test_leave():
    # Goal in the clear.
    assert leave(1.0, 1.2, 1.0, 0.1, 0.25)
    # Driving at it ends 0.25 short of 1.5 free: 2.0 - 1.25 <= 1.0 - 0.1.
    assert leave(2.0, 1.5, 1.0, 0.1, 0.25)
    assert not leave(2.0, 1.0, 1.0, 0.1, 0.25)


def test_slides_along_a_wall_in_front():
    front = wall(0.4, 0.4, -1.0, 1.0)
    assert next_axis(front, LEFT, -1, HALF, P) == LEFT


def test_turns_away_at_an_inner_corner():
    # Wall in front and another on the left: slide right.
    corner = np.concatenate([wall(0.4, 0.4, -1.0, 1.0),
                             wall(-1.0, 0.4, 0.35, 0.35)])
    assert next_axis(corner, LEFT, -1, HALF, P) == 2


def test_rounds_an_outer_corner():
    # The wall in front has ended behind the robot's right: go forward.
    ended = wall(0.4, 0.4, -1.0, -0.3)
    assert next_axis(ended, LEFT, -1, HALF, P) == 0
    # Right after rounding one it keeps going straight.
    assert next_axis(ended, LEFT, -1, HALF, P, toward=False) == LEFT


def test_boxed_in():
    box = np.concatenate([wall(-0.35, 0.35, y, y) for y in (-0.3, 0.3)]
                         + [wall(x, x, -0.3, 0.3) for x in (-0.35, 0.35)])
    assert next_axis(box, LEFT, -1, HALF, P) is None


def test_square_up():
    assert abs(square_up(wall(0.4, 0.4, -0.4, 0.4), 1.0)) < 1e-6
    # A wall that leans away 20 deg on the left: turn 20 deg right.
    t = math.radians(20)
    turned = wall(0.4 - 0.4 * math.sin(t), 0.4 + 0.4 * math.sin(t),
                  -0.4 * math.cos(t), 0.4 * math.cos(t))
    assert math.isclose(square_up(turned, 1.0), -t, abs_tol=1e-6)
    assert square_up(np.empty((0, 2)), 1.0) == 0.0
