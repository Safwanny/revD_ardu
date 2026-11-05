import math

import numpy as np
from rev_base_tools.tof_depth_colour import (
    corner_tracks, draw_path, path, project)

HALF = (0.15, 0.11)
K = [120.0, 0, 120.0, 0, 120.0, 68.0, 0, 0, 1]
# Level camera 0.1 m up at the base centre looking +x (optical in base).
R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
ORIGIN = np.array([0.0, 0.0, 0.1])


def test_path_straight_and_arc():
    poses = path(0.3, 0.0, 0.0, 1.0, 0.25)
    assert np.allclose(poses[-1], (1.0, 0.0, 0.0))
    # A quarter turn at radius 1 m: travel pi/2 m ends at (1, 1).
    poses = path(0.5, 0.0, 0.5, math.pi / 2, math.pi / 8, steps_per_mark=50)
    assert np.allclose(poses[-1], (1.0, 1.0, math.pi / 2), atol=1e-3)
    # Strafing keeps the heading.
    assert np.allclose(path(0.0, 0.2, 0.0, 0.5, 0.25)[-1], (0.0, 0.5, 0.0))


def test_corner_tracks_follow_the_hull():
    tracks = corner_tracks([(0.0, 0.0, 0.0), (1.0, 0.0, math.pi / 2)], HALF)
    assert np.allclose(tracks[0, 0], (0.15, 0.11, 0.0))
    assert np.allclose(tracks[0, 1], (1.0 - 0.11, 0.15, 0.0))


def test_projection_straight_ahead():
    pixels, front = project(np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]),
                            R, ORIGIN, K)
    assert front.tolist() == [True, False]
    # On the centre column, 0.1 m below the axis at 1 m: 12 px under centre.
    assert np.allclose(pixels[0], (120.0, 80.0))


def test_forward_path_is_drawn_symmetric_below_the_horizon():
    image = np.zeros((136, 240, 3), np.uint8)
    tracks = corner_tracks(path(0.3, 0.0, 0.0, 1.0, 0.25), HALF)
    draw_path(image, tracks, R, ORIGIN, K)
    rows, cols = np.nonzero(image[..., 0])
    assert len(rows) and rows.min() > 68
    assert abs(cols.mean() - 120.0) < 2.0
    # Turning the camera away leaves nothing to draw.
    image[:] = 0
    behind = R @ np.diag([-1.0, 1.0, -1.0])
    draw_path(image, tracks, behind, ORIGIN, K)
    assert not image.any()
