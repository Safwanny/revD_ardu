import math
from types import SimpleNamespace

import numpy as np
from rev_base_tools.camera_floor import (
    cell_centres, cell_keys, classify, distance_ahead, DROP, FLOOR, OBSTACLE,
    OTHER, rays, remember)

P = SimpleNamespace(floor_tolerance=0.02, drop_depth=0.03, obstacle_min=0.02,
                    obstacle_max=0.15, max_range=2.0)
INFO = SimpleNamespace(k=[40.0, 0, 20.0, 0, 40.0, 10.0, 0, 0, 1],
                       width=40, height=20)
# Level camera 0.1 m up looking along odom +x: optical z -> x, x -> -y, y -> -z.
R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
ORIGIN = np.array([0.0, 0.0, 0.1])
RAY = rays(INFO)


def floor_depth(height=0.1):
    """z-depth where each ray meets a floor ``height`` below the lens."""
    down = RAY[..., 1]
    with np.errstate(divide='ignore'):
        return np.where(down > 0, height / down, np.inf).astype(np.float32)


def test_flat_floor_is_floor():
    label, xyz = classify(floor_depth(), RAY, R, ORIGIN, P)
    below = RAY[..., 1] > 0
    seen = below & (floor_depth() <= P.max_range)
    assert np.all(label[seen] == FLOOR)
    assert np.allclose(xyz[seen][:, 2], 0.0, atol=1e-6)
    assert np.all(label[~below] == OTHER)


def test_ledge_is_a_drop_at_the_missing_floor():
    depth = floor_depth()
    far = depth > 1.0
    depth[far] = floor_depth(0.3)[far]     # 0.2 m lower past x = 1 m
    label, xyz = classify(depth, RAY, R, ORIGIN, P)
    drop = far & (floor_depth() <= P.max_range)
    assert np.all(label[drop] == DROP)
    # Placed where the floor should have been, not on the lower floor.
    assert np.allclose(xyz[drop][:, 2], 0.0, atol=1e-6)
    assert xyz[drop][:, 0].min() > 1.0


def test_no_return_where_floor_expected_is_a_drop():
    depth = floor_depth()
    depth[-1, :] = np.inf
    label, _ = classify(depth, RAY, R, ORIGIN, P)
    assert np.all(label[-1, :] == DROP)


def test_low_box_is_an_obstacle_and_overhang_ignored():
    depth = floor_depth()
    # A box face at x = 0.5 m: pixels whose floor lies beyond it see the face.
    face = depth > 0.5
    depth[face] = 0.5
    label, xyz = classify(depth, RAY, R, ORIGIN, P)
    height = xyz[..., 2]
    assert np.all(label[face & (height > 0.02) & (height <= 0.15)] == OBSTACLE)
    assert not np.any(label[face & (height > 0.15)] == OBSTACLE)


def test_memory_clears_on_floor_and_keeps_unseen():
    cell = 0.02
    xyz = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    memory = remember(np.empty(0, np.int64), np.array([DROP, DROP]), xyz, cell)
    assert len(memory) == 2
    # Floor seen at 1 m clears it; 2 m was not looked at and stays.
    memory = remember(memory, np.array([FLOOR]), xyz[:1], cell)
    assert np.allclose(cell_centres(memory, cell), [[2.01, 0.01]])


def test_cell_keys_round_trip_negative():
    xy = np.array([[-1.005, -2.5], [0.3, -0.01]])
    centres = cell_centres(cell_keys(xy, 0.02), 0.02)
    assert np.all(np.abs(centres - xy) <= 0.01 + 1e-9)


def test_distance_ahead():
    points = np.array([[1.0, 0.05], [0.5, 0.3], [-0.5, 0.0]])
    assert math.isclose(distance_ahead(points, (0.0, 0.0), 0.0, 0.11), 1.0)
    assert distance_ahead(points, (0.0, 0.0), math.pi / 2, 0.11) == math.inf
