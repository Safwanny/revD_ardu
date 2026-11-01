#!/usr/bin/env python3

# Copyright 2026 Safwan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Import the Rev_D_T4M_ardu ROS 2 meshes at URDF size.

The source is Rev D v4 with the type_04M_Ardu pan turret.

The CAD export ros2/meshes/ holds one STL per link, metres, REP-103:
  base_link.stl    hull, deck, motors, wheel supports, turret pod; ground frame
  wheel_xx.stl     wheel, rollers, axle, hub hardware; wheel frame
  turret_link.stl  pan gear, platform, cradle, camera envelope; pan-axis frame

Each is ~100k triangles, so parts are simplified on their own scale. The
export puts each wheel frame at the bounding-box centre of wheel + hub, 3.5 mm
inboard of the roller centre, so the wheels are re-centred on the rollers:
that is the contact line the kinematics use. The CAD folder is only read.

  import_rev_d_meshes.py <Rev_D_T4M_ardu dir> <output dir>

Fails loudly if the wheels or the turret are not where the READMEs say.
"""

from pathlib import Path
import sys

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

WHEEL_Y = 0.09325        # track 186.5 / 2, roller centre
WHEEL_R = 0.02996        # roller envelope 59.92 / 2
WHEEL_W = 0.0315         # over the side plates
TURRET_TOP = 0.065       # head top above the deck (type_04M_Ardu README)
TOL = 0.3e-3
# Mesh name -> (output name, side: +1 left wheel, -1 right, 0 not a wheel).
MESHES = {
    'base_link': ('body', 0),
    'turret_link': ('turret', 0),
    'wheel_fl': ('wheel_fl', 1),
    'wheel_fr': ('wheel_fr', -1),
    'wheel_rl': ('wheel_rl', 1),
    'wheel_rr': ('wheel_rr', -1),
}


def load_stl(path):
    data = Path(path).read_bytes()
    n = int(np.frombuffer(data[80:84], '<u4')[0])
    rec = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return np.frombuffer(data[84:84 + 50 * n], rec)['v'].astype(np.float64)


def save_stl(path, tris):
    tris = tris.astype('<f4')
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), np.dtype(
        [('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    rec['n'], rec['v'] = n, tris
    with open(path, 'wb') as f:
        f.write(b'Mecanum_01 Rev D T4M import'.ljust(80, b' '))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())


def components(tris):
    q = np.round(tris.reshape(-1, 3) * 1e6).astype(np.int64)
    _, vid = np.unique(q, axis=0, return_inverse=True)
    t = len(tris)
    rows = np.repeat(np.arange(t), 3)
    cols = t + vid.ravel()
    size = t + vid.max() + 1
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(size, size))
    _, label = connected_components(graph, directed=False)
    label = label[:t]
    return [tris[label == k] for k in np.unique(label)]


def simplify(tris, cell):
    """Vertex clustering: merge vertices sharing a grid cell, drop slivers."""
    pts = tris.reshape(-1, 3)
    key = np.floor(pts / cell).astype(np.int64)
    _, cid = np.unique(key, axis=0, return_inverse=True)
    cid = cid.ravel()
    mean = np.zeros((cid.max() + 1, 3))
    np.add.at(mean, cid, pts)
    mean /= np.bincount(cid)[:, None]
    f = cid.reshape(-1, 3)
    keep = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = f[keep]
    _, first = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
    return mean[f[np.sort(first)]]


def save_simplified(path, parts, offset):
    """Simplify each part on its own scale, so small screws keep their shape."""
    out = []
    for part in parts:
        pts = part.reshape(-1, 3)
        size = (pts.max(0) - pts.min(0)).max()
        out.append(simplify(part, float(np.clip(size / 15, 0.3e-3, 1.5e-3))))
    mesh = np.concatenate(out) - offset
    save_stl(path, mesh)
    return len(mesh)


def main(cad_dir, out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for name, (dest, side) in MESHES.items():
        tris = load_stl(Path(cad_dir) / 'ros2' / 'meshes' / f'{name}.stl')
        offset = np.zeros(3)
        if side:
            pts = tris.reshape(-1, 3)
            # Rollers and side plates, clear of the hub and its bolts.
            rim = pts[np.hypot(pts[:, 0], pts[:, 2]) > 0.025]
            offset[1] = (rim[:, 1].min() + rim[:, 1].max()) / 2
            width = rim[:, 1].max() - rim[:, 1].min()
            assert abs(width - WHEEL_W) < TOL, f'{name} width {width}'
            assert abs(np.hypot(pts[:, 0], pts[:, 2]).max() - WHEEL_R) < TOL
        if name == 'turret_link':
            assert abs(tris[..., 2].max() - TURRET_TOP) < TOL
        n = save_simplified(out / f'{dest}.stl', components(tris), offset)
        print(f'{dest}: {len(tris)} -> {n} tris, frame shift '
              f'{np.round(offset * 1e3, 2)} mm')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
