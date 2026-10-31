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

"""Near is red, far is blue, no return is black."""

from rev_base_tools.tof_depth_colour import colourise
import numpy as np


def test_near_red_far_blue_gaps_black():
    depth = np.array([[0.2, 4.0, np.nan, np.inf, 0.1, 9.0]], dtype=np.float32)
    blue, green, red = colourise(depth, 0.2, 4.0)[0].T.astype(int)
    assert red[0] > blue[0]   # nearest: red end of JET
    assert blue[1] > red[1]   # farthest: blue end of JET
    for gap in (2, 3, 4, 5):  # NaN, inf, inside min, past max
        assert (blue[gap], green[gap], red[gap]) == (0, 0, 0)
