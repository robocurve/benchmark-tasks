"""Development check: can the left hand hold hanger B at the cup with the hump pinch tilted by various angles, either
roll? Usage: python dev/reach2.py"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import hotel_hanger as t  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import READY  # noqa: E402

ring_p = np.array([t.ROD_X, t.RING_B_Y, t.ROD_Z - t.RING_DROP])
ball = t.ball_target(ring_p, 0, 0.0, t.OVER_Z)
for tilt in (-30, -15, 0, 15, 30, 45):
    for yaw in (0, 25, 45):
        for roll in (1.0, -1.0):
            a = np.array([0.0, -math.cos(math.radians(tilt)), math.sin(math.radians(tilt))])
            r_h = t._rz(math.radians(yaw))
            r = r_h @ t._frame(a, (roll, 0.0, 0.0))
            p = ball + r_h @ (np.array([0.0, 0.0, t.GRIP_Z]) + (t.GRIP_IN - 0.013) * a)
            best = 1e9
            for q0 in (READY, [0.0, 1.5, 1.2, -0.3, 0.0, 0.0], [0.6, 1.2, 1.0, 0.0, 0.5, 0.0]):
                q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), np.array(q0, float), base_pos=station.BASES["left"],
                                     iters=80)
                best = min(best, pe * 1000 + math.degrees(re))
            print(f"tilt {tilt:4d} hanger yaw {yaw:3d} roll {roll:+.0f}: err {best:6.1f}  tcp {np.round(p, 3)}")
