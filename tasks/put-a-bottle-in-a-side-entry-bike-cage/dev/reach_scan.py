"""Offline reach scan: where can the bike stand so the right hand reaches the seated bottle's cap with the gripper level
(fingertips along +x, fingers closing square to the bottle in the frame plane)? Created 11_16_26 (America/New_York)."""
import itertools
import sys

import numpy as np
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, ".")
from bike_cage import geom as G  # noqa: E402

TOP = TABLE_TOP["station"]


def frame_rot():
    """TCP rotation: z (fingertips) along +x, x (closing) along the cage's n axis in the world."""
    f = np.array((1.0, 0.0, 0.0))
    c = np.array((0.0, G.N[0], G.N[1]))
    return np.column_stack((c, np.cross(f, c), f))


def solve(p, r, arm="right"):
    best = None
    for q0 in (READY, [0.0, 1.5, 1.5, -0.3, 0.0, 0.0], [0.3, 1.2, 0.8, 0.2, 0.0, 1.5], [-0.3, 1.2, 0.8, 0.2, 0.0, -1.5]):
        q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), np.array(q0, float), base_pos=np.array(BASES[arm]), iters=200)
        if best is None or pe + 0.05 * re < best[1] + 0.05 * best[2]:
            best = (q, pe, re)
    return best


if __name__ == "__main__":
    r = frame_rot()
    cap = G.cage_to_frame(0.0, 0.0, 0.172)  # (lateral, X, Z)
    for bb_h, xf, cap_y in itertools.product((0.02, 0.06, 0.10), (0.30, 0.34, 0.38), (-0.30, -0.22, -0.14, -0.06)):
        p = np.array((xf, cap_y, TOP + bb_h + cap[2]))
        out = []
        for back in (0.0, 0.08):  # seated, and backed 8 cm off toward the robot
            q, pe, re = solve(p - (back, 0.0, 0.0), r)
            out.append(f"{pe * 1000:5.1f}mm/{np.degrees(re):4.1f}deg")
        print(f"bb_h {bb_h:.2f} xf {xf:.2f} cap_y {cap_y:+.2f} cap_z {p[2]:.3f}: " + "  ".join(out))
