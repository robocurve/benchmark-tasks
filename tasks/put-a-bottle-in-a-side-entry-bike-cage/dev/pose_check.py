"""Multi-start IK at the demo's key poses (offline). Created 11_16_26 (America/New_York)."""
import sys

import numpy as np

sys.path.insert(0, ".")
import task as T  # noqa: E402
from dev.reach_scan import solve  # noqa: E402

if __name__ == "__main__":
    up = np.array((0.0, 0.0, 1.0))
    for sign in (1, -1):
        c = sign * T.C_SEAT
        rs = T.GR.frame(T.F_SEAT, c)
        r0 = T.rot_x(-T.TILT) @ rs
        for name, p, r in (("grasp", T.held_tcp(np.array((*T.BOTTLE_START, T.TOP)), up), r0),
                           ("pre", T.tcp_at_cage(T.PRE_S, 0, T.LIFT_A), rs), ("seat", T.tcp_at_cage(0, 0, 0), rs),
                           ("back", T.tcp_at_cage(T.BACK_S, 0, 0), rs)):
            q, pe, re = solve(p, r)
            print(sign, name, np.round(p, 3), f"{pe * 1000:.1f} mm {np.degrees(re):.1f} deg", np.round(q, 2))
