"""Offline reach test of the demo plan: IK error, joint-limit margin and duration. No simulator. Created 11_15_26."""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import task
from yam_arena import ik

for arm, plans in task.demo().items():
    pl = plans[0]
    lo, hi = pl.q.min(0), pl.q.max(0)
    margin = np.minimum(pl.q - ik.LOWER, ik.UPPER - pl.q).min(0)
    jump = np.abs(np.diff(pl.q, axis=0)).max(0) / pl.dt
    print(arm, f"duration {pl.duration:.1f} s  worst IK {pl.worst_ik_mm:.2f} mm {pl.worst_ik_deg:.2f} deg")
    print("  q min", np.round(lo, 2), "\n  q max", np.round(hi, 2), "\n  limit margin", np.round(margin, 2), "\n  max joint speed", np.round(jump, 2))
    seen = None
    for k, lab in enumerate(pl.labels):
        if lab != seen:
            p, r = ik.tcp_pose(pl.q[k], (0, 0, 0))
            seen = lab

if "-v" in sys.argv:
    from cookie_rack.plan import ry
    from yam_arena.station import BASES
    path = task._right_path()
    q = path.q0.copy()
    for dur, pos, pitch, finger, label in path.legs:
        if pos is None:
            continue
        r1 = ry(pitch) @ path.down
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(r1), q, base_pos=path.base, iters=300, rot_weight=0.5)
        print(f"{label:18s} pos {np.round(pos, 3)} pitch {np.degrees(pitch):5.1f} err {1000 * pe:6.2f} mm {np.degrees(re):5.2f} deg q {np.round(q, 2)}")
