"""Development check: cold multi-start IK at every left-hand waypoint of the demo plan (is each pose reachable at
all?). Usage: python dev/wpcheck.py <seed>"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import hotel_hanger as t  # noqa: E402
from bodykit import state_from_pose  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import READY  # noqa: E402

seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
arm = sys.argv[2] if len(sys.argv) > 2 else "left"
st = {n: state_from_pose(t.SPEC[n], p, q) for n, (p, q) in t.poses(t.layout(seed)).items()}
plan = t.plan_from_state(st)
for w in plan[arm]:
    rot = np.asarray(w.yaw, float)
    best, bq = 1e9, None
    for q0 in (READY, [0.0, 1.5, 1.2, -0.3, 0.0, 0.0], [0.6, 1.2, 1.0, 0.0, 0.5, 1.5], [-0.6, 1.2, 1.0, 0.0, -0.5, -1.5]):
        q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(rot), np.array(q0, float),
                             base_pos=station.BASES[arm], iters=80)
        e = pe * 1000 + math.degrees(re)
        if e < best:
            best, bq = e, q
    print(f"{w.label:32s} pos {np.round(w.pos, 3)} err {best:6.1f} q {np.round(bq, 2)}")
