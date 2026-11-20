"""Development check: worst warm-started IK error (mm+deg) of the left hand's pick moves for several approach distances,
on the given seeds. Usage: python dev/approachcheck.py <seed> ..."""
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import hotel_hanger as t  # noqa: E402
from bodykit import state_from_pose  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

import itertools

if sys.argv[1:2] == ["sweep"]:  # hanger shifts toward the robot, turned, at both y ends
    lays = [dict(ring_dy=0.0, ring_yaw=15.0, hx=a, hy=b, hyaw=c) for a in (-0.03, -0.025, -0.02, -0.015, -0.01)
            for b in (-0.02, 0.02) for c in (-10.0, 10.0)]
elif sys.argv[1:] == ["corners"]:  # the corners of the random range (ring at its reference place)
    lays = [dict(ring_dy=dy, ring_yaw=ry, hx=a, hy=b, hyaw=c) for a, b, c in itertools.product(
        (-t.HANGER_DX, t.HANGER_DX), (-t.HANGER_DY, t.HANGER_DY), (-t.HANGER_YAW, t.HANGER_YAW))
        for dy, ry in ((-t.RING_SHIFT, 22.0), (t.RING_SHIFT, -22.0))]
else:
    lays = [t.layout(int(a)) for a in sys.argv[1:]]
for back, upz in ((0.06, 0.06),):
    t.APPROACH_BACK, t.APPROACH_UP = back, upz
    out = []
    for lay in lays:
        st = {n: state_from_pose(t.SPEC[n], p, q) for n, (p, q) in t.poses(lay).items()}
        wps = t.plan_from_state(st)["left"]
        ap = ArmPlan([t.park("left", 0.5, label="look")] + list(wps), station.BASES["left"])
        worst = 0.0
        for k in range(int(ap.duration * 20) + 1):
            pos, rot, _, label = ap.target(k / 20.0)
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), ap.q, base_pos=ap.base_pos, iters=60)
            ap.q = q
            e = pe * 1000 + np.degrees(re)
            if e > worst:
                worst, wl = e, label[:12]
        out.append(f"{worst:.0f}{':' + wl if worst > 5 else ''}")
    print(f"back {back*1000:.0f} up {upz*1000:.0f}: worst {out}", flush=True)
