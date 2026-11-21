"""Development check: replay the demo plan through the scripted policy's own IK stepping (20 Hz, warm-started) and
report the worst IK error per waypoint label. Usage: DEMO_VARIANT=... python dev/ikcheck.py"""
import pathlib
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import spray_nozzle_off as t  # noqa: E402
from bodykit import state_from_pose  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

st = {n: state_from_pose(t.SPEC[n], p, q) for n, (p, q) in t.poses().items()}
print("nozzle", t.nozzle_angle(st), "setting", t.setting(st), "in cabinet", t.in_cabinet(st), "door", t.door_open_deg(st))
plan = t.plan_from_state(st)
for arm, wps in plan.items():
    ap = ArmPlan([t.park(arm, 0.5, label="look")] + list(wps), station.BASES[arm])
    worst = defaultdict(float)
    q6 = []
    for k in range(int(ap.duration * 20) + 1):
        pos, rot, _, label = ap.target(k / 20.0)
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), ap.q, base_pos=ap.base_pos, iters=60)
        ap.q = q
        worst[label] = max(worst[label], pe * 1000 + np.degrees(re))
        q6.append(q[5])
    bad = {k: round(v) for k, v in worst.items() if v > 3}
    print(f"  {arm}: {ap.duration:.1f} s, q6 {min(q6):.2f}..{max(q6):.2f}, labels over 3 mm+deg: {bad}")
