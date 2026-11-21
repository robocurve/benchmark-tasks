"""Development check: replay the demo's stages through the scripted policy's own IK stepping (20 Hz, warm-started)
for one layout seed, and report the worst IK error per waypoint label. The defective car is treated as found.
Usage: DEMO_VARIANT=... python dev/ikcheck.py <episode seed>"""
import pathlib
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import toy_car_loose_wheel as t  # noqa: E402
from bodykit import state_from_pose  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
st = {n: state_from_pose(t.SPEC[n], p, q) for n, (p, q) in t.poses(seed).items()}
print(t.variant(seed), "travel", t.wheel_travel(st))
t.PLANNER._travel = lambda st_, car: 0.01 if car == "car_bad" else 0.0
plans = {"left": [t.park("left", 0.5, label="look")], "right": [t.park("right", 0.5, label="look")]}
while True:
    nxt = t.next_stage(st)
    if nxt is None:
        break
    tl, tr = (sum(w.duration for w in plans[a]) for a in ("left", "right"))
    end = max(tl, tr)
    for a in plans:
        gap = end - sum(w.duration for w in plans[a])
        if gap > 1e-6:
            w = plans[a][-1]
            plans[a].append(t.Waypoint(gap, w.pos, w.finger, "look", w.yaw))
        plans[a] += nxt.get(a, [])
for arm, wps in plans.items():
    ap = ArmPlan(list(wps), station.BASES[arm])
    worst = defaultdict(float)
    for k in range(int(ap.duration * 20) + 1):
        pos, rot, _, label = ap.target(k / 20.0)
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), ap.q, base_pos=ap.base_pos, iters=60)
        ap.q = q
        worst[label] = max(worst[label], pe * 1000 + np.degrees(re))
    bad = {k: round(v) for k, v in worst.items() if v > 3}
    print(f"  {arm}: {ap.duration:.1f} s, labels over 3 mm+deg: {bad}")
