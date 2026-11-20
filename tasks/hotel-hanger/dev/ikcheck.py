"""Development check: replay the demo plan through the scripted policy's own IK stepping (20 Hz, warm-started) and
report the worst IK error per waypoint label. Usage: python dev/ikcheck.py <seed> ..."""
import pathlib
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import hotel_hanger as t  # noqa: E402
from bodykit import state_from_pose  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

print("seat (ring frame, mm):", np.round(t.SEAT * 1000, 2))
for seed in [int(a) for a in sys.argv[1:]] or [0]:
    lay = t.layout(seed)
    st = {n: state_from_pose(t.SPEC[n], p, q) for n, (p, q) in t.poses(lay).items()}
    print(f"seed {seed}: layout { {k: round(v, 3) for k, v in lay.items()} }, A seated {t.seated(st, 'hanger_a', 'ring_a')}"
          f", B low {t.lowest_z(st, 'hanger_b') - t.TOP:.4f}")
    plan = t.plan_from_state(st)
    for arm, wps in plan.items():
        ap = ArmPlan([t.park(arm, 0.5, label="look")] + list(wps), station.BASES[arm])
        worst = defaultdict(float)
        for k in range(int(ap.duration * 20) + 1):
            tt = k / 20.0
            pos, rot, _, label = ap.target(tt)
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), ap.q, base_pos=ap.base_pos, iters=60)
            ap.q = q
            worst[label] = max(worst[label], pe * 1000 + np.degrees(re))
        bad = {k: round(v) for k, v in worst.items() if v > 5}
        print(f"  {arm}: steps {int(ap.duration * 20)}, labels over 5 (mm+deg): {bad}")
