"""Development check (no simulation): replay the demo's stages for one layout seed through IK at 20 Hz, warm-started like
the scripted policy, moving each carried pan to its destination and turned pans to their new heading by hand, and report
the worst IK error (mm + deg) per waypoint label. Usage: DEMO_VARIANT=... python dev/ikcheck.py <seed>"""
import math
import pathlib
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import layout as L  # noqa: E402
import pandemo  # noqa: E402
import saucepan_loose_handle as t  # noqa: E402
from bodykit import state_from_pose  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
st = {n: state_from_pose(t.SPEC[n], p, q) for n, (p, q) in t.poses(seed).items()}
print(f"seed {seed}: loose slot {t.loose_slot(seed) + 1}")
plans = {"left": list(pandemo.demo()["left"]), "right": list(pandemo.demo()["right"])}
P = pandemo.Planner(t)
while True:
    q0 = list(P.queue)
    nxt = P.next_stage(st)
    if nxt is None:
        break
    if any(w.label == "squeeze the handle" for w in nxt.get("left", []) + nxt.get("right", [])) and \
            any(w.label.startswith("rock") for a in nxt.values() for w in a):
        t._EP["angle_max"] = 4.0  # pretend the rock found the loose handle (only pan_loose is read)
    end = max(sum(w.duration for w in plans[a]) for a in plans)
    for arm in plans:
        gap = end - sum(w.duration for w in plans[arm])
        if gap > 1e-6:
            w = plans[arm][-1]
            plans[arm].append(pandemo.Waypoint(gap, w.pos, w.finger, w.label, w.yaw) if hasattr(pandemo, "Waypoint")
                              else type(w)(gap, w.pos, w.finger, w.label, w.yaw))
        plans[arm] += nxt.get(arm, [])
    # move the pans as the stage would
    for arm, ws in nxt.items():
        labels = [w.label for w in ws]
        if "carry the pan" in labels:
            grab = ws[labels.index("squeeze the rim")].pos
            put = ws[labels.index("press the pan down")].pos
            name = min(t.PAN_NAMES, key=lambda n: np.linalg.norm(st[n].pos[:2] - np.array(grab[:2])))
            p = st[name].pos + (np.array(put) - np.array(grab)) + [0, 0, 0.004]
            st[name] = state_from_pose(t.SPEC[name], p, st[name].quat_xyzw)
        if "turn the handle in" in labels:
            b = L.BURNERS[0 if arm == "left" else 1]
            name = min(t.PAN_NAMES, key=lambda n: np.linalg.norm(st[n].pos[:2] - np.array(b[:2])))
            a = math.radians(pandemo.TURN_TO_DEG) * (1 if arm == "left" else -1)
            q = (0.0, 0.0, math.sin(-a / 2), math.cos(-a / 2))
            st[name] = state_from_pose(t.SPEC[name], st[name].pos, q)
for arm, wps in plans.items():
    ap = ArmPlan(list(wps), station.BASES[arm])
    worst = defaultdict(float)
    for k in range(int(ap.duration * 20) + 1):
        pos, rot, _, label = ap.target(k / 20.0)
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), ap.q, base_pos=ap.base_pos, iters=60)
        ap.q = q
        key = f"{label}@({pos[0]:.2f},{pos[1]:.2f},{pos[2]-L.TOP:.2f})" if pe * 1000 + np.degrees(re) > 3 else label
        worst[key] = max(worst[key], pe * 1000 + np.degrees(re))
    bad = {k: round(v) for k, v in worst.items() if v > 3}
    print(f"  {arm}: {ap.duration:.1f} s, labels over 3 mm+deg: {bad}")
print("final places:", {n: t.where(st, n) for n in t.PAN_NAMES})
