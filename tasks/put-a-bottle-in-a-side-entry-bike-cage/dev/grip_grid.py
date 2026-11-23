"""Offline grid over the grip (diagonal, height up the bottle, depth along the fingers): the demo's worst arm clearance
to the frame and cage, and the bottle's worst unintended overlap. Created 11_16_26 (America/New_York)."""
import itertools
import os
import subprocess
import sys

CODE = r'''
import sys; sys.path.insert(0, ".")
import numpy as np
from yam_arena import analysis as A
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
import task as T
from dev import clearance as C
wps = T.demo(); ap = ArmPlan(list(wps["right"]), BASES["right"]); bx = C.boxes()
worst, wb, t, where = 1.0, 1.0, 0.0, ""
ok = ("cage_floor", "cage_wall_far")
while t <= ap.duration:
    a7, label = ap.action7(t)
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(a7[:6])}; q["joint7"] = q["joint8"] = T.FINGER_OPEN * float(a7[6])
    hulls = A.world_hulls(A.fk_link_poses_world(q, BASES["right"]), C.LINKS)
    pos, rot, _, _ = ap.target(t)
    for name, c, r, s in bx:
        d, link = A.hulls_to_box(hulls, c, r, s)
        if d < worst: worst, where = d, f"{label}/{name}/{link}"
        if label in ("push in from the side", "steady", "down onto the floor") and not name.startswith("arm_") and name not in ok:
            wb = min(wb, A.box_signed_distance(C.bottle_points(pos, rot), c, r, s).min())
    t += 0.1
print(f"{worst*1000:6.1f} {wb*1000:6.1f} {ap.worst_ik_mm:5.1f} {where}")
'''

if __name__ == "__main__":
    for psi, ga, gf in itertools.product(*(map(float, a.split(",")) for a in sys.argv[1:4])):
        env = dict(os.environ, BIKE_GRIP_PSI=str(psi), BIKE_GRIP_A=str(ga), BIKE_GRIP_F=str(gf))
        out = subprocess.run([sys.executable, "-c", CODE], env=env, capture_output=True, text=True)
        print(f"psi {psi:4.0f} a {ga:.3f} f {gf:.3f}: " + (out.stdout.strip() or out.stderr.strip()[-300:]), flush=True)
