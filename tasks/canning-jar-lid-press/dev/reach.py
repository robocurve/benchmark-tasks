"""Reach check without the simulator: inverse kinematics for every waypoint of the demo plan on the fixed layout,
for each of the three unsealed jars, plus the hand's clearance to the fridge opening while it is inside.

Run on the GPU machine from the task folder: /data/IsaacLab-Arena/.venv/bin/python dev/reach.py
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import layout as L  # noqa: E402
import canning_jars as T  # noqa: E402
from bodykit import state_from_pose  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import READY  # noqa: E402

spec = {o.name: o for o in T.OBJECTS}
q0 = (0.0, 0.0, 0.0, 1.0)
T._EP["start"] = {o.name: (np.array(o.pos, float), np.array(q0)) for o in T.OBJECTS if o.name in T.LOOSE}
now = {n: state_from_pose(spec[n], T._EP["start"][n][0], q0) for n in T.LOOSE}

for u in range(3):
    worst = {"left": (0.0, ""), "right": (0.0, "")}
    T._EP["now"] = now
    T._EP["travel"] = [0.0015 if i == u else 0.0 for i in range(3)]
    for stage in range(2):
        plans = T.plan_stage(stage, None, T)
        for arm, ws in plans.items():
            q = np.array(READY, float)
            for w in ws:
                rot = np.asarray(w.yaw, float)
                q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(rot), q,
                                     base_pos=station.BASES[arm], iters=300)
                if pe > worst[arm][0]:
                    worst[arm] = (pe, f"s{stage} {w.label} rot err {math.degrees(re):.1f} deg")
                if "fridge" in w.label and arm == "right":
                    p, r = ik.tcp_pose(q, station.BASES[arm])
                    # gripper carriage corners (gripper frame x +-59 mm, y +-34 mm, z 50-90 mm) and finger corners
                    worst_side = 9.0
                    for gx in (-0.059, 0.059):
                        for gy in (-0.034, 0.034):
                            for gz in (0.050, 0.090, 0.100, 0.147):
                                lx = gx if gz <= 0.09 else math.copysign(0.059 if gz < 0.12 else 0.054, gx)
                                ly = gy if gz <= 0.09 else math.copysign(0.020, gy)
                                pt = p + r @ np.array([lx, ly, gz - 0.134])
                                if pt[0] > L.FRIDGE_FRONT_X:
                                    worst_side = min(worst_side, L.FRIDGE_IN[1] / 2 - abs(pt[1] - L.FRIDGE_Y),
                                                     pt[2] - L.FRIDGE_FLOOR_TOP)
                    if worst_side < 9.0:
                        print(f"u{u} {w.label[:34]:34s} pos err {1000 * pe:5.1f} mm, smallest gap to the opening "
                              f"(side or floor) {1000 * worst_side:5.1f} mm")
    print(f"unsealed jar {u + 1}: worst IK error left {1000 * worst['left'][0]:.1f} mm ({worst['left'][1]}), "
          f"right {1000 * worst['right'][0]:.1f} mm ({worst['right'][1]})")
