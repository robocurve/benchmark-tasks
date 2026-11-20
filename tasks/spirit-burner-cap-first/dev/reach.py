"""Reach check without the simulator: inverse kinematics for every waypoint of the demo plan on the nominal layout.

Run on the GPU machine from the task folder: /data/IsaacLab-Arena/.venv/bin/python dev/reach.py
Prints, per arm and waypoint, the position error (mm) and rotation error (deg) of the best IK solution, and the
smallest distance between the two hands' finger points at the end of each waypoint pair in time.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import layout as L  # noqa: E402
import spirit_burner as T  # noqa: E402
from bodykit import state_from_pose  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import READY  # noqa: E402

spec = {o.name: o for o in T.OBJECTS}
q0 = (0.0, 0.0, 0.0, 1.0)
bb = np.array([L.BURNER_XY[0], L.BURNER_XY[1], L.MAT_TOP])
T._EP["start"] = {"burner": (bb, np.array(q0)), "cap": (np.array([L.CAP_XY[0], L.CAP_XY[1], L.TOP]), np.array(q0)),
                  "matchbox": (np.array([L.MATCH_XY[0], L.MATCH_XY[1], L.TRAY_FLOOR_TOP]), np.array(q0))}


def now(cap_pos, burner_pos=bb):
    return {"burner": state_from_pose(spec["burner"], burner_pos, q0), "cap": state_from_pose(spec["cap"], cap_pos, q0)}


worst = {"left": 0.0, "right": 0.0}
timeline = {"left": [], "right": []}
for stage in range(4):
    if stage == 0:
        T._EP["now"] = None
    elif stage == 1:
        T._EP["now"] = now(bb + np.array([0.004, -0.003, T.HOVER]))
    elif stage == 2:
        T._EP["now"] = now(bb + np.array([-0.003, 0.002, T.LOW]))
    else:
        T._EP["now"] = now(bb + np.array([0, 0, L.SEAT_Z]))
    plans = T.plan_stage(stage, None, T)
    for arm, ws in plans.items():
        q = np.array(READY, float)
        for w in ws:
            rot = np.asarray(w.yaw, float)
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(rot), q, base_pos=station.BASES[arm],
                                 iters=300)
            worst[arm] = max(worst[arm], pe)
            p, r = ik.tcp_pose(q, station.BASES[arm])
            timeline[arm].append((w.duration, T.finger_points(p, r, abs(w.finger))))
            print(f"s{stage} {arm:5s} {w.label[:44]:44s} pos err {1000 * pe:6.1f} mm  rot err {math.degrees(re):5.1f} deg "
                  f"tcp=({w.pos[0]:.3f},{w.pos[1]:.3f},{w.pos[2] - L.TOP:.3f} above table)")


def ends(tl):
    t, out = 0.0, []
    for d, pts in tl:
        t += d
        out.append((t, pts))
    return out


el, er = ends(timeline["left"]), ends(timeline["right"])
best = 9.0
for t, pl in el:
    pr = min(er, key=lambda e: abs(e[0] - t))[1]
    best = min(best, min(float(np.linalg.norm(a - b)) for a in pl for b in pr))
print(f"worst IK position error: left {1000 * worst['left']:.1f} mm, right {1000 * worst['right']:.1f} mm; "
      f"closest finger points between hands at waypoint ends {1000 * best:.0f} mm")
