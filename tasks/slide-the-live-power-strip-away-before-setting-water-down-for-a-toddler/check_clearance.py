"""Offline: how close does the scripted demo bring any part of the arms (their full link meshes, not just the link
origins the scorer uses) to the girl? Plays the demo plan through IK and forward kinematics every control step.
(no simulator)  Run: python check_clearance.py"""
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial import cKDTree
from yam_arena import YAM_URDF, station, task_file
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, READY
from yam_arena.demo import ArmPlan
from yam_arena.urdf import forward_kinematics, rpy_matrix

T = task_file.load("task.py").module
urdf = pathlib.Path(YAM_URDF)
MESH = {}
for link in ET.parse(urdf).getroot().findall("link"):
    vis = link.find("visual")
    if vis is None:
        continue
    o = vis.find("origin")
    xyz = np.array([float(a) for a in o.get("xyz").split()])
    rpy = np.array([float(a) for a in o.get("rpy").split()])
    v = trimesh.load(urdf.parent / vis.find("geometry/mesh").get("filename")).vertices
    MESH[link.get("name")] = v @ rpy_matrix(rpy).T + xyz

girl = cKDTree(T.GIRL_POINTS)
hands = cKDTree(T.HAND_POINTS)


def arm_points(q6, finger, base):
    qd = {n: float(a) for n, a in zip(ARM_JOINTS, q6)}
    qd["joint7"] = qd["joint8"] = float(finger)
    fk = forward_kinematics(YAM_URDF, qd)
    return np.concatenate([MESH[n] @ fk[n][:3, :3].T + fk[n][:3, 3] for n in MESH]) + np.asarray(base)


worst = {}
for arm in ("left", "right"):
    wps = T.demo().get(arm, [])
    base = station.BASES[arm]
    plan = ArmPlan(wps, base) if wps else None
    best = (np.inf, "start", None)
    t, duration = 0.0, plan.duration if plan else 0.0
    q, finger, label = np.array(READY, float), FINGER_OPEN, "ready pose"
    while True:
        if plan is not None and t <= duration:
            pos, rot, finger, label = plan.target(t)
            a7, _ = plan.action7(t)
            q = a7[:6]
        pts = arm_points(q, finger, base)
        d, i = girl.query(pts)
        k = int(np.argmin(d))
        if d[k] < best[0]:
            best = (float(d[k]), f"{label} (t = {t:.2f} s)", pts[k])
        t += 0.05
        if plan is None or t > duration:
            break
    worst[arm] = best
    print(f"{arm}: nearest any arm surface comes to her: {best[0] * 100:.1f} cm, during '{best[1]}' at "
          f"{np.round(best[2], 3).tolist()}")
low = min(w[0] for w in worst.values())
print("OK" if low >= T.ARM_TOUCH else "TOO CLOSE", f"(limit {T.ARM_TOUCH * 100:.0f} cm)")
raise SystemExit(0 if low >= T.ARM_TOUCH else 1)
