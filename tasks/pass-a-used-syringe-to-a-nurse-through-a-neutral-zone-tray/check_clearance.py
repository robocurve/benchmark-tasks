"""Offline: how close does the scripted demo bring any part of the arms (their full link meshes, not just the link
origins the scorer uses) to the nurse, and the syringe's surface to her hand? Plays the demo plan through IK and
forward kinematics every control step, the syringe carried rigidly in the grip. (no simulator)
    python check_clearance.py"""
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial import cKDTree
from yam_arena import YAM_URDF, station, task_file
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, READY
from yam_arena.demo import ArmPlan, grip_rotation
from yam_arena.urdf import forward_kinematics, rpy_matrix

T = task_file.load("task.py").module
urdf = pathlib.Path(YAM_URDF)
MESH = {}
for link in ET.parse(urdf).getroot().findall("link"):
    vis = link.find("visual")
    o = vis.find("origin")
    xyz = np.array([float(a) for a in o.get("xyz").split()])
    rpy = np.array([float(a) for a in o.get("rpy").split()])
    MESH[link.get("name")] = trimesh.load(urdf.parent / vis.find("geometry/mesh").get("filename")).vertices @ rpy_matrix(rpy).T + xyz

her, hand = cKDTree(T.NURSE_POINTS), cKDTree(T.HAND_POINTS)


def arm_points(q6, finger, base):
    qd = {n: float(a) for n, a in zip(ARM_JOINTS, q6)}
    qd["joint7"] = qd["joint8"] = float(finger)
    fk = forward_kinematics(YAM_URDF, qd)
    return np.concatenate([MESH[n] @ fk[n][:3, :3].T + fk[n][:3, 3] for n in MESH]) + np.asarray(base)


low = np.inf
for arm in ("left", "right"):
    wps = T.demo().get(arm, [])
    base = station.BASES[arm]
    plan = ArmPlan(wps, base) if wps else None
    best = (np.inf, "ready pose", None)
    t = 0.0
    while True:
        if plan is not None:
            _, _, finger, label = plan.target(min(t, plan.duration))
            q = plan.action7(min(t, plan.duration))[0][:6]
        else:
            q, finger, label = np.array(READY, float), FINGER_OPEN, "ready pose"
        d, i = her.query(arm_points(q, finger, base))
        k = int(np.argmin(d))
        if d[k] < best[0]:
            best = (float(d[k]), f"{label} (t = {t:.2f} s)", None)
        t += 0.05
        if plan is None or t > plan.duration:
            break
    low = min(low, best[0])
    print(f"{arm}: nearest any arm surface comes to her: {best[0] * 100:.1f} cm, during '{best[1]}'")

# the syringe, carried rigidly by the TCP between the grip and the let-go (its grip point under the TCP)
plan = ArmPlan(T.demo()["left"], station.BASES["left"])
worst = (np.inf, "")
t, held = 0.0, False
from yam_arena.task_file import ObjectState  # noqa: E402
while t <= plan.duration:
    pos, rot, finger, label = plan.target(t)
    held = held or label.startswith("grip")
    if label.startswith("let go"):
        held = False
    if held:
        grip = np.asarray(pos)
        rz = np.asarray(rot) @ np.asarray(grip_rotation(0.0)).T          # the gripper's turn about the vertical
        yaw = np.arctan2(rz[1, 0], rz[0, 0]) + np.pi                      # the syringe lies at the gripper's yaw + 180
        q = np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])
        origin = np.r_[T.syringe_origin_for_grip(grip[:2], q), grip[2] - (T.GRIP_Z - T.TOP - 0.001)]   # as gripped
        st = ObjectState(pos=origin, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                         low=origin, high=origin)
        g = T.hand_gap({"syringe": st})
        if g < worst[0]:
            worst = (g, f"{label} (t = {t:.2f} s)")
    t += 0.05
print(f"syringe (carried): nearest its surface comes to her hand: {worst[0] * 100:.1f} cm, during '{worst[1]}'")
ok = low >= T.ARM_CLEAR and worst[0] >= T.HAND_CLEAR
print("OK" if ok else "TOO CLOSE", f"(limits: arm {T.ARM_CLEAR * 100:.0f} cm from her, syringe {T.HAND_CLEAR * 100:.0f} cm from her hand)")
raise SystemExit(0 if ok else 1)
