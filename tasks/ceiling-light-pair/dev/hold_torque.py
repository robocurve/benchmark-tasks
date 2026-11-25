"""Offline statics (URDF kinematics and inertials, no simulator): every joint's torque in each pose the task needs,
holding its load indefinitely, against the motors' rated torque (DM-J4340 9 N m, J1-J3; DM-J4310 3 N m, J4-J6).

Poses, in each station's own frame, solved with yam_arena's IK and the demo's seeds:
- the pinch on each long wall, 80 mm in from this end, approach 35 degrees up: on the stands (A's end 15 mm farther
  from A, B's 15 mm nearer B), lifted, raised (2 mm under the ceiling, before and after the 10 mm slide) and lowered;
  with the hand carrying a quarter of the light's weight (9.1 N, both stations' four hands sharing it) and, as a bound,
  half of it (18.1 N: one hand carrying its whole end);
- the screwdriver: taking it off its holder, under the head and pushing (the driver's 3.0 N; the push adds up to
  10 N upward);
- one station alone: the farthest spot along the walls toward the light's middle its hands can pinch, and what holding
  the whole light from there would take;
- for comparison, the bare arm's own weight at J2, stretched out level.

Usage: python dev/hold_torque.py  (writes dev/hold_torque.txt)"""
from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import geometry as g  # noqa: E402
import scripted as sc  # noqa: E402
from yam_arena import YAM_URDF, ik  # noqa: E402
from yam_arena.constants import ARM_JOINTS, TCP_OFFSET  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, parse_joints, rpy_matrix  # noqa: E402

RATED = np.array(g.RATED)
J = parse_joints(YAM_URDF)
INERT = {}
for link in ET.parse(YAM_URDF).getroot().findall("link"):
    i = link.find("inertial")
    if i is not None and link.get("name") != "base":
        INERT[link.get("name")] = (float(i.find("mass").get("value")),
                                   np.array([float(v) for v in i.find("origin").get("xyz").split()]))
BASES = {"left": np.array([0.0, 0.305, 0.75]), "right": np.array([0.0, -0.305, 0.75])}


def frames(q):
    """Joint positions and axes (arm-base frame), every link's pose, and the tool point."""
    T = np.eye(4)
    out, links = [], {}
    for n, qi in zip(ARM_JOINTS, q):
        j = J[n]
        To = np.eye(4)
        To[:3, :3], To[:3, 3] = rpy_matrix(j.rpy), j.xyz
        T = T @ To
        out.append((T[:3, 3].copy(), T[:3, :3] @ (j.axis / np.linalg.norm(j.axis))))
        Tj = np.eye(4)
        Tj[:3, :3] = axis_angle_matrix(j.axis, qi)
        T = T @ Tj
        links[j.child] = T.copy()
    for n in ("joint7", "joint8"):
        j = J[n]
        To = np.eye(4)
        To[:3, :3], To[:3, 3] = rpy_matrix(j.rpy), j.xyz
        links[j.child] = links["gripper"] @ To
    tcp = links["gripper"][:3, :3] @ np.array(TCP_OFFSET) + links["gripper"][:3, 3]
    return out, links, tcp


def torques(q, load_up: float) -> np.ndarray:
    """|motor torque| per joint holding q still: the arm's own weight, and a vertical force load_up (N, + up) on the
    hand (applied at the tool point; a vertical force's joint torques do not depend on its height)."""
    joints, links, tcp = frames(q)
    order = ["link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right"]
    coms = [(INERT[n][0], links[n][:3, :3] @ INERT[n][1] + links[n][:3, 3]) for n in order]
    tau = np.zeros(6)
    for i, (p, ax) in enumerate(joints):
        t = sum(ax @ np.cross(c - p, np.array([0.0, 0.0, -m * 9.81])) for k, (m, c) in enumerate(coms) if k >= i)
        t += ax @ np.cross(tcp - p, np.array([0.0, 0.0, load_up]))
        tau[i] = -t
    return np.abs(tau)


def solve(side, pos, R, seeds):
    best = None
    for q0 in seeds:
        q0 = np.asarray(q0, float) if side == "left" else sc.mirror(q0)
        q, err, rerr = ik.solve(pos, ik.matrix_to_quat_xyzw(R), q_init=q0, base_pos=BASES[side], iters=400)
        key = (err > 0.001 or rerr > 0.01, err + 0.05 * rerr)
        if best is None or key < best[0]:
            best = (key, q, err)
    return best[1], best[2]


def pinch_pose(side, x, z_back):
    """The pinch on this arm's long wall at station x, the light's back z_back below the ceiling."""
    y = (g.HOUSING_W / 2 - g.WALL_T / 2) * (1 if side == "left" else -1)
    p = np.array([x, y, g.CEIL + z_back + sc.PINCH_Z])
    best = None
    for f in ((0, -1, 0),):                      # the demo's finger direction
        R = sc.rotm(sc.elevated(sc.ELEV), f)
        q, err = solve(side, p, R, sc.PINCH_SEEDS)
        k = torques(q, -9.07).max() if err < 0.001 else 1e9
        if best is None or k < best[0]:
            best = (k, q, err)
    return best[1], best[2]


def row(name, q, loads, err):
    lines = []
    for load in loads:
        t = torques(q, load)
        r = t / RATED
        lines.append(f"| {name} | {load:+.1f} | " + " | ".join(f"{v:.2f}" for v in t)
                     + f" | {r.max():.2f} (J{int(r.argmax()) + 1}) |" + ("" if err < 0.001 else f" IK error {err * 1000:.1f} mm"))
    return lines


def main():
    out = ["# Static joint torques in the task's poses (N m), held indefinitely; ratio = |torque| / rated "
           "(9 N m J1-J3, 3 N m J4-J6)", "",
           "Load: vertical force on the hand, N (+ up). -9.1 N = a quarter of the light's 3.7 kg (each of the four hands); "
           "-18.1 N = half of it (one hand carrying its whole end, a bound). The arm's own weight is always included.", "",
           "| Pose (station frame) | Load N | J1 | J2 | J3 | J4 | J5 | J6 | max ratio |", "|---|---|---|---|---|---|---|---|---|"]
    a0 = g.END_GAP + g.GRIP_IN
    S, C = g.START_SHIFT, sc.CENTRED_DX
    poses = [(f"A on stands (x {a0 + S:.3f}, back -30 mm)", a0 + S, sc.REST_Z),
             (f"B on stands (x {a0 - S:.3f}, back -30 mm)", a0 - S, sc.REST_Z),
             (f"A lifted (x {a0 + S:.3f}, back -18 mm)", a0 + S, sc.LIFT_Z),
             (f"A raised (x {a0 + C:.3f}, back -2 mm)", a0 + C, sc.UP_Z),
             (f"B raised (x {a0 - C:.3f}, back -2 mm)", a0 - C, sc.UP_Z),
             (f"both after the slide (x {a0:.3f}, back -2 mm)", a0, sc.UP_Z),
             (f"both lowered (x {a0:.3f}, back -11 mm)", a0, sc.LOWER_Z)]
    worst = (0.0, "")
    for name, x, zb in poses:
        for side in ("left", "right"):
            q, err = pinch_pose(side, x, zb)
            out += row(f"pinch, {side} arm: {name}", q, (-9.07, -18.15), err)
            r = (torques(q, -9.07) / RATED).max()
            if r > worst[0]:
                worst = (r, f"pinch, {side} arm: {name}")
    # the screwdriver (left arm; the right is its mirror image)
    hx, hy = g.HOLDER_LOCAL["left"]
    yaw = math.radians(g.HOLDER_YAW["left"])
    Rh = sc.rotm(sc.elevated(sc.DRV_ELEV, g.HOLDER_YAW["left"]), (math.sin(yaw), -math.cos(yaw), 0.0))
    grip = np.array([hx, hy, g.TOP + g.HOLDER_FLOOR + g.DRV_GRIP])
    q, err = solve("left", grip, Rh, ([-0.06, 0.37, 0.86, -0.49, -1.1, 0.0], [-0.06, 0.22, 0.65, -0.42, -1.1, 0.0]))
    out += row("screwdriver: taking it off its holder", q, (-3.04,), err)
    head = np.array([g.SCREWS["a_left"][0] + g.SEPARATION / 2, g.SCREWS["a_left"][1], g.CEIL - g.GAP - g.HEAD_H])
    dyaw = math.radians(-sc.DRV_YAW)
    Rs = sc.rotm(sc.elevated(sc.DRV_ELEV, math.degrees(dyaw)), (math.sin(dyaw), -math.cos(dyaw), 0.0))
    for name, dz in (("under the head", -0.030), ("pushing the head, as set", 0.002), ("pushing the head, tight", 0.010)):
        q, err = solve("left", head + [0.0, 0.0, dz - (g.DRV_TIP - g.DRV_GRIP)], Rs, (sc.DRIVE_SEED,))
        out += row(f"screwdriver: {name}", q, (-3.04,) if dz < 0 else (-3.04, -3.04 + 10.0), err)
    q, err = solve("left", np.array([0.18, 0.22, g.TOP + 0.58]), Rs, (sc.DRIVE_SEED, [-1.45, 1.11, 1.99, -0.88, -1.45, 0.0]))
    out += row("screwdriver: held beside the light at the end", q, (-3.04,), err)
    out += ["", f"Worst pinch pose at 9.1 N: {worst[1]}, {worst[0]:.2f} of rated.", ""]
    # one station alone
    out += ["## One station alone", ""]
    far_keys = [np.array([g.SCREWS[s][0] + g.SEPARATION / 2, g.SCREWS[s][1], g.CEIL]) for s in g.STATION_SCREWS["b"]]
    for side in ("left", "right"):
        joints, _, _ = frames(np.zeros(6))
        shoulder = joints[1][0] + BASES[side]
        d = min(np.linalg.norm(k - shoulder) for k in far_keys)
        out.append(f"- A's {side} shoulder (J2) to the nearer of B's two screws: {d * 1000:.0f} mm (YAM reach 750 mm).")
    com = g.END_GAP + g.START_SHIFT + g.HOUSING_L / 2
    x_far = None
    for side in ("left", "right"):
        for x in np.arange(0.70, 0.10, -0.01):
            q, err = pinch_pose(side, x, sc.REST_Z)
            if err < 0.001:
                x_far = x
                break
        t = torques(q, -18.15)
        out.append(f"- A's {side} hand pinches its wall at most {x_far * 1000:.0f} mm from A's arms (light on its stands); "
                   f"the light's centre of mass is {com * 1000:.0f} mm away. Carrying half the light there: |torque| "
                   + ", ".join(f"J{i + 1} {v:.1f}" for i, v in enumerate(t)) + f" N m (max {(t / RATED).max():.2f} of rated).")
    lever = com - x_far
    out.append(f"- To lift the whole light flat from there, A's two pinches would also have to hold "
               f"{g.HOUSING_MASS * 9.81:.1f} N x {lever * 1000:.0f} mm = {g.HOUSING_MASS * 9.81 * lever:.1f} N m of pitch "
               f"between their pads (each pad pair grips about 20 mm of wall height), on top of the arms' torques above.")
    # for comparison: the bare arm's own weight, whatever a policy does with it
    best = (0.0, None, None)
    for q2 in np.linspace(0.0, 3.6, 73):
        for q3 in np.linspace(0.0, 3.14, 64):
            for q4 in np.linspace(-1.6, 1.6, 9):
                q = np.array([0.0, q2, q3, q4, 0.0, 0.0])
                tcp = frames(q)[2]
                if tcp[0] <= 0.05:                # reaching forward, over the table
                    continue
                t = torques(q, 0.0)
                if t[1] > best[0]:
                    best = (t[1], q, tcp)
    # what one hand can carry within 70 % of rated, against its reach: the demo's pinch (left arm, its wall at y 160 mm,
    # the light lifted), moved along x in front of the arm's base
    out += ["", "## What one hand can carry, against reach", "",
            "The demo's pinch on the left long wall (light lifted, back 18 mm under the ceiling), moved along x; the largest "
            "vertical load on the hand with every joint within 70 % of rated (the arm's own weight included).", "",
            "| x in front of the base, mm | load within 70 % of rated, N | limiting joint |", "|---|---|---|"]
    for x in (0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45):
        q, err = pinch_pose("left", x, sc.LIFT_Z)
        if err >= 0.001:
            out.append(f"| {x * 1000:.0f} | no IK solution | |")
            continue
        load, lim = 0.0, ""
        for f in np.arange(0.0, 60.01, 0.25):
            r = torques(q, -f) / RATED
            if r.max() > 0.70:
                lim = f"J{int(r.argmax()) + 1}"
                break
            load = f
        out.append(f"| {x * 1000:.0f} | {load:.1f} | {lim or '-'} |")
    out += ["", "## The bare arm", "",
            f"- With no load, the arm's own weight needs at most {best[0]:.1f} N m at J2 ({best[0] / RATED[1]:.2f} of rated), "
            f"stretched out level in front of it (q = {np.round(best[1], 2).tolist()}, the tool point "
            f"{best[2][0] * 1000:.0f} mm in front of the base, {best[2][2] * 1000:.0f} mm above it); a policy that holds an arm out like that loads J2 "
            f"past its rating with no task load at all."]
    text = "\n".join(out)
    print(text)
    open(os.path.join(HERE, "hold_torque.txt"), "w").write(text + "\n")


if __name__ == "__main__":
    main()
