"""Offline clearance audit of the scripted demo (real IK and the URDF's collision hulls; no simulator).

    python check_collisions.py [--modes demo ...] [--variants A B]     # needs yam_arena importable (numpy only)

Every obstacle is built in WORLD coordinates (the table top is at z = 0.7475 m): the table, the grinder's motor, rear
cap, gear head, collar, flange, disc, nut and side handle, the guard's skirt (12 boxes, at the guard's angle at that
moment) and the lug and lever. The arm hulls come from the forward kinematics of the IK solution at every second control
step. Reported, per mode and layout:

- the smallest clearance between any arm hull and the scene, leaving out the parts a hand is meant to touch: the finger
  links and the gripper body may touch the lug, the lever and (the holding arm only) the motor housing;
- the smallest clearance of the finger links to the rest of the grinder (skirt, disc, head ...);
- the smallest hull-to-hull clearance between the two arms.

Pass: the first two at least 4 mm, the arm-arm clearance at least 10 mm.
"""

from __future__ import annotations

import argparse
import math
import sys
from types import SimpleNamespace

import numpy as np

import demo
import geometry as g

SCENE_MIN_MM, ARM_ARM_MIN_MM = 4.0, 10.0
FINGER_LINKS = ("tip_left", "tip_right", "gripper")
STEP = 3
LUG_ABOVE = demo.LUG_ABOVE


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def box_d(points, centre, rot, size):
    from yam_arena import analysis

    return analysis.box_signed_distance(points, np.asarray(centre, float), rot, np.asarray(size, float))


def cyl_d(points, centre, rot, radius, half):
    """Signed distance to a cylinder whose axis is the local z axis of ``rot`` (half: half its length)."""
    local = (points - centre) @ rot
    dr = np.hypot(local[:, 0], local[:, 1]) - radius
    dz = np.abs(local[:, 2]) - half
    outside = np.hypot(np.maximum(dr, 0.0), np.maximum(dz, 0.0))
    return outside + np.minimum(np.maximum(dr, dz), 0.0)


def scene(L: dict, q_guard: float):
    """(name, kind, distance function) for everything on the table, in world coordinates. ``kind``: 'table', 'grinder'
    (the body: the holding hand may touch the motor), 'guard' (the skirt), 'touch' (lug and lever)."""
    sx, sy = L["spindle"]
    R = rot_z(L["yaw"])                                      # the asset's frame in the world
    top = g.TOP
    o = np.array([sx, sy, top])

    def at(x, y, z):
        return o + R @ np.array([x, y, z])

    out = [("table", "table", lambda p: box_d(p, [0.3, 0.0, top - 0.03], np.eye(3), [1.1, 1.3, 0.06]))]
    # the motor and the rear cap lie along the asset's y: a cylinder whose z axis is turned to y
    to_y = R @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)      # local z -> asset y
    out.append(("motor", "motor", lambda p: cyl_d(p, at(0, (g.MOTOR_Y0 + g.MOTOR_Y1) / 2, g.MOTOR_R), to_y, g.MOTOR_R,
                                                  (g.MOTOR_Y1 - g.MOTOR_Y0) / 2)))
    out.append(("rear cap", "motor", lambda p: cyl_d(p, at(0, (g.CAP_Y0 + g.MOTOR_Y0) / 2, g.MOTOR_R), to_y, g.CAP_R,
                                                       (g.MOTOR_Y0 - g.CAP_Y0) / 2)))
    out.append(("gear head", "grinder", lambda p: box_d(p, at(0, (g.HEAD_Y0 + g.HEAD_Y1) / 2, g.HEAD_H / 2), R,
                                                        [2 * g.HEAD_HX, g.HEAD_Y1 - g.HEAD_Y0, g.HEAD_H])))
    to_x = R @ np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)      # local z -> asset x
    out.append(("handle", "grinder", lambda p: cyl_d(p, at(-(g.HEAD_HX + g.HANDLE_LEN / 2), 0, g.HANDLE_Z), to_x,
                                                     g.HANDLE_R, g.HANDLE_LEN / 2)))
    for name, r, z0, z1 in (("collar", g.NECK_R, g.NECK_Z0, g.NECK_Z1), ("flange", g.FLANGE_R, g.FLANGE_Z0, g.FLANGE_Z1),
                            ("disc", g.DISC_R, g.DISC_Z0, g.DISC_Z1), ("nut", g.NUT_R, g.NUT_Z0, g.NUT_Z1)):
        out.append((name, "grinder", lambda p, r=r, z0=z0, z1=z1: cyl_d(p, at(0, 0, (z0 + z1) / 2), np.eye(3), r, (z1 - z0) / 2)))
    # the guard: its skirt as 12 boxes (as in build_assets.py), the lug and the lever, turned with the guard
    a_g = L["yaw"] + q_guard
    h = g.SKIRT_Z1 - g.PLATE_Z0
    r_c = g.SKIRT_R_IN + g.SKIRT_T / 2
    chord = 2 * r_c * math.sin(g.SKIRT_SPAN / 12 / 2) * 1.08
    for k in range(12):
        a = a_g - g.SKIRT_SPAN / 2 + (k + 0.5) * g.SKIRT_SPAN / 12
        c = o + [r_c * math.cos(a), r_c * math.sin(a), g.PLATE_Z0 + h / 2]
        out.append((f"skirt{k}", "guard", lambda p, c=c, a=a: box_d(p, c, rot_z(a), [g.SKIRT_T + 0.0005, chord, h])))
    a_l = a_g + g.LUG_PHI
    c = o + [(g.SKIRT_R_IN + g.SKIRT_T + g.LUG_L / 2) * math.cos(a_l), (g.SKIRT_R_IN + g.SKIRT_T + g.LUG_L / 2) * math.sin(a_l),
             g.LUG_MID]
    out.append(("lug", "touch", lambda p: box_d(p, c, rot_z(a_l), [g.LUG_L, g.LUG_W, g.LUG_H])))
    phi = a_g + g.LEVER_PHI
    hinge = np.array(g.lever_hinge(L, q_guard))
    for name, beta in (("lever (flat)", 0.0), ("lever (raised)", math.radians(90))):
        d = np.array([math.cos(phi) * math.cos(beta), math.sin(phi) * math.cos(beta), math.sin(beta)])
        c = hinge + d * g.LEVER_L / 2
        # the lever's long axis is d; build a frame with x along d and z roughly up (flat) or radial (raised)
        zax = np.array([0.0, 0.0, 1.0]) if beta < 1 else np.array([math.cos(phi), math.sin(phi), 0.0]) * -1
        yax = np.cross(zax, d)
        yax /= np.linalg.norm(yax)
        zax = np.cross(d, yax)
        out.append((name, "touch", lambda p, c=c, F=np.c_[d, yax, zax]: box_d(p, c, F, [g.LEVER_L, g.LEVER_W, g.LEVER_T])))
    return out


def guard_angle(L: dict, op_arm, q_prev: float) -> float:
    """The guard's joint angle while the operating hand is turning it: the lug follows the hand's bearing (the hand is
    closed on the lug and out at the lug's radius); otherwise the guard stays where it was."""
    sx, sy = L["spindle"]
    d = op_arm.pos[:2] - np.array([sx, sy])
    r = float(np.hypot(*d))
    on_lug = (op_arm.grip < 0.6 and g.SKIRT_R_IN < r < g.SKIRT_R_IN + g.LUG_L + 0.02
              and op_arm.pos[2] < g.TOP + g.LUG_MID + LUG_ABOVE + 0.01)
    if not on_lug:
        return q_prev
    return g.wrap(math.atan2(d[1], d[0]) - g.LUG_PHI - L["yaw"])


def audit(mode: str, variant: str) -> dict:
    from yam_arena import analysis, ik
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN

    from demokit import Arm

    L = g.layout(variant)
    op_side = "left" if L["spindle"][1] > 0 else "right"
    ctx = SimpleNamespace(left=Arm("left"), right=Arm("right"))
    q_guard = L["guard_q0"]
    res = {"scene": (np.inf, ""), "fingers": (np.inf, ""), "arms": (np.inf, ""), "steps": 0}
    for step, _ in enumerate(demo.script(ctx, mode, variant)):
        res["steps"] = step + 1
        if step % STEP:
            continue
        op_arm = getattr(ctx, op_side)
        q_guard = guard_angle(L, op_arm, q_guard)
        obstacles = scene(L, q_guard)
        hulls = {}
        for side in ("left", "right"):
            arm = getattr(ctx, side)
            q = {n: float(v) for n, v in zip(ARM_JOINTS, arm.q, strict=True)}
            q.update({n: float(FINGER_OPEN * arm.grip) for n in FINGER_JOINTS})
            hulls[side] = analysis.world_hulls(analysis.fk_link_poses_world(q, arm.base))
            for name, h in hulls[side]:
                if name == "base":
                    continue
                pts = h.vertices
                for oname, kind, fn in obstacles:
                    finger = name in FINGER_LINKS
                    if kind == "touch" and finger:
                        continue                                      # the lug and the lever are what the hand is for
                    if kind == "motor" and finger and side != op_side:
                        continue                                      # the holding hand grips the motor housing
                    d = float(fn(pts).min() * 1000)
                    key = "fingers" if (finger and kind in ("guard", "grinder")) else "scene"
                    if d < res[key][0]:
                        res[key] = (d, f"{side} {name} vs {oname} at step {step}")
        if step % (2 * STEP) == 0:
            d, pair = analysis.hull_set_distance([h for h in hulls["left"] if h[0] != "base"],
                                                 [h for h in hulls["right"] if h[0] != "base"])
            if d * 1000 < res["arms"][0]:
                res["arms"] = (d * 1000, f"{pair} at step {step}")
    return res


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--modes", nargs="*", default=["demo"])
    ap.add_argument("--variants", nargs="*", default=["A", "B"])
    a = ap.parse_args()
    ok = True
    worst_scene = worst_arms = np.inf
    for v in a.variants:
        for mode in a.modes:
            r = audit(mode, v)
            good = min(r["scene"][0], r["fingers"][0]) >= SCENE_MIN_MM and r["arms"][0] >= ARM_ARM_MIN_MM
            ok &= good
            worst_scene = min(worst_scene, r["scene"][0], r["fingers"][0])
            worst_arms = min(worst_arms, r["arms"][0])
            arms = ">50 mm (never within 5 cm)" if not np.isfinite(r["arms"][0]) else f"{r['arms'][0]:.2f} mm ({r['arms'][1]})"
            print(f"layout {v} {mode:12s} {r['steps']:4d} steps  scene {r['scene'][0]:7.2f} mm ({r['scene'][1]})  "
                  f"fingers-vs-grinder {r['fingers'][0]:7.2f} mm ({r['fingers'][1]})  arm-arm {arms}{'' if good else '  <-- FAIL'}")
    print(f"\nSmallest scene clearance {worst_scene:.2f} mm (need {SCENE_MIN_MM}); smallest arm-arm "
          f"{'>50' if not np.isfinite(worst_arms) else f'{worst_arms:.2f}'} mm (need {ARM_ARM_MIN_MM})")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
