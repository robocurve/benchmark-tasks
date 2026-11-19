"""Offline clearance along the scripted demo: the right arm's collision hulls and the carried bottle (sampled surface
points) against every frame and cage box, and the arm against the bottle. Created 11_16_26 (America/New_York)."""
import sys

import numpy as np
from yam_arena import analysis as A
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES

sys.path.insert(0, ".")
import task as T  # noqa: E402
from bike_cage import geom as G  # noqa: E402
from bike_cage import mech as M  # noqa: E402

LINKS = ["link3", "link4", "link5", "gripper", "tip_left", "tip_right"]


def boxes():
    out = [(n, T.ORIGIN + c, r, s) for n, c, s, r, _ in M.frame_boxes()]
    hinge = G.cage_to_frame(G.HINGE[0], G.HINGE[1], 0.0)
    for n, c, s, r in M.arm_boxes():
        out.append(("arm_" + n, T.ORIGIN + hinge + T.RC @ c, T.RC @ r, s))
    return out


def bottle_points(tcp, rot):
    """Surface points of the bottle held in the grip (the grip holds it as at the seated pose; base GRIP_A below the TCP)."""
    ax = rot @ (T.R_SEAT.T @ T.RC[:, 2])
    e1 = rot[:, 0]
    e2 = np.cross(ax, e1)
    base = tcp + T.GRIP_F * rot[:, 2] - T.GRIP_A * ax
    pts = []
    for a in np.linspace(0.0, G.BOTTLE_H, 16):
        r = G.BOTTLE_R if a < G.BODY_TOP else G.CAP_R
        for k in range(20):
            th = 2 * np.pi * k / 20
            pts.append(base + a * ax + r * (np.cos(th) * e1 + np.sin(th) * e2))
    return np.array(pts)


if __name__ == "__main__":
    wps = T.demo()
    ap = ArmPlan(list(wps["right"]), BASES["right"])
    bx = boxes()
    t, worst = 0.0, {}
    carrying = ("grip", "settle", "lift", "carry, tilting", "square to the cage, near side", "steady", "push in from the side", "down onto the floor")
    while t <= ap.duration:
        a7, label = ap.action7(t)
        q = {f"joint{i + 1}": float(v) for i, v in enumerate(a7[:6])}
        fing = T.FINGER_OPEN * float(a7[6])
        q["joint7"] = q["joint8"] = fing
        hulls = A.world_hulls(A.fk_link_poses_world(q, BASES["right"]), LINKS)
        pos, rot, _, _ = ap.target(t)
        for name, c, r, s in bx:
            d, link = A.hulls_to_box(hulls, c, r, s)
            key = (label, name)
            if d < 0.01 and (key not in worst or d < worst[key][0]):
                worst[key] = (round(d * 1000, 1), link, round(t, 2))
            if label in carrying and not name.startswith("arm_"):
                db = A.box_signed_distance(bottle_points(pos, rot), c, r, s).min()
                key = (label, "BOTTLE~" + name)
                if db < 0.004 and (key not in worst or db * 1000 < worst[key][0]):
                    worst[key] = (round(db * 1000, 1), "bottle", round(t, 2))
        t += 0.1
    for k, v in sorted(worst.items(), key=lambda kv: kv[1][0]):
        print(k, v)
    print("done; entries are (signed distance mm, link, t) under 10 mm (arm) or 4 mm (bottle)")
