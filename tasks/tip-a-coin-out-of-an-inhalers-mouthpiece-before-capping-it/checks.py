#!/usr/bin/env python3
"""Checks (a) of the inhaler task without the simulator: every hand pose through the arms' IK with its joint margins,
the hands' collision hulls (the URDF's) against each other, the bench, the inhaler and the cap (as boxes), and what the
wrist cameras see (the coin projected through the calibrated wrist camera).

    PYTHONPATH=<yam_arena checkout> python3 mech/tasks/inhaler_coin/checks.py

Plan checked. The right hand takes the inhaler lying on its side by the barrel from above, 30 mm from the mouthpiece's
axis, and lifts it; turns it about the vertical so that the mouthpiece points at the arms; the left wrist camera
looks into the mouthpiece; the right hand tips the mouthpiece down (a turn about the barrel) and shakes; tips it back
level; turns the inhaler upright (about the mouthpiece's axis) so that the long axis of the opening lies level; the
left hand takes the cap lying on its broad side from above and pushes it along the mouthpiece; the right hand lays the
capped inhaler in the tray.
"""

from __future__ import annotations

import json
import math
import sys

import numpy as np
from yam_arena import analysis, ik, station
from yam_arena import constants as C
from yam_arena.constants import READY

TOP = 0.7475  # bench top of the station scene (yam_arena.task_file.TABLE_TOP["station"])
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ["link4", "link5", "gripper", "tip_left", "tip_right"]

# --- the inhaler and cap, as in parts.py (m; (a) assumed) ---
BARREL_R, BOOT_LOW, BOOT_HIGH, CAN_TOP = 0.015, -0.015, 0.045, 0.065
MOUTH_X, BORE_BACK, OUT_A, OUT_B = 0.047, 0.005, 0.0145, 0.0115  # mouthpiece's outside: half-widths along I's y and z
GRASP_Z = 0.033  # the barrel is taken 33 mm from the mouthpiece's axis
CAP = np.array([0.020, 0.0312, 0.0252])  # the cap's box along its axis, long and short axes
DIME_R = 0.01791 / 2
COIN_OUT = 0.45
LYING_OUT = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # I's x to -y, y up, z to -x
TRAY_AT, TRAY = np.array([0.30, 0.22]), np.array([0.180, 0.110, 0.025])


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def frame(closing, approach) -> np.ndarray:
    """TCP rotation from the closing axis and the approach axis (columns: closing, pads' width, approach)."""
    a = np.asarray(approach, float) / np.linalg.norm(approach)
    c = np.asarray(closing, float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def box_points(center, rot, size, step=0.002) -> np.ndarray:
    h = np.asarray(size) / 2
    pts = []
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        u = np.linspace(-h[others[0]], h[others[0]], max(2, int(2 * h[others[0]] / step) + 1))
        v = np.linspace(-h[others[1]], h[others[1]], max(2, int(2 * h[others[1]] / step) + 1))
        uu, vv = np.meshgrid(u, v)
        for sign in (-1.0, 1.0):
            p = np.zeros((uu.size, 3))
            p[:, axis], p[:, others[0]], p[:, others[1]] = sign * h[axis], uu.ravel(), vv.ravel()
            pts.append(p)
    return np.vstack(pts) @ np.asarray(rot).T + np.asarray(center)


def solve(side: str, pos, rot, start=None):
    """Best IK of a TCP pose: (q, mm off, deg off, smallest joint margin deg); both finger orders, a few starts."""
    best = None
    starts = [start] if start is not None else []
    starts += [np.array(s, float) for s in ([0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.0, 1.6, 1.4, -0.4, 0.0, 0.0], [0.0, 0.6, 0.6, -1.2, 0.0, 0.0],
                                            [0.0, 1.2, 1.8, 0.4, 0.0, 0.0], [0.0, 1.0, 1.0, 0.6, 0.0, 1.5], [0.0, 1.0, 1.0, 0.6, 0.0, -1.5])]  # fmt: skip
    turns = [rot] if start is not None else [rot, rot @ np.diag([-1.0, -1.0, 1.0])]
    for turned in turns:
        for q0 in starts:
            q, e, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(turned), q0, base_pos=BASES[side], iters=150)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            key = (e < 0.001 and re < 0.01, margin if e < 0.001 and re < 0.01 else -(e + 0.05 * re))
            if best is None or key > best[0]:
                best = (key, q, e * 1000, math.degrees(re), margin)
        if start is not None and best[0][0]:
            break
    return best[1], best[2], best[3], best[4]


def link_poses(side: str, q, opening: float):
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, -opening / 2)}
    return analysis.fk_link_poses_world(joints, tuple(BASES[side]))


def hulls(side: str, q, opening: float):
    poses = link_poses(side, q, opening)
    out = []
    for link in HAND_LINKS:
        for i, h in enumerate(analysis.link_hulls().get(link, [])):
            r, p = poses[link][:3, :3], poses[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, i, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def to_box(hs, box) -> float:
    center, rot, size = box
    pts = box_points(center, rot, size)
    return float(min(min(analysis.box_signed_distance(h.vertices, center, rot, size).min(), analysis._signed(pts, h).min()) for _, _, h in hs))


def hands_apart(ha, hb) -> float:
    best = np.inf
    for _, _, a in ha:
        for _, _, b in hb:
            lo_a, hi_a, lo_b, hi_b = a.vertices.min(0), a.vertices.max(0), b.vertices.min(0), b.vertices.max(0)
            if np.maximum(lo_b - hi_a, lo_a - hi_b).max() > 0.05:
                continue
            best = min(best, analysis._signed(a.vertices, b).min(), analysis._signed(b.vertices, a).min())
    return float(best)


def lowest(hs) -> float:
    return float(min(h.vertices[:, 2].min() for _, _, h in hs))


def qm(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])  # fmt: skip


def camera(side: str, q, opening: float = 0.094):
    """The wrist camera's world frame (OpenGL: looks along -z, y up) and its lens."""
    t = link_poses(side, q, opening)["gripper"]
    cam = station.wrist_camera(side)
    return t[:3, :3] @ qm(cam.rot_xyzw), t[:3, :3] @ np.array(cam.pos) + t[:3, 3], cam.lens


def project(cam, p) -> tuple[np.ndarray, float]:
    r, o, lens = cam
    c = r.T @ (np.asarray(p, float) - o)
    return np.array([lens.cx + lens.fx * c[0] / -c[2], lens.cy - lens.fy * c[1] / -c[2]]), float(-c[2])


def coin_view(cam, centre, normal_axis_a, normal_axis_b) -> dict:
    """The coin's middle in the picture and its size across two of its diameters (px)."""
    uv, d = project(cam, centre)
    sizes = [float(np.linalg.norm(project(cam, centre + DIME_R * ax)[0] - project(cam, centre - DIME_R * ax)[0])) for ax in (normal_axis_a, normal_axis_b)]
    inside = bool(0 <= uv[0] <= cam[2].width and 0 <= uv[1] <= cam[2].height)
    return {"px": uv.round(0).tolist(), "dist_m": round(d, 3), "across_px": [round(s, 1) for s in sizes], "in_frame": inside}


class Inhaler:
    """The inhaler's frame I (origin where the mouthpiece's axis meets the barrel's) in the world."""

    def __init__(self, p, r) -> None:
        self.p, self.r = np.asarray(p, float), np.asarray(r, float)

    def world(self, local) -> np.ndarray:
        return self.p + self.r @ np.asarray(local, float)

    def boxes(self):
        boot = (self.world([0.0, 0.0, 0.5 * (BOOT_LOW + CAN_TOP)]), self.r, np.array([2 * BARREL_R, 2 * BARREL_R, CAN_TOP - BOOT_LOW]))
        mouth = (self.world([0.5 * (BORE_BACK + MOUTH_X), 0.0, 0.0]), self.r, np.array([MOUTH_X - BORE_BACK, 2 * OUT_A, 2 * OUT_B]))
        return boot, mouth

    def coin(self) -> np.ndarray:
        return self.world([MOUTH_X - DIME_R + COIN_OUT * 2 * DIME_R, -0.0048, 0.0])


GRASP_IN_I = None  # the right hand's TCP rotation in I, fixed by the grasp at the start


def grasp(inh: Inhaler):
    """The right hand's TCP on the barrel for the inhaler at ``inh``: position and rotation."""
    return inh.world([0.0, 0.0, GRASP_Z]), inh.r @ GRASP_IN_I


def main() -> dict:
    global GRASP_IN_I
    out: dict = {}
    ready_q = np.array(READY, float)
    cam_r0 = camera("right", ready_q)
    # (1) the start: the barrel pinch from above, and the coin in the right wrist camera at the ready pose, for several places
    rows = []
    for x0 in (0.375, 0.39, 0.40, 0.41, 0.42, 0.43):
        inh = Inhaler([x0, -0.256, TOP + BARREL_R + 0.001], LYING_OUT)
        rot = frame([0.0, 1.0, 0.0], [0.0, 0.0, -1.0])
        q, e, re, m = solve("right", inh.world([0.0, 0.0, GRASP_Z]), rot)
        h = hulls("right", q, 0.045)
        boot, mouth = inh.boxes()
        rows.append({"x0": x0, "ik_mm": round(e, 2), "ik_deg": round(re, 2), "margin_deg": round(m, 1), "low_mm": round(1000 * (lowest(h) - TOP), 1),
                     "to_mouthpiece_mm": round(1000 * to_box(h, mouth), 1), "coin": coin_view(cam_r0, inh.coin(), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))})  # fmt: skip
    out["1_start"] = rows
    # the grasp fixed in I for the chosen place
    x0 = float(sys.argv[1]) if len(sys.argv) > 1 else 0.375
    inh0 = Inhaler([x0, -0.256, TOP + BARREL_R + 0.001], LYING_OUT)
    GRASP_IN_I = inh0.r.T @ frame([0.0, 1.0, 0.0], [0.0, 0.0, -1.0])
    # (2) lifted and turned about the vertical so that the mouthpiece points at the arms; the left wrist camera looks in
    r_turn = rz(-math.pi / 2) @ LYING_OUT
    out["2_held"] = []
    for held in ([0.27, -0.16, TOP + 0.13], [0.29, -0.10, TOP + 0.13], [0.30, -0.06, TOP + 0.12], [0.30, -0.06, TOP + 0.16]):
        held = np.array(held)
        inh_t = Inhaler(held - r_turn @ np.array([0.0, 0.0, GRASP_Z]), r_turn)
        g, rot = grasp(inh_t)
        q_hold, e, re, m = solve("right", g, rot)
        h_hold = hulls("right", q_hold, 0.030)
        mouth_c = inh_t.world([MOUTH_X, 0.0, 0.0])
        looks = []
        for back in (0.06, 0.09, 0.12):
            for up in (0.04, 0.08, 0.12):
                for side_off in (0.0, 0.05, 0.10):
                    for roll in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                        tcp = mouth_c + np.array([-back, side_off, up])
                        a = (mouth_c - tcp) / np.linalg.norm(mouth_c - tcp)
                        c0 = np.cross(a, [0.0, 0.0, 1.0])
                        c0 /= np.linalg.norm(c0)
                        ql, el, rel, ml = solve("left", tcp, frame(c0, a) @ rz(roll))
                        if el > 1.0 or rel > 1.0:
                            continue
                        hl = hulls("left", ql, 0.030)
                        view = coin_view(camera("left", ql, 0.030), inh_t.coin(), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]))
                        looks.append({"back": back, "up": up, "side": side_off, "roll": round(math.degrees(roll)), "margin": round(ml, 1),
                                      "hands_mm": round(1000 * hands_apart(hl, h_hold), 1), "coin": view})
        looks = [r for r in looks if r["coin"]["in_frame"] and r["hands_mm"] > 10.0]
        looks.sort(key=lambda r: -min(r["coin"]["across_px"]))
        # the tip to 70 and 80 degrees, and the shake at 70
        tips, q = [], q_hold
        for deg in range(0, 81, 10):
            r_i = ry(-math.radians(deg)) @ r_turn
            inh = Inhaler(held - r_i @ np.array([0.0, 0.0, GRASP_Z]), r_i)
            g2, rot2 = grasp(inh)
            qn, e2, re2, m2 = solve("right", g2, rot2, start=q)
            tips.append((deg, round(e2, 2), round(m2, 1), round(float(np.degrees(np.abs(qn - q)).max()), 1), round(1000 * (lowest(hulls("right", qn, 0.03)) - TOP), 1)))
            q = qn
            if deg == 70:
                q70, g70, rot70 = qn, g2, rot2
        shake = [round(solve("right", g70 + np.array([0.0, 0.0, dz]), rot70, start=q70)[1], 2) for dz in (-0.015, 0.015)]
        # the cap pushed on by the left hand, the cap turned so that its long axis stands (approach -y, closing upright)
        caps = []
        for gap in (0.010, 0.0):
            centre = inh_t.world([MOUTH_X + gap - (0.5 * CAP[0] - 0.0015), 0.0, 0.0])
            cap_box = (centre, np.column_stack([-r_turn[:, 0], r_turn[:, 1], np.cross(-r_turn[:, 0], r_turn[:, 1])]), CAP)
            qc, ec, rec, mc = solve("left", centre + np.array([-0.004, 0.0, 0.0]), frame([0.0, 0.0, 1.0], [0.0, -1.0, 0.0]))
            hc = hulls("left", qc, 0.0252)
            caps.append({"gap_mm": round(1000 * gap), "ik_mm": round(ec, 2), "margin": round(mc, 1), "hands_mm": round(1000 * hands_apart(hc, h_hold), 1),
                         "left_to_boot_mm": round(1000 * to_box(hc, inh_t.boxes()[0]), 1), "right_to_cap_mm": round(1000 * to_box(h_hold, cap_box), 1)})
        out["2_held"].append({"held": (held - [0, 0, TOP]).round(3).tolist(), "hold_ik_mm": round(e, 2), "hold_margin": round(m, 1), "j6": round(math.degrees(q_hold[5])),
                              "looks_ok": len(looks), "look_best": looks[:2], "tip(deg,ik,margin,step,low)": tips, "shake_ik_mm": shake, "cap": caps})
    # (3) the cap taken from the bench by the left hand (lying on its broad side at (0.16, 0.06), long axis along x), from above
    cap_c = np.array([0.16, 0.06, TOP + 0.0126 + 0.001])
    qk, ek, rek, mk_ = solve("left", cap_c + np.array([0.0, 0.004, 0.0]), frame([1.0, 0.0, 0.0], [0.0, 0.0, -1.0]))
    out["3_cap_pick"] = {"ik_mm": round(ek, 2), "margin": round(mk_, 1), "low_mm": round(1000 * (lowest(hulls("left", qk, 0.0312)) - TOP), 1)}
    # (4) laid in a tray by the right hand: tray places
    out["4_tray_right"] = []
    for tx, ty in ((0.30, 0.22), (0.22, 0.02), (0.28, -0.02), (0.20, -0.42), (0.26, -0.44), (0.16, -0.38)):
        lay = Inhaler([tx, ty - 0.02, TOP + 0.003 + BARREL_R + 0.002], LYING_OUT)
        q5, e5, re5, m5 = solve("right", lay.world([0.0, 0.0, GRASP_Z]), frame([0.0, 1.0, 0.0], [0.0, 0.0, -1.0]))
        out["4_tray_right"].append({"tray": [tx, ty], "ik_mm": round(e5, 2), "margin": round(m5, 1), "low_mm": round(1000 * (lowest(hulls("right", q5, 0.03)) - TOP), 1)})
    return out


if __name__ == "__main__":
    res = main()
    print(json.dumps(res, indent=1, default=str))
