"""Check 1 (stop-loss): reach and clearance for the sling / edge-guard / latched-hook task. Kinematics only:
yam_arena's damped-least-squares IK on the YAM URDF, and the URDF's collision hulls posed by forward kinematics.
No Isaac Sim.

Frame: tabletop, x forward, y left, z up from the table top; arm bases at (0, +-0.305, 0.003) (yam_arena.station).

Key poses (see the task brief):
  guards  : the hand placing / pushing a 3 mm L-shaped magnetic guard on each of the bar's four edges at the
            sling (top edges from above, bottom edges from the side into the 33-38 mm gap);
  hang    : a hand carrying a triangle fitting (held by its base bar, apex 55 mm ahead) over the hook tip, down
            through the latch, into the bowl (bowl bottom z = HOOK_Z over the sling);
  steady  : the other hand pinching the hook's shank;
  pairs   : hang x steady with the two arms' hulls clear of each other.

Clearances are point-cloud distances (hull triangles sampled at <= 4 mm) to boxes and capsules; they are
accurate to about 2 mm.

Run on the GPU box (numpy, scipy and yam_arena):
    python ik_scan.py --layout X --jobs 28 > out/ik_X.json
"""

import argparse
import itertools
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

IN = 0.0254
BAR_S = 4 * IN                 # 4 x 4 in cold-finished square, sharp edges
BAR_L = 12 * IN
BLOCK_H = 1.5 * IN             # 2x4 laid flat
Z_BOT = BLOCK_H
Z_TOP = BLOCK_H + BAR_S
SLING_W = 2 * IN
SLING_T = 0.004
GUARD_LEN = 0.100              # along the edge (2 in sling + 24 mm each side)
GUARD_FL = 0.050               # flange width from the corner
GUARD_T = 0.003
PAD = 0.013                    # pad tips past the TCP (yam_arena.constants.PAD_TIP_PAST_TCP)

# hook: 2 t carbon eye hook with a spring latch (Crosby 320 class). Bowl inside diameter 1.63 in and throat
# 1.13 in are the catalogue's F and J for the 2 t hook (which letter is which is still to be confirmed); the
# section and shank are ESTIMATES.
BOWL_R_IN = 1.63 * IN / 2
HOOK_R = 0.011
FIT_H = 0.055                  # fitting: base bar (held) to apex bar (hooked)
FIT_BAR_R = 0.007

B1_GAP = .010                 # guard held this far off the face while the bottom flange slides in
STEADY_V = (.010, .030, .050, .065)   # heights (above the bowl centre) where the steadying hand pinches the hook
END_PINCH = False
GRIP_X = 0.                    # where along the base bar the hand holds the fitting (0: the middle, where the hook passes)
MARGIN_DEG = 3.
POS_TOL, ROT_TOL = 1e-3, math.radians(1.)
CLEAR_GOAL = 0.020

SEEDS = [np.array(s, dtype=float) for s in (
    (0, 1, 1, -.6, 0, 0), (0, 1.6, 1.3, -1.2, 0, 0), (0, 2.1, 1.0, -.4, 1.5, -.1), (0, 2.1, 1.0, -.4, -1.5, .1),
    (.35, 2.2, .8, 1.4, -1.2, 0), (-.35, 2.2, .8, 1.4, 1.2, -1.57), (.4, 2.5, 1.4, -1.6, 1.1, 0),
    (0, 1.2, 1.6, -1.0, 0, 1.57), (0, 1.2, 1.6, -1.0, 0, -1.57), (0, 2.4, 2.0, -1.4, 0, 0),
    (-.6, 1.5, 1.2, -.3, 0, 0), (.6, 1.5, 1.2, -.3, 0, 0), (0, 0.6, 0.6, 0.6, 0, 0), (0, 1.0, 0.5, 0.9, 0, 0))]


# ---------------------------------------------------------------------------------------------------- hulls
def _stl_triangles(path):
    data = Path(path).read_bytes()
    count = int.from_bytes(data[80:84], "little")
    record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(data[84:84 + record.itemsize * count], dtype=record)["vertices"].astype(float)


def _sample(tris, step=0.004):
    pts = [tris.reshape(-1, 3)]
    for a, b, c in tris:
        n = int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / step))
        if n < 2:
            continue
        for i in range(n + 1):
            for j in range(n + 1 - i):
                u, v = i / n, j / n
                pts.append((a + u * (b - a) + v * (c - a))[None])
    pts = np.concatenate(pts)
    _, keep = np.unique(np.floor(pts / .0025).astype(np.int64), axis=0, return_index=True)   # one point per 2.5 mm voxel
    return pts[keep]


class Hulls:
    """Every collision hull of one YAM arm (URDF), sampled to points, posed by FK."""

    FINGERS = ("tip_left", "tip_right")

    def __init__(self):
        from yam_arena import YAM_URDF
        from yam_arena.urdf import rpy_matrix

        self.urdf = Path(str(YAM_URDF))
        self.parts = []
        for link in ET.parse(self.urdf).getroot().iter("link"):
            name = link.get("name")
            if name == "base":
                continue
            for col in link.findall("collision"):
                o = col.find("origin")
                xyz = np.array([float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split()])
                rpy = np.array([float(v) for v in (o.get("rpy") if o is not None else "0 0 0").split()])
                tris = _stl_triangles(self.urdf.parent / col.find("geometry/mesh").get("filename"))
                pts = _sample(tris) @ rpy_matrix(rpy).T + xyz
                self.parts.append((name, pts))
        self.names = sorted({n for n, _ in self.parts})

    def points(self, q, base, finger=0.):
        """{link: world points}"""
        from yam_arena.constants import ARM_JOINTS
        from yam_arena.urdf import forward_kinematics

        poses = forward_kinematics(self.urdf, {**{n: float(v) for n, v in zip(ARM_JOINTS, q)},
                                               "joint7": float(finger), "joint8": float(finger)})
        out = {}
        for name, pts in self.parts:
            p = pts @ poses[name][:3, :3].T + poses[name][:3, 3] + base
            out.setdefault(name, []).append(p)
        return {k: np.concatenate(v) for k, v in out.items()}


def box_dist(points, low, high):
    low, high = np.asarray(low), np.asarray(high)
    d_out = np.sqrt((np.maximum(low - points, 0.) ** 2 + np.maximum(points - high, 0.) ** 2).sum(axis=1))
    d_in = np.minimum(points - low, high - points).min(axis=1)
    inside = np.all((points > low) & (points < high), axis=1)
    return float(np.where(inside, -d_in, d_out).min()) if len(points) else 9.


def capsule_dist(points, a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = np.clip(((points - a) @ ab) / max(ab @ ab, 1e-12), 0., 1.)
    return float((np.linalg.norm(points - (a + t[:, None] * ab), axis=1) - r).min()) if len(points) else 9.


# ---------------------------------------------------------------------------------------------------- scene
def frame(a, c):
    """Gripper rotation: columns (closing x_g, y_g, approach z_g)."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


def rot(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array(((0., -axis[2], axis[1]), (axis[2], 0., -axis[0]), (-axis[1], axis[0], 0.)))
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


class Layout:
    """``axis`` "x": bar's long axis along x, sling around it in the y-z plane, legs at y = +-S/2, hook plane y-z;
    ``axis`` "y": bar along y, sling in the x-z plane, legs at x = xs -+ S/2, hook plane x-z.
    ``throat``: horizontal unit vector the hook's opening faces."""

    def __init__(self, name, axis="x", xs=0.25, hook_z=0.20, throat=(0., -1., 0.), bar_len=BAR_L, bar_x0=None, hook_y=0.):
        self.name, self.axis, self.xs, self.hook_z = name, axis, xs, hook_z
        self.t = np.asarray(throat, float)
        self.v = np.array((0., 0., 1.))
        self.n = np.cross(self.t, self.v)
        h = BAR_S / 2
        if axis == "x":
            x0 = bar_x0 if bar_x0 is not None else xs - bar_len / 2
            self.bar = ((x0, -h, Z_BOT), (x0 + bar_len, h, Z_TOP))
            self.e = np.array((1., 0., 0.))
            sides = {"L": np.array((0., 1., 0.)), "R": np.array((0., -1., 0.))}
            self.blocks = [((x0 + .02, -.15, 0.), (x0 + .02 + .089, .15, Z_BOT)),
                           ((x0 + bar_len - .109, -.15, 0.), (x0 + bar_len - .02, .15, Z_BOT))]
            base_pt = np.array((xs, 0., 0.))
        else:
            self.bar = ((xs - h, -bar_len / 2, Z_BOT), (xs + h, bar_len / 2, Z_TOP))
            self.e = np.array((0., 1., 0.))
            sides = {"N": np.array((-1., 0., 0.)), "F": np.array((1., 0., 0.))}
            self.blocks = [((xs - .15, -bar_len / 2 + .02, 0.), (xs + .15, -bar_len / 2 + .109, Z_BOT)),
                           ((xs - .15, bar_len / 2 - .109, 0.), (xs + .15, bar_len / 2 - .02, Z_BOT))]
            base_pt = np.array((xs, 0., 0.))
        self.edges = {}
        for sname, n2 in sides.items():
            for level, z, n1 in (("top", Z_TOP, np.array((0., 0., 1.))), ("bot", Z_BOT, np.array((0., 0., -1.)))):
                self.edges[f"{sname}-{level}"] = dict(C=base_pt + n2 * h + np.array((0., 0., z)), n1=n1, n2=n2,
                                                      level=level)
        # hook (in its plane: s along t, v up), origin O at the bowl's centre
        self.O = np.array((xs, hook_y, hook_z + BOWL_R_IN))
        self.caps = self._hook_capsules()

    def hp(self, s, v):
        return self.O + s * self.t + v * self.v

    def _hook_capsules(self):
        Rc = BOWL_R_IN + HOOK_R
        caps = []
        # bowl from the back (s = -Rc, v = 0) under to the tip side (s = +Rc, v = 0)
        angs = np.radians(np.linspace(180, 360, 9))
        pts = [self.hp(Rc * math.cos(a), Rc * math.sin(a)) for a in angs]
        for a, b in zip(pts[:-1], pts[1:]):
            caps.append(("bowl", a, b, HOOK_R))
        caps.append(("tip", self.hp(Rc, 0.), self.hp(Rc - .004, .024), .008))
        # shank up the back to the eye, eye ring, shackle and the lifting point above
        caps.append(("shank", self.hp(-Rc, 0.), self.hp(-.012, .035), .010))
        caps.append(("shank", self.hp(-.012, .035), self.hp(0., .075), .009))
        ring = [self.hp(.018 * math.cos(a), .093 + .018 * math.sin(a)) for a in np.radians(np.arange(0, 361, 45))]
        for a, b in zip(ring[:-1], ring[1:]):
            caps.append(("eye", a, b, .006))
        caps.append(("shackle", self.hp(0., .105), self.hp(0., .170), .012))
        caps.append(("latch", self.hp(-.010, .030), self.hp(Rc - .010, .026), .003))
        return caps

    def hook_dist(self, pts, skip=()):
        return min(capsule_dist(pts, a, b, r) for name, a, b, r in self.caps if name not in skip)

    def bar_dist(self, pts):
        return min([box_dist(pts, *self.bar)] + [box_dist(pts, *b) for b in self.blocks])


# ---------------------------------------------------------------------------------------------------- IK
BASES = None
HULLS = None


def init():
    global BASES, HULLS
    from yam_arena import station

    top = float(station.TABLE_TOP)
    BASES = {a: np.asarray(station.BASES[a], float) - (0., 0., top) for a in ("left", "right")}
    HULLS = Hulls()


def solve(arm, p, R, seeds=None):
    from yam_arena import ik

    side = 1. if arm == "left" else -1.
    quat = ik.matrix_to_quat_xyzw(R)
    best = None
    pool = list(seeds or []) + [s * np.array((side, 1, 1, 1, side, side)) for s in SEEDS] \
        + [s * np.array((-side, 1, 1, 1, -side, -side)) for s in SEEDS]
    for seed in pool:
        q, err, rerr = ik.solve(p, quat, seed, BASES[arm], iters=120)
        margin = math.degrees(float(np.minimum(q - ik.LOWER, ik.UPPER - q).min()))
        ok = err < POS_TOL and rerr < ROT_TOL and margin >= MARGIN_DEG
        key = (ok, margin if ok else -err - rerr)
        if best is None or key > best[0]:
            best = (key, q, err, rerr, margin)
        if ok and margin > 12:
            break
    _, q, err, rerr, margin = best
    return q, bool(err < POS_TOL and rerr < ROT_TOL and margin >= MARGIN_DEG), err, rerr, margin


def worst_joint(q):
    from yam_arena import ik

    lo, hi = np.degrees(q - ik.LOWER), np.degrees(ik.UPPER - q)
    j = int(np.argmin(np.minimum(lo, hi)))
    return f"j{j + 1}:{'low' if lo[j] < hi[j] else 'high'}"


def evaluate(layout, arm, p, R, finger, tag, seeds=None, finger_skip_bar=False, hook_skip=()):
    q, ok, err, rerr, margin = solve(arm, p, R, seeds)
    row = {"tag": tag, "arm": arm, "tcp_mm": [round(1000 * v, 1) for v in p], "approach": [round(v, 3) for v in R[:, 2]],
           "close": [round(v, 3) for v in R[:, 0]], "ok": ok, "err_mm": round(1000 * err, 2),
           "rerr_deg": round(math.degrees(rerr), 2), "margin_deg": round(margin, 1), "q_deg": [round(math.degrees(v), 1) for v in q],
           "worst_joint": worst_joint(q)}
    if not ok:
        return row, None
    pts = HULLS.points(q, BASES[arm], finger)
    fingers = np.concatenate([pts[k] for k in Hulls.FINGERS])
    body = np.concatenate([v for k, v in pts.items() if k not in Hulls.FINGERS])
    hand = np.concatenate([v for k, v in pts.items() if k in ("link5", "gripper")])
    row.update(bar_body_mm=round(1000 * layout.bar_dist(body), 1), bar_fingers_mm=round(1000 * layout.bar_dist(fingers), 1),
               hook_body_mm=round(1000 * layout.hook_dist(body, hook_skip), 1),
               hook_fingers_mm=round(1000 * layout.hook_dist(fingers, hook_skip), 1),
               table_mm=round(1000 * float(np.concatenate(list(pts.values()))[:, 2].min()), 1),
               hand_z_mm=round(1000 * float(hand[:, 2].min()), 1))
    return row, (q, pts)


# ---------------------------------------------------------------------------------------------------- poses
def guard_candidates(layout):
    """(edge, family, arm, tcp, R, finger) for every edge."""
    from yam_arena.constants import FINGER_OPEN

    out = []
    for ename, E in layout.edges.items():
        C, n1, n2, e = E["C"], E["n1"], E["n2"], layout.e
        for arm in ("left", "right"):
            if E["level"] == "top":
                # T1: corner pinch from above: pads on the side flange's outside and the top flange's free edge
                mid = C + n2 * (GUARD_T - GUARD_FL) / 2
                tip = mid + np.array((0., 0., .001))
                for th in (-30, -15, 0, 15, 30, 45):
                    t = math.radians(th)
                    a = -n1 * math.cos(t) - n2 * math.sin(t)
                    c = n2 * math.cos(t) - n1 * math.sin(t)
                    for yaw in (-20, 0, 20):           # turn the hand about the face normal (pads stay on the flanges)
                        Rz = rot(n1, math.radians(yaw))
                        for sgn in (1, -1):
                            out.append((ename, "T1-pinch", arm, tip - PAD * (Rz @ a), frame(Rz @ a, sgn * (Rz @ c)),
                                        -(GUARD_FL + GUARD_T + .004) / 2, dict(th=th, yaw=yaw)))
                # T2: closed fingertips pushing the guard's outer corner along the inward bisector
                b = (n1 + n2) / math.sqrt(2)
                tip = C + b * (GUARD_T * math.sqrt(2) + .002)
                for ph in (-30, -15, 0, 15, 30):
                    a = rot(e, math.radians(ph)) @ (-b)
                    for roll in range(0, 180, 30):
                        c = rot(a, math.radians(roll)) @ e
                        out.append((ename, "T2-push", arm, tip - PAD * a, frame(a, c), 0., dict(ph=ph, roll=roll)))
            else:
                # B1: side flange held near its top edge from above, guard 10 mm off the face (inner pad between)
                pl = C + n2 * (B1_GAP + GUARD_T / 2)
                tip = pl + np.array((0., 0., GUARD_FL - .015))
                for th in (-30, -15, 0, 15, 30):
                    a = rot(n2, math.radians(th)) @ np.array((0., 0., -1.))
                    for sgn in (1, -1):
                        out.append((ename, "B1-hold", arm, tip - PAD * a, frame(a, sgn * n2), -.006, dict(th=th)))
                # B2: closed fingertips pushing the side flange onto the face, from outside and above
                tip = C + n2 * (GUARD_T + .001) + np.array((0., 0., .025))
                for ph in (0, 20, 40, 60):
                    a = -n2 * math.cos(math.radians(ph)) - np.array((0., 0., 1.)) * math.sin(math.radians(ph))
                    for roll in range(0, 180, 30):
                        c = rot(a, math.radians(roll)) @ e
                        out.append((ename, "B2-push", arm, tip - PAD * a, frame(a, c), 0., dict(ph=ph, roll=roll)))
        if END_PINCH:
            # E: pinch the guard by its two short ends (closing along the edge; needs a guard <= 90 mm long), hand
            # outside the side face, approach toward the face, tilted up/down; pads on the side flange's end strips
            for arm in ("left", "right"):
                tip = C + n2 * .0015 - n1 * .020
                for ph in (-30, -15, 0, 15, 30, 45):
                    a = rot(e, math.radians(ph)) @ (-n2)
                    for sgn in (1, -1):
                        out.append((ename, "E-endpinch", arm, tip - PAD * a, frame(a, sgn * e), -(.075 + .008) / 2, dict(ph=ph)))
    return out


def hang_waypoints(layout):
    """Fitting apex (s, v) and its base->apex direction angle alpha from the throat direction toward the back
    (alpha = 90: apex straight up)."""
    Rc = BOWL_R_IN + HOOK_R
    return [("W1-over-tip", (Rc + .012, .062)), ("W2-over-throat", (.004, .055)), ("W3-on-latch", (-.004, .020)),
            ("W4-seated", (0., -BOWL_R_IN + FIT_BAR_R))]


def hang_pose(layout, wp, alpha, gamma, sgn):
    """TCP (base bar), rotation: u = base->apex at ``alpha``; approach = u turned by ``gamma`` about w."""
    s, v = wp
    apex = layout.hp(s, v)
    al = math.radians(alpha)
    u = -layout.t * math.cos(al) + layout.v * math.sin(al)
    w = layout.n
    a = rot(w, math.radians(gamma)) @ u
    c = sgn * np.cross(w, a)
    return apex - FIT_H * u + GRIP_X * w, frame(a, c), apex


def steady_candidates(layout):
    out = []
    Rc = BOWL_R_IN + HOOK_R
    for vv in STEADY_V:
        # shank centreline at height vv (piecewise as in _hook_capsules; the eye ring above .075)
        if vv <= .035:
            s = -Rc + (-.012 + Rc) * vv / .035
        elif vv <= .075:
            s = -.012 + .012 * (vv - .035) / .040
        else:
            s = 0.
        p = layout.hp(s, vv)
        for arm in ("left", "right"):
            # closing along n (pads on the flat faces), approach anywhere in the hook plane but not from the throat
            for az in range(0, 360, 20):
                a = rot(layout.n, math.radians(az)) @ layout.t         # az = 180: a = -t, the hand on the throat side
                out.append(("n-close", arm, p, frame(a, layout.n), dict(v=vv, az=az)))
            # closing along t (pads on the back and the throat side of the shank), approach in the n-v plane
            if vv >= .040 and vv <= .080:
                for az in range(0, 360, 20):
                    a = rot(layout.t, math.radians(az)) @ layout.n
                    out.append(("t-close", arm, p, frame(a, layout.t), dict(v=vv, az=az)))
    return out


# ---------------------------------------------------------------------------------------------------- jobs
LAYOUT = None


def _guard_job(c):
    ename, fam, arm, p, R, finger, extra = c
    row, _ = evaluate(LAYOUT, arm, p, R, finger, f"{ename}/{fam}")
    row.update(edge=ename, family=fam, **extra)
    return row


def _hang_job(job):
    arm, alpha, gamma, sgn = job
    alphas = alpha if isinstance(alpha, (tuple, list)) else (alpha,) * 4
    rows, prev = [], None
    for (wname, wp), al in zip(hang_waypoints(LAYOUT), alphas):
        p, R, apex = hang_pose(LAYOUT, wp, al, gamma, sgn)
        row, res = evaluate(LAYOUT, arm, p, R, -.009, wname, seeds=[prev] if prev is not None else None,
                            hook_skip=("latch",))
        if res is not None:
            jump = float(np.degrees(np.abs(res[0] - prev)).max()) if prev is not None else 0.
            row["jump_deg"] = round(jump, 1)
            prev = res[0]
        rows.append(row)
    ok_all = all(r["ok"] for r in rows)
    return {"arm": arm, "alpha": alpha, "gamma": gamma, "sgn": sgn, "ok_all": ok_all, "waypoints": rows,
            "min_hook_body_mm": min((r.get("hook_body_mm", -99) for r in rows), default=-99) if ok_all else None,
            "min_bar_body_mm": min((r.get("bar_body_mm", -99) for r in rows), default=-99) if ok_all else None,
            "max_jump_deg": max((r.get("jump_deg", 0) for r in rows), default=0) if ok_all else None,
            "min_margin_deg": min(r["margin_deg"] for r in rows)}


def _steady_job(c):
    fam, arm, p, R, extra = c
    row, res = evaluate(LAYOUT, arm, p, R, -.010, f"steady/{fam}")
    row.update(family=fam, **extra)
    return row


def _apply_env():
    import os
    global FIT_H, B1_GAP, STEADY_V
    FIT_H = float(os.environ.get("SLING_FIT_H", FIT_H))
    B1_GAP = float(os.environ.get("SLING_B1_GAP", B1_GAP))
    global END_PINCH, GRIP_X
    END_PINCH = bool(os.environ.get("SLING_END_PINCH"))
    GRIP_X = float(os.environ.get("SLING_GRIP_X", 0.))
    if os.environ.get("SLING_STEADY_V"):
        STEADY_V = tuple(float(v) for v in os.environ["SLING_STEADY_V"].split(","))


def _init(name):
    global LAYOUT
    _apply_env()
    init()
    LAYOUT = LAYOUTS[name]()


LAYOUTS = {
    "X": lambda: Layout("X", axis="x", xs=0.25, hook_z=0.20, throat=(0., -1., 0.)),
    "X-h18": lambda: Layout("X-h18", axis="x", xs=0.25, hook_z=0.18, throat=(0., -1., 0.)),
    "X-h22": lambda: Layout("X-h22", axis="x", xs=0.25, hook_z=0.22, throat=(0., -1., 0.)),
    "X-x22": lambda: Layout("X-x22", axis="x", xs=0.22, hook_z=0.20, throat=(0., -1., 0.)),
    "X-x28": lambda: Layout("X-x28", axis="x", xs=0.28, hook_z=0.20, throat=(0., -1., 0.)),
    "Xf": lambda: Layout("Xf", axis="x", xs=0.25, hook_z=0.20, throat=(-1., 0., 0.)),
    "Y": lambda: Layout("Y", axis="y", xs=0.25, hook_z=0.20, throat=(-1., 0., 0.)),
    "Y-x22": lambda: Layout("Y-x22", axis="y", xs=0.22, hook_z=0.20, throat=(-1., 0., 0.)),
}


def pair_check(layout, hang_rows, steady_rows, top=6):
    """For the best hanging paths and steadying grips with different arms, the two arms' clearance at every
    waypoint, and the steadying hand's clearance from the fitting's apex path."""
    from scipy.spatial import cKDTree

    out = []
    hangs = sorted([h for h in hang_rows if h["ok_all"]],
                   key=lambda h: (min(h["min_hook_body_mm"], 20), h["min_margin_deg"]), reverse=True)
    for arm in ("left", "right"):
        H = [h for h in hangs if h["arm"] == arm][:top]
        S = sorted([s for s in steady_rows if s["ok"] and s["arm"] != arm and s["hook_body_mm"] > 0],
                   key=lambda s: (min(s["hook_body_mm"], 15), s["margin_deg"]), reverse=True)[:top * 2]
        for h in H:
            hp = [HULLS.points(np.radians(np.array(w["q_deg"])), BASES[arm], -.009) for w in h["waypoints"]]
            apex = [hang_pose(layout, wp, h["alpha"], h["gamma"], h["sgn"])[2] for _, wp in hang_waypoints(layout)]
            for s in S:
                sp = HULLS.points(np.radians(np.array(s["q_deg"])), BASES[s["arm"]], -.010)
                allp = np.concatenate(list(sp.values()))
                tree = cKDTree(allp)
                d = min(float(tree.query(np.concatenate(list(p.values())))[0].min()) for p in hp)
                # the fitting (a 64 x 55 mm triangle) swept along the apex path: sample its outline
                fit = []
                for (_, wp), ap in zip(hang_waypoints(layout), apex):
                    tcp, R, _ = hang_pose(layout, wp, h["alpha"], h["gamma"], h["sgn"])
                    w = layout.n
                    for k in np.linspace(0, 1, 12):
                        fit.append(tcp + (2 * k - 1) * .032 * w)                       # base bar
                        for sd in (-1, 1):                                            # the two sides
                            fit.append(tcp + sd * .032 * w + k * (ap + sd * .010 * w - tcp - sd * .032 * w))
                        fit.append(ap + (2 * k - 1) * .010 * w)                        # apex bar
                fd = float(tree.query(np.array(fit))[0].min()) - FIT_BAR_R
                out.append({"hang_arm": arm, "alpha": h["alpha"], "gamma": h["gamma"], "sgn": h["sgn"],
                            "steady_arm": s["arm"], "steady": {k: s[k] for k in ("family", "v", "az", "tcp_mm", "approach", "margin_deg", "hook_body_mm", "bar_body_mm")},
                            "arm_arm_mm": round(1000 * d, 1), "steady_to_fitting_mm": round(1000 * fd, 1),
                            "hang_min_hook_body_mm": h["min_hook_body_mm"], "hang_min_margin_deg": h["min_margin_deg"]})
    return sorted(out, key=lambda r: min(r["arm_arm_mm"], r["steady_to_fitting_mm"], 30), reverse=True)


def main():
    import os
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", default="X", choices=sorted(LAYOUTS))
    ap.add_argument("--jobs", type=int, default=28)
    ap.add_argument("--only", default="guards,hang,steady,pairs")
    ap.add_argument("--fit-h", type=float, default=FIT_H, help="TCP to the fitting's bearing point (m)")
    ap.add_argument("--b1-gap", type=float, default=B1_GAP)
    ap.add_argument("--steady-v", default=None, help="comma list of steadying heights (m above the bowl centre)")
    ap.add_argument("--end-pinch", action="store_true")
    ap.add_argument("--grip-x", type=float, default=0.)
    args = ap.parse_args()
    os.environ["SLING_GRIP_X"] = str(args.grip_x)
    if args.end_pinch:
        os.environ["SLING_END_PINCH"] = "1"
    if args.steady_v:
        os.environ["SLING_STEADY_V"] = args.steady_v
    os.environ["SLING_FIT_H"], os.environ["SLING_B1_GAP"] = str(args.fit_h), str(args.b1_gap)
    _apply_env()
    from multiprocessing import Pool

    t0 = time.time()
    _init(args.layout)
    only = set(args.only.split(","))
    report = {"layout": args.layout, "fit_h": FIT_H, "b1_gap": B1_GAP, "grip_x": GRIP_X, "bar": [[round(1000 * v, 1) for v in c] for c in LAYOUT.bar],
              "blocks": [[[round(1000 * v, 1) for v in c] for c in b] for b in LAYOUT.blocks],
              "hook_bowl_bottom_mm": [round(1000 * v, 1) for v in LAYOUT.hp(0, -BOWL_R_IN)],
              "throat": LAYOUT.t.tolist(), "edges": {k: [round(1000 * v, 1) for v in E["C"]] for k, E in LAYOUT.edges.items()}}
    with Pool(args.jobs, initializer=_init, initargs=(args.layout,)) as pool:
        if "guards" in only:
            rows = pool.map(_guard_job, guard_candidates(LAYOUT), chunksize=4)
            best = {}
            for r in rows:
                k = (r["edge"], r["family"], r["arm"])
                clr = min(r.get("bar_body_mm", -99), r.get("hook_body_mm", -99))
                key = (r["ok"], clr >= 1000 * CLEAR_GOAL, min(clr, 40) + r["margin_deg"] / 3)
                if k not in best or key > best[k][0]:
                    best[k] = (key, r)
            report["guards"] = {f"{k[0]}|{k[1]}|{k[2]}": dict(v[1], n_ok=sum(1 for r in rows if (r["edge"], r["family"], r["arm"]) == k and r["ok"]),
                                                              n_clear=sum(1 for r in rows if (r["edge"], r["family"], r["arm"]) == k and r["ok"] and r["bar_body_mm"] >= 20 and r["hook_body_mm"] >= 20),
                                                              n=sum(1 for r in rows if (r["edge"], r["family"], r["arm"]) == k))
                                for k, v in sorted(best.items())}
            print(f"guards done {time.time() - t0:.0f}s", file=sys.stderr, flush=True)
        if "hang2" in only:          # flatter fittings over the tip, tilting down into the bowl
            paths = [(0, 0, 30, 60), (15, 15, 30, 60), (15, 30, 45, 60), (20, 20, 40, 60), (30, 30, 45, 60),
                     (30, 30, 60, 75), (15, 15, 45, 75), (0, 15, 30, 45), (30, 30, 30, 30), (15, 15, 15, 15)]
            jobs = [(arm, al, ga, sg) for arm in ("right", "left") for al in paths for ga in range(-90, 91, 15) for sg in (1, -1)]
            hang = pool.map(_hang_job, jobs, chunksize=2)
            ok = [h for h in hang if h["ok_all"]]
            for h in ok:
                h["min_hook_fingers_mm"] = min(w.get("hook_fingers_mm", -99) for w in h["waypoints"])
            report["hang2_summary"] = {arm: {"paths": sum(1 for h in hang if h["arm"] == arm), "ok": sum(1 for h in ok if h["arm"] == arm),
                                             "ok_fingers_clear5": sum(1 for h in ok if h["arm"] == arm and h["min_hook_fingers_mm"] >= 5 and h["min_hook_body_mm"] >= 20),
                                             "ok_fingers_clear10": sum(1 for h in ok if h["arm"] == arm and h["min_hook_fingers_mm"] >= 10 and h["min_hook_body_mm"] >= 20)}
                                       for arm in ("left", "right")}
            report["hang2_best"] = sorted(ok, key=lambda h: (min(h["min_hook_fingers_mm"], 15), min(h["min_hook_body_mm"], 25), h["min_margin_deg"]), reverse=True)[:10]
            print(f"hang2 done {time.time() - t0:.0f}s", file=sys.stderr, flush=True)
        if "hang" in only or "pairs" in only:
            jobs = [(arm, al, ga, sg) for arm in ("left", "right") for al in (45, 60, 75, 90, 105)
                    for ga in range(-90, 91, 15) for sg in (1, -1)]
            hang = pool.map(_hang_job, jobs, chunksize=2)
            report["hang_summary"] = {arm: {"paths": sum(1 for h in hang if h["arm"] == arm),
                                            "ok_paths": sum(1 for h in hang if h["arm"] == arm and h["ok_all"]),
                                            "ok_clear_paths": sum(1 for h in hang if h["arm"] == arm and h["ok_all"] and h["min_hook_body_mm"] >= 20 and h["min_bar_body_mm"] >= 20)}
                                      for arm in ("left", "right")}
            report["hang_best"] = sorted([h for h in hang if h["ok_all"]], key=lambda h: (min(h["min_hook_body_mm"], 25), min(h["min_bar_body_mm"], 25), h["min_margin_deg"]), reverse=True)[:8]
            per_wp = {}
            for h in hang:
                for w in h["waypoints"]:
                    k = (h["arm"], w["tag"])
                    per_wp.setdefault(k, [0, 0])
                    per_wp[k][0] += 1
                    per_wp[k][1] += int(w["ok"])
            report["hang_waypoint_ok"] = {f"{a}|{t}": f"{v[1]}/{v[0]}" for (a, t), v in sorted(per_wp.items())}
            print(f"hang done {time.time() - t0:.0f}s", file=sys.stderr, flush=True)
        if "steady" in only or "pairs" in only:
            steady = pool.map(_steady_job, steady_candidates(LAYOUT), chunksize=4)
            report["steady_summary"] = {arm: {"n": sum(1 for s in steady if s["arm"] == arm),
                                              "ok": sum(1 for s in steady if s["arm"] == arm and s["ok"]),
                                              "ok_body_clear5": sum(1 for s in steady if s["arm"] == arm and s["ok"] and s["hook_body_mm"] >= 5)}
                                        for arm in ("left", "right")}
            report["steady_best"] = {arm: sorted([s for s in steady if s["arm"] == arm and s["ok"]], key=lambda s: (min(s["hook_body_mm"], 15), s["margin_deg"]), reverse=True)[:6]
                                     for arm in ("left", "right")}
            print(f"steady done {time.time() - t0:.0f}s", file=sys.stderr, flush=True)
    if "pairs" in only:
        report["pairs"] = pair_check(LAYOUT, hang, steady)[:12]
        print(f"pairs done {time.time() - t0:.0f}s", file=sys.stderr, flush=True)
    report["seconds"] = round(time.time() - t0, 1)
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
