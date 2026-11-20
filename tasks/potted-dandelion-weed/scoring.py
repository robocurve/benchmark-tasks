"""Scorer for the potted dandelion: "pull it out with its whole root, onto the tarp; pot left standing".

State read each control step (20 Hz) from the bodies' measured poses:
  * root intact: the lower taproot's top and the upper taproot's neck still coincide (gap < 3 mm, bend
    < 25 deg). Once the neck has snapped (the lower root left in the pot) the trial is a failure and stays 0:
    the weed grows back from it.
  * loosened: the lower root has risen 5 mm or more relative to the pot (its bond to the soil has let go).
  * out: the lowest point of the taproot is 3 mm or more above the pot's soil surface, or outside the pot.
  * on tarp: crown and root tip inside the tarp's outline (5 mm in from its edge), below 6 cm over the bench,
    weed still (< 3 cm/s), no finger pad within 30 mm of the crown.
  * pot standing: upright within 10 deg and within 40 mm of where it started.
  * pot lifted: the pot has risen more than 10 mm off the bench at any time. The root holds harder (13 N) than
    the pot weighs (7.8 N), so pulling the weed without holding the pot down lifts the pot with it. That is a
    failure and stays 0 (the instruction says to leave the pot standing where it is).
Ladder (primary score):
  0.00  nothing loosened, or the root snapped at any time, or the pot was lifted off the bench at any time
  0.25  root loosened from the soil, intact
  0.50  whole weed out of the soil, root intact
  1.00  out, intact, lying on the tarp and let go, with the pot standing where it was
Real trial: the tarp's outline and the pot's start ring are taped; a 10 mm lift of the pot is seen on video
against a 10 mm mark on a ruler standing beside it; root intact and out are seen on video
(the root is joined by magnets, see README), the pot's tilt with a square, its offset with a ruler.
"""
from __future__ import annotations

import math

import numpy as np

import weed as W

GAP_SNAP = 0.003
BEND_SNAP = math.radians(25.0)
LOOSE = 0.005
OUT = 0.003
TARP_MARGIN = 0.005
TARP_Z = 0.06
STILL = 0.03
CLEAR = 0.030
POT_TILT = math.radians(10.0)
POT_MOVE = 0.040
POT_LIFT = 0.010


def _quat_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _field(o, name):
    return o[name] if isinstance(o, dict) else getattr(o, name)


def _box_pts(box):
    c, R, h = (np.asarray(x, float) for x in box)
    pts = [c]
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                pts.append(c + R @ (h * np.array([sx, sy, sz])))
    return np.asarray(pts)


def _frame(pose):
    p = np.asarray(pose, float)
    return p[:3], _quat_mat(p[3:7])


class TrialScorer:
    def __init__(self):
        self.step = 0
        self.snapped = False
        self.loosened = False
        self.root0 = None
        self.pot_z0 = None
        self.pot_lifted = False

    def update(self, step, objects, root_poses, robot, mechanics=None, **_):
        if step != self.step + 1:
            raise ValueError("scorer needs one update per control step")
        self.step = step
        wp, wR = _frame(root_poses["weed"])
        rp, rR = _frame(root_poses["root"])
        pp, pR = _frame(root_poses["pot"])
        neck_w = wp + wR @ np.array([0.0, 0.0, W.NECK_REL])
        neck_r = rp + rR @ np.array([0.0, 0.0, W.LO_ROOT_L / 2])
        gap = float(np.linalg.norm(neck_w - neck_r))
        bend = math.acos(max(-1.0, min(1.0, float(wR[:, 2] @ rR[:, 2]))))
        if gap > GAP_SNAP or bend > BEND_SNAP:
            self.snapped = True
        # root position relative to the pot
        rel = pR.T @ (rp - pp)
        if self.root0 is None:
            self.root0 = rel.copy()
        if rel[2] - self.root0[2] >= LOOSE:
            self.loosened = True
        tip = rp + rR @ np.array([0.0, 0.0, -W.LO_ROOT_L / 2 - 0.002])
        tip_rel = pR.T @ (tip - pp)
        soil_rel = W.SOIL_REL - 0.003            # soil surface in the pot frame (origin 3 mm over the bench)
        in_pot = math.hypot(tip_rel[0], tip_rel[1]) < W.R_TOP and tip_rel[2] < W.POT_H
        out = bool(tip_rel[2] > soil_rel + OUT or not in_pot)
        # pot standing, never lifted
        if self.pot_z0 is None:
            self.pot_z0 = float(pp[2])
        if float(pp[2]) - self.pot_z0 > POT_LIFT:
            self.pot_lifted = True
        tilt = math.acos(max(-1.0, min(1.0, float(pR[2, 2]))))
        moved = math.hypot(pp[0] - W.POT_C[0], pp[1] - W.POT_C[1])
        standing = bool(tilt < POT_TILT and moved < POT_MOVE and pp[2] > W.TOP - 0.01)
        # on the tarp, let go
        hx, hy = W.TARP_S[0] / 2 - TARP_MARGIN, W.TARP_S[1] / 2 - TARP_MARGIN

        def on(p):
            return abs(p[0] - W.TARP_C[0]) < hx and abs(p[1] - W.TARP_C[1]) < hy and p[2] < W.TOP + TARP_Z

        v = float(np.linalg.norm(np.asarray(_field(objects["weed"], "lin_vel"), float)))
        pad_gap = 1.0
        if mechanics and mechanics.get("pad_boxes"):
            pads = np.vstack([_box_pts(b) for b in mechanics["pad_boxes"]])
            pad_gap = float(np.min(np.linalg.norm(pads - wp, axis=1)))
        on_tarp = bool(on(wp) and on(tip) and v < STILL and pad_gap >= CLEAR)
        score = 0.0
        if not self.snapped and not self.pot_lifted:
            if self.loosened:
                score = 0.25
            if self.loosened and out:
                score = 0.5
        success = bool(not self.snapped and not self.pot_lifted and out and on_tarp and standing)
        if success:
            score = 1.0
        return {"score": score, "success": success, "snapped": self.snapped, "pot_lifted": self.pot_lifted, "loosened": self.loosened,
                "out": out, "on_tarp": on_tarp, "pot_standing": standing,
                "root_rise_mm": round(1000 * float(rel[2] - self.root0[2]), 1), "neck_gap_mm": round(1000 * gap, 2),
                "neck_bend_deg": round(math.degrees(bend), 1), "pot_tilt_deg": round(math.degrees(tilt), 1),
                "pot_moved_mm": round(1000 * moved, 1),
                "pot_rise_mm": round(1000 * (float(pp[2]) - self.pot_z0), 1), "pad_gap_mm": round(1000 * pad_gap, 1),
                "weed_z": round(float(wp[2]), 4)}
