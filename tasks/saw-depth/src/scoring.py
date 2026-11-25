"""Scorer for the saw-depth task (engine-independent, numpy only).

Inputs every control step: the saw's three body poses (shoe, upper assembly, depth lever) and speeds, the lever angle,
the depth clamp state, and the four finger pad tips. Read at the end of the trial (person check in brackets):

  1.0  the lowest tooth 1/8 to 1/4 in (3.2 to 6.4 mm) below the board's underside (steel rule against the board's
       edge face), the depth lever locked (lever down; the clamp holds), the saw's shoe resting flat on the board (no
       rock, no gap), every fingertip 5 cm or more from the saw, saw still for 1 s. Ends the trial.
  0.4  as 1.0 but the tooth 1 to 3.2 mm or 6.4 to 9 mm below the underside.
  0    otherwise: lever released (the motor then drops to full depth under its own weight), tooth above the
       underside or more than 9 mm below it, shoe not flat on the board, a hand on the saw; or at any time the saw
       tipped (shoe more than 10 deg off flat) or fell off the board (vetoes, latched).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import layout as L
import numpy as np

BAND_FULL = (0.003175, 0.00635)  # 1/8 to 1/4 in below the stock (the common trade rule)
BAND_PART = ((0.001, 0.003175), (0.00635, 0.009))
PART_SCORE = 0.4
FLAT_TILT_DEG = 1.0  # shoe flat on the board
FLAT_DZ_M = 0.0015  # shoe bottom within 1.5 mm of the board top
TIP_VETO_DEG = 10.0
FALL_VETO_M = 0.010  # shoe bottom 10 mm below the board top: off the board
CLEAR_M = 0.050  # hands 5 cm or more off the saw
STILL_V_M_S = 0.003
STILL_W_DEG_S = 1.5
HOLD_STEPS = 20  # 1 s at 20 Hz


def tilt_deg(rot):
    """Angle between the body's w axis and vertical."""
    return math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))


def seg_dist(p, a, b):
    p, a, b = (np.asarray(x, float) for x in (p, a, b))
    d = b - a
    t = float(np.clip((p - a) @ d / max(d @ d, 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * d)))


def saw_segments(up, ur, kp, kr):
    """Capsules (a, b, radius) covering the saw's upper assembly and lever (station frame), for the hands-clear
    check."""
    P = L.PIVOT

    def U(x):
        return up + ur @ (np.asarray(x, float) - P)

    arb = L.ARBOR0
    tip = kp + kr @ L.lever_tip_k()
    return [
        (U(L.H_F), U(L.H_R), 0.016),
        (U(L.H_R), U(L.H_B), 0.014),
        (U(L.H_B), U(L.H_M), 0.014),
        (U(L.MOTOR_C + [0, L.MOTOR_V[0], 0]), U(L.MOTOR_C + [0, L.CAP_V[1], 0]), L.MOTOR_R),
        (U(arb + [-0.050, 0.0, 0.0]), U(arb + [0.050, 0.0, 0.0]), L.BLADE_R - 0.045),
        (U(arb + [0.0, 0.0, 0.0]), U(arb + [0.0, 0.0, 0.060]), 0.06),
        (kp, tip, 0.010),
    ]


def clearance(tips, up, ur, kp, kr):
    segs = saw_segments(up, ur, kp, kr)
    return float(min(seg_dist(t, a, b) - r for t in tips for a, b, r in segs)) if tips else 1.0


@dataclass
class Measure:
    proj_mm: float  # lowest tooth below the board's underside (mm; negative = above)
    depth_mm: float  # blade below the shoe's bottom (mm), from the depth angle
    theta_deg: float
    phi_deg: float
    locked: bool
    shoe_tilt_deg: float
    shoe_dz_mm: float  # shoe bottom above the board top (mm)
    flat: bool
    clear_mm: float
    still: bool


def measure(poses, speeds, theta, phi, clamp_on, t_board, tips):
    """poses: {'shoe'|'upper'|'lever': (pos, rot)}; speeds: (max linear m/s, max angular deg/s) over the saw's
    bodies; theta, phi in rad; clamp_on: the clamp is engaged; t_board: board thickness (m)."""
    sp, sr = poses["shoe"]
    up, ur = poses["upper"]
    kp, kr = poses["lever"]
    proj = L.projection_below(up, ur)
    tl = tilt_deg(sr)
    dz = float(sp[2] - L.board_top(t_board))
    clr = clearance(tips, up, ur, kp, kr)
    return Measure(
        proj_mm=proj * 1e3,
        depth_mm=L.depth_of(theta) * 1e3,
        theta_deg=math.degrees(theta),
        phi_deg=math.degrees(phi),
        locked=bool(clamp_on and phi < L.LEVER_PHI_ENGAGE),
        shoe_tilt_deg=tl,
        shoe_dz_mm=dz * 1e3,
        flat=bool(tl <= FLAT_TILT_DEG and abs(dz) <= FLAT_DZ_M),
        clear_mm=clr * 1e3,
        still=bool(speeds[0] <= STILL_V_M_S and speeds[1] <= STILL_W_DEG_S),
    )


def band_level(proj_m):
    if BAND_FULL[0] <= proj_m <= BAND_FULL[1]:
        return 1.0
    if any(lo <= proj_m < hi for lo, hi in BAND_PART) or proj_m == BAND_PART[1][1]:
        return PART_SCORE
    return 0.0


def instant_level(m: Measure):
    """Level of the current state ignoring the 1 s hold and the vetoes."""
    if not (m.locked and m.flat and m.clear_mm >= CLEAR_M * 1e3):
        return 0.0
    return band_level(m.proj_mm / 1e3)


@dataclass
class Scorer:
    tipped: bool = False
    fell: bool = False
    hold: int = 0
    level: float = 0.0
    score: float = 0.0
    last: dict = field(default_factory=dict)
    peak: dict = field(default_factory=lambda: dict(tilt_deg=0.0, drop_mm=0.0))

    @property
    def veto(self):
        return self.tipped or self.fell

    def step(self, m: Measure):
        self.peak["tilt_deg"] = max(self.peak["tilt_deg"], m.shoe_tilt_deg)
        self.peak["drop_mm"] = max(self.peak["drop_mm"], -m.shoe_dz_mm)
        if m.shoe_tilt_deg > TIP_VETO_DEG:
            self.tipped = True
        if m.shoe_dz_mm < -FALL_VETO_M * 1e3:
            self.fell = True
        lv = 0.0 if self.veto else instant_level(m)
        if lv > 0 and m.still:
            self.hold += 1
        else:
            self.hold = 0
        # partial and full credit both need the saw at rest, hands off, for 1 s
        self.level = lv if self.hold >= HOLD_STEPS else 0.0
        self.score = self.level
        self.last = dict(
            measure={k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.__dict__.items()},
            instant=lv,
            hold=self.hold,
            level=self.level,
            veto=dict(tipped=self.tipped, fell=self.fell),
            peak={k: round(v, 3) for k, v in self.peak.items()},
        )
        return self.level
