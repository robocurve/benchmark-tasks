"""Scorer for the recessed-trim task (engine-independent, numpy only).

Inputs every control step: the trim's pose and velocity, the four spring legs' poses, the housing's bracket rotation
phi, and whether the robot touches the trim or its springs (contact force from the per-physics-step monitor).

A spring counts as captured in its bracket when both of its legs pass through that bracket's slot (each leg's centre
line crosses the tab's mid-plane inside the slot outline) and both leg tips are above the tab's top face.
A person checks the same thing by looking up into the housing (both wire ends above the bracket, through the slot)
and by letting go of the trim (it stays up).

Levels (score at the end of the trial; every level needs the robot off the trim and its springs):
  1.0   both springs captured and the flange flat on the ceiling: the gap between the flange's top face and the ceiling
        at 12 points round the flange (radius 92 mm) is at most 2 mm everywhere; trim still for 1 s. Ends the trial.
  0.5   both springs captured, flange not flush.
  0.25  one spring captured.
  0     no spring captured (trim on the carton, dropped, or held by the robot).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import layout as L
import numpy as np

TIP_ABOVE_M = 0.0  # leg tip (centre line end) above the tab's top face: a hook resting on the tab is 0.8 mm above
SLOT_MARGIN_M = 0.0  # crossing point inside the slot outline (the wire itself keeps it 0.8 mm inside)
FLUSH_GAP_M = 0.002  # flange-to-ceiling gap at every check point
STILL_V_M_S = 0.003
STILL_W_DEG_S = 3.0
HOLD_STEPS = 20  # 1 s at 20 Hz
ROBOT_TOUCH_N = 0.05


def housing_to_local(p_world, phi_deg, beta_deg):
    """World point -> (r, u, z) in bracket coordinates (radial from the housing axis along the bracket direction,
    tangential, height above the ceiling underside)."""
    d = np.asarray(p_world, float) - L.H
    rad, tan = L.bracket_frame(beta_deg)
    return float(d @ rad), float(d @ tan), float(d[2])


def leg_line(leg_pos, leg_rot):
    """(root, tip) of a leg's centre line in the world."""
    return np.asarray(leg_pos, float), np.asarray(leg_pos, float) + np.asarray(leg_rot) @ L.leg_tip_local()


def leg_through_slot(leg_pos, leg_rot, beta_deg):
    """(inside_slot, tip_above_m, (r, u)) of one leg against the bracket at angle beta."""
    a, b = leg_line(leg_pos, leg_rot)
    zm = L.H[2] + L.TAB_Z - L.TAB_T / 2
    tip_above = float(b[2] - (L.H[2] + L.TAB_Z))
    if (a[2] - zm) * (b[2] - zm) > 0:
        return False, tip_above, None
    t = (zm - a[2]) / (b[2] - a[2])
    p = a + t * (b - a)
    r, u, _ = housing_to_local(p, 0.0, beta_deg)
    ok = (
        L.SLOT_R[0] + SLOT_MARGIN_M <= r <= L.SLOT_R[1] - SLOT_MARGIN_M
        and abs(u) <= L.SLOT_U - SLOT_MARGIN_M
    )
    return ok, tip_above, (r, u)


def spring_captured(legs, spring, phi_deg):
    """legs: {leg name: (pos, rot)}. True when both legs of ``spring`` pass through its bracket slot with their tips
    TIP_ABOVE_M above the tab."""
    beta = L.bracket_angles(phi_deg)[spring]
    out = []
    for side in "pm":
        ok, above, ru = leg_through_slot(*legs[spring + side], beta)
        out.append(ok and above >= TIP_ABOVE_M)
    return all(out)


def flange_gaps(pos, rot):
    """Gap (m) between the flange's top face and the ceiling underside at the 12 check points (negative = above)."""
    return [float(L.H[2] - (pos + rot @ p)[2]) for p in L.flange_points()]


def tilt_deg(rot):
    return math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))


@dataclass
class Measure:
    captured: dict
    gaps_mm: list
    max_gap_mm: float
    tilt_deg: float
    trim_z_mm: float
    touched: bool
    leg_theta_deg: dict = field(default_factory=dict)

    def as_dict(self):
        return dict(
            captured=self.captured,
            max_gap_mm=round(self.max_gap_mm, 3),
            min_gap_mm=round(min(self.gaps_mm), 3),
            tilt_deg=round(self.tilt_deg, 3),
            trim_z_mm=round(self.trim_z_mm, 2),
            touched=self.touched,
            leg_theta_deg={k: round(v, 2) for k, v in self.leg_theta_deg.items()},
        )


def measure(pos, rot, legs, phi_deg, robot_force_n, leg_theta=None):
    gaps = flange_gaps(pos, rot)
    return Measure(
        captured={s: spring_captured(legs, s, phi_deg) for s in ("A", "B")},
        gaps_mm=[g * 1e3 for g in gaps],
        max_gap_mm=max(gaps) * 1e3,
        tilt_deg=tilt_deg(rot),
        trim_z_mm=float(pos[2] * 1e3),
        touched=robot_force_n > ROBOT_TOUCH_N,
        leg_theta_deg={k: math.degrees(v) for k, v in (leg_theta or {}).items()},
    )


def level_of(m: Measure):
    """Instantaneous level (before the 1 s still hold that 1.0 needs)."""
    if m.touched:
        return 0.0
    n = sum(1 for v in m.captured.values() if v)
    if n == 2 and m.max_gap_mm <= FLUSH_GAP_M * 1e3:
        return 1.0
    if n == 2:
        return 0.5
    if n == 1:
        return 0.25
    return 0.0


class Scorer:
    def __init__(self):
        self.hold = 0
        self.level = 0.0
        self.last = {}
        self.best = 0.0

    def step(self, m: Measure, lin_v, ang_v_deg):
        raw = level_of(m)
        still = lin_v < STILL_V_M_S and ang_v_deg < STILL_W_DEG_S
        if raw >= 1.0 and still:
            self.hold += 1
        elif raw < 1.0:
            self.hold = 0
        lvl = raw
        if raw >= 1.0 and self.hold < HOLD_STEPS:
            lvl = 0.5  # flush but not yet held still for 1 s
        self.level = lvl
        self.best = max(self.best, lvl)
        self.last = dict(
            measure=m.as_dict(), raw=raw, hold=self.hold, level=lvl, still=bool(still), speed=round(lin_v, 4)
        )
        return lvl
