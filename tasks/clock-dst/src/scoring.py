"""Scoring of the clock-dst task: pure functions of measured state (no simulator imports), unit-tested in
``../tests/test_scoring.py``.

Person check (in the README): read the time on the face (photo), see that the clock hangs on its nail on its own (the
hands are off it and it stays), and check it is level (the 12-6 line within 5 deg of vertical, e.g. a phone level on the
bezel top). In simulation: the knob joint angle gives the time (the minute hand turns with the knob 1:1, the hour hand
1:12), the clock pose gives the tilt, the keyhole/nail geometry plus the nail's contact force give "hanging".
"""

from __future__ import annotations

import math

import numpy as np

import layout as L

G = 9.81
HOLD_S = 1.0  # the end state must hold (released, still, hung) for 1 s
DT = 0.05  # control step (20 Hz)
LEVEL_DEG = 5.0
FLAT_DEG = 6.0  # back flat on the wall (face normal within 6 deg of the wall normal)
SUPPORT_FRAC = 0.5  # the nail carries at least half the clock's weight
STILL_MPS = 0.005
STILL_DEGPS = 3.0
TOUCH_N = 0.05  # any robot contact force above this means "not released"
FALL_MPS = 0.45  # downward speed of a free fall from about 10 mm: dropped
# minutes off the instruction's time -> level (when hung, level, released and still)
TIERS = ((2.0, 1.0), (5.0, 0.5))


def tilt_in_wall_plane(rot):
    """Angle (deg) of the clock's 12 o'clock direction from straight up, measured in the clock's own plane."""
    y = rot[:, 1]
    zk = rot[:, 2]
    up = np.array([0.0, 0.0, 1.0])
    up_p = up - zk * float(up @ zk)
    n = float(np.linalg.norm(up_p))
    if n < 1e-9:
        return 90.0
    up_p /= n
    c = float(np.clip(y @ up_p, -1.0, 1.0))
    return math.degrees(math.acos(c))


def flat_on_wall_deg(rot):
    """Angle (deg) between the face normal z_K and the wall normal (-x)."""
    c = float(np.clip(rot[:, 2] @ np.array([-1.0, 0.0, 0.0]), -1.0, 1.0))
    return math.degrees(math.acos(c))


FLUSH_MM = 3.0  # the clock's back at the keyhole within 3 mm of the wall


def nail_in_keyhole(nail_k, head_k):
    """The clock hangs properly by its keyhole: the shank is up in the 4 mm slot, above the top of the 11 mm entry hole
    (so the 6.8 mm head is held by the slot's edges, not resting in the hole), and the back of the clock at the keyhole
    is within FLUSH_MM of the wall (head well inside the case). nail_k / head_k: nail axis point at the wall face and
    head underside point, in the clock frame K (the wall-face point has z_K = minus the back's gap to the wall)."""
    x, y = float(nail_k[0]), float(nail_k[1])
    in_slot = abs(x) < L.KEY_SLOT_W / 2 + 0.0005 and L.KEY_HOLE_C[1] + L.KEY_HOLE_R + 0.0005 < y < L.KEY_SLOT_TOP + 0.0005
    flush = float(nail_k[2]) > -FLUSH_MM / 1000.0
    head_behind = float(head_k[2]) > L.BACK_T
    return bool(in_slot and flush and head_behind)


def time_error_min(shown_min, target_min=L.TARGET_MIN):
    """Signed minutes from the target on a 12 h dial (in -360..360)."""
    d = (shown_min - target_min) % 720.0
    return d - 720.0 if d > 360.0 else d


def tier(err_min):
    e = abs(err_min)
    for lim, lvl in TIERS:
        if e <= lim + 1e-9:
            return lvl
    return 0.0


def measure(pos, rot, nail_point, shown_min, nail_n, robot_n, lin_speed, ang_speed_deg):
    """One control step's measurements (dict) from the clock pose (K origin, rotation), the setup's nail point, the
    shown time, the nail's contact force on the clock (N), the largest robot contact force on the clock (N) and the
    clock's speeds."""
    nk, hk = L.nail_in_k(pos, rot, nail_point)
    err = time_error_min(shown_min)
    return dict(
        shown=round(float(shown_min), 3),
        shown_hm=L.fmt(shown_min),
        err_min=round(float(err), 3),
        tilt_deg=round(tilt_in_wall_plane(rot), 3),
        flat_deg=round(flat_on_wall_deg(rot), 3),
        nail_k_mm=[round(float(v) * 1e3, 3) for v in nk],
        head_k_mm=round(float(hk[2]) * 1e3, 3),
        on_nail=nail_in_keyhole(nk, hk),
        nail_n=round(float(nail_n), 3),
        robot_n=round(float(robot_n), 3),
        speed=round(float(lin_speed), 5),
        ang_speed=round(float(ang_speed_deg), 3),
    )


def state_level(m):
    """Level of one measurement, before the hold: tier of the time error if the clock hangs on the nail by its keyhole
    (geometry + the nail carries half its weight), flat on the wall, level within 5 deg, untouched and still."""
    hung = (
        m["on_nail"]
        and m["nail_n"] >= SUPPORT_FRAC * L.CLOCK_MASS * G
        and m["flat_deg"] <= FLAT_DEG
    )
    ok = (
        hung
        and m["tilt_deg"] <= LEVEL_DEG
        and m["robot_n"] <= TOUCH_N
        and m["speed"] <= STILL_MPS
        and m["ang_speed"] <= STILL_DEGPS
    )
    return (tier(m["err_min"]) if ok else 0.0), hung


class Scorer:
    """Per-control-step scorer. ``step(m, vz_min)`` returns the trial's current level: 0 forever once the clock fell
    (downward speed above FALL_MPS at any physics step: vz_min is the lowest vertical velocity since the last control
    step); otherwise the level of the state (``state_level``) once it has held unchanged for HOLD_S, else 0."""

    def __init__(self):
        self.dropped = False
        self.held_s = 0.0
        self.cand = 0.0
        self.level = 0.0
        self.last = {}
        self.min_vz = 0.0

    def step(self, m, vz_min):
        self.min_vz = min(self.min_vz, float(vz_min))
        if vz_min < -FALL_MPS:
            self.dropped = True
        lvl, hung = state_level(m)
        if lvl > 0 and lvl == self.cand:
            self.held_s += DT
        else:
            self.cand = lvl
            self.held_s = DT if lvl > 0 else 0.0
        self.level = (
            0.0 if self.dropped else (self.cand if self.held_s >= HOLD_S - 1e-9 else 0.0)
        )
        self.last = dict(
            measure=m,
            hung=hung,
            candidate=self.cand,
            held_s=round(self.held_s, 3),
            dropped=self.dropped,
            min_vz=round(self.min_vz, 4),
            level=self.level,
        )
        return self.level
