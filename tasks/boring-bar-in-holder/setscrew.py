"""The set-screw thread model, as pure functions of the key's and the bar's pose. NumPy only. PhysX cannot cut a
thread, so this is our own rule, run once per control step by task.py's on_step in the simulator and by
offline_boring.py offline.

    ENGAGED  (a screw i) the key's hex tip within 1.5 mm of the socket's axis (x, y), its shaft within 6 deg of the
             vertical, and the tip at least 7 mm under the top face ("pushed down").
    TURNS    while the key stays engaged in the same screw from one step to the next, the screw turns with the key's
             heading about the vertical: clockwise (the heading falling) screws it in, one pitch (1.0 mm) a turn.
             Its depth d = PITCH x turns, never below 0.
    PRESENT  the bar is under a screw: its axis within 4 mm of the screw's line and covering it by 3 mm at each end.
    CONTACT  the depth at which the screw's tip meets the bar: the tip starts BACKOFF (0.2 mm) over the top of a
             seated round bar, so contact is at 0.2 mm for a bar lying on its round and 1.0 mm for a bar with its flat
             up within 10 deg (the flat is 0.8 mm lower). With no bar under it a screw stops at 0.2 mm (it never
             reaches into the bore); with the bar under it, it stops 0.5 mm past contact.
    TIGHT    PRESENT and d at least contact + 0.4 mm (0.4 turn past contact); it stays tight down to contact + 0.2.
    CLAMP    the first screw to go tight fires the bar's weld to the holder; it is released when no screw is tight.
             The record kept at that moment: the flat's angle from vertical (more than 10 deg: the bar is clamped on
             the round, a bad clamp), the stick-out, and whether the cutting end points away from the chuck.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import layout as L

TWO_PI = 2 * math.pi


def wrap_pi(a: float) -> float:
    return (a + math.pi) % TWO_PI - math.pi


def heading(rot) -> float:
    """The key's heading about the vertical (rad): the crossbar's direction."""
    return math.atan2(float(rot[1, 0]), float(rot[0, 0]))


def tilt_deg(rot) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))


def key_engaged(key_origin, key_rot, i: int) -> bool:
    """Is the key's tip in screw ``i``'s socket, pushed down, shaft upright?"""
    o = np.asarray(key_origin, dtype=float)
    if not (np.all(np.isfinite(o)) and np.all(np.isfinite(key_rot))):
        return False
    dx = (o[0] - L.X_AX * L.MM)
    dy = (o[1] - L.SCREW_Y[i] * L.MM)
    if math.hypot(dx, dy) > L.ENGAGE_XY * L.MM + 1e-12:
        return False
    if tilt_deg(key_rot) > L.ENGAGE_TILT + 1e-9:
        return False
    return (L.HOLDER_TOP * L.MM + L.TOP) - o[2] >= L.ENGAGE_DEPTH * L.MM - 1e-12


def flat_err_deg(bar_rot) -> float:
    n = np.asarray(bar_rot, dtype=float) @ L.FLAT_N
    return math.degrees(math.acos(max(-1.0, min(1.0, float(n[2])))))


def under_screw(bar_origin, bar_rot, i: int):
    """(present, axis point (m) under screw i)."""
    o = np.asarray(bar_origin, dtype=float)
    r = np.asarray(bar_rot, dtype=float)
    s = np.array([L.X_AX * L.MM, L.SCREW_Y[i] * L.MM, L.TOP + L.Z_AX * L.MM])
    p = r.T @ (s - o)  # the screw point in the bar frame
    present = (abs(p[1]) <= L.BAR_UNDER_XY * L.MM and abs(p[2]) <= L.BAR_UNDER_XY * L.MM
               and abs(p[0]) <= (L.BAR_L / 2 - L.BAR_UNDER_ENDS) * L.MM)
    return present, o + r[:, 0] * p[0]


def contact_depth_mm(bar_origin, bar_rot, i: int):
    """(present, the depth in mm at which screw i meets the bar)."""
    present, axis_pt = under_screw(bar_origin, bar_rot, i)
    if not present:
        return False, math.inf
    top = L.FLAT_Z if flat_err_deg(bar_rot) <= L.FLAT_OK_DEG else L.BAR_R
    z_surf = (axis_pt[2] - L.TOP) / L.MM + top
    return True, L.Z_SCREW0 - z_surf


@dataclass
class Step:
    fire: bool = False
    release: bool = False
    record: dict | None = None


@dataclass
class ScrewModel:
    turns: list = field(default_factory=lambda: [0.0, 0.0])
    tight: list = field(default_factory=lambda: [False, False])
    engaged: list = field(default_factory=lambda: [False, False])
    weld: bool = False
    clamped_away: bool = False
    record: dict | None = None
    psi_prev: float | None = None
    eng_prev: list = field(default_factory=lambda: [False, False])
    skipped: int = 0

    def __post_init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.turns = [0.0, 0.0]
        self.tight = [False, False]
        self.engaged = [False, False]
        self.eng_prev = [False, False]
        self.psi_prev = None
        self.weld = False
        self.clamped_away = False
        self.record = None
        self.skipped = 0

    def depth_mm(self, i: int) -> float:
        return L.PITCH * self.turns[i]

    def snapshot(self) -> dict:
        return {"tight": list(self.tight), "engaged": list(self.engaged), "weld": self.weld,
                "clamped_away": self.clamped_away, "turns": [round(t, 3) for t in self.turns],
                "record": dict(self.record) if self.record else None}

    def update(self, key_origin, key_rot, bar_origin, bar_rot) -> Step:
        """One control step. A pose that cannot be read counts as nothing engaged and no bar under any screw."""
        try:
            ko, kr = np.asarray(key_origin, dtype=float), np.asarray(key_rot, dtype=float)
            bo, br = np.asarray(bar_origin, dtype=float), np.asarray(bar_rot, dtype=float)
            ok = all(np.all(np.isfinite(v)) for v in (ko, kr, bo, br))
        except (TypeError, ValueError):
            ok = False
        if not ok:
            self.skipped += 1
            self.psi_prev = None
            self.eng_prev = [False, False]
            self.engaged = [False, False]
            self.tight = [False, False]
            was = self.weld
            self.weld = False
            return Step(release=was)
        psi = heading(kr)
        eng = [key_engaged(ko, kr, i) for i in range(2)]
        for i in range(2):
            present, contact = contact_depth_mm(bo, br, i)
            if eng[i] and self.eng_prev[i] and self.psi_prev is not None:
                self.turns[i] += -wrap_pi(psi - self.psi_prev) / TWO_PI
            cap = (contact + L.CAP_PAST) if present else L.BACKOFF
            d = min(max(L.PITCH * self.turns[i], 0.0), cap)
            self.turns[i] = d / L.PITCH
            if present:
                if d >= contact + L.TIGHT_PAST * L.PITCH - 1e-9:
                    self.tight[i] = True
                elif d < contact + L.TIGHT_KEEP * L.PITCH - 1e-9:
                    self.tight[i] = False
            else:
                self.tight[i] = False
        self.engaged = eng
        self.eng_prev = list(eng)
        self.psi_prev = psi
        fire = release = False
        record = None
        if any(self.tight) and not self.weld:
            self.weld = True
            fire = True
            a = br[:, 0]
            record = {"flat_err_deg": round(flat_err_deg(br), 2),
                      "bad_clamp": bool(flat_err_deg(br) > L.FLAT_OK_DEG),
                      "stick_mm": round(L.stickout_mm(float((bo + a * L.BAR_L / 2 * L.MM)[1])), 2),
                      "cutting_end_away": bool(a[1] < 0.0)}
            self.record = record
            if record["cutting_end_away"]:
                self.clamped_away = True
        elif not any(self.tight) and self.weld:
            self.weld = False
            release = True
        return Step(fire, release, record)
