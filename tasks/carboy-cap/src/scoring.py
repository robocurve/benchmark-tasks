"""Score the final closure and funnel containment in either handling order.

A bottle tilt beyond 25 degrees latches the spill veto. The first stow and seal
times are recorded independently for reviewing the handling sequence.
"""

from __future__ import annotations

import layout as L

SEALED_GAP_MM = L.SEALED_GAP * 1000.0
SEALED_MIN_TURNS = (L.TRAVEL - L.SEALED_GAP) / L.PITCH
STOW_S = 0.5
CLEAR_S = 4.0
TIP_DEG = 25.0


class Scorer:
    def __init__(self):
        self.t = 0.0
        self.veto = None
        self.veto_t = None
        self.stow_s = 0.0
        self.stowed = False
        self.stowed_t = None
        self.sealed_at_stow = None
        self.sealed_t = None  # first time sealed
        self.clear_s = 0.0
        self.cap_clear_s = 0.0
        self.max_tilt = 0.0
        self.min_gap = None
        self.last = {}

    def _veto(self, why):
        if self.veto is None:
            self.veto, self.veto_t = why, round(self.t, 3)

    def step(self, dt, gap_mm, engaged, funnel_in_tray, funnel_on_bench, contact, bottle_tilt_deg, phi_deg=0.0, funnel_still=True):
        self.t += dt
        sealed = bool(engaged and gap_mm is not None and gap_mm <= SEALED_GAP_MM
                      and phi_deg >= 360.0 * SEALED_MIN_TURNS)
        if gap_mm is not None:
            self.min_gap = gap_mm if self.min_gap is None else min(self.min_gap, gap_mm)
        if sealed and self.sealed_t is None:
            self.sealed_t = round(self.t, 3)
        self.max_tilt = max(self.max_tilt, bottle_tilt_deg)
        if bottle_tilt_deg > TIP_DEG:
            self._veto("bottle_tipped")
        if not self.stowed:
            if funnel_in_tray and funnel_still and not contact.get("funnel", False):
                self.stow_s += dt
            else:
                self.stow_s = 0.0
            if self.stow_s >= STOW_S - 1e-9:
                self.stowed, self.stowed_t, self.sealed_at_stow = True, round(self.t, 3), sealed
        any_contact = any(contact.get(k, False) for k in ("cap", "funnel", "bottle"))
        final_pose = sealed and funnel_in_tray and funnel_still and not any_contact
        self.clear_s = self.clear_s + dt if final_pose else 0.0
        self.cap_clear_s = 0.0 if not sealed or contact.get("cap", False) else self.cap_clear_s + dt
        level = self.level(sealed, funnel_in_tray, funnel_on_bench, bottle_tilt_deg)
        self.last = dict(
            level=level,
            veto=self.veto,
            veto_t=self.veto_t,
            sealed=sealed,
            engaged=bool(engaged),
            gap_mm=None if gap_mm is None else round(gap_mm, 3),
            min_gap_mm=None if self.min_gap is None else round(self.min_gap, 3),
            phi_deg=round(phi_deg, 1),
            sealed_t=self.sealed_t,
            stowed=self.stowed,
            stowed_t=self.stowed_t,
            sealed_at_stow=self.sealed_at_stow,
            funnel_still=bool(funnel_still),
            funnel_in_tray=bool(funnel_in_tray),
            funnel_on_bench=bool(funnel_on_bench),
            contact={k: bool(v) for k, v in contact.items()},
            clear_s=round(self.clear_s, 3),
            bottle_tilt_deg=round(bottle_tilt_deg, 2),
            max_tilt_deg=round(self.max_tilt, 2),
        )
        return level

    def level(self, sealed, funnel_in_tray, funnel_on_bench, tilt):
        if self.veto is not None or tilt > TIP_DEG:
            return 0.0
        if sealed and self.stowed and funnel_in_tray and self.clear_s >= CLEAR_S - 1e-9:
            return 1.0
        if sealed and self.cap_clear_s >= CLEAR_S - 1e-9 and funnel_on_bench and not funnel_in_tray:
            return 0.5
        return 0.0
