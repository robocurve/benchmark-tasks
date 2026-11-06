"""Scorer for finding top dead centre with a piston stop. Pure Python (no Isaac, no NumPy).

Fed once per control step (20 Hz) with measured state only:
    theta_deg      crank angle from true TDC, wrapped to (-180, 180] (on a real trainer: the degree wheel read
                   against a reference mark made at TDC when the trainer was built)
    omega_dps      crank speed
    stop_seated    the stop is fully down in its slot (pose within 2 mm and 3 degrees of seated)
    stop_in_tray   the stop rests inside the tray (centre within the tray's inner footprint, below 8 cm above the
                   tray floor, moving under 5 cm/s)
    grip_open      both grippers' openings, 0 closed .. 1 open
    tcp_clear_m    each tool point's distance from the handle sleeve centre
    gap_mm         piston crown to the stop's face along the bore (measured from the part poses; used only while
                   the stop is seated)

Credit (latched, best so far):
    0.25 per side  the piston is in contact with the seated stop on that side and the crank is at rest there:
                   piston-to-stop gap under CONTACT_GAP_MM (0.3 mm), crank under 2 deg/s, and theta within SIDE_TOL
                   of +CONTACT (crank turned back from after TDC) or -CONTACT (turned forward from before TDC)
    1.0 success    |theta| <= TDC_TOL, stop in the tray, both grippers >= 80 % open and both tool points >= 5 cm
                   from the handle, crank under 1 deg/s, all held for 1 s. The trial ends.
Zero, latched: the crank more than FORCED past the contact angle (toward TDC) while the stop is seated (forced
through the stop: a bent rod on a real engine), or a non-finite input. The forced rule is a guard: the stop is rigid and
backed by the head wall, so reaching it would need about 1 mm of penetration (measured: 0.05 mm at most).
"""

from __future__ import annotations

import math

OPEN_MIN = 0.8
CLEAR_MIN_M = 0.05
STILL_DPS = 1.0
SIDE_STILL_DPS = 2.0  # a side counts only with the crank at rest against the stop, not passing through the band
CONTACT_GAP_MM = 0.3  # a side counts only with the piston touching the stop (a shove short of it earns nothing)
HOLD_S = 1.0


class TdcScore:
    def __init__(self, control_hz: float, contact_deg: float, side_tol_deg: float = 1.0, forced_deg: float = 3.0,
                 tdc_tol_deg: float = 2.0):
        self.dt = 1.0 / float(control_hz)
        self.contact, self.side_tol, self.forced_tol, self.tdc_tol = contact_deg, side_tol_deg, forced_deg, tdc_tol_deg
        self.reset()

    def reset(self) -> None:
        self.sides = {"after": False, "before": False}  # theta > 0 side, theta < 0 side
        self.failed = self.success = False
        self.window_s = 0.0
        self.reason = "start"

    def info(self) -> dict:
        return {"side_after_tdc": self.sides["after"], "side_before_tdc": self.sides["before"],
                "forced_or_invalid": self.failed, "success": self.success, "hold_window_s": round(self.window_s, 3),
                "score_reason": self.reason}

    def partial(self) -> float:
        return 0.25 * sum(self.sides.values())

    def update(self, theta_deg, omega_dps, stop_seated, stop_in_tray, grip_open, tcp_clear_m, gap_mm) -> float:
        nums = [theta_deg, omega_dps, gap_mm, *grip_open, *tcp_clear_m]
        if self.failed:
            return 0.0
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in nums):
            self.failed, self.reason, self.window_s = True, "non-finite state", 0.0
            return 0.0
        if self.success:
            return 1.0
        if stop_seated:
            if abs(theta_deg) < self.contact - self.forced_tol:
                self.failed, self.reason, self.window_s = True, "crank forced past the seated stop", 0.0
                return 0.0
            at_rest = abs(omega_dps) < SIDE_STILL_DPS and gap_mm < CONTACT_GAP_MM
            if at_rest and abs(theta_deg - self.contact) <= self.side_tol:
                self.sides["after"] = True
            if at_rest and abs(theta_deg + self.contact) <= self.side_tol:
                self.sides["before"] = True
        released = all(g >= OPEN_MIN for g in grip_open) and all(c >= CLEAR_MIN_M for c in tcp_clear_m)
        final = (abs(theta_deg) <= self.tdc_tol and stop_in_tray and not stop_seated and released
                 and abs(omega_dps) < STILL_DPS)
        self.window_s = self.window_s + self.dt if final else 0.0
        if final and self.window_s >= HOLD_S - 1e-9:
            self.success, self.reason = True, "at TDC, stop in the tray, hands clear, crank still"
            return 1.0
        n = sum(self.sides.values())
        self.reason = ("both sides measured" if n == 2 else "one side measured" if n == 1 else "no side measured")
        if final:
            self.reason += "; final condition holding"
        return self.partial()
