"""Scorer for: seat the adaptor bar across the steam well, then seat one quarter pan in the far channel on bar + far rim.

Reuses the bar logic of w50_baseline_bar_scorer.py + w50_bar_scorer_v2.py (bar tier, holding, lift arm,
drop / fell / fixture / pushed latches) and the pan seat check pan_ok_bar_frame() of w50_four_pan_waypoints.py.
Checked every control step (20 Hz); the score is the state over the final 1.0 s (21 frames). All numbers in SI units.

1.0  bar armed (lifted >= 25 mm above the stand top while BOTH hands held it for >= 10 consecutive steps) and, in all 21
     frames: bar tier 1.0 (centred, level, square, all four end corners on the end rims); the pan seated in the far channel
     (pan centre beyond the well centre line toward the far long rim; tilt <= PAN_TILT_MAX_DEG; at +-80 % of its long side
     the inner long flange underside within 3 mm of the bar top and overlapping the bar >= 5 mm, the outer long flange
     underside within 3 mm of the far rim and overlapping it >= 5 mm; all four pan body corners inside the well opening);
     both hands' finger pads >= 80 mm from bar and pan; bar and pan each moved < 1 mm across the 21 frames.
0.5  the same bar conditions, hands clear and stillness, with the pan not seated (and nothing latched).
0.0  otherwise, and latched 0 for the rest of the trial after any of the latches below.
"""
from __future__ import annotations

from collections import deque
import math

import numpy as np

import w50_baseline_bar_scorer as O
import w50_bar_scorer_v2 as P
import w50_four_pan_waypoints as F

Q_W, Q_L, Q_D, Q_FL, SHEET = F.Q_W, F.Q_L, F.Q_D, F.Q_FL, F.SHEET
W_IN, W_OUT = F.W_IN, F.W_OUT
BAR_H = F.BAR_H

HOLD_FRAMES = 21
HAND_CLEAR_M = 0.080
STILL_M = 0.0030            # fix (review 1333): two marked points on each part stay within 3 mm in the final second
STAND_X = (0.1300, 0.8920)   # as-built stand top, metres (scene_w50 / live scorer v4)
STAND_Y_HALF = 0.6095
OFF_STAND_BELOW_M = 0.015    # a part more than 15 mm below the stand top has fallen off the stand
REST_TOL_M = 0.003           # pan resting on the stand: base within 3 mm of the stand top
_RZ180 = np.diag([-1.0, -1.0, 1.0])


def canonical_pan_R(R):
    """The pan is symmetric under a half turn about its vertical axis: pick the representative whose local +x points to
    world +x, so every flange check is the same for a pan turned end-for-end (review 1333 item 2)."""
    R = np.asarray(R, float)
    return R @ _RZ180 if float(R[0, 0]) < 0.0 else R


def on_stand_xy(p) -> bool:
    return STAND_X[0] <= float(p[0]) <= STAND_X[1] and abs(float(p[1])) <= STAND_Y_HALF


def marked_points(pos, R, half_len):
    """two marked points on a part: its two ends along local y, at the part origin height"""
    p, Rm = np.asarray(pos, float), np.asarray(R, float)
    return np.stack([p + Rm @ np.array([0.0, +half_len, 0.0]), p + Rm @ np.array([0.0, -half_len, 0.0])])
PAN_TILT_MAX_DEG = 5.0
GAP_TOL_M = 0.003
MIN_OVERLAP_M = 0.005
PAN_DROP_ABOVE_M = 0.005
SPILL_TILT_DEG = 15.0      # v2 food: pan tilted more than this at any time after it is first lifted = exceeded the carrying tilt
PAN_LIFTED_M = 0.010       # pan counts as lifted once its base is 10 mm above where it started
FELL_BELOW_RIM_M = 0.015
FELL_TILT_DEG = 20.0
PAN_CONTACT_BAR_M = 0.003


def _tilt_deg(R) -> float:
    return math.degrees(math.acos(float(np.clip(np.asarray(R, float)[2, 2], -1.0, 1.0))))


def pan_obbs(pos, R):
    """The quarter pan's 9 sheet boxes as (centre, R, half-size) in world coordinates (same boxes as the scene)."""
    p, Rp = np.asarray(pos, float), np.asarray(R, float)
    bw, bl = Q_W - 2.0 * Q_FL, Q_L - 2.0 * Q_FL
    raw = [
        ((0.0, 0.0, SHEET / 2.0), (bw, bl, SHEET)),
        ((-bw / 2.0 + SHEET / 2.0, 0.0, Q_D / 2.0), (SHEET, bl, Q_D)),
        ((+bw / 2.0 - SHEET / 2.0, 0.0, Q_D / 2.0), (SHEET, bl, Q_D)),
        ((0.0, -bl / 2.0 + SHEET / 2.0, Q_D / 2.0), (bw, SHEET, Q_D)),
        ((0.0, +bl / 2.0 - SHEET / 2.0, Q_D / 2.0), (bw, SHEET, Q_D)),
        ((-Q_W / 2.0 + Q_FL / 2.0, 0.0, Q_D - SHEET / 2.0), (Q_FL, Q_L, SHEET)),
        ((+Q_W / 2.0 - Q_FL / 2.0, 0.0, Q_D - SHEET / 2.0), (Q_FL, Q_L, SHEET)),
        ((0.0, -Q_L / 2.0 + Q_FL / 2.0, Q_D - SHEET / 2.0), (bw, Q_FL, SHEET)),
        ((0.0, +Q_L / 2.0 - Q_FL / 2.0, Q_D - SHEET / 2.0), (bw, Q_FL, SHEET)),
    ]
    return [(p + Rp @ np.asarray(c, float), Rp, 0.5 * np.asarray(s, float)) for c, s in raw]


def dist_pts_to_pan_m(pts, pan_pos, pan_R) -> float:
    pts = np.asarray(pts, float)
    if pts.size == 0:
        return 999.0
    best = float("inf")
    for c, R, h in pan_obbs(pan_pos, pan_R):
        ex = np.maximum(np.abs((pts - c) @ R) - h, 0.0)
        best = min(best, float(np.sqrt(np.min(np.sum(ex * ex, axis=1)))))
    return best


def pan_sample_pts(pan_pos, pan_R) -> np.ndarray:
    """Corners of all 9 pan boxes (for the pan-to-bar contact test)."""
    out = []
    for c, R, h in pan_obbs(pan_pos, pan_R):
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    out.append(c + R @ (h * np.array([sx, sy, sz], float)))
    return np.asarray(out)


def pan_holding(pts, q, pan_pos, pan_R) -> bool:
    """A hand holds the pan: both finger joints in the loaded band [-16.0, -0.9] mm and its pads within 5 mm of the pan."""
    if q is None or pts is None or len(pts) == 0:
        return False
    if not all(P.LOADED_BAND_MM[0] <= float(v) <= P.LOADED_BAND_MM[1] for v in q):
        return False
    return dist_pts_to_pan_m(pts, pan_pos, pan_R) <= P.NEAR_M


def pan_over_well(pan_pos, wc_xy) -> bool:
    p = np.asarray(pan_pos, float)
    return abs(float(p[0]) - wc_xy[0]) <= W_OUT[0] / 2.0 and abs(float(p[1]) - wc_xy[1]) <= W_OUT[1] / 2.0


def pan_support_gaps_m(pan_pos, pan_R, wc_xy, rim_z, bar_present: bool) -> tuple[float, float]:
    """(inner, outer) long-flange underside height above its support (bar top or rim; far channel assumed when the pan
    centre is beyond the well centre line, near channel otherwise). Lowest of 5 points along each flange's free edge."""
    p, Rp = np.asarray(pan_pos, float), np.asarray(pan_R, float)
    side = 1.0 if float(p[0]) < wc_xy[0] else -1.0  # +1: pan in the near channel, inner flange at +x
    z_in_sup = rim_z + (BAR_H if bar_present else 0.0)
    zi, zo = [], []
    for yl in np.linspace(-0.8 * Q_L / 2.0, 0.8 * Q_L / 2.0, 5):
        for xl in (Q_W / 2.0 - Q_FL, Q_W / 2.0):
            zi.append(float((p + Rp @ np.array([side * xl, yl, Q_D - SHEET]))[2]))
            zo.append(float((p + Rp @ np.array([-side * xl, yl, Q_D - SHEET]))[2]))
    return min(zi) - z_in_sup, min(zo) - rim_z


def pan_seated_far(pan_pos, pan_R, bar_pos, bar_R, wc_xy, rim_z) -> bool:
    if bar_pos is None:
        return False
    if float(np.asarray(pan_pos, float)[0]) <= wc_xy[0]:
        return False
    if _tilt_deg(pan_R) > PAN_TILT_MAX_DEG:
        return False
    return bool(F.pan_ok_bar_frame(pan_pos, pan_R, wc_xy, rim_z, bar_pos, bar_R, gap_tol=GAP_TOL_M, min_overlap=MIN_OVERLAP_M))


def _seat_diag(p, Rp, bp, Rb, wc_xy, rim_z):
    """diagnostic only: which pan_ok_bar_frame sub-check fails (same geometry as F.pan_ok_bar_frame)"""
    if bp is None:
        return "no_bar"
    out = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            c = p + Rp @ np.array([sx * (Q_W / 2.0 - F.Q_FL), sy * (Q_L / 2.0 - F.Q_FL), 0.0])
            if abs(float(c[0]) - wc_xy[0]) > F.W_IN[0] / 2.0 or abs(float(c[1]) - wc_xy[1]) > F.W_IN[1] / 2.0:
                out.append("corner_out")
    side = 1.0 if float(p[0]) < wc_xy[0] else -1.0
    rim_edge_x = wc_xy[0] - side * (F.W_IN[0] / 2.0)
    for yl in (-0.8 * Q_L / 2.0, +0.8 * Q_L / 2.0):
        e_out = p + Rp @ np.array([side * Q_W / 2.0, yl, Q_D - SHEET])
        e_in = p + Rp @ np.array([side * (Q_W / 2.0 - F.Q_FL), yl, Q_D - SHEET])
        lo = Rb.T @ (e_out - bp); li = Rb.T @ (e_in - bp)
        f0, f1 = sorted((float(li[0]), float(lo[0])))
        ov_bar = min(f1, F.BAR_W / 2.0) - max(f0, -F.BAR_W / 2.0)
        o_under = p + Rp @ np.array([-side * Q_W / 2.0, yl, Q_D - SHEET])
        o_ov = (rim_edge_x - float(o_under[0])) if side > 0 else (float(o_under[0]) - rim_edge_x)
        out.append(f"y{yl*1e3:+.0f}:barz{(float(lo[2])-F.BAR_H)*1e3:+.1f},barov{ov_bar*1e3:.1f},rimz{(float(o_under[2])-rim_z)*1e3:+.1f},rimov{o_ov*1e3:.1f}")
    return ";".join(out)


class _BarTracker(P.SteamWellBarBaselineTrackerPatched):
    """The patched bar tracker, except that pan contact on the bar counts like a holding hand for the pushed latch
    (the pan's flange is meant to bear on the bar)."""

    def record(self, bar_pos, bar_R, pad_pts_L, pad_pts_R, q78_mm=None, fixtures=None, pan_contact=False) -> dict:
        bp = np.asarray(bar_pos, float); Rb = np.asarray(bar_R, float)
        qL = None if q78_mm is None else q78_mm.get("L"); qR = None if q78_mm is None else q78_mm.get("R")
        hL, hR = P.holding(pad_pts_L, qL, bp, Rb), P.holding(pad_pts_R, qR, bp, Rb)
        any_hold = hL or hR
        lifted = float(bp[2]) >= self.stand_top + 0.0250
        self.arm_streak = self.arm_streak + 1 if (hL and hR and lifted) else 0
        if self.arm_streak >= P.ARM_TICKS:
            self.bar_lifted_grasped = True
        keep = self.bar_lifted_grasped
        self.full_credit_achieved_once = False  # bar-only goal_undone is replaced by the combined latches
        r = O.SteamWellBarBaselineTracker.record(self, bar_pos, bar_R, pad_pts_L, pad_pts_R, q78_mm=q78_mm, fixtures=fixtures)
        self.bar_lifted_grasped = keep
        if self.latched is None and self.over_well_entered and not any_hold and float(bp[2]) > self.rim_z + P.DROP_ABOVE_RIM_M:
            self.latched = "bar_thrown_or_dropped_from_height"
        tier = r["geom_tier"]
        if any_hold or pan_contact:
            self.release_xy = None if any_hold else (bp[:2].copy() if tier > 0.0 else None)
        elif tier > 0.0 and self.release_xy is None:
            self.release_xy = bp[:2].copy()
        elif self.release_xy is not None and self.latched is None and float(np.linalg.norm(bp[:2] - self.release_xy)) > P.PUSH_XY_M:
            self.latched = "bar_pushed_without_grasp"
        r.update(latched=self.latched, bar_lifted_grasped=self.bar_lifted_grasped, holding={"L": hL, "R": hR})
        return r


class BarPanTracker:
    def __init__(self, wc_xy=(0.48, 0.0), rim_z=0.8261, stand_top=0.5975):
        self.wc_xy = (float(wc_xy[0]), float(wc_xy[1]))
        self.rim_z = float(rim_z)
        self.stand_top = float(stand_top)
        self.bar = _BarTracker(wc_xy=self.wc_xy, rim_z=self.rim_z, stand_top=self.stand_top)
        self.reset()

    def reset(self) -> None:
        self.bar.reset()
        self.latched: str | None = None
        self.pan_placed = False
        self.pan_z0 = None
        self.pan_lifted = False
        self.max_tilt_after_lift = 0.0
        self.full_once = False
        self.buf: deque = deque(maxlen=HOLD_FRAMES)
        self.latest_score = 0.0
        self.latest_reason = "init"

    def _latch(self, why: str) -> None:
        if self.latched is None:
            self.latched = why

    def record(self, bar_pose, pan_pose, pad_pts_L, pad_pts_R, q78_mm=None, fixtures=None) -> dict:
        """bar_pose: (pos, R) or None when the scene has no bar (control run). pan_pose: (pos, R)."""
        pp, pR = (np.asarray(pan_pose[0], float).copy(), canonical_pan_R(np.asarray(pan_pose[1], float).copy()))
        qL = None if q78_mm is None else q78_mm.get("L"); qR = None if q78_mm is None else q78_mm.get("R")
        bar_present = bar_pose is not None

        # pan / hand relations
        pan_dL, pan_dR = dist_pts_to_pan_m(pad_pts_L, pp, pR), dist_pts_to_pan_m(pad_pts_R, pp, pR)
        pan_held = pan_holding(pad_pts_L, qL, pp, pR) or pan_holding(pad_pts_R, qR, pp, pR)

        if bar_present:
            bp, bR = np.asarray(bar_pose[0], float).copy(), np.asarray(bar_pose[1], float).copy()
            pan_contact = O._bar_obb_dist_m(pan_sample_pts(pp, pR), bp, bR) <= PAN_CONTACT_BAR_M
            rb = self.bar.record(bp, bR, pad_pts_L, pad_pts_R, q78_mm=q78_mm, fixtures=fixtures, pan_contact=pan_contact)
            bar_tier = float(rb["geom_tier"])
            bar_dL, bar_dR = rb["pad_clear_mm"]["L"] / 1e3, rb["pad_clear_mm"]["R"] / 1e3
            if rb["latched"] is not None:
                self._latch(str(rb["latched"]))
            armed = bool(self.bar.bar_lifted_grasped)
        else:
            bp = bR = None
            pan_contact = False
            bar_tier, bar_dL, bar_dR, armed = 0.0, 999.0, 999.0, False
            if fixtures:  # fixture latch still applies in the control run
                self.bar.record(np.array([0.0, 0.0, -1.0]), np.eye(3), np.zeros((0, 3)), np.zeros((0, 3)), fixtures=fixtures)
                if self.bar.latched and self.bar.latched.startswith("fixture"):
                    self._latch(self.bar.latched)

        # pan latches
        if self.pan_z0 is None:
            self.pan_z0 = float(pp[2])
        if float(pp[2]) > self.pan_z0 + PAN_LIFTED_M:
            self.pan_lifted = True
        if self.pan_lifted:
            self.max_tilt_after_lift = max(self.max_tilt_after_lift, _tilt_deg(pR))
            if _tilt_deg(pR) > SPILL_TILT_DEG:
                self._latch("exceeded_carrying_tilt")
        over = pan_over_well(pp, self.wc_xy)
        gin, gout = pan_support_gaps_m(pp, pR, self.wc_xy, self.rim_z, bar_present)
        flange_z = float((pp + pR @ np.array([0.0, 0.0, Q_D - SHEET]))[2])
        if over and (flange_z < self.rim_z - FELL_BELOW_RIM_M or _tilt_deg(pR) > FELL_TILT_DEG):
            self._latch("pan_fell_in")
        if over and not pan_held and min(gin, gout) > PAN_DROP_ABOVE_M:
            self._latch("pan_dropped_from_height")
        # fix (review 1333 item 1): either part off the stand = 0 for the rest of the trial
        if float(pp[2]) < self.stand_top - OFF_STAND_BELOW_M or (not over and not on_stand_xy(pp) and not pan_held):
            self._latch("pan_off_stand")
        if bp is not None and float(bp[2]) < self.stand_top - OFF_STAND_BELOW_M:  # an actual fall (codex 1341), not a held excursion
            self._latch("bar_off_stand")
        pan_on_stand_upright = (not over and on_stand_xy(pp) and abs(float(pp[2]) - self.stand_top) <= REST_TOL_M
                                and _tilt_deg(pR) <= PAN_TILT_MAX_DEG and not pan_held)
        seated = pan_seated_far(pp, pR, bp, bR, self.wc_xy, self.rim_z)
        if seated and not pan_held:
            self.pan_placed = True
        if self.pan_placed and bar_present and bar_tier < 1.0:
            self._latch("loaded_bar_unseated")

        hands_clear = min(bar_dL, bar_dR, pan_dL, pan_dR) >= HAND_CLEAR_M
        self.buf.append({"bar_m": None if bp is None else marked_points(bp, bR, F.BAR_L / 2.0), "pan_m": marked_points(pp, pR, Q_L / 2.0),
                         "bar_tier": bar_tier, "seated": seated, "pan_rest": pan_on_stand_upright, "hands_clear": hands_clear})

        score, reason = 0.0, "ok"
        if self.latched is not None:
            reason = f"latched:{self.latched}"
        elif not bar_present:
            reason = "no_bar"
        elif not armed:
            reason = "bar_not_lifted_by_both_hands"
        elif len(self.buf) < HOLD_FRAMES:
            reason = "window_incomplete"
        elif not all(f["hands_clear"] for f in self.buf):
            reason = "hands_within_80mm"
        elif not all(f["bar_tier"] >= 1.0 for f in self.buf):
            reason = "bar_not_seated"
        else:
            bspread = max(float(np.max(np.linalg.norm(f["bar_m"] - self.buf[0]["bar_m"], axis=1))) for f in self.buf)
            pspread = max(float(np.max(np.linalg.norm(f["pan_m"] - self.buf[0]["pan_m"], axis=1))) for f in self.buf)
            if bspread >= STILL_M or pspread >= STILL_M:
                reason = f"not_still(bar={bspread*1e3:.2f}mm,pan={pspread*1e3:.2f}mm)"
            elif all(f["seated"] for f in self.buf):
                score, reason = 1.0, "bar_and_pan_seated"
            elif all(f["pan_rest"] for f in self.buf):
                score, reason = 0.5, "bar_seated_pan_upright_on_stand"
            else:
                reason = "pan_not_seated_nor_upright_on_stand"
        if self.full_once and score < 1.0 and self.latched is None:
            self._latch("full_credit_lost")
            score, reason = 0.0, "latched:full_credit_lost"
        if score >= 1.0:
            self.full_once = True
        self.latest_score, self.latest_reason = score, reason
        return {
            "score": score, "reason": reason, "latched": self.latched, "bar_tier": bar_tier, "bar_armed": armed,
            "pan_seated": seated, "pan_held": pan_held, "pan_placed": self.pan_placed, "pan_contact_bar": pan_contact,
            "pan_gap_in_mm": round(gin * 1e3, 2), "pan_gap_out_mm": round(gout * 1e3, 2),
            "pan_tilt_deg": round(_tilt_deg(pR), 3), "pan_over_well": over, "pan_lifted": self.pan_lifted,
            "pan_yaw_deg": round(math.degrees(math.atan2(float(pR[1, 0]), float(pR[0, 0]))), 2),
            "seat_diag": _seat_diag(pp, pR, bp, bR, self.wc_xy, self.rim_z),
            "max_tilt_after_lift_deg": round(self.max_tilt_after_lift, 3),
            "pad_clear_mm": {"bar": [round(bar_dL * 1e3, 1), round(bar_dR * 1e3, 1)], "pan": [round(pan_dL * 1e3, 1), round(pan_dR * 1e3, 1)]},
        }
