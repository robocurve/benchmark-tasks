"""Shortcut-Proof Numeric S2 Scorer for W50 Baseline Checkpoint (`w50_baseline_bar_scorer.py`, `BUILD-PLAN.md` S2 + `Codex 0856`).

Numeric Conditions (all measurable with a tape measure or photo):
- **Full credit (`1.0`):** `adaptor_bar` (`25.4 x 531.0 x 8.0 mm`) seated across the `304.8 x 508.0 mm` steam well (`wc_xy = (0.4800, 0.0000)` m,
  flat rim `z = 0.8260 m`) with `|bar_x - 0.4800| <= 10.0 mm` (`1.0 cm`), `tilt <= 2.0 deg`, `yaw <= 5.0 deg` from Y, and all four underside
  end-land corners `(+-12.7 mm, +-265.5 mm, 0)` resting on the flat end rims (`|z - 0.8260| <= 3.0 mm`, `|y| in [259.0, 290.8] mm`,
  `|x - 0.4800| <= 152.4 mm`); both left and right gripper pad tips `>= 80.0 mm` (`8.0 cm`) from `adaptor_bar`; and `< 1.0 mm` bar position
  spread across the last `20` steps (`21` frames = `1.00 s` at `20 Hz`).
- **Partial credit (`0.5`):** bar spans level across the end rims (`tilt <= 3.0 deg`, all four end-land corners within `|z - 0.8260| <= 4.0 mm`
  and `|y| in [258.0, 290.8] mm`, `|x - 0.4800| <= 152.4 mm`), both pad tips `>= 80.0 mm` away and `< 1.0 mm` drift over `1.00 s`, with
  `10.0 mm < |bar_x - 0.4800| <= 60.0 mm` or `5.0 deg < yaw <= 12.0 deg`.
- **Zero (`0.0`):** otherwise, or if ANY latched shortcut/violation fires:
  1. `pushed_without_grasp`: `bar_lifted_grasped` never armed (`requires bar lifted z >= stand_top + 25.0 mm while grasped with pad dist <= 5.0 mm and closed/loaded jaws`).
  2. `thrown_or_dropped_in`: hands release (`min_pad_dist > 15.0 mm`) while `bar_x >= 0.300 m` and `bar_z > rim_z + 20.0 mm`.
  3. `bar_fell`: while `bar_x >= 0.300 m`, `bar_z < rim_z - 15.0 mm` or `tilt > 20.0 deg`.
  4. `fixture_moved_or_tipped`: `stand` or `warmer` moves `> 2.0 mm` or tilts `> 1.0 deg` from initial pose.
  5. `goal_undone`: once `1.0` clear & still was achieved, bar later becomes unseated (`geom_tier < 1.0`).
"""
from __future__ import annotations

from collections import deque
import math
import numpy as np

BAR_W_M = 0.0254
BAR_L_M = 0.5310
BAR_H_M = 0.0080
W_IN_M = (0.3048, 0.5080)
W_OUT_M = (0.3683, 0.5715)
STAND_TOP_M = 0.5975
RIM_Z_M = 0.8260
WC_XY_M = (0.4800, 0.0000)

HAND_CLEAR_MIN_M = 0.0800
STILL_WINDOW_FRAMES = 21  # 20 steps = 1.00 s at 20 Hz
STILL_MAX_SPREAD_M = 0.0010


def _bar_obb_dist_m(pts_w: np.ndarray, bar_pos: np.ndarray, bar_R: np.ndarray) -> float:
    pts = np.asarray(pts_w, dtype=np.float64)
    if pts.size == 0:
        return 999.0
    bp = np.asarray(bar_pos, dtype=np.float64)
    Rb = np.asarray(bar_R, dtype=np.float64)
    center = bp + Rb @ np.array([0.0, 0.0, BAR_H_M / 2.0])
    half = np.array([BAR_W_M / 2.0, BAR_L_M / 2.0, BAR_H_M / 2.0])
    loc = np.abs((pts - center) @ Rb) - half
    outside = np.maximum(loc, 0.0)
    return float(np.min(np.linalg.norm(outside, axis=1)))


def evaluate_bar_geom_tier(bar_pos: np.ndarray, bar_R: np.ndarray, wc_xy=WC_XY_M, rim_z: float = RIM_Z_M) -> tuple[float, str]:
    bp = np.asarray(bar_pos, dtype=np.float64)
    Rb = np.asarray(bar_R, dtype=np.float64)
    tilt_deg = math.degrees(math.acos(float(np.clip(Rb[2, 2], -1.0, 1.0))))
    ax = Rb @ np.array([0.0, 1.0, 0.0])
    yaw_deg = math.degrees(math.acos(min(1.0, abs(float(ax[1])) / max(float(np.linalg.norm(ax[:2])), 1e-9))))
    dx = abs(float(bp[0]) - float(wc_xy[0]))

    corners_strict_ok = True
    corners_partial_ok = True
    for sy in (-1.0, +1.0):
        for sx in (-1.0, +1.0):
            under = bp + Rb @ np.array([sx * BAR_W_M / 2.0, sy * BAR_L_M / 2.0, 0.0])
            dz = abs(float(under[2]) - float(rim_z))
            dy = abs(float(under[1]) - float(wc_xy[1]))
            ux = abs(float(under[0]) - float(wc_xy[0]))
            if not (dz <= 0.0030 and (W_IN_M[1] / 2.0 + 0.0050) <= dy <= (W_OUT_M[1] / 2.0 + 0.0050) and ux <= W_IN_M[0] / 2.0):
                corners_strict_ok = False
            if not (dz <= 0.0040 and (W_IN_M[1] / 2.0 + 0.0040) <= dy <= (W_OUT_M[1] / 2.0 + 0.0050) and ux <= W_IN_M[0] / 2.0):
                corners_partial_ok = False

    if tilt_deg <= 2.0 and dx <= 0.0100 and yaw_deg <= 5.0 and corners_strict_ok:
        return 1.0, "seated_centered_1p0"
    if tilt_deg <= 3.0 and dx <= 0.0600 and yaw_deg <= 12.0 and corners_partial_ok:
        return 0.5, "seated_offcenter_0p5"
    return 0.0, "unseated"


class SteamWellBarBaselineTracker:
    def __init__(self, wc_xy=WC_XY_M, rim_z: float = RIM_Z_M, stand_top: float = STAND_TOP_M):
        self.wc_xy = (float(wc_xy[0]), float(wc_xy[1]))
        self.rim_z = float(rim_z)
        self.stand_top = float(stand_top)
        self.reset()

    def reset(self) -> None:
        self.latched: str | None = None
        self.bar_lifted_grasped: bool = False
        self.over_well_entered: bool = False
        self.full_credit_achieved_once: bool = False
        self.fixture_ref: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.buf: deque[dict] = deque(maxlen=STILL_WINDOW_FRAMES)
        self.latest_score: float = 0.0
        self.latest_reason: str = "init"

    def record(
        self,
        bar_pos: np.ndarray,
        bar_R: np.ndarray,
        pad_pts_L: np.ndarray,
        pad_pts_R: np.ndarray,
        q78_mm: dict[str, list[float]] | None = None,
        fixtures: dict[str, tuple[np.ndarray, np.ndarray]] | None = None,
    ) -> dict:
        bp = np.asarray(bar_pos, dtype=np.float64).copy()
        Rb = np.asarray(bar_R, dtype=np.float64).copy()
        tilt_deg = math.degrees(math.acos(float(np.clip(Rb[2, 2], -1.0, 1.0))))

        # 1) Fixture move/tip check
        if fixtures:
            for fk, (fp, fR) in fixtures.items():
                fp_a, fR_a = np.asarray(fp, float), np.asarray(fR, float)
                if fk not in self.fixture_ref:
                    self.fixture_ref[fk] = (fp_a.copy(), fR_a.copy())
                else:
                    p_ref, R_ref = self.fixture_ref[fk]
                    drift_m = float(np.linalg.norm(fp_a - p_ref))
                    rel_R = R_ref.T @ fR_a
                    ftilt_deg = math.degrees(math.acos(float(np.clip((np.trace(rel_R) - 1.0) * 0.5, -1.0, 1.0))))
                    if (drift_m > 0.0020 or ftilt_deg > 1.0) and self.latched is None:
                        self.latched = f"fixture_moved_or_tipped:{fk}"

        dL = _bar_obb_dist_m(pad_pts_L, bp, Rb)
        dR = _bar_obb_dist_m(pad_pts_R, bp, Rb)
        min_pad_d = min(dL, dR)

        jaws_closed_or_loaded = True
        if q78_mm is not None:
            jL = max(float(v) for v in q78_mm.get("L", [-5.0])) <= -0.10 and min(float(v) for v in q78_mm.get("L", [-5.0])) >= -16.0
            jR = max(float(v) for v in q78_mm.get("R", [-5.0])) <= -0.10 and min(float(v) for v in q78_mm.get("R", [-5.0])) >= -16.0
            jaws_closed_or_loaded = bool(jL or jR)

        # 2) Arm grasped lift when bar is lifted >= 25 mm above stand while held by pads (`<= 5 mm`)
        if float(bp[2]) >= self.stand_top + 0.0250 and min_pad_d <= 0.0050 and jaws_closed_or_loaded:
            self.bar_lifted_grasped = True

        geom_tier, geom_reason = evaluate_bar_geom_tier(bp, Rb, wc_xy=self.wc_xy, rim_z=self.rim_z)
        hands_clear = bool(dL >= HAND_CLEAR_MIN_M and dR >= HAND_CLEAR_MIN_M)

        if self.full_credit_achieved_once and geom_tier < 1.0 and self.latched is None:
            self.latched = "goal_undone"

        # 3) Well-zone drop / throw / fall latches
        if float(bp[0]) >= 0.3000:
            self.over_well_entered = True
        if self.over_well_entered and self.latched is None:
            if min_pad_d > 0.0150 and float(bp[2]) > self.rim_z + 0.0200:
                self.latched = "thrown_or_dropped_from_height"
            elif float(bp[2]) < self.rim_z - 0.0150 or tilt_deg > 20.0:
                self.latched = "bar_fell"

        self.buf.append({"pos": bp, "geom_tier": geom_tier, "hands_clear": hands_clear})

        if self.latched is not None:
            self.latest_score, self.latest_reason = 0.0, f"latched:{self.latched}"
        elif not self.bar_lifted_grasped:
            self.latest_score, self.latest_reason = 0.0, "pushed_without_grasp_or_not_lifted"
        elif len(self.buf) < STILL_WINDOW_FRAMES:
            self.latest_score, self.latest_reason = 0.0, "stillness_window_incomplete"
        elif not all(f["hands_clear"] for f in self.buf):
            self.latest_score, self.latest_reason = 0.0, "hands_within_80mm"
        else:
            pts_win = np.stack([f["pos"] for f in self.buf], axis=0)
            spread_m = float(np.max(np.linalg.norm(pts_win - pts_win[0], axis=1)))
            min_tier = float(min(f["geom_tier"] for f in self.buf))
            if spread_m >= STILL_MAX_SPREAD_M:
                self.latest_score, self.latest_reason = 0.0, f"not_still_spread_{spread_m*1e3:.2f}mm"
            elif min_tier <= 0.0:
                self.latest_score, self.latest_reason = 0.0, geom_reason
            else:
                self.latest_score, self.latest_reason = min_tier, geom_reason
                if min_tier >= 1.0:
                    self.full_credit_achieved_once = True

        return {
            "score": self.latest_score,
            "reason": self.latest_reason,
            "geom_tier": geom_tier,
            "pad_clear_mm": {"L": round(dL * 1e3, 2), "R": round(dR * 1e3, 2)},
            "bar_lifted_grasped": self.bar_lifted_grasped,
            "latched": self.latched,
        }
