"""claude-ebeam PROPOSED patch (queue 0914) for the owner's W50 bar-only baseline scorer w50_baseline_bar_scorer.py
(1dcf11ad). The owner's file is imported read-only: geometry tier, constants and OBB distance are reused unchanged.
v2 (native Brev run p1, 09:41): the v1 band [-11, -0.9] missed the real two-hand pinch across the 25.4 mm width (q78 -11.0 to
-14.2 mm), so an honest carry read as "not holding" and false-latched a drop. v2 holding = q78 band [-16.0, -0.9] mm (closed on
the bar's 25.4 mm width or 8 mm thickness; closed EMPTY is about -0.2, open -46.9) AND that hand's pads within 5 mm of the bar.
Changes, all numeric:
  holding(hand): both finger q78 in [-16.0, -0.9] mm AND the hand's pad points within 5 mm of the bar box. Missing q78 = not holding.
  grasp arm: BOTH hands holding for >= 10 consecutive ticks while the bar underside is >= 25 mm above the stand top.
  drop latch: over the well (bar x >= 0.300), no hand holding and bar underside > rim + 5 mm -> thrown_or_dropped_from_height.
  push latch: after the bar is first released on the rims (geom tier > 0 with no hand holding), any later bar xy motion
    > 2 mm from the release pose while no hand is holding -> pushed_without_grasp. Re-grasp (holding) re-arms the pose.
Everything else (fixture latch, bar_fell, goal_undone, hands >= 80 mm, 1 s stillness < 1 mm, tiers) is the owner's."""
from __future__ import annotations
from collections import deque
import math
import numpy as np
import w50_baseline_bar_scorer as O

LOADED_BAND_MM = (-16.0, -0.9)
NEAR_M = 0.0050
ARM_TICKS = 10
DROP_ABOVE_RIM_M = 0.0050
PUSH_XY_M = 0.0020


def _local(pts, bp, Rb):
    center = np.asarray(bp, float) + np.asarray(Rb, float) @ np.array([0.0, 0.0, O.BAR_H_M / 2.0])
    return (np.asarray(pts, float) - center) @ np.asarray(Rb, float)


def holding(pts, q, bp, Rb) -> bool:
    if q is None or pts is None or len(pts) == 0:
        return False
    if not all(LOADED_BAND_MM[0] <= float(v) <= LOADED_BAND_MM[1] for v in q):
        return False
    return O._bar_obb_dist_m(pts, bp, Rb) <= NEAR_M


class SteamWellBarBaselineTrackerPatched(O.SteamWellBarBaselineTracker):
    def reset(self) -> None:
        super().reset()
        self.arm_streak = 0
        self.release_xy = None

    def record(self, bar_pos, bar_R, pad_pts_L, pad_pts_R, q78_mm=None, fixtures=None) -> dict:
        bp = np.asarray(bar_pos, float); Rb = np.asarray(bar_R, float)
        qL = None if q78_mm is None else q78_mm.get("L"); qR = None if q78_mm is None else q78_mm.get("R")
        hL, hR = holding(pad_pts_L, qL, bp, Rb), holding(pad_pts_R, qR, bp, Rb)
        any_hold = hL or hR
        # grasp arm (replaces the owner's single-tick, either-hand, proximity test)
        lifted = float(bp[2]) >= self.stand_top + 0.0250
        self.arm_streak = self.arm_streak + 1 if (hL and hR and lifted) else 0
        armed_before = self.bar_lifted_grasped
        if self.arm_streak >= ARM_TICKS:
            self.bar_lifted_grasped = True
        # owner logic runs with its own arm flag frozen to ours
        keep = self.bar_lifted_grasped
        r = super().record(bar_pos, bar_R, pad_pts_L, pad_pts_R, q78_mm=q78_mm, fixtures=fixtures)
        self.bar_lifted_grasped = keep
        if self.latched is None and self.over_well_entered and not any_hold and float(bp[2]) > self.rim_z + DROP_ABOVE_RIM_M:
            self.latched = "thrown_or_dropped_from_height"
        tier = r["geom_tier"]
        if any_hold:
            self.release_xy = None
        elif tier > 0.0 and self.release_xy is None:
            self.release_xy = bp[:2].copy()
        elif self.release_xy is not None and self.latched is None and float(np.linalg.norm(bp[:2] - self.release_xy)) > PUSH_XY_M:
            self.latched = "pushed_without_grasp"
        if self.latched is not None:
            self.latest_score, self.latest_reason = 0.0, f"latched:{self.latched}"
        elif not self.bar_lifted_grasped:
            self.latest_score, self.latest_reason = 0.0, "pushed_without_grasp_or_not_lifted"
        r.update(score=self.latest_score, reason=self.latest_reason, latched=self.latched, bar_lifted_grasped=self.bar_lifted_grasped,
                 holding={"L": hL, "R": hR})
        return r
