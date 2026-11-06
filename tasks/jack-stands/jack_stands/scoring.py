"""Scorer for jacking a load, setting two stands, lowering it onto them and then pulling the work out. Pure Python.

Fed once per control step (20 Hz) with measured state only (the embodiment derives it from part poses and the
robot's measured joints):

    raised_mm        beam underside at the lift point minus the stands' top height (mm)
    carry_gap_mm     beam underside minus the jack saddle's top, at the lift point (mm)
    under            per stand: its top is under the beam's centre line (within 2 cm) at its support mark (1.5 cm)
    upright          per stand: tilted under 5 degrees, base on the table (within 3 mm)
    stand_gap_mm     per stand: beam underside minus the stand's top, above the stand (mm)
    stand_still      per stand: slower than 1 cm/s
    tilt_deg         beam tilt from level
    load_still       beam slower than 1 cm/s and 0.05 rad/s
    load_vz          beam vertical speed (m/s)
    load_shift_m     beam's horizontal distance from its start
    hand_in_zone     a gripper's tool point or a finger tip is in the crush zone (the beam's footprint grown by 6 cm,
                     below its underside)
    part_xy          wrench's measured position (x, y, m)
    part_clear_m     smallest distance from the beam's footprint to any wrench corner (positive: outside)
    part_still       wrench slower than 2 cm/s

Credit (the best reached; each stage only at rest):
    0.25  the jack holds the beam raised: underside >= 5 mm above the stands' tops, saddle carrying it (within 4 mm)
    0.5   both stands upright under their support marks while the beam is still raised above them
    0.75  the beam rests on both stands (each within 1.5 mm), the saddle >= 5 mm clear below it, level within 2 degrees,
          for 0.5 s
    1.0   then the wrench fully out (every corner >= 1 cm outside the footprint) and still, with the beam still on
          both stands, for 1 s. The trial ends.
Zero, latched for the trial: a hand in the crush zone, or the wrench moved over 2 cm from where it was when the beam
last rested on both stands (from its start if it never has), while the beam is not resting on both stands; the beam tipped (over 10 degrees), dropped (falling faster than 0.25 m/s) or shoved (over 3 cm); or
a non-finite input.
"""

from __future__ import annotations

import math

RAISED_MIN_MM = 5.0
CARRY_TOL_MM = 4.0
REST_TOL_MM = 1.5  # resting: a beam held even 2 mm above a stand by the jack is not on it
CLEAR_MM = 5.0
LEVEL_DEG = 2.0
TIP_DEG = 10.0
DROP_MPS = 0.25
SHOVE_M = 0.03
PART_MOVED_M = 0.02
PART_OUT_M = 0.01
STAGE3_HOLD_S = 0.5
HOLD_S = 1.0


def on_both_stands(under, upright, stand_gap_mm) -> bool:
    """The beam rests on both stands: each upright, under its mark, the underside on its top (within 1.5 mm)."""
    return all(u and s and abs(g) <= REST_TOL_MM for u, s, g in zip(under, upright, stand_gap_mm))


class JackStandsScore:
    def __init__(self, control_hz: float):
        self.dt = 1.0 / float(control_hz)
        self.reset()

    def reset(self) -> None:
        self.best = 0.0
        self.failed = self.success = False
        self.on_stands_s = 0.0
        self.window_s = 0.0
        self.reason = "start"
        self.stage = {"raised": False, "stands_set": False, "on_stands": False}
        self.part_anchor = None  # the wrench's position when the beam last rested on both stands (or its start)
        self.part_moved_m = 0.0

    def info(self) -> dict:
        return {**{f"stage_{k}": v for k, v in self.stage.items()}, "violation": self.failed, "success": self.success,
                "hold_window_s": round(self.window_s, 3), "part_moved_unsafe_m": round(self.part_moved_m, 4),
                "score_reason": self.reason}

    def _fail(self, why: str) -> float:
        self.failed, self.reason, self.window_s = True, why, 0.0
        return 0.0

    def update(self, *, raised_mm, carry_gap_mm, under, upright, stand_gap_mm, stand_still, tilt_deg, load_still,
               load_vz, load_shift_m, hand_in_zone, part_xy, part_clear_m, part_still) -> float:
        if self.failed:
            return 0.0
        nums = [raised_mm, carry_gap_mm, *stand_gap_mm, tilt_deg, load_vz, load_shift_m, *part_xy, part_clear_m]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in nums):
            return self._fail("non-finite state")
        if self.success:
            return 1.0
        on_stands = on_both_stands(under, upright, stand_gap_mm)
        # The wrench may move only while the beam rests on both stands: it is measured from where it was the last time
        # the beam did (from its start before that), so a wrench taken out correctly is never counted against later.
        if self.part_anchor is None or on_stands:
            self.part_anchor = (float(part_xy[0]), float(part_xy[1]))
        self.part_moved_m = math.hypot(part_xy[0] - self.part_anchor[0], part_xy[1] - self.part_anchor[1])
        if tilt_deg > TIP_DEG:
            return self._fail(f"beam tipped ({tilt_deg:.1f} deg)")
        if load_vz < -DROP_MPS:
            return self._fail(f"beam dropped ({load_vz:.2f} m/s)")
        if load_shift_m > SHOVE_M:
            return self._fail(f"beam shoved ({load_shift_m * 100:.1f} cm)")
        if not on_stands and hand_in_zone:
            return self._fail("a gripper under the beam while it was not on both stands")
        if not on_stands and self.part_moved_m > PART_MOVED_M:
            return self._fail("the wrench was moved while the beam was not on both stands")

        rest = load_still
        raised = raised_mm >= RAISED_MIN_MM and abs(carry_gap_mm) <= CARRY_TOL_MM and rest
        if raised:
            self.stage["raised"] = True
            self.best = max(self.best, 0.25)
        set_under = all(u and s and still for u, s, still in zip(under, upright, stand_still))
        if raised and set_under and all(g > REST_TOL_MM for g in stand_gap_mm):
            self.stage["stands_set"] = True
            self.best = max(self.best, 0.5)
        settled = (on_stands and carry_gap_mm >= CLEAR_MM and tilt_deg <= LEVEL_DEG and rest
                   and all(stand_still))
        self.on_stands_s = self.on_stands_s + self.dt if settled else 0.0
        if self.on_stands_s >= STAGE3_HOLD_S - 1e-9:
            self.stage["on_stands"] = True
            self.best = max(self.best, 0.75)
        final = settled and self.stage["on_stands"] and part_clear_m >= PART_OUT_M and part_still
        self.window_s = self.window_s + self.dt if final else 0.0
        if final and self.window_s >= HOLD_S - 1e-9:
            self.success, self.reason = True, "beam on both stands, saddle clear, wrench out"
            return 1.0
        self.reason = ("on stands" if self.stage["on_stands"] else "stands set" if self.stage["stands_set"]
                       else "raised" if self.stage["raised"] else "not raised")
        if final:
            self.reason += "; final condition holding"
        return self.best
