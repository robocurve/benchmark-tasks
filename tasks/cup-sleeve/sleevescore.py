"""The scorer for the cup sleeve task. NumPy only: it reads one Frame per control step (task.py builds it from the
simulation; in a real trial the scorer reads the same quantities off the video, a ruler and the cup-stiffness rule).

Zero for the rest of the trial, at any moment: either hand squeezes the cup across its wall harder than SQUEEZE_N
(directly, or through the sleeve where the sleeve is pressed on the cup); the cup tilts more than TIP_DEG; the cup
falls off the counter.

At the end: 1.0 when the sleeve is round and snug on the cup (every strip's middle line within SNUG_M of the wall), its
lower edge LOW_BAND above the cup's base all round and level to within LEVEL_M, the cup stands upright on the counter
(tilt under UPRIGHT_DEG, base within 3 mm of the counter), and the grippers are at least 3 cm from the cup and sleeve
(the harness then needs it held 1 s). 0.5 when the sleeve goes round the cup and the cup stands, but the rest is
not met (or the hands are not clear). 0.25 when the sleeve has been opened at some moment (OPEN_M both ways) but is
not round the standing cup. 0 otherwise.
"""

from __future__ import annotations

from dataclasses import dataclass, field

SQUEEZE_N = 6.0
TIP_DEG = 30.0
UPRIGHT_DEG = 5.0
LOW_BAND = (0.015, 0.040)
LEVEL_M = 0.010
SNUG_M = 0.004  # no strip's middle line more than this off the wall
SNUG_TOUCH_M = 0.001  # and at least SNUG_SHARE of them within this of it
SNUG_SHARE = 0.75
OPEN_M = 0.055
CLEAR_M = 0.030
BASE_M = 0.003


@dataclass
class Frame:
    t: float
    squeeze: dict  # "hands": the force across the wall from the hands together (N); and each hand's own
    cup_tilt_deg: float
    cup_base_z: float  # the cup's base above the counter (m)
    around: bool  # the sleeve loop goes round the cup's axis
    lows: list  # each strip's lower-edge middle, above the cup's base (m)
    gaps: list  # each strip's middle line, out from the cup wall (m)
    opened: float  # how far open the sleeve is (m)
    robot_dist: float  # nearest robot point to the cup or the sleeve (m)


@dataclass
class Scorer:
    squeezed: bool = False
    tipped: bool = False
    fell: bool = False
    opened: bool = False
    max_squeeze: float = 0.0
    last: dict = field(default_factory=dict)
    score: float = 0.0
    success: bool = False

    def update(self, f: Frame) -> None:
        sq = f.squeeze.get("hands", max(f.squeeze.values())) if f.squeeze else 0.0
        self.max_squeeze = max(self.max_squeeze, sq)
        if sq > SQUEEZE_N:
            self.squeezed = True
        if f.cup_tilt_deg > TIP_DEG:
            self.tipped = True
        if f.cup_base_z < -0.05:
            self.fell = True
        if f.opened >= OPEN_M:
            self.opened = True
        stands = f.cup_tilt_deg < UPRIGHT_DEG and abs(f.cup_base_z) < BASE_M
        on = bool(f.around)
        band = on and all(LOW_BAND[0] <= h <= LOW_BAND[1] for h in f.lows)
        level = on and (max(f.lows) - min(f.lows)) <= LEVEL_M
        snug = on and all(g <= SNUG_M for g in f.gaps) and \
            sum(g <= SNUG_TOUCH_M for g in f.gaps) >= SNUG_SHARE * len(f.gaps)
        clear = f.robot_dist >= CLEAR_M
        full = stands and on and band and level and snug
        self.last = {"squeeze": round(sq, 2), "stands": stands, "on": on, "band": band, "level": level, "snug": snug,
                     "clear": clear, "opened": self.opened, "low_mm": [round(1000 * min(f.lows), 1), round(1000 * max(f.lows), 1)],
                     "gap_mm": round(1000 * max(f.gaps), 1), "open_mm": round(1000 * f.opened, 1)}
        if self.squeezed or self.tipped or self.fell:
            self.score, self.success = 0.0, False
            return
        if full and clear:
            self.score, self.success = 1.0, True
        elif stands and on:
            self.score, self.success = 0.5, False
        elif self.opened:
            self.score, self.success = 0.25, False
        else:
            self.score, self.success = 0.0, False
