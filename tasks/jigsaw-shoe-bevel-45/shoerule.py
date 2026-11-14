"""The one declared software model: the shoe's 0 degree positive stop, its detents and the lever's clamp, as drive
targets on the saw's three joints. Plain Python and NumPy; task.py writes the targets through mechkit every control
step, the tests and the offline model call the same function.

Joints (mechkit, SI): ``slide`` (m, 0 at the stop, forward positive, 0 to 10 mm), ``bevel`` (rad, the shoe's turn
about the blade's line, -45 to 45 deg), ``lever`` (rad, 0 flush under the body, out to 70 deg).

- LEVER: a cam with an over-centre spring. Under 25 deg the target pulls it toward flush, above toward 60 deg, each
  with at most 0.05 N m (LEVER_K x LEVER_PULL), so a finger moves it anywhere and it stays out once pulled out.
- CLAMPED (lever under 10 deg): the slide and the bevel are held where they were when it clamped (stiff springs:
  10,000 N/m and 20 N m/rad).
- FREE (lever 10 deg or more): the slide follows where it is pushed (its target is its own position led by 0.9 of its
  last step's travel, the pliers task's GPU-tested drag); while it is less than 6 mm forward and the shoe within
  2 deg of square, the bevel is held at 0 (the positive stop); once the shoe is 6 mm forward (or already tilted) the
  bevel follows the same way, with a detent pull of at most 0.03 N m within 3 deg of 0, 15, 30 and 45 either way.
  A tilted shoe cannot slide back past 6 mm (it is off the stop until it is square again).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import layout as L


@dataclass
class Memory:
    mode: str = "clamped"
    frozen_slide: float = 0.0
    frozen_bevel: float = 0.0
    prev_slide: float | None = None
    prev_bevel: float | None = None
    switches: int = 0

    def reset(self) -> None:
        self.mode, self.frozen_slide, self.frozen_bevel = "clamped", 0.0, 0.0
        self.prev_slide = self.prev_bevel = None
        self.switches = 0


def _clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def lever_target(q_lever: float) -> float:
    notch = 0.0 if math.degrees(q_lever) < L.LEVER_SNAP_DEG else math.radians(L.LEVER_OPEN_DEG)
    return q_lever + _clip(notch - q_lever, -L.LEVER_PULL, L.LEVER_PULL)


def clamped(q_lever: float) -> bool:
    return math.degrees(q_lever) < L.LOCK_DEG


def off_stop(q_slide: float, q_bevel: float) -> bool:
    """The shoe is free to tilt: slid 6 mm forward, or already tilted."""
    return q_slide >= L.SLIDE_REL * L.MM - 1e-9 or abs(math.degrees(q_bevel)) > L.TILTED_DEG


def detent_pull(q_bevel: float) -> float:
    a = math.degrees(q_bevel)
    d = L.nearest_detent_deg(a)
    if abs(a - d) > L.DETENT_CAPTURE_DEG:
        return 0.0
    return _clip(math.radians(d) - q_bevel, -L.DETENT_PULL, L.DETENT_PULL)


def targets(q_slide: float, q_bevel: float, q_lever: float, mem: Memory) -> dict:
    """The three drive targets for this control step (SI) and the mode; updates ``mem``."""
    t_lever = lever_target(q_lever)
    if clamped(q_lever):
        if mem.mode != "clamped":
            mem.frozen_slide, mem.frozen_bevel = q_slide, q_bevel
            mem.switches += 1
        mem.mode = "clamped"
        mem.prev_slide = mem.prev_bevel = None
        return {"slide": mem.frozen_slide, "bevel": mem.frozen_bevel, "lever": t_lever, "mode": "clamped"}
    if mem.mode != "free":
        mem.switches += 1
        mem.prev_slide = mem.prev_bevel = None  # the clamp's last pull is not led on
    mem.mode = "free"
    lead_s = 0.0 if mem.prev_slide is None else _clip(L.FOLLOW_GAIN * (q_slide - mem.prev_slide),
                                                       -L.FOLLOW_LEAD_SLIDE, L.FOLLOW_LEAD_SLIDE)
    lo = L.SLIDE_REL * L.MM if abs(math.degrees(q_bevel)) > L.TILTED_DEG else 0.0
    t_slide = _clip(q_slide + lead_s, lo, L.SLIDE_TRAVEL * L.MM)
    if not off_stop(q_slide, q_bevel):
        t_bevel, mode = 0.0, "free_on_stop"
        mem.prev_bevel = None
    else:
        lead_b = 0.0 if mem.prev_bevel is None else _clip(L.FOLLOW_GAIN * (q_bevel - mem.prev_bevel),
                                                           -L.FOLLOW_LEAD_BEVEL, L.FOLLOW_LEAD_BEVEL)
        lim = math.radians(L.BEVEL_MAX_DEG)
        t_bevel = _clip(q_bevel + lead_b + detent_pull(q_bevel), -lim, lim)
        mode = "free"
        mem.prev_bevel = q_bevel
    mem.prev_slide = q_slide
    return {"slide": t_slide, "bevel": t_bevel, "lever": t_lever, "mode": mode}
