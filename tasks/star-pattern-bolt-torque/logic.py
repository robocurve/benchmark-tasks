"""Hidden-state logic for the star-pattern cold-plate bolt torque task.

Pure Python, no simulator dependencies. Fully unit-tested.

Rules:
- Four bolt corners: 'L_near', 'L_far', 'R_near', 'R_far'.
- Diagonal partner pairs:
  (L_near, R_far) and (L_far, R_near).
- Hidden state:
  * stage[corner] in 0..3 (starts at 0 for all 4 corners).
  * Rising edge of bolt button raises stage by 1 (counted <= -6.4 mm, re-armed >= -3.5 mm).
- Invariants (any violation immediately latches WARP, lamp red, permanent score 0):
  (1) LEVEL: after press, max(stage) - min(stage) <= 1.
  (2) STAR ORDER: presses come in groups of four (a pass). A pass is a sequence p1, p2, p3, p4
      of four distinct corners with p2 = diagonal partner of p1 and p4 = diagonal partner of p3
      (each diagonal pair is 1 left, 1 right). Neighbouring corners are never pressed consecutively
      within a pair. Each corner is pressed once per pass.
  (3) NO OVER-TORQUE: press on bolt already at stage 3 latches WARP.
- When all four stages are 3 and no WARP: status lamp GREEN (TORQUED).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence

import geometry as g

CORNERS: tuple[str, ...] = g.CORNERS
DIAGONAL_PARTNER: dict[str, str] = g.DIAGONAL_PARTNER

# Button thresholds along -Z joint displacement (travel 8 mm = 0.008 m)
PRESS_THRESHOLD = g.BUTTON_PRESS_THRESHOLD    # -0.0064 m (80% travel)
RELEASE_THRESHOLD = g.BUTTON_RELEASE_THRESHOLD  # -0.0035 m (hysteresis re-arm)


@dataclass(frozen=True)
class LogicState:
    stages: dict[str, int]
    warp: bool
    warp_reason: str | None
    status_lamp: str  # "off", "red", "green"
    completed_passes: int
    current_pass: list[str]
    cyan_lamps: dict[str, int]  # corner -> stage (0..3)
    last_press: str | None


class StarPatternLogic:
    def __init__(self) -> None:
        self.stages: dict[str, int] = {c: 0 for c in CORNERS}
        self.warp: bool = False
        self.warp_reason: str | None = None
        self.completed_passes: int = 0
        self.current_pass: list[str] = []
        self.history: list[str] = []
        self.status_lamp: str = "off"
        self._pressed: dict[str, bool] = {c: False for c in CORNERS}
        self._last_state: LogicState | None = None

    def reset(self) -> None:
        self.stages = {c: 0 for c in CORNERS}
        self.warp = False
        self.warp_reason = None
        self.completed_passes = 0
        self.current_pass = []
        self.history = []
        self.status_lamp = "off"
        self._pressed = {c: False for c in CORNERS}
        self._last_state = None

    def register_press(self, corner: str) -> None:
        """Process a counted rising-edge press on corner."""
        if corner not in CORNERS:
            raise ValueError(f"Unknown corner: {corner}")

        if self.warp:
            return

        # Rule (3) NO OVER-TORQUE: press on bolt at stage 3 latches WARP
        if self.stages[corner] >= 3:
            self.warp = True
            self.warp_reason = f"OVER_TORQUE: press on bolt {corner} already at stage 3"
            self.status_lamp = "red"
            return

        # Rule (2) STAR ORDER: sequence p1, p2, p3, p4 within each pass of 4
        k = len(self.current_pass)
        if k == 0:
            # p1: first bolt of the pass, always allowed
            pass
        elif k == 1:
            # p2: must be diagonal partner of p1
            p1 = self.current_pass[0]
            if corner != DIAGONAL_PARTNER[p1]:
                self.warp = True
                self.warp_reason = f"STAR_ORDER: p2 ({corner}) is not diagonal partner of p1 ({p1})"
                self.status_lamp = "red"
                return
        elif k == 2:
            # p3: must be a new corner (not p1 or p2)
            if corner in self.current_pass[:2]:
                self.warp = True
                self.warp_reason = f"STAR_ORDER: p3 ({corner}) already pressed in pass"
                self.status_lamp = "red"
                return
        elif k == 3:
            # p4: must be diagonal partner of p3
            p3 = self.current_pass[2]
            if corner != DIAGONAL_PARTNER[p3]:
                self.warp = True
                self.warp_reason = f"STAR_ORDER: p4 ({corner}) is not diagonal partner of p3 ({p3})"
                self.status_lamp = "red"
                return
            if corner in self.current_pass[:3]:
                self.warp = True
                self.warp_reason = f"STAR_ORDER: p4 ({corner}) already pressed in pass"
                self.status_lamp = "red"
                return

        # Increment stage
        self.stages[corner] += 1
        self.current_pass.append(corner)
        self.history.append(corner)

        # Rule (1) LEVEL: after press, max(stage) - min(stage) <= 1
        max_s = max(self.stages.values())
        min_s = min(self.stages.values())
        if max_s - min_s > 1:
            self.warp = True
            self.warp_reason = f"LEVEL: max stage {max_s} - min stage {min_s} > 1"
            self.status_lamp = "red"
            return

        # If 4th press of pass succeeded, pass is complete!
        if len(self.current_pass) == 4:
            self.completed_passes += 1
            self.current_pass = []

        # Check TORQUED completion
        if not self.warp and all(s == 3 for s in self.stages.values()):
            self.status_lamp = "green"
        elif self.warp:
            self.status_lamp = "red"
        else:
            self.status_lamp = "off"

    def update(self, slider_displacements: Mapping[str, float]) -> LogicState:
        """Update logic with current slider positions (metres, <= 0)."""
        last_press = None
        # Detect rising edge per corner (sorted for deterministic multi-trigger resolution)
        for c in CORNERS:
            q = float(slider_displacements.get(f"btn_{c}", 0.0))
            if q <= PRESS_THRESHOLD:
                if not self._pressed[c]:
                    self._pressed[c] = True
                    self.register_press(c)
                    last_press = c
            elif q >= RELEASE_THRESHOLD:
                self._pressed[c] = False

        state = LogicState(
            stages=dict(self.stages),
            warp=self.warp,
            warp_reason=self.warp_reason,
            status_lamp=self.status_lamp,
            completed_passes=self.completed_passes,
            current_pass=list(self.current_pass),
            cyan_lamps=dict(self.stages),
            last_press=last_press,
        )
        self._last_state = state
        return state
