"""Hidden-state logic for the servo bus commissioning task.

Pure Python, no simulator dependencies. Fully unit-tested.

Rules:
- Three servos: 'red', 'green', 'blue'.
- Each servo has an assigned ID: None, 1, 2, or 3.
- All servos start with ID = None (un-numbered).
- A servo is either seated in a bus socket or not seated.
- The bus lamp is RED whenever two or more un-numbered servos are seated on the bus (bus ID clash).
  Otherwise (at most one un-numbered servo seated on the bus), the bus lamp is GREEN.
- ASSIGN button (rising edge):
  If exactly one un-numbered servo is seated on the bus, it receives ID = n + 1 (where n is the count of
  IDs assigned so far, starting from 0). If 0 or >= 2 un-numbered servos are seated, ASSIGN does nothing.
- RESET button (rising edge):
  Clears all servo IDs to None and resets the assignment counter to 0.
- Readout lamps: For each servo, if it has an ID j in {1, 2, 3}, lamp (servo, j) is illuminated.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence

SERVOS: tuple[str, ...] = ("red", "green", "blue")

# Button thresholds along -Z joint displacement (travel 8 mm = 0.008 m)
PRESS_THRESHOLD = -0.0064  # 80% of travel (negative position)
RELEASE_THRESHOLD = -0.0016  # 20% of travel (hysteresis)


@dataclass(frozen=True)
class LogicState:
    ids: dict[str, int | None]
    counter: int
    bus_lamp: str  # "red" or "green"
    readout: dict[str, int | None]  # servo -> lit column (1, 2, 3 or None)
    assign_event: bool
    reset_event: bool


class CommissioningLogic:
    def __init__(self) -> None:
        self.ids: dict[str, int | None] = {s: None for s in SERVOS}
        self.counter: int = 0
        self._assign_pressed: bool = False
        self._reset_pressed: bool = False
        self._last_state: LogicState | None = None

    def reset(self) -> None:
        self.ids = {s: None for s in SERVOS}
        self.counter = 0
        self._assign_pressed = False
        self._reset_pressed = False
        self._last_state = None

    def update(
        self,
        seated: Mapping[str, bool],
        assign_slider_q: float,
        reset_slider_q: float,
    ) -> LogicState:
        """Update logic with current seated states and slider joint positions (metres, <= 0)."""
        assign_event = False
        reset_event = False

        # ASSIGN edge detection with hysteresis
        if assign_slider_q <= PRESS_THRESHOLD:
            if not self._assign_pressed:
                assign_event = True
                self._assign_pressed = True
        elif assign_slider_q >= RELEASE_THRESHOLD:
            self._assign_pressed = False

        # RESET edge detection with hysteresis
        if reset_slider_q <= PRESS_THRESHOLD:
            if not self._reset_pressed:
                reset_event = True
                self._reset_pressed = True
        elif reset_slider_q >= RELEASE_THRESHOLD:
            self._reset_pressed = False

        # Process RESET first if both triggered in same step
        if reset_event:
            self.ids = {s: None for s in SERVOS}
            self.counter = 0

        # Process ASSIGN
        if assign_event:
            unassigned_seated = [s for s in SERVOS if seated.get(s, False) and self.ids[s] is None]
            if len(unassigned_seated) == 1 and self.counter < len(SERVOS):
                target = unassigned_seated[0]
                self.counter += 1
                self.ids[target] = self.counter

        # Evaluate bus lamp: red if >= 2 unassigned servos seated, otherwise green
        unassigned_seated_count = sum(1 for s in SERVOS if seated.get(s, False) and self.ids[s] is None)
        bus_lamp = "red" if unassigned_seated_count >= 2 else "green"

        state = LogicState(
            ids=dict(self.ids),
            counter=self.counter,
            bus_lamp=bus_lamp,
            readout=dict(self.ids),
            assign_event=assign_event,
            reset_event=reset_event,
        )
        self._last_state = state
        return state
