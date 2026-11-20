"""The electrolyte as events, not a fluid. Pure Python: fed once per control step with what the pipette's tip and plunger
are doing, it keeps how much is in the tip, on the separator and spilled.

- Load: the plunger, held at the first stop, is let back up to rest while the tip stays at least 2 mm under the
  electrolyte's surface in the tube the whole way: the tip now holds 80 uL. If the tip leaves the liquid on the way up,
  it draws air: nothing is loaded.
- Dispense: with liquid in the tip, the plunger reaches the first stop. If the tip is 1 to 6 mm above the separator's top
  and within 3 mm of the can's axis, and the separator lies in the can, the separator is wetted: 80 uL of 2 M ZnSO4
  (1.31 g/mL), 0.105 g, goes into the can. Anywhere else, the same mass is spilled.
"""

from __future__ import annotations

from dataclasses import dataclass, field

FIRST_STOP = 0.0060  # m of plunger travel
STOP_TOL = 0.0003
REST_TOL = 0.0008
DOSE_UL = 80.0
DENSITY = 1.31  # g/mL, 2 M ZnSO4 at 20 C
DOSE_G = DOSE_UL * 1e-3 * DENSITY  # 0.1048 g
LOAD_DEPTH = 0.002  # m under the surface
WINDOW_Z = (0.001, 0.006)  # m above the separator's top
WINDOW_R = 0.003  # m from the can's axis


@dataclass
class Pipetting:
    tip_ul: float = 0.0
    can_g: float = 0.0
    spilled_g: float = 0.0
    loads: int = 0
    dispenses: list = field(default_factory=list)  # (time, where, grams)
    _held: bool = False  # the plunger is at the first stop (or past it)
    _drawing: bool = False  # on its way up from the first stop, unloaded
    _wet_all_way: bool = True
    _pressed_dispensed: bool = False

    def update(self, t: float, travel: float, tip_depth: float | None, window: tuple | None, sep_in_can: bool) -> None:
        """`travel`: plunger travel (m, 0 at rest). `tip_depth`: how far the tip's end is under the electrolyte's surface
        in the tube (m; None when not in the tube). `window`: (height above the separator's top, distance from the can's
        axis), m. `sep_in_can`: the separator lies in the can."""
        at_stop = travel >= FIRST_STOP - STOP_TOL
        at_rest = travel <= REST_TOL
        if at_stop and not self._held:
            self._held = True
            self._drawing = False
            if self.tip_ul > 0.0:
                self._dispense(t, window, sep_in_can)
        if self._held and not at_stop:
            self._held = False
            if self.tip_ul == 0.0:
                self._drawing = True
                self._wet_all_way = True
        if self._drawing:
            if tip_depth is None or tip_depth < LOAD_DEPTH:
                self._wet_all_way = False
            if at_rest:
                self._drawing = False
                if self._wet_all_way:
                    self.tip_ul = DOSE_UL
                    self.loads += 1

    def _dispense(self, t: float, window, sep_in_can: bool) -> None:
        grams = self.tip_ul * 1e-3 * DENSITY
        self.tip_ul = 0.0
        ok = window is not None and sep_in_can and WINDOW_Z[0] <= window[0] <= WINDOW_Z[1] and window[1] <= WINDOW_R
        if ok:
            self.can_g += grams
            self.dispenses.append((round(t, 2), "separator", round(grams, 4)))
        else:
            self.spilled_g += grams
            self.dispenses.append((round(t, 2), "spill", round(grams, 4)))

    @property
    def wet(self) -> bool:
        return self.can_g > 0.0
