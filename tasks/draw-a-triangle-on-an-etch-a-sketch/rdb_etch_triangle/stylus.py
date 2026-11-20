"""Where the stylus is, from the two knob angles: the toy's string drive, written out as a rule.

Inside the real toy each knob winds a drum; strings over pulleys pull the stylus along two rails. Here the
drive is kinematic: each knob angle passes through a dead band (the slack of the strings, 20 degrees) to a drum
angle, and the stylus moves 35.5 mm per drum turn. Turning the left knob clockwise, seen from above, moves the
stylus to the right; turning the right knob clockwise moves it up. At the ends of its travel the stylus stops
and the strings slip, as on the real toy (the knobs keep turning). Nothing else in the simulation reads the
stylus: it is behind the glass. NumPy only.
"""

from dataclasses import dataclass, field

import numpy as np

from . import layout

TRAVEL_LO = np.asarray(layout.TRAVEL_LL) - np.asarray(layout.START)          # travel limits in the drawing frame
TRAVEL_HI = TRAVEL_LO + np.asarray(layout.TRAVEL)


@dataclass
class Stylus:
    """Knob angles (left, right) in radians, counter-clockwise seen from above, unwrapped."""
    knobs: np.ndarray = field(default_factory=lambda: np.zeros(2))
    drums: np.ndarray = field(default_factory=lambda: np.zeros(2))        # angles after the dead band
    point: np.ndarray = field(default_factory=lambda: np.zeros(2))        # metres from the start dot: right, up
    backlash: float = layout.BACKLASH

    def update(self, knobs):
        knobs = np.asarray(knobs, dtype=float)
        drums = np.clip(self.drums, knobs - self.backlash / 2, knobs + self.backlash / 2)
        moved = -(drums - self.drums) * layout.PER_RAD                    # clockwise (negative) moves right / up
        self.point = np.clip(self.point + moved, TRAVEL_LO, TRAVEL_HI)
        self.drums = drums
        self.knobs = knobs.copy()
        return self.point.copy()

    def engaged(self, side, direction):
        """Whether turning knob ``side`` (0 left, 1 right) by ``direction`` (+1 counter-clockwise) moves the stylus
        at once, that is, the slack is already taken up on that side."""
        return bool(direction * (self.knobs[side] - self.drums[side]) >= self.backlash / 2 - 1e-6)


def unwrap(previous, wrapped):
    """Continue an angle reading that the simulator reports in (-pi, pi]."""
    return previous + (wrapped - previous + np.pi) % (2 * np.pi) - np.pi
