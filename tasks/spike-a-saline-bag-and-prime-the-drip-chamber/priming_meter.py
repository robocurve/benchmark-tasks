"""Pure state model for the drip chamber's liquid height, driven by measured compression of its squeezable wall.

The simulator cannot resolve liquid flow. The drip chamber's flexible wall is two sprung side panels (see ``task.py``);
squeezing the chamber pushes both panels in, and on release the spring pushes them back out while saline is drawn up
from the bag. This meter integrates that: each completed squeeze-and-release cycle raises the liquid in proportion to
how far the panels were pushed in (their mean travel, counted only once both have moved at least 1 mm), up to one full
squeeze. A cycle ends only when both panels are back at rest. Nothing else moves the panels: a gripper closing near the
chamber, or pressing on one side only, adds nothing. Scoring reads the height in m.
"""

from __future__ import annotations

from dataclasses import dataclass


def squeeze_of(travels, both_sides: float = 0.001) -> float:
    """The squeeze on the chamber from both panels' inward travels (m): their mean, but only once each panel has been
    pushed in at least ``both_sides``; pressing one side, or a light brush, is no squeeze."""
    a, b = (float(t) for t in travels)
    return (a + b) / 2 if min(a, b) >= both_sides else 0.0


@dataclass
class PrimingMeter:
    usable_height: float = 0.039
    target_min: float = 0.015
    target_max: float = 0.027
    spill_height: float = 0.034
    open_below: float = 0.0005  # m: both panels back within this of rest counts as released
    both_sides: float = 0.001  # m: each panel pushed in at least this far for a squeeze to count
    full_squeeze: float = 0.004  # m: panel travel of one full squeeze (the panels' stop)
    full_squeeze_rise: float = 0.011  # m: liquid rise for one full squeeze
    fill_height: float = 0.0
    max_compression: float | None = None
    armed: bool = False
    spilled: bool = False

    def observe(self, travels, *, valid: bool) -> None:
        """Observe one control step.

        ``travels`` is the measured inward travel of each wall panel (m). ``valid``: the spike is locked in the port and
        the chamber is upright with its liquid region below the port. Only then can a squeeze draw saline. A cycle needs
        an open baseline first (both panels at rest, so a squeeze held through the spike's locking, then released, does
        not count), a squeeze of both panels, and a release of both back to rest. Easing off one side while the other
        stays pressed is not a release. Leaving the valid state abandons the cycle.
        """
        released = max(float(t) for t in travels) <= self.open_below
        if not valid:
            self.max_compression = None
            self.armed = False
            return
        if not self.armed:
            self.armed = released
            return
        squeeze = squeeze_of(travels, self.both_sides)
        if squeeze > 0.0:
            self.max_compression = squeeze if self.max_compression is None else max(self.max_compression, squeeze)
            return
        if released and self.max_compression is not None:
            rise = self.full_squeeze_rise * min(1.0, self.max_compression / self.full_squeeze)
            self.fill_height = min(self.usable_height, self.fill_height + rise)
            self.spilled = self.spilled or self.fill_height > self.spill_height
            self.max_compression = None

    @property
    def primed(self) -> bool:
        return self.target_min <= self.fill_height <= self.target_max and not self.spilled
