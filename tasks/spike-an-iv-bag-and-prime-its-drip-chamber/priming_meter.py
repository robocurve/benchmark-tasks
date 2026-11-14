"""Pure state model for measured drip-chamber fill height.

The simulator cannot resolve liquid flow.  Instead of granting a fixed volume for a
nominal squeeze, this meter integrates the *measured gripper compression* during a
complete squeeze/release cycle and converts it to a liquid-column height.  Scoring
reads that height in metres.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PrimingMeter:
    usable_height: float = 0.039
    target_min: float = 0.015
    target_max: float = 0.027
    spill_height: float = 0.034
    # The measured YAM joint is negative when open and approaches zero when
    # closed.  Interior thresholds reject endpoint servo noise.
    open_finger: float = -0.035
    closed_finger: float = -0.012
    full_squeeze_rise: float = 0.011
    fill_height: float = 0.0
    max_finger: float | None = None
    armed: bool = False
    spilled: bool = False

    def observe(self, finger: float, *, near: bool, locked: bool) -> None:
        """Observe one control step and update height on a completed release.

        A squeeze is accepted only while the hand is near the chamber and the spike
        is locked.  The height increment is proportional to measured finger travel,
        capped at one calibrated full squeeze.  Moving away abandons an incomplete
        cycle, so closing the gripper elsewhere cannot prime the chamber.
        """
        if not (near and locked):
            self.max_finger = None
            self.armed = False
            return
        # Require a measured open baseline after entering the valid chamber
        # region.  Releasing the insertion grasp must not masquerade as a
        # completed squeeze.
        if not self.armed:
            if finger <= self.open_finger:
                self.armed = True
            return
        if finger > self.open_finger:
            self.max_finger = finger if self.max_finger is None else max(self.max_finger, finger)
            return
        if self.max_finger is not None and finger <= self.open_finger:
            travel = max(0.0, self.max_finger - self.open_finger)
            full_travel = self.closed_finger - self.open_finger
            rise = self.full_squeeze_rise * min(1.0, travel / full_travel)
            self.fill_height = min(self.usable_height, self.fill_height + rise)
            self.spilled = self.spilled or self.fill_height > self.spill_height
            self.max_finger = None

    @property
    def primed(self) -> bool:
        return self.target_min <= self.fill_height <= self.target_max and not self.spilled

