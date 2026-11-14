"""Reduced-order, volume-conserving air-pocket model; not CFD or a clinical model.
Inputs are measured piston displacement and the barrel axis's vertical component.
Air rises axially at a bounded speed. An open tip expels whichever phase is at it.
Pulling the piston admits air. No frame count or scripted stage can clear a bubble.
"""
from dataclasses import dataclass
import math

AREA = math.pi * 0.0075**2 * 1e6  # mL / m
CAPACITY = 12.0
INITIAL_AIR = 1.5
INITIAL_SAMPLE = CAPACITY - INITIAL_AIR

@dataclass
class Fluid:
    air: float = INITIAL_AIR
    front_air: float = 0.0  # newly aspirated air at the outlet, separate from the trapped pocket
    sample: float = INITIAL_SAMPLE
    pocket: float = 0.025  # distance from outlet to bubble's leading edge, metres
    last_q: float = 0.0
    expelled_air: float = 0.0
    lost_sample: float = 0.0
    aspirated: float = 0.0
    failed: bool = False

    def update(self, q, upward, dt):
        if not all(math.isfinite(v) for v in (q, upward, dt)) or dt <= 0:
            self.failed = True
            return
        q = min(max(q, 0.0), CAPACITY / AREA)
        dv = (q - self.last_q) * AREA
        self.last_q = q
        # A single coalesced pocket migrates toward the highest axial location.
        available = max(self.sample / AREA, 0.0)
        self.pocket = min(available, max(0., self.pocket - 0.025 * upward * dt))
        if dv < 0:
            self.air -= dv
            self.aspirated -= dv
            self.front_air -= dv
        elif dv > 0:
            # Aspirated air leaves first; it cannot teleport the original pocket.
            da = min(dv, self.front_air)
            self.front_air -= da; self.air -= da; self.expelled_air += da; dv -= da
            # Liquid between the original pocket and outlet, then trapped air, then liquid.
            front = min(dv, self.pocket * AREA, self.sample)
            self.sample -= front; self.lost_sample += front; dv -= front
            self.pocket = max(0., self.pocket - front / AREA)
            if self.pocket < 1e-8:
                da = min(dv, self.air)
                self.air -= da; self.expelled_air += da; dv -= da
            ds = min(dv, self.sample)
            self.sample -= ds; self.lost_sample += ds
        # Corrective retraction is recoverable: newly admitted air must be purged
        # again, but aspiration history alone does not invalidate a good endpoint.
        # Keep the counter for conservation checks and diagnostics only.
        if self.lost_sample > 0.50 + 1e-8:
            self.failed = True

    def score(self, released=False, parked=False, lifted=False):
        if self.failed or not lifted or not released or not parked:
            return 0.0
        return self.volume_score()

    def volume_score(self):
        """Phase-quality score only; the task must separately enforce its physical endpoint."""
        if self.failed:
            return 0.0
        if self.air <= 0.05 and self.lost_sample <= 0.30:
            return 1.0
        if self.air <= 0.30 and self.lost_sample <= 0.50:
            return 0.6
        if self.air <= 0.75 and self.lost_sample <= 0.50:
            return 0.3
        return 0.0
