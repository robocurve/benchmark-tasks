"""Evaluate observed cart states in time order. Distances are metres."""
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class Observation:
    # Separation in the direction of withdrawal, from the entire moving envelope.
    worker_gap: float
    left_in_position: bool
    right_in_position: bool
    release_open: bool
    deck_height: float
    # Physical bearing contact at the two factory stop positions.
    left_bearing: bool
    right_bearing: bool
    # Ram force unloaded or equivalent native loss of hydraulic support.
    ram_unloaded: bool
    # Hands cannot prop up the platform during the load-bearing proof.
    external_support: bool
    worker_struck: bool = False


class SupportSequence:
    """Call once per physics sample, beginning with a settled pressurized reset."""

    def __init__(self, reset_height, *, worker_margin=0.02, motion_tol=0.002,
                 dwell_seconds=1.0, drift_tol=0.002):
        if not isfinite(reset_height) or min(worker_margin, motion_tol, dwell_seconds, drift_tol) <= 0:
            raise ValueError("A finite height and positive tolerances are required")
        self.peak_height = reset_height
        self.worker_margin = worker_margin
        self.motion_tol = motion_tol
        self.dwell_seconds = dwell_seconds
        self.drift_tol = drift_tol
        self.ready_before = False
        self.open_before = False
        self.released = False
        self.fault = ""
        self.stable_s = 0.0
        self.low = self.high = reset_height
        self.score = 0.0

    def observe(self, sample, dt):
        if not all(isfinite(x) for x in (dt, sample.deck_height, sample.worker_gap)) or dt <= 0:
            raise ValueError("Incomplete cart measurement")
        if self.fault:
            return 0.0
        bars = sample.left_in_position and sample.right_in_position
        clear = sample.worker_gap >= self.worker_margin
        ready = bars and clear
        self.peak_height = max(self.peak_height, sample.deck_height)
        descended = self.peak_height - sample.deck_height > self.motion_tol
        opening = sample.release_open and not self.open_before
        if sample.worker_struck:
            self.fault = "worker_struck"
        elif opening and not (ready and self.ready_before):
            self.fault = "premature_release"
        elif (descended or sample.release_open) and not ready:
            self.fault = "unprotected_motion_or_release"
        elif descended and not self.released and not opening:
            self.fault = "uncommanded_descent"
        if self.fault:
            self.score = 0.0
            return self.score
        self.released |= opening
        self.open_before = sample.release_open
        self.ready_before = ready
        # Partial credit is the current safe state, not a historical milestone.
        self.score = 0.5 if bars and not self.released and not descended else 0.0
        bearing = (self.released and ready and descended and sample.ram_unloaded
                   and sample.left_bearing and sample.right_bearing and not sample.external_support)
        if bearing:
            if self.stable_s == 0:
                self.low = self.high = sample.deck_height
            self.low = min(self.low, sample.deck_height)
            self.high = max(self.high, sample.deck_height)
            if self.high - self.low > self.drift_tol:
                self.low = self.high = sample.deck_height
                self.stable_s = 0.0
            self.stable_s += dt
            if self.stable_s + 1e-9 >= self.dwell_seconds:
                self.score = 1.0
        else:
            self.stable_s = 0.0
        return self.score
