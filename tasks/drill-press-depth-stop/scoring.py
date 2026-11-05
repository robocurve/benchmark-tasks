"""Score independently loaded strokes of the depth-stop training fixture."""

from dataclasses import dataclass, field
import math

# Metres, seconds, and newtons.
FULL = (0.009, 0.011)
PARTIAL = (0.005, 0.015)
HOME = 0.002
LOAD_S = 1.0
RAMP_S = 0.5
HOLD_S = 0.25
RETURN_S = 1.0
CONTACT_TOL = 0.001
LOADS = (8.0, 12.0)


@dataclass(frozen=True)
class Sample:
    travel: float
    depth: float
    stop_gap: float
    speed: float
    latch: bool
    clear: bool
    pressed: bool


@dataclass
class Proof:
    """Press and release the separate test button for each stroke."""

    start: float | None = None
    button_was_down: bool = False
    readings: list[float] = field(default_factory=list)
    peaks: list[float] = field(default_factory=list)
    candidate: list[float] = field(default_factory=list)
    peak: float = 0.0
    verified_limit: float | None = None
    valid: bool = True
    invalid: bool = False
    force: float = 0.0

    def step(self, t: float, sample: Sample) -> float:
        if not all(math.isfinite(value) for value in (
            sample.travel, sample.depth, sample.stop_gap, sample.speed
        )):
            self.invalid = True
        edge = sample.pressed and not self.button_was_down
        self.button_was_down = sample.pressed
        limit = sample.depth + sample.stop_gap
        if self.readings and (
            not sample.latch or abs(limit - self.verified_limit) > 0.0015
        ):
            self.readings.clear()
            self.peaks.clear()
            self.valid = False
        if self.invalid or len(self.readings) == len(LOADS):
            self.force = 0.0
            return self.score
        if self.start is None and edge and sample.clear and abs(sample.travel) <= HOME:
            self.start = t
            self.valid = True
            self.candidate.clear()
            self.peak = sample.depth
        if self.start is None:
            return self.score

        age = t - self.start
        self.valid &= sample.clear and sample.latch
        if age <= LOAD_S + 1e-8:
            self.peak = max(self.peak, sample.depth)
        if age < LOAD_S - 1e-8:
            self.force = LOADS[len(self.readings)] * min(1.0, age / RAMP_S)
        else:
            self.force = 0.0
        if LOAD_S - HOLD_S - 1e-8 <= age <= LOAD_S + 1e-8:
            settled = abs(sample.stop_gap) <= CONTACT_TOL and abs(sample.speed) <= 0.003
            self.valid &= settled
            self.candidate.append(sample.depth)
        if age >= LOAD_S + RETURN_S - 1e-8:
            # Invalid strokes may be retried. Only a completed, returned stroke
            # with continuously unassisted latch contact can contribute.
            if self.valid and abs(sample.travel) <= HOME and not sample.pressed and self.candidate:
                lower = min(self.candidate)
                if PARTIAL[0] <= lower <= self.peak <= PARTIAL[1]:
                    self.readings.append(lower)
                    self.peaks.append(self.peak)
                    self.verified_limit = limit
                else:
                    self.readings.clear()
                    self.peaks.clear()
            else:
                self.readings.clear()
                self.peaks.clear()
            self.start = None
        return self.score

    @property
    def score(self) -> float:
        if self.invalid or len(self.readings) != len(LOADS):
            return 0.0
        if all(depth >= FULL[0] and peak <= FULL[1]
               for depth, peak in zip(self.readings, self.peaks, strict=True)):
            if abs(self.readings[1] - self.readings[0]) <= 0.001:
                return 1.0
        return 0.5
