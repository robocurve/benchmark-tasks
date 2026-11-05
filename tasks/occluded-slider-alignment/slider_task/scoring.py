"""Deterministic evaluation only. No arm role, camera pose or gesture enters the score."""

from dataclasses import dataclass, asdict
import math

from .layout import TARGET, TRAVEL


@dataclass(frozen=True)
class Measurement:
    displacement: float
    speed: float = 0.0
    lateral_error: float = 0.0
    angular_error: float = 0.0
    fixture_error: float = 0.0
    protected_contact_s: float = 0.0  # continuous prohibited contact, independent of force
    penetration: float = 0.0  # diagnostic only: solver penetration, m
    peak_tab_force: float = 0.0  # diagnostic only: normal force, N
    instrumented: bool = True
    released: bool = False  # no robot-slider contact; independent of arm role or jaw pose


class Evaluator:
    """One update per 20 Hz action; reset creates a fresh safety latch and hold counter."""

    def __init__(self, hz=20):
        if hz != 20:
            raise ValueError('This fixed prototype is validated for 20 Hz only')
        self.hz = hz
        self.reset()

    def reset(self):
        self.held_steps = 0
        self.invalid_reasons = set()
        self.last = None

    def update(self, m: Measurement):
        values = [v for v in asdict(m).values() if not isinstance(v, bool)]
        if not all(math.isfinite(v) for v in values):
            self.invalid_reasons.add('nonfinite_state')
        if not m.instrumented:
            self.invalid_reasons.add('missing_contact_instrumentation')
        if not -.001 <= m.displacement <= TRAVEL + .001:
            self.invalid_reasons.add('joint_limit')
        if m.lateral_error > .002 or m.angular_error > math.radians(2):
            self.invalid_reasons.add('guide_constraint')
        if m.fixture_error > .002:
            self.invalid_reasons.add('fixture_displaced')
        if m.protected_contact_s >= .100 - 1e-9:
            self.invalid_reasons.add('prohibited_collision')
        error = abs(m.displacement - TARGET)
        safe = not self.invalid_reasons
        resting = abs(m.speed) <= .005
        aligned = safe and resting and m.released and error <= .008 + 1e-12
        self.held_steps = self.held_steps + 1 if aligned else 0
        complete = self.held_steps >= self.hz
        score = 0.0
        if safe and resting:
            if complete:
                score = 1.0
            elif error <= .015 + 1e-12:
                score = .75
            elif error <= .025 + 1e-12:
                score = .50
            elif m.displacement >= .015 - 1e-12:
                score = .25
        self.last = dict(score=score, complete=complete, error_m=error,
                         held_steps=self.held_steps, invalid_reasons=sorted(self.invalid_reasons),
                         measurement=asdict(m))
        return self.last
