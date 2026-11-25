"""Score the seated roller, a supported door-to-tub crossing and the released rack."""
from dataclasses import dataclass
import math


WHEEL_RADIUS_M = .017
BEFORE_HINGE_M = -.037
AFTER_HINGE_M = .038
ROLL_TOLERANCE_M = .014
DWELL_STEPS = 10


@dataclass(frozen=True)
class Reading:
    wheel_seated: bool
    wheel_clear: bool
    wheel_supported: bool
    rack_supported: bool
    axle_x_m: float
    spoke_angle_rad: float
    rack_in_tub: bool
    grippers_clear: bool
    valid: bool = True


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.valid = True
        self.installed_ticks = 0
        self.finish_ticks = 0
        self.crossing_start_x = None
        self.crossing_dx = 0.
        self.crossing_arc_m = 0.
        self.prior_x = None
        self.prior_spoke = None
        self.crossed = False
        self.score = 0.

    def update(self, reading):
        self.valid &= reading.valid and math.isfinite(reading.axle_x_m) and math.isfinite(reading.spoke_angle_rad)
        seated = self.valid and reading.wheel_seated and reading.wheel_clear
        self.installed_ticks = self.installed_ticks + 1 if seated else 0
        moving_on_track = seated and reading.wheel_supported and reading.rack_supported
        x = reading.axle_x_m
        if not moving_on_track:
            self.crossed = False
        if not moving_on_track or x <= BEFORE_HINGE_M:
            self.crossing_start_x = None
        if moving_on_track and x <= BEFORE_HINGE_M:
            self.crossing_start_x = x
            self.crossing_arc_m = self.crossing_dx = 0.
        elif self.crossing_start_x is not None:
            dx = x - self.prior_x
            dtheta = reading.spoke_angle_rad - self.prior_spoke
            roll_m = WHEEL_RADIUS_M * math.atan2(math.sin(dtheta), math.cos(dtheta))
            self.crossing_dx += dx
            self.crossing_arc_m += roll_m
            if abs(dx) > .022 or dx < -.006 or abs(self.crossing_dx - self.crossing_arc_m) > ROLL_TOLERANCE_M:
                self.crossing_start_x = None
            elif x >= AFTER_HINGE_M and self.crossing_dx >= AFTER_HINGE_M - BEFORE_HINGE_M:
                self.crossed = True
                self.crossing_start_x = None
        self.prior_x = x
        self.prior_spoke = reading.spoke_angle_rad
        full = self.crossed and moving_on_track and reading.rack_in_tub and reading.grippers_clear
        self.finish_ticks = self.finish_ticks + 1 if full else 0
        self.score = 0.
        if self.finish_ticks >= DWELL_STEPS:
            self.score = 1.
        elif self.crossed and moving_on_track:
            self.score = .5
        elif self.installed_ticks >= DWELL_STEPS and moving_on_track:
            self.score = .25
        return self.score
