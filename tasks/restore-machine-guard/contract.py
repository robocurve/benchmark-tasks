"""Independent, dependency-free primary scorer for a genuine hinged-guard fixture.

Evidence is observed native joint state + observed arm bodies, never the planner's
commanded goal. Failing to provide a required channel is NOT interpreted as a
safe zero. A physically engaged interlock still needs a native USD audit.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

import geometry as g


@dataclass(frozen=True)
class Sample:
    time_s: float
    hinge_rad: float
    bolt_m: float
    switch_m: float
    clear_of_guard: bool
    protected_volume_clear: bool
    all_link_poses_valid: bool
    access_violated: bool = False


class GuardScorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.count = 0
        self.failed = None
        self.last_time = None
        self.closed_seen = False
        self.switch_seen = False
        self.latched_seen = False
        self.stable_ticks = 0
        self.complete = False
        self.last = None
        self.log = []

    def fail(self, why):
        if self.failed is None:
            self.failed = why

    def observe(self, s: Sample):
        required = (s.time_s, s.hinge_rad, s.bolt_m, s.switch_m)
        if not all(isinstance(v, (float, int)) and math.isfinite(v) for v in required):
            self.fail('missing_or_nonfinite_telemetry')
            return self.score()
        if not s.all_link_poses_valid:
            self.fail('missing_robot_link_geometry')
        if not s.protected_volume_clear or s.access_violated:
            self.fail('entered_protected_machinery_volume')
        if not g.within_joint_limits(s.hinge_rad, s.bolt_m, s.switch_m):
            self.fail('mechanical_joint_limit')
        if self.last_time is not None:
            dt = s.time_s - self.last_time
            if dt <= 0 or dt > .25:
                self.fail('clock_jump_or_duplicate_sample')
            else:
                if self.last is not None:
                    if abs(s.hinge_rad - self.last.hinge_rad)/dt > 2.6:
                        self.fail('excessive_guard_angular_speed')
                    if abs(s.bolt_m - self.last.bolt_m)/dt > .25:
                        self.fail('excessive_latch_translation_speed')
        self.last_time = s.time_s
        self.last = s
        self.count += 1
        # A forced switch actuation while the guard is appreciably open is a
        # bypass. An already engaged latch that ceases to retain the door is
        # another irreversible failure even if the door closes later.
        if s.switch_m >= g.SWITCH_ACTIVE and abs(s.hinge_rad) > g.UNSAFE_SWITCH_OPEN_RAD:
            self.fail('interlock_bypassed_with_guard_open')
        if self.latched_seen and (abs(s.hinge_rad) > math.radians(9)):
            self.fail('latched_guard_reopened')
        if self.latched_seen and s.bolt_m < g.BOLT_SEATED - 0.005:
            self.fail('latch_disengaged_after_engagement')
        if self.latched_seen and s.switch_m < g.SWITCH_RELEASE:
            self.fail('interlock_lost_after_engagement')
        closed = abs(s.hinge_rad) <= math.radians(2.5)
        switch = closed and s.switch_m >= g.SWITCH_ACTIVE
        latched = switch and s.bolt_m >= g.BOLT_SEATED
        self.closed_seen |= closed
        self.switch_seen |= switch
        self.latched_seen |= latched
        # Any unverified contact/release or movement resets the success dwell.
        if latched and s.clear_of_guard and s.protected_volume_clear and self.failed is None:
            self.stable_ticks += 1
        else:
            self.stable_ticks = 0
        # True elapsed time rather than count alone is independently checked.
        self.log.append((s.time_s, closed, switch, latched, s.clear_of_guard))
        if len(self.log) > g.CLEAR_TICKS:
            self.log.pop(0)
        self.complete = False
        if (self.stable_ticks >= g.CLEAR_TICKS
            and len(self.log) >= g.CLEAR_TICKS
            and self.log[-1][0] - self.log[-g.CLEAR_TICKS][0] >= g.INTERLOCK_HOLD_S - 1e-5
            and self.failed is None):
            self.complete = True
        return self.score()

    def score(self):
        if self.failed is not None:
            return 0.0
        if self.complete:
            return 1.0
        if self.latched_seen:
            return 0.75
        if self.switch_seen:
            return 0.50
        if self.closed_seen:
            return 0.25
        return 0.0

    def diagnostic(self):
        return dict(score=self.score(), failed=self.failed, count=self.count,
                    closed=self.closed_seen, switch=self.switch_seen,
                    latched=self.latched_seen, dwell=self.stable_ticks,
                    complete=self.complete)
