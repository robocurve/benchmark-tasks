"""Current-state scoring and explicit integrity gates, independent of a policy."""
from dataclasses import dataclass
import math
from ratchet_task.config import (INITIAL_GAP,PITCH,SEAT_ROUNDOFF,SEAT_MAX_PENETRATION,
                                SEAT_MAX_EDGE_GAP,SEAT_MIN_FORCE,SEAT_CONTACT_FRACTION)
@dataclass
class State:
    captured: bool=False
    valid: bool=True
    turns: float=0.
    gap: float=1.
    interface_torque: float=0.
    socket_engaged: bool=False
    ratchet_cycles: int=0
    released: bool=False
    still: bool=False
    fixture_displacement: float=0.
    proof_seconds: float=0.
    seat_contact: bool=False
    seat_contact_fraction: float=0.
    seat_force_n: float=0.
    seat_gap_min_m: float=1.
    seat_gap_max_m: float=1.

def seat_confirmed(s):
    return (s.seat_contact and s.seat_contact_fraction>=SEAT_CONTACT_FRACTION and
            s.seat_force_n>=SEAT_MIN_FORCE and -SEAT_MAX_PENETRATION<=s.gap<=SEAT_ROUNDOFF and
            s.seat_gap_min_m>=-SEAT_MAX_PENETRATION and s.seat_gap_max_m<=SEAT_MAX_EDGE_GAP)
@dataclass
class History:
    score: float=0.
    invalid: bool=False
    reason: str=''
    tightened: bool=False
    proof: float=0.
    released_seconds: float=0.
    def update(self,s,dt):
        numbers=(s.turns,s.gap,s.interface_torque,s.fixture_displacement,dt,
                 s.seat_contact_fraction,s.seat_force_n,s.seat_gap_min_m,s.seat_gap_max_m)
        if not all(math.isfinite(v) for v in numbers) or dt<=0:self.invalid=True;self.reason='nonfinite_state'
        elif not s.valid:self.invalid=True;self.reason='mechanism_integrity'
        elif s.fixture_displacement>.002:self.invalid=True;self.reason='fixture_displaced'
        elif s.captured and s.seat_gap_min_m < -SEAT_MAX_PENETRATION:self.invalid=True;self.reason='seat_penetration'
        self.score=0.
        if self.invalid:return self.score
        if not s.captured:
            self.tightened=False;self.proof=0.;self.released_seconds=0.;return self.score
        if s.gap<=INITIAL_GAP-PITCH/4:self.score=.2
        if s.gap<=INITIAL_GAP-PITCH:self.score=.4
        seated=seat_confirmed(s)
        if seated:self.score=.7
        good=seated
        self.proof=self.proof+dt if good else 0.
        if self.proof>=.5-1e-9:self.tightened=True
        if not seated:self.tightened=False
        if self.tightened and seated:self.score=1.
        return self.score
