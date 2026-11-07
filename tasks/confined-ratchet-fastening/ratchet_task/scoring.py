"""Current-state scoring and explicit integrity gates, independent of a policy."""
from dataclasses import dataclass
import math
from ratchet_task.config import REQUIRED_TURNS
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
@dataclass
class History:
    score: float=0.
    invalid: bool=False
    reason: str=''
    tightened: bool=False
    proof: float=0.
    released_seconds: float=0.
    def update(self,s,dt):
        numbers=(s.turns,s.gap,s.interface_torque,s.fixture_displacement,dt)
        if not all(math.isfinite(v) for v in numbers) or dt<=0:self.invalid=True;self.reason='nonfinite_state'
        elif not s.valid:self.invalid=True;self.reason='mechanism_integrity'
        elif s.fixture_displacement>.002:self.invalid=True;self.reason='fixture_displaced'
        elif abs(s.interface_torque)>1.:self.invalid=True;self.reason='over_torque'
        self.score=0.
        if self.invalid:return self.score
        if not s.captured:
            self.tightened=False;self.proof=0.;self.released_seconds=0.;return self.score
        if s.turns>=.25:self.score=.2
        if s.turns>=1.:self.score=.4
        seated=s.gap<=.0002 and s.gap>=-.0003
        if seated and s.turns>=REQUIRED_TURNS:self.score=.7
        good=seated and s.turns>=REQUIRED_TURNS and s.socket_engaged and s.ratchet_cycles>=6
        self.proof=self.proof+dt if good else 0.
        if self.proof>=.5-1e-9:self.tightened=True
        if not seated:self.tightened=False
        if self.tightened and seated:self.score=1.
        return self.score
