"""Passive ratchet indexing and native screw references, in SI units."""
from dataclasses import dataclass
import math
@dataclass
class Ratchet:
    backlash:float=math.radians(5)
    drag:float=.002
    phase:float|None=None
    def update(self,relative,speed):
        if self.phase is None:self.phase=relative+self.backlash
        # Re-index with a finite opening gap while the hard upper stop separates.
        # A loaded or closing stop never changes its reference.
        if speed<0.:self.phase=min(self.phase,relative+self.backlash)
        return self.phase,self.drag*speed
@dataclass
class Screw:
    """Capture reference for a native helix. This class never writes a pose."""
    pitch:float=.00125
    captured:bool=False
    z0:float=0.
    angle0:float=0.
    def capture(self,z,angle):self.captured=True;self.z0=z;self.angle0=angle
    def expected_z(self,angle):
        if not self.captured:raise RuntimeError('Thread not captured')
        return self.z0+self.pitch*(angle-self.angle0)/(2*math.pi)
