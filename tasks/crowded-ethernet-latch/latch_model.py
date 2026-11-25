"""Parameterized latch/retention checks for future native connector telemetry.

Dimensions are mandatory inputs from the selected connector CAD/probe, not
invented universal RJ45 dimensions. This is not a native latch simulation.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Geometry:
    seated_depth: float
    clear_depth: float
    release_travel: float
    spring_stiffness: float
    elastic_limit: float

    def __post_init__(self):
        if not (0 <= self.clear_depth < self.seated_depth and
                0 < self.release_travel < self.elastic_limit and self.spring_stiffness > 0):
            raise ValueError('invalid measured connector geometry')

    def restoring_force(self,travel):
        if not math.isfinite(travel):raise ValueError('nonfinite latch travel')
        return self.spring_stiffness*max(0.,travel)


class ReleaseHistory:
    def __init__(self,geometry):
        self.g=geometry;self.released=False;self.disconnected=False;self.invalid=None

    def update(self,depth,travel):
        if not (math.isfinite(depth) and math.isfinite(travel)):
            self.invalid='nonfinite state';return
        if travel > self.g.elastic_limit:self.invalid='latch overstressed'
        # Establish release while the plug is still seated, before pulling out.
        if depth >= self.g.seated_depth-.0005 and travel >= self.g.release_travel:
            self.released=True
        if depth < self.g.seated_depth-.001 and not self.released:
            self.invalid='withdrawal without first releasing retention clip'
        if depth <= self.g.clear_depth and self.released:self.disconnected=True
