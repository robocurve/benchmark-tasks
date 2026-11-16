"""Finite ordinary tape-roll geometry and material accounting, in SI units.

No pre-laid strip, hidden leader, virtual tape gun, contact-activated teleport
or unbounded tape source. Physics still has to realize peeling and adhesion.
"""
from math import pi,sqrt,isfinite

TAPE_WIDTH=.048
TAPE_THICKNESS=.00005
CORE_RADIUS=.0381
OUTER_RADIUS=.05
CARTON_LENGTH,CARTON_WIDTH,CARTON_HEIGHT=.22,.18,.10
SEAM_LENGTH=.22
END_WRAP=.04
MIN_STRIP=SEAM_LENGTH+2*END_WRAP
MAX_STRAIN=.05


class TapeLedger:
    """Track measured natural-length material leaving a roll and severing.

    The native adapter supplies the live peel-front material coordinate. This
    class checks accounting; it does not produce ribbon nodes or adhesion.
    """
    def __init__(self):
        self.capacity=pi*(OUTER_RADIUS**2-CORE_RADIUS**2)/TAPE_THICKNESS
        self.paid_out=0.;self.cut_at=None;self.invalid=None

    def update(self,coordinate,measured_ribbon_length,cut_coordinate=None,blade_crossing=False):
        values=[coordinate,measured_ribbon_length]+([] if cut_coordinate is None else [cut_coordinate])
        if not all(isfinite(x) for x in values):self.invalid='nonfinite material state';return
        if coordinate < self.paid_out-1e-6:self.invalid='material rewound without a rewind model'
        if not 0 <= coordinate <= self.capacity:self.invalid='roll capacity exceeded'
        if self.cut_at is None:
            if not coordinate*(1-MAX_STRAIN)-.002 <= measured_ribbon_length <= coordinate*(1+MAX_STRAIN)+.002:
                self.invalid='ribbon length inconsistent with finite payout'
        self.paid_out=max(self.paid_out,coordinate)
        if cut_coordinate is not None:
            if self.cut_at is not None and abs(cut_coordinate-self.cut_at)>1e-6:
                self.invalid='cut coordinate changed'
            elif not blade_crossing or not MIN_STRIP <= cut_coordinate <= self.paid_out:
                self.invalid='unverified or too-short cut'
            else:self.cut_at=cut_coordinate

    @property
    def radius(self):
        return sqrt(max(CORE_RADIUS**2,OUTER_RADIUS**2-self.paid_out*TAPE_THICKNESS/pi))
