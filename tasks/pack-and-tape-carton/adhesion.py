"""Candidate contact/cohesion law, not a validated adhesive simulation.

Unlike tool-path painting, bonding needs measured adhesive-face contact and
compression. The future native adapter must apply equal/opposite returned
forces to the ribbon and contacted body at every physics substep. This module
does not move material or switch a carton hinge to a closed pose.
"""
from math import sqrt,isfinite


class ContactBond:
    def __init__(self):
        self.contact_time=0.;self.attached=False;self.broken=False

    def contact(self,dt,gap,alignment,normal_force,adhesive_side):
        if not all(isfinite(v) for v in (dt,gap,alignment,normal_force)):
            raise ValueError('nonfinite contact')
        if not 0 < dt <= 1/120:raise ValueError('requires physical substep samples')
        valid=(adhesive_side and -.0002 <= gap <= .0005 and alignment >= .9 and normal_force >= .5)
        self.contact_time=self.contact_time+dt if valid else 0.
        if not self.broken and self.contact_time >= .1:self.attached=True
        return self.attached

    def force(self,displacement):
        if not all(isfinite(v) for v in displacement):raise ValueError('nonfinite deformation')
        if not self.attached:return (0.,0.,0.)
        length=sqrt(sum(x*x for x in displacement))
        # Candidate cohesive stiffness and failure load per 5x48 mm cell.
        # Values require a product-specific peel/shear calibration experiment.
        if 2000.*length > 2.:
            self.attached=False;self.broken=True;return (0.,0.,0.)
        return tuple(-2000.*x for x in displacement)
