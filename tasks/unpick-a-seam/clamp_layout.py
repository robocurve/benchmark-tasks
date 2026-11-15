"""Ordinary bench sewing clamp, using the demonstrated vertical contact layout.

Only the upper swatch's top 8 mm sits between the jaws. Both meshes and all
thread bodies remain dynamic. The stand is fixed to the table; no cloth node
is attached. This is a separate, initially unvalidated YAM reset.
"""
import numpy as np
from geometry import TABLE_TOP

# The retained-contact probe is additionally yawed 180 degrees: the upper
# patch faces the robot bases at smaller X; the connected stand stays behind.
ROTATION=np.array(((0.,0.,-1.),(0.,-1.,0.),(-1.,0.,0.)))
TOP=TABLE_TOP+.150
PLANE_X=.300
SUPPORT_SHIFT=.0198  # Lower edge centre becomes table top + cloth rest offset.


def transform(points,*,lower_edge_supported=False):
    """Rigidly relocate the exact 020 vertical retention-probe geometry."""
    top=TOP-SUPPORT_SHIFT if lower_edge_supported else TOP
    return (np.asarray(points)-(.235,0.,.7511))@ROTATION.T+(PLANE_X,0.,top)


def fixture_boxes():
    """Name, dimensions and centres of connected, fixed metal fixture parts.

    The 6 x 85 x 8 mm jaws, 0.30 mm gap and their relation to the fabric are
    identical to the native retention probe. Side bridges stay 2 mm beyond
    either cloth edge. The other frame parts stay behind the rear jaw.
    """
    center_x=PLANE_X-.0004
    result=[(f'clamp_jaw_{i}',(.006,.085,.008),
             (center_x+sign*.00315,0.,TOP-.004)) for i,sign in enumerate((-1,1))]
    result += [('clamp_base',(.080,.110,.010),(.320,0.,TABLE_TOP+.005)),
               ('clamp_post',(.012,.012,.140),(.320,0.,TABLE_TOP+.080)),
               ('clamp_arm',(.020,.012,.008),(.313,0.,TOP-.004))]
    # Two side bridges join the jaws without intersecting the fabric between.
    result += [(f'clamp_bridge_{i}',(.0123,.003,.012),
                (center_x,sign*.0435,TOP+.002)) for i,sign in enumerate((-1,1))]
    return result
