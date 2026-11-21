"""Guide both ends of the loose leaf, then rotate its native retaining lever."""
import numpy as np

from . import config as C
from .motion import Program, rotation

FRONT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


EDGE = rotation("Y", np.radians(52.)) @ FRONT
THUMB = rotation("Y", np.radians(35.)) @ FRONT


def push(x, y, z=.739):
    return ([x, y, z], EDGE, .06)


def latch(a, clearance=0.):
    r = rotation("Y", np.radians(a))
    p = C.LATCH_PIVOT + r @ [-.047, 0., -.0075]
    p[0] -= clearance
    return p, THUMB, .06


def waypoints(mode):
    if mode == "hold":
        return {}
    program = Program()
    program.add(1.5, "Move above both ends of the leaf",
                left=push(.234, .226, .897), right=push(.234, -.226, .897))
    program.add(1.2, "Approach both ends of the leaf", left=push(.215, .226), right=push(.215, -.226))
    program.add(1.2, "Reach both separated ends of the outer edge",
                left=push(.245, .209), right=push(.245, -.209))
    program.add(2.0, "Square the extension leaf and guide its two fittings into the seam",
                left=push(.320, .209), right=push(.320, -.209))
    program.add(.70, "Settle the full-width tabletop seam")
    program.add(.8, "Clear the seam and approach the underside retainer",
                left=push(.250, .209), right=push(.213, -.209))
    if mode != "unlatched":
        program.add(1.4, "Approach the thumb lever from below", right=latch(0., .026))
        program.add(1.0, "Contact the underside of the lever", right=latch(0.))
        for a in np.linspace(0., C.LATCH_ANGLE + 4., 15)[1:]:
            program.add(.24, "Flip the thumb lever into the receiver", right=latch(a))
    program.add(.9, "Withdraw both hands and leave the table ready",
                left=push(.221, .258, .904), right=push(.222, -.258, .904))
    program.add(.6, "Release and check the retained leaf")
    return program.moves
