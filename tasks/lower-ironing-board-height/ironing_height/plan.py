"""Unweight the safety notch, operate its lever, and guide the deck down."""

import numpy as np

from . import config as C
from .motion import Program, rotation

FRONT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
FRONT = rotation("Y", np.radians(33.0)) @ FRONT


def support(z, x=.342):
    return [x, .173, z], FRONT, .06


def lever(height, degrees, clearance=0., grip=.06):
    position = C.HANDLE_PIVOT + rotation("Y", np.radians(degrees)) @ C.HANDLE_END
    position += [-clearance, 0., height - C.UPPER_Z]
    return position, FRONT, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(1.5, "Bring both arms to the front of the ironing board",
          left=support(C.UPPER_Z-.018, .212),
          right=lever(C.UPPER_Z, 0., .051, .74))
    p.add(1.1, "Set a closed fingertip below the loaded worktop",
          left=support(C.UPPER_Z-.018))
    p.add(1.1, "Lift the worktop slightly to unload the upper stop",
          left=support(C.LIFT_Z+.014))
    if mode == "no-release":
        p.add(1.25, "Lower the hand without operating the release",
              left=support(C.UPPER_Z-.018))
    else:
        p.add(.85, "Bring open fingers around the release handle",
              right=lever(C.LIFT_Z, 0., grip=.74))
        p.add(.65, "Grip the release handle", right=lever(C.LIFT_Z, 0.))
        for theta in np.linspace(0., C.LEVER_STOP+1., 9)[1:]:
            p.add(.23, "Lift the lever against its spring",
                  right=lever(C.LIFT_Z, theta))
        p.add(2.5, "Lower the deck under control toward the marked working height",
              left=support(C.LOWER_Z+.036),
              right=lever(C.LOWER_Z+.029, C.LEVER_STOP+1.))
        p.add(.55, "Release the lever at the lower notch",
              right=lever(C.LOWER_Z+.029, C.LEVER_STOP+1., grip=.79))
        p.add(.55, "Clear the release handle while the other hand holds the board",
              right=lever(C.LOWER_Z+.055, 22., .041, .79))
        p.add(.75, "Allow the bearing latch to spring closed above the marked stop")
        p.add(1.20, "Seat the lower lock and let it support the board",
              left=support(C.LOWER_Z-.018), right=lever(C.LOWER_Z+.06, 18., .057, .79))
    p.add(1.0, "Withdraw and leave the locked ironing board bearing its own weight",
          left=support(C.UPPER_Z+.125, .196),
          right=lever(C.UPPER_Z+.17, 0., .058, .79))
    return p.moves
