"""Move both actual spring strips through their swept arcs by fingertip contact."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def push(side, angle, x_offset=0., opening=.015, z_offset=0.):
    sign = C.sign(side)
    frame = rotation("X", np.radians(-sign * angle))
    # The closed fingertips engage the underside. The contact direction follows
    # the lever tangent while the gripper faces into the frame.
    orientation = (rotation("Z", np.radians(12. * sign))
                   @ rotation("Y", np.radians(17.))
                   @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]))
    local = C.pivot(side) + frame @ [-.016, -sign * .044, -.008]
    return C.world(local) + [x_offset, 0., z_offset], orientation, opening


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    sides = ("left", "right")
    stop = C.CLIP_END - 1.
    p.add(2.6, "approach the two spring clips from the reader side",
          **{s: push(s, 0., -.086, .01, .050) for s in sides})
    p.add(1.4, "lower the closed fingertips below the clips",
          **{s: push(s, 0., -.085, .01, -.008) for s in sides})
    p.add(1.4, "bring a fingertip underneath each clip",
          **{s: push(s, 0., -.003, .01, -.004) for s in sides})
    for a in np.linspace(0., stop, 23)[1:]:
        p.add(.28, "swivel the spring strips into the open spread",
              left=push("left", a, -.001, .01),
              right=push("right", a if mode != "parked" else 0., -.001, .01))
    p.add(1.35, "withdraw the fingertips and leave the recipe held open",
          left=push("left", stop, -.106, .20, .060),
          right=push("right", 0. if mode == "parked" else stop, -.106, .20, .060))
    return p.moves
