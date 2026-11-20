"""Reference moves for separating the lip and presenting both leading corners."""

import math

import numpy as np

from . import config as c
from .motion import Program, rotation

APPROACH = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
RIGHT_WRIST = rotation("Z", math.radians(19)) @ APPROACH
LEFT_WRIST = rotation("Y", math.radians(9)) @ rotation("Z", math.radians(-57)) @ APPROACH
GUIDE_PHASES = (
    "square the sheet to the mouth",
    "lower the squared sheet",
    "present both leading corners",
    "pass the corners under the film",
    "feed the sheet",
)


def waypoints(mode):
    if mode not in ("demo", "hold", "on-top"):
        raise ValueError(f"Unknown policy {mode!r}")
    p = Program()
    if mode == "hold":
        return p.moves

    lx = -c.UPPER_OVERHANG + .003
    ly = .105
    lz = c.LAYERS_GAP + .023
    left_open = .60
    wrist_open = .19

    def left_at(x, y, z, grip, orientation=LEFT_WRIST):
        return c.sleeve_point(x, y, z), c.SLEEVE_ROTATION @ orientation, grip

    def stock_at(x, z, grip):
        return (
            c.STOCK_REAR + c.STOCK_ROTATION @ np.array([x, c.HAND_OFFSET_Y, z]),
            c.STOCK_ROTATION @ RIGHT_WRIST,
            grip,
        )

    def lip_open():
        pivot = np.array([c.SLEEVE_DEPTH, ly, c.LAYERS_GAP])
        opening = rotation("Y", math.radians(7.5))
        pos = pivot + opening @ (np.array([lx, ly, lz]) - pivot)
        pos += [-.003, 0., .004]
        return left_at(*pos, left_open, rotation("Y", math.radians(4.)) @ LEFT_WRIST)

    def guide(duration, label, leading_x, leading_z, wrist_z, grip=0.):
        target = c.sleeve_point(leading_x, 0., leading_z)
        rear_x = leading_x - c.PAPER_LENGTH + .003 + (.018 if leading_x < 0 else 0.)
        right = (c.sleeve_point(rear_x, c.HAND_OFFSET_Y, wrist_z),
                 c.SLEEVE_ROTATION @ RIGHT_WRIST, grip)
        p.add(duration, label, right=right)
        p.moves["right"][-1]["leading_target"] = target.tolist()

    p.add(3.1, "approach the two exposed edges",
          left=left_at(lx - .028, ly + .006, lz + .065, left_open),
          right=stock_at(-.025, .058, wrist_open))
    p.add(1.35, "align the open jaws",
          left=left_at(lx - .018, ly + .006, lz, left_open),
          right=stock_at(-.015, .002, wrist_open))
    p.add(1.4, "enter the edge overhang",
          left=left_at(lx, ly, lz, left_open),
          right=stock_at(.003, .0015, wrist_open))
    p.add(1.0, "pinch the loose page", right=stock_at(.003, .0015, 0.))
    p.add(.65, "clear the lower sleeve edge",
          left=left_at(lx - .002, ly, lz + .003, left_open))
    p.add(1.8, "raise the sleeve lip and lift the sheet",
          left=lip_open() if mode == "demo" else left_at(lx - .06, ly, lz + .10, 1.),
          right=stock_at(.003, .157, 0.))
    p.add(2.85, "carry the loose page beside the opening",
          right=(c.sleeve_point(-.306, -.111, .129), c.SLEEVE_ROTATION @ RIGHT_WRIST, 0.))

    if mode == "on-top":
        guide(5.5, "carry the sheet over the protector", .238, .074, .100)
        guide(1.1, "lower onto the outside of the protector", .254, .041, .057)
        guide(1., "release on top of the closed film", .254, .041, .057, .30)
    else:
        guide(2.75, GUIDE_PHASES[0], -.062, .066, .135)
        guide(2.05, GUIDE_PHASES[1], -.052, .013, .080)
        guide(2.0, GUIDE_PHASES[2], -.004, .010, .078)
        guide(2.35, GUIDE_PHASES[3], .046, .003, .061)
        guide(4.6, GUIDE_PHASES[4], .176, .002, .043)
        guide(.9, "release the sheet into the sleeve", .176, .002, .044, wrist_open)
    p.add(1.4, "withdraw the feeding hand",
          right=(c.sleeve_point(-.082, -.092, .090), c.SLEEVE_ROTATION @ RIGHT_WRIST, .75))
    p.add(1.1, "release the protector",
          left=left_at(lx - .056, ly, lz + .030, .70) if mode == "demo" else None)
    p.add(1.3, "leave both hands clear",
          left=left_at(lx - .081, ly + .019, lz + .112, .80),
          right=(c.sleeve_point(-.112, -.162, .150), c.SLEEVE_ROTATION @ RIGHT_WRIST, .85))
    return p.moves
