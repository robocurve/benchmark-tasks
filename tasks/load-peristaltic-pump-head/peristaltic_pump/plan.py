"""Guide each continuous tube at both exits before closing the common cam."""

import numpy as np

from . import config as C
from .motion import Program, rotation

LEFT = rotation("X", np.pi - 0.33) @ rotation("Y", 0.02)
RIGHT = rotation("X", np.pi + 0.32) @ rotation("Y", 0.03)
LEVER_TOP = rotation("X", np.pi + 0.27) @ rotation("Y", 0.045)


def line(i, x, z, grip):
    return (x, C.HANDS_Y[i], z), (LEFT, RIGHT)[i], grip


def lever(angle, outward=0.0, above=0.0):
    p = C.HEAD + C.PIVOT + rotation("Y", angle) @ C.LEVER
    return p + [0.003 + outward, -0.001, 0.009 + above], LEVER_TOP, 0.01


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    staged = C.TABLE + C.STAGE_Z
    loaded = C.TABLE + C.BED_Z + C.TUBE_R
    raised = C.TABLE + 0.163
    tubes = list(zip(("orange", "blue"), C.STAGING, C.LANES))
    for tube, source, lane in tubes[:1] if mode == "single-tube" else tubes:
        x = C.HEAD[0] + lane + (0.0015 if lane > 0 else 0.0)
        inlet_x = x + (0.010 if lane > 0 else 0.0)
        p.add(2.0, f"approach the {tube} tube from above", left=line(0, source, raised, 0.235), right=line(1, source, raised, 0.235))
        p.add(2.25, f"straddle the {tube} tube", left=line(0, source, staged - 0.008, 0.235), right=line(1, source, staged - 0.008, 0.235))
        p.add(0.55, "settle on the grasp line")
        p.add(1.1, f"pinch both sides of the {tube} tube", left=line(0, source, staged - 0.008, 0.015), right=line(1, source, staged - 0.008, 0.015))
        p.add(1.0, f"lift the {tube} tube out of its rest", left=line(0, source, staged + 0.022, 0.015), right=line(1, source, staged + 0.022, 0.015))
        p.add(1.8, f"route the {tube} tube over the two-track roller", left=line(0, x, staged + 0.022, 0.015), right=line(1, inlet_x, staged + 0.022, 0.015))
        p.add(1.3, f"press the {tube} tube through both retainers", left=line(0, x, loaded - 0.013, 0.015), right=line(1, inlet_x, loaded - 0.013, 0.015))
        p.add(0.8, "release the seated tube", left=line(0, x, loaded - 0.013, 0.14), right=line(1, inlet_x, loaded - 0.013, 0.14))
        p.add(1.3, "clear the retainers", left=line(0, x, raised, 0.235), right=line(1, x, raised, 0.14))
    if mode == "open-head":
        return p.moves
    p.add(1.5, "approach the side lever from above", right=lever(C.OPEN, above=0.058))
    p.add(1.75, "touch the top of the side lever", right=lever(C.OPEN))
    for angle in np.linspace(C.OPEN, 0.0, 10)[1:]:
        p.add(0.5, "close the cam by pushing its lever", right=lever(angle, above=-0.005))
    p.add(0.90, "lift away from the lever", right=lever(0, above=0.050))
    p.add(1.4, "withdraw for the water check", right=line(1, 0.32, C.TABLE + 0.19, 0.50), left=line(0, 0.33, C.TABLE + 0.19, 0.50))
    p.add(3.8, "water check after both grippers are clear")
    return p.moves
