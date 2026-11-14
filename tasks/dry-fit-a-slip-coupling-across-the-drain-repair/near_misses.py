"""Near-successes and unintended strategies, scored offline by the factory checker. Each case: (label, {object:
ready state}, expected score, expected success). Offline there is no robot, so the arms are set here as measured
clear; in the simulator they are measured every step and are None (no credit) until then.
"""

import math

import parts
from task import (AXIS_Z, COUPLING, CPL_L, FAR_COUPLING, LINE_Y, NEAR_CPL_X, NEAR_END, NEW_LEN, NEW_PIPE, NEW_X,
                  NEW_X0, TOP)

grip = parts.__dict__.setdefault("_drain_coupling_task_grip", {})
grip["clear"] = True

CENTRED = (NEAR_END + NEW_X0) / 2  # the coupling's middle on the open joint (4 mm gap)


def quat(axis, deg):
    a = math.radians(deg) / 2
    s = math.sin(a)
    return tuple([s * v for v in axis] + [math.cos(a)])


def cpl(x, dy=0.0, dz=0.0, q=None, vel=None):
    return {"coupling": COUPLING.state(pos=(x, LINE_Y + dy, AXIS_Z + dz), quat_xyzw=q, lin_vel=vel)}


def new(dx=0.0, dz=0.0, q=None, vel=None):
    return {"new_pipe": NEW_PIPE.state(pos=(NEW_X + dx, LINE_Y, AXIS_Z + dz), quat_xyzw=q, lin_vel=vel)}


def droop(deg):
    """The replacement pivoted down about its far end by ``deg``."""
    a = math.radians(deg)
    far = NEW_X + NEW_LEN / 2
    cx = far - math.cos(a) * NEW_LEN / 2
    return {"new_pipe": NEW_PIPE.state(pos=(cx, LINE_Y, AXIS_Z - math.sin(a) * NEW_LEN / 2),
                                       quat_xyzw=quat((0, 1, 0), deg))}


ON = cpl(CENTRED)
HALF = 0.006 / 2  # the coupling may sit 3 mm off centre (depths 6 mm apart)

CASES = [
    ("start: coupling back on the near end", {}, 0.0, False),
    ("centred on the marks, in line", ON, 1.0, True),
    ("3 mm toward the replacement (depths 6 mm apart, limit)", cpl(CENTRED + HALF - 1e-7), 1.0, True),
    ("3 mm toward the fixed end (limit)", cpl(CENTRED - HALF + 1e-7), 1.0, True),
    ("3.5 mm off centre", cpl(CENTRED + 0.0035), 0.5, False),
    ("across, 15 mm on the replacement (limit)", cpl(NEW_X0 + 0.015 - CPL_L / 2 + 1e-7), 0.5, False),
    ("only 14 mm on the replacement", cpl(NEW_X0 + 0.014 - CPL_L / 2), 0.0, False),
    ("slid too far: 14 mm left on the fixed end", cpl(NEAR_END - 0.014 + CPL_L / 2), 0.0, False),
    ("slid back toward the robot, 10 mm still on the replacement", cpl(NEW_X0 + 0.010 - CPL_L / 2), 0.0, False),
    ("centred but still sliding", cpl(CENTRED, vel=(0.03, 0.0, 0.0)), 0.0, False),
    ("centred, replacement drooping 1.2 degrees (end 4.4 mm low)", {**ON, **droop(1.2)}, 0.0, False),
    ("centred, replacement end 1.4 mm low (limit 1.5)", {**ON, **droop(0.37)}, 1.0, True),
    ("replacement pushed 4 mm on, far joint still made", {**ON, **new(dx=0.004)}, 1.0, True),
    ("far coupling pulled 9 mm toward the robot",
     {**ON, "far_coupling": FAR_COUPLING.state(pos=(FAR_COUPLING.pos[0] - 0.009, LINE_Y, AXIS_Z))}, 0.0, False),
    ("coupling lying on the table under the joint", cpl(CENTRED, dz=-(AXIS_Z - TOP) + 0.02875), 0.0, False),
    ("coupling 2 mm off the line (impossible on the pipe)", cpl(CENTRED, dy=0.002), 0.0, False),
    ("coupling left where it started, replacement in line", cpl(NEAR_CPL_X), 0.0, False),
    ("centred, replacement end 1.6 mm low", {**ON, **droop(0.433)}, 0.0, False),
    ("replacement lifted off and held 3 mm high", {**ON, **new(dz=0.003)}, 0.0, False),
]
