"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state or (centre, size or None, velocity or None[, quat_xyzw])}, expected score,
expected success). Cases run in order on one copy of task.py, so the whole-trial records (a grey tote gone past a
blue tote's ledges, a parts box moved over 5 mm) carry from one case to the next; a "new trial" case (every object
back at its start after a jump) clears them. Each blue tote that qualifies scores 0.5.
"""

import math

import numpy as np

from task import BLUE_XY, GREY, LEDGE_TOP, PARTS, PARTS_TOP, TOP, WALL


def _q(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return (*(a * math.sin(angle / 2)), math.cos(angle / 2))


def _mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


YAW0, YAW180 = (0.0, 0.0, 0.0, 1.0), _q((0, 0, 1), math.pi)


def grey(name, side, bottom, quat=YAW0, dx=0.0, vel=None):
    """Grey tote `name` with its underside `bottom` m above the table, centred over blue tote `side`."""
    bx, by = BLUE_XY[side]
    return GREY[name].state(pos=(bx + dx, by, TOP + bottom), quat_xyzw=quat, lin_vel=vel)


def on_ledges(name, side, quat, **kw):
    return grey(name, side, LEDGE_TOP, quat, **kw)


def nested(name, side, quat):
    return grey(name, side, PARTS_TOP, quat)


def parts_moved(side, d):
    p = np.asarray(PARTS[side].pos, float) + (d, 0.0, 0.0)
    return (tuple(p), None, None)


# Turned so solid wall is over the ledges: the left tote at yaw 180 degrees, the right one as it starts (180).
L_OK, R_OK = YAW180, YAW180
L_NEST, R_NEST = YAW0, YAW0
NEW = ("new trial: everything back at its start", {}, 0.0, False)

CASES = [
    ("both stacked", {"grey_left": on_ledges("left", "left", L_OK), "grey_right": on_ledges("right", "right", R_OK)},
     1.0, True),
    ("left stacked, right untouched", {"grey_left": on_ledges("left", "left", L_OK)}, 0.5, False),
    ("right stacked, left untouched", {"grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    ("never turn: right stacked, left nested on its parts box",
     {"grey_left": nested("left", "left", L_NEST), "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    ("then the left tote lifted out, turned and stacked: it pressed earlier, so only the right blue tote counts",
     {"grey_left": on_ledges("left", "left", L_OK), "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    NEW,
    ("always turn: left stacked, right turned and nested",
     {"grey_left": on_ledges("left", "left", L_OK), "grey_right": nested("right", "right", R_NEST)}, 0.5, False),
    ("then the right tote lifted out, turned back and stacked: it pressed earlier, so only the left counts",
     {"grey_left": on_ledges("left", "left", L_OK), "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    NEW,
    ("swapped: each grey tote stacked on the other blue tote",
     {"grey_left": on_ledges("left", "right", L_OK), "grey_right": on_ledges("right", "left", R_OK)}, 1.0, True),
    ("left lowered nest-wise until its underside is 43.0 mm up, 2 mm below the ledge tops (limit)",
     {"grey_left": grey("left", "left", LEDGE_TOP - 0.002, L_NEST)}, 0.0, False),
    ("then turned and both stacked: never went past the ledges",
     {"grey_left": on_ledges("left", "left", L_OK), "grey_right": on_ledges("right", "right", R_OK)}, 1.0, True),
    ("left lowered nest-wise to 42.9 mm: past the ledges, counts",
     {"grey_left": grey("left", "left", LEDGE_TOP - 0.0021, L_NEST)}, 0.0, False),
    ("then both stacked: the left blue tote stays at 0, the right earns 0.5",
     {"grey_left": on_ledges("left", "left", L_OK), "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    NEW,
    ("left 1.5 mm above the ledges (exact limit)",
     {"grey_left": grey("left", "left", LEDGE_TOP + 0.0015, L_OK), "grey_right": on_ledges("right", "right", R_OK)},
     1.0, True),
    ("left 2 mm above the ledges: held", {"grey_left": grey("left", "left", LEDGE_TOP + 0.002, L_OK),
                                          "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    ("left held 10 mm above the ledges", {"grey_left": grey("left", "left", LEDGE_TOP + 0.010, L_OK),
                                          "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    ("left on the ledges but tilted 2 degrees about x (corners 3.6 mm apart)",
     {"grey_left": on_ledges("left", "left", _mul(_q((1, 0, 0), math.radians(2)), L_OK)),
      "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    ("left on the ledges, still moving",
     {"grey_left": on_ledges("left", "left", L_OK, vel=(0.0, 0.0, -0.05)),
      "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    ("left 7 mm off centre in x", {"grey_left": on_ledges("left", "left", L_OK, dx=0.007),
                                   "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    ("left 6 mm off centre in x (limit)", {"grey_left": on_ledges("left", "left", L_OK, dx=0.006),
                                           "grey_right": on_ledges("right", "right", R_OK)}, 1.0, True),
    ("both stacked, left parts box moved 5 mm (limit)",
     {"grey_left": on_ledges("left", "left", L_OK), "grey_right": on_ledges("right", "right", R_OK),
      "parts_left": parts_moved("left", 0.005)}, 1.0, True),
    ("both stacked, left parts box moved 6 mm",
     {"grey_left": on_ledges("left", "left", L_OK), "grey_right": on_ledges("right", "right", R_OK),
      "parts_left": parts_moved("left", 0.006)}, 0.5, False),
    ("then the left parts box put back where it started: it was moved earlier, so only the right counts",
     {"grey_left": on_ledges("left", "left", L_OK), "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    NEW,
    ("left upside down, its rim on the ledges",
     {"grey_left": grey("left", "left", LEDGE_TOP + 0.055, _mul(_q((1, 0, 0), math.pi), L_OK)),
      "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    ("both grey totes on the right blue tote, one on top of the other: only the right blue tote counts",
     {"grey_right": on_ledges("right", "right", R_OK), "grey_left": grey("left", "right", LEDGE_TOP + 0.055, L_OK)},
     0.5, False),
    ("left grey tote left on the table beside its blue tote",
     {"grey_left": GREY["left"].state(pos=(0.33, 0.30, TOP + 0.0005), quat_xyzw=L_OK),
      "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    NEW,
    ("parts box taken out, left grey tote nested to the blue floor",
     {"parts_left": ((0.20, 0.25, TOP + 0.015), None, None), "grey_left": grey("left", "left", WALL, L_NEST)}, 0.0, False),
    ("then parts box put back and both stacked: the left pressed and its parts moved earlier; the right earns 0.5",
     {"grey_left": on_ledges("left", "left", L_OK), "grey_right": on_ledges("right", "right", R_OK)}, 0.5, False),
    NEW,
]
