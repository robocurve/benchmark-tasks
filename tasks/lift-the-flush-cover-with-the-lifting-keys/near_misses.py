"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state or (centre, size or None, velocity or None[, quat_xyzw])}, expected score,
expected success). Cases run in order on one copy of task.py, so the whole-trial drop record carries from one case
to the next; a "new trial" case (every object back at its start after a jump) clears it.
"""

import math

from task import BEARER, BEARER_XY, COVER, COVER_PART, FRAME_XY, HOLE_Y, KEYS, LEDGE_TOP, TOP


def _q(axis, angle):
    s = math.sin(angle / 2)
    n = math.sqrt(sum(a * a for a in axis))
    return (axis[0] / n * s, axis[1] / n * s, axis[2] / n * s, math.cos(angle / 2))


def cover(x, y, bottom, quat=(0.0, 0.0, 0.0, 1.0), vel=None):
    return COVER_PART.state(pos=(x, y, TOP + bottom), quat_xyzw=quat, lin_vel=vel)


BX, BY = BEARER_XY
ON = BEARER[2]
OK = cover(BX, BY, ON)
NEW = ("new trial: everything back at its start", {}, 0.0, False)


def key_in_hole(side, cx, cy, foot):
    s = 1.0 if side == "left" else -1.0
    return KEYS[side].state(pos=(cx, cy + s * HOLE_Y, TOP + foot), quat_xyzw=_q((0, 0, 1), -s * math.pi / 2))


CASES = [
    ("cover flat across both bearers, keys back in their holders", {"cover": OK}, 1.0, True),
    ("on the bearers, left key still through its keyhole",
     {"cover": OK, "key_left": key_in_hole("left", BX, BY, ON - 0.010)}, 0.0, False),
    ("on the bearers, right key in its keyhole with its foot just below the cover top",
     {"cover": OK, "key_right": key_in_hole("right", BX, BY, ON + COVER[2] - 0.002)}, 0.0, False),
    ("on the bearers, right key foot above the cover top (drawn out)",
     {"cover": OK, "key_right": key_in_hole("right", BX, BY, ON + COVER[2] + 0.002)}, 1.0, True),
    ("on the bearers, left key through its keyhole but tilted 20 degrees, foot beside the hole",
     {"cover": OK, "key_left": KEYS["left"].state(pos=(BX + 0.012, BY + HOLE_Y, TOP + ON - 0.004),
                                                  quat_xyzw=_q((0, 1, 0), math.radians(-20)))}, 0.0, False),
    ("on the bearers, a key lying on top of the cover (out of the keyhole)",
     {"cover": OK, "key_left": KEYS["left"].state(pos=(BX, BY, TOP + ON + COVER[2] + 0.0131),
                                                  quat_xyzw=_q((1, 0, 0), math.pi / 2))}, 1.0, True),
    ("25 mm off centre towards the robot (limit)", {"cover": cover(BX - 0.025, BY, ON)}, 1.0, True),
    ("26 mm off centre towards the robot", {"cover": cover(BX - 0.026, BY, ON)}, 0.0, False),
    ("30 mm off centre side to side (limit)", {"cover": cover(BX, BY + 0.030, ON)}, 1.0, True),
    ("31 mm off centre side to side", {"cover": cover(BX, BY - 0.031, ON)}, 0.0, False),
    ("1.5 mm above the bearers (limit)", {"cover": cover(BX, BY, ON + 0.0015)}, 1.0, True),
    ("2 mm above the bearers: still held", {"cover": cover(BX, BY, ON + 0.002)}, 0.0, False),
    ("upside down across the bearers", {"cover": cover(BX, BY, ON + COVER[2], _q((1, 0, 0), math.pi))}, 1.0, True),
    ("tilted 3.5 degrees on the bearers",
     {"cover": cover(BX, BY, ON + 0.0042, _q((0, 1, 0), math.radians(3.5)))}, 0.0, False),
    ("on the bearers, still moving", {"cover": cover(BX, BY, ON, vel=(0.05, 0.0, 0.0))}, 0.0, False),
    ("flat on the table nearer the robot, clear of the frame, keys out", {"cover": cover(0.090, 0.0, 0.0)}, 0.0, False),
    ("still in the frame, lifted 3 mm", {"cover": cover(*FRAME_XY, LEDGE_TOP + 0.003)}, 0.0, False),
    ("tilted up 10 degrees in the frame",
     {"cover": cover(*FRAME_XY, LEDGE_TOP + 0.012, _q((1, 0, 0), math.radians(10)))}, 0.0, False),
    ("propped across the frame's near wall and the table",
     {"cover": cover(FRAME_XY[0] - 0.13, 0.0, 0.020, _q((0, 1, 0), math.radians(-17)))}, 0.0, False),
    NEW,
    ("carried 70 mm up, held still", {"cover": cover(BX, BY, 0.070)}, 0.0, False),
    ("one step later 25 mm lower (limit, not a drop)", {"cover": cover(BX, BY, 0.045)}, 0.0, False),
    NEW,
    ("then laid on the bearers", {"cover": OK}, 1.0, True),
    ("carried 70 mm up again", {"cover": cover(BX, BY, 0.070)}, 0.0, False),
    ("one step later 26 mm lower: dropped", {"cover": cover(BX, BY, 0.044)}, 0.0, False),
    ("then on the bearers: it was dropped", {"cover": OK}, 0.0, False),
    NEW,
    ("cover falling at 0.75 m/s (limit): not a drop", {"cover": cover(BX, BY, 0.03, vel=(0.0, 0.0, -0.75))}, 0.0, False),
    ("then on the bearers", {"cover": OK}, 1.0, True),
    ("cover falling at 0.8 m/s: dropped", {"cover": cover(BX, BY, 0.03, vel=(0.0, 0.0, -0.8))}, 0.0, False),
    ("then on the bearers: it was dropped", {"cover": OK}, 0.0, False),
    NEW,
    ("after a new trial, laid on the bearers", {"cover": OK}, 1.0, True),
]
