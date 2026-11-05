"""On the station table: two dark grey storage pads (fixed blocks, 10 x 10 cm, 3 cm tall), one 27 cm in front of
each arm base. Between the arms stand five loose cardboard cartons. On the left of the midline, 14 cm from it: a
blue narrow carton (4 x 4 cm, 9 cm tall, 150 g) and two green cube cartons (5 cm on every side, 200 g each), 18, 27
and 36 cm in front of the arm bases. On the right of the midline, 14 cm from it: two red wide cartons (7 x 7 cm,
4 cm tall, 300 g each), 21 and 33 cm in front of the arm bases. Done, every carton is on a pad, one stack per pad,
and no stack is taller above its pad than three times the shorter side of the carton at its bottom: 12 cm on the
blue carton standing, 15 cm on a green cube, 21 cm on a red carton lying flat.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The pads: fixed blocks, raised 3 cm so a carton can't be slid onto one; each sits straight in front of an arm.
PAD_SIZE = (0.10, 0.10, 0.03)
PAD_HALF = PAD_SIZE[0] / 2
PAD_TOP = TOP + PAD_SIZE[2]
LEFT_PAD_XY, RIGHT_PAD_XY = (0.27, 0.305), (0.27, -0.305)
PAD_GREY = (0.22, 0.23, 0.25)
PADS = {"left_pad": LEFT_PAD_XY, "right_pad": RIGHT_PAD_XY}
PAD_BOXES = [
    Box(name, pos=(*xy, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=PAD_GREY) for name, xy in PADS.items()
]

# The cartons, standing as they start (x, y, z edge lengths). Each starts 1 mm above the table and settles.
N_SIZE, N_KG = (0.04, 0.04, 0.09), 0.15  # standing on its 4 x 4 cm end
C_SIZE, C_KG = (0.05, 0.05, 0.05), 0.2
W_SIZE, W_KG = (0.07, 0.07, 0.04), 0.3  # lying on its 7 x 7 cm face
BLUE, GREEN, RED = (0.12, 0.30, 0.85), (0.10, 0.62, 0.18), (0.82, 0.13, 0.10)  # saturated for the top camera
START = {
    "narrow": (0.18, 0.14),
    "cube_1": (0.27, 0.14),
    "cube_2": (0.36, 0.14),
    "wide_1": (0.21, -0.14),
    "wide_2": (0.33, -0.14),
}
CARTON_SIZE = {"narrow": N_SIZE, "cube_1": C_SIZE, "cube_2": C_SIZE, "wide_1": W_SIZE, "wide_2": W_SIZE}
_LOOK = {"narrow": (N_KG, BLUE), "cube_1": (C_KG, GREEN), "cube_2": (C_KG, GREEN), "wide_1": (W_KG, RED),
         "wide_2": (W_KG, RED)}
CARTONS = tuple(START)
CARTON_BOXES = [
    Box(name, pos=(*START[name], TOP + CARTON_SIZE[name][2] / 2 + 0.001), size=CARTON_SIZE[name],
        mass=_LOOK[name][0], color=_LOOK[name][1])
    for name in CARTONS
]

OBJECTS = [*PAD_BOXES, *CARTON_BOXES]
SUCCESS_HOLD_S = 2.5  # stacked and still for 2.5 s: long enough to see both stacks stand

RATIO = 3.0  # a stack's height above its pad at most 3 times the shorter footprint side of its bottom carton
LIMIT_SLACK = 1e-6  # 0.001 mm of floating-point slack, so a stack exactly at its limit passes ("no more than")
CARTON_CREDIT = 0.2  # each of the five cartons counted in a stack
SEAT_TOL = 0.005  # a carton's bottom within 5 mm of the pad top (or of the top of the carton below it)
CENTRE_TOL = 0.015  # an upper carton's centre within 15 mm (across the table) of the centre of the carton below
FLAT_DEG = 5.0  # every counted carton tilted at most 5 degrees from lying on one of its faces

# Heights and footprints come from each carton's centre, size and orientation, not from the simulator's bounding
# box (which reads a few millimetres wide on the GPU).


def _rot(state):
    """The carton's rotation matrix (rows: world x, y, z; columns: the box's own axes)."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def _half(name, state):
    """Half the carton's extent along world x, y and z, as it rests."""
    r, size = _rot(state), CARTON_SIZE[name]
    return [sum(abs(r[j][i]) * size[i] / 2 for i in range(3)) for j in range(3)]


def _bottom(name, state) -> float:
    return float(state.pos[2]) - _half(name, state)[2]


def _top(name, state) -> float:
    return float(state.pos[2]) + _half(name, state)[2]


def _footprint(name, state):
    """(x min, x max, y min, y max) of the carton seen from above."""
    h, p = _half(name, state), state.pos
    return (float(p[0]) - h[0], float(p[0]) + h[0], float(p[1]) - h[1], float(p[1]) + h[1])


def _overlaps(f, g) -> bool:
    return f[0] < g[1] and g[0] < f[1] and f[2] < g[3] and g[2] < f[3]


def _face_down(name, state):
    """(tilt from the nearest face-down pose in degrees, height, shorter footprint side) of a carton as it rests."""
    up = _rot(state)[2]  # world-z part of each box axis
    k = max(range(3), key=lambda i: abs(up[i]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(up[k])))))
    size = CARTON_SIZE[name]
    return tilt, size[k], min(size[i] for i in range(3) if i != k)


def _flat(name, state) -> bool:
    return _face_down(name, state)[0] <= FLAT_DEG


def _over_pad(state, pad_xy) -> bool:
    """Centre inside the pad's 10 x 10 cm outline (seen from above)."""
    return abs(float(state.pos[0]) - pad_xy[0]) <= PAD_HALF and abs(float(state.pos[1]) - pad_xy[1]) <= PAD_HALF


def _rests_on_pad(name, state, pad_xy) -> bool:
    """Bottom within SEAT_TOL of the pad top and footprint overlapping the pad top at all (centre anywhere)."""
    pad = (pad_xy[0] - PAD_HALF, pad_xy[0] + PAD_HALF, pad_xy[1] - PAD_HALF, pad_xy[1] + PAD_HALF)
    return abs(_bottom(name, state) - PAD_TOP) <= SEAT_TOL and _overlaps(_footprint(name, state), pad)


def _rests_on(name, upper, lower_name, lower) -> bool:
    """Bottom within SEAT_TOL of the lower carton's top and the two footprints overlapping (centres anywhere)."""
    return abs(_bottom(name, upper) - _top(lower_name, lower)) <= SEAT_TOL and _overlaps(
        _footprint(name, upper), _footprint(lower_name, lower)
    )


def _on(name, upper, lower_name, lower) -> bool:
    """Counted on the carton below: resting on it, flat, centre within CENTRE_TOL of its centre."""
    return bool(
        _rests_on(name, upper, lower_name, lower)
        and math.hypot(float(upper.pos[0]) - float(lower.pos[0]), float(upper.pos[1]) - float(lower.pos[1]))
        <= CENTRE_TOL
        and _flat(name, upper)
    )


def _above(objects, floor, region):
    """The cartons counted in a stack's height: every carton whose bottom is above floor - SEAT_TOL and whose
    footprint overlaps the region or the footprint of a carton already counted, repeated until none joins. No seat
    test: a carton lying tilted on a tilted carton (its bottom below the lower carton's highest corner) still counts."""
    stack, grew = [], True
    while grew:
        grew = False
        for n in CARTONS:
            if n in stack or _bottom(n, objects[n]) <= floor - SEAT_TOL:
                continue
            f = _footprint(n, objects[n])
            if _overlaps(f, region) or any(_overlaps(f, _footprint(m, objects[m])) for m in stack):
                stack.append(n)
                grew = True
    return stack


def _table_stacks_safe(objects) -> bool:
    """Every stack standing on the table (not on a pad) keeps its height above the table top within RATIO times the
    shorter footprint side of its bottom carton: a tower built beside the pads is the same hazard."""
    for base in CARTONS:
        if abs(_bottom(base, objects[base]) - TOP) > SEAT_TOL:
            continue
        stack = _above(objects, TOP, _footprint(base, objects[base]))
        height = max(_top(n, objects[n]) - TOP for n in stack)
        if height > RATIO * _face_down(base, objects[base])[2] + LIMIT_SLACK:
            return False
    return True


def _pad(objects, pad_xy):
    """(the cartons counted in this pad's stack, bottom first; whether the pad keeps H <= RATIO * b).

    The height limit counts every carton above the pad (bottom above PAD_TOP - SEAT_TOL) whose footprint overlaps
    the pad top or a carton counted in its stack (see _above), credited or not. H is the highest point of any of
    them, above the pad top; b is the shorter footprint side of the carton resting on the pad top (the narrowest, if
    several do); with no carton resting on the pad there is no stack to limit. Credit: a pad counts nothing unless
    exactly one carton rests on its top, flat with its centre inside the outline; above it, cartons are counted up
    to the first one that is not resting flat and centred on the one below."""
    touching = [n for n in CARTONS if _rests_on_pad(n, objects[n], pad_xy)]
    pad = (pad_xy[0] - PAD_HALF, pad_xy[0] + PAD_HALF, pad_xy[1] - PAD_HALF, pad_xy[1] + PAD_HALF)
    height = max((_top(n, objects[n]) - PAD_TOP for n in _above(objects, PAD_TOP, pad)), default=0.0)
    safe = not touching or height <= RATIO * min(_face_down(n, objects[n])[2] for n in touching) + LIMIT_SLACK
    if len(touching) != 1:
        return [], safe
    base = touching[0]
    if not (_over_pad(objects[base], pad_xy) and _flat(base, objects[base])):
        return [], safe
    stack = [base]
    while True:
        below = stack[-1]
        above = [n for n in CARTONS if n not in stack and _on(n, objects[n], below, objects[below])]
        if len(above) != 1:
            return stack, safe
        stack.append(above[0])


def score(objects):
    """Partial credit (success scores 1.0): 0.2 for each carton counted in a pad's stack (see _pad). 0 overall if
    either pad's stack, every carton above the pad whose footprint overlaps the pad or a carton in that stack
    included, is taller above the pad than three times the shorter footprint side of its bottom carton, or if any
    stack standing on the table, counted the same way from the table top, is taller than three times the shorter
    footprint side of its bottom carton. score() can't tell a held carton from a resting one; success() needs every
    carton still."""
    if not _table_stacks_safe(objects):
        return 0.0
    total = 0
    for pad_xy in PADS.values():
        stack, safe = _pad(objects, pad_xy)
        if not safe:
            return 0.0
        total += len(stack)
    return round(CARTON_CREDIT * total, 6)


def success(objects):
    """All five cartons stacked on the pads within the height limit, and still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in CARTONS)


# The demo: the left arm stands cube_1 on the left pad, sets cube_2 down on the midline for the right arm, and
# stands the blue carton on cube_1 (14 cm, under the 15 cm a cube allows). The right arm lays both red cartons on the
# right pad, waits for the left arm to leave the midline, and puts cube_2 on top (13 cm, under the 21 cm limit).
# Every grip is straight down with the fingers closing along world y.
TRAVEL_Z = TOP + 0.15  # carrying a carton low, where nothing tall is in the way
HIGH_Z = TOP + 0.17  # carrying a carton onto a stack: its bottom clears the stack top by 3 cm
CLEAR_Z = TOP + 0.20  # after the last release on a stack: fingertips 1.7 cm above the stack's top
N_GRIP, C_GRIP, W_GRIP = 0.06, 0.032, 0.025  # TCP above each carton's bottom: the pads cover 2.8 to 3.3 cm of it
DROP = 0.008  # each carton is let go this far above where it lands
STAGE_XY = (0.27, 0.0)  # where cube_2 crosses: on the midline, in reach of both arms
RIGHT_WAIT_S = 3.0  # the right arm waits above its pad until the left arm has carried the blue carton away
LEFT_PARK = (0.15, 0.305, CLEAR_Z)  # behind each stack, so the camera sees the result
RIGHT_PARK = (0.15, -0.305, CLEAR_Z)


def demo():
    left = pick_place_waypoints(
        (*START["cube_1"], TOP + C_GRIP), (*LEFT_PAD_XY, PAD_TOP + DROP + C_GRIP), travel_z=TRAVEL_Z,
    )
    left += pick_place_waypoints(
        (*START["cube_2"], TOP + C_GRIP), (*STAGE_XY, TOP + DROP + C_GRIP), travel_z=TRAVEL_Z,
    )
    left += pick_place_waypoints(
        (*START["narrow"], TOP + N_GRIP), (*LEFT_PAD_XY, PAD_TOP + C_SIZE[2] + DROP + N_GRIP), travel_z=HIGH_Z,
    )
    left.append(Waypoint(0.8, (*LEFT_PAD_XY, CLEAR_Z), FINGER_OPEN, "lift clear of the stack"))
    left.append(Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"))

    right = pick_place_waypoints(
        (*START["wide_1"], TOP + W_GRIP), (*RIGHT_PAD_XY, PAD_TOP + DROP + W_GRIP), travel_z=TRAVEL_Z,
    )
    right += pick_place_waypoints(
        (*START["wide_2"], TOP + W_GRIP), (*RIGHT_PAD_XY, PAD_TOP + W_SIZE[2] + DROP + W_GRIP), travel_z=TRAVEL_Z,
    )
    right.append(Waypoint(RIGHT_WAIT_S, (*RIGHT_PAD_XY, TRAVEL_Z), FINGER_OPEN, "wait for the left arm"))
    right += pick_place_waypoints(
        (*STAGE_XY, TOP + C_GRIP), (*RIGHT_PAD_XY, PAD_TOP + 2 * W_SIZE[2] + DROP + C_GRIP), travel_z=HIGH_Z,
    )
    right.append(Waypoint(0.8, (*RIGHT_PAD_XY, CLEAR_Z), FINGER_OPEN, "lift clear of the stack"))
    right.append(Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
