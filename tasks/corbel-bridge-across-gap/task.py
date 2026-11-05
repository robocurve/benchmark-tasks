"""On the station table, straight ahead between the arms: two fixed grey stone piers, 100 x 100 mm and 40 mm tall,
with a 180 mm gap between them (the gap runs side to side). Beside each arm, four loose red-orange clay bricks,
120 x 40 x 25 mm, lie flat on the table in a row, their 120 mm length running side to side.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
PIER = (0.100, 0.100, 0.040)  # x, y, z
PIER_X = 0.30
GAP = 0.180
EDGE = GAP / 2  # each pier's inner face is EDGE from the midline (y = 0)
PIER_Y = EDGE + PIER[1] / 2
PIER_TOP = TOP + PIER[2]
BRICK = (0.040, 0.120, 0.025)  # x, y (long side, across the gap), z
BRICK_MASS = 0.23
SUPPLY_X = (0.14, 0.22, 0.30, 0.38)  # a row beside each arm, long side across the table (along y)
SUPPLY_Y = 0.30

STONE = (0.55, 0.55, 0.52)
CLAY = (0.72, 0.33, 0.22)

PIERS = {
    "pier_left": Box("pier_left", pos=(PIER_X, PIER_Y, TOP + PIER[2] / 2), size=PIER, color=STONE),
    "pier_right": Box("pier_right", pos=(PIER_X, -PIER_Y, TOP + PIER[2] / 2), size=PIER, color=STONE),
}
BRICKS = {}
for side, sy in (("l", SUPPLY_Y), ("r", -SUPPLY_Y)):
    for i, x in enumerate(SUPPLY_X):
        n = f"brick_{side}{i + 1}"
        BRICKS[n] = Box(n, pos=(x, sy, TOP + BRICK[2] / 2), size=BRICK, mass=BRICK_MASS, color=CLAY)
OBJECTS = [*PIERS.values(), *BRICKS.values()]

TABLE_TOL = 0.002  # a brick is on the table when its lowest point is less than 2 mm above it
ON_TOL = 0.001  # a brick rests on something when its bottom face is within 1 mm of that thing's top ...
OVERLAP_MIN = 0.002  # ... and their outlines overlap by more than 2 mm in both directions, seen from above
MEET_GAP = 0.005  # full score: the two top bricks' facing ends at most 5 mm apart ...
MEET_REACH = 0.080  # ... each side reaching at least 80 mm out from its pier
HALF_REACH = 0.060  # partial credit: a side reaching at least 60 mm out
TIP_X_OFF = 0.020  # the facing ends' centres at most 20 mm apart front to back (the 40 mm ends overlap by 20 mm) ...
TIP_Z_OFF = 0.015  # ... and at most 15 mm apart in height (the 25 mm ends overlap by 10 mm)
TILT_MAX = np.radians(3.0)  # a brick off the table lies flat: its 25 mm side within 3 degrees of vertical ...
TURN_MAX = np.radians(5.0)  # ... and its 120 mm length within 5 degrees of the gap's direction (side to side)


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _flat(state) -> bool:
    r = _rot(state.quat_xyzw)
    return bool(abs(r[2, 2]) >= np.cos(TILT_MAX) and abs(r[1, 1]) >= np.cos(TURN_MAX))


def _end(state, sign: float) -> np.ndarray:
    """Centre of the brick's end face toward -y (sign -1) or +y (sign +1), from its true size and orientation."""
    axis = _rot(state.quat_xyzw)[:, 1]
    axis = axis if axis[1] > 0 else -axis
    return np.asarray(state.pos, float) + sign * axis * BRICK[1] / 2


def _overlap(a_lo, a_hi, b_lo, b_hi) -> float:
    return float(min(a_hi, b_hi) - max(a_lo, b_lo))


def _in_gap_on_table(b) -> bool:
    """On the table with any part between the piers' inner faces, in line with the piers."""
    return bool(
        b.low[2] < TOP + TABLE_TOL
        and b.high[1] > -EDGE and b.low[1] < EDGE
        and _overlap(b.low[0], b.high[0], PIER_X - PIER[0] / 2, PIER_X + PIER[0] / 2) > 0
    )


def _structure(objects):
    """(bricks off the table, their supports), or None if any brick off the table is not flat, rests on nothing, or
    fails the stability check.

    A flat brick's bottom face is its centre height minus 12.5 mm; it rests on a pier or another flat brick whose top
    face is within 1 mm of it, with outlines overlapping. Stability: each brick's weight is shared equally between
    the objects it rests on, and passed down. For every brick, the centre of mass of the brick and the share of
    everything it carries must lie, seen from above, within the span of its contact patches with all its supports."""
    up = [n for n in BRICKS if objects[n].low[2] >= TOP + TABLE_TOL]
    if not all(_flat(objects[n]) for n in up):
        return None
    bottom = {n: objects[n].pos[2] - BRICK[2] / 2 for n in up}
    top = {**{n: objects[n].pos[2] + BRICK[2] / 2 for n in up}, **{m: PIER_TOP for m in PIERS}}
    supports, patches = {}, {}
    for n in up:
        b = objects[n]
        for m in [*PIERS, *up]:
            if m == n:
                continue
            s = objects[m]
            lo, hi = np.maximum(b.low[:2], s.low[:2]), np.minimum(b.high[:2], s.high[:2])
            if abs(bottom[n] - top[m]) <= ON_TOL and np.all(hi - lo > OVERLAP_MIN):
                supports.setdefault(n, []).append(m)
                patches.setdefault(n, []).append((lo, hi))
        if n not in supports:
            return None
    # Pass loads down, highest brick first: load[n] = weight it carries (its own 1 + shares), com[n] = their centre.
    load = {n: 1.0 for n in up}
    moment = {n: np.asarray(objects[n].pos[:2], float).copy() for n in up}
    for n in sorted(up, key=lambda k: -bottom[k]):
        com = moment[n] / load[n]
        lo = np.min([p[0] for p in patches[n]], axis=0)
        hi = np.max([p[1] for p in patches[n]], axis=0)
        if np.any(com < lo) or np.any(com > hi):
            return None
        share = load[n] / len(supports[n])
        for m in supports[n]:
            if m in load:
                load[m] += share
                moment[m] = moment[m] + share * com
    return up, supports


def _sides(objects):
    """Per side: (reach in m out from its pier's inner face, the innermost brick), or None if the build fails."""
    if any(_in_gap_on_table(objects[n]) for n in BRICKS):
        return None
    if not all(objects[n].resting() for n in BRICKS):
        return None
    built = _structure(objects)
    if built is None:
        return None
    up, supports = built
    out = {}
    for side, pier, sign in (("left", "pier_left", -1.0), ("right", "pier_right", 1.0)):
        on = set()  # every brick resting on this pier, directly or through other bricks
        changed = True
        while changed:
            changed = False
            for n in up:
                if n not in on and any(m == pier or m in on for m in supports[n]):
                    on.add(n)
                    changed = True
        if not on:
            out[side] = (0.0, None)
            continue
        # The innermost end: the brick end face nearest the other pier.
        tip = max(on, key=lambda n: sign * _end(objects[n], sign)[1])
        end_y = _end(objects[tip], sign)[1]
        out[side] = (max(0.0, EDGE + sign * float(end_y)), tip)
    return out


def _met(objects, sides) -> bool:
    (rl, tl), (rr, tr) = sides["left"], sides["right"]
    if tl is None or tr is None or rl < MEET_REACH or rr < MEET_REACH:
        return False
    if tl == tr:  # a capstone resting on both sides is the innermost brick of each: the sides are joined
        return True
    a, b = _end(objects[tl], -1.0), _end(objects[tr], 1.0)
    return bool(a[1] - b[1] <= MEET_GAP and abs(a[0] - b[0]) <= TIP_X_OFF and abs(a[2] - b[2]) <= TIP_Z_OFF)


def success(objects):
    s = _sides(objects)
    return bool(s is not None and _met(objects, s))


def distance(objects):
    s = _sides(objects)
    if s is None:
        return float(GAP)
    return float(max(0.0, GAP - s["left"][0] - s["right"][0]))


def score(objects):
    """1.0 the two sides meet; 0.5 both sides reach 60 mm; 0.25 one side does; 0 if any brick is on the table in the
    gap, any brick off the table is not flat, rests on nothing or fails the stability check, or anything moves."""
    s = _sides(objects)
    if s is None:
        return 0.0
    if _met(objects, s):
        return 1.0
    n = sum(r >= HALF_REACH for r, _ in s.values())
    return {2: 0.5, 1: 0.25}.get(n, 0.0)


# Demo: three courses per side, stepped so the top course overhangs most (17, 23 and 48.5 mm), which keeps the
# stack's centre of mass at least 11 mm inside every support edge at every stage; equal 30 mm steps would leave it
# 0.5 mm inside the pier's edge. Tip of each course, measured from the midline: 73, 50 and 1.5 mm, so the top tips
# end 3 mm apart. The arms grip each brick across its 40 mm width
# (fingers closing front to back), so no finger passes beyond a brick's tip into the gap.
TIPS = (0.073, 0.050, 0.0015)
TIP_PAST_TCP = 0.013
GRIP_DEPTH = 0.017  # finger tips this far below the brick's top
GRIP_DZ = BRICK[2] / 2 - GRIP_DEPTH + TIP_PAST_TCP
TRAVEL_Z = TOP + 0.16
RELEASE_LIFT = 0.006  # the brick is let go 6 mm above its course and drops onto it
RELEASE_FINGER = FINGER_OPEN / 2  # open only partway (pads about 47 mm apart), slowly, so the brick is not dragged
YAW = {"left": -np.pi / 2, "right": np.pi / 2}  # fingers closing along x; the signs keep joint6 within its limits
# Where each brick came to rest minus where the demo aimed it, per arm and course (x, y, z in m), measured with the
# trace wrapper; the demo aims off by that much. The left arm's bricks land 8 to 12 mm short in x (they shift as the
# fingers open); the right arm's top brick lands 10 mm short of the middle.
SAG = {
    "left": [(-0.0118, 0.0025, 0.0), (-0.0082, 0.0021, 0.0), (-0.0089, 0.0014, 0.0)],
    "right": [(0.0028, -0.0018, 0.0), (0.0016, -0.0011, 0.0), (0.0101, -0.0099, 0.0)],
}


def _course(arm, brick, k):
    sgn = 1.0 if arm == "left" else -1.0
    bx, by, bz = BRICKS[brick].pos
    grip = (bx, by, bz + GRIP_DZ)
    sx, sy, sz = SAG[arm][k]
    cy = sgn * (TIPS[k] + BRICK[1] / 2) - sy
    cz = PIER_TOP + k * BRICK[2] + BRICK[2] / 2 + RELEASE_LIFT + GRIP_DZ + sz
    cx = PIER_X - sx
    yaw = YAW[arm]
    return [
        Waypoint(1.5, (bx, by, TRAVEL_Z), FINGER_OPEN, f"over {brick}", yaw),
        Waypoint(1.2, grip, FINGER_OPEN, "descend", yaw),
        Waypoint(0.7, grip, FINGER_CLOSED, "grip", yaw),
        Waypoint(1.0, (bx, by, TRAVEL_Z), FINGER_CLOSED, "lift", yaw),
        Waypoint(1.8, (cx, cy, TRAVEL_Z), FINGER_CLOSED, f"carry to course {k + 1}", yaw),
        Waypoint(1.2, (cx, cy, cz + 0.02), FINGER_CLOSED, "lower", yaw),
        Waypoint(1.0, (cx, cy, cz), FINGER_CLOSED, "set down", yaw),
        Waypoint(1.0, (cx, cy, cz), RELEASE_FINGER, "release", yaw),
        Waypoint(0.8, (cx, cy, TRAVEL_Z), FINGER_OPEN, "up", yaw),
    ]


def demo():
    plan = {}
    for arm, side in (("left", "l"), ("right", "r")):
        w = []
        for k in range(3):
            w += _course(arm, f"brick_{side}{k + 1}", k)
        plan[arm] = w
    # The right arm waits before its top course so the two hands are never over the middle together.
    i = 2 * 9
    plan["right"] = plan["right"][:i] + [Waypoint(10.0, plan["right"][i - 1].pos, FINGER_OPEN, "wait", YAW["right"])] + plan["right"][i:]
    return plan
