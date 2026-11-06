"""A scale model of a steel-framed floor bay (1 ft = 40 mm, 1:7.6) on the station table, in front of the arms. Six
grey steel joists (20 mm wide, 50 mm tall, 300 mm long) run straight away from the arms, 80 mm apart on centre
(2 ft), their centres 40, 120 and 200 mm left and right of the centre line between the arm bases; they span from 230
to 530 mm in front of the arm bases. They bear against two dark red girders (20 mm thick, 520 mm long side to side,
70 mm tall, so their tops stand 20 mm above the joist tops): the front girder from 210 to 230 mm in front of the arm
bases, the back one from 530 to 550 mm. A blue rubber handover mat (70 mm front to back, 260 mm side to side, 4 mm
thick) lies on the centre line, 130 mm in front of the arm bases. Everything so far is fixed. Two identical yellow
decking bundles (220 mm long, 60 mm wide, 40 mm tall, 0.4 kg; 5.5 x 1.5 x 1 ft at scale) lie flat on the table to
the left of the bay, long sides running left to right, centres 400 mm left of the centre line and 190 and 300 mm in
front of the arm bases.

Landed as OSHA 1926.757(e)(4) and (e)(5) require, each bundle lies flat and square across three joists, its long edge within
40 mm (1 ft) of a girder face where the joists bear: one across the left three joists, one across the right three.
Landed means still, and sitting down on each joist it is credited for: where each of its two long bottom edges
crosses that joist's centre line, its bottom is from 3 mm below to 0.5 mm above the joist's top (a bundle still held
with one end or one edge up is not). The right arm cannot reach the bundles' laydown spot, so the left arm passes it
one on the mat. A bundle landed with its centre in the middle 100 mm of the span (however it is turned), or with any
part of its outline reaching the middle of the span (380 mm in front of the arm bases), or stacked on the other,
can buckle joists not yet fully bridged and scores 0.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The bay: x is away from the arms, y to the robot's left. Joists butt against the girders' inner faces.
JOIST_W, JOIST_H = 0.020, 0.050
FRONT_FACE, BACK_FACE = 0.23, 0.53  # the girder faces the joists bear on (300 mm span)
JOIST_Y = (0.20, 0.12, 0.04, -0.04, -0.12, -0.20)  # left to right, 80 mm on centre
JOIST_TOP = TOP + JOIST_H
GIRDER_T, GIRDER_LEN, GIRDER_H = 0.020, 0.52, 0.070
GIRDER_TOP = TOP + GIRDER_H
JOIST_GREY = (0.42, 0.45, 0.50)
GIRDER_RED = (0.55, 0.08, 0.06)
_JX = (FRONT_FACE + BACK_FACE) / 2
JOISTS = [
    Box(f"joist_{i + 1}", pos=(_JX, y, TOP + JOIST_H / 2), size=(BACK_FACE - FRONT_FACE, JOIST_W, JOIST_H),
        color=JOIST_GREY)
    for i, y in enumerate(JOIST_Y)
]
GIRDERS = [
    Box("front_girder", pos=(FRONT_FACE - GIRDER_T / 2, 0.0, TOP + GIRDER_H / 2), size=(GIRDER_T, GIRDER_LEN, GIRDER_H),
        color=GIRDER_RED),
    Box("back_girder", pos=(BACK_FACE + GIRDER_T / 2, 0.0, TOP + GIRDER_H / 2), size=(GIRDER_T, GIRDER_LEN, GIRDER_H),
        color=GIRDER_RED),
]

# The handover mat on the centre line, in front of the front girder: both arms reach it.
MAT_X, MAT_SIZE = 0.13, (0.07, 0.26, 0.004)
MAT_TOP = TOP + MAT_SIZE[2]
MAT = Box("handover_mat", pos=(MAT_X, 0.0, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=(0.10, 0.25, 0.80))

# The bundles: local y is the long side; they start with it running left to right (square to the joists).
BUNDLE_W, BUNDLE_L, BUNDLE_H, BUNDLE_KG = 0.06, 0.22, 0.04, 0.4
BUNDLE_YELLOW = (0.95, 0.75, 0.05)
BUNDLE_START = {"bundle_a": (0.19, 0.40), "bundle_b": (0.30, 0.40)}
BUNDLES = [
    Box(name, pos=(x, y, TOP + BUNDLE_H / 2 + 0.001), size=(BUNDLE_W, BUNDLE_L, BUNDLE_H), mass=BUNDLE_KG,
        color=BUNDLE_YELLOW)
    for name, (x, y) in BUNDLE_START.items()
]  # each starts 1 mm above the table, and settles
BUNDLE_NAMES = tuple(BUNDLE_START)

OBJECTS = [*JOISTS, *GIRDERS, MAT, *BUNDLES]
SUCCESS_HOLD_S = 2.5  # landed and still for 2.5 s

HALF_CREDIT = 0.5  # each half of the bay: joists 1-3 (left), joists 4-6 (right)
HALVES = ((0, 1, 2), (3, 4, 5))  # indices into JOIST_Y, numbered 1 to 6 from the left
SEAT_BELOW, SEAT_ABOVE = 0.003, 0.0005  # landed: bottom from 3 mm below to 0.5 mm above the joist tops, at its lowest
# point and, for each joist credited, at both long bottom edges over that joist's centre line (a bundle still in the
# gripper hangs above them, or tilted with one end or edge up)
BAY_TOL = 0.002  # its footprint between the two girder faces, within 2 mm
TILT_MAX_DEG = 3.0  # flat: tilted at most 3 degrees (either big face up)
SQUARE_MAX_DEG = 15.0  # its long side within 15 degrees of square to the joists
COVER_MIN = 0.015  # a joist counts when at least 15 of its 20 mm width lie under the bundle's centre line
FULL_JOISTS, HALF_JOISTS = 3, 2  # all 3 of its half's joists: support factor 1; exactly 2 of them: 0.5
BEARING_FULL, BEARING_ZERO = 0.04, 0.07  # gap to the girder face: full credit to 40 mm, none from 70 mm (a square
# bundle's centre then reaches the mid-span band)
MIDSPAN = 0.10  # a bundle on the joists with its centre 100 mm or more from both girder faces: 0 overall
MID_X = (FRONT_FACE + BACK_FACE) / 2  # ... or with its outline reaching the middle of the span (0.38 m): 0 overall
STACK_BELOW, STACK_ABOVE = 0.005, 0.0005  # resting on the other bundle: bottom 5 mm below to 0.5 mm above its top
STACK_OVERLAP = 0.002  # ... with their outlines overlapping by more than 2 mm


def _rot(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(state) -> float:
    """Angle between the bundle's up axis and the vertical; upside down reads 0 (a bundle may lie either face up)."""
    return math.degrees(math.acos(min(1.0, abs(float(_rot(state)[2, 2])))))


def _square_off_deg(state) -> float:
    """How far the bundle's long side is turned from square to the joists (from the left-right direction)."""
    r = _rot(state)
    return math.degrees(math.atan2(abs(float(r[0, 1])), abs(float(r[1, 1]))))


def _seated(state) -> bool:
    """Bottom from SEAT_BELOW below to SEAT_ABOVE above the joist tops."""
    return JOIST_TOP - SEAT_BELOW <= float(state.low[2]) <= JOIST_TOP + SEAT_ABOVE


def _in_bay(state) -> bool:
    """Footprint between the two girder faces."""
    return bool(float(state.low[0]) >= FRONT_FACE - BAY_TOL and float(state.high[0]) <= BACK_FACE + BAY_TOL)


def _edge_bottoms_at(state, y) -> list:
    """Heights of the bundle's bottom face along its two long edges where they cross the line at world y (a joist's
    centre line), each clamped to the edge's ends. Either big face may be the bottom."""
    r, c = _rot(state), np.asarray(state.pos, float)
    if abs(float(r[1, 1])) < 1e-6:
        return [math.inf]
    w = BUNDLE_H / 2 * (-1.0 if r[2, 2] > 0 else 1.0)  # local z of the face pointing down
    out = []
    for u in (-BUNDLE_W / 2, BUNDLE_W / 2):
        v = (y - c[1] - r[1, 0] * u - r[1, 2] * w) / r[1, 1]
        v = min(max(v, -BUNDLE_L / 2), BUNDLE_L / 2)
        out.append(float(c[2] + r[2, 0] * u + r[2, 1] * v + r[2, 2] * w))
    return out


def _sits_on(state, j) -> bool:
    """Both long bottom edges, over joist j's centre line, from SEAT_BELOW below to SEAT_ABOVE above its top."""
    return all(JOIST_TOP - SEAT_BELOW <= z <= JOIST_TOP + SEAT_ABOVE for z in _edge_bottoms_at(state, JOIST_Y[j]))


def _joists_covered(state, joists) -> int:
    """Of the joists listed (indices into JOIST_Y), those with at least COVER_MIN of their width under the bundle's
    centre line (its long axis through its centre, BUNDLE_L long), measured across the joists, and on which the
    bundle sits down (_sits_on)."""
    half = BUNDLE_L / 2 * abs(float(_rot(state)[1, 1]))
    cy = float(state.pos[1])
    return sum(
        min(cy + half, JOIST_Y[j] + JOIST_W / 2) - max(cy - half, JOIST_Y[j] - JOIST_W / 2) >= COVER_MIN - 1e-9
        and _sits_on(state, j)
        for j in joists
    )


def _bearing_gap(state) -> float:
    """Gap from the nearer girder face to the bundle's long edge on that side, at the end of the edge farther from it."""
    r = _rot(state)
    a_x, b_x = abs(float(r[0, 1])), abs(float(r[0, 0]))
    cx = float(state.pos[0])
    front = cx - BUNDLE_W / 2 * b_x + BUNDLE_L / 2 * a_x - FRONT_FACE
    back = BACK_FACE - (cx + BUNDLE_W / 2 * b_x - BUNDLE_L / 2 * a_x)
    return min(front, back)


def bundle_credit(state, joists) -> float:
    """S x D for a bundle landed flat and square on the joists and still, S counted over the joists listed (one half
    of the bay); 0 if it fails any of those."""
    if not (state.resting() and _seated(state) and _in_bay(state) and _tilt_deg(state) <= TILT_MAX_DEG
            and _square_off_deg(state) <= SQUARE_MAX_DEG):
        return 0.0
    n = _joists_covered(state, joists)
    s = 1.0 if n >= FULL_JOISTS else 0.5 if n == HALF_JOISTS else 0.0
    g = _bearing_gap(state)
    d = 1.0 if g <= BEARING_FULL else 0.0 if g >= BEARING_ZERO else (BEARING_ZERO - g) / (BEARING_ZERO - BEARING_FULL)
    return s * d


def _corners_xy(state) -> np.ndarray:
    r, c = _rot(state), np.asarray(state.pos, float)
    half = np.array([BUNDLE_W, BUNDLE_L, BUNDLE_H]) / 2
    return np.array([(c + r @ (half * np.array([sx, sy, sz])))[:2]
                     for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def _at_midspan(state) -> bool:
    """On the joists (lowest point at their tops, over one of them) with its centre in the middle 100 mm of the span,
    or any part of its outline reaching the middle of the span, whatever its turn."""
    over_joist = any(float(state.low[1]) < y + JOIST_W / 2 and float(state.high[1]) > y - JOIST_W / 2
                     for y in JOIST_Y)
    cx = float(state.pos[0])
    xs = _corners_xy(state)[:, 0]
    centre_mid = min(cx - FRONT_FACE, BACK_FACE - cx) >= MIDSPAN - 1e-9
    reaches_mid = float(xs.min()) <= MID_X + 1e-9 and float(xs.max()) >= MID_X - 1e-9
    return bool(_seated(state) and over_joist and (centre_mid or reaches_mid))


def _outline_overlap(a, b) -> float:
    """How far the two bundles' outlines seen from above overlap (m), turned or not; <= 0 means apart."""
    pa, pb = _corners_xy(a), _corners_xy(b)
    depth = math.inf
    for s in (a, b):
        r = _rot(s)
        for k in range(3):
            d = r[:2, k]
            n = math.hypot(float(d[0]), float(d[1]))
            if n < 1e-6:
                continue
            axis = np.array([-d[1], d[0]]) / n
            p, q = pa @ axis, pb @ axis
            depth = min(depth, float(min(p.max(), q.max()) - max(p.min(), q.min())))
    return depth


def _stacked(objects) -> bool:
    """One bundle resting on the other."""
    a, b = (objects[n] for n in BUNDLE_NAMES)
    for up, down in ((a, b), (b, a)):
        if (float(down.high[2]) - STACK_BELOW <= float(up.low[2]) <= float(down.high[2]) + STACK_ABOVE
                and _outline_overlap(up, down) > STACK_OVERLAP):
            return True
    return False


def score(objects):
    """0.5 for each half of the bay, one bundle per half (whichever pairing scores more): a bundle earns 0.5 x S x D
    for a half, S for that half's joists under it and sat on (all 3: 1, 2: 0.5), D for its gap to the girder (to
    40 mm: 1, falling to 0 at 70 mm). 0 overall if a bundle on the joists has its centre in the middle 100 mm of the
    span or its outline reaching the span's middle, or one bundle rests on the other."""
    a, b = (objects[n] for n in BUNDLE_NAMES)
    if _stacked(objects) or _at_midspan(a) or _at_midspan(b):
        return 0.0
    left, right = HALVES
    best = max(bundle_credit(a, left) + bundle_credit(b, right), bundle_credit(b, left) + bundle_credit(a, right))
    return round(HALF_CREDIT * best, 6)


def success(objects):
    """Both halves landed in full (bundle credit needs the bundles still); must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in BUNDLE_NAMES)


# The demo: the left arm lands bundle_a across the left three joists, then sets bundle_b on the mat; the right arm
# waits until the left has parked, takes bundle_b off the mat and lands it across the right three joists. Both grip
# straight down across a bundle's 60 mm width (yaw 90 degrees: the fingers close front to back).
YAW = math.pi / 2
GRIP = -0.026  # finger target: 4 mm inside the bundle's sides, so the fingers don't saturate and push it off centre
GRIP_H = 0.032  # TCP above a bundle's bottom: the pads cover 21 mm of its side; let go on the joists, the pad tips
# are 5 mm above the girder tops
DROP = 0.006  # each bundle is let go this far above where it lands
TRAVEL_Z = TOP + 0.15  # a carried bundle's bottom clears the girders by 4.8 cm
PLACE_GAP = 0.010  # landed: the bundle's near long edge 10 mm from the front girder face
PLACE_X = FRONT_FACE + PLACE_GAP + BUNDLE_W / 2
LEFT_Y, RIGHT_Y = 0.12, -0.12  # centred on joists 2 and 5: 25 mm of slack either way before a joist drops out
BACK_PLACE_X = BACK_FACE - PLACE_GAP - BUNDLE_W / 2  # the same spot against the back girder (for the states file)
LAND_Z = JOIST_TOP + DROP + GRIP_H
LEFT_PARK = (0.18, 0.36, TRAVEL_Z)  # beside the bay, over the emptied laydown spot
RIGHT_WAIT = (0.20, -0.36, TRAVEL_Z)
RIGHT_PARK = (0.18, -0.36, TRAVEL_Z)


def _pick_place(pick, place, yaw=YAW):
    """pick_place_waypoints, closing to GRIP rather than fully."""
    return [Waypoint(w.duration, w.pos, GRIP if w.finger == FINGER_CLOSED else w.finger, w.label, w.yaw)
            for w in pick_place_waypoints(pick, place, travel_z=TRAVEL_Z, yaw=yaw)]


def demo():
    ax, ay = BUNDLE_START["bundle_a"]
    bx, by = BUNDLE_START["bundle_b"]
    # the left arm turns its grip the other way (-90 degrees): at +90 its wrist runs out of range over the mat
    left = _pick_place((ax, ay, TOP + GRIP_H), (PLACE_X, LEFT_Y, LAND_Z), yaw=-YAW)
    left += _pick_place((bx, by, TOP + GRIP_H), (MAT_X, 0.0, MAT_TOP + DROP + GRIP_H), yaw=-YAW)
    left.append(Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"))
    left_s = sum(w.duration for w in left)
    right = [Waypoint(left_s + 0.5, RIGHT_WAIT, FINGER_OPEN, "wait for the left arm", YAW)]
    right += _pick_place((MAT_X, 0.0, MAT_TOP + GRIP_H), (PLACE_X, RIGHT_Y, LAND_Z))
    right.append(Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
