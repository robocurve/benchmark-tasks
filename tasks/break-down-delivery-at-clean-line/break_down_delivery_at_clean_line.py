"""On the station table, a yellow partition 20 cm tall and 2 cm thick stands on the midline between the arms, from 10
to 55 cm in front of the arm bases. It is the line between the dirty receiving side (the robot's left) and clean
storage (the robot's right). A 15 cm gap in it, 17.5 to 32.5 cm in front of the bases, holds a grey pass-through
shelf (15 x 15 cm, 2 cm tall) centred on the line. On the left, an opened brown shipping carton (outside 22.5 x 15 cm,
walls 5 cm tall) is fixed 30 cm in front of the left arm base. In it stand three supply boxes in a row front to back
(5.5 cm cubes, 80 g each: blue nearest the arms, then green, then orange), and a brown corrugated cardboard insert
(18 x 6 x 3 cm, 60 g) lies across their tops. Between the carton and the partition, 32 cm in front of the bases and
14.5 cm left of the midline, is a flat dark grey breakdown pad (24 x 10 cm). On the right, a teal clean-supply shelf
(22 x 12 cm, top 6 cm above the table) is fixed 30 cm in front of the right arm base. The partition, pass-through,
carton, pad and shelf are fixed; the boxes and the insert are loose. Done: the insert lies flat on the breakdown pad,
the three boxes stand on a face on the clean shelf, and at the end no brown cardboard is over the pass-through or past
the line.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The line: a yellow partition on y = 0, too tall for the arms to reach over, with a gap holding the pass-through.
LINE_Y = 0.0
WALL_T, WALL_H = 0.02, 0.20
LINE_FACE_Y = LINE_Y + WALL_T / 2  # the partition's dirty-side face
WALL_X = (0.10, 0.55)  # the partition runs from 10 to 55 cm in front of the arm bases
PASS_X, PASS_SIZE = 0.25, (0.15, 0.15, 0.02)
PASS_TOP = TOP + PASS_SIZE[2]
PASS_LO = (PASS_X - PASS_SIZE[0] / 2, LINE_Y - PASS_SIZE[1] / 2)
PASS_HI = (PASS_X + PASS_SIZE[0] / 2, LINE_Y + PASS_SIZE[1] / 2)
YELLOW = (0.95, 0.78, 0.08)
PASS_GREY = (0.50, 0.52, 0.55)
_near_len, _far_len = PASS_LO[0] - WALL_X[0], WALL_X[1] - PASS_HI[0]
LINE = [
    Box("partition_near", pos=(WALL_X[0] + _near_len / 2, LINE_Y, TOP + WALL_H / 2), size=(_near_len, WALL_T, WALL_H),
        color=YELLOW),
    Box("partition_far", pos=(PASS_HI[0] + _far_len / 2, LINE_Y, TOP + WALL_H / 2), size=(_far_len, WALL_T, WALL_H),
        color=YELLOW),
    Box("pass_through", pos=(PASS_X, LINE_Y, TOP + PASS_SIZE[2] / 2), size=PASS_SIZE, color=PASS_GREY),
]

# The opened shipping carton: a 5 mm floor and four 1 cm walls, 5 cm tall, around a 20.5 x 13 cm inside.
CARTON_X, CARTON_Y = 0.30, 0.305
CARTON_IN = (0.205, 0.13)
C_WALL_T, C_WALL_H, C_FLOOR_T = 0.01, 0.05, 0.005
CARTON_FLOOR = TOP + C_FLOOR_T
CARTON_BROWN = (0.45, 0.28, 0.12)
_cx0, _cx1 = CARTON_X - CARTON_IN[0] / 2, CARTON_X + CARTON_IN[0] / 2
_cy0, _cy1 = CARTON_Y - CARTON_IN[1] / 2, CARTON_Y + CARTON_IN[1] / 2
_cz = TOP + C_WALL_H / 2
CARTON = [
    Box("carton_floor", pos=(CARTON_X, CARTON_Y, TOP + C_FLOOR_T / 2), size=(*CARTON_IN, C_FLOOR_T),
        color=CARTON_BROWN),
    Box("carton_wall_near", pos=(_cx0 - C_WALL_T / 2, CARTON_Y, _cz), size=(C_WALL_T, CARTON_IN[1] + 2 * C_WALL_T, C_WALL_H),
        color=CARTON_BROWN),
    Box("carton_wall_far", pos=(_cx1 + C_WALL_T / 2, CARTON_Y, _cz), size=(C_WALL_T, CARTON_IN[1] + 2 * C_WALL_T, C_WALL_H),
        color=CARTON_BROWN),
    Box("carton_wall_left", pos=(CARTON_X, _cy1 + C_WALL_T / 2, _cz), size=(CARTON_IN[0], C_WALL_T, C_WALL_H),
        color=CARTON_BROWN),
    Box("carton_wall_right", pos=(CARTON_X, _cy0 - C_WALL_T / 2, _cz), size=(CARTON_IN[0], C_WALL_T, C_WALL_H),
        color=CARTON_BROWN),
]

# The breakdown pad on the dirty side, where the corrugated insert goes: between the carton and the partition, 2 cm
# clear of the pass-through's left edge and 3.5 cm clear of the carton, out from under the left arm's home pose.
PAD_X, PAD_Y, PAD_SIZE = 0.32, 0.145, (0.24, 0.10, 0.002)
PAD_TOP = TOP + PAD_SIZE[2]
PAD = Box("breakdown_pad", pos=(PAD_X, PAD_Y, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=(0.22, 0.22, 0.25))

# The clean-supply shelf on the clean side: a solid block, its top 6 cm above the table.
SHELF_X, SHELF_Y, SHELF_SIZE = 0.30, -0.305, (0.22, 0.12, 0.06)
SHELF_TOP = TOP + SHELF_SIZE[2]
SHELF_LO = (SHELF_X - SHELF_SIZE[0] / 2, SHELF_Y - SHELF_SIZE[1] / 2)
SHELF_HI = (SHELF_X + SHELF_SIZE[0] / 2, SHELF_Y + SHELF_SIZE[1] / 2)
SHELF = Box("clean_shelf", pos=(SHELF_X, SHELF_Y, TOP + SHELF_SIZE[2] / 2), size=SHELF_SIZE, color=(0.08, 0.55, 0.55))

# The three supply boxes, in a row front to back in the carton, 1 cm apart.
BOX, BOX_KG = 0.055, 0.08
BOX_COLORS = {"box_blue": (0.15, 0.35, 0.85), "box_green": (0.15, 0.62, 0.22), "box_orange": (0.95, 0.50, 0.08)}
BOX_NAMES = tuple(BOX_COLORS)
BOX_START_X = (CARTON_X - 0.065, CARTON_X, CARTON_X + 0.065)
BOXES = [
    Box(name, pos=(x, CARTON_Y, CARTON_FLOOR + BOX / 2 + 0.001), size=(BOX, BOX, BOX), mass=BOX_KG, color=color)
    for (name, color), x in zip(BOX_COLORS.items(), BOX_START_X, strict=True)
]  # each starts 1 mm above the carton floor, and settles

# The corrugated insert lies across the boxes' tops, its long side front to back.
INSERT_SIZE, INSERT_KG = (0.18, 0.06, 0.03), 0.06
INSERT = Box("insert", pos=(CARTON_X, CARTON_Y, CARTON_FLOOR + 0.001 + BOX + 0.001 + INSERT_SIZE[2] / 2),
             size=INSERT_SIZE, mass=INSERT_KG, color=(0.62, 0.42, 0.20))

OBJECTS = [*LINE, *CARTON, PAD, SHELF, *BOXES, INSERT]
SUCCESS_HOLD_S = 2.5  # unpacked and still for 2.5 s

BOX_CREDIT, INSERT_CREDIT = 0.3, 0.1
BOX_UPRIGHT_DEG = 10.0  # a box on the shelf: one of its faces down, tilted at most 10 degrees
INSERT_FLAT_DEG = 15.0  # the insert on the pad: its 18 x 6 cm face down (either side), tilted at most 15 degrees
ON_SURFACE_TOL = 0.003  # "on" a surface: the object's bottom within 3 mm of the surface's top (contact rests at zero
# gap, so a box or insert still held still in the gripper at the 8 mm release height is not on the surface)
STILL_SPEED, STILL_SPIN = 0.01, 0.5  # "at rest": moving under 1 cm/s and turning under 0.5 rad/s


def _axes(state) -> np.ndarray:
    """The rotation matrix from the quaternion: its columns are the object's own x, y, z axes in the world."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _axis_tilt_deg(state, axis: int) -> float:
    """Angle between one of the object's own axes and the vertical, whichever way the axis points (0 to 90)."""
    return math.degrees(math.acos(min(1.0, abs(float(_axes(state)[2, axis])))))


def _cube_tilt_deg(state) -> float:
    """A one-colour cube looks the same on any face: the smallest angle from any of its axes to the vertical."""
    return min(_axis_tilt_deg(state, i) for i in range(3))


def _bottom_on(state, surface_z) -> bool:
    return abs(float(state.low[2]) - surface_z) <= ON_SURFACE_TOL


def _at_rest(state) -> bool:
    return state.resting(max_speed=STILL_SPEED, max_spin=STILL_SPIN)


def _footprint_inside(state, size, lo, hi) -> bool:
    """The object's real outline from above (its box, however turned or tipped) inside the axis-aligned rectangle
    lo..hi. The outline reaches sum_j |R[i, j]| * size_j / 2 from the centre along world axis i (i = x, y), so it is
    inside exactly when those reaches fit on both sides."""
    centre = np.asarray(state.pos, dtype=float)[:2]
    reach = np.abs(_axes(state)[:2, :]) @ (np.array(size) / 2)
    return bool(np.all(centre - reach >= np.array(lo)) and np.all(centre + reach <= np.array(hi)))


def _box_on_shelf(state) -> bool:
    """The box's real outline from above inside the shelf top's edges, its bottom on the shelf top, a face down, at
    rest."""
    return bool(
        _footprint_inside(state, (BOX, BOX, BOX), SHELF_LO, SHELF_HI)
        and _bottom_on(state, SHELF_TOP)
        and _cube_tilt_deg(state) <= BOX_UPRIGHT_DEG
        and _at_rest(state)
    )


def _insert_on_pad(state) -> bool:
    """The insert's centre inside the pad's outline, its bottom on the pad, its thin (3 cm) axis vertical, at rest."""
    x, y, _ = (float(v) for v in state.pos)
    return bool(
        abs(x - PAD_X) <= PAD_SIZE[0] / 2
        and abs(y - PAD_Y) <= PAD_SIZE[1] / 2
        and _bottom_on(state, PAD_TOP)
        and _axis_tilt_deg(state, 2) <= INSERT_FLAT_DEG
        and _at_rest(state)
    )


def _over_pass(state) -> bool:
    """The insert's real outline from above (its box, however turned or tipped) overlaps the pass-through's outline.
    Separating-axis test: the outline from above is the sum of the insert's three edges seen from above, so the
    pass-through's two edge directions and the normals to those three edges are the only axes to try."""
    centre = np.asarray(state.pos, dtype=float)[:2]
    edges = _axes(state)[:2, :] * (np.array(INSERT_SIZE) / 2)  # half-edges seen from above, one per column
    pass_c = np.array([PASS_X, LINE_Y])
    pass_half = np.array(PASS_SIZE[:2]) / 2
    normals = [np.array([1.0, 0.0]), np.array([0.0, 1.0])]
    for e in edges.T:
        if np.linalg.norm(e) > 1e-9:
            normals.append(np.array([-e[1], e[0]]) / np.linalg.norm(e))
    for n in normals:
        reach = float(np.sum(np.abs(n @ edges))) + float(np.sum(np.abs(n) * pass_half))
        if abs(float(n @ (centre - pass_c))) >= reach:
            return False  # a gap along this axis: no overlap
    return True


def _insert_crossed(state) -> bool:
    """Brown cardboard at the line, judged on the current (end) state only: any part of the insert past the
    partition's dirty-side face, or any part of its outline from above over the pass-through's outline, at any
    height."""
    return float(state.low[1]) < LINE_FACE_Y or _over_pass(state)


def score(objects):
    """Partial credit (success scores 1.0): 0.3 for each supply box on the clean shelf, a face down, at rest; 0.1 for
    the insert flat on the breakdown pad, at rest. 0 overall if any part of the insert is past the line or over the
    pass-through."""
    if _insert_crossed(objects["insert"]):
        return 0.0
    boxes = sum(_box_on_shelf(objects[b]) for b in BOX_NAMES)
    insert = INSERT_CREDIT if _insert_on_pad(objects["insert"]) else 0.0
    return round(BOX_CREDIT * boxes + insert, 6)


def success(objects):
    """All three boxes on the shelf and the insert on the pad, all at rest; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(_at_rest(objects[n]) for n in (*BOX_NAMES, "insert"))


# The demo: the left arm lifts the insert onto the breakdown pad, then passes the boxes one at a time onto the
# pass-through; the right arm takes each from there to the clean shelf, farthest slot first. Both grip straight down
# with the fingers closing along world y.
TRAVEL_Z = TOP + 0.15  # a carried box's bottom 5.5 cm above the shelf top; the carried insert clears the carton rim
BOX_GRIP = 0.035  # TCP above a box's bottom: pads 3.3 cm up its sides; the thicker fingers stay above the carton rim
INSERT_GRIP = 0.018  # TCP above the insert's bottom: the fingertips stay 5 mm above the boxes' tops
DROP = 0.008  # each object is let go this far above where it lands
SLOT_X = (SHELF_X + 0.065, SHELF_X, SHELF_X - 0.065)  # the boxes' places on the shelf, in the order they arrive
HANDOFF_GAP_S = 1.6  # after the left arm leaves the pass-through, the right arm starts toward it this much later
HOLD_S = 2.5  # the left arm holds each later box above the carton until the right arm has left the pass-through
LEFT_PARK = (0.12, 0.42, TRAVEL_Z)  # outside the carton's left wall, clear of the pad, the carton and the camera's view
RIGHT_WAIT = (0.30, -0.20, TRAVEL_Z)
RIGHT_PARK = (0.15, -0.30, TRAVEL_Z)


def _plan_s(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def demo():
    left = pick_place_waypoints(
        (CARTON_X, CARTON_Y, CARTON_FLOOR + BOX + INSERT_GRIP), (PAD_X, PAD_Y, PAD_TOP + INSERT_GRIP + DROP),
        travel_z=TRAVEL_Z,
    )
    handed = []  # when the left arm has let go of each box and risen clear of the pass-through
    for k, x in enumerate(BOX_START_X):
        leg = pick_place_waypoints(
            (x, CARTON_Y, CARTON_FLOOR + BOX_GRIP), (PASS_X, LINE_Y, PASS_TOP + BOX_GRIP + DROP), travel_z=TRAVEL_Z,
        )
        if k > 0:  # after "lift": wait with the box above the carton
            leg.insert(4, Waypoint(HOLD_S, leg[3].pos, leg[3].finger, "hold until the pass-through is clear"))
        left += leg
        handed.append(_plan_s(left))
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"))

    right = []
    for k, slot_x in enumerate(SLOT_X):
        start = handed[k] + HANDOFF_GAP_S
        wait_at = RIGHT_WAIT if k == 0 else right[-1].pos
        right.append(Waypoint(start - _plan_s(right), wait_at, FINGER_OPEN, "wait for the next box"))
        right += pick_place_waypoints(
            (PASS_X, LINE_Y, PASS_TOP + BOX_GRIP), (slot_x, SHELF_Y, SHELF_TOP + BOX_GRIP + DROP), travel_z=TRAVEL_Z,
        )
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
