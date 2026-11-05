"""On the station table, in front of the left arm: a grey mat taped to the table, and on it nine 5 cm cardboard
cubes packed side to side in a 3 x 3 square, with no gaps between them. The cube in the middle of the square is
blue; the other eight are brown. A green pad is taped to the table beside the mat, nearer the left arm's base.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
CUBE = 0.05
CUBE_MASS = 0.06
PITCH = 0.0502  # 0.2 mm between neighbouring cubes: touching, with no room for a finger
CENTRE = (0.32, 0.17)  # the blue cube

MAT_H = 0.002
MAT_X = (0.12, 0.43)  # mat edges (m)
MAT_Y = (-0.04, 0.28)
MAT_TOP = TOP + MAT_H

PAD = (0.08, 0.08, 0.002)
PAD_XY = (0.20, 0.345)
PAD_TOP = TOP + PAD[2]

BROWN, BLUE, GREY, GREEN = (0.62, 0.45, 0.28), (0.15, 0.30, 0.85), (0.55, 0.55, 0.57), (0.20, 0.65, 0.25)

MAT = Box("grey_mat", pos=((MAT_X[0] + MAT_X[1]) / 2, (MAT_Y[0] + MAT_Y[1]) / 2, TOP + MAT_H / 2),
          size=(MAT_X[1] - MAT_X[0], MAT_Y[1] - MAT_Y[0], MAT_H), color=GREY)
PAD_BOX = Box("green_pad", pos=(*PAD_XY, TOP + PAD[2] / 2), size=PAD, color=GREEN)

TARGET = "blue_box"
CUBES = {}
for i in (-1, 0, 1):  # rows along x: -1 nearest the robot
    for j in (-1, 0, 1):  # columns along y: -1 on the robot's right
        name = TARGET if (i, j) == (0, 0) else f"box_{'nmf'[i + 1]}{'rcl'[j + 1]}"
        CUBES[name] = Box(name, pos=(CENTRE[0] + i * PITCH, CENTRE[1] + j * PITCH, MAT_TOP + CUBE / 2),
                          size=(CUBE, CUBE, CUBE), mass=CUBE_MASS, color=BLUE if name == TARGET else BROWN)
OTHERS = [n for n in CUBES if n != TARGET]

OBJECTS = [MAT, PAD_BOX, *CUBES.values()]

SIT_TOL = 0.003  # a cube sits on the mat: its bottom within 3 mm of the mat's top
PAD_TOL = 0.001  # the blue cube sits on the pad or table: bottom within 1 mm (the demo releases it 7 mm up)
FLAT_H = CUBE + 0.003  # lying flat: the cube's height above its bottom at most 53 mm (about 3.4 degrees of tilt)
CLEAR = 0.015  # finger room: 15 mm clear beside each of two opposite faces
EPS = 1e-6  # a value exactly at a stated limit passes


# ---- geometry -------------------------------------------------------------------------------------------------

def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _footprint(s) -> np.ndarray:
    """The cube's outline seen from above: the convex hull of its eight corners projected onto the table."""
    r = _rot(s.quat_xyzw)
    corners = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * CUBE / 2
    pts = sorted(map(tuple, (corners @ r.T)[:, :2] + s.pos[:2]))
    def half(points):
        h = []
        for p in points:
            while len(h) >= 2 and _cross(h[-2], h[-1], p) <= 0:
                h.pop()
            h.append(p)
        return h
    lower, upper = half(pts), half(pts[::-1])
    return np.array(lower[:-1] + upper[:-1])


def _overlap(a: np.ndarray, b: np.ndarray) -> bool:
    """Do two convex polygons overlap (separating-axis test)?"""
    for poly in (a, b):
        for k in range(len(poly)):
            e = poly[(k + 1) % len(poly)] - poly[k]
            length = float(np.hypot(e[0], e[1]))
            if length < 1e-7:
                continue  # a near-zero edge (an upright cube's corners project in near-duplicate pairs) is no axis
            n = np.array([-e[1], e[0]]) / length  # unit normal, so EPS is in metres
            pa, pb = a @ n, b @ n
            if pa.max() <= pb.min() + EPS or pb.max() <= pa.min() + EPS:  # touching is not overlapping
                return False
    return True


def _flat_on(s, surface_z: float, tol: float) -> bool:
    """Lying flat with its bottom on a surface at height surface_z."""
    return bool(abs(s.low[2] - surface_z) <= tol + EPS and s.high[2] - s.low[2] <= FLAT_H + EPS)


def _over_mat(xy) -> bool:
    return bool(MAT_X[0] - EPS <= xy[0] <= MAT_X[1] + EPS and MAT_Y[0] - EPS <= xy[1] <= MAT_Y[1] + EPS)


def _on_mat(s) -> bool:
    """Sitting flat and directly on the mat (not on another cube), its centre over the mat."""
    return _over_mat(s.pos) and _flat_on(s, MAT_TOP, SIT_TOL)


def _clear_pair(objects) -> bool:
    """Finger room: for one pair of the blue cube's opposite side faces, a 15 mm deep strip along each whole face
    (out to 15 mm from the face) overlaps no other cube; touching its outer edge is allowed."""
    s = objects[TARGET]
    r = _rot(s.quat_xyzw)
    horiz = [r[:, k] for k in range(3) if abs(r[2, k]) < 0.5]  # the cube's two horizontal axes
    obstacles = [_footprint(objects[n]) for n in OTHERS]
    for u in horiz:
        u2 = u[:2] / np.linalg.norm(u[:2])
        v2 = np.array([-u2[1], u2[0]])
        ok = True
        for sign in (1, -1):
            c = s.pos[:2] + sign * u2 * (CUBE / 2 + CLEAR / 2)
            strip = np.array([c + a * u2 * CLEAR / 2 + b * v2 * CUBE / 2 for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
            if any(_overlap(strip, o) for o in obstacles):
                ok = False
                break
        if ok:
            return True
    return False


# ---- history: the eight brown cubes must stay on the mat for the whole trial -----------------------------------

_START = {n: np.array(c.pos) for n, c in CUBES.items()}
_left_mat = set()  # brown cubes whose centre has been past the mat's edge at some step this trial
_prev = {}  # every cube's centre at the previous call
RESET_AT = 0.0005  # a new trial: every cube within 0.5 mm of its start and at rest...
RESET_JUMP = 0.05  # ...having got there by a jump of over 50 mm since the previous call (a reset, not a move)


def _track(objects) -> None:
    pos = {n: np.array(objects[n].pos, float) for n in CUBES}
    at_start = all(np.linalg.norm(pos[n] - _START[n]) <= RESET_AT and objects[n].resting() for n in CUBES)
    jumped = not _prev or max(np.linalg.norm(pos[n] - _prev[n]) for n in CUBES) > RESET_JUMP
    if at_start and jumped:
        _left_mat.clear()  # the simulator put every cube back: a new trial
    _prev.clear()
    _prev.update(pos)
    for n in OTHERS:
        if not _over_mat(objects[n].pos):
            _left_mat.add(n)


def _others_ok(objects) -> bool:
    return not _left_mat and all(_on_mat(objects[n]) and objects[n].resting() for n in OTHERS)


# ---- scoring --------------------------------------------------------------------------------------------------

def _on_pad(s) -> bool:
    lo, hi = s.low[:2], s.high[:2]
    px, py = PAD_XY
    return bool(
        px - PAD[0] / 2 - EPS <= lo[0] and hi[0] <= px + PAD[0] / 2 + EPS
        and py - PAD[1] / 2 - EPS <= lo[1] and hi[1] <= py + PAD[1] / 2 + EPS
        and _flat_on(s, PAD_TOP, PAD_TOL)
    )


def success(objects):
    """The blue cube flat on the green pad, wholly within its edges, at rest; the eight brown cubes each sitting
    flat on the mat and at rest, and none of them ever off the mat during the trial."""
    _track(objects)
    blue = objects[TARGET]
    return bool(_others_ok(objects) and _on_pad(blue) and blue.resting())


def distance(objects):
    return float(np.linalg.norm(objects[TARGET].pos[:2] - np.array(PAD_XY)))


def score(objects):
    """1.0 on success. 0.5: the blue cube set down flat off the mat (on the table, or on the pad but over its
    edge), at rest. 0.25: the blue cube flat on the mat, at rest, with finger room on two opposite faces. Each
    tier also needs the brown cubes intact (see success); otherwise 0."""
    _track(objects)
    if not _others_ok(objects):
        return 0.0
    blue = objects[TARGET]
    if not blue.resting():
        return 0.0
    if _on_pad(blue):
        return 1.0
    over_pad = abs(blue.pos[0] - PAD_XY[0]) <= PAD[0] / 2 + EPS and abs(blue.pos[1] - PAD_XY[1]) <= PAD[1] / 2 + EPS
    on_pad_part = over_pad and _flat_on(blue, PAD_TOP, PAD_TOL)  # centre over the pad, overhanging its edge
    if not _over_mat(blue.pos) and (_flat_on(blue, TOP, PAD_TOL) or on_pad_part):
        return 0.5
    if _over_mat(blue.pos) and _flat_on(blue, MAT_TOP, PAD_TOL) and _clear_pair(objects):
        return 0.25
    return 0.0


# ---- demo -----------------------------------------------------------------------------------------------------

TIP_PAST_TCP = 0.013
TRAVEL_Z = TOP + 0.14
PUSH = 0.115  # how far the middle row is pushed toward the robot's right
# Closed fingers pointing down: above the lowest 2 cm the fingers are about 19 mm thick to each side of the TCP
# along y. That part meets the cubes first.
FINGER_SIDE = 0.019
ROW_X = CENTRE[0]
ROW_END_Y = CENTRE[1] + PITCH + CUBE / 2  # the middle row's left end face
PUSH_LAG = 0.015  # measured in simulation (Isaac Sim on an L40S): the row stops 15 mm short of the commanded push
PUSH_Z = MAT_TOP + TIP_PAST_TCP + 0.004  # finger tips 4 mm above the mat
GRIP_Z = MAT_TOP + 0.032  # tips 19 mm above the mat; the fingers' thicker upper part stays above the cubes
GRIP_OPEN = -0.036  # pads 72 mm apart: 11 mm clear of each face of the cube
# Measured in simulation (Isaac Sim on an L40S): with the fingers turned to close along x, the left arm's TCP settles about 18 mm short of
# its target in x (toward the robot), both over the cluster and over the pad. The grip and place aim that far out.
REACH_DX = 0.018
YAW = -np.pi / 2  # fingers close along x (+90 degrees would need joint6 past its limit)


def demo():
    y0 = ROW_END_Y + FINGER_SIDE + 0.008  # 8 mm short of the row's end
    y1 = ROW_END_Y + FINGER_SIDE - PUSH - PUSH_LAG
    bx, by = ROW_X + REACH_DX, CENTRE[1] - PUSH  # where the blue cube ends up (x aimed past it, see REACH_DX)
    px, py = PAD_XY[0] + REACH_DX, PAD_XY[1]
    place_z = PAD_TOP + CUBE / 2 + 0.007
    w = [
        Waypoint(1.5, (ROW_X, y0, TRAVEL_Z), FINGER_CLOSED, "over the row's end"),
        Waypoint(1.5, (ROW_X, y0, PUSH_Z), FINGER_CLOSED, "down beside it"),
        Waypoint(5.0, (ROW_X, y1, PUSH_Z), FINGER_CLOSED, "push the middle row"),
        Waypoint(0.8, (ROW_X, y1 + 0.012, PUSH_Z), FINGER_CLOSED, "back off"),
        Waypoint(1.2, (ROW_X, y1 + 0.012, TRAVEL_Z), FINGER_CLOSED, "up"),
        Waypoint(1.5, (bx, by, TRAVEL_Z), GRIP_OPEN, "over the blue cube", YAW),
        Waypoint(1.5, (bx, by, GRIP_Z), GRIP_OPEN, "descend", YAW),
        Waypoint(0.8, (bx, by, GRIP_Z), FINGER_CLOSED, "grip", YAW),
        Waypoint(1.2, (bx, by, TRAVEL_Z), FINGER_CLOSED, "lift", YAW),
        Waypoint(1.8, (px, py, TRAVEL_Z), FINGER_CLOSED, "carry", YAW),
        Waypoint(1.2, (px, py, place_z), FINGER_CLOSED, "lower", YAW),
        Waypoint(0.6, (px, py, place_z), FINGER_OPEN, "release", YAW),
        Waypoint(1.0, (px, py, TRAVEL_Z), FINGER_OPEN, "retreat", YAW),
    ]
    return {"left": w}
