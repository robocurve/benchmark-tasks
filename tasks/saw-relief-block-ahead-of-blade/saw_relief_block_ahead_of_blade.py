"""A mock table saw for setting up a crosscut; nothing runs. On the station table, between the arms, a grey saw
table (36 cm front to back, 58 cm side to side, 3 cm tall) is fixed with its near edge 10 cm in front of the arm bases.
On it are fixed: a yellow steel plate standing in for the blade (7 cm front to back, 4 mm thick, 4 cm tall), 32 to
39 cm in front of the arm bases and 1.6 to 2.0 cm right of centre; a blue rip fence (30 cm long, 2 cm wide, 4 cm tall)
whose working face is 12 cm right of centre, 10 cm right of the blade; and a black miter-gauge bar (16 cm long, 2 cm
deep, 4 cm tall) left of the blade, its working face 26 cm in front of the arm bases, facing away from the arms. Loose:
a red wooden relief block (4 cm cube) on the bare table right of the saw table, and an orange wooden board
(30 x 5 x 3 cm) lying flat on the saw table's near edge, its long side running left to right.

Set up: the block stands against the fence with all of it at least 2 cm nearer the arms than the blade, and the board
lies against the miter gauge with its right end touching the block, 6 cm right of the blade: the offcut. Nothing may
be on the saw table between the blade and the fence (a trapped offcut is thrown back at the operator), the board must
not touch the blade, and the board must not reach the fence.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The saw table: a fixed 30 mm-tall box; everything else on the saw stands on its top.
SAW_X, SAW_Y, SAW_H = (0.10, 0.46), (-0.28, 0.30), 0.030
SAW_TOP = TOP + SAW_H
# The blade stand-in: a 4 mm plate across x, its infeed (near) edge at x 0.32 and its right face at y -0.020.
BLADE_X, BLADE_Y, BLADE_H = (0.32, 0.39), (-0.020, -0.016), 0.040
# The rip fence, right of the blade: its working (left) face at y -0.120, 100 mm from the blade's right face.
FENCE_X, FENCE_Y, FENCE_H = (0.13, 0.43), (-0.140, -0.120), 0.040
FENCE_FACE_Y = FENCE_Y[1]
# The miter gauge, left of the blade: its working face at x 0.26, facing the blade.
GAUGE_X, GAUGE_Y, GAUGE_H = (0.24, 0.26), (0.115, 0.275), 0.040
GAUGE_FACE_X = GAUGE_X[1]
# The beside-blade zone: the saw table between the blade's right face and the fence, along the blade's length.
ZONE_X, ZONE_Y = BLADE_X, (FENCE_FACE_Y, BLADE_Y[0])


def _fixed_box(name, xs, ys, height, color):
    return Box(name, pos=((xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2, SAW_TOP + height / 2),
               size=(xs[1] - xs[0], ys[1] - ys[0], height), color=color)


SAW = [
    Box("saw_table", pos=((SAW_X[0] + SAW_X[1]) / 2, (SAW_Y[0] + SAW_Y[1]) / 2, TOP + SAW_H / 2),
        size=(SAW_X[1] - SAW_X[0], SAW_Y[1] - SAW_Y[0], SAW_H), color=(0.50, 0.52, 0.55)),
    _fixed_box("saw_blade", BLADE_X, BLADE_Y, BLADE_H, (0.95, 0.78, 0.10)),
    _fixed_box("rip_fence", FENCE_X, FENCE_Y, FENCE_H, (0.10, 0.25, 0.75)),
    _fixed_box("miter_gauge", GAUGE_X, GAUGE_Y, GAUGE_H, (0.07, 0.07, 0.08)),
]

BLOCK, BLOCK_KG = 0.040, 0.045  # a 40 mm hardwood cube
BLOCK_SIZE = (BLOCK, BLOCK, BLOCK)
BLOCK_START = (0.22, -0.36)  # on the bare table, right of the saw table
BLOCK_RED = (0.85, 0.12, 0.10)
BOARD_SIZE, BOARD_KG = (0.05, 0.30, 0.03), 0.2  # 300 mm long left to right (y), 50 mm front to back, 30 mm thick
BOARD_START = (0.15, 0.14)  # flat on the saw table's near edge, its right end 1 cm right of centre
BOARD_ORANGE = (0.90, 0.50, 0.12)
OBJECTS = [
    *SAW,
    Box("relief_block", pos=(*BLOCK_START, TOP + BLOCK / 2 + 0.001), size=BLOCK_SIZE, mass=BLOCK_KG,
        color=BLOCK_RED),
    Box("board", pos=(*BOARD_START, SAW_TOP + BOARD_SIZE[2] / 2 + 0.001), size=BOARD_SIZE, mass=BOARD_KG,
        color=BOARD_ORANGE),
]  # each loose object starts 1 mm above what it rests on
SIZES = {"relief_block": BLOCK_SIZE, "board": BOARD_SIZE}
SUCCESS_HOLD_S = 2.5  # set up and still for 2.5 s

BLOCK_CREDIT, BOARD_CREDIT = 0.5, 0.5
SEAT_UP, SEAT_DOWN = 0.0005, 0.003  # "on the saw table": the bottom 0.5 mm above to 3 mm below its top
CONTACT_TOL = 0.005  # "touching": the narrowest gap between the two outlines at most 5 mm
SIDE_TOL = 0.001  # no part more than 1 mm past the fence face, nor the board's part opposite the gauge bar past its face
AHEAD_MIN = 0.020  # every part of the block at least 20 mm nearer the arms than the blade's infeed edge
END_OVERLAP_MIN = 0.015  # the board's end and the block overlap at least 15 mm front to back
BLOCK_TILT_MAX, BLOCK_YAW_MAX = 5.0, 5.0  # degrees; any face of the cube may be down
BOARD_TILT_MAX, BOARD_YAW_MAX = 3.0, 3.0  # degrees; either broad face up, long side within 3 degrees of left-right
ANGLE_EPS = 1e-6  # degrees: a limit reached exactly ("at most 3 degrees" at 3.000) passes despite float rounding
BLADE_CLEAR = 0.003  # the board or block within 3 mm of the blade: touching it
FENCE_CLEAR = 0.010  # the board within 10 mm of the fence: butted against it, the offcut trapped
HAZARD_H = BLADE_H  # the zero rules apply to objects whose bottom is at or below the blade's top: up to 40.5 mm up
# (40 mm plus SEAT_UP), so an object lying on the blade, the fence or the gauge still counts


def _rot(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


_SIGNS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)


def _corners(state, size) -> np.ndarray:
    """The box's eight corners in the world (8 x 3)."""
    return np.asarray(state.pos, dtype=float) + (_SIGNS * np.asarray(size) / 2) @ _rot(state).T


def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _hull(points) -> np.ndarray:
    """Convex hull of 2-D points, counter-clockwise."""
    pts = sorted({(round(float(p[0]), 9), round(float(p[1]), 9)) for p in points})
    if len(pts) < 3:
        return np.array(pts)

    def chain(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and _cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out

    return np.array(chain(pts)[:-1] + chain(pts[::-1])[:-1])


def _footprint(state, size) -> np.ndarray:
    """The box's outline seen from above (its true, turned outline, not the world-aligned box)."""
    return _hull(_corners(state, size)[:, :2])


def _rect(xs, ys) -> np.ndarray:
    return np.array([[xs[0], ys[0]], [xs[1], ys[0]], [xs[1], ys[1]], [xs[0], ys[1]]])


BLADE_RECT, FENCE_RECT, GAUGE_RECT, ZONE_RECT = (
    _rect(BLADE_X, BLADE_Y), _rect(FENCE_X, FENCE_Y), _rect(GAUGE_X, GAUGE_Y), _rect(ZONE_X, ZONE_Y)
)


def _overlap(p, q) -> bool:
    """Two convex outlines share area (touching edges don't count)."""
    for poly in (p, q):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            axis = np.array([-e[1], e[0]])
            if not np.any(axis):
                continue
            pp, qq = p @ axis, q @ axis
            if pp.max() <= qq.min() + 1e-12 or qq.max() <= pp.min() + 1e-12:
                return False
    return True


def _seg_dist(pt, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(pt - a, ab) / max(float(np.dot(ab, ab)), 1e-18), 0.0, 1.0))
    return float(np.linalg.norm(pt - (a + t * ab)))


def _gap(p, q) -> float:
    """The narrowest gap between two convex outlines seen from above (0 if they overlap)."""
    if _overlap(p, q):
        return 0.0
    return min(
        _seg_dist(pt, poly[i], poly[(i + 1) % len(poly)])
        for a, poly in ((p, q), (q, p)) for pt in a for i in range(len(poly))
    )


def _clip_y(poly, lo, hi) -> np.ndarray:
    """The part of a convex outline between y = lo and y = hi (empty if none)."""
    for bound, keep_ge in ((lo, True), (hi, False)):
        out = []
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            a_in = a[1] >= bound if keep_ge else a[1] <= bound
            b_in = b[1] >= bound if keep_ge else b[1] <= bound
            if a_in:
                out.append(a)
            if a_in != b_in:
                out.append(a + (bound - a[1]) / (b[1] - a[1]) * (b - a))
        poly = np.array(out)
        if len(poly) == 0:
            break
    return poly


def _tilt_deg(state, axis: int) -> float:
    """Angle between one of the object's own axes and the vertical, either end up."""
    return math.degrees(math.acos(min(1.0, abs(float(_rot(state)[2, axis])))))


def _cube_tilt_deg(state) -> float:
    """A cube's tilt from resting on whichever face is down."""
    return min(_tilt_deg(state, k) for k in range(3))


def _cube_yaw_off_deg(state) -> float:
    """How far a cube's sides are turned from square to the fence (0 to 45 degrees)."""
    r = _rot(state)
    up = int(np.argmax(np.abs(r[2])))
    side = r[:, (up + 1) % 3]
    a = math.degrees(math.atan2(float(side[1]), float(side[0]))) % 90.0
    return min(a, 90.0 - a)


def _board_yaw_off_deg(state) -> float:
    """How far the board's long side is turned from left-right (world y), 0 to 90 degrees."""
    v = _rot(state)[:, 1]
    a = abs(math.degrees(math.atan2(float(v[0]), float(v[1])))) % 180.0
    return min(a, 180.0 - a)


def _seated(state) -> bool:
    """The bottom on the saw table's top: 0.5 mm above it to 3 mm below."""
    return bool(SAW_TOP - SEAT_DOWN <= float(state.low[2]) <= SAW_TOP + SEAT_UP)


def _hazard(objects) -> bool:
    """A zero rule holds: an object on the saw table beside the blade, the block or board touching the blade, or the
    board reaching the fence. Each applies to an object whose bottom is at or below the blade's top (up to 40.5 mm
    above the saw table), so one lying on the blade or the fence counts."""
    for name, size in SIZES.items():
        s = objects[name]
        if float(s.low[2]) > SAW_TOP + HAZARD_H + SEAT_UP:
            continue
        fp = _footprint(s, size)
        if _overlap(fp, ZONE_RECT) or _gap(fp, BLADE_RECT) < BLADE_CLEAR:
            return True
        if name == "board" and _gap(fp, FENCE_RECT) < FENCE_CLEAR:
            return True
    return False


def _block_set(objects) -> bool:
    """The relief block on the saw table, square, touching the fence on the blade's side, wholly ahead of the blade."""
    b = objects["relief_block"]
    return bool(
        b.resting()
        and _seated(b)
        and _cube_tilt_deg(b) <= BLOCK_TILT_MAX + ANGLE_EPS
        and _cube_yaw_off_deg(b) <= BLOCK_YAW_MAX + ANGLE_EPS
        and _gap(_footprint(b, BLOCK_SIZE), FENCE_RECT) <= CONTACT_TOL
        and float(b.low[1]) >= FENCE_FACE_Y - SIDE_TOL
        and float(b.low[0]) >= FENCE_X[0]
        and float(b.high[0]) <= BLADE_X[0] - AHEAD_MIN
    )


def _board_set(objects) -> bool:
    """The board flat on the saw table against the gauge face, its right end touching the block and bearing on it."""
    w, b = objects["board"], objects["relief_block"]
    corners = _corners(w, BOARD_SIZE)
    end = corners[np.argsort(corners[:, 1])[:4]]  # the right end: its four corners furthest right
    overlap = min(float(end[:, 0].max()), float(b.high[0])) - max(float(end[:, 0].min()), float(b.low[0]))
    fp = _footprint(w, BOARD_SIZE)
    facing = _clip_y(fp, *GAUGE_Y)  # the board's part opposite the gauge bar, between its two ends
    return bool(
        len(facing) >= 3
        and w.resting()
        and _seated(w)
        and _tilt_deg(w, 2) <= BOARD_TILT_MAX + ANGLE_EPS
        and _board_yaw_off_deg(w) <= BOARD_YAW_MAX + ANGLE_EPS
        and _gap(fp, GAUGE_RECT) <= CONTACT_TOL
        and float(facing[:, 0].min()) >= GAUGE_FACE_X - SIDE_TOL
        and _gap(fp, _footprint(b, BLOCK_SIZE)) <= CONTACT_TOL
        and overlap >= END_OVERLAP_MIN
    )


def score(objects):
    """0.5 for the relief block set against the fence ahead of the blade; 0.5 more for the board set against the
    gauge and the block (only with the block set). 0 overall if, with its bottom at or below the blade's top, the
    block or board lies over the beside-blade zone or touches the blade, or the board reaches the fence."""
    if _hazard(objects) or not _block_set(objects):
        return 0.0
    return round(BLOCK_CREDIT + (BOARD_CREDIT if _board_set(objects) else 0.0), 6)


def success(objects):
    """Block and board both set (each term already requires it at rest); must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in SIZES)


# The demo: the right arm stands the block against the fence 2 mm off it, ahead of the blade, and parks; the left
# arm, which has lifted the board meanwhile, lays it 3.5 mm off the gauge face with its end 1.5 mm off the block, and
# parks. Nothing is pushed: each object is set down inside its 5 mm contact band. Both grip with the fingers closing
# front to back (along world x).
TRAVEL_Z = TOP + 0.15  # 12 cm above the saw table: carried objects clear the fence, gauge and blade by 6 cm or more
DROP = 0.005  # each object is let go 5 mm above the saw table
SETTLE_UP = 0.035  # the fast lowering stops 35 mm above the release height, then eases down
BLOCK_GAP, BOARD_GAUGE_GAP, BOARD_END_GAP = 0.002, 0.0035, 0.0015
BLOCK_GOAL = (0.265, FENCE_FACE_Y + BLOCK_GAP + BLOCK / 2)  # outfeed face at x 0.285, 35 mm before the blade
BOARD_GOAL = (GAUGE_FACE_X + BOARD_GAUGE_GAP + BOARD_SIZE[0] / 2,
              BLOCK_GOAL[1] + BLOCK / 2 + BOARD_END_GAP + BOARD_SIZE[1] / 2)
BLOCK_GRIP = 0.030  # TCP above the block's bottom: pads on its lower 23 mm, the finger's wide part above the fence
BOARD_GRIP = 0.020  # TCP above the board's bottom: pads on its full 30 mm side, tips 7 mm above the saw table
BOARD_GRIP_DY = 0.0  # grip at the board's centre: the fingers stay 11 mm clear of the gauge's right end
BLOCK_SQUEEZE = -(BLOCK / 2 - 0.004)  # finger target 4 mm inside each side: the grip centres the block
BOARD_SQUEEZE = -(BOARD_SIZE[0] / 2 - 0.004)
RIGHT_YAW, LEFT_YAW = math.pi / 2, -math.pi / 2  # fingers close along world x
LIFT_CLEAR = SAW_TOP + 0.070  # after letting go, tips 17 mm above the 40 mm block, fence and gauge
RIGHT_PARK = (0.20, -0.40, TOP + 0.13)  # beside the saw table, clear of the camera's view of it
LEFT_PARK = (0.20, 0.40, TOP + 0.13)
PARK_YAW = 0.0
LEFT_CARRY_AT = 14.0  # the left arm holds the board up at its start until the right arm has left the fence
# The loaded left arm sags ~6.5 mm toward its base (-x, +y) at the board's goal, 0.37 m out, until the table takes the
# weight. The gauge and blade tops (40 mm) leave the board a 10 mm slot in x. So the board stages with its bottom 20 mm
# above them, aimed 5 mm further out, comes down slowly through the slot, and steps back in only below the gauge top,
# where the gauge face stops it. It's then set on the table and held so the arm recovers its sag before letting go.
BOARD_AIM_X = 0.005
BOARD_STAGE_UP = 0.060  # board bottom above the saw table while settling: 20 mm above the gauge and blade tops
BOARD_LOW = 0.003  # board bottom above the saw table when stepping in toward the gauge
BOARD_DROP = 0.001  # board bottom above the saw table when let go


def _place(xy, place_z, squeeze, yaw):
    """Lower fast to 35 mm above the release height, settle, ease down, let go, lift slowly clear."""
    x, y = xy
    return [
        Waypoint(1.0, (x, y, place_z + SETTLE_UP), squeeze, "lower", yaw),
        Waypoint(0.6, (x, y, place_z + SETTLE_UP), squeeze, "settle", yaw),
        Waypoint(2.0, (x, y, place_z), squeeze, "ease down", yaw),
        Waypoint(0.3, (x, y, place_z), squeeze, "hold", yaw),
        Waypoint(0.6, (x, y, place_z), FINGER_OPEN, "release", yaw),
        Waypoint(1.0, (x, y, LIFT_CLEAR), FINGER_OPEN, "lift clear", yaw),
        Waypoint(0.8, (x, y, TRAVEL_Z), FINGER_OPEN, "retreat", yaw),
    ]


def _place_board(xy, squeeze, yaw):
    """Stage high and aimed out, come down slowly past the gauge top, step in, set down on the table, let go."""
    x, y = xy
    xa = x + BOARD_AIM_X
    stage_z, low_z, drop_z = (SAW_TOP + h + BOARD_GRIP for h in (BOARD_STAGE_UP, BOARD_LOW, BOARD_DROP))
    return [
        Waypoint(1.5, (xa, y, stage_z), squeeze, "lower", yaw),
        Waypoint(1.5, (xa, y, stage_z), squeeze, "settle", yaw),
        Waypoint(3.0, (xa, y, low_z), squeeze, "ease down", yaw),
        Waypoint(1.0, (x, y, low_z), squeeze, "step in", yaw),
        Waypoint(0.5, (x, y, drop_z), squeeze, "set down", yaw),
        Waypoint(1.0, (x, y, drop_z), squeeze, "hold", yaw),
        Waypoint(0.6, (x, y, drop_z), FINGER_OPEN, "release", yaw),
        Waypoint(1.0, (x, y, LIFT_CLEAR), FINGER_OPEN, "lift clear", yaw),
        Waypoint(0.8, (x, y, TRAVEL_Z), FINGER_OPEN, "retreat", yaw),
    ]


def demo():
    bx, by = BLOCK_START
    pick_z = TOP + 0.001 + BLOCK_GRIP
    right = [
        Waypoint(1.5, (bx, by, TRAVEL_Z), FINGER_OPEN, "to the block", RIGHT_YAW),
        Waypoint(1.5, (bx, by, pick_z), FINGER_OPEN, "descend", RIGHT_YAW),
        Waypoint(0.8, (bx, by, pick_z), BLOCK_SQUEEZE, "close", RIGHT_YAW),
        Waypoint(1.2, (bx, by, TRAVEL_Z), BLOCK_SQUEEZE, "lift", RIGHT_YAW),
        Waypoint(1.8, (*BLOCK_GOAL, TRAVEL_Z), BLOCK_SQUEEZE, "carry", RIGHT_YAW),
        *_place(BLOCK_GOAL, SAW_TOP + DROP + BLOCK_GRIP, BLOCK_SQUEEZE, RIGHT_YAW),
        Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park", PARK_YAW),
    ]
    wx, wy = BOARD_START[0], BOARD_START[1] + BOARD_GRIP_DY
    board_pick_z = SAW_TOP + 0.001 + BOARD_GRIP
    left = [
        Waypoint(1.5, (wx, wy, TRAVEL_Z), FINGER_OPEN, "to the board", LEFT_YAW),
        Waypoint(1.5, (wx, wy, board_pick_z), FINGER_OPEN, "descend", LEFT_YAW),
        Waypoint(0.8, (wx, wy, board_pick_z), BOARD_SQUEEZE, "close", LEFT_YAW),
        Waypoint(1.2, (wx, wy, TRAVEL_Z), BOARD_SQUEEZE, "lift", LEFT_YAW),
    ]
    left.append(Waypoint(LEFT_CARRY_AT - sum(w.duration for w in left), (wx, wy, TRAVEL_Z), BOARD_SQUEEZE,
                         "wait for the block", LEFT_YAW))
    goal = (BOARD_GOAL[0], BOARD_GOAL[1] + BOARD_GRIP_DY)
    left += [
        Waypoint(1.8, (goal[0] + BOARD_AIM_X, goal[1], TRAVEL_Z), BOARD_SQUEEZE, "carry", LEFT_YAW),
        *_place_board(goal, BOARD_SQUEEZE, LEFT_YAW),
        Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park", PARK_YAW),
    ]
    return {"left": left, "right": right}
