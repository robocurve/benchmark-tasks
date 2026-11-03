"""On the station table, in front of the left arm: the start of a small lumber pile. Two dark brown beams (bolsters,
22 cm long) lie on the table running left to right, 16 cm apart, one nearer the arms and one farther out. Two tan
boards (24 cm long, 6 cm wide, 2 cm thick) lie on top of the beams, pointing straight out from the arms, side by side
with a 2 cm gap: this is the first course of the pile. Two red stickers (thin wooden spacer sticks, 16 cm long, 2 cm
square) lie on the table to the right of the pile, across the middle of the table, running left to right. One yellow
board (the same size as the tan ones) lies on the table on the far left side of the pile, pointing straight out. The
beams and the tan boards are fixed; the stickers and the yellow board are loose.

The job is the stacker's next step: lay the stickers across the tan boards, one directly above each beam, so the
weight of the pile passes straight down through sticker, board and beam, then lay the yellow board on the two
stickers directly above one of the tan boards.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x forward from the arms, y to the left, z up (m); the arm bases stand at x 0, y +-0.305.
PILE_Y = 0.21  # the pile's centreline (left to right)
BOLSTER_X = (0.18, 0.34)  # the beams' centrelines, 16 cm apart
BOLSTER = (0.02, 0.22, 0.02)  # x, y, z: each beam runs left to right, 22 cm long
BOARD = (0.24, 0.06, 0.02)  # x, y, z: a board points straight out
BOARD_X = 0.26  # the boards' centre, between the beams
LOWER_Y = (0.17, 0.25)  # the two first-course boards, a 2 cm gap between them
COURSE_Z = TOP + BOLSTER[2] + BOARD[2]  # the top of the first course
STICKER = (0.16, 0.02, 0.02)  # local x is its length
STICKER_KG, BOARD_KG = 0.030, 0.120  # dry softwood at this size

# On the table between the pile and the middle, already running left to right, so the arm carries them without turning
# (a quarter turn in the grip left the ends 4-9 mm off the line in the official sim, 2026-11-03).
STICKER_START = {"sticker_a": (0.20, 0.01), "sticker_b": (0.30, 0.01)}
STICKER_START_YAW = math.pi / 2
TOP_BOARD = "yellow_board"
TOP_BOARD_START = (0.26, 0.37)

BEAM_BROWN, BOARD_TAN = (0.32, 0.20, 0.10), (0.80, 0.56, 0.25)
STICKER_RED, BOARD_YELLOW = (0.80, 0.12, 0.08), (0.95, 0.75, 0.05)  # saturated, so they show in the 320 px top view

OBJECTS = [
    *[Box(f"beam_{k}", pos=(x, PILE_Y, TOP + BOLSTER[2] / 2), size=BOLSTER, color=BEAM_BROWN)
      for k, x in enumerate(BOLSTER_X)],
    *[Box(f"tan_board_{k}", pos=(BOARD_X, y, TOP + BOLSTER[2] + BOARD[2] / 2), size=BOARD, color=BOARD_TAN)
      for k, y in enumerate(LOWER_Y)],
    *[Box(name, pos=(*xy, TOP + STICKER[2] / 2 + 0.0005), size=STICKER, mass=STICKER_KG, color=STICKER_RED,
          yaw=STICKER_START_YAW)
      for name, xy in STICKER_START.items()],
    Box(TOP_BOARD, pos=(*TOP_BOARD_START, TOP + BOARD[2] / 2 + 0.0005), size=BOARD, mass=BOARD_KG,
        color=BOARD_YELLOW),
]
STICKERS = tuple(STICKER_START)

# Limits.
# A piece lies flat on a surface when its lowest corner is within 4 mm of it and it leans at most 10 degrees: a board on
# a broad face, either way up; a sticker (square in section) on any long face, with its length level.
# A sticker lies across the first course when it is flat on the course and its two ends are on either side of it, each
# end at least 1.5 cm past the inner edge of the outer board (so it bears on both boards).
# It is aligned with a beam when both of its ends are within 5 mm (half credit: 15 mm) of that beam's centreline,
# measured along the boards. Both ends, so a sticker laid at an angle across the beam doesn't count.
# The yellow board earns its points only on a sound pile: each sticker across the course within 15 mm of a different
# beam (stickers between the beams let the boards sag, the warp the manual warns about). It is then placed when it is
# flat on top of the stickers, its centre within 1 cm of a tan board's centre (left to right) and of the boards' centre
# (in and out), pointing the same way as the tan boards within 5 degrees.
MAX_TILT_DEG = 10.0
MAX_GAP = 0.004
BEAR = 0.015
ALIGN_FULL, ALIGN_HALF = 0.005, 0.015
BOARD_TOL, BOARD_YAW_DEG = 0.010, 5.0
STICKER_FULL, STICKER_HALF, BOARD_POINTS = 0.25, 0.125, 0.5

COURSE_NEAR_Y = LOWER_Y[0] - BOARD[1] / 2  # the course's edges, left to right
COURSE_FAR_Y = LOWER_Y[1] + BOARD[1] / 2

# Geometry comes from each piece's centre, orientation and known size, never from low/high (rules.md §3).


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def _corners(obj, size):
    r = _rot(obj.quat_xyzw)
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                local = (sx * size[0] / 2, sy * size[1] / 2, sz * size[2] / 2)
                out.append(tuple(float(obj.pos[i]) + sum(r[i][j] * local[j] for j in range(3)) for i in range(3)))
    return out


def _flat_on(obj, size, surface_z) -> bool:
    """Lying flat on the surface: lowest corner within MAX_GAP of it, and leaning at most MAX_TILT_DEG. A board must lie
    on a broad face, either way up (its thickness axis near vertical). A sticker is square in section, so any of its
    four long faces will do: only its length must be near level."""
    r = _rot(obj.quat_xyzw)
    if size[1] == size[2]:  # square in section: the long axis level
        tilt = math.degrees(math.asin(min(1.0, abs(r[2][0]))))
    else:
        tilt = math.degrees(math.acos(min(1.0, abs(r[2][2]))))
    bottom = min(c[2] for c in _corners(obj, size))
    return tilt <= MAX_TILT_DEG and abs(bottom - surface_z) <= MAX_GAP


def _ends_xy(obj, length):
    """x, y of the two ends of the piece's long axis (its local x)."""
    r = _rot(obj.quat_xyzw)
    ax, ay = r[0][0], r[1][0]
    half = length / 2
    px, py = float(obj.pos[0]), float(obj.pos[1])
    return (px - ax * half, py - ay * half), (px + ax * half, py + ay * half)


def _across(sticker) -> bool:
    """Flat on top of the tan boards, bearing at least BEAR on each of them."""
    if not _flat_on(sticker, STICKER, COURSE_Z):
        return False
    (_, y0), (_, y1) = _ends_xy(sticker, STICKER[0])
    near, far = sorted((y0, y1))
    return near <= COURSE_NEAR_Y + BOARD[1] - BEAR and far >= COURSE_FAR_Y - BOARD[1] + BEAR


def _sticker_grade(sticker):
    """(beam index, points) for one sticker: STICKER_FULL or STICKER_HALF when it lies across the course aligned with a
    beam, else (None, 0)."""
    if not _across(sticker):
        return None, 0.0
    (x0, _), (x1, _) = _ends_xy(sticker, STICKER[0])
    beam = min(range(len(BOLSTER_X)), key=lambda k: abs(float(sticker.pos[0]) - BOLSTER_X[k]))
    off = max(abs(x0 - BOLSTER_X[beam]), abs(x1 - BOLSTER_X[beam]))
    if off <= ALIGN_FULL:
        return beam, STICKER_FULL
    if off <= ALIGN_HALF:
        return beam, STICKER_HALF
    return None, 0.0


def _sticker_points(objects) -> float:
    """The best sticker over each beam; two stickers over the same beam count once."""
    best = {}
    for name in STICKERS:
        beam, pts = _sticker_grade(objects[name])
        if beam is not None:
            best[beam] = max(best.get(beam, 0.0), pts)
    return sum(best.values())


def _board_placed(board) -> bool:
    if not _flat_on(board, BOARD, COURSE_Z + STICKER[2]):
        return False
    if abs(float(board.pos[0]) - BOARD_X) > BOARD_TOL:
        return False
    if min(abs(float(board.pos[1]) - y) for y in LOWER_Y) > BOARD_TOL:
        return False
    (x0, y0), (x1, y1) = _ends_xy(board, BOARD[0])
    yaw = math.degrees(math.atan2(y1 - y0, x1 - x0)) % 180.0
    return min(yaw, 180.0 - yaw) <= BOARD_YAW_DEG


def _sound_pile(objects) -> bool:
    """Each sticker across the course within 15 mm of a different beam."""
    beams = [_sticker_grade(objects[name])[0] for name in STICKERS]
    return None not in beams and len(set(beams)) == len(BOLSTER_X)


def score(objects):
    """0.25 for each beam with a sticker across the course aligned within 5 mm (0.125 within 15 mm), plus 0.5 for the
    yellow board flat on the stickers above a tan board, once each sticker is within 15 mm of a different beam. A board
    laid straight on the course, or on stickers that are along a board, between the beams or both over one beam, earns
    nothing; so do stickers left on the table."""
    board = BOARD_POINTS if _sound_pile(objects) and _board_placed(objects[TOP_BOARD]) else 0.0
    pts = _sticker_points(objects) + board
    return round(pts, 6)


def success(objects):
    """Both stickers aligned within 5 mm, one over each beam, the yellow board placed, everything still; held 1 s."""
    return score(objects) >= 1.0 and all(objects[name].resting() for name in (*STICKERS, TOP_BOARD))


def distance(objects):
    """How far the yellow board's centre is from its nearest target spot (m)."""
    b = objects[TOP_BOARD].pos
    return min(math.dist((float(b[0]), float(b[1])), (BOARD_X, y)) for y in LOWER_Y)


# The demo: the left arm alone, coming down onto each piece and leaving straight up.
GRIP_DZ = 0.019  # the gripper's centre point this far above a piece's bottom: pad tips 6 mm above it
TRAVEL_Z = TOP + 0.11  # a carried board's bottom about 1.6 cm over the stickers on the pile
STICKER_OPEN = -0.025  # pads about 50 mm apart: room around a 2 cm sticker, clear of the other one
YAW_ALONG_Y = -math.pi / 2  # fingers close along x, across a piece that runs left to right (+90 is out of reach)
# In the official sim the gripper settles a few degrees short of a -90 degree turn, and closing the fingers turns the
# sticker with it. Measured 2026-11-03, sticker yaw after release: no trim 91.4 degrees (ends 3-4.7 mm off the line,
# limit 5); 4 degrees of trim 88.2. So the demo aims 2 degrees further.
STICKER_YAW = YAW_ALONG_Y - math.radians(2.0)


def _move(pick_xy, place_xy, pick_bottom, place_bottom, pick_yaw, place_yaw, open_to, label):
    pick_z, place_z = pick_bottom + GRIP_DZ, place_bottom + GRIP_DZ + 0.001
    return [
        Waypoint(1.5, (*pick_xy, TRAVEL_Z), open_to, f"{label}: above", pick_yaw),
        Waypoint(1.5, (*pick_xy, pick_z), open_to, f"{label}: down", pick_yaw),
        Waypoint(0.8, (*pick_xy, pick_z), FINGER_CLOSED, f"{label}: grip", pick_yaw),
        Waypoint(1.5, (*pick_xy, TRAVEL_Z), FINGER_CLOSED, f"{label}: lift", pick_yaw),
        Waypoint(2.0, (*place_xy, TRAVEL_Z), FINGER_CLOSED, f"{label}: carry", place_yaw),
        Waypoint(1.5, (*place_xy, place_z), FINGER_CLOSED, f"{label}: set down", place_yaw),
        Waypoint(0.5, (*place_xy, place_z), FINGER_CLOSED, f"{label}: settle", place_yaw),
        Waypoint(0.8, (*place_xy, place_z), open_to, f"{label}: let go", place_yaw),
        Waypoint(1.0, (*place_xy, TRAVEL_Z), open_to, f"{label}: up", place_yaw),
    ]


def demo():
    """The left arm: one sticker onto the near beam, the other onto the far beam (carried straight across, still running
    left to right), then the yellow board onto the stickers above the left tan board."""
    a, b = (STICKER_START[n] for n in STICKERS)
    return {"left": [
        *_move(a, (BOLSTER_X[0], PILE_Y), TOP, COURSE_Z, STICKER_YAW, STICKER_YAW, STICKER_OPEN,
               "sticker over the near beam"),
        *_move(b, (BOLSTER_X[1], PILE_Y), TOP, COURSE_Z, STICKER_YAW, STICKER_YAW, STICKER_OPEN,
               "sticker over the far beam"),
        *_move(TOP_BOARD_START, (BOARD_X, LOWER_Y[1]), TOP, COURSE_Z + STICKER[2], 0.0, 0.0, FINGER_OPEN,
               "yellow board onto the stickers"),
    ]}
