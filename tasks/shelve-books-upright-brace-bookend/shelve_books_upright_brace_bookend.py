"""On the station table, a fixed brown wooden shelf runs right from just left of the midline: a 15 mm board 11 cm deep
and 30 cm long, its front edge 22.5 cm in front of the arm bases, with an end wall closing its left end (8 cm left of
the midline) and a back wall behind it, both rising 8 cm above the board's top. In front of the left arm, a fixed dark
grey book return tray (inside 9 cm front to back, 13 cm side to side, walls 4 cm tall) holds three pocket books
standing upright in three slots, red on the left, green in the middle, blue on the right: each 76 mm deep, 34 mm
thick, 120 mm tall, 250 g, its 76 mm side pointing away from the arms. In front of the right arm stands a black acetal
plastic bookend block (70 mm deep, 40 mm thick, 120 mm tall, 450 g). Shelved, the three books stand upright on the
board in one closed row starting at the end wall, in any order, and the bookend stands against the right side of the
last book.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The shelf: a 15 mm board on the table, an end wall at its left end and a back wall, both 8 cm above the board.
SHELF_X = (0.225, 0.335)  # the board's front and back edges (the back wall's front face)
END_Y = 0.08  # the end wall's inner (right) face; the board runs right from here
SHELF_END_Y = -0.22  # the board's open right end
BOARD_T, WALL_T, WALL_H = 0.015, 0.01, 0.08
SHELF_TOP = TOP + BOARD_T
WOOD = (0.55, 0.33, 0.15)
_WALL_Z, _WALL_SZ = (TOP + SHELF_TOP + WALL_H) / 2, BOARD_T + WALL_H
SHELF = [
    Box("shelf_board", pos=(sum(SHELF_X) / 2, (END_Y + SHELF_END_Y) / 2, TOP + BOARD_T / 2),
        size=(SHELF_X[1] - SHELF_X[0], END_Y - SHELF_END_Y, BOARD_T), color=WOOD),
    Box("shelf_back", pos=(SHELF_X[1] + WALL_T / 2, (END_Y + WALL_T + SHELF_END_Y) / 2, _WALL_Z),
        size=(WALL_T, END_Y + WALL_T - SHELF_END_Y, _WALL_SZ), color=WOOD),
    Box("shelf_end", pos=(sum(SHELF_X) / 2, END_Y + WALL_T / 2, _WALL_Z),
        size=(SHELF_X[1] - SHELF_X[0], WALL_T, _WALL_SZ), color=WOOD),
]

# The return tray: a 5 mm floor, 5 mm walls 4 cm tall, and two dividers making three 40 mm slots.
TRAY_X, TRAY_Y = 0.15, 0.23
TRAY_IN = (0.09, 0.13)
TRAY_T, TRAY_H = 0.005, 0.04
TRAY_FLOOR = TOP + TRAY_T
SLOT_PITCH = 0.045
TRAY_GREY = (0.22, 0.23, 0.26)
_TZ = TOP + TRAY_H / 2
TRAY = [
    Box("tray_floor", pos=(TRAY_X, TRAY_Y, TOP + TRAY_T / 2), size=(*TRAY_IN, TRAY_T), color=TRAY_GREY),
    Box("tray_wall_near", pos=(TRAY_X - TRAY_IN[0] / 2 - TRAY_T / 2, TRAY_Y, _TZ),
        size=(TRAY_T, TRAY_IN[1] + 2 * TRAY_T, TRAY_H), color=TRAY_GREY),
    Box("tray_wall_far", pos=(TRAY_X + TRAY_IN[0] / 2 + TRAY_T / 2, TRAY_Y, _TZ),
        size=(TRAY_T, TRAY_IN[1] + 2 * TRAY_T, TRAY_H), color=TRAY_GREY),
    Box("tray_wall_left", pos=(TRAY_X, TRAY_Y + TRAY_IN[1] / 2 + TRAY_T / 2, _TZ), size=(TRAY_IN[0], TRAY_T, TRAY_H),
        color=TRAY_GREY),
    Box("tray_wall_right", pos=(TRAY_X, TRAY_Y - TRAY_IN[1] / 2 - TRAY_T / 2, _TZ), size=(TRAY_IN[0], TRAY_T, TRAY_H),
        color=TRAY_GREY),
    *[
        Box(f"tray_divider_{i}", pos=(TRAY_X, TRAY_Y + s * SLOT_PITCH / 2, (TRAY_FLOOR + TOP + TRAY_H) / 2),
            size=(TRAY_IN[0], TRAY_T, TRAY_H - TRAY_T), color=TRAY_GREY)
        for i, s in ((1, 1), (2, -1))
    ],
]

# The books: depth front to back (x), thickness side to side (y), height up (z).
BOOK_D, BOOK_T, BOOK_H, BOOK_KG = 0.076, 0.034, 0.12, 0.25
BOOK_COLORS = {"book_red": (0.80, 0.12, 0.10), "book_green": (0.10, 0.55, 0.20), "book_blue": (0.12, 0.30, 0.85)}
BOOK_SLOT_Y = {"book_red": TRAY_Y + SLOT_PITCH, "book_green": TRAY_Y, "book_blue": TRAY_Y - SLOT_PITCH}
BOOK_NAMES = tuple(BOOK_COLORS)
BOOKS = [
    Box(name, pos=(TRAY_X, BOOK_SLOT_Y[name], TRAY_FLOOR + BOOK_H / 2 + 0.001), size=(BOOK_D, BOOK_T, BOOK_H),
        mass=BOOK_KG, color=BOOK_COLORS[name])
    for name in BOOK_NAMES
]  # each starts 1 mm above the tray floor, and settles

BE_D, BE_T, BE_H, BE_KG = 0.07, 0.04, 0.12, 0.45  # as tall as the books: holding it, the fingers stay above them
BE_START = (0.15, -0.22)
BOOKEND = Box("bookend", pos=(*BE_START, TOP + BE_H / 2 + 0.001), size=(BE_D, BE_T, BE_H), mass=BE_KG,
              color=(0.08, 0.08, 0.09))

OBJECTS = [*SHELF, *TRAY, *BOOKS, BOOKEND]
SUCCESS_HOLD_S = 2.5  # shelved and still for 2.5 s: long enough to see that no book leans

UPRIGHT_DEG = 5.0  # a book or the bookend counts as upright when its tall edges are at most 5 degrees from vertical
SPINE_DEG = 10.0  # a book's 76 mm side at most 10 degrees from front to back (either 76 mm end facing the arms)
FLAT_DEG = 45.0  # a book over the board tilted more than this lies flat (on its side or its fore-edge)
GAP_TOL = 0.005  # at most 5 mm between a book and the end wall, two books, or the last book and the bookend
FACE_OVERLAP = 0.02  # the bookend's and the last book's front-to-back extents overlap by at least 2 cm
ON_BOARD_TOL = 0.002  # bottom within 2 mm of the board's top: set down on the board (a settled object sits at 0 mm)
OVERHANG_TOL = 0.01  # seen from above, no part of a shelved book or the bookend more than 1 cm beyond the board's edges
STACK_SLACK = 0.01  # a book whose bottom is within 1 cm of another's top (above or below it), over it, rests on it
ROW_CREDIT, SHELVED_CREDIT = 0.2, 0.1  # per book
BOOKEND_CREDIT, BOOKEND_SHELVED_CREDIT = 0.4, 0.2


def _tilt_deg(state) -> float:
    """Angle between the object's tall axis and the vertical, either way up."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _spine_off_deg(state) -> float:
    """Angle between the object's long horizontal side (its x edge) and the front-to-back direction, 0 to 90."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    yaw = math.degrees(math.atan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z)))
    return abs((yaw + 90.0) % 180.0 - 90.0)


def _over_board(state) -> bool:
    """Centre inside the board's outline seen from above."""
    x, y, _ = (float(v) for v in state.pos)
    return bool(SHELF_X[0] <= x <= SHELF_X[1] and SHELF_END_Y <= y <= END_Y)


def _footprint_on_board(state) -> bool:
    """Seen from above, the whole object within OVERHANG_TOL of the board's outline."""
    lo, hi = state.low, state.high
    return bool(
        float(lo[0]) >= SHELF_X[0] - OVERHANG_TOL and float(hi[0]) <= SHELF_X[1] + OVERHANG_TOL
        and float(lo[1]) >= SHELF_END_Y - OVERHANG_TOL and float(hi[1]) <= END_Y + OVERHANG_TOL
    )


def _standing_on_board(state) -> bool:
    """Footprint on the board, bottom within ON_BOARD_TOL of its top, upright."""
    return bool(
        _footprint_on_board(state)
        and abs(float(state.low[2]) - SHELF_TOP) <= ON_BOARD_TOL
        and _tilt_deg(state) <= UPRIGHT_DEG
    )


def _shelved(state) -> bool:
    """A book standing upright on the board with its 76 mm side front to back (either end facing the arms)."""
    return _standing_on_board(state) and _spine_off_deg(state) <= SPINE_DEG


def _overlap_xy(a, b) -> bool:
    return bool(np.all(a.low[:2] < b.high[:2]) and np.all(b.low[:2] < a.high[:2]))


def _book_misplaced(objects) -> bool:
    """A book lying flat over the board, or resting on another book or the bookend: the posture rule is broken."""
    for name in BOOK_NAMES:
        book = objects[name]
        if _over_board(book) and _tilt_deg(book) > FLAT_DEG:
            return True
        for other in (*BOOK_NAMES, "bookend"):
            if other != name and _overlap_xy(book, objects[other]) and (
                abs(float(book.low[2]) - float(objects[other].high[2])) <= STACK_SLACK
            ):
                return True
    return False


def _row(objects) -> list[str]:
    """The books in the closed row: from the end wall rightwards, each shelved book whose leftmost point is at most
    GAP_TOL right of the wall's face, or of the previous book's rightmost point (seen from above, measured side to
    side; turned or leaning books are measured at their outermost corner or edge)."""
    shelved = sorted((n for n in BOOK_NAMES if _shelved(objects[n])), key=lambda n: -float(objects[n].pos[1]))
    row, edge = [], END_Y
    for name in shelved:
        if edge - float(objects[name].high[1]) > GAP_TOL:
            break
        row.append(name)
        edge = float(objects[name].low[1])
    return row


def _braces(last, end) -> bool:
    """Against the last book's right side: the bookend's centre right of the book's, its leftmost point within
    GAP_TOL of the book's rightmost (either side: a gap of at most 5 mm, or at most 5 mm of overlap), and their
    front-to-back extents overlapping by at least FACE_OVERLAP (not in front of, behind, or off a corner of it)."""
    overlap_x = min(float(end.high[0]), float(last.high[0])) - max(float(end.low[0]), float(last.low[0]))
    return bool(
        float(end.pos[1]) < float(last.pos[1])
        and abs(float(last.low[1]) - float(end.high[1])) <= GAP_TOL
        and overlap_x >= FACE_OVERLAP
    )


def score(objects):
    """Partial credit (success scores 1.0): per book, 0.2 when it is shelved (upright within 5 degrees, bottom within
    2 mm of the board's top, no part more than 1 cm beyond its edges, 76 mm side within 10 degrees of front to back)
    in the closed row from the end wall (gaps at most 5 mm), or 0.1 when shelved but not in the row. The bookend: 0.4
    when upright on the board against the right side of the third book of a complete row (side-to-side gap or overlap
    at most 5 mm, front-to-back extents overlapping by at least 2 cm), or 0.2 when upright on the board otherwise with
    at least one book in the row. 0 overall if a book lies flat over the board or rests on another book or the
    bookend."""
    if _book_misplaced(objects):
        return 0.0
    row = _row(objects)
    books = sum(
        ROW_CREDIT if n in row else SHELVED_CREDIT if _shelved(objects[n]) else 0.0 for n in BOOK_NAMES
    )
    end = objects["bookend"]
    if not row or not _standing_on_board(end):
        bookend = 0.0
    elif len(row) == len(BOOK_NAMES) and _braces(objects[row[-1]], end):
        bookend = BOOKEND_CREDIT
    else:
        bookend = BOOKEND_SHELVED_CREDIT
    return round(books + bookend, 6)


def success(objects):
    """Every book in the closed row and the bookend against it, all still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in (*BOOK_NAMES, "bookend"))


# The demo: the left arm takes the books from the tray, the one nearest the shelf first, carries each along the
# shelf's front edge (the end wall is taller than a carried book's bottom), moves it in over the board 12 mm right of
# its spot, steps it left and sets it down 2 mm from the end wall or the previous book. Then the right arm stands the
# bookend against the right side of the last book. Both grip straight down turned 90 degrees, the fingers closing front to back:
# a finger between two books would leave a gap, so they hold the front and back faces.
GRIP_YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # the same grip; each turn keeps that arm's wrist in range
ROUTE_X = 0.17  # carried books pass the end wall here: their back faces 1.7 cm in front of the board
TRAVEL_Z = TOP + 0.16  # a carried book's bottom clears the tray walls by 2 cm; open fingertips clear book tops by 12 mm
GRIP_BELOW_TOP = 0.02  # TCP this far below an object's top: the pads cover its top 33 mm, the fingertips above walls
DROP = 0.008  # each object is let go this far above the board
PLACE_GAP = 0.002  # set down this far from the end wall, the previous book, or (the bookend) the last book
SIDE_STEP = 0.012  # come in over the board this far right of the spot, then step left onto it
BOOK_X = 0.27  # books' and bookend's centre: 7 mm behind the board's front edge, 27 mm in front of the back wall
DEMO_ORDER = ("book_blue", "book_green", "book_red")  # nearest the shelf first: no carried book passes over another
ROW_Y = tuple(END_Y - PLACE_GAP - BOOK_T / 2 - i * (BOOK_T + PLACE_GAP) for i in range(3))
BE_GAP = 0.0  # the bookend is set down touching the last book: the 5 mm brace tolerance then holds either way
BE_Y = ROW_Y[-1] - BOOK_T / 2 - BE_GAP - BE_T / 2
LEFT_PARK = (0.18, 0.30, TRAVEL_Z)
RIGHT_PARK = (0.15, -0.30, TRAVEL_Z)
RIGHT_WAIT_S = 36.0  # the right arm waits above the bookend until the left arm has parked
# The 450 g bookend rides 8-10 mm lower in the grip than planned (on the GPU it reached the board ~0.4 s before the
# lowering ended), so it is aimed 13 mm up: it then hangs ~4 mm above the board when let go instead of being jammed
# into it, which tilted it in the grip and made it jump sideways off the last book on release.
BE_DROP = 0.013


def _shelve(name, row_y):
    sy = BOOK_SLOT_Y[name]
    pick_z = TRAY_FLOOR + BOOK_H - GRIP_BELOW_TOP
    place_z = SHELF_TOP + DROP + BOOK_H - GRIP_BELOW_TOP
    side_y = row_y - SIDE_STEP
    o, c, a = FINGER_OPEN, FINGER_CLOSED, GRIP_YAW["left"]
    return [
        Waypoint(1.5, (TRAY_X, sy, TRAVEL_Z), o, "to the book", a),
        Waypoint(1.5, (TRAY_X, sy, pick_z), o, "descend", a),
        Waypoint(0.8, (TRAY_X, sy, pick_z), c, "close", a),
        Waypoint(1.2, (TRAY_X, sy, TRAVEL_Z), c, "lift", a),
        Waypoint(1.5, (ROUTE_X, side_y, TRAVEL_Z), c, "along the shelf front", a),
        Waypoint(1.5, (BOOK_X, side_y, TRAVEL_Z), c, "in over the board", a),
        Waypoint(0.6, (BOOK_X, row_y, TRAVEL_Z), c, "step left to the row", a),
        Waypoint(1.2, (BOOK_X, row_y, place_z), c, "lower", a),
        Waypoint(0.6, (BOOK_X, row_y, place_z), o, "release", a),
        Waypoint(1.0, (BOOK_X, row_y, TRAVEL_Z), o, "lift clear", a),
    ]


def demo():
    left = []
    for name, row_y in zip(DEMO_ORDER, ROW_Y, strict=True):
        left += _shelve(name, row_y)
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park", GRIP_YAW["left"]))
    o, c, a = FINGER_OPEN, FINGER_CLOSED, GRIP_YAW["right"]
    pick_z = TOP + BE_H - GRIP_BELOW_TOP
    place_z = SHELF_TOP + BE_DROP + BE_H - GRIP_BELOW_TOP
    right = [
        Waypoint(RIGHT_WAIT_S, (*BE_START, TRAVEL_Z), o, "wait for the books", a),
        Waypoint(1.5, (*BE_START, pick_z), o, "descend", a),
        Waypoint(0.8, (*BE_START, pick_z), c, "close", a),
        Waypoint(1.2, (*BE_START, TRAVEL_Z), c, "lift", a),
        Waypoint(1.8, (BOOK_X, BE_Y - SIDE_STEP, TRAVEL_Z), c, "carry", a),
        Waypoint(1.0, (BOOK_X, BE_Y, TRAVEL_Z), c, "step left to the row", a),
        Waypoint(1.0, (BOOK_X, BE_Y, TRAVEL_Z), c, "let it stop swinging", a),
        Waypoint(1.5, (BOOK_X, BE_Y, place_z), c, "lower", a),
        Waypoint(0.4, (BOOK_X, BE_Y, place_z), c, "hold still", a),
        Waypoint(0.6, (BOOK_X, BE_Y, place_z), o, "release", a),
        Waypoint(0.5, (BOOK_X, BE_Y, place_z), o, "let it settle", a),
        Waypoint(1.5, (BOOK_X, BE_Y, TRAVEL_Z), o, "lift clear", a),
        Waypoint(1.5, RIGHT_PARK, o, "park", a),
    ]
    return {"left": left, "right": right}
