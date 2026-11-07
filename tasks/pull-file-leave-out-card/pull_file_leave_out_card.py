"""On the station table, across the midline between the arms: a dark grey painted MDF open-front file shelf (a 1 cm
board, 13.5 cm front to back and 25 cm side to side, running from 9.5 cm right of the midline to 15.5 cm left of it, with a back panel
and two end panels 9 cm tall), its row of files 26 cm in front of the arm bases. Four rigid document cases (box files, 3 cm thick, 12 cm front to back, 15 cm tall, 150 g each) stand
on the board in one row, 6 cm apart centre to centre, from the robot's left: yellow, green, blue, orange. The blue
file stands on the midline. In front of the left arm, 30 cm left of the midline, a dark grey painted MDF out-card holder (a slot
31 mm wide between two 3.5 cm walls) holds a red out-card (2.5 cm thick, 12 cm front to back, 16 cm tall, 60 g). In
front of the right arm, 30 cm right of the midline, stands an empty purple painted MDF reader stand (a slot 36 mm wide between two
4.5 cm walls). The shelf, the holder and the stand are fixed; the files and the card are loose. Done: the blue file
stands in the reader stand, the red out-card stands in the blue file's old place between the green and orange files,
and the yellow, green and orange files are where they started.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The shelf: a 1 cm board, a back panel and two end panels (9 cm tall from the table), open at the front.
SHELF_GREY = (0.22, 0.23, 0.25)
BOARD_T = 0.01
BOARD_TOP = TOP + BOARD_T
SHELF_X = (0.195, 0.33)  # the board, front edge to the back panel
SHELF_Y = (-0.095, 0.155)  # between the end panels
PANEL_T, PANEL_H = 0.005, 0.09
_SX, _SY = sum(SHELF_X) / 2, sum(SHELF_Y) / 2
_SLX, _SLY = SHELF_X[1] - SHELF_X[0], SHELF_Y[1] - SHELF_Y[0]
SHELF = [
    Box("shelf_board", pos=(_SX, _SY, TOP + BOARD_T / 2), size=(_SLX, _SLY, BOARD_T), color=SHELF_GREY),
    Box("shelf_back", pos=(SHELF_X[1] + PANEL_T / 2, _SY, TOP + PANEL_H / 2),
        size=(PANEL_T, _SLY + 2 * PANEL_T, PANEL_H), color=SHELF_GREY),
    Box("shelf_end_left", pos=(_SX, SHELF_Y[1] + PANEL_T / 2, TOP + PANEL_H / 2), size=(_SLX, PANEL_T, PANEL_H),
        color=SHELF_GREY),
    Box("shelf_end_right", pos=(_SX, SHELF_Y[0] - PANEL_T / 2, TOP + PANEL_H / 2), size=(_SLX, PANEL_T, PANEL_H),
        color=SHELF_GREY),
]

# The files: rigid document cases, thickness along y (the row runs left to right), 6 cm apart centre to centre.
FILE_T, FILE_W, FILE_H, FILE_KG = 0.03, 0.12, 0.15, 0.15
ROW_X = 0.26
Y_YELLOW, Y_GREEN, Y_BLUE, Y_ORANGE = 0.12, 0.06, 0.0, -0.06
FILE_START = {
    "file_yellow": (Y_YELLOW, (0.95, 0.78, 0.08)),
    "file_green": (Y_GREEN, (0.10, 0.62, 0.22)),
    "file_blue": (Y_BLUE, (0.12, 0.32, 0.88)),
    "file_orange": (Y_ORANGE, (0.96, 0.45, 0.05)),
}
FILES = [
    Box(name, pos=(ROW_X, y, BOARD_TOP + FILE_H / 2 + 0.001), size=(FILE_W, FILE_T, FILE_H), mass=FILE_KG, color=c)
    for name, (y, c) in FILE_START.items()
]  # each starts 1 mm above the board, and settles
FILE_NAMES = tuple(FILE_START)
NEIGHBOURS = ("file_yellow", "file_green", "file_orange")
BLUE_START = (ROW_X, Y_BLUE)

# The out-card holder (left) and the reader stand (right): a 1 cm base between two walls, open at both ends.
BASE_T, SLOT_L = 0.01, 0.13
SLOT_FLOOR = TOP + BASE_T
CADDY_X, CADDY_Y, CADDY_SLOT, CADDY_WALL_H = 0.25, 0.30, 0.031, 0.035
STAND_X, STAND_Y, STAND_SLOT, STAND_WALL_H = 0.25, -0.30, 0.036, 0.045
STAND_PURPLE = (0.45, 0.12, 0.60)


def _slot(prefix, x, y, slot, wall_h, color):
    off = slot / 2 + PANEL_T / 2
    return [
        Box(f"{prefix}_base", pos=(x, y, TOP + BASE_T / 2), size=(SLOT_L, slot, BASE_T), color=color),
        Box(f"{prefix}_wall_left", pos=(x, y + off, TOP + wall_h / 2), size=(SLOT_L, PANEL_T, wall_h), color=color),
        Box(f"{prefix}_wall_right", pos=(x, y - off, TOP + wall_h / 2), size=(SLOT_L, PANEL_T, wall_h), color=color),
    ]


CADDY = _slot("caddy", CADDY_X, CADDY_Y, CADDY_SLOT, CADDY_WALL_H, SHELF_GREY)
STAND = _slot("reader", STAND_X, STAND_Y, STAND_SLOT, STAND_WALL_H, STAND_PURPLE)

CARD_T, CARD_W, CARD_H, CARD_KG = 0.025, 0.12, 0.16, 0.06
CARD_RED = (0.85, 0.08, 0.08)
CARD = Box("out_card", pos=(CADDY_X, CADDY_Y, SLOT_FLOOR + CARD_H / 2 + 0.001), size=(CARD_W, CARD_T, CARD_H),
           mass=CARD_KG, color=CARD_RED)

OBJECTS = [*SHELF, *FILES, *CADDY, *STAND, CARD]
SUCCESS_HOLD_S = 2.5  # done and still for 2.5 s: long enough to see the card and the files stand on their own

CARD_TOL = 0.020  # the card's centre within 20 mm of the blue file's start: its faces then clear both neighbours
POS_TOL = 0.015  # each neighbour's centre within 15 mm of its own start
TILT_MAX_DEG = 15.0  # standing: the thickness axis within 15 degrees of horizontal; upright: the height edge within
# 15 degrees of vertical as well
DOWN_DEG = 45.0  # knocked over: the thickness axis more than 45 degrees from horizontal (lying on a broad face)
SEAT_TOL = 0.005  # standing on the board or a slot's base: the bottom at most 5 mm above it
SLOT_TOL = 0.005  # in the reader stand: the centre inside the slot's outline or at most 5 mm past it
SHELF_TOL = 0.005  # over the shelf board: the centre inside the board's outline or at most 5 mm past it
TABLE_CLEAR = 0.005  # a file whose bottom is less than 5 mm above the table top stands or lies on the bare table
CARD_CREDIT, STAND_CREDIT, ROW_CREDIT = 0.4, 0.4, 0.2


def _tilt_deg(state) -> float:
    """Angle between the object's long (height) edge and the vertical, whichever end is up (0 to 90 degrees): the
    files and the card are uniform blocks, so one standing upside down looks and stands exactly like an upright one."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _lean_deg(state) -> float:
    """Angle between the object's thickness axis (its local y, across the row) and the horizontal (0 to 90 degrees):
    0 while it stands on any edge or end, 90 when it lies flat on a broad face."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.asin(min(1.0, abs(2 * (y * z + w * x)))))


def _standing(state) -> bool:
    """Standing on an edge or end: the thickness axis within 15 degrees of horizontal."""
    return _lean_deg(state) <= TILT_MAX_DEG


def _upright(state) -> bool:
    """Standing as filed: the thickness axis within 15 degrees of horizontal and the height edge within 15 degrees of
    vertical."""
    return _standing(state) and _tilt_deg(state) <= TILT_MAX_DEG


def _dist_xy(state, xy) -> float:
    return math.hypot(float(state.pos[0]) - xy[0], float(state.pos[1]) - xy[1])


def _seated(state, floor) -> bool:
    """Standing on a surface at height `floor`: bottom within SEAT_TOL of it."""
    return bool(floor - SEAT_TOL <= float(state.low[2]) <= floor + SEAT_TOL)


def _down(state) -> bool:
    """Knocked over (the thickness axis more than 45 degrees from horizontal, lying on a broad face) or off the table
    (centre below the table top)."""
    return _lean_deg(state) > DOWN_DEG or float(state.pos[2]) < TOP


def _over_shelf(state) -> bool:
    """Centre inside the shelf board's outline, or at most 5 mm past it."""
    x, y = float(state.pos[0]), float(state.pos[1])
    return bool(
        SHELF_X[0] - SHELF_TOL <= x <= SHELF_X[1] + SHELF_TOL and SHELF_Y[0] - SHELF_TOL <= y <= SHELF_Y[1] + SHELF_TOL
    )


def _on_shelf(state) -> bool:
    """Centre over the shelf board (5 mm tolerance), bottom at most 5 mm below the board's top."""
    return _over_shelf(state) and float(state.low[2]) >= BOARD_TOP - SEAT_TOL


def _card_in_gap(card) -> bool:
    """The out-card standing in the blue file's old place: centre within 20 mm of it, standing on any edge or end (it
    marks the gap either way), on the board."""
    return _dist_xy(card, BLUE_START) <= CARD_TOL and _standing(card) and _seated(card, BOARD_TOP)


def _in_stand(state) -> bool:
    """Upright in the reader stand's slot, as a reader takes it: centre inside its outline (5 mm tolerance), the
    height edge within 15 degrees of vertical, on its base."""
    return bool(
        abs(float(state.pos[0]) - STAND_X) <= SLOT_L / 2 + SLOT_TOL
        and abs(float(state.pos[1]) - STAND_Y) <= STAND_SLOT / 2 + SLOT_TOL
        and _upright(state)
        and _seated(state, SLOT_FLOOR)
    )


def _in_place(state, name) -> bool:
    """A neighbour as it started: centre within 15 mm, upright, on the board."""
    start = (ROW_X, FILE_START[name][0])
    return _dist_xy(state, start) <= POS_TOL and _upright(state) and _seated(state, BOARD_TOP)


def score(objects):
    """Partial credit (success scores 1.0): 0.4 for the red out-card standing in the blue file's old place; then 0.4
    for the blue file standing in the reader stand and 0.2 for the yellow, green and orange files all where they
    started. 0 overall without the out-card in the gap, if the blue file's centre is still over the shelf board (it was
    slid along the row, not pulled), if a file or the card lies on a broad face (knocked over) or is off the table, if
    a file stands or lies on the bare table, or if the yellow, green or orange file has left the shelf."""
    if any(_down(objects[n]) for n in (*FILE_NAMES, "out_card")):
        return 0.0
    if _over_shelf(objects["file_blue"]):
        return 0.0
    if any(float(objects[n].low[2]) < TOP + TABLE_CLEAR for n in FILE_NAMES):
        return 0.0
    if not all(_on_shelf(objects[n]) for n in NEIGHBOURS):
        return 0.0
    if not _card_in_gap(objects["out_card"]):
        return 0.0
    total = CARD_CREDIT
    if _in_stand(objects["file_blue"]):
        total += STAND_CREDIT
    if all(_in_place(objects[n], n) for n in NEIGHBOURS):
        total += ROW_CREDIT
    return round(total, 6)


def success(objects):
    """Card in the gap, blue file in the stand, the row as it was, all still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in (*FILE_NAMES, "out_card"))


# The demo. Everything is gripped straight down with the fingers closing along world y, across a file's or the
# card's thickness. A file is too tall to lift over its neighbours, so each arm slides in and out of the row from the
# front (the open side of the shelf), its fingers in the 3 cm gaps beside the blue file's slot.
TRAVEL_Z = TOP + 0.165  # a carried file's bottom clears the stand's walls by 1.2 cm; the card's the holder's by 1.3 cm
FILE_GRIP = 0.108  # TCP above a file's bottom: its top 4.2 mm below the palm, the pads on its top 5.5 to 2.2 cm
CARD_GRIP = 0.117  # TCP above the card's bottom: its top 4.3 mm below the palm
DROP = 0.007  # each object is let go this far above where it lands
NUDGE = 0.015  # the blue file is lifted this much off the board before it slides out (clear of any drag)
FINGER_SLOT = -0.020  # fingers 40 mm apart: 5 mm beside the blue file or 7.5 mm beside the card, 5 mm from neighbours
OUT_X = 0.12  # in front of the row: a file or card here (6 to 18 cm from the bases) is clear of the shelf
LEFT_WAIT_UNTIL = 12.0  # the left arm holds the card in front of its holder until the right arm has left the midline


def demo():
    blue_grip = BOARD_TOP + FILE_GRIP
    right = [
        Waypoint(1.5, (OUT_X, Y_BLUE, TRAVEL_Z), FINGER_SLOT, "in front of the blue file, fingers 40 mm apart"),
        Waypoint(1.5, (OUT_X, Y_BLUE, blue_grip), FINGER_SLOT, "descend in front of the row"),
        Waypoint(2.0, (ROW_X, Y_BLUE, blue_grip), FINGER_SLOT, "slide the fingers in beside the blue file"),
        Waypoint(0.8, (ROW_X, Y_BLUE, blue_grip), FINGER_CLOSED, "close"),
        Waypoint(0.8, (ROW_X, Y_BLUE, blue_grip + NUDGE), FINGER_CLOSED, "lift off the board"),
        Waypoint(2.5, (OUT_X, Y_BLUE, blue_grip + NUDGE), FINGER_CLOSED, "slide it out of the row"),
        Waypoint(1.2, (OUT_X, Y_BLUE, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.5, (OUT_X, -0.16, TRAVEL_Z), FINGER_CLOSED, "carry past the shelf"),
        Waypoint(1.5, (STAND_X, STAND_Y, TRAVEL_Z), FINGER_CLOSED, "carry over the reader stand"),
        Waypoint(0.8, (STAND_X, STAND_Y, TRAVEL_Z), FINGER_CLOSED, "let the file stop swinging"),
        Waypoint(1.8, (STAND_X, STAND_Y, SLOT_FLOOR + FILE_GRIP + DROP), FINGER_CLOSED, "lower into the slot"),
        Waypoint(0.6, (STAND_X, STAND_Y, SLOT_FLOOR + FILE_GRIP + DROP), FINGER_OPEN, "release"),
        Waypoint(0.4, (STAND_X, STAND_Y, SLOT_FLOOR + FILE_GRIP + DROP), FINGER_OPEN, "fingers open, still"),
        Waypoint(1.2, (OUT_X, STAND_Y, SLOT_FLOOR + FILE_GRIP + DROP), FINGER_OPEN, "back out along the slot"),
        Waypoint(1.5, (OUT_X, STAND_Y, TRAVEL_Z), FINGER_OPEN, "park"),
    ]
    card_grip = SLOT_FLOOR + CARD_GRIP
    card_place = BOARD_TOP + DROP + CARD_GRIP
    left = [
        Waypoint(1.5, (OUT_X, CADDY_Y, TRAVEL_Z), FINGER_OPEN, "in front of the out-card"),
        Waypoint(1.0, (OUT_X, CADDY_Y, card_grip), FINGER_OPEN, "descend in front of the holder"),
        Waypoint(1.5, (CADDY_X, CADDY_Y, card_grip), FINGER_OPEN, "slide the fingers in beside the card"),
        Waypoint(0.8, (CADDY_X, CADDY_Y, card_grip), FINGER_CLOSED, "close"),
        Waypoint(1.2, (CADDY_X, CADDY_Y, TRAVEL_Z), FINGER_CLOSED, "lift out of the holder"),
        Waypoint(1.5, (OUT_X, CADDY_Y, TRAVEL_Z), FINGER_CLOSED, "draw back"),
    ]
    left.append(Waypoint(LEFT_WAIT_UNTIL - sum(w.duration for w in left), (OUT_X, CADDY_Y, TRAVEL_Z), FINGER_CLOSED,
                         "wait for the blue file to leave"))
    left += [
        Waypoint(1.5, (OUT_X, Y_BLUE, TRAVEL_Z), FINGER_CLOSED, "carry in front of the gap"),
        Waypoint(1.0, (OUT_X, Y_BLUE, card_place), FINGER_CLOSED, "lower to the board's height"),
        Waypoint(2.0, (ROW_X, Y_BLUE, card_place), FINGER_CLOSED, "slide the card into the gap"),
        Waypoint(0.6, (ROW_X, Y_BLUE, card_place), FINGER_SLOT, "release"),
        Waypoint(1.5, (OUT_X, Y_BLUE, card_place), FINGER_SLOT, "back out of the gap"),
        Waypoint(1.0, (OUT_X, Y_BLUE, TRAVEL_Z), FINGER_SLOT, "lift"),
        Waypoint(1.5, (OUT_X, CADDY_Y, TRAVEL_Z), FINGER_SLOT, "park"),
    ]
    return {"left": left, "right": right}
