"""On the station table: a black shelf 15 cm tall and 50 cm long runs away from the arms, left of centre. At its near
end lies a pile of four white A6 pages of card, each printed with a large black numeral: 1, 2, 3 or 4. The pages
are out of order. The pile is sheared: each page lies 1.8 cm further to the right than the one below it, and the
top page's right edge sticks out 1.8 cm past the shelf's right edge. The shelf's top is slick, so the whole pile
slides on it easily, and the rest of the shelf is free: pages can be laid on it. On the table below the pile, against
the shelf's right side, stands one shallow grey tray. The pages are 0.5 mm thick and lie flat, so a gripper cannot
pick one up from above; a page can be pinched by an edge that sticks out past the shelf.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python pick_demo.py task.py out_dir
Second pile order: set the environment variable PAGES_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from sheetkit import Page, Slick  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("File the four numbered pages in the tray in order: page 4 at the bottom, then 3, then 2, with page 1 "
               "on top, each lying flat. The pages are in a mixed pile on the shelf, and you can lay pages on the "
               "shelf while you sort them.")

WHITE = (0.96, 0.96, 0.94)
BLACK = (0.10, 0.10, 0.11)
GREY = (0.52, 0.53, 0.55)
DARK = (0.25, 0.26, 0.28)

# -- pages: A6 card, long side along x (away from the arms) -------------------------------------------------------
SHEET_X, SHEET_Y = 0.148, 0.105
SHEET_T = 0.0005                                               # 0.5 mm, about 400 gsm card
SHEET_MASS = 0.0062                                            # 400 g/m2 x 0.01554 m2
SPAWN_GAP = 0.0002                                             # between pages at spawn; they settle onto each other
# Card on card: static 0.30, kinetic 0.25. With PhysX's default "average" combine mode a finger pad (0.5) on card
# gives 0.40, and card on the slick shelf top (0.02) gives 0.16: about half of card on card, so a hand pressing on
# the pile and sliding moves the whole pile over the shelf rather than one page over another.
PAPER = dict(static_friction=0.30, dynamic_friction=0.25, combine="average")
SLICK = dict(static_friction=0.02, dynamic_friction=0.02, combine="average")

# Numerals: strokes 1.2 cm wide in a box 9 cm tall and 5 cm wide, upright as the top camera sees the page (the
# numeral's top is away from the arms). Each stroke is (centre across, centre up, width across, height up) in m.
_H, _W, _S = 0.090, 0.050, 0.012
_SEG = {
    "a": (0.0, _H / 2 - _S / 2, _W, _S), "g": (0.0, 0.0, _W, _S), "d": (0.0, -_H / 2 + _S / 2, _W, _S),
    "b": (_W / 2 - _S / 2, _H / 4, _S, _H / 2), "c": (_W / 2 - _S / 2, -_H / 4, _S, _H / 2),
    "f": (-_W / 2 + _S / 2, _H / 4, _S, _H / 2), "e": (-_W / 2 + _S / 2, -_H / 4, _S, _H / 2),
    "bar": (0.0, 0.0, _S, _H),
}
_NUMERAL = {1: ("bar",), 2: ("a", "b", "g", "e", "d"), 3: ("a", "b", "g", "c", "d"), 4: ("f", "g", "b", "c")}


def numeral_marks(n: int) -> tuple:
    """The numeral's strokes as page marks (centre x, centre y, size x, size y): up on the page is +x, right is -y."""
    return tuple((up, -across, tall, wide) for across, up, wide, tall in (_SEG[s] for s in _NUMERAL[n]))


# -- layout (world frame: x forward from the midpoint between the arm bases, y left, m) --------------------------
# The shelf's right edge is left of centre so the right hand can reach it level, and the shelf is 15 cm tall
# because a level grip is out of the arm's reach lower down.
SHELF_EDGE = 0.0775                    # y of the shelf's right face
SHELF_DEPTH = 0.200
SHELF_H = 0.150
SHELF_X0, SHELF_X1 = 0.116, 0.612
PILE_X = 0.200                         # the pile, at the shelf's near end, above the tray
PILE_STEP = 0.018
SPOTS = (0.364, 0.528)                 # where along the shelf the demo lays pages down (any free part will do)
TRAY_X = PILE_X
TRAY_LEN, TRAY_WID = 0.160, 0.170      # inside: along the shelf, and outward from the shelf's face
WALL, WALL_H, FLOOR_T = 0.004, 0.025, 0.004
GAP = 0.002                            # shelf face to the tray's floor

# Pile order, top page first. Lot B is a second layout, so the order cannot be learned by rote. In both, pages
# have to be laid aside before page 4 can go into the tray.
LOTS = {"A": (2, 4, 1, 3), "B": (3, 1, 4, 2)}
LOT = os.environ.get("PAGES_LOT", "A")
if LOT not in LOTS:
    raise ValueError(f"PAGES_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
ORDER = LOTS[LOT]
N = len(ORDER)
PAGES = tuple(f"page_{n}" for n in ORDER)              # in pile order, top first
NUMBER_OF = dict(zip(PAGES, ORDER))
WANTED = (4, 3, 2, 1)                                  # the order in the tray, bottom page first

SHELF_TOP = TOP + SHELF_H
TRAY_X0, TRAY_X1 = TRAY_X - TRAY_LEN / 2, TRAY_X + TRAY_LEN / 2     # inside
TRAY_Y_IN = SHELF_EDGE - GAP
TRAY_Y_WALL = TRAY_Y_IN - TRAY_WID                     # inside face of the outer wall
TRAY_Y_OUT = TRAY_Y_WALL - WALL


def _fixed(name, x0, x1, y0, y1, z0, z1, color, material=PAPER):
    """A fixed box from its bounds: x and y in the world frame, z above the table."""
    return Slick(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, TOP + (z0 + z1) / 2),
                 size=(x1 - x0, y1 - y0, z1 - z0), color=color, **material)


def _page(i, name):
    """Page ``i`` counted from the top of the pile (0 = top). It starts level, just above the page below it; the
    top page's right edge is PILE_STEP past the shelf's edge."""
    below = N - 1 - i
    y = SHELF_EDGE - PILE_STEP + SHEET_Y / 2 + i * PILE_STEP
    z = SHELF_TOP + below * (SHEET_T + SPAWN_GAP) + SPAWN_GAP + SHEET_T / 2
    return Page(name, pos=(PILE_X, y, z), size=(SHEET_X, SHEET_Y, SHEET_T), mass=SHEET_MASS, color=WHITE,
                marks=numeral_marks(NUMBER_OF[name]), linear_damping=1.0, angular_damping=2.0, roughness=0.8,
                **PAPER)


SHELF = _fixed("shelf", SHELF_X0, SHELF_X1, SHELF_EDGE, SHELF_EDGE + SHELF_DEPTH, 0.0, SHELF_H, BLACK, material=SLICK)
TRAY = [
    _fixed("tray_floor", TRAY_X0 - WALL, TRAY_X1 + WALL, TRAY_Y_OUT, TRAY_Y_IN, 0.0, FLOOR_T, GREY),
    _fixed("tray_wall_near", TRAY_X0 - WALL, TRAY_X0, TRAY_Y_WALL, TRAY_Y_IN, FLOOR_T, WALL_H, DARK),
    _fixed("tray_wall_far", TRAY_X1, TRAY_X1 + WALL, TRAY_Y_WALL, TRAY_Y_IN, FLOOR_T, WALL_H, DARK),
    _fixed("tray_wall_outer", TRAY_X0 - WALL, TRAY_X1 + WALL, TRAY_Y_OUT, TRAY_Y_WALL, FLOOR_T, WALL_H, DARK),
]
OBJECTS = [SHELF, *TRAY, *(_page(i, name) for i, name in enumerate(PAGES))]

# -- scoring ------------------------------------------------------------------------------------------------------
TRAY_AREA = (TRAY_X0 - WALL, TRAY_X1 + WALL, TRAY_Y_OUT, SHELF_EDGE)     # seen from above: x0, x1, y0, y1
FLAT_DEG = 10.0      # a page lies flat when it is tilted at most this much
EDGE_TOL = 0.002     # a page may reach this far past the tray's area
REST_TOL = 0.006     # a page rests in the tray when its lowest point is within this of the tray's floor


def tilt_deg(state) -> float:
    """Angle between the page's face and the horizontal (0 flat, either side up)."""
    x, y, _, _ = state.quat_xyzw
    return math.degrees(math.acos(min(1.0, abs(1.0 - 2.0 * (x * x + y * y)))))


def where(state) -> str:
    """Where a page is: "tray" (lying flat in it), "shelf" (lying on the shelf) or "loose"."""
    flat = tilt_deg(state) <= FLAT_DEG
    x0, x1, y0, y1 = TRAY_AREA
    inside = (state.low[0] >= x0 - EDGE_TOL and state.high[0] <= x1 + EDGE_TOL
              and state.low[1] >= y0 - EDGE_TOL and state.high[1] <= y1 + EDGE_TOL)
    resting = state.low[2] <= TOP + FLOOR_T + REST_TOL and state.high[2] <= TOP + WALL_H
    if inside and flat and resting:
        return "tray"
    over_shelf = SHELF_X0 <= state.pos[0] <= SHELF_X1 and SHELF_EDGE - PILE_STEP <= state.pos[1] <= SHELF_EDGE + SHELF_DEPTH
    if over_shelf and flat and state.low[2] >= SHELF_TOP - 0.002:
        return "shelf"
    return "loose"


def tray_pile(objects) -> list[int]:
    """The numbers of the pages lying flat in the tray, bottom page first."""
    inside = [(float(objects[name].pos[2]), NUMBER_OF[name]) for name in PAGES if where(objects[name]) == "tray"]
    return [number for _, number in sorted(inside)]


def in_order(objects) -> int:
    """How many pages, counted up from the bottom of the tray's pile, are the wanted ones: 4, then 3, 2, 1."""
    count = 0
    for have, want in zip(tray_pile(objects), WANTED):
        if have != want:
            break
        count += 1
    return count


def tally(objects) -> tuple[int, int]:
    """(pages in order from the bottom of the tray's pile, other pages lying in the tray)."""
    return in_order(objects), len(tray_pile(objects)) - in_order(objects)


def score(objects) -> float:
    """0.25 for each page in order from the bottom of the tray's pile (page 4 lowest, then 3, 2, 1), minus 0.25 for
    each other page lying in the tray; never below 0. A pile dumped into the tray therefore scores 0."""
    right, wrong = tally(objects)
    return max(0, right - wrong) / N


def success(objects) -> bool:
    """All four pages in the tray in order and at rest (the embodiment requires this to hold for 1 s)."""
    return in_order(objects) == N and all(objects[name].resting() for name in PAGES)


# -- two-handed demo (played by pick_demo.py; open loop from the layout above) ------------------------------------
PICK_GRIP_DEPTH = 0.011    # how far the fingertips reach onto a page; less than PILE_STEP
PICK_OPEN = 0.030          # right gripper opening as it comes in (m)
PICK_LIFT = 0.030          # how high above the shelf a page is carried
PICK_OVERLAP = 0.015       # a page drawn out for the tray still reaches this far over the shelf's edge
PICK_SET_GAP = 0.003       # a page is let go this far above what it comes to rest on
PICK_CLEAR = 0.025         # how far the open hand backs away once it has let go
PUSH_PRESS = 0.002         # left fingertips commanded this far below the pile's top while pushing
PUSH_Y = SHELF_EDGE + 0.0925   # where the left fingertips press: over every page left on the pile
GRIP_FULL_OPEN = 0.0939    # the gripper's widest opening (m), for the normalized grip command
