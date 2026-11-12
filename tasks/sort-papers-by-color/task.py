"""On the station table: a black shelf 15 cm tall and 50 cm long runs away from the arms, left of centre. At its near
end lies a pile of four A6 sheets of colored card (white, yellow and pink, one color twice, in mixed order). The pile is
sheared: each sheet lies 1.8 cm further to the right than the one below it, and the top sheet's right edge sticks
out 1.8 cm past the shelf's right edge. The shelf's top is slick, so the whole pile slides on it easily. On the
table along the shelf's right side stand three shallow trays in a row, each marked with the color it takes by its
outer wall and a strip on its floor: yellow nearest the arms, below the pile, then pink, then white. The sheets are
0.5 mm thick and lie flat, so a gripper cannot pick one up from above; the top sheet can be pinched by the edge
that sticks out.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Pick demo: python pick_demo.py task.py out_dir
Second pile order: set the environment variable PAPER_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from sheetkit import Slick  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Sort the stack of colored sheets: take each sheet off the stack and put it in the tray marked with "
               "the same color, and leave every sheet lying flat in its tray.")

# -- colors (sRGB) ------------------------------------------------------------------------------------------------
COLORS = {"white": (0.96, 0.96, 0.94), "yellow": (1.00, 0.84, 0.10), "pink": (1.00, 0.42, 0.62)}
BLACK = (0.10, 0.10, 0.11)
GREY = (0.52, 0.53, 0.55)

# -- sheets: A6 card, long side along x (away from the arms) ------------------------------------------------------
SHEET_X, SHEET_Y = 0.148, 0.105
SHEET_T = 0.0005                                               # 0.5 mm, about 400 gsm card
SHEET_MASS = 0.0062                                             # 400 g/m2 x 0.01554 m2
SPAWN_GAP = 0.0002                                              # between sheets at spawn; they settle onto each other
# Card on card: static 0.30, kinetic 0.25. With PhysX's default "average" combine mode a finger pad (0.5) on card
# gives 0.40, and card on the slick shelf top (0.02) gives 0.16: about half of card on card, so a hand pressing on
# the pile and sliding moves the whole pile over the shelf rather than one sheet over another.
PAPER = dict(static_friction=0.30, dynamic_friction=0.25, combine="average")
SLICK = dict(static_friction=0.02, dynamic_friction=0.02, combine="average")

# -- layout (world frame: x forward from the midpoint between the arm bases, y left, m) --------------------------
# The shelf's right edge is left of centre so the right hand can reach it level, and the shelf is 15 cm tall
# because a level grip is out of the arm's reach lower down. The trays' x positions are where the right hand can
# set a sheet down.
SHELF_EDGE = 0.0775                    # y of the shelf's right face
SHELF_DEPTH = 0.200                    # room for the sheared pile and for a hand to wait beside it
SHELF_H = 0.150
TRAYS = (("yellow", 0.200), ("pink", 0.364), ("white", 0.528))   # color, x of the tray's centre
TRAY_X = dict(TRAYS)
TRAY_LEN, TRAY_WID = 0.160, 0.170      # inside: along the shelf, and outward from the shelf's face
WALL, WALL_H, FLOOR_T = 0.004, 0.025, 0.004
GAP = 0.002                            # shelf face to the trays' floor
LABEL_W, LABEL_T = 0.012, 0.001        # the colored strip on each tray's floor, along its outer wall
# The pile: above the nearest tray. A squared stack was tried first: under a fingertip every layer has almost the
# same friction, so which layer slips is not determined, and three sheets moved together. Sheared, only the top
# sheet's edge sticks out, and a pinch shallower than the step cannot take a second sheet.
PILE_X = 0.200
PILE_STEP = 0.018

# Stack order, top sheet first. Lot B is a second layout, so the order cannot be learned by rote.
LOTS = {"A": ("pink", "white", "yellow", "pink"),
        "B": ("yellow", "pink", "white", "yellow")}
LOT = os.environ.get("PAPER_LOT", "A")
if LOT not in LOTS:
    raise ValueError(f"PAPER_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
ORDER = LOTS[LOT]
N = len(ORDER)
SHEETS = tuple(f"sheet_{i + 1}" for i in range(N))      # sheet_1 is on top
# Each sheet's centre across the shelf: the top sheet's right edge is PILE_STEP past the shelf's edge.
SHEET_YC = {name: SHELF_EDGE - PILE_STEP + SHEET_Y / 2 + i * PILE_STEP for i, name in enumerate(SHEETS)}
COLOR_OF = dict(zip(SHEETS, ORDER))

SHELF_TOP = TOP + SHELF_H
ROW_X0 = TRAYS[0][1] - TRAY_LEN / 2 - WALL              # the tray row, outside to outside
ROW_X1 = TRAYS[-1][1] + TRAY_LEN / 2 + WALL
ROW_Y_IN = SHELF_EDGE - GAP                             # the row's edge by the shelf
ROW_Y_WALL = ROW_Y_IN - TRAY_WID                        # inside face of the outer wall
ROW_Y_OUT = ROW_Y_WALL - WALL


def _fixed(name, x0, x1, y0, y1, z0, z1, color, material=PAPER):
    """A fixed box from its bounds: x and y in the world frame, z above the table."""
    return Slick(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, TOP + (z0 + z1) / 2),
                 size=(x1 - x0, y1 - y0, z1 - z0), color=color, **material)


def _tray_row():
    """Three trays as one unit: a floor, four dividers, and for each tray an outer wall and a floor strip in its
    color. The shelf's face is the trays' fourth side."""
    parts = [_fixed("tray_floor", ROW_X0, ROW_X1, ROW_Y_OUT, ROW_Y_IN, 0.0, FLOOR_T, GREY)]
    pitch = TRAY_LEN + WALL
    for k in range(len(TRAYS) + 1):
        x = ROW_X0 + k * pitch
        parts.append(_fixed(f"tray_divider_{k + 1}", x, x + WALL, ROW_Y_WALL, ROW_Y_IN, FLOOR_T, WALL_H, GREY))
    for color, xc in TRAYS:
        c = COLORS[color]
        parts.append(_fixed(f"tray_{color}_wall", xc - TRAY_LEN / 2 - WALL / 2, xc + TRAY_LEN / 2 + WALL / 2,
                            ROW_Y_OUT, ROW_Y_WALL, FLOOR_T, WALL_H, c))
        parts.append(_fixed(f"tray_{color}_label", xc - TRAY_LEN / 2, xc + TRAY_LEN / 2,
                            ROW_Y_WALL, ROW_Y_WALL + LABEL_W, FLOOR_T, FLOOR_T + LABEL_T, c))
    return parts


def _sheet(i, name):
    """Sheet ``i`` counted from the top (0 = top). It starts level, just above the sheet below it."""
    below = N - 1 - i
    z = SHELF_TOP + below * (SHEET_T + SPAWN_GAP) + SPAWN_GAP + SHEET_T / 2
    return Slick(name, pos=(PILE_X, SHEET_YC[name], z), size=(SHEET_X, SHEET_Y, SHEET_T), mass=SHEET_MASS,
                 color=COLORS[COLOR_OF[name]], linear_damping=1.0, angular_damping=2.0, roughness=0.8, **PAPER)


SHELF = _fixed("shelf", ROW_X0, ROW_X1, SHELF_EDGE, SHELF_EDGE + SHELF_DEPTH, 0.0, SHELF_H, BLACK, material=SLICK)
OBJECTS = [SHELF, *_tray_row(), *(_sheet(i, name) for i, name in enumerate(SHEETS))]

# -- scoring ------------------------------------------------------------------------------------------------------
# A tray's area, seen from above: between the middles of its dividers, from the outside of its outer wall to the
# shelf's face (x0, x1, y0, y1).
TRAY_AREA = {color: (xc - TRAY_LEN / 2 - WALL / 2, xc + TRAY_LEN / 2 + WALL / 2, ROW_Y_OUT, SHELF_EDGE)
             for color, xc in TRAYS}
FLAT_DEG = 10.0      # a sheet lies flat when it is tilted at most this much
EDGE_TOL = 0.002     # a sheet may reach this far past a tray's area
REST_TOL = 0.006     # a sheet rests in a tray when its lowest point is within this of the tray's floor


def tilt_deg(state) -> float:
    """Angle between the sheet's face and the horizontal (0 flat, either side up)."""
    x, y, _, _ = state.quat_xyzw
    return math.degrees(math.acos(min(1.0, abs(1.0 - 2.0 * (x * x + y * y)))))


def where(state) -> str:
    """Where a sheet is: a tray's color, "shelf" (lying on the shelf) or "loose"."""
    flat = tilt_deg(state) <= FLAT_DEG
    for color, (x0, x1, y0, y1) in TRAY_AREA.items():
        inside = (state.low[0] >= x0 - EDGE_TOL and state.high[0] <= x1 + EDGE_TOL
                  and state.low[1] >= y0 - EDGE_TOL and state.high[1] <= y1 + EDGE_TOL)
        resting = state.low[2] <= TOP + FLOOR_T + REST_TOL and state.high[2] <= TOP + WALL_H
        if inside and flat and resting:
            return color
    over_shelf = ROW_X0 <= state.pos[0] <= ROW_X1 and SHELF_EDGE - PILE_STEP <= state.pos[1] <= SHELF_EDGE + SHELF_DEPTH
    if over_shelf and flat and state.low[2] >= SHELF_TOP - 0.002:
        return "shelf"
    return "loose"


def tally(objects) -> tuple[int, int]:
    """(sheets lying flat in the tray of their own color, sheets lying flat in another color's tray)."""
    right = wrong = 0
    for name in SHEETS:
        place = where(objects[name])
        right += place == COLOR_OF[name]
        wrong += place in TRAY_AREA and place != COLOR_OF[name]
    return right, wrong


def score(objects) -> float:
    """Sheets lying flat in their own tray minus sheets lying flat in a wrong tray, divided by four; never below 0."""
    right, wrong = tally(objects)
    return max(0, right - wrong) / N


def success(objects) -> bool:
    """All four sheets flat in their own trays and at rest (the embodiment requires this to hold for 1 s)."""
    return tally(objects)[0] == N and all(objects[name].resting() for name in SHEETS)


# -- two-handed sort (played by pick_demo.py; open loop from the layout above) ------------------------------------
# The right hand, level and pointing left, pinches the top sheet by the edge that sticks out, no deeper than the
# pile's step, and lifts it straight up. It draws the sheet out, carries it along the shelf to its tray and lowers
# it until the sheet's far edge is just above the shelf's edge, then opens fully and backs away: the sheet tips off
# the edge and slides down into the tray. Meanwhile the left hand, closed and pointing down, presses on the pile
# where every remaining sheet overlaps and slides the whole pile one step to the right, so the next sheet sticks out.
PICK_GRIP_DEPTH = 0.011    # how far the fingertips reach onto the sheet; less than PILE_STEP
PICK_OPEN = 0.030          # right gripper opening as it comes in (m)
PICK_LIFT = 0.030          # straight up, before drawing the sheet out
PICK_OVERLAP = 0.015       # the drawn-out sheet's far edge still reaches this far over the shelf's edge
PICK_SET_GAP = 0.003       # that far edge is let go this far above what it comes to rest on
PICK_CLEAR = 0.025         # how far the open hand backs away once it has let go
PUSH_PRESS = 0.002         # left fingertips commanded this far below the pile's top while pushing
PUSH_Y = SHELF_EDGE + 0.0925   # where the left fingertips press: over every remaining sheet
GRIP_FULL_OPEN = 0.0939    # the gripper's widest opening (m), for the normalized grip command
