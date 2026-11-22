"""On the station table: a change dish in the middle of the far side, a printed card beyond it reading "PAID 1.00" and
"PRICE 0.30", and five chunky play coins standing in a row in front of the arms, each with a printed card in front of it
giving its value in pence: 50, 10, 5, 20 and 10. The customer paid one pound for a 30p item, so their change is 70p:
50 + 20, or 50 + 10 + 10. Short-changing the customer is the failure. The dish and cards are fixed; the coins are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "give the customer their change: they paid one pound for a 30p item; put exactly the right coins in the change dish" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DREAM_YAW = np.pi / 2  # the station's own objects are turned this way
TRAVEL_Z = TOP + 0.15

# -- printed cards (fixed): a white board with black block letters, read from the top camera -----------------------
FONT = {  # 5 x 7 bitmaps, top row first
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "D": ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "G": ("01111", "10000", "10000", "10011", "10001", "10001", "01111"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "P": ("11110", "10001", "10001", "11110", "10000", "10000", "10000"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "V": ("10001", "10001", "10001", "10001", "10001", "01010", "00100"),
    "0": ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    "1": ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    "2": ("01110", "10001", "00001", "00110", "01000", "10000", "11111"),
    "3": ("11110", "00001", "00001", "01110", "00001", "00001", "11110"),
    "5": ("11111", "10000", "11110", "00001", "00001", "10001", "01110"),
    ".": ("00000", "00000", "00000", "00000", "00000", "01100", "01100"),
    " ": ("00000", "00000", "00000", "00000", "00000", "00000", "00000"),
}
GLYPH_W, GLYPH_H, GAP, LINE_GAP, MARGIN = 5, 7, 1, 3, 2  # in font cells
BOARD_T, INK_T = 0.003, 0.001


def card(name, lines, center_xy, cell):
    """A fixed white card centred at ``center_xy`` with ``lines`` of block capitals, one black box per horizontal run of
    lit cells. The top camera looks back at the arms, so the first line is the farthest from them and letters run from
    left (+y) to right (-y), readable from the arms' side. Returns (objects, (depth, width))."""
    cols = max(len(t) for t in lines) * (GLYPH_W + GAP) - GAP + 2 * MARGIN
    rows = len(lines) * GLYPH_H + (len(lines) - 1) * LINE_GAP + 2 * MARGIN
    size = (rows * cell, cols * cell)
    far_x, left_y = center_xy[0] + size[0] / 2, center_xy[1] + size[1] / 2
    ink = []
    for li, text in enumerate(lines):
        start = (cols - 2 * MARGIN - (len(text) * (GLYPH_W + GAP) - GAP)) // 2
        for r in range(GLYPH_H):
            row = "".join(FONT[ch][r] + "0" * GAP for ch in text)
            c = 0
            while c < len(row):
                if row[c] != "1":
                    c += 1
                    continue
                e = c
                while e < len(row) and row[e] == "1":
                    e += 1
                x = far_x - (MARGIN + li * (GLYPH_H + LINE_GAP) + r + 0.5) * cell
                y = left_y - (MARGIN + start + (c + e) / 2) * cell
                ink.append(Box(f"{name}_ink_{len(ink)}", pos=(x, y, TOP + BOARD_T + INK_T / 2), size=(cell, (e - c) * cell, INK_T),
                               color=(0.05, 0.05, 0.05)))
                c = e
    board = Box(name, pos=(*center_xy, TOP + BOARD_T / 2), size=(*size, BOARD_T), color=(0.97, 0.97, 0.95))
    return [board, *ink], size


def walls(name, xy, inner, h, color, t=0.01):
    """Four fixed walls ``t`` thick and ``h`` tall around an ``inner`` (front to back, left to right) space centred at
    ``xy``; the table is the floor."""
    (x, y), (dx, dy) = xy, inner
    ox, oy, z = dx / 2 + t / 2, dy / 2 + t / 2, TOP + h / 2
    return [Box(f"{name}_front", pos=(x - ox, y, z), size=(t, dy + 2 * t, h), color=color),
            Box(f"{name}_back", pos=(x + ox, y, z), size=(t, dy + 2 * t, h), color=color),
            Box(f"{name}_left", pos=(x, y + oy, z), size=(dx, t, h), color=color),
            Box(f"{name}_right", pos=(x, y - oy, z), size=(dx, t, h), color=color)]


def inside(item, xy, inner) -> bool:
    return abs(item.pos[0] - xy[0]) <= inner[0] / 2 and abs(item.pos[1] - xy[1]) <= inner[1] / 2


# -- stillness ----------------------------------------------------------------------------------------------------
_STEP = {"objects": None, "prev": {}, "cur": {}}


def still(objects, name) -> bool:
    """At rest: yam_arena's resting() (under 1 cm/s and 0.5 rad/s), or not moved at all since the previous control
    step (under 0.5 mm, and under 0.01 in each quaternion component). The second test is needed because PhysX can
    put a light, settled body to sleep while it still reports the small velocity it had just before, so resting()
    would never become true for it. The hooks run once per control step on the same ``objects``, so a new
    ``objects`` dict marks a new step."""
    if objects is not _STEP["objects"]:
        _STEP["objects"], _STEP["prev"] = objects, _STEP["cur"]
        _STEP["cur"] = {n: np.r_[np.asarray(o.pos, dtype=float), np.asarray(o.quat_xyzw, dtype=float)]
                        for n, o in objects.items()}
    now, last = _STEP["cur"][name], _STEP["prev"].get(name)
    unmoved = (last is not None and float(np.linalg.norm(now[:3] - last[:3])) < 0.0005
               and float(np.abs(now[3:] - last[3:]).max()) < 0.01)
    return bool(objects[name].resting() or unmoved)



# Positions on the table from the midpoint between the arm bases: x forward, y to the left (m).
DISH_XY, DISH_INNER = (0.30, 0.0), (0.14, 0.14)
DISH = walls("change_dish", DISH_XY, DISH_INNER, 0.025, (0.4, 0.3, 0.2))
RECEIPT, _ = card("receipt", ("PAID 1.00", "PRICE 0.30"), (0.44, 0.0), 0.0035)

# -- the coins (loose): play coins 3 cm thick, bigger for higher values, 12 cm apart ---------------------------------
SILVER, BRASS, COPPER = (0.78, 0.79, 0.8), (0.82, 0.68, 0.3), (0.72, 0.45, 0.25)
COINS = {  # name: (pence, radius, colour, start xy)
    "p50": (50, 0.030, SILVER, (0.14, 0.24)), "p10_a": (10, 0.024, SILVER, (0.14, 0.12)),
    "p5": (5, 0.021, SILVER, (0.14, 0.0)), "p20": (20, 0.027, BRASS, (0.14, -0.12)),
    "p10_b": (10, 0.024, SILVER, (0.14, -0.24)),
}
H = 0.03
COIN_OBJECTS = [Cylinder(n, pos=(*xy, TOP + H / 2), radius=r, height=H, mass=0.04, color=c)
                for n, (v, r, c, xy) in COINS.items()]
LABELS = [o for n, (v, r, c, xy) in COINS.items() for o in card(f"label_{n}", (str(v),), (xy[0] - 0.06, xy[1]), 0.003)[0]]
CHANGE = 70

OBJECTS = [*DISH, *RECEIPT, *COIN_OBJECTS, *LABELS]


# -- scoring ------------------------------------------------------------------------------------------------------
def in_dish(objects, name) -> bool:
    o = objects[name]
    return bool(inside(o, DISH_XY, DISH_INNER) and o.low[2] <= TOP + 0.07 and still(objects, name))


def given(objects) -> int:
    return sum(COINS[n][0] for n in COINS if in_dish(objects, n))


def outcome(objects) -> str:
    """'exact' (70p: success), 'short_N' (less than 70p: the customer is short-changed), or 'over_N'."""
    g = given(objects)
    return "exact" if g == CHANGE else (f"short_{g}" if g < CHANGE else f"over_{g}")


def success(objects):
    return outcome(objects) == "exact"


def distance(objects):
    return abs(given(objects) - CHANGE) / 100


def score(objects):
    """1.0 for exactly 70p; 0.25 for too much (the shop loses); 0 for too little (the customer is short-changed)."""
    g = given(objects)
    return 1.0 if g == CHANGE else (0.25 if g > CHANGE else 0.0)


# -- scripted demo ------------------------------------------------------------------------------------------------
def _move(name, xy):
    start = COINS[name][3]
    return pick_place_waypoints((*start, TOP + H / 2), (*xy, TOP + H / 2 + 0.008), travel_z=TRAVEL_Z)


def demo():
    """The left arm puts the 50p in the dish; the right arm waits above its side, then puts the 20p beside it."""
    left = _move("p50", (DISH_XY[0], DISH_XY[1] + 0.035))
    wait = Waypoint(sum(w.duration for w in left), (0.15, -0.16, TRAVEL_Z), FINGER_OPEN, "wait for the left arm")
    return {"left": left, "right": [wait] + _move("p20", (DISH_XY[0], DISH_XY[1] - 0.035))}
