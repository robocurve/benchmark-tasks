"""On the station table: an open cool box in the middle of the far side with room for two food packs side by side,
and three packs of shopping in front of the arms: a pink pack of raw chicken (left), a white sandwich box (right) and a
yellow block of cheese (far left). All three must go in the cool box, so two of them have to be stacked; the raw
chicken must not go on top of the others, where its juices could drip onto food that is eaten without cooking. The cool box is fixed; the packs are
loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "pack the shopping into the cool box" --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
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
    "V": ("10001", "10001", "01010", "01010", "01010", "00100", "00100"),
    "F": ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    "H": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "K": ("10001", "10010", "10100", "11000", "10100", "10010", "10001"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "W": ("10001", "10001", "10001", "10101", "10101", "10101", "01010"),
    "Y": ("10001", "10001", "01010", "00100", "00100", "00100", "00100"),
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
BOX_XY, BOX_INNER, BOX_H = (0.28, 0.0), (0.11, 0.22), 0.05  # two packs side by side, 10 cm apart
COOL_BOX = walls("cool_box", BOX_XY, BOX_INNER, BOX_H, (0.2, 0.55, 0.75))

# -- the packs (loose): 8 cm front to back, 6 cm across, 3 cm tall ----------------------------------------------------
PACK = (0.08, 0.06, 0.03)
START = {"raw_chicken": (0.14, 0.12), "sandwich": (0.14, -0.12), "cheese": (0.14, 0.25)}
COLOR = {"raw_chicken": (0.95, 0.6, 0.62), "sandwich": (0.95, 0.94, 0.9), "cheese": (0.98, 0.82, 0.2)}
MASS = {"raw_chicken": 0.12, "sandwich": 0.08, "cheese": 0.08}
PACKS = [Box(n, pos=(*xy, TOP + PACK[2] / 2), size=PACK, mass=MASS[n], color=COLOR[n]) for n, xy in START.items()]
READY_TO_EAT = ("sandwich", "cheese")

OBJECTS = [*COOL_BOX, *PACKS]


# -- scoring ------------------------------------------------------------------------------------------------------
def in_box(objects, name) -> bool:
    o = objects[name]
    return bool(inside(o, BOX_XY, BOX_INNER) and o.low[2] <= TOP + 0.10 and still(objects, name))


def drips(objects) -> bool:
    """The chicken sits above a ready-to-eat pack: higher, and overlapping it from above."""
    c = objects["raw_chicken"]
    for f in READY_TO_EAT:
        o = objects[f]
        over = abs(c.pos[0] - o.pos[0]) < PACK[0] and abs(c.pos[1] - o.pos[1]) < PACK[0]
        if over and c.low[2] > o.pos[2]:
            return True
    return False


def outcome(objects) -> str:
    """'chicken_on_top' (the raw chicken above a ready-to-eat food: 0), 'packed' (all three in the box, chicken not on
    top: success), or 'N_packed'."""
    if drips(objects):
        return "chicken_on_top"
    n = sum(in_box(objects, p) for p in START)
    return "packed" if n == 3 else f"{n}_packed"


def success(objects):
    return outcome(objects) == "packed"


def distance(objects):
    return float(sum(np.hypot(*(np.asarray(objects[p].pos[:2]) - BOX_XY)) for p in START))


def score(objects):
    """0 if the raw chicken is above a ready-to-eat food; otherwise one third per pack in the cool box."""
    o = outcome(objects)
    return 0.0 if o == "chicken_on_top" else (1.0 if o == "packed" else int(o[0]) / 3)


# -- scripted demo ------------------------------------------------------------------------------------------------
def _stack(name, level, dy):
    g = PACK[2] / 2
    return pick_place_waypoints((*START[name], TOP + g), (BOX_XY[0], BOX_XY[1] + dy, TOP + level * PACK[2] + g + 0.008),
                                travel_z=TOP + 0.17)


def demo():
    """The left arm lays the chicken in the left half of the box; the right arm lays the sandwich box beside it in the
    right half; then the left arm stacks the cheese on the sandwich box, so nothing sits under the
    chicken for it to drip onto."""
    first = _stack("raw_chicken", 0, 0.05)
    t = sum(w.duration for w in first)  # one pick and place
    park_l, park_r = (0.15, 0.17, TOP + 0.17), (0.15, -0.17, TOP + 0.17)
    left = first + [Waypoint(t, park_l, FINGER_OPEN, "wait for the sandwich")] + _stack("cheese", 1, -0.05)
    right = [Waypoint(t, park_r, FINGER_OPEN, "wait for the chicken")] + _stack("sandwich", 0, -0.05)
    return {"left": left, "right": right}
