"""On the station table: four place mats in a row across the far side, two of them with a printed name card beyond
them, "ANA" (far left) and "SAM" (inner right), and two white cups in front of the arms. Two guests, Ana and Sam, are
coming: each should find a cup at their own place. The nearest mat to the left cup is an unnamed one, so laying cups by
proximity seats a guest at the wrong place. The mats and cards are fixed; the cups are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "Ana and Sam are coming for tea: put a cup at each of their places" --scorer task_score ...
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
MAT_X, MAT_SIZE = 0.31, (0.12, 0.12)
MATS = {"far_left": 0.24, "inner_left": 0.08, "inner_right": -0.08, "far_right": -0.24}
MAT_OBJECTS = [Box(f"mat_{k}", pos=(MAT_X, y, TOP + 0.0004), size=(*MAT_SIZE, 0.0008), color=(0.55, 0.75, 0.6))
               for k, y in MATS.items()]
NAMED = {"far_left": "ANA", "inner_right": "SAM"}
NAME_CARDS = [o for k, n in NAMED.items() for o in card(f"card_{n.lower()}", (n,), (0.415, MATS[k]), 0.004)[0]]

# -- the cups (loose): plain white cups 5.6 cm across, 8 cm tall -----------------------------------------------------
R, H = 0.028, 0.08
START = {"cup_left": (0.14, 0.12), "cup_right": (0.14, -0.12)}
CUPS = [Cylinder(n, pos=(*xy, TOP + H / 2), radius=R, height=H, mass=0.05, color=(0.96, 0.96, 0.96))
        for n, xy in START.items()]

OBJECTS = [*MAT_OBJECTS, *NAME_CARDS, *CUPS]


# -- scoring ------------------------------------------------------------------------------------------------------
def mat_of(objects, name):
    o = objects[name]
    if not (still(objects, name) and o.low[2] <= TOP + 0.01):
        return None
    return next((k for k, y in MATS.items() if inside(o, (MAT_X, y), MAT_SIZE)), None)


def tally(objects):
    m = [mat_of(objects, c) for c in START]
    right = len({k for k in m if k in NAMED})
    wrong = sum(k is not None and k not in NAMED for k in m)
    return right, wrong


def outcome(objects) -> str:
    """'set' (a cup at Ana's place and at Sam's: success), or 'R_right_W_wrong'."""
    right, wrong = tally(objects)
    return "set" if right == 2 else f"{right}_right_{wrong}_wrong"


def success(objects):
    return outcome(objects) == "set"


def distance(objects):
    return float(sum(min(np.hypot(*(np.asarray(objects[c].pos[:2]) - (MAT_X, MATS[k]))) for k in NAMED) for c in START))


def score(objects):
    """0.5 for each named place with a cup, minus 0.25 for each cup at an unnamed place, never below 0."""
    right, wrong = tally(objects)
    return max(0.0, right / 2 - wrong / 4)


# -- scripted demo ------------------------------------------------------------------------------------------------
def _move(name, mat):
    return pick_place_waypoints((*START[name], TOP + 0.04), (MAT_X, MATS[mat], TOP + 0.048), travel_z=TOP + 0.17)


def demo():
    """The left arm puts its cup at Ana's place; the right arm puts its cup at Sam's."""
    return {"left": _move("cup_left", "far_left"), "right": _move("cup_right", "inner_right")}
