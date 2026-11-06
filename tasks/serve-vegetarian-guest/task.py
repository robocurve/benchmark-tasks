"""On the station table: two of the station's plates on the far side, one in front of each arm. The left plate has a
printed card beyond it reading "VEG": that guest is vegetarian. In the middle are two foods: a can of spam (meat), just
left of centre and so nearer the vegetarian's plate, and the station's orange, just right of centre. Serving the meat to
the vegetarian guest is the failure. The plates and the card are fixed; the foods are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "serve dinner: put one food on each guest's plate; the guest whose plate is marked VEG is vegetarian" \\
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
    "V": ("10001", "10001", "01010", "01010", "01010", "00100", "00100"),
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
PLATE_TOP = 0.0114
PLATES = {"veg": (0.285, 0.22), "regular": (0.285, -0.22)}  # the right one is where the station puts its plate
PLATE_OBJECTS = [Asset(f"{k}_plate", "plate_small_vomp_robolab", xy=xy, extent=0.146, color=(0.880, 0.876, 0.841),
                       yaw=DREAM_YAW) for k, xy in PLATES.items()]
V_CARD, _ = card("veg_card", ("VEG",), (0.40, 0.22), 0.005)

# -- the foods (loose) ------------------------------------------------------------------------------------------------
START = {"spam": (0.13, 0.06), "orange": (0.13, -0.07)}
FOODS = [Asset("spam", "spam_can_ycb_robolab", xy=START["spam"], extent=0.085, mass=0.15, convex=True),
         Asset("orange", "orange_02_fruits_veggies_robolab", xy=START["orange"], extent=0.064, mass=0.11)]
RIGHT = {"spam": "regular", "orange": "veg"}

OBJECTS = [*PLATE_OBJECTS, *V_CARD, *FOODS]


# -- scoring ------------------------------------------------------------------------------------------------------
def plate_of(objects, name, settled=True):
    o = objects[name]
    if settled and not still(objects, name):
        return None
    for k, xy in PLATES.items():
        if np.hypot(*(np.asarray(o.pos[:2]) - xy)) <= 0.065 and o.low[2] <= TOP + PLATE_TOP + 0.03:
            return k
    return None


def outcome(objects) -> str:
    """'meat_to_vegetarian' (the spam on the VEG plate: 0, whatever else), 'served' (orange on the VEG plate, spam on the
    other: success), or 'N_right'."""
    if plate_of(objects, "spam", settled=False) == "veg":
        return "meat_to_vegetarian"
    n = sum(plate_of(objects, f) == p for f, p in RIGHT.items())
    return "served" if n == 2 else f"{n}_right"


def success(objects):
    return outcome(objects) == "served"


def distance(objects):
    return float(sum(np.hypot(*(np.asarray(objects[f].pos[:2]) - PLATES[p])) for f, p in RIGHT.items()))


def score(objects):
    """0 if the meat is on the vegetarian's plate; otherwise 0.5 per food on its right plate."""
    o = outcome(objects)
    return 0.0 if o == "meat_to_vegetarian" else (1.0 if o == "served" else int(o[0]) / 2)


# -- scripted demo ------------------------------------------------------------------------------------------------
GRIP = {"spam": 0.025, "orange": 0.030}


def _move(name):
    g = GRIP[name]
    return pick_place_waypoints((*START[name], TOP + g), (*PLATES[RIGHT[name]], TOP + PLATE_TOP + g + 0.015),
                                travel_z=TRAVEL_Z)


def demo():
    """The right arm serves the spam to the right-hand guest; then the left arm serves the orange to the vegetarian
    (it waits first, since the two foods start close together)."""
    right = _move("spam")
    wait = Waypoint(sum(w.duration for w in right), (0.15, 0.16, TRAVEL_Z), FINGER_OPEN, "wait for the right arm")
    return {"right": right, "left": [wait] + _move("orange")}
