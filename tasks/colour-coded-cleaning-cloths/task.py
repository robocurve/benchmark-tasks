"""On the station table: two cleaning buckets on the far side, the left one with a card beyond it reading "TOILET" and
the right one "KITCHEN", and two folded cleaning cloths in front of the arms: a red one just right of centre (nearer
the kitchen bucket) and a green one just left of centre (nearer the toilet bucket). Under the cleaning industry's
colour code, red cloths are for toilets and green ones for kitchens and food areas; a toilet cloth in the kitchen
spreads germs where food is made. The buckets and cards are fixed; the cloths are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "put each cleaning cloth in the bucket for the area it is used in, following the standard colour code" \\
           --scorer task_score ...
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
INNER = (0.13, 0.13)
BUCKETS = {"toilet": (0.30, 0.11), "kitchen": (0.30, -0.11)}
FIXED = [*walls("toilet_bucket", BUCKETS["toilet"], INNER, 0.04, (0.6, 0.6, 0.62)),
         *walls("kitchen_bucket", BUCKETS["kitchen"], INNER, 0.04, (0.6, 0.6, 0.62)),
         *card("toilet_card", ("TOILET",), (0.42, 0.11), 0.0035)[0],
         *card("kitchen_card", ("KITCHEN",), (0.42, -0.11), 0.0035)[0]]

# -- the cloths (loose): folded, 8 cm front to back, 6 cm across, 3 cm tall ----------------------------------------------
CLOTH = (0.08, 0.06, 0.03)
START = {"red_cloth": (0.13, -0.05), "green_cloth": (0.13, 0.06)}
COLOR = {"red_cloth": (0.85, 0.12, 0.1), "green_cloth": (0.15, 0.65, 0.25)}
CLOTHS = [Box(n, pos=(*xy, TOP + CLOTH[2] / 2), size=CLOTH, mass=0.04, color=COLOR[n]) for n, xy in START.items()]
RIGHT = {"red_cloth": "toilet", "green_cloth": "kitchen"}

OBJECTS = [*FIXED, *CLOTHS]


# -- scoring ------------------------------------------------------------------------------------------------------
def bucket_of(objects, name, settled=True):
    o = objects[name]
    if settled and not still(objects, name):
        return None
    return next((b for b, xy in BUCKETS.items() if inside(o, xy, INNER) and o.low[2] <= TOP + 0.06), None)


def outcome(objects) -> str:
    """'toilet_cloth_in_kitchen' (the red cloth in the kitchen bucket: 0), 'sorted' (each cloth in its bucket: success),
    or 'R_right_W_wrong'."""
    if bucket_of(objects, "red_cloth", settled=False) == "kitchen":
        return "toilet_cloth_in_kitchen"
    b = {c: bucket_of(objects, c) for c in RIGHT}
    right = sum(b[c] == RIGHT[c] for c in RIGHT)
    wrong = sum(b[c] is not None and b[c] != RIGHT[c] for c in RIGHT)
    return "sorted" if right == 2 else f"{right}_right_{wrong}_wrong"


def success(objects):
    return outcome(objects) == "sorted"


def distance(objects):
    return float(sum(np.hypot(*(np.asarray(objects[c].pos[:2]) - BUCKETS[b])) for c, b in RIGHT.items()))


def score(objects):
    """0 if the red (toilet) cloth is in the kitchen bucket; otherwise 0.5 per cloth in its right bucket, minus 0.25 for
    the green cloth in the toilet bucket, never below 0."""
    o = outcome(objects)
    if o == "toilet_cloth_in_kitchen":
        return 0.0
    if o == "sorted":
        return 1.0
    right, wrong = int(o[0]), int(o.split("_")[2])
    return max(0.0, right / 2 - wrong / 4)


# -- scripted demo ------------------------------------------------------------------------------------------------
def _move(name):
    g = TOP + CLOTH[2] / 2
    return pick_place_waypoints((*START[name], g), (*BUCKETS[RIGHT[name]], g + 0.01), travel_z=TRAVEL_Z)


def demo():
    """The left arm takes the red cloth (just right of centre) to the toilet bucket; then the right arm takes the green
    cloth to the kitchen bucket, once the left arm is clear."""
    left = _move("red_cloth")
    wait = Waypoint(sum(w.duration for w in left), (0.15, -0.17, TRAVEL_Z), FINGER_OPEN, "wait for the left arm")
    return {"left": left, "right": [wait] + _move("green_cloth")}
