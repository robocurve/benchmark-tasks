"""On the station table: a patient lying on a bed at the far right, and at the left of the bed two bedside stands, a
low one (2 cm) nearer the arms and a tall one (16 cm, above the patient) beyond it. In front of the arms are the
patient's two bags, each with a card in front of it: an IV drip bag (left, nearer the low stand) and a urine drainage
bag (middle). An IV drip runs by gravity, so its bag must hang above the patient; a drainage bag must hang below the
bladder, or urine flows back into it and causes infection. The bed, patient, stands and cards are fixed; the bags are
loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "hang the patient's IV bag and urine drainage bag on the bedside stands, each at the right height" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

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
BED = Box("bed", pos=(0.33, -0.08, TOP + 0.03), size=(0.12, 0.26, 0.06), color=(0.92, 0.93, 0.95))
PATIENT = Box("patient", pos=(0.33, -0.08, TOP + 0.075), size=(0.07, 0.22, 0.03), color=(0.85, 0.7, 0.6))
LEDGE = (0.08, 0.08)
STANDS = {"low": ((0.24, 0.18), 0.02), "high": ((0.36, 0.18), 0.16)}  # top heights above the table; the patient is at 9 cm
STAND_OBJS = [Box(f"{k}_stand", pos=(*xy, TOP + h / 2), size=(*LEDGE, h), color=(0.6, 0.62, 0.66)) for k, (xy, h) in STANDS.items()]

# -- the bags (loose): 6 cm front to back, 4 cm across, 7 cm tall ---------------------------------------------------
BAG = (0.06, 0.04, 0.07)
START = {"iv_bag": (0.12, 0.20), "drainage_bag": (0.12, 0.07)}
COLOR = {"iv_bag": (0.75, 0.88, 0.95), "drainage_bag": (0.95, 0.88, 0.45)}
BAGS = [Box(n, pos=(*xy, TOP + BAG[2] / 2), size=BAG, mass=0.08, color=COLOR[n]) for n, xy in START.items()]
CARDS = [*card("iv_card", ("IV", "DRIP"), (0.055, START["iv_bag"][1]), 0.0025)[0],
         *card("drain_card", ("URINE", "BAG"), (0.055, START["drainage_bag"][1]), 0.0025)[0]]
RIGHT = {"iv_bag": "high", "drainage_bag": "low"}

OBJECTS = [BED, PATIENT, *STAND_OBJS, *BAGS, *CARDS]


# -- scoring ------------------------------------------------------------------------------------------------------
def stand_of(objects, name, settled=True):
    o = objects[name]
    if settled and not still(objects, name):
        return None
    for k, (xy, h) in STANDS.items():
        if inside(o, xy, LEDGE) and abs(o.low[2] - (TOP + h)) <= 0.01:
            return k
    return None


def outcome(objects) -> str:
    """'drainage_above_patient' (the drainage bag on the tall stand: 0), 'hung' (IV bag high, drainage bag low:
    success), or 'N_right'."""
    if stand_of(objects, "drainage_bag", settled=False) == "high":
        return "drainage_above_patient"
    n = sum(stand_of(objects, b) == s for b, s in RIGHT.items())
    return "hung" if n == 2 else f"{n}_right"


def success(objects):
    return outcome(objects) == "hung"


def distance(objects):
    return float(sum(np.hypot(*(np.asarray(objects[b].pos[:2]) - STANDS[s][0])) for b, s in RIGHT.items()))


def score(objects):
    """0 if the drainage bag is on the tall stand (above the patient); otherwise 0.5 for each bag on its right stand."""
    o = outcome(objects)
    return 0.0 if o == "drainage_above_patient" else (1.0 if o == "hung" else int(o[0]) / 2)


# -- scripted demo ------------------------------------------------------------------------------------------------
def _move(name):
    xy, h = STANDS[RIGHT[name]]
    g = BAG[2] / 2
    return pick_place_waypoints((*START[name], TOP + g), (*xy, TOP + h + g + 0.008), travel_z=TOP + 0.27)


def demo():
    """The left arm sets the IV bag on the tall stand, then the drainage bag on the low stand."""
    return {"left": _move("iv_bag") + _move("drainage_bag")}
