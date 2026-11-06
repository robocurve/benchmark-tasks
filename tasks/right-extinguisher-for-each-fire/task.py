"""On the station table: a deep-fat fryer at the far left with a card beyond it reading "FRYER", and a computer
monitor at the far right with a card reading "PC", each with a red floor mark in front of it where an extinguisher
should stand. In a row in front of the arms stand four fire extinguishers, each with a card in front of it naming its
type: WATER (far left), CO2, WET CHEM and FOAM (far right). The instruction names the wet-chemical one for the fryer and
the CO2 one for the computer, so the robot must read the type cards; water on burning fat throws it out in a fireball, and water on live electrics conducts.
The fryer, monitor, marks and cards are fixed; the extinguishers are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "stand the wet chemical extinguisher on the mark by the fryer and the CO2 extinguisher on the mark by the computer" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DREAM_YAW = np.pi / 2  # the station's own objects are turned this way

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
FRYER = Box("fryer", pos=(0.34, 0.18, TOP + 0.04), size=(0.10, 0.10, 0.08), color=(0.72, 0.73, 0.75))
OIL = Box("fryer_oil", pos=(0.34, 0.18, TOP + 0.0805), size=(0.08, 0.08, 0.002), color=(0.75, 0.55, 0.15))
MONITOR = Box("monitor", pos=(0.36, -0.18, TOP + 0.07), size=(0.03, 0.16, 0.12), color=(0.1, 0.1, 0.12))
KEYBOARD = Box("keyboard", pos=(0.31, -0.18, TOP + 0.005), size=(0.04, 0.12, 0.01), color=(0.2, 0.2, 0.22))
SPOTS = {"fryer": (0.22, 0.18), "pc": (0.22, -0.18)}
MAT = (0.10, 0.10)
MARKS = [Box(f"{k}_mark", pos=(*xy, TOP + 0.0004), size=(*MAT, 0.0008), color=(0.8, 0.15, 0.12)) for k, xy in SPOTS.items()]
RISK_CARDS = [*card("fryer_card", ("FRYER",), (0.435, 0.18), 0.0035)[0], *card("pc_card", ("PC",), (0.435, -0.18), 0.0035)[0]]

# -- the extinguishers (loose): 6 cm across, 14 cm tall, 12 cm apart, each with its type card in front -------------
R, H = 0.03, 0.14
TYPES = {  # name: (card lines, colour, start xy)
    "water": (("WATER",), (0.8, 0.12, 0.1), (0.12, 0.18)),
    "co2": (("CO2",), (0.12, 0.12, 0.12), (0.12, 0.06)),
    "wet_chemical": (("WET", "CHEM"), (0.95, 0.8, 0.15), (0.12, -0.06)),
    "foam": (("FOAM",), (0.93, 0.88, 0.7), (0.12, -0.18)),
}
EXTINGUISHERS = [Cylinder(n, pos=(*xy, TOP + H / 2), radius=R, height=H, mass=0.2, color=c)
                 for n, (lines, c, xy) in TYPES.items()]
TYPE_CARDS = [o for n, (lines, c, xy) in TYPES.items() for o in card(f"{n}_card", lines, (0.055, xy[1]), 0.0025)[0]]
RIGHT = {"fryer": "wet_chemical", "pc": "co2"}

OBJECTS = [FRYER, OIL, MONITOR, KEYBOARD, *MARKS, *RISK_CARDS, *EXTINGUISHERS, *TYPE_CARDS]


# -- scoring ------------------------------------------------------------------------------------------------------
def standing_at(objects, name, spot, settled=True) -> bool:
    o = objects[name]
    upright = (o.high[2] - o.low[2]) > 0.12
    return bool(upright and inside(o, SPOTS[spot], MAT) and (still(objects, name) or not settled))


def outcome(objects) -> str:
    """'water_on_fire_risk' (the water extinguisher on either mark: 0), 'ready' (wet chemical by the fryer and CO2 by
    the computer: success), or 'N_right'."""
    if any(standing_at(objects, "water", s, settled=False) for s in SPOTS):
        return "water_on_fire_risk"
    n = sum(standing_at(objects, e, s) for s, e in RIGHT.items())
    return "ready" if n == 2 else f"{n}_right"


def success(objects):
    return outcome(objects) == "ready"


def distance(objects):
    return float(sum(np.hypot(*(np.asarray(objects[e].pos[:2]) - SPOTS[s])) for s, e in RIGHT.items()))


def score(objects):
    """0 if the water extinguisher stands on either mark; otherwise 0.5 for each mark with its right extinguisher
    standing on it (wet chemical by the fryer, CO2 by the computer)."""
    o = outcome(objects)
    return 0.0 if o == "water_on_fire_risk" else (1.0 if o == "ready" else int(o[0]) / 2)


# -- scripted demo ------------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.24  # high enough to carry one 14 cm extinguisher over the others


def _move(name, spot):
    start = TYPES[name][2]
    g = 0.08
    return pick_place_waypoints((*start, TOP + g), (*SPOTS[spot], TOP + g + 0.008), travel_z=TRAVEL_Z)


def demo():
    """The left arm carries the wet-chemical extinguisher to the fryer's mark; then the right arm carries the CO2 one to
    the computer's mark (it waits, parked out of the way, while the left arm crosses the middle)."""
    left = _move("wet_chemical", "fryer")
    wait = Waypoint(sum(w.duration for w in left), (0.10, -0.25, TOP + 0.20), FINGER_OPEN, "wait for the left arm")
    return {"left": left, "right": [wait] + _move("co2", "pc")}
