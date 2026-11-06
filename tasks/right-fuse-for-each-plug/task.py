"""On the station table: two open plugs at the far side, the lamp's on the left and the kettle's on the right, each
with a card beyond it naming the appliance and its power ("LAMP 100 W", "KETTLE 3000 W") and an empty fuse slot. In
front of the arms lie three fuses in carriers, each with a card naming its rating: 13 A (far left, nearest the lamp's
plug), 3 A (just right of centre) and 13 A (right). A UK plug takes a 3 A fuse for appliances under about 700 W and
13 A above: a 13 A fuse in the lamp's plug would let its thin cord overheat before the fuse blew, and a 3 A fuse in the
kettle's plug blows at once. The plugs and cards are fixed; the fuses are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "fit the right fuse in the lamp's plug and in the kettle's plug" --scorer task_score ...
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
PLUGS = {"lamp": (0.28, 0.14), "kettle": (0.28, -0.14)}
BASE_T, RAIL_H, SLOT_W = 0.005, 0.015, 0.040  # base plate, rail height, gap between the rails (8 mm clearance each side)
PLUG_OBJS = []
for k, (x, y) in PLUGS.items():
    PLUG_OBJS += [Box(f"{k}_plug", pos=(x, y, TOP + BASE_T / 2), size=(0.09, 0.07, BASE_T), color=(0.96, 0.96, 0.96)),
                  *[Box(f"{k}_rail_{'left' if s > 0 else 'right'}", pos=(x, y + s * (SLOT_W / 2 + 0.005), TOP + BASE_T + RAIL_H / 2),
                        size=(0.05, 0.01, RAIL_H), color=(0.75, 0.62, 0.3)) for s in (1, -1)]]
APP_CARDS = [*card("lamp_card", ("LAMP", "100 W"), (0.37, 0.14), 0.0025)[0],
             *card("kettle_card", ("KETTLE", "3000 W"), (0.37, -0.14), 0.0025)[0]]

# -- the fuses (loose), each in a carrier 6 cm long, 2.4 cm across and 4.5 cm tall ------------------------------------
FUSE = (0.06, 0.024, 0.045)
FUSES = {"fuse_13a_left": (13, (0.13, 0.18)), "fuse_3a": (3, (0.13, -0.04)), "fuse_13a_right": (13, (0.13, -0.18))}
COLOR = {3: (0.85, 0.15, 0.12), 13: (0.45, 0.28, 0.15)}
FUSE_OBJS = [Box(n, pos=(*xy, TOP + FUSE[2] / 2), size=FUSE, mass=0.03, color=COLOR[a]) for n, (a, xy) in FUSES.items()]
FUSE_CARDS = [o for n, (a, xy) in FUSES.items() for o in card(f"{n}_card", (f"{a} A",), (0.07, xy[1]), 0.0025)[0]]

OBJECTS = [*PLUG_OBJS, *APP_CARDS, *FUSE_OBJS, *FUSE_CARDS]


# -- scoring ------------------------------------------------------------------------------------------------------
def fitted(objects, name, plug, settled=True) -> bool:
    o, (x, y) = objects[name], PLUGS[plug]
    flat = (o.high[2] - o.low[2]) < FUSE[2] + 0.003  # not propped up on a rail
    seated = flat and abs(o.pos[0] - x) < 0.015 and abs(o.pos[1] - y) < 0.009 and o.low[2] <= TOP + BASE_T + 0.006
    return bool(seated and (still(objects, name) or not settled))


def in_plug(objects, plug, settled=True):
    return [n for n in FUSES if fitted(objects, n, plug, settled)]


def outcome(objects) -> str:
    """'lamp_overfused' (a 13 A fuse in the lamp's plug: 0), 'fused' (3 A in the lamp, 13 A in the kettle: success), or
    'N_right'."""
    if any(FUSES[n][0] == 13 for n in in_plug(objects, "lamp", settled=False)):
        return "lamp_overfused"
    n = (any(FUSES[f][0] == 3 for f in in_plug(objects, "lamp")) + any(FUSES[f][0] == 13 for f in in_plug(objects, "kettle")))
    return "fused" if n == 2 else f"{n}_right"


def success(objects):
    return outcome(objects) == "fused"


def distance(objects):
    return float(np.hypot(*(np.asarray(objects["fuse_3a"].pos[:2]) - PLUGS["lamp"]))
                 + np.hypot(*(np.asarray(objects["fuse_13a_right"].pos[:2]) - PLUGS["kettle"])))


def score(objects):
    """0 if a 13 A fuse is in the lamp's plug; otherwise 0.5 for a 3 A fuse in the lamp's plug and 0.5 for a 13 A fuse
    in the kettle's."""
    o = outcome(objects)
    return 0.0 if o == "lamp_overfused" else (1.0 if o == "fused" else int(o[0]) / 2)


# -- scripted demo ------------------------------------------------------------------------------------------------
def _move(name, plug):
    g = 0.033  # grip 3.3 cm up, above the 1.5 cm rails on their 0.5 cm base
    return pick_place_waypoints((*FUSES[name][1], TOP + g), (*PLUGS[plug], TOP + BASE_T + g + 0.003),
                                travel_z=TOP + 0.15, speed=0.6)


def demo():
    """The left arm fits the 3 A fuse in the lamp's plug; then the right arm fits the right-hand 13 A fuse in the
    kettle's plug (it waits, parked, while the left arm crosses the middle)."""
    left = _move("fuse_3a", "lamp")
    wait = Waypoint(sum(w.duration for w in left), (0.10, -0.25, TOP + 0.20), FINGER_OPEN, "wait for the left arm")
    return {"left": left, "right": [wait] + _move("fuse_13a_right", "kettle")}
