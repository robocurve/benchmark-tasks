"""On the station table: a grey bin at the right of the far side and a terracotta plant pot at the left with a card
beyond it reading "GARDEN". In front of the right arm are two bread crusts; in the middle, among them, sits a toy
ladybird (red, 5 cm across). The instruction is to put the crumbs in the bin and the toy ladybird in the garden
pot. Binning the ladybird with the crumbs is the failure. The bin, pot and card are fixed; the crusts and the
ladybird are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "put the crumbs in the bin and move the toy ladybird to the garden pot" \\
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
INNER = (0.14, 0.16)
BIN_XY, POT_XY = (0.30, -0.14), (0.30, 0.14)
FIXED = [*walls("bin", BIN_XY, INNER, 0.03, (0.4, 0.4, 0.42)), *walls("garden_pot", POT_XY, INNER, 0.04, (0.72, 0.38, 0.22)),
         Box("soil", pos=(*POT_XY, TOP + 0.002), size=(*INNER, 0.004), color=(0.3, 0.2, 0.12)),
         *card("garden_card", ("GARDEN",), (0.43, 0.14), 0.0035)[0]]

# -- loose: two crusts (6 x 5 x 3 cm) and the ladybird (a red disc 5 cm across, 3 cm tall) ---------------------------
CRUST = (0.06, 0.05, 0.03)
LB_R, LB_H = 0.025, 0.03
START = {"crust_1": (0.14, -0.12), "crust_2": (0.14, -0.24), "ladybird": (0.14, 0.0)}
LOOSE = [*[Box(c, pos=(*START[c], TOP + CRUST[2] / 2), size=CRUST, mass=0.03, color=(0.7, 0.5, 0.25))
           for c in ("crust_1", "crust_2")],
         Cylinder("ladybird", pos=(*START["ladybird"], TOP + LB_H / 2), radius=LB_R, height=LB_H, mass=0.03,
                  color=(0.85, 0.08, 0.06))]
CRUSTS = ("crust_1", "crust_2")

OBJECTS = [*FIXED, *LOOSE]


# -- scoring ------------------------------------------------------------------------------------------------------
def at(objects, name, xy, settled=True) -> bool:
    o = objects[name]
    return bool(inside(o, xy, INNER) and o.low[2] <= TOP + 0.06 and (still(objects, name) or not settled))


def outcome(objects) -> str:
    """'ladybird_binned' (in the bin or off the table: 0, whatever else), 'cleared' (both crusts in the bin and the
    ladybird in the garden pot: success), or 'C_crusts_ladybird_saved' / 'C_crusts'."""
    lb = objects["ladybird"]
    if at(objects, "ladybird", BIN_XY, settled=False) or lb.pos[2] < TOP - 0.02:
        return "ladybird_binned"
    c = sum(at(objects, n, BIN_XY) for n in CRUSTS)
    saved = at(objects, "ladybird", POT_XY)
    return "cleared" if c == 2 and saved else f"{c}_crusts" + ("_ladybird_saved" if saved else "")


def success(objects):
    return outcome(objects) == "cleared"


def distance(objects):
    d = sum(np.hypot(*(np.asarray(objects[n].pos[:2]) - BIN_XY)) for n in CRUSTS)
    return float(d + np.hypot(*(np.asarray(objects["ladybird"].pos[:2]) - POT_XY)))


def score(objects):
    """0 if the ladybird was binned; otherwise one third for each crust in the bin and one third for the ladybird in
    the garden pot."""
    o = outcome(objects)
    if o == "ladybird_binned":
        return 0.0
    if o == "cleared":
        return 1.0
    return (int(o[0]) + o.endswith("saved")) / 3


# -- scripted demo ------------------------------------------------------------------------------------------------
def demo():
    """The left arm carries the ladybird to the garden pot; then the right arm puts both crusts in the bin, 8 cm apart
    (it waits first: the ladybird sits only 12 cm from the nearer crust, too close for both arms at once)."""
    g = TOP + CRUST[2] / 2
    left = pick_place_waypoints((*START["ladybird"], TOP + LB_H / 2), (*POT_XY, TOP + 0.004 + LB_H / 2 + 0.01),
                                travel_z=TRAVEL_Z)
    right = (pick_place_waypoints((*START["crust_1"], g), (BIN_XY[0], BIN_XY[1] + 0.04, g + 0.01), travel_z=TRAVEL_Z)
             + pick_place_waypoints((*START["crust_2"], g), (BIN_XY[0], BIN_XY[1] - 0.04, g + 0.01), travel_z=TRAVEL_Z))
    wait = Waypoint(sum(w.duration for w in left), (0.15, -0.17, TRAVEL_Z), FINGER_OPEN, "wait for the left arm")
    return {"left": left, "right": [wait] + right}
