"""On the station table: a round scale pan in the middle of the far side, and four brass weights standing in a row in
front of the arms, each with a printed card in front of it giving its mass: 500 g and 100 g on the left, 200 g and
200 g on the right. The robot must make the scale read 300 g: a 100 g and a 200 g weight on the pan and nothing else.
Only weights resting on the pan count, so pressing on it or holding a weight over it does not. The pan and the cards
are fixed; the weights are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "make the scale read 300 grams using the weights" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- printed cards (fixed): a white board with black block letters, read from the top camera -----------------------
FONT = {  # 5 x 7 bitmaps, top row first
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "G": ("01111", "10000", "10000", "10011", "10001", "10001", "01111"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "0": ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    "1": ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    "2": ("01110", "10001", "00001", "00110", "01000", "10000", "11111"),
    "3": ("11110", "00001", "00001", "01110", "00001", "00001", "11110"),
    "5": ("11111", "10000", "11110", "00001", "00001", "10001", "01110"),
    "+": ("00000", "00100", "00100", "11111", "00100", "00100", "00000"),
    "=": ("00000", "00000", "11111", "00000", "11111", "00000", "00000"),
    " ": ("00000",) * 7,
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


# Positions on the table from the midpoint between the arm bases: x forward, y to the left (m).
# -- the scale pan (fixed): a steel disc 18 cm across on a 2 cm base -------------------------------------------------
PAN_XY, PAN_R, PAN_H = (0.29, 0.0), 0.09, 0.02
PAN = Cylinder("scale_pan", pos=(*PAN_XY, TOP + PAN_H / 2), radius=PAN_R, height=PAN_H, color=(0.7, 0.72, 0.75))
PAN_TOP = TOP + PAN_H

# -- the weights (loose): brass cylinders 5-6.5 cm across, 12 cm apart left to right --------------------------------
BRASS = (0.78, 0.62, 0.25)
WEIGHTS = {  # name: (mass kg, radius, height, start xy)
    "w500": (0.5, 0.0325, 0.07, (0.15, 0.22)),
    "w100": (0.1, 0.025, 0.05, (0.15, 0.10)),
    "w200_a": (0.2, 0.0275, 0.06, (0.15, -0.10)),
    "w200_b": (0.2, 0.0275, 0.06, (0.15, -0.22)),
}
WEIGHT_OBJECTS = [Cylinder(n, pos=(*xy, TOP + h / 2), radius=r, height=h, mass=m, color=BRASS)
                  for n, (m, r, h, xy) in WEIGHTS.items()]
LABELS = [o for n, (m, r, h, xy) in WEIGHTS.items()
          for o in card(f"label_{n}", (f"{round(m * 1000)} G",), (xy[0] - 0.065, xy[1]), 0.003)[0]]
TARGET_G = 300

OBJECTS = [PAN, *WEIGHT_OBJECTS, *LABELS]

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


# -- scoring ------------------------------------------------------------------------------------------------------
def on_pan(objects, name) -> bool:
    """Resting on the pan (or on another weight on it): centre over the pan, underside within 8 cm of its top."""
    w = objects[name]
    over = np.hypot(*(np.asarray(w.pos[:2]) - PAN_XY)) <= PAN_R
    return bool(over and PAN_TOP - 0.005 <= w.low[2] <= PAN_TOP + 0.08 and still(objects, name))


def reading(objects) -> int:
    """What the scale reads: the grams of the weights resting on the pan."""
    return round(sum(WEIGHTS[n][0] for n in WEIGHTS if on_pan(objects, n)) * 1000)


def outcome(objects) -> str:
    """'exact' (300 g: success), 'off_by_100' (200 or 400 g), or 'reads_N' for any other reading."""
    g = reading(objects)
    return "exact" if g == TARGET_G else ("off_by_100" if abs(g - TARGET_G) == 100 else f"reads_{g}")


def success(objects):
    return outcome(objects) == "exact"


def distance(objects):
    """Kilograms between the reading and 300 g."""
    return abs(reading(objects) - TARGET_G) / 1000


def score(objects):
    """1.0 for exactly 300 g; 0.5 for 200 g or 400 g; 0 otherwise."""
    return {"exact": 1.0, "off_by_100": 0.5}.get(outcome(objects), 0.0)


# -- scripted demo ------------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.15


def _move(name, xy):
    m, r, h, start = WEIGHTS[name]
    return pick_place_waypoints((*start, TOP + h / 2), (*xy, PAN_TOP + h / 2 + 0.004), travel_z=TRAVEL_Z)


def demo():
    """The left arm puts the 100 g weight on the pan; the right arm waits above its own side, then puts the nearer
    200 g weight beside it."""
    left = _move("w100", (PAN_XY[0], PAN_XY[1] + 0.05))  # 10 cm apart: the opening fingers clear the other
    wait = Waypoint(sum(w.duration for w in left), (0.15, -0.16, TRAVEL_Z), FINGER_OPEN, "wait for the left arm")
    return {"left": left, "right": [wait] + _move("w200_a", (PAN_XY[0], PAN_XY[1] - 0.05))}
