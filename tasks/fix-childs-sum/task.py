"""On the station table: a printed card across the far side reading "2 + 3 =", an answer mat in front of the right
arm on which a child has laid six wooden blocks as the answer, and an empty spare tray in the middle. The sum is
wrong: two plus three is five. The robot must fix the answer, which means taking one block off the mat and putting it
in the spare tray. The card, the mat and the tray are fixed; the six blocks are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "the child's sum is wrong: fix the blocks on the answer mat, putting any spare block in the spare tray" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

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


# Positions on the table from the midpoint between the arm bases: x forward, y to the left (m).
SUM_CARD, _ = card("sum_card", ("2 + 3 =",), (0.43, -0.05), 0.005)  # letters 2.5 x 3.5 cm

# -- the answer mat (fixed, thin) and the spare tray (fixed walls) ------------------------------------------------
MAT_XY, MAT_SIZE = (0.21, -0.20), (0.26, 0.24)
MAT = Box("answer_mat", pos=(*MAT_XY, TOP + 0.0004), size=(*MAT_SIZE, 0.0008), color=(0.95, 0.85, 0.3))
TRAY_XY, TRAY_INNER = (0.15, 0.06), (0.10, 0.10)
TRAY = walls("spare_tray", TRAY_XY, TRAY_INNER, 0.03, (0.55, 0.55, 0.58))

# -- the six blocks (loose): 5 cm wooden cubes in two rows of three, 12 cm apart left to right ------------------------
BLOCK = 0.05
SPOTS = [(x, y) for y in (-0.14, -0.26) for x in (0.12, 0.21, 0.30)]
BLOCKS = [Box(f"block_{i + 1}", pos=(x, y, TOP + BLOCK / 2), size=(BLOCK,) * 3, mass=0.05, color=(0.85, 0.35, 0.2))
          for i, (x, y) in enumerate(SPOTS)]
NAMES = [b.name for b in BLOCKS]
ANSWER = 5

OBJECTS = [*SUM_CARD, MAT, *TRAY, *BLOCKS]

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
def on_mat(objects, name) -> bool:
    b = objects[name]
    return bool(inside(b, MAT_XY, MAT_SIZE) and b.low[2] <= TOP + 0.01 and still(objects, name))


def in_tray(objects, name) -> bool:
    b = objects[name]
    return bool(inside(b, TRAY_XY, TRAY_INNER) and b.low[2] <= TOP + 0.04 and still(objects, name))


def outcome(objects) -> str:
    """'fixed' (five blocks on the mat, the sixth in the spare tray: success), 'fixed_untidy' (five on the mat, the
    sixth elsewhere), or 'N_on_mat' (any other count, including the untouched six)."""
    n = sum(on_mat(objects, b) for b in NAMES)
    if n == ANSWER:
        return "fixed" if sum(in_tray(objects, b) for b in NAMES) == len(NAMES) - ANSWER else "fixed_untidy"
    return f"{n}_on_mat"


def success(objects):
    return outcome(objects) == "fixed"


def distance(objects):
    """How many blocks the answer mat is off by."""
    return float(abs(sum(on_mat(objects, b) for b in NAMES) - ANSWER))


def score(objects):
    """1.0 fixed with the spare in the tray; 0.5 five on the mat with the spare elsewhere; 0 for any other count."""
    return {"fixed": 1.0, "fixed_untidy": 0.5}.get(outcome(objects), 0.0)


# -- scripted demo ------------------------------------------------------------------------------------------------
def demo():
    """The right arm takes the nearest block on the mat's inner row and sets it in the spare tray."""
    x, y = SPOTS[0]
    return {"right": pick_place_waypoints((x, y, TOP + BLOCK / 2), (*TRAY_XY, TOP + BLOCK / 2 + 0.01),
                                          travel_z=TOP + 0.13)}
