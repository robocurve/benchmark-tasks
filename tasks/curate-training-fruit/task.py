"""On the station table: the station's bowl in the middle and its plate in front of the right arm, a grey "TRAINING
SET" mat at the far left showing an apple and an orange (fixed, for reference), and three loose fruits in front of
the arms: an apple on the left, a lemon in the middle and an orange on the right. The robot is curating a robot
dataset: fruit of a kind already in the training set goes in the bowl, a new kind goes on the plate. The nearest
target is the wrong one for the lemon and the orange. The bowl, plate and mat are fixed; the three fruits are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "put fruit of a kind shown on the training set mat in the bowl, and fruit of a new kind on the plate" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DREAM_YAW = np.pi / 2  # the station's own objects are turned this way

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
# -- the station's bowl and plate (fixed), where the station puts them ----------------------------------------------
BOWL_XY, PLATE_XY = (0.28, 0.0), (0.285, -0.22)
BOWL_RIM, PLATE_TOP = 0.0575, 0.0114  # heights above the table
BOWL = Asset("bowl", "bowl_ycb_robolab", xy=BOWL_XY, extent=0.166, color=(0.861, 0.841, 0.798), yaw=DREAM_YAW)
PLATE = Asset("plate", "plate_small_vomp_robolab", xy=PLATE_XY, extent=0.146, color=(0.880, 0.876, 0.841), yaw=DREAM_YAW)

# -- the training-set mat (fixed): a grey board with one fixed example of each kind already in the dataset ---------
MAT_XY, MAT_SIZE = (0.43, 0.20), (0.12, 0.22)
MAT = Box("training_mat", pos=(*MAT_XY, TOP + 0.0015), size=(*MAT_SIZE, 0.003), color=(0.45, 0.45, 0.48))
EXAMPLES = [Asset("example_apple", "apple_01_objaverse_robolab", xy=(MAT_XY[0], MAT_XY[1] + 0.05), extent=0.066,
                  convex=True),
            Asset("example_orange", "orange_02_fruits_veggies_robolab", xy=(MAT_XY[0], MAT_XY[1] - 0.05), extent=0.064)]
LABEL, _ = card("label", ("TRAINING SET",), (MAT_XY[0] - MAT_SIZE[0] / 2 - 0.035, MAT_XY[1]), 0.0035)

# -- the fruit to curate (loose): the station's apple and orange, and a lemon ----------------------------------------
START = {"apple": (0.135, 0.15), "lemon": (0.12, 0.0), "orange": (0.14, -0.13)}
FRUIT = [Asset("apple", "apple_01_objaverse_robolab", xy=START["apple"], extent=0.066, mass=0.12, convex=True),
         Asset("lemon", "lemon_01_fruits_veggies_robolab", xy=START["lemon"], extent=0.065, mass=0.10, convex=True),
         Asset("orange", "orange_02_fruits_veggies_robolab", xy=START["orange"], extent=0.064, mass=0.11)]
TARGET = {"apple": "bowl", "orange": "bowl", "lemon": "plate"}

OBJECTS = [BOWL, PLATE, MAT, *EXAMPLES, *LABEL, *FRUIT]

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
IN_BOWL_R, ON_PLATE_R = 0.06, 0.06  # centre within 6 cm of the target's centre


def where(objects, name):
    """'bowl', 'plate' or None for a fruit at rest."""
    f = objects[name]
    if not still(objects, name):
        return None
    if np.hypot(*(np.asarray(f.pos[:2]) - BOWL_XY)) <= IN_BOWL_R and f.low[2] <= TOP + BOWL_RIM:
        return "bowl"
    if np.hypot(*(np.asarray(f.pos[:2]) - PLATE_XY)) <= ON_PLATE_R and f.low[2] <= TOP + PLATE_TOP + 0.03:
        return "plate"
    return None


def tally(objects):
    w = {n: where(objects, n) for n in TARGET}
    right = sum(w[n] == TARGET[n] for n in TARGET)
    wrong = sum(w[n] is not None and w[n] != TARGET[n] for n in TARGET)
    return right, wrong


def outcome(objects) -> str:
    """'curated' (apple and orange in the bowl, lemon on the plate: success), or 'R_right_W_wrong' counts."""
    right, wrong = tally(objects)
    return "curated" if right == 3 else f"{right}_right_{wrong}_wrong"


def success(objects):
    return outcome(objects) == "curated"


def distance(objects):
    """Metres (horizontal), summed over the fruit, from each one to its right target."""
    xy = {"bowl": BOWL_XY, "plate": PLATE_XY}
    return float(sum(np.hypot(*(np.asarray(objects[n].pos[:2]) - xy[t])) for n, t in TARGET.items()))


def score(objects):
    """One third for each fruit on its right target, minus one sixth for each on the wrong one, never below 0
    (success scores 1.0)."""
    right, wrong = tally(objects)
    return max(0.0, right / 3 - wrong / 6)


# -- scripted demo ------------------------------------------------------------------------------------------------
GRIP = {"apple": 0.031, "lemon": 0.03, "orange": 0.030}  # the station's grip heights (the lemon measured in Isaac)
RELEASE = 0.015  # let go with the fruit's bottom this far above the bowl's rim or the plate
TRAVEL_Z = TOP + 0.16


def _move(name, xy):
    top = BOWL_RIM if TARGET[name] == "bowl" else PLATE_TOP
    return pick_place_waypoints((*START[name], TOP + GRIP[name]), (*xy, TOP + top + GRIP[name] + RELEASE),
                                travel_z=TRAVEL_Z)


def demo():
    """The left arm puts the apple in the bowl. The right arm puts the lemon on the plate, then the orange in the bowl
    once the left arm is clear of it."""
    left = _move("apple", (BOWL_XY[0], BOWL_XY[1] + 0.03))
    right = _move("lemon", PLATE_XY) + _move("orange", (BOWL_XY[0], BOWL_XY[1] - 0.03))
    return {"left": left, "right": right}
