"""On the station table: three open bins in a row across the far side (a green compost bin on the left, a grey
landfill bin in the middle and a blue recycling bin on the right) and four pieces of rubbish in front of the arms: a
broken mug and a banana on the left, a soup can and a tuna can on the right. The instruction gives the sorting
rule: cans in the blue bin, food scraps in the green bin, everything else in the grey bin. The nearest bin is the
wrong one for three of the four items. The bins are fixed; the rubbish is loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "sort the rubbish: cans in the blue bin, food scraps in the green bin, everything else in the grey bin" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Positions on the table from the midpoint between the arm bases: x forward, y to the left (m).
# -- the bins (fixed): four walls each, the table is their floor ------------------------------------------------
BIN_X, WALL_T, WALL_H = 0.32, 0.01, 0.04
# name: (centre y, inside front to back, inside left to right, colour). The blue bin is wider: it takes both cans.
BINS = {"green": (0.20, 0.14, 0.14, (0.25, 0.6, 0.3)), "grey": (0.0, 0.14, 0.14, (0.5, 0.5, 0.52)),
        "blue": (-0.21, 0.14, 0.18, (0.2, 0.4, 0.8))}


def _bin(name, y, dx, dy, color):
    ox, oy, z = dx / 2 + WALL_T / 2, dy / 2 + WALL_T / 2, TOP + WALL_H / 2
    return [
        Box(f"{name}_bin_front", pos=(BIN_X - ox, y, z), size=(WALL_T, dy + 2 * WALL_T, WALL_H), color=color),
        Box(f"{name}_bin_back", pos=(BIN_X + ox, y, z), size=(WALL_T, dy + 2 * WALL_T, WALL_H), color=color),
        Box(f"{name}_bin_left", pos=(BIN_X, y + oy, z), size=(dx, WALL_T, WALL_H), color=color),
        Box(f"{name}_bin_right", pos=(BIN_X, y - oy, z), size=(dx, WALL_T, WALL_H), color=color),
    ]


WALLS = [w for name, (y, dx, dy, color) in BINS.items() for w in _bin(name, y, dx, dy, color)]

# -- the rubbish (loose): library models --------------------------------------------------------------------------
START = {"mug": (0.14, 0.24), "banana": (0.15, 0.08), "soup_can": (0.14, -0.08), "tuna_can": (0.14, -0.24)}
RUBBISH = [
    Asset("mug", "mug_ycb_robolab", xy=START["mug"], extent=0.10, mass=0.25, convex=True),
    Asset("banana", "banana_ycb_robolab", xy=START["banana"], extent=0.14, mass=0.12, yaw=np.pi / 2),
    Asset("soup_can", "tomato_soup_can_ycb_robolab", xy=START["soup_can"], extent=0.07, mass=0.05, convex=True),
    Asset("tuna_can", "tuna_can_ycb_robolab", xy=START["tuna_can"], extent=0.085, mass=0.03, convex=True),
]
TARGET = {"mug": "grey", "banana": "green", "soup_can": "blue", "tuna_can": "blue"}

OBJECTS = [*WALLS, *RUBBISH]

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
IN_BIN_DZ = 0.04  # an item's underside within 4 cm of the table: inside the bin, not held over it


def bin_of(objects, name):
    """The bin an item rests in ('green', 'grey', 'blue'), or None."""
    item = objects[name]
    if not (item.low[2] <= TOP + IN_BIN_DZ and still(objects, name)):
        return None
    for b, (y, dx, dy, _) in BINS.items():
        if abs(item.pos[0] - BIN_X) <= dx / 2 and abs(item.pos[1] - y) <= dy / 2:
            return b
    return None


def tally(objects):
    """(items in their right bin, items in a wrong bin)."""
    bins = {n: bin_of(objects, n) for n in TARGET}
    right = sum(b == TARGET[n] for n, b in bins.items())
    wrong = sum(b is not None and b != TARGET[n] for n, b in bins.items())
    return right, wrong


def outcome(objects) -> str:
    """'sorted' (all four in their right bins: success), or 'R_right_W_wrong' counts."""
    right, wrong = tally(objects)
    return "sorted" if right == len(TARGET) else f"{right}_right_{wrong}_wrong"


def success(objects):
    """Every item rests in the bin the rule gives it."""
    return outcome(objects) == "sorted"


def distance(objects):
    """Metres (horizontal), summed over the items, from each one's centre to its right bin's centre."""
    return float(sum(np.hypot(objects[n].pos[0] - BIN_X, objects[n].pos[1] - BINS[b][0]) for n, b in TARGET.items()))


def score(objects):
    """0.25 for each item in its right bin, minus 0.125 for each item in a wrong bin (it contaminates that bin), never
    below 0 (success scores 1.0)."""
    right, wrong = tally(objects)
    return max(0.0, 0.25 * right - 0.125 * wrong)


# -- scripted demo ------------------------------------------------------------------------------------------------
GRIP_Z = {"mug": TOP + 0.035, "banana": TOP + 0.018, "soup_can": TOP + 0.05, "tuna_can": TOP + 0.016}
LIFT = {"mug": 0.01, "banana": 0.01, "soup_can": 0.01, "tuna_can": 0.01}  # let go 1 cm above where it rests: dropped further, the can lands tilted and rocks for seconds
TRAVEL_Z = TOP + 0.13  # every item clears the 4 cm walls; 16 cm is past reach over the middle bin
SPOT = {"mug": (BIN_X, BINS["grey"][0]), "banana": (BIN_X, BINS["green"][0]),
        "tuna_can": (BIN_X, BINS["blue"][0] + 0.045), "soup_can": (BIN_X, BINS["blue"][0] - 0.045)}


def _move(name):
    return pick_place_waypoints((*START[name], GRIP_Z[name]), (*SPOT[name], GRIP_Z[name] + LIFT[name]),
                                travel_z=TRAVEL_Z)


def demo():
    """The left arm puts the banana in the green bin, then the mug in the grey one; the right arm puts the tuna can
    and then the soup can in the blue bin. Each arm works its outer item while the other works its inner one, so the
    grippers are never side by side in the middle."""
    return {"left": _move("banana") + _move("mug"), "right": _move("tuna_can") + _move("soup_can")}
