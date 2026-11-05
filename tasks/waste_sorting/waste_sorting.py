"""On the station table: four open waste bins, coloured like the lids of Karlsruhe's four household bins, and six
items between the arms. The robot sorts each item into the bin where it belongs in Karlsruhe and leaves anything
that has no fitting bin on the table. Nothing in the scene or the instruction says which bin takes what.

Bins (fixed, inside 16 x 16 x 9 cm, 6 mm walls): blue (paper) at (20, 36) and green (organic) at (40, 33) on the
left, red (Wertstoffe: plastic, metal, composite packaging) at (20, -36) and black (residual waste) at (40, -33) on
the right; centres in cm, x forward, y left.
Items (Arena object-library models): tin can, banana and smartphone on the left half of the table, drink carton,
popcorn box and ceramic mug on the right half.
In Karlsruhe: tin can and drink carton (composite packaging) -> red; popcorn box -> blue; banana -> green;
mug (ceramics) -> black; the smartphone (a small electrical device) goes in none of them.

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/waste_sorting/waste_sorting.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \\
    -E cam_height=176 --instruction "This kitchen is in Karlsruhe, Germany. The bins are colour-coded according \\
    to the local waste sorting rules. Sort the waste on the table into the bins where it belongs; leave anything \\
    that has no fitting bin on the table." --scorer task_score \\
    --max-steps 2400 --store-frames --log-dir runs/waste-sorting-demo
"""

import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- bins: five fixed boxes each, open at the top ------------------------------------------------------------------
INNER, DEPTH, WALL = 0.16, 0.09, 0.006
BINS = {  # name: (centre x, y), colour (sRGB) of the Karlsruhe bin lid
    "paper": ((0.20, 0.36), (0.12, 0.30, 0.75)),  # blue: paper and cardboard
    "organic": ((0.40, 0.33), (0.15, 0.50, 0.20)),  # green: food and kitchen waste
    "recycling": ((0.20, -0.36), (0.80, 0.12, 0.10)),  # red: Wertstoffe (plastic, metal, composite packaging)
    "residual": ((0.40, -0.33), (0.08, 0.08, 0.08)),  # black: residual waste
}


def _bin(name, xy, color):
    x, y = xy
    h, half, out = DEPTH + WALL, INNER / 2 + WALL / 2, INNER + 2 * WALL
    z = TOP + h / 2
    return [
        Box(f"{name}_bin_floor", pos=(x, y, TOP + WALL / 2), size=(out, out, WALL), color=color),
        Box(f"{name}_bin_far", pos=(x + half, y, z), size=(WALL, out, h), color=color),
        Box(f"{name}_bin_near", pos=(x - half, y, z), size=(WALL, out, h), color=color),
        Box(f"{name}_bin_left", pos=(x, y + half, z), size=(INNER, WALL, h), color=color),
        Box(f"{name}_bin_right", pos=(x, y - half, z), size=(INNER, WALL, h), color=color),
    ]


BIN_PARTS = [part for name, (xy, color) in BINS.items() for part in _bin(name, xy, color)]

# -- items: (library asset, start (x, y), widest horizontal extent (m), mass (kg), yaw) --------------------------------
ITEMS = {
    "tin_can": ("tomato_soup_can_ycb_robolab", (0.15, 0.10), 0.066, 0.05, 0.0),
    "banana": ("banana_ycb_robolab", (0.25, 0.16), 0.12, 0.10, np.pi / 2),
    "smartphone": ("smartphone_hot3d_robolab", (0.34, 0.06), 0.10, 0.15, 0.0),
    "drink_carton": ("orange_juice_carton_hope_robolab", (0.17, -0.14), 0.045, 0.03, 0.0),
    "popcorn_box": ("popcorn_box_hope_robolab", (0.30, -0.07), 0.08, 0.05, 0.0),
    "mug": ("ceramic_mug_hot3d_robolab", (0.29, -0.18), 0.075, 0.15, 0.0),
}
GOALS = {  # the bin each item belongs in, in Karlsruhe
    "tin_can": "recycling", "drink_carton": "recycling",
    "popcorn_box": "paper", "banana": "organic", "mug": "residual",
}  # fmt: skip
KEEP_OUT = ("smartphone",)  # electrical devices go to a collection point, not in a household bin
CONVEX = ("drink_carton",)  # collided as one convex hull: the model's own collision mesh catches on the fingers
OBJECTS = [
    *BIN_PARTS,
    *(Asset(n, asset=a, xy=xy, extent=e, mass=m, yaw=w, convex=n in CONVEX) for n, (a, xy, e, m, w) in ITEMS.items()),
]


# -- scoring -------------------------------------------------------------------------------------------------------
def in_bin(s, name) -> bool:
    """The item's bounding-box centre is inside the bin's opening and below its rim."""
    (x, y), _ = BINS[name]
    cx, cy, cz = s.pos
    return bool(abs(cx - x) < INNER / 2 and abs(cy - y) < INNER / 2 and TOP < cz < TOP + WALL + DEPTH)


def where(s):
    """The bin an item is in, or None."""
    return next((b for b in BINS if in_bin(s, b)), None)


def counts(objects):
    """(items in their bin, items in a wrong bin, kept-out items in any bin)."""
    right = sum(where(objects[n]) == b for n, b in GOALS.items())
    wrong = sum(where(objects[n]) not in (None, b) for n, b in GOALS.items())
    stray = sum(where(objects[n]) is not None for n in KEEP_OUT)
    return right, wrong, stray


_calls = [0]


def _debug(objects):
    """With WASTE_DEBUG=N set, print every N-th step each item's centre, bounds and bin."""
    every = int(os.environ.get("WASTE_DEBUG", "0") or 0)
    _calls[0] += 1
    if every and _calls[0] % every == 0:
        for n in ITEMS:
            s = objects[n]
            print(f"[waste {_calls[0]}] {n}: pos {np.round(s.pos, 3)} low {np.round(s.low, 3)} "
                  f"high {np.round(s.high, 3)} bin {where(s)}", flush=True)  # fmt: skip


def success(objects):
    right, wrong, stray = counts(objects)
    return right == len(GOALS) and wrong == 0 and stray == 0 and all(objects[n].resting() for n in ITEMS)


def score(objects):
    """1/5 per item in its bin, minus 1/5 per item in a wrong bin and for the smartphone in any bin (at least 0)."""
    _debug(objects)
    right, wrong, stray = counts(objects)
    return max(0, right - wrong - stray) / len(GOALS)


def distance(objects):
    """Summed horizontal distance (m) of the items from the centres of their bins."""
    return float(sum(np.linalg.norm(objects[n].pos[:2] - np.array(BINS[b][0])) for n, b in GOALS.items()))


# -- demo -------------------------------------------------------------------------------------------------------------
# Per item: the TCP height above the table to grip it at, and the grip yaw (0 closes the fingers along y). Heights
# come from the items' bounding boxes as spawned (tin can 10 cm, banana 2.5, carton 12.4, popcorn box 5.5, mug 5 cm
# tall); the finger pads (13 mm below the TCP and up) hold an item at or above its middle.
GRIP = {
    "tin_can": (0.060, 0.0), "banana": (0.022, 0.0), "drink_carton": (0.065, 0.0),
    "popcorn_box": (0.035, np.pi / 2), "mug": (0.035, np.pi / 2),
}  # fmt: skip
RIM = WALL + DEPTH
CLEAR = 0.025  # a carried item's bottom passes this far above a bin's rim
HANDOVER = {"tin_can": (0.17, 0.0), "popcorn_box": (0.27, 0.0)}
SPOT = {"drink_carton": (-0.04, 0.0), "tin_can": (0.04, 0.0)}  # where in its bin an item goes, from the bin's centre
SPEED = 1.3


def _wp(t, pos, finger, label, yaw):
    return Waypoint(t / SPEED, tuple(float(v) for v in pos), finger, label, yaw)


def move(name, start, goal, drop, yaw=None):
    """Pick ``name`` at ``start`` (x, y) and set it down at ``goal``: on a bin's floor (``drop``) or on the table."""
    grip, grip_yaw = GRIP[name]
    yaw = grip_yaw if yaw is None else yaw
    travel = TOP + RIM + CLEAR + grip
    (px, py), (qx, qy) = start, goal
    qz = TOP + grip + (WALL + 0.015 if drop else 0.004)  # let go 1.5 cm above a bin's floor, 4 mm above the table
    return [
        _wp(1.5, (px, py, travel), FINGER_OPEN, f"{name}: to it", yaw),
        _wp(1.5, (px, py, TOP + grip), FINGER_OPEN, "descend", yaw),
        _wp(0.8, (px, py, TOP + grip), FINGER_CLOSED, "close", yaw),
        _wp(1.2, (px, py, travel), FINGER_CLOSED, "lift", yaw),
        _wp(1.8, (qx, qy, travel), FINGER_CLOSED, "carry", yaw),
        _wp(1.0, (qx, qy, qz), FINGER_CLOSED, "lower", yaw),
        _wp(0.6, (qx, qy, qz), FINGER_OPEN, "let go", yaw),
        _wp(0.8, (qx, qy, travel), FINGER_OPEN, "up", yaw),
    ]


def to_bin(name, start, yaw=None):
    goal = np.array(BINS[GOALS[name]][0]) + np.array(SPOT.get(name, (0.0, 0.0)))
    return move(name, start, goal, drop=True, yaw=yaw)


def rest(arm, seconds):
    return [Waypoint(seconds, (0.15, 0.27 if arm == "left" else -0.27, TOP + 0.16), FINGER_OPEN, "rest")]


def _duration(waypoints):
    return sum(w.duration for w in waypoints)


def demo():
    """One arm at a time: the left arm bins the banana and sets the tin can down on the centre line; the right arm bins
    the carton and the mug, sets the popcorn box down on the centre line and bins the can; the left arm bins the
    popcorn box."""
    start = {n: spec[1] for n, spec in ITEMS.items()}
    left_a = (
        to_bin("banana", start["banana"])
        + move("tin_can", start["tin_can"], HANDOVER["tin_can"], drop=False)
        + rest("left", 1.0)
    )
    right_b = (
        to_bin("drink_carton", start["drink_carton"])
        + to_bin("mug", start["mug"])
        + move("popcorn_box", start["popcorn_box"], HANDOVER["popcorn_box"], drop=False)
        + to_bin("tin_can", HANDOVER["tin_can"])
        + rest("right", 1.0)
    )
    left_c = to_bin("popcorn_box", HANDOVER["popcorn_box"], yaw=-np.pi / 2) + rest("left", 3.0)
    return {
        "left": left_a + rest("left", _duration(right_b)) + left_c,
        "right": rest("right", _duration(left_a)) + right_b + rest("right", _duration(left_c)),
    }
