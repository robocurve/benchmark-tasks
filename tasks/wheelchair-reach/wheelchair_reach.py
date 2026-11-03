"""On the station table, in front of the right arm: a 1:10 scale cream wall, 14 cm tall (1.4 m at scale), fixed, its
face toward the arms. Three fixed brown ledges, 60 mm deep, stick out from it side by side, none above another: the
high ledge (top 135 mm above the table) nearest the right arm's base, the middle ledge (top 80 mm, 140 mm wide), and
the low ledge (top 12 mm) nearest the midline. A green first-aid kit (24 x 30 x 22 mm) sits on the high ledge and a
blue box of bandages (24 x 26 x 22 mm) on the low ledge. The table stands for the floor.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SCALE = 10
INCH = 0.0254 / SCALE  # one inch at scale (m)

WALL_X = 0.335  # the wall's face
# WHY: only a little above the high ledge: a taller wall stops the gripper on its top before it reaches the kit.
WALL_H = 0.14
WALL = Box("wall", pos=(WALL_X + 0.01, -0.16, TOP + WALL_H / 2), size=(0.02, 0.34, WALL_H), color=(0.85, 0.83, 0.78))
LEDGE_DEPTH, LEDGE_T = 0.060, 0.006
# WHY: the low ledge puts the bandages wholly below 15 in at scale (12 to 34 mm), the high one the kit wholly above 48.
LEDGE_TOPS = {"low_ledge": 0.012, "middle_ledge": 0.080, "high_ledge": 0.135}  # above the table (m)
# WHY: the high ledge nearest the right arm's base (y -0.305): 0.19 m up, the arm reaches about 0.3 m out, not 0.4.
LEDGE_Y = {"high_ledge": (-0.29, 0.06), "middle_ledge": (-0.17, 0.14), "low_ledge": (-0.03, 0.06)}  # centre, width
LEDGES = [Box(n, pos=(WALL_X - LEDGE_DEPTH / 2, LEDGE_Y[n][0], TOP + h - LEDGE_T / 2),
              size=(LEDGE_DEPTH, LEDGE_Y[n][1], LEDGE_T), color=(0.45, 0.30, 0.18)) for n, h in LEDGE_TOPS.items()]

ITEM_X = WALL_X - 0.035  # the items' centres, out from the wall so the fingers stay clear of it
KIT = (0.024, 0.030, 0.022)  # deep, wide, tall
BANDAGES = (0.024, 0.026, 0.022)
ITEMS = [
    Box("first_aid_kit", pos=(ITEM_X, LEDGE_Y["high_ledge"][0], TOP + LEDGE_TOPS["high_ledge"] + KIT[2] / 2 + 0.001),
        size=KIT, mass=0.05, color=(0.10, 0.55, 0.25)),
    Box("bandages", pos=(ITEM_X, LEDGE_Y["low_ledge"][0], TOP + LEDGE_TOPS["low_ledge"] + BANDAGES[2] / 2 + 0.001),
        size=BANDAGES, mass=0.05, color=(0.20, 0.40, 0.80)),
]
OBJECTS = [WALL, *LEDGES, *ITEMS]

# ADA 2010 Standards, 308.2.1 (forward reach) and 308.3.1 (side reach), both unobstructed: 48 in maximum, 15 in
# minimum above the floor; 811.3, storage complies with one of them.
REACH_IN = (15.0, 48.0)
FLAT_DEG = 10.0
ON_MM = 2.0  # an item's bottom within this of what it stands on


def _span_in(item) -> tuple[float, float]:
    """The item's bottom and top above the table, in inches at scale."""
    return (float(item.low[2]) - TOP) / INCH, (float(item.high[2]) - TOP) / INCH


def _in_range(item) -> bool:
    """The whole item between 15 and 48 in, so whichever part is reached for, it is in range."""
    bottom, top = _span_in(item)
    return REACH_IN[0] <= bottom and top <= REACH_IN[1]


def _flat(item) -> bool:
    """On any of its faces, within FLAT_DEG."""
    x, y, z, w = (float(v) for v in item.quat_xyzw)
    up = (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))  # world z of each of the item's axes
    return max(abs(u) for u in up) >= math.cos(math.radians(FLAT_DEG))


def _supported(item, other) -> bool:
    """Standing on a ledge, or on the other item: not held up in the air."""
    x, y, bottom = (float(v) for v in (item.pos[0], item.pos[1], item.low[2]))
    for name, h in LEDGE_TOPS.items():
        cy, width = LEDGE_Y[name]
        if (abs(bottom - (TOP + h)) <= ON_MM / 1000 and WALL_X - LEDGE_DEPTH <= x <= WALL_X
                and abs(y - cy) <= width / 2):
            return True
    return (abs(bottom - float(other.high[2])) <= ON_MM / 1000
            and float(other.low[0]) <= x <= float(other.high[0]) and float(other.low[1]) <= y <= float(other.high[1]))


def _reachable(objects, name) -> bool:
    item, other = objects[name], objects[next(i.name for i in ITEMS if i.name != name)]
    return _in_range(item) and _flat(item) and _supported(item, other)


def success(objects):
    """Both items within reach, standing on something and still; holds 1 s."""
    return all(_reachable(objects, i.name) and objects[i.name].resting() for i in ITEMS)


def distance(objects):
    """How far, in inches at scale, the items reach outside the range, summed."""
    return sum(max(REACH_IN[0] - b, t - REACH_IN[1], 0.0) for b, t in (_span_in(objects[i.name]) for i in ITEMS))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 for each item within reach."""
    return sum(0.5 for i in ITEMS if _reachable(objects, i.name))


# WHY: carry each item just above what it passes; the arm reaches less far out the higher it goes.
TRAVEL_Z = {"first_aid_kit": TOP + 0.185, "bandages": TOP + 0.13}
PAD_CLEAR = 0.017  # TCP above a surface to grip a 22 mm item: pad tips 13 mm below the TCP, 4 mm above the surface
# WHY: 90 mm apart, so the open fingers setting one down stay clear of the other.
MIDDLE = {"first_aid_kit": LEDGE_Y["middle_ledge"][0] + 0.045, "bandages": LEDGE_Y["middle_ledge"][0] - 0.045}
START = {"first_aid_kit": "high_ledge", "bandages": "low_ledge"}


def _move(name):
    start = LEDGE_Y[START[name]][0], TOP + LEDGE_TOPS[START[name]] + PAD_CLEAR
    place = MIDDLE[name], TOP + LEDGE_TOPS["middle_ledge"] + PAD_CLEAR + 0.002
    return pick_place_waypoints((ITEM_X, *start), (ITEM_X, *place), travel_z=TRAVEL_Z[name])


def demo():
    """The right arm moves the kit down from the high ledge and the bandages up from the low ledge, both onto the
    middle ledge."""
    right = [*_move("first_aid_kit"), *_move("bandages")]
    right.append(Waypoint(1.0, (0.20, -0.20, TRAVEL_Z["bandages"]), FINGER_OPEN, "clear"))
    return {"right": right}
