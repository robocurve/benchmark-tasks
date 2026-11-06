"""On the station table: three identical blue-grey mats, 20 x 20 cm, side by side across the far half of the table
(left, middle, right). Each mat is a household. Behind each mat stands a white card with short black pegs fixed on
it, one peg per person living in that household: three pegs behind the left mat, one behind the middle mat and two
behind the right mat. Across the front of the table lie twelve identical orange ration packs (boxed meal rations,
70 x 55 x 35 mm), six in front of each arm. The mats, cards and pegs are fixed; the packs are loose.

The goal: every pack handed out, and every household given the same number of packs per person. Twelve packs for
six people is two each, so six packs on the left mat, two on the middle mat and four on the right mat. Stacking packs
on a mat is fine.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- packs: boxed meal rations, loose ------------------------------------------------------------------------------
PACK_X, PACK_Y, PACK_H = 0.070, 0.055, 0.035  # long side along x; the fingers close across the 55 mm side
PACK_MASS = 0.15
PACK_COLOR = (0.93, 0.50, 0.10)  # orange carton
REST_GAP = 0.001  # loose objects spawn this far above their support and settle onto it
N_PACKS = 12

# -- households: mats with a peg card behind each (fixed) ----------------------------------------------------------
MAT = 0.200  # square, 200 mm
MAT_T = 0.005
MAT_TOP = TOP + MAT_T
MAT_X0, MAT_X1 = 0.240, 0.440
MAT_COLOR = (0.36, 0.44, 0.56)  # blue-grey rubber mat, the same for all three
HOUSEHOLDS = ("left", "middle", "right")
MAT_Y = {"left": 0.300, "middle": 0.000, "right": -0.300}  # centres; 100 mm of bare table between neighbours
PEOPLE = {"left": 3, "middle": 1, "right": 2}
PER_PERSON = N_PACKS // sum(PEOPLE.values())  # 2
TARGET = {h: PER_PERSON * n for h, n in PEOPLE.items()}  # left 6, middle 2, right 4

CARD_X0, CARD_X1 = 0.445, 0.485  # a 40 mm deep white strip just behind each mat, as wide as the mat
CARD_T = 0.006
CARD_TOP = TOP + CARD_T
CARD_COLOR = (0.95, 0.95, 0.93)
PEG_R, PEG_H = 0.010, 0.040  # 20 mm across, 40 mm tall
PEG_PITCH = 0.050  # 30 mm of white card between neighbouring pegs
PEG_SHIFT = 0.040  # side cards' pegs sit toward the table centre: the ready-pose grippers hide |y| > ~0.33 in the top view
PEG_COLOR = (0.08, 0.08, 0.09)


def _box(name, x0, x1, y0, y1, z0, z1, color):
    return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), size=(x1 - x0, y1 - y0, z1 - z0), color=color)


def mat_bounds(h):
    """(x0, x1, y0, y1) of a household's mat."""
    y = MAT_Y[h]
    return MAT_X0, MAT_X1, y - MAT / 2, y + MAT / 2


def peg_ys(h):
    n = PEOPLE[h]
    c = MAT_Y[h] - PEG_SHIFT * (MAT_Y[h] > 0) + PEG_SHIFT * (MAT_Y[h] < 0)
    return tuple(c + PEG_PITCH * (k - (n - 1) / 2) for k in range(n))


FIXTURES = []
for _h in HOUSEHOLDS:
    _x0, _x1, _y0, _y1 = mat_bounds(_h)
    FIXTURES.append(_box(f"mat_{_h}", _x0, _x1, _y0, _y1, TOP, MAT_TOP, MAT_COLOR))
    FIXTURES.append(_box(f"card_{_h}", CARD_X0, CARD_X1, _y0, _y1, TOP, CARD_TOP, CARD_COLOR))
    for _k, _y in enumerate(peg_ys(_h)):
        FIXTURES.append(
            Cylinder(
                f"peg_{_h}_{_k + 1}",
                pos=((CARD_X0 + CARD_X1) / 2, _y, CARD_TOP + PEG_H / 2),
                radius=PEG_R,
                height=PEG_H,
                color=PEG_COLOR,
            )
        )

# Supply: two rows of six across the front, three columns in front of each arm. Columns are 105 mm apart, so an
# open finger (57 mm from the pack's centre line) leaves 20 mm to the next pack; rows are 85 mm apart (15 mm gap).
SUPPLY_ROW_X = (0.105, 0.190)
SUPPLY_COL_Y = (0.360, 0.255, 0.150, -0.150, -0.255, -0.360)
SUPPLY = tuple((x, y) for y in SUPPLY_COL_Y for x in SUPPLY_ROW_X)  # pack_01/02: left outer column, back to front
PACK_NAMES = tuple(f"pack_{i + 1:02d}" for i in range(N_PACKS))
PACKS = [
    Box(name, pos=(x, y, TOP + REST_GAP + PACK_H / 2), size=(PACK_X, PACK_Y, PACK_H), mass=PACK_MASS, color=PACK_COLOR)
    for name, (x, y) in zip(PACK_NAMES, SUPPLY, strict=True)
]
OBJECTS = [*FIXTURES, *PACKS]

# -- scoring --------------------------------------------------------------------------------------------------------
NEAR_MAT = 0.080  # a pack's lowest point at most 80 mm above the mat: on it, or in a stack up to three high
HOUSE_EXACT = 1 / 3
HOUSE_OFF_BY_ONE = 1 / 6


def on_mat(state, h) -> bool:
    """Centre (seen from above) over the household's mat, lowest point within 80 mm of the mat top."""
    x0, x1, y0, y1 = mat_bounds(h)
    x, y, _ = (float(v) for v in state.pos)
    return bool(x0 <= x <= x1 and y0 <= y <= y1 and float(state.low[2]) <= MAT_TOP + NEAR_MAT)


def off_table(state) -> bool:
    """Centre below the table top: the pack has fallen to the floor."""
    return float(state.pos[2]) < TOP


def counts(objects) -> dict[str, int]:
    return {h: sum(on_mat(objects[n], h) for n in PACK_NAMES) for h in HOUSEHOLDS}


def ordered_by_size(c) -> bool:
    """Equity check: a household with more people holds strictly more packs than one with fewer."""
    by_size = sorted(HOUSEHOLDS, key=lambda h: PEOPLE[h])
    return all(c[a] < c[b] for a, b in zip(by_size, by_size[1:]))


def household_credit(n, target) -> float:
    return HOUSE_EXACT if n == target else HOUSE_OFF_BY_ONE if abs(n - target) == 1 else 0.0


def score(objects):
    """0 if any pack is on the floor, or if the households' counts are not strictly ordered by their sizes (an even
    split, or a larger household given no more than a smaller one). Otherwise, per household, 1/3 for exactly two
    packs per person and 1/6 for one pack more or fewer than that."""
    if any(off_table(objects[n]) for n in PACK_NAMES):
        return 0.0
    c = counts(objects)
    if not ordered_by_size(c):
        return 0.0
    return min(1.0, sum(household_credit(c[h], TARGET[h]) for h in HOUSEHOLDS))


def success(objects):
    """Six, two and four packs on the left, middle and right mats, every pack still."""
    return counts(objects) == TARGET and all(objects[n].resting() for n in PACK_NAMES)


def distance(objects):
    """0.05 m per pack by which a household's count misses its share, summed (0.6 at the start, 0 at the goal)."""
    c = counts(objects)
    return 0.05 * sum(abs(c[h] - TARGET[h]) for h in HOUSEHOLDS)


# -- demo -----------------------------------------------------------------------------------------------------------
GRIP_ABOVE_BOTTOM = 0.018  # TCP 18 mm above a pack's bottom: pad tips 5 mm above it, pads on its lower 30 mm
RELEASE_ABOVE = 0.003  # set a pack down 3 mm above its support
GRIP_SHIFT_Y = 0.006  # pads hold an object 11-20 mm toward -y (seen in sim); aim each place this much to +y
TRAVEL_Z = TOP + 0.130  # carried pack's bottom 112 mm up: 37 mm over a two-pack stack
LIFT_AFTER_RELEASE = 0.030
HOLD_S = 2.0
PARK = {"left": (0.090, 0.400, TRAVEL_Z), "right": (0.090, -0.400, TRAVEL_Z)}  # out of the camera's view of the mats

# Stack spots: one stack of two packs per person. Left mat: one stack at the back, two in front 120 mm apart; right
# mat: two stacks one behind the other; middle mat: one stack.
STACKS = {
    "left": ((0.380, 0.305), (0.290, 0.365), (0.290, 0.245)),
    "middle": ((0.290, -0.020),),
    "right": ((0.380, -0.300), (0.290, -0.300)),
}
# Per arm, in order: (supply pack (row x, column y), (household, stack)). Each stack takes the two packs of one
# supply column, back pack first. The left arm lays the bottom layer of its two front stacks before either gets its
# second pack, so an opening finger never comes down beside a full two-pack stack.
_B, _F = SUPPLY_ROW_X[1], SUPPLY_ROW_X[0]
PLAN = {
    "left": (
        ((_B, 0.255), ("left", 0)),
        ((_F, 0.255), ("left", 0)),
        ((_B, 0.360), ("left", 1)),
        ((_B, 0.150), ("left", 2)),
        ((_F, 0.360), ("left", 1)),
        ((_F, 0.150), ("left", 2)),
    ),
    "right": (
        ((_B, -0.150), ("middle", 0)),
        ((_F, -0.150), ("middle", 0)),
        ((_B, -0.255), ("right", 0)),
        ((_F, -0.255), ("right", 0)),
        ((_B, -0.360), ("right", 1)),
        ((_F, -0.360), ("right", 1)),
    ),
}


def demo_moves(side):
    """(pick xy, place xy, support top) for each of the arm's six moves, in order."""
    out, level = [], {}
    for pick, (h, k) in PLAN[side]:
        sx, sy = STACKS[h][k]
        n = level.get((h, k), 0)
        level[(h, k)] = n + 1
        out.append((pick, (sx, sy + GRIP_SHIFT_Y), MAT_TOP + n * PACK_H))
    return out


def _arm_plan(side):
    wps = []
    for (px, py), (qx, qy), support in demo_moves(side):
        pick_z = TOP + REST_GAP + GRIP_ABOVE_BOTTOM
        place_z = support + RELEASE_ABOVE + GRIP_ABOVE_BOTTOM
        wps += [
            Waypoint(1.4, (px, py, TRAVEL_Z), FINGER_OPEN, "to the pack"),
            Waypoint(1.2, (px, py, pick_z), FINGER_OPEN, "descend"),
            Waypoint(0.7, (px, py, pick_z), FINGER_CLOSED, "close"),
            Waypoint(1.0, (px, py, TRAVEL_Z), FINGER_CLOSED, "lift"),
            Waypoint(1.6, (qx, qy, TRAVEL_Z), FINGER_CLOSED, "carry"),
            Waypoint(1.4, (qx, qy, place_z), FINGER_CLOSED, "lower"),
            Waypoint(0.5, (qx, qy, place_z), FINGER_OPEN, "release"),
            Waypoint(0.5, (qx, qy, place_z + LIFT_AFTER_RELEASE), FINGER_OPEN, "lift off"),
            Waypoint(0.6, (qx, qy, TRAVEL_Z), FINGER_OPEN, "up"),
        ]
    wps += [
        Waypoint(1.5, PARK[side], FINGER_OPEN, "park"),
        Waypoint(HOLD_S, PARK[side], FINGER_OPEN, "hold still"),
    ]
    return wps


def demo():
    """Both arms at once, six packs each, fingers closing across the packs (yaw 0). The left arm makes the left
    household's three stacks of two from its own three supply columns; the right arm makes the middle household's
    stack, then the right household's two. Each pack is lowered onto the mat or onto the pack below, released 3 mm
    above it, and the gripper rises 3 cm before moving on. Then both arms park beside the table's front corners and
    hold still for 2 s."""
    return {side: _arm_plan(side) for side in ("left", "right")}


MOVE_LEN = 9  # waypoints per move in _arm_plan
DEMO_S = math.fsum(w.duration for w in _arm_plan("left"))
