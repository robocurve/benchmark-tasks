"""On the station table: a small flatbed trailer is fixed in front of the arms, its bed (240 mm side to side, 60 mm
front to back, 20 mm thick) running left to right with its tongue and hitch pointing left. A red stripe across the bed
marks the axle line, 20 mm right of the bed's centre, with a wheel on each side. Three loose 40 mm crates stand on the
table nearer the arms: a grey steel crate (160 g) on the left and two brown wooden crates (40 g each), one in the middle
and one on the right.

Goal: all three crates standing on the bed with at least 60% of their total weight ahead of the axle (on the hitch
side), the usual rule against trailer sway. Only the steel crate's position decides it.

Run it:  inspect-robots run --embodiment yam_arena -E task=trailer_sixty_forty.py -E object_jitter=0 \\
           --instruction "Load all three crates onto the trailer bed so that at least 60% of their total weight is \\
           ahead of the axle (the red line), on the hitch side. The grey steel crate weighs 160 g and each wooden crate \\
           40 g." --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the trailer (all fixed) ---------------------------------------------------------------------------------------
BED_X, BED_DEPTH, BED_Y0, BED_Y1, BED_T = 0.33, 0.060, -0.12, 0.12, 0.020  # bed: x centre, depth, right/left ends
BED_TOP = TOP + BED_T
AXLE_Y, STRIPE_W = -0.02, 0.006  # axle line (y) and the red stripe's width; the hitch side is +y
WOOD = (0.55, 0.42, 0.30)
RED = (0.80, 0.12, 0.10)
BLACK = (0.08, 0.08, 0.08)
STEEL = (0.50, 0.52, 0.55)
BROWN = (0.60, 0.40, 0.20)
_zc = TOP + BED_T / 2
_l_front = BED_Y1 - (AXLE_Y + STRIPE_W / 2)  # bed length ahead of the stripe
_l_back = (AXLE_Y - STRIPE_W / 2) - BED_Y0

OBJECTS = [
    Box("bed_front", pos=(BED_X, BED_Y1 - _l_front / 2, _zc), size=(BED_DEPTH, _l_front, BED_T), color=WOOD),
    Box("axle_stripe", pos=(BED_X, AXLE_Y, _zc), size=(BED_DEPTH, STRIPE_W, BED_T), color=RED),
    Box("bed_back", pos=(BED_X, BED_Y0 + _l_back / 2, _zc), size=(BED_DEPTH, _l_back, BED_T), color=WOOD),
    Box("tongue", pos=(BED_X, BED_Y1 + 0.035, TOP + 0.006), size=(0.012, 0.070, 0.012), color=BLACK),
    Box("hitch", pos=(BED_X, BED_Y1 + 0.075, TOP + 0.010), size=(0.024, 0.016, 0.020), color=BLACK),
    *[Box(f"wheel_{s}", pos=(BED_X + d * (BED_DEPTH / 2 + 0.010), AXLE_Y, TOP + 0.022), size=(0.014, 0.044, 0.044),
          color=BLACK) for s, d in (("front", -1), ("back", 1))],
]

# -- the crates ----------------------------------------------------------------------------------------------------
CRATE = 0.040
CRATES = {
    "steel_crate": {"mass": 0.16, "start": (0.22, 0.15), "color": STEEL},
    "wood_crate_a": {"mass": 0.04, "start": (0.22, 0.05), "color": BROWN},
    "wood_crate_b": {"mass": 0.04, "start": (0.22, -0.12), "color": BROWN},
}
OBJECTS += [Box(n, pos=(c["start"][0], c["start"][1], TOP + CRATE / 2), size=(CRATE, CRATE, CRATE), mass=c["mass"],
                color=c["color"]) for n, c in CRATES.items()]

# -- scoring -------------------------------------------------------------------------------------------------------
AHEAD_MIN = 0.60
FLAT_DZ, FLAT_TILT_DEG = 0.004, 5.0


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def on_bed(st):
    """Standing on the bed: centre within 4 mm of its resting height, tilted under 5 degrees, and its whole top-down
    footprint (from its corners) over the bed."""
    R = _rot(st.quat_xyzw)
    tilt = math.degrees(math.acos(min(1.0, abs(R[2, 2]))))
    c = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)]) * (CRATE / 2) @ R.T
    xy = st.pos[:2] + c[:, :2]
    inside = (xy[:, 0].min() >= BED_X - BED_DEPTH / 2 and xy[:, 0].max() <= BED_X + BED_DEPTH / 2
              and xy[:, 1].min() >= BED_Y0 and xy[:, 1].max() <= BED_Y1)
    return abs(st.pos[2] - (BED_TOP + CRATE / 2)) < FLAT_DZ and tilt < FLAT_TILT_DEG and inside


def ahead_fraction(objects):
    """Share of the crates' total weight whose centres are ahead of the axle line (hitch side)."""
    total = sum(c["mass"] for c in CRATES.values())
    return sum(c["mass"] for n, c in CRATES.items() if objects[n].pos[1] > AXLE_Y) / total


def score(objects):
    """0.2 per crate standing on the bed; +0.4 if all three are on it with at least 60% of the weight ahead of the axle."""
    ok = [on_bed(objects[n]) for n in CRATES]
    s = 0.2 * sum(ok)
    if all(ok) and ahead_fraction(objects) >= AHEAD_MIN:
        s += 0.4
    return round(s, 3)


def success(objects):
    return score(objects) >= 0.999 and all(objects[n].resting() for n in CRATES)


TARGET = {"steel_crate": (BED_X, 0.085), "wood_crate_a": (BED_X, 0.025), "wood_crate_b": (BED_X, -0.070)}


def distance(objects):
    return float(sum(math.hypot(objects[n].pos[0] - x, objects[n].pos[1] - y) for n, (x, y) in TARGET.items()))


# -- scripted demo -------------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.09
YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along world x
GRIP_ABOVE_BOTTOM = PAD_TIP_PAST_TCP + 0.010
RELEASE_GAP = 0.003
SETTLE_S = 0.8
OPEN_W = -(CRATE / 2 + 0.012)


def _load(name, yaw):
    sx, sy = CRATES[name]["start"]
    gx, gy = TARGET[name]
    pick_z = TOP + GRIP_ABOVE_BOTTOM
    drop_z = BED_TOP + RELEASE_GAP + GRIP_ABOVE_BOTTOM
    return [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), OPEN_W, f"to the {name}", yaw),
        Waypoint(1.2, (sx, sy, pick_z), OPEN_W, "descend", yaw),
        Waypoint(0.8, (sx, sy, pick_z), 0.0, "grip", yaw),
        Waypoint(1.0, (sx, sy, TRAVEL_Z), 0.0, "lift", yaw),
        Waypoint(1.8, (gx, gy, TRAVEL_Z), 0.0, "over the bed", yaw),
        Waypoint(1.0, (gx, gy, drop_z + 0.012), 0.0, "lower", yaw),
        Waypoint(SETTLE_S, (gx, gy, drop_z + 0.012), 0.0, "settle", yaw),
        Waypoint(0.8, (gx, gy, drop_z), 0.0, "to the bed", yaw),
        Waypoint(SETTLE_S, (gx, gy, drop_z), 0.0, "settle", yaw),
        Waypoint(0.6, (gx, gy, drop_z), OPEN_W, "let go", yaw),
        Waypoint(1.0, (gx, gy, TRAVEL_Z), OPEN_W, "up", yaw),
    ]


def demo():
    """Left arm loads the steel crate near the hitch end, then the middle wooden crate just ahead of the axle, and moves
    clear; the right arm loads the right wooden crate behind the axle. 200 g of 240 g (83%) ends up ahead of the axle."""
    left = _load("steel_crate", YAW["left"]) + _load("wood_crate_a", YAW["left"]) + [
        Waypoint(1.5, (0.20, 0.25, TRAVEL_Z), FINGER_OPEN, "clear", YAW["left"])]
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, (0.20, -0.25, TRAVEL_Z), FINGER_OPEN, "wait for the left arm")] + _load(
        "wood_crate_b", YAW["right"]) + [Waypoint(1.5, (0.20, -0.25, TRAVEL_Z), FINGER_OPEN, "clear", YAW["right"])]
    return {"left": left, "right": right}
