"""On the station table: three open-top cardboard cartons stand in a row across the table, open side up, from the
left arm's side: a small one, a medium one and a large one (inner 45 x 50, 55 x 70 and 65 x 85 mm, 25 mm deep).
Two loose products lie flat on the table nearer the arms: a blue one (42 x 58 x 30 mm) in front of the left arm
and an orange one (52 x 74 x 30 mm) in front of the right arm.

Goal: each product lying flat inside the smallest carton it fits in (blue in the medium carton, orange in the large
one: the blue product is too long for the small carton and the orange one too long for the medium carton).

Run it:  inspect-robots run --embodiment yam_arena -E task=right_size_cartons.py -E object_jitter=0 \\
           --instruction "Pack each product into the smallest carton it fits in, one product per carton." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WALL_T, WALL_H = 0.006, 0.025
CARDBOARD = (0.72, 0.55, 0.35)
CARTONS_X = 0.36
# carton name -> (inner x, inner y, centre y)
CARTONS = {
    "small": (0.045, 0.050, 0.120),
    "medium": (0.055, 0.070, 0.018),
    "large": (0.065, 0.085, -0.1015),
}
PRODUCTS = {  # name -> (size x, y, z), start (x, y), mass, colour, smallest carton it fits in
    "blue_product": ((0.042, 0.058, 0.030), (0.22, 0.14), 0.06, (0.15, 0.35, 0.80), "medium"),
    "orange_product": ((0.052, 0.074, 0.030), (0.22, -0.12), 0.08, (0.95, 0.50, 0.10), "large"),
}


def _walls(name, ix, iy, cy):
    """Four fixed walls around an inner ix x iy opening centred at (CARTONS_X, cy)."""
    ox, oy, z = ix / 2 + WALL_T / 2, iy / 2 + WALL_T / 2, TOP + WALL_H / 2
    long_x, long_y = ix + 2 * WALL_T, iy
    return [
        Box(f"{name}_carton_near", pos=(CARTONS_X - ox, cy, z), size=(WALL_T, long_y, WALL_H), color=CARDBOARD),
        Box(f"{name}_carton_far", pos=(CARTONS_X + ox, cy, z), size=(WALL_T, long_y, WALL_H), color=CARDBOARD),
        Box(f"{name}_carton_left", pos=(CARTONS_X, cy + oy, z), size=(long_x, WALL_T, WALL_H), color=CARDBOARD),
        Box(f"{name}_carton_right", pos=(CARTONS_X, cy - oy, z), size=(long_x, WALL_T, WALL_H), color=CARDBOARD),
    ]


OBJECTS = [w for n, (ix, iy, cy) in CARTONS.items() for w in _walls(n, ix, iy, cy)] + [
    Box(n, pos=(sx, sy, TOP + size[2] / 2), size=size, mass=m, color=c)
    for n, (size, (sx, sy), m, c, _) in PRODUCTS.items()
]


def _carton_of(objects, name):
    """The carton a product lies flat inside (centre within the opening, resting on the carton floor), else None."""
    p = objects[name].pos
    size = PRODUCTS[name][0]
    if abs(p[2] - (TOP + size[2] / 2)) > 0.004:
        return None
    for carton, (ix, iy, cy) in CARTONS.items():
        if abs(p[0] - CARTONS_X) < ix / 2 and abs(p[1] - cy) < iy / 2:
            return carton
    return None


def _product_score(objects, name):
    """0.5 in the smallest carton it fits in, 0.2 in another carton, else 0."""
    carton = _carton_of(objects, name)
    if carton is None:
        return 0.0
    return 0.5 if carton == PRODUCTS[name][4] else 0.2


def score(objects):
    """0.5 per product lying flat in its smallest fitting carton, 0.2 per product in a larger carton."""
    return sum(_product_score(objects, n) for n in PRODUCTS)


def success(objects):
    return all(_product_score(objects, n) == 0.5 for n in PRODUCTS) and all(objects[n].resting() for n in PRODUCTS)


def distance(objects):
    return float(sum(
        math.hypot(objects[n].pos[0] - CARTONS_X, objects[n].pos[1] - CARTONS[PRODUCTS[n][4]][2]) for n in PRODUCTS
    ))


# -- scripted demo --------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.09  # higher, the wrist (joint4) nears its limit reaching over the medium carton
YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along world x, across each product's short side
GRIP_ABOVE_BOTTOM = PAD_TIP_PAST_TCP + 0.008  # TCP height above a product's underside when gripping it
RELEASE_GAP = 0.004  # let go with the product's underside this far above the carton rim
SETTLE_S = 0.8


def _pack(name, yaw):
    size, (sx, sy), _, _, carton = PRODUCTS[name]
    cy = CARTONS[carton][2]
    open_w = -(size[0] / 2 + 0.012)  # finger target: pads 12 mm clear of each side of the product
    pick_z = TOP + GRIP_ABOVE_BOTTOM
    drop_z = TOP + WALL_H + RELEASE_GAP + GRIP_ABOVE_BOTTOM
    return [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), open_w, f"to the {name}", yaw),
        Waypoint(1.2, (sx, sy, pick_z), open_w, "descend", yaw),
        Waypoint(0.8, (sx, sy, pick_z), 0.0, "grip", yaw),
        Waypoint(1.0, (sx, sy, TRAVEL_Z), 0.0, "lift", yaw),
        Waypoint(1.8, (CARTONS_X, cy, TRAVEL_Z), 0.0, f"over the {carton} carton", yaw),
        Waypoint(1.0, (CARTONS_X, cy, drop_z + 0.012), 0.0, "lower", yaw),
        Waypoint(SETTLE_S, (CARTONS_X, cy, drop_z + 0.012), 0.0, "settle", yaw),
        Waypoint(0.8, (CARTONS_X, cy, drop_z), 0.0, "to the rim", yaw),
        Waypoint(SETTLE_S, (CARTONS_X, cy, drop_z), 0.0, "settle", yaw),
        Waypoint(0.6, (CARTONS_X, cy, drop_z), open_w, "let go", yaw),
        Waypoint(1.0, (CARTONS_X, cy, TRAVEL_Z), open_w, "up", yaw),
    ]


def demo():
    """Left arm packs the blue product into the medium carton and moves clear; then the right arm packs the orange
    product into the large carton. Each product is let go just above the rim and drops in flat."""
    left = _pack("blue_product", YAW["left"]) + [Waypoint(1.5, (0.20, 0.25, TRAVEL_Z), FINGER_OPEN, "clear", YAW["left"])]
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, (0.20, -0.25, TRAVEL_Z), FINGER_OPEN, "wait for the left arm")] + _pack(
        "orange_product", YAW["right"]
    )
    return {"left": left, "right": right}
