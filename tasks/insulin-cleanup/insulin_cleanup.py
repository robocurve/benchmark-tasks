"""At a kitchen table after a home insulin shot: a used syringe and an empty yogurt pot lie on the placemat beside
the insulin vial and glucose meter. A yellow sharps container and a kitchen bin stand behind. Used syringes go
only in the sharps container, never in household rubbish, and the vial and meter are not rubbish.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SLAB = 0.003
DESK = TOP + SLAB
BIN_H, WALL = 0.12, 0.012
SHARPS = (np.array([0.40, 0.20]), np.array([0.18, 0.12]))  # centre, outer size
KITCHEN = (np.array([0.40, -0.22]), np.array([0.20, 0.20]))
YELLOW, RED, GREY = (0.95, 0.80, 0.10), (0.80, 0.12, 0.10), (0.80, 0.80, 0.78)

SYRINGE = Box("syringe", pos=(0.20, 0.10, DESK + 0.008), size=(0.11, 0.016, 0.016), mass=0.01,
              color=(0.96, 0.96, 0.96))
YOGURT = Asset("yogurt_pot", "yogurt_cup_hope_robolab", xy=(0.17, -0.17), extent=0.07, mass=0.01)
VIAL = Cylinder("insulin_vial", pos=(0.32, 0.06, DESK + 0.0225), radius=0.015, height=0.045, mass=0.03,
                color=(0.90, 0.55, 0.15))
METER = Box("glucose_meter", pos=(0.33, -0.05, DESK + 0.01), size=(0.09, 0.055, 0.02), mass=0.08, yaw=0.3,
            color=(0.25, 0.25, 0.28))
KEEP = [VIAL, METER]


def _bin(name, centre, size, colour, rim):
    (cx, cy), (lx, ly), z = centre, size, DESK + BIN_H / 2
    walls = [
        Box(f"{name}_near", pos=(cx - lx / 2 + WALL / 2, cy, z), size=(WALL, ly, BIN_H), color=colour),
        Box(f"{name}_far", pos=(cx + lx / 2 - WALL / 2, cy, z), size=(WALL, ly, BIN_H), color=colour),
        Box(f"{name}_left", pos=(cx, cy + ly / 2 - WALL / 2, z), size=(lx - 2 * WALL, WALL, BIN_H), color=colour),
        Box(f"{name}_right", pos=(cx, cy - ly / 2 + WALL / 2, z), size=(lx - 2 * WALL, WALL, BIN_H), color=colour),
    ]
    rims = [Box(f"{name}_rim_{i}", pos=(cx + dx, cy + dy, DESK + BIN_H + 0.005), size=s, color=rim) for i, (dx, dy, s)
            in enumerate([(-(lx - WALL) / 2, 0, (WALL, ly, 0.01)), ((lx - WALL) / 2, 0, (WALL, ly, 0.01)),
                          (0, (ly - WALL) / 2, (lx, WALL, 0.01)), (0, -(ly - WALL) / 2, (lx, WALL, 0.01))])]
    return walls + rims


DRESSING = [
    Box("tabletop", pos=(0.45, 0.0, TOP + SLAB / 2), size=(0.90, 1.0, SLAB), color=(0.62, 0.45, 0.30)),
    Box("placemat", pos=(0.26, -0.06, DESK + 0.001), size=(0.22, 0.46, 0.002), color=(0.25, 0.40, 0.60)),
    *_bin("sharps", *SHARPS, YELLOW, RED),
    *_bin("kitchen_bin", *KITCHEN, GREY, (0.55, 0.55, 0.55)),
    Box("bin_pedal", pos=(0.29, -0.22, DESK + 0.01), size=(0.03, 0.08, 0.02), color=(0.40, 0.40, 0.40)),
    Box("meter_case", pos=(0.58, 0.02, DESK + 0.015), size=(0.14, 0.09, 0.03), color=(0.15, 0.20, 0.35)),
    Cylinder("plant_pot", pos=(0.62, 0.30, DESK + 0.05), radius=0.05, height=0.10, color=(0.70, 0.38, 0.22)),
    Sphere("plant", pos=(0.62, 0.30, DESK + 0.14), radius=0.06, color=(0.25, 0.55, 0.25)),
    Box("newspaper", pos=(0.62, -0.30, DESK + 0.003), size=(0.20, 0.14, 0.006), yaw=0.15, color=(0.88, 0.87, 0.82)),
]
OBJECTS = [*DRESSING, SYRINGE, YOGURT, *KEEP]


def _inside(obj, container):
    """Whole footprint inside the container's walls (2 cm tolerance, for an item leaning on a wall) and below
    its rim."""
    centre, size = container
    lo, hi = centre - size / 2 + WALL - 0.02, centre + size / 2 - WALL + 0.02
    return bool(np.all(obj.low[:2] > lo) and np.all(obj.high[:2] < hi) and obj.high[2] < DESK + BIN_H + 0.01)


def score(objects):
    """0 unless the syringe ends inside the sharps container, or if the vial or meter ends in either bin. Otherwise
    1.0 with the yogurt pot in the kitchen bin, 0.5 if not. Scored at the end of the trial."""
    bins = (SHARPS, KITCHEN)
    if not _inside(objects["syringe"], SHARPS) or any(_inside(objects[o.name], b) for o in KEEP for b in bins):
        return 0.0
    return 1.0 if _inside(objects["yogurt_pot"], KITCHEN) else 0.5


def demo():
    """The left arm drops the syringe into the sharps container while the right arm drops the yogurt pot into the
    kitchen bin. Each arm then moves clear of the camera's view."""
    def drop(obj, grip_height, container):
        xy = obj.xy if isinstance(obj, Asset) else obj.pos[:2]
        plan = pick_place_waypoints((*map(float, xy), DESK + 0.013 + grip_height),
                                    (*map(float, container[0]), DESK + BIN_H + 0.04), travel_z=TOP + 0.17, yaw=obj.yaw)
        i = next(k for k, w in enumerate(plan) if w.label == "descend") + 1
        settle = Waypoint(0.6, plan[i - 1].pos, FINGER_OPEN, "settle", obj.yaw)  # the wrist trails its target
        return [*plan[:i], settle, *plan[i:]]

    clear = "move clear of the camera's view"
    left = [*drop(SYRINGE, 0.008, SHARPS), Waypoint(2.0, (0.16, 0.32, TOP + 0.20), FINGER_OPEN, clear)]
    right = [*drop(YOGURT, 0.025, KITCHEN), Waypoint(2.0, (0.16, -0.32, TOP + 0.20), FINGER_OPEN, clear)]
    return {"left": left, "right": right}
