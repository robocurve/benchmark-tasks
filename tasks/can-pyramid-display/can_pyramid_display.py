"""In a grocery shop: six cans of tomato soup wait in a delivery tray next to a green display mat. A pyramid
display (three cans, then two, then one) only stands if the bottom row is spaced so each upper can bridges two
below it, and every can is set down square and centred.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
FLOOR = TOP + 0.003
MAT_TOP = FLOOR + 0.002
D, H = 0.068, 0.06
MAT = (np.array([0.32, 0.12]), np.array([0.12, 0.26]))  # centre, size
TRAY = [(0.14, 0.29), (0.14, 0.37), (0.14, 0.45), (0.26, 0.29), (0.26, 0.37), (0.26, 0.45)]
PX, CY, PITCH = 0.32, 0.12, 0.070
PLAN = [((PX, CY - PITCH), 0), ((PX, CY), 0), ((PX, CY + PITCH), 0),
        ((PX, CY - PITCH / 2), 1), ((PX, CY + PITCH / 2), 1), ((PX, CY), 2)]
SOUP = (0.80, 0.15, 0.12)

CANS = [Cylinder(f"can_{i}", pos=(*xy, FLOOR + H / 2), radius=D / 2, height=H, mass=0.3, color=SOUP)
        for i, xy in enumerate(TRAY)]
SHELF = (0.66, 0.0)
DRESSING = [
    Box("shop_floor", pos=(0.45, 0.0, TOP + 0.0015), size=(0.90, 1.0, 0.003), color=(0.86, 0.84, 0.80)),
    Box("display_mat", pos=(*map(float, MAT[0]), FLOOR + 0.001), size=(*map(float, MAT[1]), 0.002),
        color=(0.20, 0.50, 0.30)),
    Box("delivery_tray", pos=(0.20, 0.37, FLOOR + 0.002), size=(0.22, 0.26, 0.004), color=(0.70, 0.55, 0.35)),
    Box("shelf_back", pos=(SHELF[0] + 0.05, 0.0, FLOOR + 0.15), size=(0.02, 0.80, 0.30), color=(0.95, 0.95, 0.93)),
    *[Box(f"shelf_{k}", pos=(SHELF[0], 0.0, FLOOR + 0.002 + 0.12 * k), size=(0.10, 0.80, 0.012),
          color=(0.75, 0.75, 0.78)) for k in (0, 1)],
    *[Asset(f"stock_{i}", a, xy=(SHELF[0], y), extent=0.07) for i, (a, y) in enumerate(
        [("ketchup_bottle_hope_robolab", -0.25), ("mustard_bottle_hope_robolab", -0.12),
         ("bbq_sauce_bottle_hope_robolab", 0.14), ("mayonnaise_bottle_hope_robolab", 0.27)])],
    Box("price_card", pos=(0.43, 0.12, FLOOR + 0.04), size=(0.008, 0.10, 0.08), color=(0.98, 0.85, 0.15)),
    Box("basket", pos=(0.50, -0.28, FLOOR + 0.04), size=(0.16, 0.12, 0.08), color=(0.20, 0.35, 0.70)),
]
OBJECTS = [*DRESSING, *CANS]


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _on_mat(obj):
    centre, size = MAT
    return bool(np.all(np.abs(obj.pos[:2] - centre) <= size / 2))


def score(objects):
    """0 if any can ends tipped over (more than 45 degrees). Otherwise, counting upright cans (under 10 degrees) by
    tier from the mat: 1.0 with three on the mat, two on those and one on top; 0.5 with the bottom two tiers only;
    0.25 with exactly three upright on the mat and nothing above; 0 otherwise. Scored at the end of the trial."""
    cans = [objects[c.name] for c in CANS]
    if any(_tilt_deg(c) > 45 for c in cans):
        return 0.0
    tiers = [0, 0, 0]
    for c in cans:
        level = round((c.low[2] - MAT_TOP) / H)
        if _tilt_deg(c) < 10 and c.resting() and 0 <= level <= 2 and (level > 0 or _on_mat(c)):
            tiers[level] += 1
    if tiers == [3, 2, 1]:
        return 1.0
    if tiers == [3, 2, 0]:
        return 0.5
    return 0.25 if tiers == [3, 0, 0] else 0.0


def demo():
    """The left arm builds the pyramid can by can: the bottom row 7 cm apart, then the two cans bridging it, then the
    top one, gripping each with the fingers closing front to back so they never touch a neighbour."""
    left = []
    for (sx, sy), ((dx, dy), level) in zip(TRAY, PLAN):
        grip = FLOOR + 0.013 + H / 2
        place = MAT_TOP + level * H + 0.013 + H / 2 + 0.008
        left += pick_place_waypoints((sx, sy, grip), (dx, dy, place), travel_z=TOP + 0.175, yaw=math.pi / 2)
    left.append(Waypoint(2.0, (0.16, 0.32, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view"))
    return {"left": left}
