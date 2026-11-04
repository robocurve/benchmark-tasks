"""At a dinner table: Mia, who is blind, sits at the far end facing the robot, with her plate in front of her and
her folded cane beside it. She was told her water is at 2 o'clock and her salt at 10 o'clock, in the clock
positions blind diners use: 12 o'clock is the side of the plate farthest from her, 3 o'clock is on her right.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CLOTH = 0.003
PLATE = np.array([0.44, 0.0])
TARGET_HOUR = {"water_glass": 2, "salt_shaker": 10}

GLASS = Cylinder("water_glass", pos=(0.20, -0.06, TOP + CLOTH + 0.05), radius=0.0325, height=0.10, mass=0.2,
                 color=(0.70, 0.85, 0.95))
SALT = Cylinder("salt_shaker", pos=(0.20, 0.06, TOP + CLOTH + 0.04), radius=0.02, height=0.08, mass=0.08,
                color=(0.96, 0.96, 0.94))
CANE = [Box(f"cane_{i}", pos=(0.68, -0.06 + 0.014 * i, TOP + CLOTH + 0.006), size=(0.15, 0.012, 0.012),
            color=(0.85, 0.10, 0.10) if i == 3 else (0.97, 0.97, 0.97)) for i in range(4)]
DRESSING = [
    Box("tablecloth", pos=(0.45, 0.0, TOP + CLOTH / 2), size=(0.80, 0.80, CLOTH), color=(0.12, 0.16, 0.32)),
    Asset("plate", "plate_large_vomp_robolab", xy=tuple(PLATE), extent=0.24, color=(0.97, 0.97, 0.95)),
    Sphere("bread_roll", pos=(0.44, 0.0, TOP + CLOTH + 0.045), radius=0.035, color=(0.80, 0.58, 0.30)),
    Asset("fork", "fork_big_vomp_robolab", xy=(0.44, -0.24), extent=0.18, yaw=np.pi / 2),
    Box("knife", pos=(0.44, 0.24, TOP + CLOTH + 0.002), size=(0.20, 0.018, 0.004), color=(0.78, 0.78, 0.80)),
    Cylinder("vase", pos=(0.64, 0.28, TOP + CLOTH + 0.08), radius=0.04, height=0.16, color=(0.20, 0.55, 0.52)),
    Sphere("flower", pos=(0.64, 0.28, TOP + CLOTH + 0.18), radius=0.035, color=(0.95, 0.75, 0.20)),
    *CANE,
]
OBJECTS = [*DRESSING, GLASS, SALT]


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _at_hour(obj, hour):
    """Mia faces the robot (-x), so her 12 o'clock points at the robot and her 3 o'clock is +y."""
    if _tilt_deg(obj) > 15 or obj.low[2] > TOP + CLOTH + 0.01 or not obj.resting():
        return False
    v = obj.pos[:2] - PLATE
    r = np.linalg.norm(v)
    a = np.radians(30 * hour)
    error = np.degrees(np.arccos(np.clip(v @ np.array([-np.cos(a), np.sin(a)]) / max(r, 1e-9), -1.0, 1.0)))
    return 0.08 <= r <= 0.25 and error <= 15


def score(objects):
    """0.5 for each of the water glass and salt shaker standing upright 8 to 25 cm from the plate centre and
    within 15 degrees of its clock position from Mia's seat. Scored at the end of the trial."""
    return 0.5 * sum(_at_hour(objects[name], hour) for name, hour in TARGET_HOUR.items())


def _place(obj, hour, r=0.17):
    a = np.radians(30 * hour)
    return float(PLATE[0] - r * np.cos(a)), float(PLATE[1] + r * np.sin(a))


def demo():
    """The left arm sets the water at Mia's 2 o'clock, on the robot's left; then the right arm sets the salt at her
    10 o'clock, on the robot's right. Both go on the robot's side of the plate; each arm then moves clear of the
    camera's view."""
    plans = {}
    for side, obj, hour, clear_y in (("left", GLASS, 2, 0.32), ("right", SALT, 10, -0.32)):
        grip = obj.pos[2] + 0.013
        plan = pick_place_waypoints((*obj.pos[:2], grip), (*_place(obj, hour), grip + 0.005), travel_z=TOP + 0.16)
        plan.append(Waypoint(2.0, (0.16, clear_y, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view"))
        plans[side] = plan
    wait = sum(w.duration for w in plans["left"])
    plans["right"].insert(0, Waypoint(wait, (0.16, -0.32, TOP + 0.20), FINGER_OPEN, "wait for the left arm"))
    return plans
