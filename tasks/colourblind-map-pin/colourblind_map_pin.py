"""A route-planning map on the table. Mia's red pin marks the first stop; a white cross marks the second. Mia is
red-green colour-blind: red, green, orange and brown all look alike to her, so only the blue pin in the tray can be
told apart from hers.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MAP_TOP = TOP + 0.004
TRAY_TOP = TOP + 0.01
PIN_R, PIN_H = 0.015, 0.07
STOP_A, STOP_B = np.array([0.46, -0.12]), np.array([0.36, 0.02])
TOWNS = [STOP_A, STOP_B, np.array([0.26, 0.17]), np.array([0.54, 0.15]), np.array([0.28, -0.16])]
COLOURS = {"red": (0.80, 0.12, 0.10), "green": (0.20, 0.55, 0.15), "orange": (0.90, 0.50, 0.10),
           "brown": (0.45, 0.28, 0.12), "blue": (0.15, 0.30, 0.85)}
TRAY_Y = {"blue": 0.12, "green": 0.04, "brown": -0.04, "orange": -0.12}

RED_PIN = Cylinder("pin_red", pos=(*map(float, STOP_A), MAP_TOP + PIN_H / 2), radius=PIN_R, height=PIN_H, mass=0.02,
                   color=COLOURS["red"])
SPARE_PINS = [Cylinder(f"pin_{c}", pos=(0.16, y, TRAY_TOP + PIN_H / 2), radius=PIN_R, height=PIN_H, mass=0.02,
                       color=COLOURS[c]) for c, y in TRAY_Y.items()]
CONFUSABLE = ["pin_green", "pin_orange", "pin_brown"]


def _road(name, a, b):
    (ax, ay), (bx, by) = a, b
    return Box(name, pos=((ax + bx) / 2, (ay + by) / 2, TOP + 0.0035), size=(float(np.hypot(bx - ax, by - ay)), 0.008,
               0.001), yaw=math.atan2(by - ay, bx - ax), color=(0.55, 0.55, 0.55))


MAP = [
    Box("sea", pos=(0.40, 0.0, TOP + 0.001), size=(0.44, 0.56, 0.002), color=(0.62, 0.78, 0.90)),
    Box("mainland", pos=(0.40, 0.02, TOP + 0.0025), size=(0.36, 0.42, 0.001), color=(0.90, 0.85, 0.70)),
    Box("island", pos=(0.57, -0.21, TOP + 0.0025), size=(0.07, 0.08, 0.001), color=(0.90, 0.85, 0.70)),
    Box("park_north", pos=(0.30, 0.10, TOP + 0.003), size=(0.06, 0.05, 0.001), color=(0.55, 0.75, 0.45)),
    Box("park_east", pos=(0.52, 0.04, TOP + 0.003), size=(0.05, 0.07, 0.001), color=(0.55, 0.75, 0.45)),
    _road("road_ab", STOP_A, STOP_B), _road("road_bc", STOP_B, TOWNS[2]), _road("road_bd", STOP_B, TOWNS[3]),
    _road("road_ae", STOP_A, TOWNS[4]),
    *[Box(f"town_{i}", pos=(*map(float, t), TOP + 0.004), size=(0.03, 0.03, 0.001), color=(0.30, 0.30, 0.32))
      for i, t in enumerate(TOWNS)],
    *[Box(f"cross_{i}", pos=(*map(float, STOP_B), MAP_TOP + 0.0005), size=(0.045, 0.007, 0.001), yaw=yaw,
          color=(0.98, 0.98, 0.98)) for i, yaw in enumerate((math.pi / 4, -math.pi / 4))],
]
DRESSING = [
    *MAP,
    Box("pin_tray", pos=(0.16, 0.0, TOP + 0.005), size=(0.06, 0.32, 0.01), color=(0.55, 0.40, 0.25)),
    Box("notepad", pos=(0.20, -0.32, TOP + 0.004), size=(0.14, 0.10, 0.008), color=(0.95, 0.92, 0.60)),
    Box("pencil", pos=(0.21, -0.25, TOP + 0.004), size=(0.15, 0.008, 0.008), yaw=0.2, color=(0.95, 0.75, 0.15)),
    Box("ruler", pos=(0.66, 0.0, TOP + 0.002), size=(0.03, 0.30, 0.004), color=(0.85, 0.85, 0.80)),
    Asset("coffee_mug", "mug_ycb_robolab", xy=(0.62, 0.32), extent=0.085),
]
OBJECTS = [*DRESSING, RED_PIN, *SPARE_PINS]


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def score(objects):
    """0 if Mia's red pin moved more than 2 cm or a green, orange or brown pin stands within 5 cm of the cross.
    Otherwise 1.0 with the blue pin standing within 3 cm of the cross, 0.5 within 10 cm, 0 beyond. Scored at the end
    of the trial."""
    if np.linalg.norm(objects["pin_red"].pos[:2] - STOP_A) > 0.02:
        return 0.0
    if any(np.linalg.norm(objects[n].pos[:2] - STOP_B) <= 0.05 for n in CONFUSABLE):
        return 0.0
    blue = objects["pin_blue"]
    if _tilt_deg(blue) > 15 or not blue.resting() or blue.low[2] > MAP_TOP + 0.01:
        return 0.0
    d = np.linalg.norm(blue.pos[:2] - STOP_B)
    return 1.0 if d <= 0.03 else 0.5 if d <= 0.10 else 0.0


def demo():
    """The left arm stands the blue pin on the white cross, then moves clear of the camera's view."""
    grip = TRAY_TOP + 0.013 + PIN_H / 2
    place = (*map(float, STOP_B), MAP_TOP + 0.013 + PIN_H / 2 + 0.005)
    plan = pick_place_waypoints((0.16, TRAY_Y["blue"], grip), place, travel_z=TOP + 0.13)
    plan.append(Waypoint(2.0, (0.16, 0.32, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view"))
    return {"left": plan}
