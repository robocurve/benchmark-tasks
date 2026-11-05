"""At a kitchen counter: a tall cereal box stands in front of a low cupboard whose opening is only 9 cm high. The
box fits inside only lying on its side, and the gripper can only turn it about the vertical, so it has to be tipped
over with a push and then slid in.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SLAB = 0.003
DESK = TOP + SLAB
CY = -0.14
FRONT, BACK, INNER_W, OPENING = 0.455, 0.663, 0.236, 0.094  # cupboard inside: x from FRONT to BACK, y +-INNER_W/2
RECESS = 0.03  # the top is set back from the front of the side walls
WOOD = (0.55, 0.40, 0.26)

CEREAL = Box("cereal_box", pos=(0.26, CY, DESK + 0.09), size=(0.05, 0.08, 0.18), mass=0.35, color=(0.92, 0.55, 0.12))
CUPBOARD = [
    Box("cupboard_top", pos=((FRONT + RECESS + BACK) / 2 + 0.006, CY, DESK + OPENING + 0.006),
        size=(BACK - FRONT - RECESS + 0.012, INNER_W + 0.024, 0.012), color=WOOD),
    *[Box(f"cupboard_side_{i}", pos=((FRONT + BACK) / 2, CY + s * (INNER_W + 0.012) / 2, DESK + OPENING / 2),
          size=(BACK - FRONT, 0.012, OPENING), color=WOOD) for i, s in enumerate((1, -1))],
    Box("cupboard_back", pos=(BACK + 0.006, CY, DESK + OPENING / 2), size=(0.012, INNER_W + 0.024, OPENING),
        color=WOOD),
]
DRESSING = [
    Box("counter", pos=(0.45, 0.0, TOP + SLAB / 2), size=(0.90, 1.0, SLAB), color=(0.85, 0.84, 0.80)),
    *CUPBOARD,
    Asset("fruit_bowl", "wooden_bowl_hot3d_robolab", xy=(0.46, 0.24), extent=0.16),
    Cylinder("jar", pos=(0.58, 0.10, DESK + 0.07), radius=0.04, height=0.14, color=(0.75, 0.55, 0.35)),
    Box("tea_towel", pos=(0.24, 0.08, DESK + 0.002), size=(0.16, 0.10, 0.004), yaw=0.2, color=(0.80, 0.25, 0.25)),
]
OBJECTS = [*DRESSING, CEREAL]


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def score(objects):
    """1.0 with the box lying on its side (tilted 60 degrees or more) wholly inside the cupboard (1 cm tolerance),
    0.5 lying with at least half its length inside, 0 otherwise. Scored at the end of the trial."""
    box = objects["cereal_box"]
    if _tilt_deg(box) < 60 or not box.resting() or box.low[2] < TOP - 0.01:
        return 0.0
    if box.low[1] < CY - INNER_W / 2 - 0.01 or box.high[1] > CY + INNER_W / 2 + 0.01:
        return 0.0
    length = box.high[0] - box.low[0]
    inside = min(box.high[0], BACK) - max(box.low[0], FRONT)
    if box.low[0] >= FRONT - 0.01 and box.high[0] <= BACK + 0.01 and box.high[2] < DESK + OPENING + 0.005:
        return 1.0
    return 0.5 if inside >= length / 2 else 0.0


def demo():
    """The right arm pushes the top of the box away from the robot so it tips onto its side, then pushes it from
    behind into the cupboard and moves clear of the camera's view."""
    high, top, low = TOP + 0.17, DESK + 0.155, DESK + 0.032
    steps = [(1.5, (0.16, -0.30, high), "up and over"), (1.5, (0.19, CY, high), "behind the box"),
             (1.0, (0.19, CY, top), "level with its top"), (2.0, (0.33, CY, top), "push the top over"),
             (1.0, (0.33, CY, top), "let it settle"), (1.0, (0.33, CY, high), "lift"),
             (1.5, (0.25, CY, high), "behind the fallen box"), (1.5, (0.25, CY, low), "down to the counter"),
             (3.0, (0.49, CY, low), "slide it in"), (1.0, (0.49, CY, low), "hold"), (1.5, (0.35, CY, high), "back out"),
             (1.5, (0.16, -0.32, TOP + 0.20), "move clear of the camera's view")]
    return {"right": [Waypoint(d, p, FINGER_CLOSED, label) for d, p, label in steps]}
