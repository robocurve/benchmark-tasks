"""In a living room: Mia sits at the far end of the table, at the green placemat with her teacup, facing the
robot. A large cardboard box stands in the middle of the table, and a small wrapped present lies near the robot's
left arm, in plain view of Mia.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CLOTH = 0.003
EYE = np.array([0.88, 0.0, TOP + 0.35])  # Mia's eyes, seated at the far end of her placemat

CARDBOARD = Box("cardboard_box", pos=(0.40, 0.0, TOP + CLOTH + 0.11), size=(0.10, 0.22, 0.22), mass=1.0,
                color=(0.70, 0.55, 0.35))
PRESENT = Box("present", pos=(0.15, 0.27, TOP + CLOTH + 0.03), size=(0.06, 0.06, 0.06), mass=0.15,
              color=(0.60, 0.20, 0.55))
DRESSING = [
    Box("tablecloth", pos=(0.45, 0.0, TOP + CLOTH / 2), size=(0.80, 0.72, CLOTH), color=(0.92, 0.90, 0.84)),
    Box("placemat", pos=(0.80, 0.0, TOP + CLOTH + 0.0015), size=(0.14, 0.30, 0.003), color=(0.25, 0.60, 0.35)),
    Asset("teacup", "mug_ycb_robolab", xy=(0.79, 0.08), extent=0.085),
    Asset("pumpkin", "pumpkinsmall_vomp_robolab", xy=(0.46, -0.30), extent=0.10),
    Cylinder("candle", pos=(0.48, 0.30, TOP + CLOTH + 0.07), radius=0.025, height=0.14, color=(0.95, 0.93, 0.80)),
]
OBJECTS = [*DRESSING, CARDBOARD, PRESENT]
START = {o.name: np.array(o.pos[:2]) for o in (CARDBOARD, PRESENT)}


def _blocked(point, box):
    """Whether the cardboard box's bounding box cuts the line from point to Mia's eyes."""
    d, t0, t1 = EYE - point, 0.0, 1.0
    for a in range(3):
        if abs(d[a]) < 1e-9:
            if not box.low[a] <= point[a] <= box.high[a]:
                return False
            continue
        u, v = (box.low[a] - point[a]) / d[a], (box.high[a] - point[a]) / d[a]
        t0, t1 = max(t0, min(u, v)), min(t1, max(u, v))
    return t0 <= t1


def score(objects):
    """0 if the cardboard box ends more than 2 cm from its start or the present is not resting on the table.
    Otherwise 1.0 if all four top corners of the present are hidden from Mia's eyes, 0.5 if only the top centre
    is, and 0 if she can see it. Scored at the end of the trial."""
    box, present = objects["cardboard_box"], objects["present"]
    if np.linalg.norm(box.pos[:2] - START["cardboard_box"]) > 0.02:
        return 0.0
    if present.low[2] > TOP + CLOTH + 0.01 or not present.resting():
        return 0.0
    top = present.high[2]
    corners = [np.array([x, y, top]) for x in (present.low[0], present.high[0]) for y in (present.low[1], present.high[1])]
    if all(_blocked(c, box) for c in corners):
        return 1.0
    return 0.5 if _blocked(np.array([*present.pos[:2], top]), box) else 0.0


def demo():
    """Left arm sets the present down between the cardboard box and the robot, out of Mia's sight, then moves
    clear of the camera's view."""
    grip = TOP + CLOTH + 0.013 + PRESENT.size[2] / 2
    plan = pick_place_waypoints((*PRESENT.pos[:2], grip), (0.27, 0.0, grip + 0.005), travel_z=TOP + 0.16)
    plan.append(Waypoint(2.0, (0.16, 0.32, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view"))
    return {"left": plan}
