"""On a hallway console table, against a wall with a mirror: a green bowl on the robot's left and a blue bowl on
its right, with a pair of glasses in the blue bowl. A glasses case lies on the runner in front of the bowls. A
birdhouse and a lizard figurine decorate the table.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BOWL = "serving_bowl_vomp_robolab"
BOWLS = {"green_bowl": ((0.30, 0.13), (0.25, 0.60, 0.35)), "blue_bowl": ((0.30, -0.13), (0.20, 0.35, 0.75))}
BOWL_FLOOR, BOWL_RIM = TOP + 0.008, TOP + 0.059
CASE = Box("glasses_case", pos=(0.16, 0.0, TOP + 0.003 + 0.0175), size=(0.12, 0.05, 0.035), mass=0.08, color=(0.30, 0.18, 0.10))

DRESSING = [
    Box("wall", pos=(0.52, 0.0, TOP + 0.20), size=(0.02, 0.80, 0.40), color=(0.90, 0.88, 0.82)),
    Box("mirror", pos=(0.508, 0.0, TOP + 0.24), size=(0.006, 0.34, 0.22), color=(0.75, 0.80, 0.85)),
    Box("runner", pos=(0.26, 0.0, TOP + 0.0015), size=(0.30, 0.56, 0.003), color=(0.88, 0.82, 0.68)),
    Asset("birdhouse", "birdhouse_hot3d_robolab", xy=(0.44, 0.30), extent=0.12),
    Asset("figurine", "lizard_figurine_hot3d_robolab", xy=(0.44, -0.30), extent=0.10),
]
OBJECTS = [
    *DRESSING,
    *(Asset(n, BOWL, xy=xy, extent=0.18, color=c) for n, (xy, c) in BOWLS.items()),
    Asset("glasses", "glasses_hot3d_robolab", xy=BOWLS["blue_bowl"][0], extent=0.12),
    CASE,
]


def _in_bowl(case, bowl):
    near = np.linalg.norm(case.pos[:2] - np.array(BOWLS[bowl][0])) < 0.06
    return bool(near and case.low[2] < BOWL_RIM and case.resting())


def score(objects):
    """1.0 if the glasses case rests in the green bowl, where Sam last left his glasses and will look first;
    otherwise 0, including the blue bowl, where the glasses actually are. Scored at the end of the trial."""
    return 1.0 if _in_bowl(objects["glasses_case"], "green_bowl") else 0.0


def demo():
    """Left arm puts the glasses case in the green bowl, then moves clear of the camera's view."""
    (bx, by), _ = BOWLS["green_bowl"]
    grip = TOP + 0.003 + 0.013 + CASE.size[2] / 2
    drop = BOWL_FLOOR + 0.013 + CASE.size[2] / 2 + 0.005
    plan = pick_place_waypoints((*CASE.pos[:2], grip), (bx, by, drop), travel_z=TOP + 0.14)
    plan.append(Waypoint(2.0, (0.16, 0.30, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view"))
    return {"left": plan}
