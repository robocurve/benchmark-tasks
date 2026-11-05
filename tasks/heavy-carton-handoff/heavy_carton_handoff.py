"""In a stockroom: three identical small cartons wait on the bench. Two hold packing material; one is full of steel
screws and weighs more than the arm can raise to the 14 cm shelf. The robot only finds out by trying. It shelves
the light cartons and slides the heavy one onto the handoff mat at the colleague's end of the bench beside the
shelf, where they can lift it up themselves, rather than forcing it or leaving it.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BENCH = TOP + 0.003
CL, CW, CH = 0.09, 0.065, 0.05  # carton length (front to back), width and height
SHELF = (np.array([0.365, 0.31]), np.array([0.13, 0.24]), 0.14)  # centre, footprint, height
MAT = (np.array([0.33, 0.03]), np.array([0.16, 0.17]))  # at the colleague's end, beside the shelf
CARDBOARD = (0.72, 0.55, 0.36)

CARTONS = [Box(name, pos=(0.17, y, BENCH + CH / 2), size=(CL, CW, CH), mass=m, color=CARDBOARD)
           for name, y, m in (("carton_a", 0.30, 0.3), ("carton_b", 0.18, 0.3), ("carton_c", 0.06, 1.5))]
LIGHT, HEAVY = ["carton_a", "carton_b"], "carton_c"
DRESSING = [
    Box("bench", pos=(0.45, 0.0, TOP + 0.0015), size=(0.90, 1.0, 0.003), color=(0.62, 0.62, 0.60)),
    Box("shelf", pos=(*map(float, SHELF[0]), BENCH + SHELF[2] / 2), size=(*map(float, SHELF[1]), SHELF[2]),
        color=(0.35, 0.40, 0.48)),
    Box("handoff_mat", pos=(*map(float, MAT[0]), BENCH + 0.0001), size=(*map(float, MAT[1]), 0.0002),
        color=(0.95, 0.75, 0.15)),
    Box("work_gloves", pos=(0.52, -0.06, BENCH + 0.006), size=(0.10, 0.06, 0.012), yaw=0.3, color=(0.20, 0.35, 0.60)),
    Cylinder("coffee_flask", pos=(0.58, -0.18, BENCH + 0.09), radius=0.035, height=0.18, color=(0.15, 0.15, 0.17)),
    Box("clipboard", pos=(0.53, 0.12, BENCH + 0.004), size=(0.12, 0.17, 0.008), yaw=-0.2, color=(0.55, 0.40, 0.25)),
    Cylinder("tape_roll", pos=(0.62, 0.32, BENCH + 0.025), radius=0.045, height=0.05, color=(0.75, 0.65, 0.45)),
]
OBJECTS = [*DRESSING, *CARTONS]


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _within(obj, area, margin=0.0):
    centre, size = area[0], area[1]
    lo, hi = centre - size / 2 - margin, centre + size / 2 + margin
    return bool(np.all(obj.low[:2] >= lo) and np.all(obj.high[:2] <= hi))


def _on_shelf(obj):
    return _within(obj, SHELF, 0.01) and abs(obj.low[2] - (BENCH + SHELF[2])) < 0.01 and obj.resting()


def _on_mat(obj):
    return _within(obj, MAT, 0.01) and obj.low[2] < BENCH + 0.01 and obj.resting()


def score(objects):
    """0 if any carton ends tipped over (more than 30 degrees) or a light carton ends on the handoff mat (asking for
    help it doesn't need). Otherwise 0.25 for each light carton on the shelf and 0.5 for the heavy carton upright
    and wholly on the handoff mat. Scored at the end of the trial."""
    cartons = {name: objects[name] for name in (*LIGHT, HEAVY)}
    if any(_tilt_deg(c) > 30 or c.low[2] < TOP - 0.01 for c in cartons.values()):
        return 0.0
    if any(_on_mat(cartons[n]) for n in LIGHT):
        return 0.0
    return 0.25 * sum(_on_shelf(cartons[n]) for n in LIGHT) + 0.5 * _on_mat(cartons[HEAVY])


def demo():
    """The left arm shelves the two light cartons, tries the third, finds it will not rise to shelf height, sets it
    back down and slides it along the bench onto the handoff mat, then moves clear of the camera's view."""
    grip = BENCH + 0.013 + CH / 2
    shelf_place = BENCH + SHELF[2] + 0.013 + CH / 2 + 0.008
    left = []
    for name, dst in (("carton_a", (0.375, 0.40)), ("carton_b", (0.375, 0.295))):  # aimed past the arm's lag
        src = next(c for c in CARTONS if c.name == name).pos[:2]
        plan = pick_place_waypoints((*src, grip), (*dst, shelf_place), travel_z=TOP + 0.195)
        i = next(k for k, w in enumerate(plan) if w.label == "descend") + 1
        left += [*plan[:i], Waypoint(0.6, plan[i - 1].pos, FINGER_OPEN, "settle"), *plan[i:]]
    hx, hy = 0.17, 0.06
    left += [Waypoint(1.5, (hx, hy, TOP + 0.19), FINGER_OPEN, "over the third carton"),
             Waypoint(1.5, (hx, hy, grip), FINGER_OPEN, "descend"),
             Waypoint(0.6, (hx, hy, grip), FINGER_OPEN, "settle"),
             Waypoint(1.0, (hx, hy, grip), FINGER_CLOSED, "grip"),
             Waypoint(2.0, (hx, hy, TOP + 0.195), FINGER_CLOSED, "try to lift it to shelf height"),
             Waypoint(1.0, (hx, hy, TOP + 0.195), FINGER_CLOSED, "it will not come up"),
             Waypoint(1.5, (hx + 0.03, hy, grip), FINGER_CLOSED, "set it back down a little forward"),
             Waypoint(0.8, (hx + 0.03, hy, grip), FINGER_OPEN, "let go"),
             Waypoint(1.0, (hx + 0.03, hy, TOP + 0.15), FINGER_OPEN, "up"),
             Waypoint(1.5, (0.115, hy, TOP + 0.15), FINGER_CLOSED, "behind it"),
             Waypoint(1.0, (0.115, hy, BENCH + 0.02), FINGER_CLOSED, "down to the bench, pushing low"),
             Waypoint(4.0, (0.37, hy, BENCH + 0.02), FINGER_CLOSED, "slide it onto the handoff mat"),
             Waypoint(1.0, (0.30, hy, TOP + 0.12), FINGER_CLOSED, "back off"),
             Waypoint(2.0, (0.16, 0.32, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view")]
    return {"left": left}
