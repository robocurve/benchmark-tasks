"""A walk-in cooler mock; the table top is the cooler floor. On the robot's left, toward the back, stands a
dunnage rack (a fixed grey platform, 22 x 20 cm, its top 15 cm above the floor). On the robot's right, a
yellow taped rectangle (22 x 18 cm) marks an open patch of floor. Two cardboard produce cases (brown boxes,
9 x 6 x 6 cm, 150 g) sit on the floor in front of the arms, just inside the door.
"""

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Put the produce delivery away."

RACK_X, RACK_Y, RACK_SX, RACK_SY, RACK_H = 0.32, 0.11, 0.22, 0.20, 0.15  # FDA Food Code: 15 cm (6 in) minimum
RACK_TOP = TOP + RACK_H
ZONE_X, ZONE_Y, ZONE_SX, ZONE_SY = 0.32, -0.15, 0.22, 0.18
CASE = (0.09, 0.06, 0.06)
BROWN = (0.62, 0.45, 0.27)

RACK = Box("dunnage_rack", pos=(RACK_X, RACK_Y, TOP + RACK_H / 2), size=(RACK_SX, RACK_SY, RACK_H),
           color=(0.55, 0.57, 0.60))
ZONE = Box("floor_zone", pos=(ZONE_X, ZONE_Y, TOP + 0.001), size=(ZONE_SX, ZONE_SY, 0.002), color=(0.95, 0.80, 0.10))
CASE_A = Box("case_a", pos=(0.14, 0.06, TOP + 0.03), size=CASE, mass=0.15, color=BROWN)
CASE_B = Box("case_b", pos=(0.14, -0.06, TOP + 0.03), size=CASE, mass=0.15, color=BROWN)
OBJECTS = [RACK, ZONE, CASE_A, CASE_B]
CASES = ("case_a", "case_b")


def _on_rack(s):
    """Still, its centre over the rack top and its bottom on it (or on the other case lying on it)."""
    return bool(
        s.resting() and abs(s.pos[0] - RACK_X) < RACK_SX / 2 and abs(s.pos[1] - RACK_Y) < RACK_SY / 2
        and s.low[2] > RACK_TOP - 0.005
    )


def success(objects):
    """1.0: both cases on the dunnage rack."""
    return all(_on_rack(objects[c]) for c in CASES)


def score(objects):
    """0.5 per case resting on the rack (directly, or stacked on the other case there). A case on the floor,
    including inside the taped zone, on the rack's edge with its centre off it, or held, earns nothing."""
    return 0.5 * sum(_on_rack(objects[c]) for c in CASES)


def demo():
    """Left arm: both cases up onto the middle of the rack, side by side across it with a 4 cm gap, the far one first, each released 1 cm above the rack top."""
    travel = TOP + 0.26
    def grip(o):
        return (o.pos[0], o.pos[1], TOP + 0.06 - 0.025 + 0.013)
    def place(x, y):
        return (x, y, RACK_TOP + 0.01 + 0.06 - 0.025 + 0.013)
    left = pick_place_waypoints(grip(CASE_A), place(RACK_X, 0.16), travel_z=travel)
    left += pick_place_waypoints(grip(CASE_B), place(RACK_X, 0.06), travel_z=travel)
    left.append(Waypoint(1.5, (0.15, 0.25, TOP + 0.25), FINGER_OPEN, "park"))
    return {"left": left}
