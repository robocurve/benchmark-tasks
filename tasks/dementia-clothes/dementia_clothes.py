"""On the station table: a pale blue bed lying across the table between the arms. Four folded clothes lie on the table
beside it, two at each end: a white vest, a blue shirt, a green jumper and a black coat. All four are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=dementia_clothes.py --scorer task_score \\
           --instruction "Mr Lee has dementia. Lay out his black coat, white vest, green jumper and blue shirt on the bed so he can dress himself." ...
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BED_H = 0.002
BED_CENTRE = (0.25, 0.0)
BED_SIZE = (0.16, 0.38)
BED = Box("bed", pos=(*BED_CENTRE, TOP + BED_H / 2), size=(*BED_SIZE, BED_H), color=(0.72, 0.82, 0.92))

FOLDED = (0.10, 0.06, 0.03)  # x, y, z of each folded item (m); the narrow side is across the grip


def _clothes(name, x, y, color):
    return Box(name, pos=(x, y, TOP + FOLDED[2] / 2), size=FOLDED, mass=0.1, color=color)


VEST = _clothes("vest", 0.18, 0.27, (0.95, 0.95, 0.92))
SHIRT = _clothes("shirt", 0.32, 0.27, (0.15, 0.30, 0.85))
JUMPER = _clothes("jumper", 0.32, -0.27, (0.12, 0.55, 0.25))
COAT = _clothes("coat", 0.18, -0.27, (0.08, 0.08, 0.09))
OBJECTS = [BED, VEST, SHIRT, JUMPER, COAT]

ORDER = ("vest", "shirt", "jumper", "coat")  # the order they go on, innermost first
ROW_X_TOLERANCE = 0.05  # items must lie within this of the bed's centre line along x (m)
MIN_GAP = 0.07  # neighbouring items' centres must be at least this far apart along the row (m)
TARGETS = {"vest": 0.135, "shirt": 0.045, "jumper": -0.045, "coat": -0.135}


def _on_bed(item):
    """Lying flat on the bed (not on another item), still, within the row."""
    over = abs(item.pos[1] - BED_CENTRE[1]) < BED_SIZE[1] / 2 and abs(item.pos[0] - BED_CENTRE[0]) < ROW_X_TOLERANCE
    return bool(over and abs(item.low[2] - (TOP + BED_H)) < 0.005 and item.resting())


def _pairs_in_order(objects):
    """The longest unbroken run of neighbouring pairs (vest-shirt, shirt-jumper, jumper-coat) in order along the row,
    in either direction."""
    best = 0
    for direction in (1.0, -1.0):
        run = 0
        for first, second in zip(ORDER, ORDER[1:]):
            a, b = objects[first], objects[second]
            run = run + 1 if _on_bed(a) and _on_bed(b) and direction * (b.pos[1] - a.pos[1]) >= MIN_GAP else 0
            best = max(best, run)
    return best


def success(objects):
    return all(_on_bed(objects[name]) for name in ORDER) and _pairs_in_order(objects) == 3


def distance(objects):
    return float(max(abs(objects[name].pos[1] - TARGETS[name]) for name in ORDER))


def score(objects):
    """Partial credit (success scores 1.0): 0.1 for each item on the bed, 0.2 for each pair in the longest unbroken
    run of neighbouring pairs in order."""
    return 0.1 * sum(_on_bed(objects[name]) for name in ORDER) + 0.2 * _pairs_in_order(objects)


GRIP_Z = TOP + 0.02  # the lowest grip height proven on this table
TRAVEL_Z = TOP + 0.14
CYCLE_S = 12.0  # each arm spends this long per item
LEFT_PARK = (0.20, 0.30, TRAVEL_Z)
RIGHT_PARK = (0.20, -0.30, TRAVEL_Z)


def _cycle(item, target_y, park):
    """One pick and place, then a move clear of the bed, padded to CYCLE_S so the arms stay in step."""
    pick = (item.pos[0], item.pos[1], GRIP_Z)
    place = (BED_CENTRE[0], target_y, GRIP_Z + BED_H + 0.005)
    steps = pick_place_waypoints(pick, place, travel_z=TRAVEL_Z)
    steps.append(Waypoint(1.5, park, FINGER_OPEN, "clear"))
    steps.append(Waypoint(CYCLE_S - sum(w.duration for w in steps), park, FINGER_OPEN, "wait"))
    return steps


def demo():
    """Both arms lay the outer two items at once; then the left arm lays the shirt and, once it is clear, the right
    arm lays the jumper, so the arms never work side by side at the middle of the bed."""
    left = _cycle(VEST, TARGETS["vest"], LEFT_PARK) + _cycle(SHIRT, TARGETS["shirt"], LEFT_PARK)
    right = _cycle(COAT, TARGETS["coat"], RIGHT_PARK)
    right.append(Waypoint(CYCLE_S, RIGHT_PARK, FINGER_OPEN, "wait"))
    right += _cycle(JUMPER, TARGETS["jumper"], RIGHT_PARK)
    return {"left": left, "right": right}
