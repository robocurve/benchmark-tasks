"""A grocery checkout mock. On the robot's right is the belt end (a fixed dark-grey mat, 24 x 14 cm, 1 cm tall)
holding three groceries: a soup can (a red cylinder, 6 cm across, 8 cm tall), a cracker box (an orange box standing
upright, 6 x 4 x 9 cm) and a pasta box (a blue box lying flat, 10 x 4.5 x 4 cm). In the middle is the flatbed
barcode scanner: a dark glass window (fixed, 12 x 10 cm, 1 cm tall) with the scanner's grey upright tower behind it.
Past the scanner is a short stretch of bare counter, then on the robot's left the bagging area: an open paper bag
(four fixed brown walls, 9 cm tall, around an 18 x 16 cm inside, with the table as its floor).
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Check out the customer's items."

# Scanner window (x0, x1, y0, y1) and its top.
GLASS_X, GLASS_Y, GLASS_W, GLASS_D, GLASS_H = 0.27, -0.08, 0.12, 0.10, 0.01
GLASS = (GLASS_X - GLASS_W / 2, GLASS_X + GLASS_W / 2, GLASS_Y - GLASS_D / 2, GLASS_Y + GLASS_D / 2)
GLASS_TOP = TOP + GLASS_H
SCAN_HEIGHT = 0.06  # an item's bottom within 6 cm above the glass is in the scanner's read zone
SCAN_SPEED = 0.15  # and moving slower than 15 cm/s
SCAN_STEPS = 2  # for two control steps in a row

# Bag inside (x0, x1, y0, y1), wall height and thickness.
BAG_X, BAG_Y, BAG_IN_X, BAG_IN_Y, BAG_H, WALL = 0.28, 0.20, 0.18, 0.16, 0.09, 0.01
BAG = (BAG_X - BAG_IN_X / 2, BAG_X + BAG_IN_X / 2, BAG_Y - BAG_IN_Y / 2, BAG_Y + BAG_IN_Y / 2)
BAG_RIM = TOP + BAG_H

BELT_X, BELT_Y, BELT_TOP = 0.27, -0.23, TOP + 0.01
ITEMS = ("soup_can", "cracker_box", "pasta_box")

BROWN = (0.66, 0.50, 0.32)
OBJECTS = [
    Box("belt_end", pos=(BELT_X, BELT_Y, TOP + 0.005), size=(0.24, 0.14, 0.01), color=(0.18, 0.18, 0.18)),
    Box("scanner_glass", pos=(GLASS_X, GLASS_Y, TOP + GLASS_H / 2), size=(GLASS_W, GLASS_D, GLASS_H),
        color=(0.05, 0.08, 0.12)),
    Box("scanner_tower", pos=(GLASS_X + GLASS_W / 2 + 0.02, GLASS_Y, TOP + 0.05), size=(0.03, 0.12, 0.10),
        color=(0.55, 0.57, 0.60)),
    Box("bag_back", pos=(BAG[1] + WALL / 2, BAG_Y, TOP + BAG_H / 2), size=(WALL, BAG_IN_Y + 2 * WALL, BAG_H),
        color=BROWN),
    Box("bag_front", pos=(BAG[0] - WALL / 2, BAG_Y, TOP + BAG_H / 2), size=(WALL, BAG_IN_Y + 2 * WALL, BAG_H),
        color=BROWN),
    Box("bag_left", pos=(BAG_X, BAG[3] + WALL / 2, TOP + BAG_H / 2), size=(BAG_IN_X, WALL, BAG_H), color=BROWN),
    Box("bag_right", pos=(BAG_X, BAG[2] - WALL / 2, TOP + BAG_H / 2), size=(BAG_IN_X, WALL, BAG_H), color=BROWN),
    Cylinder("soup_can", pos=(0.18, BELT_Y, BELT_TOP + 0.04), radius=0.03, height=0.08, mass=0.2,
             color=(0.80, 0.10, 0.10)),
    Box("cracker_box", pos=(0.27, BELT_Y, BELT_TOP + 0.045), size=(0.06, 0.04, 0.09), mass=0.15,
        color=(0.95, 0.55, 0.10)),
    Box("pasta_box", pos=(0.36, BELT_Y, BELT_TOP + 0.02), size=(0.10, 0.045, 0.04), mass=0.15,
        color=(0.15, 0.35, 0.80)),
]
_START = {o.name: np.array(o.pos) for o in OBJECTS if o.mass is not None}


# --- What happened during the run. success() and score() are called after every control step, with no
# reset hook, so the run's history is kept here. A reset puts every free object back at its start, at rest, in one
# step; that jump is how a new trial is recognized. Objects moved by the arms travel a few cm per step at most.
class _History:
    def __init__(self):
        self.last = None  # free objects' positions at the previous call
        self.scan_run = {n: 0 for n in ITEMS}  # consecutive steps in the scanner's read zone
        self.scanned = set()  # items scanned so far
        self.unscanned_bagged = set()  # items that went into the bag before being scanned (never forgiven)


_H = _History()


def _in_read_zone(s):
    """Over the window, its bottom within 6 cm above the glass, moving slower than 15 cm/s."""
    return bool(
        GLASS[0] < s.pos[0] < GLASS[1] and GLASS[2] < s.pos[1] < GLASS[3]
        and GLASS_TOP - 0.005 < s.low[2] < GLASS_TOP + SCAN_HEIGHT
        and np.linalg.norm(s.lin_vel) < SCAN_SPEED
    )


def _in_bag(s):
    """Its center inside the bag's walls and below the rim."""
    return bool(BAG[0] < s.pos[0] < BAG[1] and BAG[2] < s.pos[1] < BAG[3] and s.pos[2] < BAG_RIM)


def _update(objects):
    global _H
    now = {n: np.asarray(objects[n].pos, dtype=float) for n in _START}
    if _H.last is not None:
        jumped = any(np.linalg.norm(now[n] - _H.last[n]) > 0.06 for n in now)
        at_start = all(np.linalg.norm(now[n] - _START[n]) < 0.015 for n in now)
        still = all(objects[n].resting() for n in now)
        if jumped and at_start and still:
            _H = _History()
    _H.last = now
    for n in ITEMS:
        s = objects[n]
        _H.scan_run[n] = _H.scan_run[n] + 1 if _in_read_zone(s) else 0
        if _H.scan_run[n] >= SCAN_STEPS:
            _H.scanned.add(n)
        if _in_bag(s) and n not in _H.scanned:
            _H.unscanned_bagged.add(n)


def _bagged(objects):
    return [n for n in ITEMS if _in_bag(objects[n]) and objects[n].resting()]


def success(objects):
    """1.0: all three items rest in the bag and every one was scanned before it went in. Called first each
    step, so it also records the step's history."""
    _update(objects)
    return not _H.unscanned_bagged and len(_bagged(objects)) == 3


def score(objects):
    """0 if any item ever went into the bag before it had been scanned, even if it was taken out again later.
    Otherwise one third per item resting in the bag. Possible scores: 0, 1/3, 2/3, 1."""
    if _H.unscanned_bagged:
        return 0.0
    return len(_bagged(objects)) / 3


def demo():
    """A cashier and a bagger. For each item in turn, the right arm lifts it off the belt, sweeps it across the
    scanner window about 2.5 cm above the glass at under 15 cm/s, and slides it on to the counter past the scanner.
    The left arm then lifts it from there into the bag. Nothing is set down on the glass."""

    def grip(top):  # TCP height that puts the finger pads 12 mm below an object's top
        return top - 0.012 + 0.013

    o, c = FINGER_OPEN, FINGER_CLOSED
    travel = TOP + 0.22
    scan_lift = 0.025  # item bottom above the glass during the sweep
    heights = {"soup_can": 0.08, "cracker_box": 0.09, "pasta_box": 0.04}
    belt_x = {"soup_can": 0.18, "cracker_box": 0.27, "pasta_box": 0.36}
    bag_spot = {"soup_can": (0.23, 0.16), "cracker_box": (0.24, 0.245), "pasta_box": (0.33, 0.16)}
    counter = (0.27, 0.045)  # bare counter just past the scanner (clear of its 1 cm lip), short of the bag
    sweep_in, sweep_out = GLASS[2] + 0.005, GLASS[3] - 0.005  # 9 cm across the window, along y
    right_park = (0.18, -0.24, TOP + 0.20)
    left_park = (0.18, 0.24, TOP + 0.24)

    def dur(ws):
        return sum(w.duration for w in ws)

    right, left = [], []
    counter_free_at = 0.0  # when the left arm has lifted the previous item clear of the counter
    for n in ITEMS:
        h = heights[n]
        bx = belt_x[n]
        z_scan = grip(GLASS_TOP + scan_lift + h)
        z_counter = grip(TOP + h) + 0.004
        fetch = [
            Waypoint(1.5, (bx, BELT_Y, travel), o, "to the item"),
            Waypoint(1.5, (bx, BELT_Y, grip(BELT_TOP + h)), o, "descend"),
            Waypoint(0.8, (bx, BELT_Y, grip(BELT_TOP + h)), c, "close"),
            Waypoint(1.2, (bx, BELT_Y, travel), c, "lift"),
            # Carry at full height, clear of the items still on the belt, then lower straight down beside the glass.
            Waypoint(1.5, (GLASS_X, sweep_in - 0.02, travel), c, "to the scanner"),
            Waypoint(1.2, (GLASS_X, sweep_in - 0.02, z_scan), c, "down beside the glass"),
        ]
        t_now = dur(right)
        # Hold at the scanner's near edge until the counter is clear.
        wait = max(0.0, counter_free_at - (t_now + dur(fetch)))
        sweep = [
            Waypoint(wait + 0.2, fetch[-1].pos, c, "wait"),
            Waypoint(1.4, (GLASS_X, sweep_out, z_scan), c, "sweep over the glass"),  # 9 cm in 1.4 s: peak ~10 cm/s
            Waypoint(1.0, (*counter, z_scan), c, "on past the scanner"),
            Waypoint(0.8, (*counter, z_counter), c, "set down on the counter"),
            Waypoint(0.6, (*counter, z_counter), o, "release"),
            Waypoint(1.0, (*counter, travel), o, "up"),
            Waypoint(1.2, right_park, o, "park"),
        ]
        right += fetch + sweep
        t_released = dur(right) - 2.2  # up + park still to come; the gripper is clear once it is up
        bx2, by2 = bag_spot[n]
        to_bag = pick_place_waypoints((*counter, grip(TOP + h)), (bx2, by2, grip(TOP + h) + 0.004), travel_z=TOP + 0.24)
        to_bag.append(Waypoint(1.2, left_park, o, "park"))
        start = max(dur(left), t_released + 1.0)
        if start > dur(left):
            left.append(Waypoint(start - dur(left), left[-1].pos if left else left_park, o, "wait"))
        left += to_bag
        counter_free_at = start + dur(to_bag[:4]) + 0.5  # to the item, descend, close, lift
    left.append(Waypoint(2.0, left_park, o, "hold while the bag settles"))
    return {"left": left, "right": right}
