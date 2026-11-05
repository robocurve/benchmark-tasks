"""On the station table, in front of the two arms: three round-bottom flasks sit in three cork rings in a row, each
flask in the wrong ring. A coloured tag in front of each ring says which flask belongs there. Round-bottom flasks are
never set down on the bench (they roll and crack), and every ring is taken, so one gripper has to hold a flask in
the air while the other moves the rest round: no flask may touch the bench at any time.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- rings: three square cork rings, fixed, each four cork bars 1.2 cm wide and 2.5 cm tall around a 5 x 5 cm opening
  (7.4 cm square outside), centred at x 0.26 and y 0.11 (left), 0 (middle) and -0.11 (right). A flask in a ring
  rests on the four inner edges, 1.6 cm clear of the bench.
- tags: coloured tape (fixed, 1 mm thick, 5 cm x 1.2 cm) on the bench just in front of each ring: red for the left
  ring, green for the middle ring, blue for the right ring.
- flasks: three glass round-bottom flasks, modelled as balls 8 cm across, 150 g each, tinted red, green and blue. At
  the start the blue flask is in the left ring, the red one in the middle ring and the green one in the right ring.

Full score: every flask rests in the ring tagged with its colour, everything still for 2 s, and no flask touched the
bench at any time.
"""

import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0

CORK = (0.70, 0.53, 0.34)
RED, GREEN, BLUE = (0.80, 0.16, 0.14), (0.16, 0.60, 0.28), (0.16, 0.34, 0.82)

RING_X = 0.26
RING_Y = {"left": 0.11, "middle": 0.0, "right": -0.11}
TAG = {"left": ("red", RED), "middle": ("green", GREEN), "right": ("blue", BLUE)}
HOLE = 0.05  # opening, square
BAR_W, BAR_H = 0.012, 0.025
FLASK_R, FLASK_KG = 0.04, 0.15
SEAT_Z = TOP + BAR_H + float(np.sqrt(FLASK_R**2 - (HOLE / 2) ** 2))  # flask centre resting on the four inner edges
START = {"blue": "left", "red": "middle", "green": "right"}  # flask colour -> ring at the start
GOAL = {colour: ring for ring, (colour, _) in TAG.items()}  # flask colour -> its own ring
COLOUR = {"red": RED, "green": GREEN, "blue": BLUE}


def _ring(name: str, cy: float):
    off = HOLE / 2 + BAR_W / 2
    outer = HOLE + 2 * BAR_W
    z = TOP + BAR_H / 2
    return [
        Box(f"ring_{name}_front", pos=(RING_X - off, cy, z), size=(BAR_W, outer, BAR_H), color=CORK),
        Box(f"ring_{name}_back", pos=(RING_X + off, cy, z), size=(BAR_W, outer, BAR_H), color=CORK),
        Box(f"ring_{name}_left", pos=(RING_X, cy + off, z), size=(HOLE, BAR_W, BAR_H), color=CORK),
        Box(f"ring_{name}_right", pos=(RING_X, cy - off, z), size=(HOLE, BAR_W, BAR_H), color=CORK),
    ]


OBJECTS = []
for _name, _cy in RING_Y.items():
    OBJECTS += _ring(_name, _cy)
    OBJECTS.append(Box(f"tag_{_name}", pos=(RING_X - HOLE / 2 - BAR_W - 0.012, _cy, TOP + 0.0005),
                       size=(0.012, 0.05, 0.001), color=TAG[_name][1]))
for _colour, _ring_name in START.items():
    OBJECTS.append(Sphere(f"flask_{_colour}", pos=(RING_X, RING_Y[_ring_name], SEAT_Z + 0.001), radius=FLASK_R,
                          mass=FLASK_KG, color=COLOUR[_colour]))

# Thresholds, as a reviewer would check them from the video and with a ruler.
IN_RING_XY = 0.012  # a flask is in a ring: its centre within 12 mm of the ring's centre ...
IN_RING_Z = 0.006  # ... and resting on the ring's edges (centre within 6 mm of the seated height)
BENCH = 0.003  # "at any time": a flask's lowest point within 3 mm of the bench scores 0

_DEBUG = os.environ.get("SF_DEBUG") == "1"
_state = {"bench": False, "step": 0, "low": 1.0}


def _latch(objects) -> None:
    for colour in COLOUR:
        # lowest point of the ball (its world-aligned box grows when the ball turns, so use the centre)
        low = float(objects[f"flask_{colour}"].pos[2]) - FLASK_R - TOP
        _state["low"] = min(_state["low"], low)
        if low < BENCH:
            _state["bench"] = True


def _ring_of(flask):
    """The ring a flask rests in, or None."""
    for name, cy in RING_Y.items():
        if (np.hypot(flask.pos[0] - RING_X, flask.pos[1] - cy) <= IN_RING_XY
                and abs(flask.pos[2] - SEAT_Z) <= IN_RING_Z):
            return name
    return None


def _placed(objects) -> int:
    """How many flasks rest in the ring tagged with their colour."""
    return sum(_ring_of(objects[f"flask_{c}"]) == GOAL[c] for c in COLOUR)


def success(objects):
    _latch(objects)
    n = _placed(objects)
    if _DEBUG:
        _state["step"] += 1
        if _state["step"] % 10 == 1:
            where = {c: _ring_of(objects[f"flask_{c}"]) for c in COLOUR}
            zs = {c: round(float(objects[f"flask_{c}"].pos[2] - TOP), 3) for c in COLOUR}
            print(f"DBG step={_state['step']} placed={n} where={where} z={zs} "
                  f"min_low={_state['low'] * 1000:.1f}mm bench={_state['bench']}", flush=True)
    still = all(objects[f"flask_{c}"].resting() for c in COLOUR)
    return n == 3 and still and not _state["bench"]


def distance(objects):
    d = 0.0
    for c in COLOUR:
        f = objects[f"flask_{c}"]
        d += float(np.hypot(f.pos[0] - RING_X, f.pos[1] - RING_Y[GOAL[c]])) + abs(float(f.pos[2]) - SEAT_Z)
    return d


def score(objects):
    """Partial credit (success scores 1.0): 0.25 for each flask resting in the ring tagged with its colour (so 0.5
    with two of them, the third still in a gripper); 0 if any flask touched the bench at any time."""
    _latch(objects)
    if _state["bench"]:
        return 0.0
    return 0.25 * min(_placed(objects), 2)


# -- scripted demo -------------------------------------------------------------------------------------------------
# Left arm: lifts the blue flask out of the left ring and holds it, parked back on its own side (the only free place
# for it). Right arm: moves the red flask from the middle ring into the left ring, then the green flask from the right
# ring into the middle ring, and parks. Left arm: carries the blue flask round behind the row and lowers it into the
# right ring. Fingers close front to back (gripper turned a quarter turn), so open fingers never reach toward the
# neighbouring flasks. Moves between empty rings stay low (the flask's bottom 4 cm up, clear of the cork bars).
TIP = 0.013
GRIP_Z = SEAT_Z - 0.016 + TIP  # pad tips 1.6 cm below the flask's centre
HIGH_Z = (TOP + 0.10 + FLASK_R) - 0.016 + TIP  # grip height with the flask's bottom 10 cm above the bench
LOW_Z = (TOP + 0.04 + FLASK_R) - 0.016 + TIP  # ... 4 cm above the bench
SET_Z = GRIP_Z + float(os.environ.get("SF_SET_GAP", "0.002"))  # let go with the flask 2 mm above its seat
PARK = {"left": (0.15, 0.28), "right": (0.12, -0.30)}
BEHIND_X = 0.15  # a path behind the row: a flask here clears the flasks in the rings
YAW = {"left": -np.pi / 2, "right": np.pi / 2}


def _xy(ring):
    return RING_X, RING_Y[ring]


def _take(arm, ring, wps, carry_z):
    y = YAW[arm]
    wps += [
        Waypoint(2.0, (*_xy(ring), carry_z), FINGER_OPEN, f"above the {ring} ring", y),
        Waypoint(1.5, (*_xy(ring), GRIP_Z), FINGER_OPEN, "down", y),
        Waypoint(1.0, (*_xy(ring), GRIP_Z), FINGER_CLOSED, "grasp", y),
        Waypoint(1.5, (*_xy(ring), carry_z), FINGER_CLOSED, "lift", y),
    ]


def _put(arm, ring, wps, carry_z, travel_s=2.0, rise_z=None):
    """Lower the held flask into `ring` and let go; then rise to rise_z (default carry_z). The open fingers sit in
    front of and behind the flask, so the gripper may then slide sideways (along y) but not back until it is above
    the flask."""
    y = YAW[arm]
    wps += [
        Waypoint(travel_s, (*_xy(ring), carry_z), FINGER_CLOSED, f"over the {ring} ring", y),
        Waypoint(1.5, (*_xy(ring), SET_Z), FINGER_CLOSED, "lower", y),
        Waypoint(1.0, (*_xy(ring), SET_Z), FINGER_OPEN, "let go", y),
        Waypoint(1.5, (*_xy(ring), carry_z if rise_z is None else rise_z), FINGER_OPEN, "rise", y),
    ]


def demo():
    left, right = [], []
    yl, yr = YAW["left"], YAW["right"]
    # left arm: take the blue flask out of the left ring and park with it
    _take("left", "left", left, HIGH_Z)
    left.append(Waypoint(2.0, (*PARK["left"], HIGH_Z), FINGER_CLOSED, "park holding the blue flask", yl))
    t_left_parked = sum(w.duration for w in left)
    # right arm: wait parked until the left ring is clear, then red -> left ring, green -> middle ring, park
    right.append(Waypoint(2.0, (*PARK["right"], HIGH_Z), FINGER_OPEN, "park", yr))
    right.append(Waypoint(t_left_parked - 2.0, (*PARK["right"], HIGH_Z), FINGER_OPEN, "wait", yr))
    right.append(Waypoint(2.0, (*_xy("middle"), HIGH_Z), FINGER_OPEN, "high over the row", yr))  # clear of the flasks
    _take("right", "middle", right, LOW_Z)
    _put("right", "left", right, LOW_Z)  # then slides sideways to the right ring, fingers clear of the flasks
    _take("right", "right", right, LOW_Z)
    _put("right", "middle", right, LOW_Z, rise_z=HIGH_Z)  # straight up above the flask before going back
    right.append(Waypoint(2.0, (*PARK["right"], HIGH_Z), FINGER_OPEN, "park", yr))
    t_right_parked = sum(w.duration for w in right)
    # left arm: hold until the right arm has parked, then round behind the row and blue -> right ring
    left.append(Waypoint(t_right_parked - t_left_parked, (*PARK["left"], HIGH_Z), FINGER_CLOSED, "hold", yl))
    left.append(Waypoint(1.5, (BEHIND_X, PARK["left"][1], LOW_Z), FINGER_CLOSED, "down behind the row", yl))
    left.append(Waypoint(3.0, (BEHIND_X, RING_Y["right"], LOW_Z), FINGER_CLOSED, "along behind the row", yl))
    _put("left", "right", left, LOW_Z, travel_s=1.5, rise_z=SET_Z + 0.008)
    # slide sideways to the middle ring (the open fingers pass in front of and behind the flasks), then straight up
    left.append(Waypoint(2.0, (*_xy("middle"), SET_Z + 0.008), FINGER_OPEN, "slide out sideways", yl))
    left.append(Waypoint(1.5, (*_xy("middle"), HIGH_Z), FINGER_OPEN, "up", yl))
    left.append(Waypoint(2.5, (*PARK["left"], HIGH_Z), FINGER_OPEN, "park", yl))
    right.append(Waypoint(sum(w.duration for w in left) - t_right_parked, (*PARK["right"], HIGH_Z), FINGER_OPEN,
                          "wait", yr))
    return {"left": left, "right": right}
