"""On the station table, in front of the two arms: a loose wooden plank rests across a narrow steel pivot ridge, level
and balanced, like the beam of a balance. Four cylinder weights of the same size stand on felt pads around the table
in no particular pattern: two red ones of 400 g and two yellow ones of 250 g. All four have to end up on the plank,
one of each colour on each side: the red weights near the two ends, the yellow ones closer in. The plank holds level
only while the centre of everything on it stays within about 1.2 cm of the ridge, so the weights can only go on in
balanced pairs, set down at the same moment and at matching distances: the two red ones together, and the two yellow
ones together. Any weight on its own, or a red one set down opposite a yellow one at the same distance, tips it.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- pivot: fixed dark steel ridge, 12 cm front to back x 2.4 cm side to side x 5 cm tall, centred at (0.30, 0), its
  flat top level and 2.4 cm wide.
- plank: pine, 9 cm front to back x 48 cm side to side x 1.8 cm thick, 300 g, lying across the ridge, centred on it
  at (0.30, 0), level (its underside on the ridge top, 5 cm above the table). Tipped until an end touches the table
  it would lean about 13 degrees.
- weights: cylinders 5 cm across x 6 cm tall, each standing on a dark green felt pad 8 x 8 cm (fixed, 1 mm
  thick): red, 400 g, at (0.12, 0.27) and (0.17, -0.16); yellow, 250 g, at (0.17, 0.14) and (0.11, -0.27).
- zones (no marks on the plank; measured along it from its centre, each within 2.5 cm of its centre line): the end
  zones, 16-22 cm out on each side (the plank's ends are 24 cm out), for the red weights; the inner zones, 8-13 cm
  out on each side, for the yellow weights.

Full score: a red weight in each end zone and a yellow weight in each inner zone, all standing on the plank, the plank
level within 2 degrees and on the ridge, everything still for 2 s; and the plank never tilted more than 6 degrees or
left the ridge on the way.
"""

import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0

PINE = (0.86, 0.72, 0.50)
STEEL = (0.25, 0.27, 0.30)
RED = (0.78, 0.14, 0.12)
YELLOW = (0.96, 0.80, 0.12)
FELT = (0.10, 0.30, 0.18)

PIVOT_X = 0.30
PIVOT = (0.12, 0.024, 0.05)  # x, y, z
PLANK = (0.09, 0.48, 0.018)
PLANK_KG = 0.30
RIDGE_TOP = TOP + PIVOT[2]
PLANK_Z = RIDGE_TOP + PLANK[2] / 2
W_R, W_H = 0.025, 0.06
W_KG = {"red": 0.40, "yellow": 0.25}
W_COLOR = {"red": RED, "yellow": YELLOW}
# weight name -> (colour, start position on the table)
WEIGHTS = {
    "red_a": ("red", (0.12, 0.27)),
    "red_b": ("red", (0.17, -0.16)),
    "yellow_a": ("yellow", (0.17, 0.14)),
    "yellow_b": ("yellow", (0.11, -0.27)),
}

OBJECTS = [
    Box("pivot", pos=(PIVOT_X, 0.0, TOP + PIVOT[2] / 2), size=PIVOT, color=STEEL),
    Box("plank", pos=(PIVOT_X, 0.0, PLANK_Z), size=PLANK, mass=PLANK_KG, color=PINE),
]
for _name, (_colour, _xy) in WEIGHTS.items():
    OBJECTS.append(Box(f"pad_{_name}", pos=(*_xy, TOP + 0.0005), size=(0.08, 0.08, 0.001), color=FELT))
    OBJECTS.append(Cylinder(_name, pos=(*_xy, TOP + 0.001 + W_H / 2), radius=W_R, height=W_H, mass=W_KG[_colour],
                            color=W_COLOR[_colour]))

# Thresholds, as a reviewer would check them with a ruler and a phone inclinometer.
ZONE = {"red": (0.16, 0.22), "yellow": (0.08, 0.13)}  # a weight's centre this far from the plank's centre, along it
ZONE_SIDE = 0.025  # and within this of the plank's centre line
LEVEL_TOL = np.radians(2.0)
TIP_MAX = np.radians(6.0)  # "at any time": tilting past this (an end dropping about 2.5 cm) scores 0
OFF_RIDGE = 0.02  # "at any time": the plank's centre more than 2 cm from the ridge, or lifted 1 cm off it, scores 0
ON_PLANK = 0.006  # a weight stands on the plank: its underside within 6 mm of the plank's top face

_DEBUG = os.environ.get("BP_DEBUG") == "1"
_state = {"tipped": False, "off": False, "max_tilt": 0.0, "step": 0}


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = quat_xyzw
    return float(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))


def _yaw(quat_xyzw) -> float:
    x, y, z, w = quat_xyzw
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _latch(plank) -> None:
    """Record, for the rest of the trial, a tip past 6 degrees or the plank leaving the ridge."""
    tilt = _tilt(plank.quat_xyzw)
    _state["max_tilt"] = max(_state["max_tilt"], tilt)
    if tilt > TIP_MAX:
        _state["tipped"] = True
    centre_off = np.hypot(plank.pos[0] - PIVOT_X, plank.pos[1])
    lifted = plank.pos[2] - PLANK_Z > 0.01 and tilt < TIP_MAX  # raised clear of the ridge, not just tilted
    if centre_off > OFF_RIDGE or lifted:
        _state["off"] = True


def _in_plank_frame(weight, plank):
    """(along, across, height above the plank's top) of a weight's centre/underside, in the plank's own frame."""
    yaw = _yaw(plank.quat_xyzw)
    d = np.asarray(weight.pos[:2]) - np.asarray(plank.pos[:2])
    along = float(-np.sin(yaw) * d[0] + np.cos(yaw) * d[1])  # along the plank's long (y) axis, + to the left
    across = float(np.cos(yaw) * d[0] + np.sin(yaw) * d[1])
    plank_top = plank.pos[2] + PLANK[2] / 2
    return along, across, float(weight.low[2] - plank_top)


def _pair_placed(objects, colour) -> bool:
    """Both weights of this colour stand on the plank in their zones, one on each side."""
    plank = objects["plank"]
    lo, hi = ZONE[colour]
    sides = []
    for name, (c, _) in WEIGHTS.items():
        if c != colour:
            continue
        along, across, gap = _in_plank_frame(objects[name], plank)
        if not (abs(gap) <= ON_PLANK and lo <= abs(along) <= hi and abs(across) <= ZONE_SIDE):
            return False
        sides.append(np.sign(along))
    return sorted(sides) == [-1.0, 1.0]


def _level(plank) -> bool:
    return bool(_tilt(plank.quat_xyzw) <= LEVEL_TOL and abs(plank.pos[2] - PLANK_Z) < 0.005)


def success(objects):
    plank = objects["plank"]
    _latch(plank)
    red, yellow, level = _pair_placed(objects, "red"), _pair_placed(objects, "yellow"), _level(plank)
    if _DEBUG:
        _state["step"] += 1
        if _state["step"] % 10 == 1:
            print(f"DBG step={_state['step']} tilt={np.degrees(_tilt(plank.quat_xyzw)):.2f} "
                  f"max={np.degrees(_state['max_tilt']):.2f} tipped={_state['tipped']} off={_state['off']} "
                  f"red={red} yellow={yellow} level={level} z={plank.pos[2] - PLANK_Z:+.4f}", flush=True)
    still = all(objects[n].resting() for n in ("plank", *WEIGHTS))
    return red and yellow and level and still and not _state["tipped"] and not _state["off"]


def distance(objects):
    plank = objects["plank"]
    d = 0.0
    for name, (colour, _) in WEIGHTS.items():
        along, across, gap = _in_plank_frame(objects[name], plank)
        d += abs(abs(along) - sum(ZONE[colour]) / 2) + abs(across) + abs(gap)
    return float(d)


def score(objects):
    """Partial credit (success scores 1.0): 0.5 when one pair is in place (the red weights in the end zones, or the
    yellow weights in the inner zones, one on each side) with the plank level and on the ridge, but not both pairs;
    0 if the plank ever tilted more than 6 degrees or left the ridge, and otherwise."""
    plank = objects["plank"]
    _latch(plank)
    if _state["tipped"] or _state["off"]:
        return 0.0
    one_pair = _pair_placed(objects, "red") or _pair_placed(objects, "yellow")
    return 0.5 if one_pair and _level(plank) else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# Two rounds. Each arm picks up the red weight on its own side, carries it above its end zone, and both arms lower
# the red weights onto the plank on the same timeline, let go together and rise. Then the same with the yellow
# weights and the inner zones. If one weight touches a moment early, its end dips only until the plank's other end
# meets the other weight, still held just above. Fingers close front to back (gripper turned a quarter turn), so
# the open fingers stay clear of the weights already on the plank.
TIP = 0.013  # pad tips below the grip point
GRIP_ABOVE_BASE = W_H - 0.035 + TIP  # grip point above a weight's underside (pad tips 3.5 cm below its top)
CARRY_LIFT = 0.10  # weight underside height above the table while carrying (the plank's top is at 6.8 cm)
SET_GAP = float(os.environ.get("BP_SET_GAP", "0.002"))  # let go with the weight's underside this far above the plank
TARGET = {"red": float(os.environ.get("BP_TARGET_RED", "0.19")),
          "yellow": float(os.environ.get("BP_TARGET_YELLOW", "0.105"))}
LOWER_S = float(os.environ.get("BP_LOWER_S", "2.5"))
YAW = {"left": -np.pi / 2, "right": np.pi / 2}
ROUNDS = (("red", {"left": "red_a", "right": "red_b"}), ("yellow", {"left": "yellow_a", "right": "yellow_b"}))


def demo():
    plank_top = RIDGE_TOP + PLANK[2]
    grip_z = TOP + 0.001 + GRIP_ABOVE_BASE
    z_carry = TOP + CARRY_LIFT + GRIP_ABOVE_BASE
    z_set = plank_top + SET_GAP + GRIP_ABOVE_BASE
    z_hover = z_carry + 0.02  # open fingertips 1.7 cm above the top of a weight standing on the plank
    plans = {"left": [], "right": []}
    for colour, by_arm in ROUNDS:
        for arm, side in (("left", 1.0), ("right", -1.0)):
            sx, sy = WEIGHTS[by_arm[arm]][1]
            tx, ty = PIVOT_X, side * TARGET[colour]
            y = YAW[arm]
            plans[arm] += [
                Waypoint(2.0, (sx, sy, z_hover), FINGER_OPEN, f"above the {colour} weight", y),
                Waypoint(1.5, (sx, sy, grip_z), FINGER_OPEN, "down", y),
                Waypoint(1.0, (sx, sy, grip_z), FINGER_CLOSED, "grasp", y),
                Waypoint(1.5, (sx, sy, z_carry), FINGER_CLOSED, "lift", y),
                Waypoint(2.0, (tx, ty, z_carry), FINGER_CLOSED, f"carry over the {colour} zone", y),
                Waypoint(0.5, (tx, ty, z_carry), FINGER_CLOSED, "steady", y),
                Waypoint(LOWER_S, (tx, ty, z_set), FINGER_CLOSED, "lower together", y),
                Waypoint(0.5, (tx, ty, z_set), FINGER_CLOSED, "settle", y),
                Waypoint(1.0, (tx, ty, z_set), FINGER_OPEN, "let go together", y),
                Waypoint(1.5, (tx, ty, z_hover), FINGER_OPEN, "rise", y),
            ]
    return plans
