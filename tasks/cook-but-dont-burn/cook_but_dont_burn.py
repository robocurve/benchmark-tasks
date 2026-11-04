"""On the station table: a round red hot plate, 18 cm wide and 2 cm high, fixed in front of the left arm. A small
grey steel pot, 7 cm wide and 6 cm high, stands on the table to the left of the hot plate.

The pot cooks while it sits on the hot plate. It must cook for 15 seconds in total, then go back onto the table,
clear of the hot plate. After more than 18 seconds in total on the hot plate, the food burns.

Score (task_score), on the final state: 0 if the pot was on the hot plate for more than 18 s in total, if it tipped
over or fell off the table at any time, or if it is not standing still on the table, clear of the hot plate. Otherwise the time on the hot plate divided by 14 s,
up to 1.0. The episode ends with 1.0 when the pot has stood clear of the hot plate for 1 s after 14 to 18 s of
cooking.

Run it:  inspect-robots run --embodiment yam_arena -E task=cook_but_dont_burn.py -E control_hz=20 \\
           --instruction "Cook the grey pot on the red hot plate for 15 seconds, then move it off onto the table. ..." \\
           --scorer task_score ...
"""

import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# ---- geometry (m) ----
PLATE_XY, PLATE_R, PLATE_H = (0.30, 0.15), 0.09, 0.02  # hot plate: centre, radius, height
POT_XY, POT_R, POT_H = (0.18, 0.36), 0.035, 0.06  # pot start: centre, radius, height
POT_MASS = 0.30  # steel pot with food
PLATE_TOP = TOP + PLATE_H

# ---- thresholds ----
FULL_S = 14.0  # seconds on the hot plate for full credit
BURN_S = 18.0  # more than this in total burns the food: score 0
ON_DZ = 0.01  # on the hot plate: the pot bottom at most this far above the plate top
TABLE_DZ = 0.01  # set down: the pot bottom within this distance of the table top; fallen: below it by more
MAX_TILT_DEG = 20.0  # spilled: the pot axis more than this from vertical, at any time

HOT_PLATE = Cylinder(
    "hot_plate", pos=(*PLATE_XY, TOP + PLATE_H / 2), radius=PLATE_R, height=PLATE_H, color=(0.75, 0.08, 0.06)
)
POT = Cylinder(
    "pot", pos=(*POT_XY, TOP + POT_H / 2), radius=POT_R, height=POT_H, mass=POT_MASS, color=(0.62, 0.64, 0.66)
)
OBJECTS = [HOT_PLATE, POT]


# ---- simulation clock ----
def _sim_time() -> float:
    """Seconds since the episode's reset, read (not changed) from the yam_arena embodiment that calls the hooks.
    The hooks get no clock of their own; this one is exact at any -E control_hz and goes back to 0 at each reset."""
    frame = sys._getframe(1)
    while frame is not None:
        owner = frame.f_locals.get("self")
        if type(owner).__name__ == "YamArenaEmbodiment":
            return float(owner._t)
        frame = frame.f_back
    raise RuntimeError("cook_but_dont_burn: no YamArenaEmbodiment on the call stack to read the clock from")


clock = _sim_time  # the offline tests replace it

# Cooking time and spills are a history, so the module keeps them between control steps. Both hooks call _update()
# every step; the clock makes the second call a no-op and shows each new episode (it goes back to 0).
_state = {"t": None, "on_s": 0.0, "spilled": False}


def _update(objects) -> None:
    t, prev = clock(), _state["t"]
    if prev is not None and t == prev:
        return
    if prev is None or t < prev:
        _state.update(on_s=0.0, spilled=False)
        prev = 0.0
    pot = objects["pot"]
    if on_plate(pot):
        _state["on_s"] += t - prev
    if not upright(pot) or pot.low[2] < TOP - TABLE_DZ:
        _state["spilled"] = True
    _state["t"] = t


# ---- task quantities ----
def _from_plate(pot) -> float:
    return float(np.hypot(pot.pos[0] - PLATE_XY[0], pot.pos[1] - PLATE_XY[1]))


def on_plate(pot) -> bool:
    """The pot's centre is over the hot plate and its bottom is on the plate top (not held above it)."""
    return bool(_from_plate(pot) <= PLATE_R and PLATE_TOP - 0.005 <= pot.low[2] <= PLATE_TOP + ON_DZ)


def upright(pot) -> bool:
    x, y, _, _ = pot.quat_xyzw
    return bool(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))) <= MAX_TILT_DEG)


def set_down(pot) -> bool:
    """The pot stands still on the table (not held up), with no part of it over the hot plate."""
    return bool(_from_plate(pot) > PLATE_R + POT_R and abs(pot.low[2] - TOP) <= TABLE_DZ and pot.resting())


def seconds_on_plate() -> float:
    return _state["on_s"]


def spilled() -> bool:
    """The pot tipped over or fell off the table at some time in this episode. It stays spilled."""
    return _state["spilled"]


def burnt() -> bool:
    return seconds_on_plate() > BURN_S + 1e-6  # the tolerance absorbs float drift in the summed step times


# ---- hooks read by yam_arena (task_score) ----
def success(objects):
    """Score 1.0: 14 to 18 s of cooking, no spill, and the pot is set down clear of the hot plate."""
    _update(objects)
    return not (burnt() or spilled()) and seconds_on_plate() >= FULL_S and set_down(objects["pot"])


def score(objects):
    """0 if burnt, spilled or not set down clear of the hot plate; otherwise the time on the plate over FULL_S."""
    _update(objects)
    if burnt() or spilled() or not set_down(objects["pot"]):
        return 0.0
    return min(seconds_on_plate() / FULL_S, 1.0)


# ---- scripted demo ----
GRASP_DZ = PAD_TIP_PAST_TCP + 0.02  # TCP above the pot bottom: the pad tips 2 cm above it
TRAVEL_Z = TOP + 0.14  # the pot bottom 10.7 cm above the table while carried
# Aim for the middle of the full-credit band. The pot also rests on the plate at the end of "lower" and the start
# of "lift", about 0.6 s in the offline replay, which the wait leaves out.
DEMO_ON_S = (FULL_S + BURN_S) / 2 - 0.5


def _put(xy, floor_z, label):
    """Carry the gripped pot to above ``xy``, lower it onto ``floor_z``, let go and rise."""
    return [
        Waypoint(2.0, (*xy, TRAVEL_Z), FINGER_CLOSED, f"carry to {label}"),
        Waypoint(0.5, (*xy, TRAVEL_Z), FINGER_CLOSED, "settle"),
        Waypoint(2.0, (*xy, floor_z + 0.003 + GRASP_DZ), FINGER_CLOSED, f"lower onto {label}"),
        Waypoint(0.8, (*xy, floor_z + 0.003 + GRASP_DZ), FINGER_OPEN, "release"),
        Waypoint(1.0, (*xy, TRAVEL_Z), FINGER_OPEN, "rise"),
    ]


def _take(xy, floor_z, label):
    """From above ``xy``, descend around the pot, grip it and lift it."""
    return [
        Waypoint(1.5, (*xy, floor_z + GRASP_DZ), FINGER_OPEN, f"descend to the pot on {label}"),
        Waypoint(0.8, (*xy, floor_z + GRASP_DZ), FINGER_CLOSED, "close"),
        Waypoint(1.2, (*xy, TRAVEL_Z), FINGER_CLOSED, "lift"),
    ]


def demo():
    """Left arm: put the pot on the hot plate, wait above it, and put it back at its start after DEMO_ON_S."""
    to_plate = _put(PLATE_XY, PLATE_TOP, "the hot plate")
    from_plate = _take(PLATE_XY, PLATE_TOP, "the hot plate")
    # On the plate from the end of "lower" to the start of "lift": release, rise, wait, descend, close.
    wait_s = DEMO_ON_S - sum(w.duration for w in to_plate[-2:] + from_plate[:2])
    return {
        "left": [
            Waypoint(1.5, (*POT_XY, TRAVEL_Z), FINGER_OPEN, "to the pot"),
            Waypoint(0.4, (*POT_XY, TRAVEL_Z), FINGER_OPEN, "settle"),
            *_take(POT_XY, TOP, "the table"),
            *to_plate,
            Waypoint(wait_s, (*PLATE_XY, TRAVEL_Z), FINGER_OPEN, "wait while it cooks"),
            *from_plate,
            *_put(POT_XY, TOP, "the table"),
        ]
    }
