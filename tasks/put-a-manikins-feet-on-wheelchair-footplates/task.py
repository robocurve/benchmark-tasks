"""On the station table and a board behind it at the same height (together they stand in for the floor): a folding
manual wheelchair with its wheel locks on, facing the robot. An adult care manikin has just been seated in it. The
wheelchair's swing-away footrests are in place, but both flip-up footplates are still folded up, as they are left for
a transfer, and the manikin's lower legs hang limp from the knee with the feet where the plates will come down.

The job is the last step of every transfer into a wheelchair, before the chair is moved: fold each footplate down and
put the foot on it.

A plate cannot be folded down while the foot hangs in its way, and a limp leg does not stay where it is put. So one
hand lifts a lower leg forward and holds it, the other folds that plate down, the foot is lowered onto the plate, and
the hand lets go. Then the other side.

The chair's place changes with ``FOOT_LOT`` (A or B).

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import footkit as K  # noqa: E402
from footkit import Chair, Leg, Manikin, Plate, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("The resident manikin has just been seated in the wheelchair. Fold both footplates down and put its "
               "feet on them, then let go.")

# -- layout (m; x away from the robot, y left, heights above the table) ---------------------------------------------------
LOT = os.environ.get("FOOT_LOT", "A")
SEAT_FRONT = np.array({"A": (0.725, 0.000), "B": (0.745, 0.015)}[LOT])    # under the middle of the seat's front edge
TABLE_FAR = 0.910                               # the table's far edge; the board behind it carries the chair's back half
BOARD = (0.700, 1.300, 0.060)
PLATES_START = os.environ.get("FOOT_PLATES", "up")     # "down" starts both plates folded down (for a control run)

SIDES = {"right": 1.0, "left": -1.0}            # the manikin's right is on the robot's left (+y)
CHAIR, MANIKIN = "chair", "manikin"
LEGS = {s: f"leg_{s}" for s in SIDES}
PLATES = {s: f"plate_{s}" for s in SIDES}

# -- scoring -----------------------------------------------------------------------------------------------------------
DOWN = math.radians(5.0)          # a plate is down on its stop when it is within this of level
ON_TOP = (-0.003, 0.005)          # the sole's lowest point is this close to the plate's top (below, above)
CLEAR = 0.030                     # both grippers this far from the leg
STILL = math.radians(5.0)         # rad/s: a leg (and its plate) at rest
TOUCH = 0.002                     # a plate within this of a leg touches it
JERK = math.radians(90.0)         # rad/s: a knee turning faster than this was jerked, or the leg was dropped
RANGE = (math.radians(-15.0), math.radians(80.0))   # a knee pushed back, or straightened, past these was forced


def chair_pos() -> np.ndarray:
    return np.array([SEAT_FRONT[0], SEAT_FRONT[1], TOP])


def knee_world(side: float) -> np.ndarray:
    return chair_pos() + K.knee_in_chair(side)


def hinge_world(side: float) -> np.ndarray:
    return chair_pos() + K.hinge_in_chair(side)


def _objects() -> list:
    c = chair_pos()
    up = 0.0 if PLATES_START == "down" else K.PLATE_UP
    out = [Box("floor_board", pos=(TABLE_FAR + BOARD[0] / 2, 0.0, TOP - BOARD[2] / 2), size=BOARD, color=(0.80, 0.74, 0.62)),
           Chair(CHAIR, pos=tuple(float(v) for v in c), size=(1.05, 0.68, 0.95)),
           Manikin(MANIKIN, pos=tuple(float(v) for v in c + K.knee_in_chair(0.0)), size=(0.60, 0.45, 0.80))]
    for name, side in SIDES.items():
        out.append(Leg(LEGS[name], pos=tuple(float(v) for v in knee_world(side)), size=(0.24, 0.10, 0.45), side=side,
                       body=MANIKIN))
        out.append(Plate(PLATES[name], pos=tuple(float(v) for v in hinge_world(side)), size=(0.16, 0.16, 0.02), side=side,
                         chair=CHAIR, quat=K.plate_quat(side, up)))
    return out


OBJECTS = _objects()


# -- what the trial looks like now ------------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(steps=0, broken=None, sides={n: dict(lift=0.0, prev=None, plate=math.radians(K.PLATE_UP), down=False,
                                                         on=False, released=False, still=False, struck=False, gap=1.0,
                                                         heel=(0.0, 0.0, 0.0), mid=(0.0, 0.0, 0.0), low=0.0, fastest=0.0,
                                                         highest=0.0) for n in SIDES})


_clear()


def side_score(down: bool, on: bool, released: bool, still: bool, struck: bool) -> float:
    """Pure rule for one side. ``down``: the plate is on its down stop. ``on``: the foot rests on the plate (heel's
    middle and foot's middle over its top, sole on it). ``released``: both grippers are clear of the leg. ``still``:
    leg and plate are at rest. ``struck``: the plate touched the leg at some time while it was not down."""
    if struck or not down:
        return 0.0
    return 0.5 if (on and released and still) else 0.25


def score_from(sides: dict, broken) -> float:
    """Pure scoring rule: the two sides' scores added, or 0 if a leg was jerked, dropped or forced."""
    if broken:
        return 0.0
    return float(sum(side_score(s["down"], s["on"], s["released"], s["still"], s["struck"]) for s in sides.values()))


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    st = _STATE
    st["steps"] += 1
    names = [*LEGS.values(), *PLATES.values()]
    obj = object_states(environment, names)
    hands = np.array([p for side in K.gripper_points(environment) for p in side])
    dt = 1.0 / 20.0
    for name, side in SIDES.items():
        s = st["sides"][name]
        leg, plate = obj[LEGS[name]], obj[PLATES[name]]
        lift = K.knee_lift(leg.quat_xyzw)
        if s["prev"] is not None and st["steps"] > 2:
            rate = abs(lift - s["prev"]) / dt
            s["fastest"] = max(s["fastest"], rate)
            if rate > JERK and not st["broken"]:
                st["broken"] = f"{name} knee turned {math.degrees(rate):.0f} deg/s at step {st['steps']}"
        s["prev"] = lift
        s["lift"], s["highest"] = lift, max(s["highest"], lift)
        if not (RANGE[0] <= lift <= RANGE[1]) and not st["broken"]:
            st["broken"] = f"{name} knee at {math.degrees(lift):.0f} deg at step {st['steps']}"
        s["plate"] = K.plate_lift(plate.quat_xyzw, side)
        s["down"] = bool(s["plate"] <= DOWN)
        s["gap"] = K.leg_plate_gap(leg.pos, leg.quat_xyzw, plate.pos, plate.quat_xyzw, side)
        if not s["down"] and s["gap"] < TOUCH:
            s["struck"] = True
        m = K.foot_marks(leg.pos, leg.quat_xyzw, plate.pos, plate.quat_xyzw, side)
        s["heel"], s["mid"], s["low"] = tuple(m["heel"]), tuple(m["mid"]), m["low"]
        s["on"] = bool(s["down"] and K.over_plate(m["heel"]) and K.over_plate(m["mid"]) and ON_TOP[0] <= m["low"] <= ON_TOP[1])
        s["released"] = bool(K.leg_distance(hands, leg.pos, leg.quat_xyzw) >= CLEAR)
        s["still"] = bool(float(np.linalg.norm(leg.ang_vel)) < STILL and float(np.linalg.norm(plate.ang_vel)) < STILL)


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """Scored at the end of the trial, each side by itself, the two added. A side scores 0 while its plate is not down
    on its stop (within 5 degrees of level), and 0 if its plate touched the leg (came within 2 mm of it) at any time
    while the plate was not down: a plate folded onto a foot that still hangs, or leaned on the leg. Otherwise 0.25:
    the plate is down. 0.5 instead: and the foot rests on it (the middle of the heel's pad and the middle of the foot
    are over the plate's top, and the sole's lowest point is no more than 5 mm above it), both grippers are at least
    3 cm from that leg, and leg and plate are at rest (under 5 deg/s). The whole trial scores 0 if a knee ever turns
    faster than 90 deg/s (the leg was jerked or dropped) or is taken more than 80 degrees forward of hanging or more
    than 15 degrees back (forced)."""
    return score_from(_STATE["sides"], _STATE["broken"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    st = _STATE
    out = {"score": score(objects), "lot": LOT, "broken": st["broken"]}
    for name, s in st["sides"].items():
        out[name] = {"score": 0.0 if st["broken"] else side_score(s["down"], s["on"], s["released"], s["still"], s["struck"]),
                     "knee_deg": round(math.degrees(s["lift"]), 1), "plate_deg": round(math.degrees(s["plate"]), 1),
                     "down": s["down"], "on": s["on"], "released": s["released"], "still": s["still"], "struck": s["struck"],
                     "gap_mm": round(1000 * s["gap"], 1), "heel_mm": [round(1000 * float(v), 1) for v in s["heel"]],
                     "mid_mm": [round(1000 * float(v), 1) for v in s["mid"]], "low_mm": round(1000 * s["low"], 1),
                     "fastest_deg_s": round(math.degrees(s["fastest"]), 1), "highest_deg": round(math.degrees(s["highest"]), 1)}
    return out


def demo(objects=None, seed=None):
    return {}
