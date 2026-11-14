"""On the station table: a 2-quart hand-crank salad spinner with washed lettuce in its basket, lid on.

The lettuce has to be spun dry. The crank on the lid turns the basket through 1:5 gears and a one-way clutch, so the
basket spins up while the crank is turned clockwise and coasts when it stops. The gears resist the crank, most of
all while the basket is spinning up, and that resistance turns the whole bowl on the worktop unless something holds
it: the bowl's plastic base slides easily. So one hand holds the bowl by its side handle while the other turns the crank.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import spinnerkit as K  # noqa: E402
from spinnerkit import Spinner, finger_points, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Spin the washed lettuce dry in the salad spinner: give its crank at least five full turns clockwise, "
               "then let go of it.")

# -- layout ---------------------------------------------------------------------------------------------------------
LOT = "A"                                # one fixed layout
SPINNER_XY = np.array([0.320, -0.070])   # the spinner's base, centred here
BOWL_MASS = 0.30                         # bowl, lid and gears; the crank, knob and the basket with lettuce add 0.30 kg
NAME = "spinner"

# -- scoring -----------------------------------------------------------------------------------------------------------
FULL_TURNS = 5.0                         # forward crank turns on the lid for full marks
HALF_TURNS = 2.0
SOME_TURNS = 0.5
KNOCKED_MOVE = 0.08                      # m: the bowl shoved this far across the table at any time ...
KNOCKED_TILT = math.radians(15.0)        # ... or tilted this far: knocked about, 0 for the trial
CLEAR = 0.040                            # both grippers' tool points this far from the knob: let go


def spinner_start() -> np.ndarray:
    return np.array([SPINNER_XY[0], SPINNER_XY[1], TOP])


OBJECTS = [Spinner(NAME, pos=tuple(float(v) for v in spinner_start()), size=(0.25, 0.20, 0.17), mass=BOWL_MASS)]


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(knocked=False, crank_turns=0.0, basket_turns=0.0, basket_rps=0.0, let_go=False, steps=0,
                  moved=0.0, yaw_turns=0.0)


_clear()


def tilt(quat) -> float:
    return math.acos(max(-1.0, min(1.0, float(K._rotation(quat)[2, 2]))))


def yaw(quat) -> float:
    r = K._rotation(quat)
    return math.atan2(r[1, 0], r[0, 0])


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    s["steps"] += 1
    st = object_states(environment, [NAME])[NAME]
    pos, quat = np.asarray(st.pos), st.quat_xyzw
    moved = float(np.linalg.norm(pos[:2] - spinner_start()[:2]))
    s["moved"] = max(s["moved"], moved)
    s["yaw_turns"] = yaw(quat) / (2 * math.pi)
    if s["steps"] > 5 and (moved > KNOCKED_MOVE or tilt(quat) > KNOCKED_TILT):
        s["knocked"] = True
    t = K.turns(NAME)
    s["crank_turns"], s["basket_turns"], s["basket_rps"] = t["crank"], t["basket"], t["basket_rps"]
    knob = K.knob_point(pos, quat, st.joints["crank"], up=K.KNOB_H / 2)
    s["let_go"] = all(float(np.linalg.norm(knob - h)) > CLEAR for h in finger_points(environment))


# -- scoring --------------------------------------------------------------------------------------------------------
def score_from(crank_turns: float, knocked: bool, let_go: bool) -> float:
    if knocked:
        return 0.0
    if crank_turns >= FULL_TURNS and let_go:
        return 1.0
    if crank_turns >= HALF_TURNS:
        return 0.5
    return 0.25 if crank_turns >= SOME_TURNS else 0.0


def score(objects) -> float:
    """0 if the spinner was ever knocked about (shoved 8 cm across the table or tilted 15 degrees). Otherwise, from
    the crank's net clockwise turns on the lid (turns of the whole spinner with it do not count) at the end: 1.0 for
    five or more with both grippers 4 cm or more from the crank's knob; 0.5 for two or more; 0.25 for half a turn or
    more; else 0."""
    s = _STATE
    return score_from(s["crank_turns"], s["knocked"], s["let_go"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    st = objects[NAME]
    s = _STATE
    return {"score": score(objects), "crank_turns": round(s["crank_turns"], 2), "basket_turns": round(s["basket_turns"], 1),
            "basket_rps": round(s["basket_rps"], 2), "knocked": s["knocked"], "let_go": s["let_go"],
            "moved_cm": round(100 * s["moved"], 2), "bowl_yaw_deg": round(360 * s["yaw_turns"], 1),
            "spinner_cm": [round(100 * float(v), 2) for v in st.pos], "tilt_deg": round(math.degrees(tilt(st.quat_xyzw)), 1)}


def demo(objects, seed):
    return {}
