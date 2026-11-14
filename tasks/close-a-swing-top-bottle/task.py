"""On the station table: a 200 mL swing-top glass bottle of lemonade, standing open. Its porcelain stopper hangs on its
wire bail beside the neck, and its lever is flipped back over the top.

The bottle has to be closed. A swing-top closes in two moves: swing the stopper up onto the mouth, then bring the
lever over the top and down the far side of the neck. Near the end the lever stretches the bail and pushes back, until
it passes its centre and snaps down, clamping the stopper onto the mouth. Let go before the centre and it springs back
open; shut the lever before the stopper is on and it closes on nothing. The push near the centre is enough to tip over
a 0.45 kg bottle that is not held.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable BOT_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bottlekit as K  # noqa: E402
from bottlekit import Bottle, finger_points, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Close the swing-top bottle of lemonade."

# -- layout ---------------------------------------------------------------------------------------------------------
LOT = os.environ.get("BOT_LOT", "A")
LOTS = {"A": dict(bottle=(0.280, -0.020)), "B": dict(bottle=(0.290, -0.030))}
if LOT not in LOTS:
    raise ValueError(f"BOT_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
BOTTLE_XY = np.array(LOTS[LOT]["bottle"])
BOTTLE_MASS = 0.45                       # 200 mL of lemonade in a 0.25 kg glass bottle
NAME = "bottle"

# -- scoring -----------------------------------------------------------------------------------------------------------
HOLD_STEPS = 40                          # sealed, upright and let go this many control steps (2 s) for full marks
TIPPED = math.radians(30.0)              # the bottle tilted this far at any time has fallen or spilled
UPRIGHT = math.radians(10.0)             # standing: tilted less than this
STANDING = 0.005                         # m: its base within this of the table
CLEAR = 0.030                            # both grippers' tool points this far from the lever's paddle: let go


def bottle_start() -> np.ndarray:
    return np.array([BOTTLE_XY[0], BOTTLE_XY[1], TOP])


OBJECTS = [Bottle(NAME, pos=tuple(float(v) for v in bottle_start()), size=(0.056, 0.056, 0.16), mass=BOTTLE_MASS)]


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(tipped=False, held_steps=0, done=False, steps=0, sealed=False, seated=False, upright=True)


_clear()


def tilt(quat) -> float:
    return math.acos(max(-1.0, min(1.0, float(K._rotation(quat)[2, 2]))))


def standing(pos, quat) -> bool:
    return tilt(quat) < UPRIGHT and abs(float(pos[2]) - TOP) < STANDING


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
    if s["steps"] > 5 and tilt(quat) > TIPPED:
        s["tipped"] = True
    s["sealed"] = K.sealed(NAME)
    s["seated"] = st.joints["stopper"] < K.SEATED
    s["upright"] = standing(pos, quat)
    paddle = K.paddle_point(pos, quat, st.joints["lever"])
    free = all(float(np.linalg.norm(paddle - h)) > CLEAR for h in finger_points(environment))
    ok = s["sealed"] and s["upright"] and free
    s["held_steps"] = s["held_steps"] + 1 if ok else 0
    s["done"] = s["done"] or s["held_steps"] >= HOLD_STEPS


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """0 if the bottle ever tilted 30 degrees or more (fallen, or tipped far enough to spill). Otherwise 1.0 once it
    is sealed (lever snapped over its centre with the stopper on the mouth), standing upright on the table, with both
    grippers 3 cm or more from the lever's paddle, for 2 s; else 0.5 if at the end the stopper is on the mouth (sealed
    or not) and the bottle stands upright; else 0."""
    s = _STATE
    if s["tipped"]:
        return 0.0
    if s["done"]:
        return 1.0
    return 0.5 if (s["seated"] or s["sealed"]) and s["upright"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    st = objects[NAME]
    j = getattr(st, "joints", {}) or {}
    return {"score": score(objects), "tipped": _STATE["tipped"], "sealed": _STATE["sealed"],
            "held_steps": _STATE["held_steps"],
            "bottle_cm": [round(100 * float(v), 2) for v in st.pos], "tilt_deg": round(math.degrees(tilt(st.quat_xyzw)), 1),
            "stopper_deg": round(math.degrees(j.get("stopper", float("nan"))), 1),
            "lever_deg": round(math.degrees(j.get("lever", float("nan"))), 1)}


def demo(objects, seed):
    return {}
