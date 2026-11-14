"""On the station table: a footing form (a 30 mm plywood floor) with five vertical #4 rebar stubs, 70 mm tall, in a row
along y. Two stubs already wear a red plastic safety cap; three are bare. Three more caps stand on staging pads in front
of the bare stubs. The robot must make the stubs safe to walk past: every stub capped.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, on_reset, on_step  # noqa: E402

import geometry as g  # noqa: E402
import logic  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = g.HOLD_S

INSTRUCTION = "Make the rebar stubs safe for the crew to walk the formwork."

TOP = g.TOP

# -- objects -------------------------------------------------------------------------------------------------------
FORM = UsdBody("form", usd="assets/form.usda", pos=(g.FORM_X, 0.0, TOP), size=g.FORM_SIZE, friction=(0.8, 0.7))


def _cap(name: str, y: float, z: float) -> UsdBody:
    return UsdBody(name, usd="assets/cap.usda", pos=(g.PAD_X if z == g.CAP_START_Z else g.FORM_X, y, z),
                   size=(g.CAP_HEAD, g.CAP_HEAD, g.CAP_H), mass=g.CAP_MASS, friction=g.CAP_FRICTION, solver_iters=32,
                   torsional_patch=0.01)


PRE_CAPS = [_cap(f"cap_fitted_{k + 1}", g.STUB_Y[i], g.SEATED_Z) for k, i in enumerate(g.PRE_CAPPED)]
LOOSE_CAPS = [_cap(f"cap_loose_{k + 1}", y, g.CAP_START_Z) for k, y in enumerate(g.CAP_START_Y)]
OBJECTS = [FORM] + PRE_CAPS + LOOSE_CAPS
CAP_NAMES = [c.name for c in PRE_CAPS + LOOSE_CAPS]

# -- monitor ---------------------------------------------------------------------------------------------------------
_SCORER = logic.Scorer()
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _SCORER.reset()
    _STATE.update(result={"score": 0.0, "success": False, "capped": [False] * 5, "n_capped": 0}, why=[], tcps=[],
                  caps={})


_reset_state()


def _tool_points(environment) -> list:
    """Both grippers' tool points (world frame), by forward kinematics from the measured arm joints."""
    from scenekit import arm_joints
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    out = []
    for side in ("left", "right"):
        q = arm_joints(environment, side, ARM_JOINTS)
        out.append(ik.tcp_pose(q, station.BASES[side])[0])
    return out


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    st = object_states(environment, CAP_NAMES)
    caps = {n: (st[n].pos.copy(), st[n].quat_xyzw.copy()) for n in CAP_NAMES}
    tcps = _tool_points(environment)
    res = _SCORER.step(caps=caps, tcps=tcps)
    _STATE.update(result=res, tcps=tcps, caps=caps)
    path = os.environ.get("TASK_TRACE")                    # a debugging aid: one JSON line per control step
    if path:
        row = {"capped": res["capped"], "clear": res["arms_clear"], "hold": res["hold_steps"], "score": res["score"],
               "tcp_left": [round(float(v), 4) for v in tcps[0]], "tcp_right": [round(float(v), 4) for v in tcps[1]],
               "caps": {n: [round(float(v), 4) for v in caps[n][0]] + [round(float(logic.tilt(caps[n][1])), 3)]
                        for n in CAP_NAMES}}
        with open(path, "a") as f:
            f.write(json.dumps(row) + "\n")


def success(objects) -> bool:
    """All five stubs wear a seated cap, both arms are clear, held 3 s."""
    return bool(_STATE["result"]["success"])


def score(objects) -> float:
    """The state of the current step. With n of the five stubs capped (two are at the start): n = 5 held 3 s with both
    arms clear 1.0; n = 5 not yet held 0.75; n = 4 0.5; n = 3 0.25; else 0. A cap is seated when the bar is at least 26 mm
    up its bore, its axis is within 3 mm of the bar's and within 8 degrees of upright. Knocking a cap off lowers the
    score again."""
    return float(_STATE["result"]["score"])
