"""Hang a baby T-shirt to dry over the free rail of a partly full airer. See README.md and DESIGN.md."""

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import bodykit  # noqa: E402
import clothkit  # noqa: E402
import fastik  # noqa: E402
import layout as L  # noqa: E402
import pinch  # noqa: E402
import rack  # noqa: E402
import scoring  # noqa: E402
import shirtplan  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0  # a full score must last 3 s: a shirt sliding off cannot end the run as a success
bodykit.allow_matrix_grips()
OBJECTS = [rack.RACK]

CORNER_IDX = {r: m.marks["corners"] for r, m in rack.CLOTH_MESH.items()}
CORNER_START = {r: m.start[m.marks["corners"]] for r, m in rack.CLOTH_MESH.items()}

_state = {"step": 0, "result": None, "trace": None}


@bodykit.on_reset
def _reset():
    _state["step"], _state["result"] = 0, None
    if _state["trace"] is not None:
        _state["trace"].close()
    path = os.environ.get("SHIRT_TRACE")
    _state["trace"] = open(path, "w") if path else None  # noqa: SIM115 -- closed at the next reset


@bodykit.on_step
def _observe(env):
    _state["step"] += 1
    tee = clothkit.points("tshirt")
    cloths = {r: clothkit.points(L.LOADED[r]) for r in L.LOADED}
    hands = {}
    for arm in pinch.BASES:
        pos, _ = pinch.tcp_frame(env, arm)
        h = pinch.held(arm)
        hands[arm] = (pos, pinch.gap(arm), h is not None and h[0] == "tshirt")
    result = scoring.evaluate(tee, cloths, CORNER_START, CORNER_IDX, hands)
    _state["result"] = result
    k = _state["step"]
    if _state["trace"] is not None and k % 5 == 0:
        row = {"step": k, "metrics": result.metrics,
               "hands": {a: {"tcp": np.round(h[0], 4).tolist(), "gap_mm": round(h[1] * 1000, 1)}
                         for a, h in hands.items()},
               "tee_lo": np.round(tee.min(0), 4).tolist(), "tee_hi": np.round(tee.max(0), 4).tolist()}
        _state["trace"].write(json.dumps(row) + "\n")
        _state["trace"].flush()
    if os.environ.get("SHIRT_PROBE") and k % 20 == 0:  # optional progress line every second
        m = result.metrics
        print(f"PROBE {k}: score {m['score']} rests {m['tee_rests_on']} low {m['tee_low']:.3f} "
              f"drop F/B {m['drop_front_mm']}/{m['drop_back_mm']} span {m['span_mm']} "
              f"cloth moves {'/'.join(str(m[f'cloth_{r}_move_mm']) for r in L.LOADED)} held {m['held']} "
              f"x {tee[:, 0].min():.3f}..{tee[:, 0].max():.3f}", flush=True)


def score(objects):
    return 0.0 if _state["result"] is None else _state["result"].score


def success(objects):
    return score(objects) == 1.0


def demo():
    """The scripted plan as TCP waypoints (forward kinematics of its joint targets, one per control step). The
    recorded demo plays the joint targets directly (run_demo.py); this lets ``--policy yam_scripted`` play it too."""
    out = {"left": [], "right": []}
    for row in shirtplan.plan().steps:
        for arm, (q, g, label) in row.items():
            pos, rot = fastik.fk(q, shirtplan.BASES[arm])
            out[arm].append(Waypoint(shirtplan.DT, tuple(pos), FINGER_OPEN * g, label, rot))
    return out
