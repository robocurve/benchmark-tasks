"""Seat a Mini-ITX motherboard on an open hardware test bench."""
from pathlib import Path
import os
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import geometry as g
import scoring
import demo_plan
import parts
from yam_arena import runtime
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Seat the Mini-ITX motherboard on all four standoffs with its rear ports in the opening, then release it."
OBJECTS = g.OBJECTS
assert abs(TABLE_TOP[SCENE] - g.TOP) < 1e-6
_hands_clear = False


def _array(value):
    for name in ("detach", "cpu", "numpy"):
        if hasattr(value, name):
            value = getattr(value, name)()
    return np.asarray(value, dtype=float)


def _clear(env):
    from yam_arena.scene import object_states
    base = env.unwrapped
    origin = _array(base.scene.env_origins[0])
    board = object_states(env, g.MOVABLE)["motherboard"]
    low, high = np.asarray(board.low), np.asarray(board.high)
    for arm in ("left", "right"):
        if not parts.gripper_open(env, arm):
            return False
        robot = base.scene[arm + "_robot"]
        tcp = _array(runtime.tcp_world(robot)).reshape(-1)[:3] - origin
        nearest = np.maximum(np.maximum(low-tcp, tcp-high), 0.)
        if np.linalg.norm(nearest) < .06:
            return False
    return True


@parts.on_reset
def _reset(env):
    global _hands_clear
    _hands_clear = False


@parts.on_step
def _step(env):
    global _hands_clear
    _hands_clear = _clear(env)


def score(objects):
    if os.environ.get("ROBOCURVE_DEBUG_SCORE"):
        print("BOARD_FINAL_MEASURES", scoring.measures(objects["motherboard"]),
              "board_pos", tuple(float(x) for x in objects["motherboard"].pos),
              "hands_clear", _hands_clear, flush=True)
    return scoring.grade(objects, _hands_clear)


def success(objects):
    return score(objects) == 1.0


def demo():
    return demo_plan.plans()
