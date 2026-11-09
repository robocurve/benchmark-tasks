"""Yam-arena task file: Move three-block tower from left mark to right mark.

Instruction:
"Move the three-block tower from the left mark to the right mark, keeping the biggest at the bottom, and never set a bigger block on a smaller one."
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo_plan
import geometry as g
import logic
import parts
from parts import env, on_reset, on_step, part, write_pose
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = g.HOLD_TIME_S

INSTRUCTION = "Move the three-block tower from the left mark to the right mark, keeping the biggest at the bottom, and never set a bigger block on a smaller one."


def world_part(name, pos, boxes=(), cylinders=(), **kw):
    px, py, pz = pos
    local_boxes = [((bx - px, by - py, bz - pz), sz, col) for (bx, by, bz), sz, col in boxes]
    local_cyls = [((cx - px, cy - py, cz - pz), r, h, col, *rest) for (cx, cy, cz), r, h, col, *rest in cylinders]
    return part(name, pos=pos, boxes=local_boxes, cylinders=local_cyls, **kw)


# 1. Three target marks (non-colliding flat visual marks)
MARK_LEFT = world_part(
    "mark_left",
    pos=g.LEFT_MARK_POS,
    boxes=((g.LEFT_MARK_POS, g.MARK_SIZE, g.MARK_WHITE),),
    mass=0.010,
    collision=False,
)

MARK_MID = world_part(
    "mark_mid",
    pos=g.MID_MARK_POS,
    boxes=((g.MID_MARK_POS, g.MARK_SIZE, g.MARK_WHITE),),
    mass=0.010,
    collision=False,
)

MARK_RIGHT = world_part(
    "mark_right",
    pos=g.RIGHT_MARK_POS,
    boxes=((g.RIGHT_MARK_POS, g.MARK_SIZE, g.MARK_WHITE),),
    mass=0.010,
    collision=False,
)

# 2. Three stackable blocks (free bodies)
BLOCK_LARGE = world_part(
    "block_large",
    pos=g.LARGE_START_POS,
    yaw=0.0,
    boxes=((g.LARGE_START_POS, g.LARGE_SIZE, g.BLOCK_LARGE_BLUE),),
    mass=g.LARGE_MASS,
    friction=g.LARGE_FRICTION,
)

BLOCK_MEDIUM = world_part(
    "block_medium",
    pos=g.MEDIUM_START_POS,
    yaw=0.0,
    boxes=((g.MEDIUM_START_POS, g.MEDIUM_SIZE, g.BLOCK_MED_ORANGE),),
    mass=g.MEDIUM_MASS,
    friction=g.MEDIUM_FRICTION,
)

BLOCK_SMALL = world_part(
    "block_small",
    pos=g.SMALL_START_POS,
    yaw=0.0,
    boxes=((g.SMALL_START_POS, g.SMALL_SIZE, g.BLOCK_SMALL_YELLOW),),
    mass=g.SMALL_MASS,
    friction=g.SMALL_FRICTION,
)

OBJECTS = [
    MARK_LEFT,
    MARK_MID,
    MARK_RIGHT,
    BLOCK_LARGE,
    BLOCK_MEDIUM,
    BLOCK_SMALL,
]

_SCORER = Scorer(hold_s=SUCCESS_HOLD_S, hz=20.0)
_last_states_ref = None
_last_result = None


def reset(_environment=None) -> None:
    global _last_states_ref, _last_result
    _last_states_ref = None
    _last_result = None
    _SCORER.reset()


on_reset(reset)


def _trace(row: dict) -> None:
    path = os.environ.get("TASK_TRACE")
    if path:
        with open(path, "a") as f:
            f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")


def _arm_info(e):
    if e is None:
        return None
    try:
        from parts import _to_numpy
        from yam_arena import runtime

        base = e.unwrapped
        origin = _to_numpy(base.scene.env_origins[0])
        tcps = {}
        for arm in ("left", "right"):
            robot = base.scene[f"{arm}_robot"]
            tcps[arm] = (runtime.tcp_world(robot) - origin).tolist()
        return tcps
    except Exception:
        return None


def _update(objects):
    global _last_states_ref, _last_result
    if objects is _last_states_ref and _last_result is not None:
        return _last_result

    e = env()
    large = objects["block_large"]
    medium = objects["block_medium"]
    small = objects["block_small"]

    arm_tcps = _arm_info(e)

    result = _SCORER.update(
        large_pos=large.pos,
        large_quat=large.quat_xyzw,
        large_vel=large.lin_vel,
        medium_pos=medium.pos,
        medium_quat=medium.quat_xyzw,
        medium_vel=medium.lin_vel,
        small_pos=small.pos,
        small_quat=small.quat_xyzw,
        small_vel=small.lin_vel,
        arm_tcps=arm_tcps,
    )

    _trace({
        "score": result.score,
        "success": result.success,
        **result.details,
    })
    _last_states_ref = objects
    _last_result = result
    return result


def success(objects) -> bool:
    return _update(objects).success


def score(objects) -> float:
    return _update(objects).score


def demo():
    return demo_plan.plans()


def plans():
    return demo_plan.plans()

