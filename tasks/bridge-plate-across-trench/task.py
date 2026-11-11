"""Yam-arena task file: Lay bridge plate across trench and push block to far platform.

Instruction:
"Lay the bridge plate across the trench, then push the block over it onto the far platform without letting the plate or the block fall into the trench."
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

INSTRUCTION = "Lay the bridge plate across the trench, then push the block over it onto the far platform without letting the plate or the block fall into the trench."


def world_part(name, pos, boxes=(), cylinders=(), **kw):
    px, py, pz = pos
    local_boxes = [((bx - px, by - py, bz - pz), sz, col) for (bx, by, bz), sz, col in boxes]
    local_cyls = [((cx - px, cy - py, cz - pz), r, h, col, *rest) for (cx, cy, cz), r, h, col, *rest in cylinders]
    return part(name, pos=pos, boxes=local_boxes, cylinders=local_cyls, **kw)


# 1. Near Platform fixture (lip y in [0.045, 0.165] plus staging ledge y in [0.165, 0.285])
PLATFORM_NEAR = world_part(
    "platform_near",
    pos=g.NEAR_PLATFORM_POS,
    boxes=(
        (g.NEAR_PLATFORM_POS, g.NEAR_PLATFORM_SIZE, g.PLATFORM_GREY),
        (g.NEAR_STAGING_POS, g.NEAR_STAGING_SIZE, g.PLATFORM_GREY),
    ),
    mass=5.0,
    friction=1.0,
)

# 2. Far Platform fixture (lip y in [-0.165, -0.045] plus rear ledge y in [-0.285, -0.165])
PLATFORM_FAR = world_part(
    "platform_far",
    pos=g.FAR_PLATFORM_POS,
    boxes=(
        (g.FAR_PLATFORM_POS, g.FAR_PLATFORM_SIZE, g.PLATFORM_GREY),
        (g.FAR_REAR_POS, g.FAR_REAR_SIZE, g.PLATFORM_GREY),
        ((g.STOP_POST_XS[0], g.STOP_POST_Y, g.STOP_POST_Z), g.STOP_POST_SIZE, g.PLATFORM_GREY),
        ((g.STOP_POST_XS[1], g.STOP_POST_Y, g.STOP_POST_Z), g.STOP_POST_SIZE, g.PLATFORM_GREY),
    ),
    mass=5.0,
    friction=1.0,
)

# 3. Target Mark (flat non-colliding mark on far platform)
TARGET_MARK = world_part(
    "target_mark",
    pos=g.TARGET_MARK_POS,
    boxes=((g.TARGET_MARK_POS, g.MARK_SIZE, g.MARK_WHITE),),
    mass=0.010,
    collision=False,
)

# 4. Bridge Plate (free body)
BRIDGE_PLATE = world_part(
    "bridge_plate",
    pos=g.BRIDGE_START_POS,
    yaw=0.0,
    boxes=((g.BRIDGE_START_POS, g.BRIDGE_SIZE, g.BRIDGE_ORANGE),
           ((g.BRIDGE_START_POS[0] + g.HANDLE_OFFSET_X, g.BRIDGE_START_POS[1], g.HANDLE_CENTER_Z), g.HANDLE_SIZE, g.BRIDGE_ORANGE)),
    mass=g.BRIDGE_MASS,
    friction=g.BRIDGE_FRICTION,
)

# 5. Sliding Block (free body)
BLOCK = world_part(
    "block",
    pos=g.BLOCK_START_POS,
    yaw=0.0,
    boxes=((g.BLOCK_START_POS, g.BLOCK_SIZE, g.BLOCK_BLUE),),
    mass=g.BLOCK_MASS,
    friction=g.BLOCK_FRICTION,
)

OBJECTS = [
    PLATFORM_NEAR,
    PLATFORM_FAR,
    TARGET_MARK,
    BRIDGE_PLATE,
    BLOCK,
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
    bridge = objects["bridge_plate"]
    block = objects["block"]

    arm_tcps = _arm_info(e)

    result = _SCORER.update(
        bridge_pos=bridge.pos,
        bridge_quat=bridge.quat_xyzw,
        bridge_vel=bridge.lin_vel,
        block_pos=block.pos,
        block_quat=block.quat_xyzw,
        block_vel=block.lin_vel,
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
