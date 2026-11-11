"""Yam-arena task file: Clear lane then push cargo block to dock.

Instruction:
"Move the two loose blocks out of the lane, then push the cargo block along the lane to the dock without touching the red post."
"""

from __future__ import annotations

import json
import os
import pathlib
import sys
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo_plan
import geometry as g
import logic
import parts
from parts import Part, env, on_reset, on_step, part, write_pose
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = g.HOLD_TIME_S

INSTRUCTION = (
    "Move the two loose blocks out of the lane, then push the cargo block along the lane to the dock without touching the red post."
)


def world_part(name, pos, boxes=(), cylinders=(), **kw):
    px, py, pz = pos
    local_boxes = [((bx - px, by - py, bz - pz), sz, col) for (bx, by, bz), sz, col in boxes]
    local_cyls = [((cx - px, cy - py, cz - pz), r, h, col, *rest) for (cx, cy, cz), r, h, col, *rest in cylinders]
    return part(name, pos=pos, boxes=local_boxes, cylinders=local_cyls, **kw)


# Lane Rails (fixed collision boxes)
RAIL_LEFT = world_part(
    "rail_left",
    pos=g.RAIL_LEFT_POS,
    boxes=(((g.RAIL_LEFT_POS[0], g.RAIL_LEFT_POS[1], g.RAIL_LEFT_POS[2]), g.RAIL_SIZE, (0.55, 0.55, 0.60)),),
    mass=10.0,
    collision=True,
)

RAIL_RIGHT = world_part(
    "rail_right",
    pos=g.RAIL_RIGHT_POS,
    boxes=(((g.RAIL_RIGHT_POS[0], g.RAIL_RIGHT_POS[1], g.RAIL_RIGHT_POS[2]), g.RAIL_SIZE, (0.55, 0.55, 0.60)),),
    mass=10.0,
    collision=True,
)

# Dock Stop Plate (fixed collision box)
STOP_PLATE = world_part(
    "stop_plate",
    pos=g.STOP_PLATE_POS,
    boxes=(((g.STOP_PLATE_POS[0], g.STOP_PLATE_POS[1], g.STOP_PLATE_POS[2]), g.STOP_PLATE_SIZE, (0.45, 0.45, 0.50)),),
    mass=10.0,
    collision=True,
)

# Dock Target Mark (visual flat mark)
DOCK_MARK = world_part(
    "dock_mark",
    pos=g.MARK_POS,
    boxes=(((g.MARK_POS[0], g.MARK_POS[1], g.MARK_POS[2]), g.MARK_SIZE, (0.95, 0.95, 0.95)),),
    mass=1.0,
    collision=False,
)

# Fragile Red Post (Free rigid body, red)
RED_POST = part(
    "red_post",
    pos=g.POST_START_POS,
    boxes=(((0.0, 0.0, 0.0), g.POST_SIZE, g.POST_COLOR),),
    mass=g.POST_MASS,
    friction=0.5,
    color=g.POST_COLOR,
)

# Blue Loose Block (Free rigid body, blue)
BLUE_BLOCK = part(
    "blue_block",
    pos=g.BLUE_START_POS,
    boxes=(((0.0, 0.0, 0.0), g.BLOCK_SIZE, g.BLUE_COLOR),),
    mass=g.BLOCK_MASS,
    friction=g.BLOCK_FRICTION,
    color=g.BLUE_COLOR,
)

# Yellow Loose Block (Free rigid body, yellow)
YELLOW_BLOCK = part(
    "yellow_block",
    pos=g.YELLOW_START_POS,
    boxes=(((0.0, 0.0, 0.0), g.BLOCK_SIZE, g.YELLOW_COLOR),),
    mass=g.BLOCK_MASS,
    friction=g.BLOCK_FRICTION,
    color=g.YELLOW_COLOR,
)

# Cargo Block (Free rigid body, orange)
CARGO_BLOCK = part(
    "cargo_block",
    pos=g.CARGO_START_POS,
    boxes=(((0.0, 0.0, 0.0), g.CARGO_SIZE, g.CARGO_COLOR),),
    mass=g.CARGO_MASS,
    friction=g.CARGO_FRICTION,
    color=g.CARGO_COLOR,
)

OBJECTS = [
    RAIL_LEFT,
    RAIL_RIGHT,
    STOP_PLATE,
    DOCK_MARK,
    RED_POST,
    BLUE_BLOCK,
    YELLOW_BLOCK,
    CARGO_BLOCK,
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
    cargo = objects["cargo_block"]
    blue = objects["blue_block"]
    yellow = objects["yellow_block"]
    post = objects["red_post"]

    arm_tcps = _arm_info(e)

    result = _SCORER.step(
        cargo_pos=cargo.pos,
        cargo_quat=cargo.quat_xyzw,
        blue_pos=blue.pos,
        yellow_pos=yellow.pos,
        post_pos=post.pos,
        post_quat=post.quat_xyzw,
        arm_tcps=arm_tcps,
    )

    _trace(
        {
            "score": result.score,
            "success": result.success,
            "dbg_post_hit": result.details.get("post_hit", False),
            "dbg_blue_cleared": result.details.get("blue_cleared", False),
            "dbg_yellow_cleared": result.details.get("yellow_cleared", False),
            "dbg_both_cleared": result.details.get("both_cleared", False),
            "dbg_cargo_x": result.details.get("cargo_x", 0.0),
            "dbg_hold_counter": result.details.get("hold_counter", 0),
        }
    )

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

