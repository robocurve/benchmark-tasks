"""Yam-arena task file: Stand four dominoes on end on marks and tip the first to topple the chain.

Instruction:
"Stand the four dominoes on end on the white marks 30 mm apart, then tip the first one so every domino falls."
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo_plan
import geometry as g
import parts
from parts import env, on_reset, on_step, part, write_pose
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"

INSTRUCTION = "Stand the four dominoes on end on the white marks 30 mm apart, then tip the first one so every domino falls."


def world_part(name, pos, boxes=(), cylinders=(), **kw):
    px, py, pz = pos
    local_boxes = [((bx - px, by - py, bz - pz), sz, col) for (bx, by, bz), sz, col in boxes]
    local_cyls = [((cx - px, cy - py, cz - pz), r, h, col, *rest) for (cx, cy, cz), r, h, col, *rest in cylinders]
    return part(name, pos=pos, boxes=local_boxes, cylinders=local_cyls, **kw)


# 1. Four target marks (non-colliding flat visual marks)
MARK_0 = world_part(
    "mark_0",
    pos=g.MARK_0_POS,
    boxes=((g.MARK_0_POS, g.MARK_SIZE, g.MARK_WHITE),),
    mass=0.010,
    collision=False,
)

MARK_1 = world_part(
    "mark_1",
    pos=g.MARK_1_POS,
    boxes=((g.MARK_1_POS, g.MARK_SIZE, g.MARK_WHITE),),
    mass=0.010,
    collision=False,
)

MARK_2 = world_part(
    "mark_2",
    pos=g.MARK_2_POS,
    boxes=((g.MARK_2_POS, g.MARK_SIZE, g.MARK_WHITE),),
    mass=0.010,
    collision=False,
)

MARK_3 = world_part(
    "mark_3",
    pos=g.MARK_3_POS,
    boxes=((g.MARK_3_POS, g.MARK_SIZE, g.MARK_WHITE),),
    mass=0.010,
    collision=False,
)

# 2. Four free-body dominoes
DOMINO_0 = world_part(
    "domino_0",
    pos=g.DOMINO_0_START_POS,
    yaw=0.0,
    boxes=((g.DOMINO_0_START_POS, g.DOMINO_SIZE, g.COLOR_ORANGE),),
    mass=g.DOMINO_MASS,
    friction=g.DOMINO_FRICTION,
)

DOMINO_1 = world_part(
    "domino_1",
    pos=g.DOMINO_1_START_POS,
    yaw=0.0,
    boxes=((g.DOMINO_1_START_POS, g.DOMINO_SIZE, g.COLOR_BLUE),),
    mass=g.DOMINO_MASS,
    friction=g.DOMINO_FRICTION,
)

DOMINO_2 = world_part(
    "domino_2",
    pos=g.DOMINO_2_START_POS,
    yaw=0.0,
    boxes=((g.DOMINO_2_START_POS, g.DOMINO_SIZE, g.COLOR_YELLOW),),
    mass=g.DOMINO_MASS,
    friction=g.DOMINO_FRICTION,
)

DOMINO_3 = world_part(
    "domino_3",
    pos=g.DOMINO_3_START_POS,
    yaw=0.0,
    boxes=((g.DOMINO_3_START_POS, g.DOMINO_SIZE, g.COLOR_GREEN),),
    mass=g.DOMINO_MASS,
    friction=g.DOMINO_FRICTION,
)

OBJECTS = [
    MARK_0,
    MARK_1,
    MARK_2,
    MARK_3,
    DOMINO_0,
    DOMINO_1,
    DOMINO_2,
    DOMINO_3,
]

_SCORER = Scorer()
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
    dom_objs = [objects[f"domino_{i}"] for i in range(4)]
    poses = [d.pos for d in dom_objs]
    quats = [d.quat_xyzw for d in dom_objs]
    vels = [d.lin_vel for d in dom_objs]

    arm_tcps = _arm_info(e)

    result = _SCORER.update(
        domino_poses=poses,
        domino_quats=quats,
        domino_vels=vels,
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
