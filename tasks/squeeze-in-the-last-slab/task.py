"""Yam-arena task file: Stand last slab into gap without knocking neighbours.

Instruction:
"Stand the last slab into the one gap between the standing slabs without knocking either neighbour."
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
    "Stand the last slab into the one gap between the standing slabs without knocking either neighbour."
)


def world_part(name, pos, boxes=(), cylinders=(), **kw):
    px, py, pz = pos
    local_boxes = [((bx - px, by - py, bz - pz), sz, col) for (bx, by, bz), sz, col in boxes]
    local_cyls = [((cx - px, cy - py, cz - pz), r, h, col, *rest) for (cx, cy, cz), r, h, col, *rest in cylinders]
    return part(name, pos=pos, boxes=local_boxes, cylinders=local_cyls, **kw)


# White Strip on table
STRIP = world_part(
    "strip",
    pos=g.STRIP_POS,
    boxes=(((g.STRIP_POS[0], g.STRIP_POS[1], g.STRIP_POS[2]), g.STRIP_SIZE, g.COLOR_STRIP),),
    mass=1.0,
    collision=False,
)

# Four Standing Neighbor Slabs (Free rigid bodies, grey)
SLAB_1 = part(
    "slab_1",
    pos=g.SLAB_1_POS,
    boxes=(((0.0, 0.0, 0.0), g.SLAB_DIMS_STANDING, g.COLOR_SLAB_GREY),),
    mass=g.SLAB_MASS,
    friction=g.SLAB_FRICTION,
    color=g.COLOR_SLAB_GREY,
)

SLAB_2 = part(
    "slab_2",
    pos=g.SLAB_2_POS,
    boxes=(((0.0, 0.0, 0.0), g.SLAB_DIMS_STANDING, g.COLOR_SLAB_GREY),),
    mass=g.SLAB_MASS,
    friction=g.SLAB_FRICTION,
    color=g.COLOR_SLAB_GREY,
)

SLAB_3 = part(
    "slab_3",
    pos=g.SLAB_3_POS,
    boxes=(((0.0, 0.0, 0.0), g.SLAB_DIMS_STANDING, g.COLOR_SLAB_GREY),),
    mass=g.SLAB_MASS,
    friction=g.SLAB_FRICTION,
    color=g.COLOR_SLAB_GREY,
)

SLAB_4 = part(
    "slab_4",
    pos=g.SLAB_4_POS,
    boxes=(((0.0, 0.0, 0.0), g.SLAB_DIMS_STANDING, g.COLOR_SLAB_GREY),),
    mass=g.SLAB_MASS,
    friction=g.SLAB_FRICTION,
    color=g.COLOR_SLAB_GREY,
)

# Fifth Slab (Blue, starting standing on its edge)
SLAB_5 = part(
    "slab_5",
    pos=g.SLAB_5_START,
    boxes=(((0.0, 0.0, 0.0), g.SLAB_DIMS_STANDING, g.COLOR_SLAB_BLUE),),
    mass=g.SLAB_MASS,
    friction=g.SLAB_FRICTION,
    color=g.COLOR_SLAB_BLUE,
)

OBJECTS = [
    STRIP,
    SLAB_1,
    SLAB_2,
    SLAB_3,
    SLAB_4,
    SLAB_5,
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
    s1 = objects["slab_1"]
    s2 = objects["slab_2"]
    s3 = objects["slab_3"]
    s4 = objects["slab_4"]
    s5 = objects["slab_5"]

    arm_tcps = _arm_info(e)

    result = _SCORER.update(
        s1_pos=s1.pos, s1_quat=s1.quat_xyzw,
        s2_pos=s2.pos, s2_quat=s2.quat_xyzw,
        s3_pos=s3.pos, s3_quat=s3.quat_xyzw,
        s4_pos=s4.pos, s4_quat=s4.quat_xyzw,
        s5_pos=s5.pos, s5_quat=s5.quat_xyzw,
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

