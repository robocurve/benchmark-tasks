"""Yam-arena task file: Slide three tiles into order without lifting.

Instruction:
"Slide the three tiles in the frame into the order shown on the card using the empty cell, without lifting a tile out of the frame."
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
    "Slide the three tiles in the frame into the order shown on the card using the empty cell, without lifting a tile out of the frame."
)


def world_part(name, pos, boxes=(), cylinders=(), **kw):
    px, py, pz = pos
    local_boxes = [((bx - px, by - py, bz - pz), sz, col) for (bx, by, bz), sz, col in boxes]
    local_cyls = [((cx - px, cy - py, cz - pz), r, h, col, *rest) for (cx, cy, cz), r, h, col, *rest in cylinders]
    return part(name, pos=pos, boxes=local_boxes, cylinders=local_cyls, **kw)


# Fixed 2x2 Frame Outer Rim (4 mm tall, 6 mm thick)
def make_frame():
    cx, cy, cz = g.FRAME_POS
    w_in = g.FRAME_INNER
    rw = g.RIM_THICK
    rh = g.RIM_HEIGHT
    boxes = (
        # North (+x)
        ((cx + w_in / 2.0 + rw / 2.0, cy, cz), (rw, w_in + 2 * rw, rh), g.COLOR_FRAME),
        # South (-x)
        ((cx - w_in / 2.0 - rw / 2.0, cy, cz), (rw, w_in + 2 * rw, rh), g.COLOR_FRAME),
        # East (+y)
        ((cx, cy + w_in / 2.0 + rw / 2.0, cz), (w_in, rw, rh), g.COLOR_FRAME),
        # West (-y)
        ((cx, cy - w_in / 2.0 - rw / 2.0, cz), (w_in, rw, rh), g.COLOR_FRAME),
    )
    return world_part("frame", pos=g.FRAME_POS, boxes=boxes, mass=10.0, friction=1.0)


FRAME = make_frame()

# Fixed Goal Card showing target arrangement
CARD = world_part(
    "card",
    pos=g.CARD_POS,
    boxes=(((g.CARD_POS[0], g.CARD_POS[1], g.CARD_POS[2]), g.CARD_SIZE, g.COLOR_CARD),),
    mass=1.0,
    collision=False,
)

# Three Free Tiles
TILE_BLUE = part(
    "tile_blue",
    pos=g.TILE_BLUE_START,
    boxes=(((0.0, 0.0, 0.0), g.TILE_DIMS, g.COLOR_TILE_BLUE),),
    mass=g.TILE_MASS,
    friction=g.TILE_FRICTION,
    color=g.COLOR_TILE_BLUE,
)

TILE_ORANGE = part(
    "tile_orange",
    pos=g.TILE_ORANGE_START,
    boxes=(((0.0, 0.0, 0.0), g.TILE_DIMS, g.COLOR_TILE_ORANGE),),
    mass=g.TILE_MASS,
    friction=g.TILE_FRICTION,
    color=g.COLOR_TILE_ORANGE,
)

TILE_YELLOW = part(
    "tile_yellow",
    pos=g.TILE_YELLOW_START,
    boxes=(((0.0, 0.0, 0.0), g.TILE_DIMS, g.COLOR_TILE_YELLOW),),
    mass=g.TILE_MASS,
    friction=g.TILE_FRICTION,
    color=g.COLOR_TILE_YELLOW,
)

OBJECTS = [
    FRAME,
    CARD,
    TILE_BLUE,
    TILE_ORANGE,
    TILE_YELLOW,
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
    blue = objects["tile_blue"]
    orange = objects["tile_orange"]
    yellow = objects["tile_yellow"]

    arm_tcps = _arm_info(e)

    result = _SCORER.update(
        blue_pos=blue.pos,
        blue_quat=blue.quat_xyzw,
        blue_vel=blue.lin_vel,
        orange_pos=orange.pos,
        orange_quat=orange.quat_xyzw,
        orange_vel=orange.lin_vel,
        yellow_pos=yellow.pos,
        yellow_quat=yellow.quat_xyzw,
        yellow_vel=yellow.lin_vel,
        arm_tcps=arm_tcps,
    )

    _trace({
        "score": result.score,
        "success": result.success,
        **result.details,
        "dbg_tcp_left": (arm_tcps or {}).get("left"),
        "dbg_tcp_right": (arm_tcps or {}).get("right"),
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

