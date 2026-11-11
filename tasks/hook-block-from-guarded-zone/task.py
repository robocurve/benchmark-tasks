"""Yam-arena task file: Pull block from guarded zone using hooked rod into tray without crossing line.

Instruction:
Pull the block out of the guarded zone with the hooked rod and set it in the tray, never letting a gripper cross the yellow line.
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
from parts import env, on_reset, on_step, part
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = g.HOLD_TIME_S

INSTRUCTION = "Pull the block out of the guarded zone with the hooked rod and set it in the tray, never letting a gripper cross the yellow line."


def world_part(name, pos, boxes=(), cylinders=(), **kw):
    """Like part(), but the shapes are given at their WORLD positions: shift them into the body frame here."""
    px, py, pz = pos

    def shift(c):
        return (c[0] - px, c[1] - py, c[2] - pz)

    return part(
        name,
        pos,
        boxes=tuple((shift(c), s, col) for c, s, col in boxes),
        cylinders=tuple((shift(c), *rest) for c, *rest in cylinders),
        **kw,
    )


# 1. Yellow Line Marker (flat stripe on table surface at x = 0.40)
YELLOW_LINE = part(
    "yellow_line",
    pos=g.LINE_POS,
    boxes=(((0.0, 0.0, 0.0), g.LINE_SIZE, g.YELLOW_LINE_COL),),
    mass=0.010,
    yaw=0.0,
    collision=False,
)

# 2. Green Tray Receptacle (130 x 130 x 20 mm)
GREEN_TRAY = world_part(
    "green_tray",
    pos=g.TRAY_POS,
    boxes=tuple(g.tray_world_boxes()),
    mass=2.0,
    yaw=0.0,
    color=g.GREEN_TRAY_COL,
    friction=1.0,
)

# 3. Hooked Rod (shaft 300x20x20 mm + cross-piece 20x60x20 mm at far end)
HOOKED_ROD = part(
    "hooked_rod",
    pos=g.ROD_INIT_POS,
    boxes=tuple(g.rod_local_boxes()),
    mass=g.ROD_MASS,
    yaw=0.0,
    friction=g.ROD_FRICTION,
    color=g.GREEN_ROD_COL,
)

# 4. Red Target Block (40 x 40 x 40 mm at (0.58, 0.0))
RED_BLOCK = part(
    "red_block",
    pos=g.BLOCK_INIT_POS,
    boxes=(((0.0, 0.0, 0.0), g.BLOCK_SIZE, g.RED_BLOCK_COL),),
    mass=g.BLOCK_MASS,
    yaw=0.0,
    friction=g.BLOCK_FRICTION,
    color=g.RED_BLOCK_COL,
)

OBJECTS = [
    YELLOW_LINE,
    GREEN_TRAY,
    HOOKED_ROD,
    RED_BLOCK,
]

_SCORER = Scorer(hold_s=SUCCESS_HOLD_S)
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
        return None, None
    try:
        from parts import _to_numpy
        from yam_arena import runtime

        base = e.unwrapped
        origin = _to_numpy(base.scene.env_origins[0])
        tcps = {}
        vels = {}
        for arm in ("left", "right"):
            robot = base.scene[f"{arm}_robot"]
            tcps[arm] = (runtime.tcp_world(robot) - origin).tolist()
            lv = _to_numpy(robot.data.root_link_lin_vel_w)[0]
            av = _to_numpy(robot.data.root_link_ang_vel_w)[0]
            vels[arm] = (lv, av)
        return tcps, vels
    except Exception:
        return None, None


def _update(objects):
    global _last_states_ref, _last_result
    if objects is _last_states_ref and _last_result is not None:
        return _last_result

    e = env()
    block = objects["red_block"]
    rod = objects["hooked_rod"]

    block_pos = block.pos
    rod_pos = rod.pos
    block_vel = getattr(block, "vel", (0.0, 0.0, 0.0))
    rod_vel = getattr(rod, "vel", (0.0, 0.0, 0.0))

    arm_tcps, arm_vels = _arm_info(e)

    result = _SCORER.update(
        block_pos=block_pos,
        rod_pos=rod_pos,
        block_vel=block_vel,
        rod_vel=rod_vel,
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
