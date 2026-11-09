"""Yam-arena task file: Interlocked drawer transfer.

Story:
A filing cabinet with a mechanical anti-tip interlock:
Opening a second drawer while one drawer is already open causes the cabinet to tip over
(TIP FAULT, illuminating the red TIP indicator lamp and latching the score).
The robot must move a white block from the left drawer to the right drawer, ensuring
that both drawers are never open at the same time.
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
from parts import Slider, env, gripper_open, on_reset, on_step, part, write_pose
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Move the white block from the left drawer to the right drawer, never letting both drawers be open at the same time."

# 1. Cabinet fixture with left and right drawer sliders
DRAWER_LEFT_LOCAL_REST = (g.DRAWER_REST_X - g.CABINET_POS[0], g.DRAWER_LEFT_Y - g.CABINET_POS[1], g.DRAWER_REST_Z - g.CABINET_POS[2])
DRAWER_RIGHT_LOCAL_REST = (g.DRAWER_REST_X - g.CABINET_POS[0], g.DRAWER_RIGHT_Y - g.CABINET_POS[1], g.DRAWER_REST_Z - g.CABINET_POS[2])

HANDLE_LEFT_LOCAL_REST = (g.HANDLE_REST_X - g.CABINET_POS[0], g.DRAWER_LEFT_Y - g.CABINET_POS[1], g.HANDLE_REST_Z - g.CABINET_POS[2])
HANDLE_RIGHT_LOCAL_REST = (g.HANDLE_REST_X - g.CABINET_POS[0], g.DRAWER_RIGHT_Y - g.CABINET_POS[1], g.HANDLE_REST_Z - g.CABINET_POS[2])

DRAWER_LEFT_SLIDER = Slider(
    "drawer_left",
    rest=DRAWER_LEFT_LOCAL_REST,
    boxes=tuple(
        ((c[0] - g.CABINET_POS[0], c[1] - g.CABINET_POS[1], c[2] - g.CABINET_POS[2]), size, colour)
        for c, size, colour in g.drawer_world_boxes(g.DRAWER_LEFT_Y)
    ) + ((HANDLE_LEFT_LOCAL_REST, g.HANDLE_SIZE, g.HANDLE_ORANGE),),
    travel=g.DRAWER_TRAVEL,
    rate=g.DRAWER_SPRING_K,
    damping=g.DRAWER_DAMPING,
    mass=g.DRAWER_MASS,
    axis="-X",
)

DRAWER_RIGHT_SLIDER = Slider(
    "drawer_right",
    rest=DRAWER_RIGHT_LOCAL_REST,
    boxes=tuple(
        ((c[0] - g.CABINET_POS[0], c[1] - g.CABINET_POS[1], c[2] - g.CABINET_POS[2]), size, colour)
        for c, size, colour in g.drawer_world_boxes(g.DRAWER_RIGHT_Y)
    ) + ((HANDLE_RIGHT_LOCAL_REST, g.HANDLE_SIZE, g.HANDLE_ORANGE),),
    travel=g.DRAWER_TRAVEL,
    rate=g.DRAWER_SPRING_K,
    damping=g.DRAWER_DAMPING,
    mass=g.DRAWER_MASS,
    axis="-X",
)


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


CABINET_FIXTURE = world_part(
    "cabinet_fixture",
    pos=g.CABINET_POS,
    yaw=0.0,
    boxes=tuple(g.cabinet_world_boxes()) + ((g.STAGING_PAD_POS, g.STAGING_PAD_SIZE, g.STAGING_PAD_COLOR),),
    mass=10.0,
    sliders=(DRAWER_LEFT_SLIDER, DRAWER_RIGHT_SLIDER),
    color=g.CABINET_GREY,
    friction=1.0,
)

# 2. Indicator lamps (OK green, TIP red)
LAMP_OK = part(
    "lamp_ok",
    pos=g.LAMP_OK_POS,
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), g.LAMP_SIZE, g.LAMP_OK_GREEN),),
    collision=False,
)

LAMP_TIP = part(
    "lamp_tip",
    pos=(g.LAMP_TIP_POS[0], g.LAMP_TIP_POS[1], g.LAMP_OFF_Z),
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), g.LAMP_SIZE, g.LAMP_TIP_RED),),
    collision=False,
)

# 3. White block
WHITE_BLOCK = world_part(
    "white_block",
    pos=g.BLOCK_START_POS,
    yaw=0.0,
    boxes=((g.BLOCK_START_POS, g.BLOCK_SIZE, g.WHITE_BLOCK),),
    mass=g.BLOCK_MASS,
    friction=g.BLOCK_FRICTION,
)

OBJECTS = [
    CABINET_FIXTURE,
    LAMP_OK,
    LAMP_TIP,
    WHITE_BLOCK,
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
    cabinet = objects["cabinet_fixture"]
    disp_left = abs(float(cabinet.sliders.get("drawer_left", 0.0)))
    disp_right = abs(float(cabinet.sliders.get("drawer_right", 0.0)))
    block_pos = objects["white_block"].pos

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    arm_tcps, arm_vels = _arm_info(e)

    result = _SCORER.update(
        disp_left=disp_left,
        disp_right=disp_right,
        block_pos=block_pos,
        grippers_open=left_open and right_open,
        arm_tcps=arm_tcps,
        arm_velocities=arm_vels,
    )

    if e is not None:
        ok_z = g.LAMP_ON_Z if result.details.get("lamp_ok") else g.LAMP_OFF_Z
        tip_z = g.LAMP_ON_Z if result.details.get("lamp_tip") else g.LAMP_OFF_Z
        write_pose(e, "lamp_ok", (g.LAMP_OK_POS[0], g.LAMP_OK_POS[1], ok_z))
        write_pose(e, "lamp_tip", (g.LAMP_TIP_POS[0], g.LAMP_TIP_POS[1], tip_z))

    _trace({
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "disp_left": disp_left,
        "disp_right": disp_right,
        **result.details,
        "dbg_block": [round(float(v), 3) for v in block_pos],
        "dbg_tcps": {a: [round(float(v), 3) for v in arm_tcps[a]] for a in (arm_tcps or {})},
        "dbg_left_tcp": [round(float(v), 4) for v in (arm_tcps.get("left") if arm_tcps else [0, 0, 0])],
        "dbg_right_tcp": [round(float(v), 4) for v in (arm_tcps.get("right") if arm_tcps else [0, 0, 0])],
        "dbg_left_drawer_disp_mm": round(disp_left * 1000.0, 2),
        "dbg_right_drawer_disp_mm": round(disp_right * 1000.0, 2),
        "dbg_tip_fault": bool(result.details.get("tip_fault", False)),
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


def plans(mode=None):
    return demo_plan.plans(mode)
