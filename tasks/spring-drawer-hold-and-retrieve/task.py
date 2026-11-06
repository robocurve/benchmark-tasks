"""Yam-arena task file: Hold a spring-return drawer open while retrieving an enclosed block.

Story:
Parts cabinet dispensing:
A spring-loaded drawer conceals a white block in its rear compartment under a top cover plate.
The drawer must be pulled open against an 18 N/m return spring. Left arm hooks the handle
and holds the drawer fully open (>= 90 mm lights the cyan indicator lamp). Right arm reaches
inside the open compartment, retrieves the white block, and places it on the green shelf.
Left arm then guides the drawer closed smoothly (< 5 mm).
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

INSTRUCTION = "Pull the drawer fully open and keep it open while you take the white block out and put it on the green shelf, then let the drawer close."

# 1. Cabinet fixture with spring-loaded drawer slider
DRAWER_LOCAL_REST = (g.DRAWER_REST_X - g.CABINET_POS[0], 0.0, g.DRAWER_REST_Z - g.CABINET_POS[2])
HANDLE_LOCAL_REST = (g.HANDLE_REST_X - g.CABINET_POS[0], 0.0, g.HANDLE_REST_Z - g.CABINET_POS[2])

DRAWER_SLIDER = Slider(
    "drawer_slider",
    rest=DRAWER_LOCAL_REST,
    boxes=tuple(
        ((c[0] - g.CABINET_POS[0], c[1] - g.CABINET_POS[1], c[2] - g.CABINET_POS[2]), size, colour)
        for c, size, colour in g.drawer_world_boxes()
    ) + ((HANDLE_LOCAL_REST, g.HANDLE_SIZE, g.HANDLE_ORANGE),),
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

    return part(name, pos, boxes=tuple((shift(c), s, col) for c, s, col in boxes),
                cylinders=tuple((shift(c), *rest) for c, *rest in cylinders), **kw)


CABINET_FIXTURE = world_part(
    "cabinet_fixture",
    pos=g.CABINET_POS,
    yaw=0.0,
    boxes=tuple(g.cabinet_world_boxes()),
    mass=8.0,
    sliders=(DRAWER_SLIDER,),
    color=g.CABINET_GREY,
    friction=1.0,
)

# 2. Static fixtures
SHELF = world_part(
    "green_shelf",
    pos=g.SHELF_POS,
    yaw=0.0,
    boxes=((g.SHELF_POS, g.SHELF_SIZE, g.GREEN_SHELF),),
    mass=3.0,
    friction=1.0,
)

# 3. Open-position cyan lamp marker on cabinet top
CYAN_LAMP = part(
    "cyan_lamp",
    pos=g.LAMP_POS,
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), g.LAMP_SIZE, g.LAMP_CYAN),),
    collision=False,
)

# 4. White block inside drawer
WHITE_BLOCK = world_part(
    "white_block",
    pos=g.BLOCK_START_POS,
    yaw=0.0,
    boxes=((g.BLOCK_START_POS, g.BLOCK_SIZE, g.WHITE_BLOCK),),
    mass=g.BLOCK_MASS,
    friction=0.8,
)

OBJECTS = [
    CABINET_FIXTURE,
    SHELF,
    CYAN_LAMP,
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
    drawer_disp = abs(float(cabinet.sliders.get("drawer_slider", 0.0)))  # the -X slider reports a negative value when pulled out
    block_pos = objects["white_block"].pos

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    arm_tcps, arm_vels = _arm_info(e)

    result = _SCORER.update(
        drawer_disp=drawer_disp,
        block_pos=block_pos,
        grippers_open=left_open and right_open,
        arm_tcps=arm_tcps,
        arm_velocities=arm_vels,
    )

    if e is not None:
        lamp_z = g.LAMP_ON_Z if result.details.get("cyan_lamp") else g.LAMP_OFF_Z
        write_pose(e, "cyan_lamp", (g.LAMP_POS[0], g.LAMP_POS[1], lamp_z))

    _trace({
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "drawer_disp": drawer_disp,
        **result.details,
        "dbg_block": [round(float(v), 3) for v in block_pos],
        "dbg_tcps": {a: [round(float(v), 3) for v in arm_tcps[a]] for a in (arm_tcps or {})},
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

