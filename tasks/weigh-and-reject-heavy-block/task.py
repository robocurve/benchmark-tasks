"""Yam-arena task file: Weigh three grey blocks on spring scale and reject the heavy one.

Story:
Incoming QC / inspection station:
Three grey blocks look identical to the camera, but one is dense (0.30 kg vs 0.10 kg).
A spring scale platform deflects under block mass, moving a needle along an indicator bar.
Left arm transfers blocks from the supply rack to the scale; the heavy block is relayed
to the right arm to be placed in the red reject tray, while light blocks are returned to
the green shelf.
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

INSTRUCTION = "Weigh each grey block on the scale, then put only the heavy one into the red reject tray, and leave the other two on the green shelf."

# 1. Scale fixture with spring-loaded platform slider
SCALE_SLIDER = Slider(
    "scale_platform",
    rest=g.SCALE_PLATFORM_LOCAL_REST,
    boxes=((g.SCALE_PLATFORM_LOCAL_REST, g.SCALE_PLATFORM_SIZE, g.SCALE_PLATFORM),),
    travel=g.SCALE_TRAVEL,
    rate=g.SCALE_SPRING_K,
    damping=g.SCALE_DAMPING,
    mass=g.SCALE_PLATFORM_MASS,
    axis="Z",
)

def world_part(name, pos, boxes=(), cylinders=(), **kw):
    """Like part(), but the shapes are given at their WORLD positions: shift them into the body frame here."""
    px, py, pz = pos

    def shift(c):
        return (c[0] - px, c[1] - py, c[2] - pz)

    return part(name, pos, boxes=tuple((shift(c), s, col) for c, s, col in boxes),
                cylinders=tuple((shift(c), *rest) for c, *rest in cylinders), **kw)


SCALE_FIXTURE = world_part(
    "scale_fixture",
    pos=g.SCALE_BASE_POS,
    yaw=0.0,
    boxes=((g.SCALE_BASE_POS, g.SCALE_BASE_SIZE, g.SCALE_HOUSING),),
    mass=6.0,
    sliders=(SCALE_SLIDER,),
    color=g.SCALE_HOUSING,
    friction=1.0,
)

# 2. Static fixtures
SUPPLY_RACK = world_part(
    "supply_rack",
    pos=g.SUPPLY_RACK_POS,
    yaw=0.0,
    boxes=((g.SUPPLY_RACK_POS, g.SUPPLY_RACK_SIZE, g.DARK_GREY),),
    mass=3.0,
    friction=1.0,
)

SHELF = world_part(
    "green_shelf",
    pos=g.SHELF_POS,
    yaw=0.0,
    boxes=((g.SHELF_POS, g.SHELF_SIZE, g.GREEN_SHELF),),
    mass=3.0,
    friction=1.0,
)

TRAY = world_part(
    "reject_tray",
    pos=g.TRAY_POS,
    yaw=0.0,
    boxes=((g.TRAY_POS, g.TRAY_SIZE, g.RED_TRAY),),
    mass=3.0,
    friction=1.0,
)

# Readout meter housing
METER_FIXTURE = world_part(
    "meter_fixture",
    pos=g.METER_POS,
    yaw=0.0,
    boxes=((g.METER_POS, g.METER_SIZE, g.DARK_GREY),),
    mass=2.0,
    friction=1.0,
)

# 3. Readout needle marker (teleported via write_pose, collision=False)
DUMMY_BOX = (((0.0, 0.0, 0.0), (0.002, 0.002, 0.002), (0.0, 0.0, 0.0)),)

NEEDLE_MARKER = part(
    "needle_marker",
    pos=(g.METER_POS[0], g.METER_POS[1], g.NEEDLE_Z),
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), g.NEEDLE_SIZE, g.NEEDLE_COLOUR),),
    collision=False,
)

# 4. Three grey blocks
BLOCKS = {}
for b_name in g.BLOCK_NAMES:
    mass = g.BLOCK_MASSES[b_name]
    pos = g.BLOCK_SUPPLY_POS[b_name]
    BLOCKS[b_name] = world_part(
        b_name,
        pos=pos,
        yaw=0.0,
        boxes=((pos, g.BLOCK_SIZE, g.GREY_BLOCK),),
        mass=mass,
        friction=0.9,
    )

OBJECTS = [
    SCALE_FIXTURE,
    SUPPLY_RACK,
    SHELF,
    TRAY,
    METER_FIXTURE,
    NEEDLE_MARKER,
    *BLOCKS.values(),
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
    scale_fixture = objects["scale_fixture"]
    scale_disp = float(scale_fixture.sliders.get("scale_platform", 0.0))

    block_positions = {b: objects[b].pos for b in g.BLOCK_NAMES}

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    arm_tcps, arm_vels = _arm_info(e)

    result = _SCORER.update(
        scale_disp=scale_disp,
        block_positions=block_positions,
        grippers_open=left_open and right_open,
        arm_tcps=arm_tcps,
        arm_velocities=arm_vels,
    )

    # Teleport needle marker according to sag
    if e is not None:
        needle_y = g.METER_POS[1] - abs(scale_disp) * g.READOUT_GAIN
        write_pose(e, "needle_marker", (g.METER_POS[0], needle_y, g.NEEDLE_Z))

    _trace({
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "scale_disp": scale_disp,
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
