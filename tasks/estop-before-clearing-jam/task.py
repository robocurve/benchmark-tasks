"""Yam-arena task file: Emergency stop and jam clearing on cycling pusher machine.

Instruction:
Press the red E-STOP so the pusher stops, take the stuck orange block out of the lane, then press RESET and START: never reach into the lane while the pusher is moving.
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
SUCCESS_HOLD_S = 2.0

INSTRUCTION = "Press the red E-STOP so the pusher stops, take the stuck orange block out of the lane, then press RESET and START: never reach into the lane while the pusher is moving."


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


# 1. Pushbuttons Console Fixture
ESTOP_LOCAL_REST = (
    g.ESTOP_REST_POS[0] - g.CONSOLE_BASE_POS[0],
    g.ESTOP_REST_POS[1] - g.CONSOLE_BASE_POS[1],
    g.ESTOP_REST_POS[2] - g.CONSOLE_BASE_POS[2],
)
RESET_LOCAL_REST = (
    g.RESET_REST_POS[0] - g.CONSOLE_BASE_POS[0],
    g.RESET_REST_POS[1] - g.CONSOLE_BASE_POS[1],
    g.RESET_REST_POS[2] - g.CONSOLE_BASE_POS[2],
)
START_LOCAL_REST = (
    g.START_REST_POS[0] - g.CONSOLE_BASE_POS[0],
    g.START_REST_POS[1] - g.CONSOLE_BASE_POS[1],
    g.START_REST_POS[2] - g.CONSOLE_BASE_POS[2],
)

ESTOP_SLIDER = Slider(
    "estop_button_slider",
    rest=ESTOP_LOCAL_REST,
    boxes=((ESTOP_LOCAL_REST, g.ESTOP_CAP_SIZE, g.RED_BTN),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis="-Z",
)

RESET_SLIDER = Slider(
    "reset_button_slider",
    rest=RESET_LOCAL_REST,
    boxes=((RESET_LOCAL_REST, g.RESET_CAP_SIZE, g.WHITE_BTN),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis="-Z",
)

START_SLIDER = Slider(
    "start_button_slider",
    rest=START_LOCAL_REST,
    boxes=((START_LOCAL_REST, g.START_CAP_SIZE, g.GREEN_BTN),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis="-Z",
)

CONSOLE_FIXTURE = world_part(
    "console_fixture",
    pos=g.CONSOLE_BASE_POS,
    yaw=0.0,
    boxes=tuple(g.console_world_boxes()),
    cylinders=tuple(g.console_world_cylinders()),
    mass=8.0,
    sliders=(ESTOP_SLIDER, RESET_SLIDER, START_SLIDER),
    color=g.DARK_GREY,
    friction=1.0,
)

# 2. Lane Fixture
LANE_FIXTURE = world_part(
    "lane_fixture",
    pos=(g.LANE_POS[0], g.LANE_POS[1], g.TOP + 0.010),
    yaw=0.0,
    boxes=tuple(g.lane_world_boxes()),
    mass=10.0,
    color=g.GREY,
    friction=1.0,
)

# 3. Green Tray
GREEN_TRAY = world_part(
    "green_tray",
    pos=g.TRAY_POS,
    yaw=0.0,
    boxes=tuple(g.tray_world_boxes()),
    mass=2.0,
    color=g.GREEN_TRAY_COL,
    friction=1.0,
)

# 4. Parked Pad (alternative staging destination)
PARKED_PAD = part(
    "parked_pad",
    pos=g.PARK_PAD_POS,
    yaw=0.0,
    boxes=(((0.0, 0.0, 0.0), g.PARK_PAD_SIZE, g.STEEL_GREY),),
    mass=1.0,
    friction=1.0,
)

# 5. Stuck Orange Block
ORANGE_BLOCK = part(
    "orange_block",
    pos=g.BLOCK_INIT_POS,
    yaw=0.0,
    boxes=(((0.0, 0.0, 0.0), g.BLOCK_SIZE, g.ORANGE_BLOCK),),
    mass=g.BLOCK_MASS,
    friction=g.BLOCK_FRICTION,
)

# 6. Pusher Bar (Code-driven non-colliding visual marker)
PUSHER = part(
    "pusher",
    pos=g.PUSHER_INIT_POS,
    mass=0.010,
    boxes=(((0.0, 0.0, 0.0), g.PUSHER_SIZE, g.STEEL_GREY),),
    collision=False,
)

# 7. Indicator Lamps (display markers teleported via write_pose)
RUN_LAMP = part(
    "run_lamp",
    pos=g.RUN_LAMP_LIT_POS,
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), (0.024, 0.024, 0.004), g.LAMP_GREEN),),
    collision=False,
)

STOP_LAMP = part(
    "stop_lamp",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), (0.024, 0.024, 0.004), g.LAMP_RED),),
    collision=False,
)

FAULT_LAMP = part(
    "fault_lamp",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), (0.024, 0.024, 0.004), g.LAMP_AMBER),),
    collision=False,
)

OBJECTS = [
    CONSOLE_FIXTURE,
    LANE_FIXTURE,
    GREEN_TRAY,
    PARKED_PAD,
    ORANGE_BLOCK,
    PUSHER,
    RUN_LAMP,
    STOP_LAMP,
    FAULT_LAMP,
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
    console = objects["console_fixture"]
    estop_disp = float(console.sliders.get("estop_button_slider", 0.0))
    reset_disp = float(console.sliders.get("reset_button_slider", 0.0))
    start_disp = float(console.sliders.get("start_button_slider", 0.0))

    block_pos = objects["orange_block"].pos

    arm_tcps, arm_vels = _arm_info(e)

    result = _SCORER.update(
        estop_disp=estop_disp,
        reset_disp=reset_disp,
        start_disp=start_disp,
        block_pos=block_pos,
        arm_tcps=arm_tcps,
    )

    if e is not None:
        # 1. Update code-driven pusher position
        pusher_x = result.details.get("pusher_x", g.PUSHER_X_MIN)
        write_pose(e, "pusher", (pusher_x, g.LANE_POS[1], g.PUSHER_Z))

        # 2. Update lamps
        run_z = g.RUN_LAMP_LIT_POS[2] if result.details.get("run_lamp_lit") else g.STOWED_Z
        stop_z = g.STOP_LAMP_LIT_POS[2] if result.details.get("stop_lamp_lit") else g.STOWED_Z
        fault_z = g.FAULT_LAMP_LIT_POS[2] if result.details.get("fault_lamp_lit") else g.STOWED_Z

        write_pose(e, "run_lamp", (g.RUN_LAMP_POS[0], g.RUN_LAMP_POS[1], run_z))
        write_pose(e, "stop_lamp", (g.STOP_LAMP_POS[0], g.STOP_LAMP_POS[1], stop_z))
        write_pose(e, "fault_lamp", (g.FAULT_LAMP_POS[0], g.FAULT_LAMP_POS[1], fault_z))

    _trace({
        "score": result.score,
        "success": result.success,
        "estop_disp": round(estop_disp, 4),
        "reset_disp": round(reset_disp, 4),
        "start_disp": round(start_disp, 4),
        **result.details,
        "dbg_block": [round(float(v), 4) for v in block_pos],
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
