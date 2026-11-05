"""Yam-arena task file: Read the status lamp's blink code, then swap the module it names.

On the station table:
- A fixed bus strip with four keyed sockets in a row (pitch 70 mm) with spring plunger latches.
- Beside each socket, white dot marks (1, 2, 3, 4 dots) identify the socket number.
- Four servo modules (grey bands) starting seated in the four sockets (module_3 in socket 3 has failed).
- One spare servo module (teal band) staged on the table.
- A DIAG controller box with a white DIAG button and a 60 mm diameter status lamp.
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo_plan
import geometry as g
from parts import Slider, env, gripper_open, on_reset, part, write_pose
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "While you hold the white DIAG button the status lamp blinks the number of the failed socket (count the dots beside each socket); replace that module with the teal spare, then hold DIAG again until the lamp stays green."

# 1. Sliders for the fixed fixture
# Four plungers (one per socket) on +y wall of socket cavity
PLUNGERS = []
for i, sy in enumerate(g.SOCKET_Y):
    plunger_x = g.MOUTH_X + g.PLUNGER_S
    plunger_y = sy + g.CAV_HX + g.PLUNGER_R - g.PLUNGER_PROUD
    rest_pt = (plunger_x, plunger_y, g.SOCKET_Z)
    s = Slider(
        f"plunger_{i}",
        rest=rest_pt,
        cylinders=((rest_pt, g.PLUNGER_R, g.PLUNGER_H, g.ALUMINIUM, "Z"),),
        travel=g.PLUNGER_TRAVEL,
        rate=g.PLUNGER_RATE,
        damping=g.PLUNGER_DAMPING,
        mass=g.PLUNGER_MASS,
        axis="+Y",
    )
    PLUNGERS.append(s)

# DIAG pushbutton on controller box
DIAG_BTN = Slider(
    "diag_btn",
    rest=g.DIAG_BTN_POS,
    boxes=((g.DIAG_BTN_POS, g.BUTTON_CAP_SIZE, g.WHITE),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis="Z",
)

# Fixed fixture: strip + sockets + controller box + plungers + diag button
FIXTURE = part(
    "fixture",
    pos=(0.0, 0.0, 0.0),
    yaw=0.0,
    boxes=g.fixture_boxes(),
    cylinders=g.fixture_cylinders(),
    mass=g.FIXTURE_MASS,
    sliders=(*PLUNGERS, DIAG_BTN),
    color=g.DARK_GREY,
    friction=g.FIXTURE_FRICTION,
)

# Four seated servo modules (grey bands)
MODULE_1 = part(
    "module_1",
    pos=g.START_POS["module_1"],
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.GREY),
    mass=g.SERVO_MASS,
    color=g.GREY,
    friction=g.SERVO_FRICTION,
)

MODULE_2 = part(
    "module_2",
    pos=g.START_POS["module_2"],
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.GREY),
    mass=g.SERVO_MASS,
    color=g.GREY,
    friction=g.SERVO_FRICTION,
)

MODULE_3 = part(
    "module_3",
    pos=g.START_POS["module_3"],
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.GREY),
    mass=g.SERVO_MASS,
    color=g.GREY,
    friction=g.SERVO_FRICTION,
)

MODULE_4 = part(
    "module_4",
    pos=g.START_POS["module_4"],
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.GREY),
    mass=g.SERVO_MASS,
    color=g.GREY,
    friction=g.SERVO_FRICTION,
)

# One spare module (teal band) staged on the table
SPARE = part(
    "spare",
    pos=g.SPARE_STAGE_POS,
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.TEAL),
    mass=g.SERVO_MASS,
    color=g.TEAL,
    friction=g.SERVO_FRICTION,
)

# Visual indicator markers (teleported with write_pose; collision=False)
LAMP_AMBER = part(
    "lamp_amber",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.LAMP_BLINK, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

LAMP_GREEN = part(
    "lamp_green",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.LAMP_GREEN, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

LAMP_RED = part(
    "lamp_red",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.LAMP_RED, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

OBJECTS = [
    FIXTURE,
    MODULE_1,
    MODULE_2,
    MODULE_3,
    MODULE_4,
    SPARE,
    LAMP_AMBER,
    LAMP_GREEN,
    LAMP_RED,
]

_SCORER = Scorer()
_last_states_ref = None
_last_result = None


@on_reset
def _reset_scorer(environment=None) -> None:
    global _last_states_ref, _last_result
    _last_states_ref = None
    _last_result = None
    _SCORER.reset()


def _trace(row: dict) -> None:
    path = os.environ.get("TASK_TRACE")
    if path:
        with open(path, "a") as f:
            f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")


def _arm_trace(e) -> dict:
    """Where each gripper's TCP and finger tips are (world frame, relative to the env origin)."""
    try:
        from parts import _to_numpy
        from yam_arena import runtime

        base = e.unwrapped
        origin = _to_numpy(base.scene.env_origins[0])
        out = {}
        for arm in ("left", "right"):
            robot = base.scene[f"{arm}_robot"]
            poses = runtime.link_poses(robot)
            out[f"{arm}_tcp"] = (runtime.tcp_world(robot) - origin).tolist()
            out[f"{arm}_gripper_x_axis"] = poses["gripper"][:3, 0].tolist()
            out[f"{arm}_gripper_y_axis"] = poses["gripper"][:3, 1].tolist()
            out[f"{arm}_gripper_z_axis"] = poses["gripper"][:3, 2].tolist()
            names = list(robot.joint_names)
            q = _to_numpy(robot.data.joint_pos)[0]
            out[f"{arm}_finger_q"] = [float(q[names.index(j)]) for j in ("joint7", "joint8")]
            for tip in ("tip_left", "tip_right"):
                out[f"{arm}_{tip}"] = (poses[tip][:3, 3] - origin).tolist()
        return out
    except Exception as exc:  # noqa: BLE001
        return {"arm_trace_error": repr(exc)}


def _update(objects):
    global _last_states_ref, _last_result
    if objects is _last_states_ref and _last_result is not None:
        return _last_result

    e = env()
    fixture = objects["fixture"]
    modules_data = {
        name: (
            objects[name].pos,
            objects[name].quat_xyzw,
            objects[name].lin_vel,
            objects[name].ang_vel,
        )
        for name in ("module_1", "module_2", "module_3", "module_4", "spare")
    }

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    result = _SCORER.update(
        fixture.pos,
        fixture.quat_xyzw,
        fixture.sliders,
        modules_data,
        left_open and right_open,
    )

    # Teleport visual indicator lamps based on logic state
    lamp_state = result.details.get("lamp_state", "off")
    if e is not None:
        if lamp_state == "blink":
            write_pose(e, "lamp_amber", g.STATUS_LAMP_LIT_POS)
            write_pose(e, "lamp_green", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "lamp_red", (0.0, 0.0, g.STOWED_Z))
        elif lamp_state == "green":
            write_pose(e, "lamp_green", g.STATUS_LAMP_LIT_POS)
            write_pose(e, "lamp_amber", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "lamp_red", (0.0, 0.0, g.STOWED_Z))
        elif lamp_state == "red":
            write_pose(e, "lamp_red", g.STATUS_LAMP_LIT_POS)
            write_pose(e, "lamp_amber", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "lamp_green", (0.0, 0.0, g.STOWED_Z))
        else:
            write_pose(e, "lamp_amber", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "lamp_green", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "lamp_red", (0.0, 0.0, g.STOWED_Z))

    _trace({
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "fixture_sliders": fixture.sliders,
        "module_pos": {n: list(modules_data[n][0]) for n in modules_data},
        "module_quat": {n: list(modules_data[n][1]) for n in modules_data},
        "fixture_pos": list(fixture.pos),
        **_arm_trace(e),
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
