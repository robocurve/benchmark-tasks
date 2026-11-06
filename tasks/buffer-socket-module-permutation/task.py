"""Yam-arena task file: Permute mis-racked keyed modules using a buffer socket.

On the station table:
- A rigid aluminium-and-steel bus strip with four keyed sockets in a row (pitch 70 mm)
  with spring plunger detents. Sockets 1-3 have coloured lamp discs (red, green, blue)
  showing the required module band colour; socket 4 has no lamp (free buffer socket).
- Three keyed plug modules with red, green, and blue central bands, starting seated so
  sockets 1 and 2 are swapped:
    * Socket 1 holds green module (requires red)
    * Socket 2 holds red module (requires green)
    * Socket 3 holds blue module (matching)
    * Socket 4 is empty (buffer)
- A controller box with a white COMMIT button (8 mm travel, 100 N/m spring) and
  a 50 mm diameter STATUS lamp (green when all 3 sockets match and no module is displaced;
  red otherwise).
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

INSTRUCTION = "Move the mis-racked modules between sockets—using the empty fourth socket as a buffer—so each module's coloured band matches its socket lamp disc, then press the white COMMIT button on the left controller box."

# 1. Sliders for the fixed fixture
# Four spring plungers (one per socket) on +y wall of socket cavity
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

# COMMIT pushbutton on controller box
COMMIT_BTN = Slider(
    "commit_btn",
    rest=g.COMMIT_BTN_POS,
    boxes=((g.COMMIT_BTN_POS, g.BUTTON_CAP_SIZE, g.WHITE),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis="Z",
)

# Fixed fixture: strip + sockets + controller box + plungers + commit button
FIXTURE = part(
    "fixture",
    pos=(0.0, 0.0, 0.0),
    yaw=0.0,
    boxes=g.fixture_boxes(),
    cylinders=g.fixture_cylinders(),
    mass=g.FIXTURE_MASS,
    sliders=(*PLUNGERS, COMMIT_BTN),
    color=g.DARK_GREY,
    friction=g.FIXTURE_FRICTION,
)

# Three keyed plug modules
MODULE_GREEN = part(
    "module_green",
    pos=g.START_POS["module_green"],
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.BAND_GREEN),
    mass=g.MODULE_MASS,
    color=g.BAND_GREEN,
    friction=g.MODULE_FRICTION,
)

MODULE_RED = part(
    "module_red",
    pos=g.START_POS["module_red"],
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.BAND_RED),
    mass=g.MODULE_MASS,
    color=g.BAND_RED,
    friction=g.MODULE_FRICTION,
)

MODULE_BLUE = part(
    "module_blue",
    pos=g.START_POS["module_blue"],
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.BAND_BLUE),
    mass=g.MODULE_MASS,
    color=g.BAND_BLUE,
    friction=g.MODULE_FRICTION,
)

# Visual indicator markers (teleported with write_pose; collision=False)
LAMP_STATUS_RED = part(
    "lamp_status_red",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.STATUS_LAMP_RED, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

LAMP_STATUS_GREEN = part(
    "lamp_status_green",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.STATUS_LAMP_GREEN, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

OBJECTS = [
    FIXTURE,
    MODULE_GREEN,
    MODULE_RED,
    MODULE_BLUE,
    LAMP_STATUS_RED,
    LAMP_STATUS_GREEN,
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
    """Where each gripper's TCP and finger tips are (world frame, relative to env origin)."""
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
        for name in g.MODULE_NAMES
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
    lamp_state = result.details.get("lamp_state", "red")
    if e is not None:
        if lamp_state == "green":
            write_pose(e, "lamp_status_green", g.STATUS_LAMP_LIT_POS)
            write_pose(e, "lamp_status_red", (0.0, 0.0, g.STOWED_Z))
        else:
            write_pose(e, "lamp_status_red", g.STATUS_LAMP_LIT_POS)
            write_pose(e, "lamp_status_green", (0.0, 0.0, g.STOWED_Z))

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
