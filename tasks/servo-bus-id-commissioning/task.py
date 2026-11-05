"""Yam-arena task file: Commissioning three servo module IDs on a shared bus.

On the station table:
- A fixed bus strip with three keyed sockets in a row (pitch 70 mm) with spring plunger latches.
- Three servo modules (red, green, blue) starting plugged into the three sockets.
- A controller box with an ASSIGN button (white), a RESET button (yellow), a bus status lamp (red/green),
  and a 3x3 readout board indicating module IDs.
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

INSTRUCTION = "Three new servo modules (red, green, blue) are plugged into the bus and the bus status lamp is red. Press the white ASSIGN button on the front of the controller box (numbers the single un-numbered servo on the bus; yellow RESET button behind it clears numbers) until red is 1, green is 2, blue is 3 (the 3 x 3 readout board lights each servo's number in its colour row, columns 1 to 3), all three are plugged in, and the lamp turns green."

# 1. Sliders for the fixed fixture
# Three plungers (one per socket) on +y wall of socket cavity
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

# Two pushbuttons on controller box
ASSIGN_BTN = Slider(
    "assign_btn",
    rest=g.ASSIGN_BTN_POS,
    boxes=((g.ASSIGN_BTN_POS, g.BUTTON_CAP_SIZE, g.WHITE),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis="Z",
)

RESET_BTN = Slider(
    "reset_btn",
    rest=g.RESET_BTN_POS,
    boxes=((g.RESET_BTN_POS, g.BUTTON_CAP_SIZE, g.YELLOW),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis="Z",
)

# Fixed fixture: strip + sockets + controller + buttons + plungers
FIXTURE = part(
    "fixture",
    pos=(0.0, 0.0, 0.0),
    yaw=0.0,
    boxes=g.fixture_boxes(),
    cylinders=g.fixture_cylinders(),
    mass=g.FIXTURE_MASS,
    sliders=(*PLUNGERS, ASSIGN_BTN, RESET_BTN),
    color=g.DARK_GREY,
    friction=g.FIXTURE_FRICTION,
)

# Three servo modules (plugs)
SERVO_RED = part(
    "servo_red",
    pos=g.START_POS["red"],
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.RED),
    mass=g.SERVO_MASS,
    color=g.RED,
    friction=g.SERVO_FRICTION,
)

SERVO_GREEN = part(
    "servo_green",
    pos=g.START_POS["green"],
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.GREEN),
    mass=g.SERVO_MASS,
    color=g.GREEN,
    friction=g.SERVO_FRICTION,
)

SERVO_BLUE = part(
    "servo_blue",
    pos=g.START_POS["blue"],
    yaw=g.PLUG_YAW_SEATED,
    boxes=g.servo_boxes(g.BLUE),
    mass=g.SERVO_MASS,
    color=g.BLUE,
    friction=g.SERVO_FRICTION,
)

# Visual indicator markers (teleported between visible position and under-table storage; collision=False)
BUS_LAMP_GREEN = part(
    "bus_lamp_green",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.LAMP_BRIGHT_GREEN, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

READOUT_RED = part(
    "readout_red",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.READOUT_LAMP_LIT_R, g.READOUT_LAMP_LIT_H, g.RED, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

READOUT_GREEN = part(
    "readout_green",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.READOUT_LAMP_LIT_R, g.READOUT_LAMP_LIT_H, g.GREEN, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

READOUT_BLUE = part(
    "readout_blue",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.READOUT_LAMP_LIT_R, g.READOUT_LAMP_LIT_H, g.BLUE, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

OBJECTS = [
    FIXTURE,
    SERVO_RED,
    SERVO_GREEN,
    SERVO_BLUE,
    BUS_LAMP_GREEN,
    READOUT_RED,
    READOUT_GREEN,
    READOUT_BLUE,
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
    """Where each gripper's TCP and finger tips are (world frame, relative to the env origin), for the trace only: it
    never reaches the scorer, and a failure to read it is recorded instead of raised."""
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
            out[f"{arm}_gripper_x_axis"] = poses["gripper"][:3, 0].tolist()  # the gripper link's own x, y, z axes in the world
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
    servos_data = {
        name: (
            objects[f"servo_{name}"].pos,
            objects[f"servo_{name}"].quat_xyzw,
            objects[f"servo_{name}"].lin_vel,
            objects[f"servo_{name}"].ang_vel,
        )
        for name in ("red", "green", "blue")
    }

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    result = _SCORER.update(
        fixture.pos,
        fixture.quat_xyzw,
        fixture.sliders,
        servos_data,
        left_open and right_open,
    )

    # Teleport visual indicators based on hidden logic state
    bus_lamp = result.details.get("bus_lamp", "red")
    ids = result.details.get("ids", {})

    if e is not None:
        # Bus status lamp (green marker covers red base dome when green)
        if bus_lamp == "green":
            write_pose(e, "bus_lamp_green", g.STATUS_LAMP_LIT_POS)
        else:
            write_pose(e, "bus_lamp_green", (0.0, 0.0, g.STOWED_Z))

        # Readout board indicators
        for s in ("red", "green", "blue"):
            col = ids.get(s)
            marker_name = f"readout_{s}"
            if col in (1, 2, 3):
                pos = (g.READOUT_ROWS[s], g.READOUT_COLS[col], g.READOUT_LAMP_LIT_Z)
                write_pose(e, marker_name, pos)
            else:
                write_pose(e, marker_name, (0.0, 0.0, g.STOWED_Z))

    _trace({
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "fixture_sliders": fixture.sliders,
        "servo_pos": {n: list(servos_data[n][0]) for n in servos_data},
        "servo_quat": {n: list(servos_data[n][1]) for n in servos_data},
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
