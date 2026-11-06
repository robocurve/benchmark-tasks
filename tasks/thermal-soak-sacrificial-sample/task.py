"""Yam-arena task file: Soak two samples to a target core temperature, learning the lag from the first.

On the station table:
- A fixed hot plate fixture with a 100 x 100 mm sample pad, red HEAT button and white OFF button.
- A cool pad on the right (200 x 100 x 5 mm, light blue) with space for two 60 mm blocks at 120 mm pitch.
- A staging area on the left with two sample blocks (60 x 60 x 40 mm, 0.3 kg each, blue = sample 1, orange = sample 2).
- A 9-disc temperature bar showing surface temperature only.
- A status lamp: off / amber (heating sample on plate) / green (target achieved).
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo_plan
import geometry as g
import parts
from parts import Slider, env, gripper_open, on_reset, part, read_world_pos, write_pose
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Press the red HEAT button on the front left to heat the plate to its 110 deg C setpoint, soak the orange sample so its core ends between 60 and 70 deg C, then leave it on the cool pad and press the white OFF button on the front right. While a sample rests on the cool pad, the status lamp indicates its core temperature band (amber < 60, green 60-70, red > 70 deg C) and the bar shows surface temperature; the blue expendable sample may be used to gauge the thermal lag."

# 1. Sliders for the fixed fixture (HEAT and OFF buttons)
HEAT_BTN = Slider(
    "heat_btn",
    rest=g.HEAT_BTN_POS,
    boxes=((g.HEAT_BTN_POS, g.BUTTON_CAP_SIZE, g.HEAT_RED),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis=g.BUTTON_AXIS,
)

OFF_BTN = Slider(
    "off_btn",
    rest=g.OFF_BTN_POS,
    boxes=((g.OFF_BTN_POS, g.BUTTON_CAP_SIZE, g.WHITE),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis=g.BUTTON_AXIS,
)

# 2. Fixed fixture: base, pad, button sleeves, indicators, cool pad, staging mat
FIXTURE = part(
    "fixture",
    pos=(0.0, 0.0, 0.0),
    yaw=0.0,
    boxes=g.fixture_boxes(),
    cylinders=g.fixture_cylinders(),
    mass=g.FIXTURE_MASS,
    sliders=(HEAT_BTN, OFF_BTN),
    color=g.DARK_GREY,
    friction=g.FIXTURE_FRICTION,
)

# 3. Two sample blocks
SAMPLE_1 = part(
    "sample_1",
    pos=g.START_POS_1,
    yaw=0.0,
    boxes=g.sample_boxes(g.BLUE),
    mass=g.SAMPLE_MASS,
    color=g.BLUE,
    friction=g.SAMPLE_FRICTION,
)

SAMPLE_2 = part(
    "sample_2",
    pos=g.START_POS_2,
    yaw=0.0,
    boxes=g.sample_boxes(g.ORANGE),
    mass=g.SAMPLE_MASS,
    color=g.ORANGE,
    friction=g.SAMPLE_FRICTION,
)

# 4. Status lamp lit markers (teleported via write_pose)
STATUS_LAMP_AMBER = part(
    "status_lamp_amber",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.STATUS_AMBER, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
)

STATUS_LAMP_GREEN = part(
    "status_lamp_green",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.STATUS_GREEN, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
)

STATUS_LAMP_RED = part(
    "status_lamp_red",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.STATUS_RED, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
)

# 5. Temperature bar lit lamp markers (9 stacked discs, k=1..9)
BAR_LAMPS = []
for k in range(1, 10):
    col = g.LAMP_CYAN if k <= 5 else g.LAMP_RED
    lamp = part(
        f"bar_lamp_{k}",
        pos=(0.0, 0.0, g.STOWED_Z),
        mass=0.005,
        cylinders=(((0.0, 0.0, 0.0), g.BAR_LAMP_LIT_R, g.BAR_LAMP_LIT_H, col, "Z"),),
        boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    )
    BAR_LAMPS.append(lamp)

OBJECTS = [
    FIXTURE,
    SAMPLE_1,
    SAMPLE_2,
    STATUS_LAMP_AMBER,
    STATUS_LAMP_GREEN,
    STATUS_LAMP_RED,
    *BAR_LAMPS,
]

_SCORER = Scorer()
_LAST_OBJECTS = None
_LAST_RESULT = None


@on_reset
def _reset_scorer(environment=None) -> None:
    global _LAST_OBJECTS, _LAST_RESULT
    _SCORER.reset()
    _LAST_OBJECTS = None
    _LAST_RESULT = None


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
    global _LAST_OBJECTS, _LAST_RESULT
    if objects is _LAST_OBJECTS and _LAST_RESULT is not None:
        return _LAST_RESULT

    e = env()
    fixture = objects["fixture"]
    s1 = objects["sample_1"]
    s2 = objects["sample_2"]

    s1_data = (s1.pos, s1.quat_xyzw, s1.lin_vel, s1.ang_vel)
    s2_data = (s2.pos, s2.quat_xyzw, s2.lin_vel, s2.ang_vel)

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    result = _SCORER.update(
        fixture.pos,
        fixture.sliders,
        s1_data,
        s2_data,
        left_open and right_open,
    )

    # Teleport visual indicators based on hidden logic state
    if e is not None:
        # Status lamp (amber, green, red)
        status = result.details.get("status_lamp", "off")
        if status == "amber":
            write_pose(e, "status_lamp_amber", g.STATUS_LAMP_LIT_POS)
            write_pose(e, "status_lamp_green", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "status_lamp_red", (0.0, 0.0, g.STOWED_Z))
        elif status == "green":
            write_pose(e, "status_lamp_amber", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "status_lamp_green", g.STATUS_LAMP_LIT_POS)
            write_pose(e, "status_lamp_red", (0.0, 0.0, g.STOWED_Z))
        elif status == "red":
            write_pose(e, "status_lamp_amber", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "status_lamp_green", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "status_lamp_red", g.STATUS_LAMP_LIT_POS)
        else:
            write_pose(e, "status_lamp_amber", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "status_lamp_green", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "status_lamp_red", (0.0, 0.0, g.STOWED_Z))

        # Temperature bar (9 discs)
        bar_count = result.details.get("bar_count", 0)
        for k in range(1, 10):
            name = f"bar_lamp_{k}"
            if k <= bar_count:
                pos = (g.BAR_DISC_X[k - 1], g.BAR_LAMP_Y, g.BAR_LAMP_LIT_Z)
                write_pose(e, name, pos)
            else:
                write_pose(e, name, (0.0, 0.0, g.STOWED_Z))

    _trace({
        **_arm_trace(e),
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "fixture_sliders": fixture.sliders,
        "write_pose_status": dict(parts.WRITE_POSE_STATUS),
        **result.details,
    })

    _LAST_OBJECTS = objects
    _LAST_RESULT = result
    return result


def success(objects) -> bool:
    return _update(objects).success


def score(objects) -> float:
    return _update(objects).score


def demo():
    return demo_plan.plans()
