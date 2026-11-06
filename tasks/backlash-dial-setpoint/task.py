"""Yam-arena task file: Calibrate two backlash dials to their target setpoints and lock them.

Story:
A real calibration dial has backlash: after you reverse direction it takes a few mm of travel
before the indicator moves.

On the station table:
- A fixed fixture comprising a base plate, two track channels (300 mm apart), two yellow LOCK buttons,
  and two 10-lamp indicator bars with target band markers.
- Two loose knobs (orange on left track, teal on right track) held by Coulomb friction.
- Code-driven indicator lamps teleported via write_pose to display positions and target band state.
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
from parts import Slider, env, gripper_open, on_reset, part, write_pose
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Adjust both sliding knobs until the white marker lamps on their indicator bars light up, then press the yellow LOCK button beside each rail."

# 1. LOCK button sliders on the fixture (in local fixture frame)
LOCK_BTN_LEFT = Slider(
    "lock_btn_left",
    rest=g.LOCK_BTN_LOCAL_REST["left"],
    boxes=((g.LOCK_BTN_LOCAL_REST["left"], g.BUTTON_CAP_SIZE, g.YELLOW),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis=g.BUTTON_AXIS,
)

LOCK_BTN_RIGHT = Slider(
    "lock_btn_right",
    rest=g.LOCK_BTN_LOCAL_REST["right"],
    boxes=((g.LOCK_BTN_LOCAL_REST["right"], g.BUTTON_CAP_SIZE, g.YELLOW),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis=g.BUTTON_AXIS,
)

# 2. Fixed fixture: base plate, track channels, button pocket rings, meter housings
FIXTURE = part(
    "fixture",
    pos=g.BASE_POS,
    yaw=0.0,
    boxes=g.fixture_boxes(),
    mass=g.BASE_MASS,
    sliders=(LOCK_BTN_LEFT, LOCK_BTN_RIGHT),
    color=g.DARK_GREY,
    friction=g.BASE_FRICTION,
)

# 3. Two loose knobs: sliding in channels held by Coulomb friction
KNOB_LEFT = part(
    "knob_left",
    pos=g.START_POS["left"],
    yaw=0.0,
    boxes=g.knob_boxes(g.ORANGE),
    mass=g.KNOB_MASS,
    color=g.ORANGE,
    friction=g.KNOB_FRICTION,
)

KNOB_RIGHT = part(
    "knob_right",
    pos=g.START_POS["right"],
    yaw=0.0,
    boxes=g.knob_boxes(g.TEAL),
    mass=g.KNOB_MASS,
    color=g.TEAL,
    friction=g.KNOB_FRICTION,
)

# 4. Indicator lamps (display markers teleported via write_pose, collision=False)
BAR_LAMPS = {}
for side in ("left", "right"):
    for k in range(1, 11):
        name = f"lamp_{side}_{k}"
        BAR_LAMPS[name] = part(
            name,
            pos=(0.0, 0.0, g.STOWED_Z),
            yaw=0.0,
            boxes=g.lamp_boxes(g.BRIGHT_CYAN),
            mass=0.005,
            color=g.BRIGHT_CYAN,
            collision=False,
        )

TARGET_LAMPS = {
    "target_lamp_left": part(
        "target_lamp_left",
        pos=(0.0, 0.0, g.STOWED_Z),
        yaw=0.0,
        boxes=g.target_marker_boxes(),
        mass=0.005,
        color=g.BRIGHT_WHITE,
        collision=False,
    ),
    "target_lamp_right": part(
        "target_lamp_right",
        pos=(0.0, 0.0, g.STOWED_Z),
        yaw=0.0,
        boxes=g.target_marker_boxes(),
        mass=0.005,
        color=g.BRIGHT_WHITE,
        collision=False,
    ),
}

LOCKED_LAMPS = {
    "locked_lamp_left": part(
        "locked_lamp_left",
        pos=(0.0, 0.0, g.STOWED_Z),
        yaw=0.0,
        boxes=(((0.0, 0.0, 0.0), (2 * g.LOCKED_LAMP_R, 2 * g.LOCKED_LAMP_R, g.LOCKED_LAMP_H), g.BRIGHT_GREEN),),
        mass=0.005,
        color=g.BRIGHT_GREEN,
        collision=False,
    ),
    "locked_lamp_right": part(
        "locked_lamp_right",
        pos=(0.0, 0.0, g.STOWED_Z),
        yaw=0.0,
        boxes=(((0.0, 0.0, 0.0), (2 * g.LOCKED_LAMP_R, 2 * g.LOCKED_LAMP_R, g.LOCKED_LAMP_H), g.BRIGHT_GREEN),),
        mass=0.005,
        color=g.BRIGHT_GREEN,
        collision=False,
    ),
}

OBJECTS = [
    FIXTURE,
    KNOB_LEFT,
    KNOB_RIGHT,
    *BAR_LAMPS.values(),
    *TARGET_LAMPS.values(),
    *LOCKED_LAMPS.values(),
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
    global _LAST_OBJECTS, _LAST_RESULT
    if objects is _LAST_OBJECTS and _LAST_RESULT is not None:
        return _LAST_RESULT

    e = env()
    fixture = objects["fixture"]
    knob_l = objects["knob_left"]
    knob_r = objects["knob_right"]

    knob_l_data = (knob_l.pos, knob_l.quat_xyzw, knob_l.lin_vel, knob_l.ang_vel)
    knob_r_data = (knob_r.pos, knob_r.quat_xyzw, knob_r.lin_vel, knob_r.ang_vel)

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    result = _SCORER.update(
        fixture.pos,
        fixture.sliders,
        knob_l_data,
        knob_r_data,
        left_open and right_open,
    )

    # Teleport visual indicator lamps via write_pose
    if e is not None:
        disp_l = result.details.get("displayed_left", 0.0)
        disp_r = result.details.get("displayed_right", 0.0)

        # 1. Bar lamps: lamp k lit if displayed >= 10*k - 5 mm (rounding threshold)
        for k in range(1, 11):
            thresh = 10.0 * k - 5.0
            pos_l = g.lamp_pos_world("left", k) if disp_l >= thresh else (0.0, 0.0, g.STOWED_Z)
            pos_r = g.lamp_pos_world("right", k) if disp_r >= thresh else (0.0, 0.0, g.STOWED_Z)
            write_pose(e, f"lamp_left_{k}", pos_l)
            write_pose(e, f"lamp_right_{k}", pos_r)

        # 2. Target band lamps: lit when displayed is within target band
        target_in_l = logic.is_in_band("left", disp_l)
        target_in_r = logic.is_in_band("right", disp_r)
        pos_t_l = g.target_lamp_pos_world("left") if target_in_l else (0.0, 0.0, g.STOWED_Z)
        pos_t_r = g.target_lamp_pos_world("right") if target_in_r else (0.0, 0.0, g.STOWED_Z)
        write_pose(e, "target_lamp_left", pos_t_l)
        write_pose(e, "target_lamp_right", pos_t_r)

        # 3. Locked indicator lamps: lit when dial is locked in band
        locked_l = result.details.get("locked_in_band_left", False)
        locked_r = result.details.get("locked_in_band_right", False)
        pos_lk_l = g.LOCKED_LAMP_POS["left"] if locked_l else (0.0, 0.0, g.STOWED_Z)
        pos_lk_r = g.LOCKED_LAMP_POS["right"] if locked_r else (0.0, 0.0, g.STOWED_Z)
        write_pose(e, "locked_lamp_left", pos_lk_l)
        write_pose(e, "locked_lamp_right", pos_lk_r)

    _trace({
        **_arm_trace(e),
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "fixture_sliders": fixture.sliders,
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
