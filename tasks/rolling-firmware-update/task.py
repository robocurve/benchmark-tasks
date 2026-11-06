"""Yam-arena task file: Roll a firmware update across two redundant units without dropping the load.

On the station table:
- A rigid fixture with redundant Unit A and Unit B panels and a shared central panel.
- On each unit: a spring slider service door (orange handle), a recessed UPDATE button (yellow cap),
  an ACTIVE lamp (blue), a VERSION lamp (grey/amber/white), and a 4-lamp progress bar (cyan).
- On the shared panel: a SWITCH button (white cap) and a LOAD lamp (green/red).
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import demo_plan
import geometry as g
from parts import Slider, env, gripper_open, on_reset, part, write_pose
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Update both units to the new firmware without ever turning the load lamp red: a unit updates only while its orange service door is held open and its yellow UPDATE button has been pressed, and the white SWITCH button moves the load between units."

# 1. Sliders on the fixture
DOOR_A = Slider(
    "door_a",
    rest=g.DOOR_LOCAL_REST["A"],
    boxes=g.door_slider_boxes("A"),
    travel=g.DOOR_TRAVEL,
    rate=g.DOOR_SPRING_RATE,
    damping=g.DOOR_DAMPING,
    mass=g.DOOR_MASS,
    axis="X",
)

DOOR_B = Slider(
    "door_b",
    rest=g.DOOR_LOCAL_REST["B"],
    boxes=g.door_slider_boxes("B"),
    travel=g.DOOR_TRAVEL,
    rate=g.DOOR_SPRING_RATE,
    damping=g.DOOR_DAMPING,
    mass=g.DOOR_MASS,
    axis="X",
)

BTN_UPDATE_A = Slider(
    "btn_update_a",
    rest=g.UPDATE_BTN_LOCAL_REST["A"],
    boxes=((g.UPDATE_BTN_LOCAL_REST["A"], g.BUTTON_CAP_SIZE, g.YELLOW),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_SPRING_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_CAP_MASS,
    axis="Z",
)

BTN_UPDATE_B = Slider(
    "btn_update_b",
    rest=g.UPDATE_BTN_LOCAL_REST["B"],
    boxes=((g.UPDATE_BTN_LOCAL_REST["B"], g.BUTTON_CAP_SIZE, g.YELLOW),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_SPRING_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_CAP_MASS,
    axis="Z",
)

BTN_SWITCH = Slider(
    "btn_switch",
    rest=g.SWITCH_BTN_LOCAL_REST,
    boxes=((g.SWITCH_BTN_LOCAL_REST, g.BUTTON_CAP_SIZE, g.WHITE),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_SPRING_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_CAP_MASS,
    axis="Z",
)

# 2. Rigid Base Fixture with all panels, pockets, sleeves and sliders
FIXTURE = part(
    "fixture",
    pos=g.BASE_POS,
    yaw=0.0,
    boxes=g.fixture_boxes(),
    cylinders=g.fixture_cylinders(),
    mass=g.BASE_MASS,
    sliders=(DOOR_A, DOOR_B, BTN_UPDATE_A, BTN_UPDATE_B, BTN_SWITCH),
    color=g.DARK_GREY,
    friction=g.BASE_FRICTION,
)

# 3. Kinematic Display Markers (collision=False, teleported with write_pose)
# ACTIVE lamps (blue disc)
ACTIVE_LAMP_A = part(
    "active_lamp_a",
    pos=g.ACTIVE_LAMP_POS["A"],
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.ACTIVE_LAMP_R, g.ACTIVE_LAMP_H, g.BLUE, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

ACTIVE_LAMP_B = part(
    "active_lamp_b",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.ACTIVE_LAMP_R, g.ACTIVE_LAMP_H, g.BLUE, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

# VERSION lamps (amber when updating, white when v1)
VERSION_LAMP_AMBER_A = part(
    "version_lamp_amber_a",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.VERSION_LAMP_R, g.VERSION_LAMP_H, g.AMBER, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

VERSION_LAMP_WHITE_A = part(
    "version_lamp_white_a",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.VERSION_LAMP_R, g.VERSION_LAMP_H, g.WHITE, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

VERSION_LAMP_AMBER_B = part(
    "version_lamp_amber_b",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.VERSION_LAMP_R, g.VERSION_LAMP_H, g.AMBER, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

VERSION_LAMP_WHITE_B = part(
    "version_lamp_white_b",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.VERSION_LAMP_R, g.VERSION_LAMP_H, g.WHITE, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

# 4 Progress Bar lamps for Unit A (cyan discs)
PROGRESS_LAMPS_A = [
    part(
        f"progress_a_{i}",
        pos=(0.0, 0.0, g.STOWED_Z),
        mass=0.005,
        cylinders=(((0.0, 0.0, 0.0), g.PROGRESS_LAMP_R, g.PROGRESS_LAMP_H, g.CYAN, "Z"),),
        boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
        collision=False,
    )
    for i in range(1, 5)
]

# 4 Progress Bar lamps for Unit B (cyan discs)
PROGRESS_LAMPS_B = [
    part(
        f"progress_b_{i}",
        pos=(0.0, 0.0, g.STOWED_Z),
        mass=0.005,
        cylinders=(((0.0, 0.0, 0.0), g.PROGRESS_LAMP_R, g.PROGRESS_LAMP_H, g.CYAN, "Z"),),
        boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
        collision=False,
    )
    for i in range(1, 5)
]

# LOAD lamps (green = served, red = outage)
LOAD_LAMP_GREEN = part(
    "load_lamp_green",
    pos=g.LOAD_LAMP_POS,
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.LOAD_LAMP_R, g.LOAD_LAMP_H, g.GREEN, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

LOAD_LAMP_RED = part(
    "load_lamp_red",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.LOAD_LAMP_R, g.LOAD_LAMP_H, g.BRIGHT_RED, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

OBJECTS = (
    FIXTURE,
    ACTIVE_LAMP_A,
    ACTIVE_LAMP_B,
    VERSION_LAMP_AMBER_A,
    VERSION_LAMP_WHITE_A,
    VERSION_LAMP_AMBER_B,
    VERSION_LAMP_WHITE_B,
    *PROGRESS_LAMPS_A,
    *PROGRESS_LAMPS_B,
    LOAD_LAMP_GREEN,
    LOAD_LAMP_RED,
)

_SCORER = Scorer()
_LAST_OBJECTS = None
_LAST_RESULT = None


@on_reset
def _reset(_environment) -> None:
    global _LAST_OBJECTS, _LAST_RESULT
    _SCORER.reset()
    _LAST_OBJECTS = None
    _LAST_RESULT = None


def _trace(row: dict) -> None:
    path = os.environ.get("TASK_TRACE")
    if not path:
        return
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(row) + "\n")


def _to_numpy(val):
    val = getattr(val, "torch", val)
    for m in ("detach", "cpu", "numpy"):
        if hasattr(val, m):
            val = getattr(val, m)()
    import numpy as np
    return np.asarray(val, dtype=np.float64)


def _arm_trace(e):
    if e is None:
        return {}
    try:
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

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    result = _SCORER.update(
        fixture_pos=fixture.pos,
        fixture_quat=fixture.quat_xyzw,
        fixture_lin_vel=fixture.lin_vel,
        fixture_ang_vel=fixture.ang_vel,
        fixture_sliders=fixture.sliders,
        grippers_open=(left_open and right_open),
    )

    # Teleport visual indicators based on hidden logic state
    logic_state = result.details.get("logic", {})
    if e is not None and logic_state:
        # ACTIVE lamps
        act = logic_state.get("active_unit", "A")
        write_pose(e, "active_lamp_a", g.ACTIVE_LAMP_POS["A"] if act == "A" else (0.0, 0.0, g.STOWED_Z))
        write_pose(e, "active_lamp_b", g.ACTIVE_LAMP_POS["B"] if act == "B" else (0.0, 0.0, g.STOWED_Z))

        # VERSION lamps A
        vl_a = logic_state.get("version_lamp_a", "grey")
        write_pose(e, "version_lamp_amber_a", g.VERSION_LAMP_POS["A"] if vl_a == "amber" else (0.0, 0.0, g.STOWED_Z))
        write_pose(e, "version_lamp_white_a", g.VERSION_LAMP_POS["A"] if vl_a == "white" else (0.0, 0.0, g.STOWED_Z))

        # VERSION lamps B
        vl_b = logic_state.get("version_lamp_b", "grey")
        write_pose(e, "version_lamp_amber_b", g.VERSION_LAMP_POS["B"] if vl_b == "amber" else (0.0, 0.0, g.STOWED_Z))
        write_pose(e, "version_lamp_white_b", g.VERSION_LAMP_POS["B"] if vl_b == "white" else (0.0, 0.0, g.STOWED_Z))

        # Progress lamps A
        p_a = logic_state.get("progress_lamps_a", 0)
        for i in range(1, 5):
            pos_i = g.progress_lamp_pos("A", i) if p_a >= i else (0.0, 0.0, g.STOWED_Z)
            write_pose(e, f"progress_a_{i}", pos_i)

        # Progress lamps B
        p_b = logic_state.get("progress_lamps_b", 0)
        for i in range(1, 5):
            pos_i = g.progress_lamp_pos("B", i) if p_b >= i else (0.0, 0.0, g.STOWED_Z)
            write_pose(e, f"progress_b_{i}", pos_i)

        # LOAD lamp
        load_state = logic_state.get("load_lamp", "green")
        write_pose(e, "load_lamp_green", g.LOAD_LAMP_POS if load_state == "green" else (0.0, 0.0, g.STOWED_Z))
        write_pose(e, "load_lamp_red", g.LOAD_LAMP_POS if load_state == "red" else (0.0, 0.0, g.STOWED_Z))

    _trace({
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "fixture_sliders": fixture.sliders,
        "fixture_pos": list(fixture.pos),
        "fixture_quat": list(fixture.quat_xyzw),
        **_arm_trace(e),
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
