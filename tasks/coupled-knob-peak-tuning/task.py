"""Yam-arena task file: Tune two coupled adjusters to the peak of a meter in a few readings.

On the station table:
- A fixed fixture comprising a base plate, two track channels (120 mm apart), a meter housing,
  a scale bar (160 mm long with a green band over the last 8 mm), and a spring-loaded READ button.
- Two loose knobs (orange on the left track, teal on the right track) held by Coulomb friction.
- A bright red meter needle whose position along the scale bar is updated via write_pose when READ is pressed.
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

INSTRUCTION = "Press the white READ button between the two tracks to show the red needle for the current positions of the orange (left) and teal (right) sliders; use at most eight READ presses, including at least three at slider positions 10 mm or more apart. Then set both sliders so the needle stands on the green band at the end of the scale, and let go."

# 1. READ button slider on the fixture (in local fixture frame)
READ_BTN = Slider(
    "read_btn",
    rest=g.READ_BTN_LOCAL_REST,
    boxes=((g.READ_BTN_LOCAL_REST, g.BUTTON_CAP_SIZE, g.WHITE),),
    travel=g.BUTTON_TRAVEL,
    rate=g.BUTTON_RATE,
    damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS,
    axis=g.BUTTON_AXIS,
)

# 2. Fixed fixture: base plate, track channels, button sleeve, meter housing, scale bar
FIXTURE = part(
    "fixture",
    pos=g.BASE_POS,
    yaw=0.0,
    boxes=g.fixture_boxes(),
    mass=g.BASE_MASS,
    sliders=(READ_BTN,),
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

# 4. Needle: visual indicator part moved along scale bar via write_pose
NEEDLE = part(
    "needle",
    pos=g.needle_pos_from_r(0.0),
    yaw=0.0,
    boxes=g.needle_boxes(),
    mass=g.NEEDLE_MASS,
    color=g.BRIGHT_RED,
)

OBJECTS = [
    FIXTURE,
    KNOB_LEFT,
    KNOB_RIGHT,
    NEEDLE,
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

    # Teleport needle visual based on current displayed reading
    needle_r = result.details.get("needle_r", 0.0)
    needle_ok = None
    needle_seen = None
    if e is not None:
        target_pos = g.needle_pos_from_r(needle_r)
        needle_ok = write_pose(e, "needle", target_pos)
        needle_seen = read_world_pos(e, "needle")  # read back: the GPU trace proves the needle really moved

    _trace({
        **_arm_trace(e),
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "fixture_sliders": fixture.sliders,
        "needle_write_ok": needle_ok,
        "needle_pos_readback": needle_seen,
        "needle_pos_target": list(g.needle_pos_from_r(needle_r)),
        "write_pose_status": dict(parts.WRITE_POSE_STATUS),
        "peak_variant": __import__("logic").VARIANT,
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
