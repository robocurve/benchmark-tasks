"""Yam-arena task file: Tighten a cold plate's four bolts in a star pattern through three torque passes.

On the station table:
- A heavy base fixture (200 x 260 x 15 mm, 6.0 kg) with an aluminium cold plate top (140 x 200 x 10 mm).
- Four bolt heads (spring buttons with 36 mm white caps, 8 mm travel to hard stop, 100 N/m spring):
  L_near, L_far, R_near, R_far.
- Beside each bolt, a column of three cyan lamp discs indicates stage 0-3.
- A large status lamp (50 mm) above the plate: off / red (WARP) / green (TORQUED).
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

INSTRUCTION = "Tighten the cold plate's four white bolt buttons in three passes, pressing each bolt once per pass with diagonal corners pressed consecutively and no bolt more than one stage ahead. The large status lamp turns green when all four bolts reach stage 3 without plate warp."

# 1. Four bolt button sliders on the fixture
BOLT_SLIDERS = []
for c in g.CORNERS:
    rest_pt = g.BOLT_REST_POS[c]
    s = Slider(
        f"btn_{c}",
        rest=rest_pt,
        boxes=((rest_pt, g.BUTTON_CAP_SIZE, g.WHITE),),
        travel=g.BUTTON_TRAVEL,
        rate=g.BUTTON_RATE,
        damping=g.BUTTON_DAMPING,
        mass=g.BUTTON_MASS,
        axis="Z",
    )
    BOLT_SLIDERS.append(s)

# Fixed plate fixture (base + cold plate + status lamp base + 12 lamp bases + 4 bolt sliders)
FIXTURE = part(
    "fixture",
    pos=(0.0, 0.0, 0.0),
    yaw=0.0,
    boxes=g.fixture_boxes(),
    cylinders=g.fixture_cylinders(),
    mass=g.FIXTURE_MASS,
    sliders=tuple(BOLT_SLIDERS),
    color=g.DARK_GREY,
    friction=g.FIXTURE_FRICTION,
)

# Visual indicator markers (teleported between visible position and under-table storage; collision=False)
STATUS_LAMP_RED = part(
    "status_lamp_red",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.LAMP_BRIGHT_RED, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

STATUS_LAMP_GREEN = part(
    "status_lamp_green",
    pos=(0.0, 0.0, g.STOWED_Z),
    mass=0.010,
    cylinders=(((0.0, 0.0, 0.0), g.STATUS_LAMP_LIT_R, g.STATUS_LAMP_LIT_H, g.LAMP_BRIGHT_GREEN, "Z"),),
    boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
    collision=False,
)

# 12 cyan stage lamp markers (4 corners x 3 stages)
STAGE_LAMP_MARKERS = {}
for c in g.CORNERS:
    for s in (1, 2, 3):
        m_name = f"lamp_{c}_{s}"
        STAGE_LAMP_MARKERS[m_name] = part(
            m_name,
            pos=(0.0, 0.0, g.STOWED_Z),
            mass=0.005,
            cylinders=(((0.0, 0.0, 0.0), g.STAGE_LAMP_LIT_R, g.STAGE_LAMP_LIT_H, g.LAMP_BRIGHT_CYAN, "Z"),),
            boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
            collision=False,
        )

OBJECTS = [
    FIXTURE,
    STATUS_LAMP_RED,
    STATUS_LAMP_GREEN,
    *STAGE_LAMP_MARKERS.values(),
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
    """Where each gripper's TCP and finger tips are (world frame, relative to the env origin), for trace only."""
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

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    result = _SCORER.update(
        fixture.pos,
        fixture.quat_xyzw,
        fixture.sliders,
        fixture.lin_vel,
        fixture.ang_vel,
        left_open and right_open,
    )

    # Teleport visual indicators based on hidden logic state
    status_lamp = result.details.get("status_lamp", "off")
    stages = result.details.get("stages", {})

    if e is not None:
        # Large status lamp
        if status_lamp == "red":
            write_pose(e, "status_lamp_red", g.STATUS_LAMP_LIT_POS)
            write_pose(e, "status_lamp_green", (0.0, 0.0, g.STOWED_Z))
        elif status_lamp == "green":
            write_pose(e, "status_lamp_green", g.STATUS_LAMP_LIT_POS)
            write_pose(e, "status_lamp_red", (0.0, 0.0, g.STOWED_Z))
        else:
            write_pose(e, "status_lamp_red", (0.0, 0.0, g.STOWED_Z))
            write_pose(e, "status_lamp_green", (0.0, 0.0, g.STOWED_Z))

        # 12 Cyan stage lamps
        for c in g.CORNERS:
            s_val = stages.get(c, 0)
            for s in (1, 2, 3):
                m_name = f"lamp_{c}_{s}"
                if s_val >= s:
                    pos = g.STAGE_LAMP_POS[(c, s)]
                    lit_pos = (pos[0], pos[1], g.STAGE_LAMP_LIT_Z)
                    write_pose(e, m_name, lit_pos)
                else:
                    write_pose(e, m_name, (0.0, 0.0, g.STOWED_Z))

    _trace({
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "fixture_sliders": fixture.sliders,
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
