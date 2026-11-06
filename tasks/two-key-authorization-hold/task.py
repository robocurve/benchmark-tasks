"""Yam-arena task file: Two-key authorization hold with sliding guards.

Story:
A dual-control / two-person rule authorization station before a critical operation:
Two operators (or both arms) must slide open protective red lids over two key buttons
spaced 0.56 m apart, then press and hold both keys simultaneously for 3.0 s until the
ARMED lamp illuminates green. Finally, press the CONFIRM button within 10.0 s while
the station remains armed.
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

INSTRUCTION = "Slide both red guards aside, then press and hold both key buttons together until the ARMED lamp turns green, and press the white CONFIRM button before the lamp turns off."

# 1. Sliders on controller fixture
# Button sliders (axis -Z into table)
SLIDERS = []
for b in g.BUTTONS:
    rest_pt = g.BUTTON_LOCAL_POS[b]
    s = Slider(
        f"btn_{b}",
        rest=rest_pt,
        boxes=((rest_pt, g.BUTTON_CAP_SIZE, g.BUTTON_CAP_COLORS[b]),),
        travel=g.BUTTON_TRAVEL,
        rate=g.BUTTON_SPRING_K,
        damping=g.BUTTON_DAMPING,
        mass=g.BUTTON_MASS,
        axis="Z",
    )
    SLIDERS.append(s)

# Guard sliders (axis +Y outward for left, -Y outward for right)
for g_name in g.GUARDS:
    rest_pt = g.GUARD_LOCAL_POS[g_name]
    s = Slider(
        f"slider_{g_name}",
        rest=rest_pt,
        boxes=((rest_pt, g.GUARD_SIZE, g.RED_LID),),
        travel=g.GUARD_TRAVEL,
        rate=1.0,         # free sliding with friction / light damping
        damping=15.0,
        mass=g.GUARD_MASS,
        axis=g.GUARD_AXIS[g_name],
    )
    SLIDERS.append(s)

def world_part(name, pos, boxes=(), cylinders=(), **kw):
    """Like part(), but the shapes are given at their WORLD positions: shift them into the body frame here."""
    px, py, pz = pos

    def shift(c):
        return (c[0] - px, c[1] - py, c[2] - pz)

    return part(name, pos, boxes=tuple((shift(c), s, col) for c, s, col in boxes),
                cylinders=tuple((shift(c), *rest) for c, *rest in cylinders), **kw)


# 2. Main controller fixture
FIXTURE = world_part(
    "controller_fixture",
    pos=g.BASE_POS,
    yaw=0.0,
    boxes=g.fixture_boxes(),
    mass=g.FIXTURE_MASS,
    sliders=tuple(SLIDERS),
    color=g.DARK_GREY,
    friction=g.FIXTURE_FRICTION,
)

# 3. Status marker lamps (teleported via write_pose, collision=False)
DUMMY_BOX = (((0.0, 0.0, 0.0), (0.002, 0.002, 0.002), (0.0, 0.0, 0.0)),)

ARMED_LAMP = part(
    "lamp_armed",
    pos=(g.ARMED_LAMP_POS[0], g.ARMED_LAMP_POS[1], g.LAMP_OFF_Z),
    mass=0.005,
    cylinders=(((0.0, 0.0, 0.0), g.LAMP_DIAMETER_STATUS / 2, 0.006, g.LAMP_GREEN, "Z"),),
    boxes=DUMMY_BOX,
    collision=False,
)

FAULT_LAMP = part(
    "lamp_fault",
    pos=(g.FAULT_LAMP_POS[0], g.FAULT_LAMP_POS[1], g.LAMP_OFF_Z),
    mass=0.005,
    cylinders=(((0.0, 0.0, 0.0), g.LAMP_DIAMETER_STATUS / 2, 0.006, g.LAMP_RED, "Z"),),
    boxes=DUMMY_BOX,
    collision=False,
)

PROGRESS_LAMPS = []
for i, y_pos in enumerate(g.PROGRESS_LAMP_Y):
    PROGRESS_LAMPS.append(
        part(
            f"lamp_progress_{i}",
            pos=(g.PROGRESS_LAMP_X, y_pos, g.LAMP_OFF_Z),
            mass=0.005,
            cylinders=(((0.0, 0.0, 0.0), g.LAMP_DIAMETER_PROGRESS / 2, 0.006, g.LAMP_CYAN, "Z"),),
            boxes=DUMMY_BOX,
            collision=False,
        )
    )

OBJECTS = [
    FIXTURE,
    ARMED_LAMP,
    FAULT_LAMP,
    *PROGRESS_LAMPS,
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


def _arm_trace(e) -> dict:
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
            names = list(robot.joint_names)
            q = _to_numpy(robot.data.joint_pos)[0]
            out[f"{arm}_finger_q"] = [float(q[names.index(j)]) for j in ("joint7", "joint8")]
        return out
    except Exception as exc:  # noqa: BLE001
        return {"arm_trace_error": repr(exc)}


def _update(objects):
    global _last_states_ref, _last_result
    if objects is _last_states_ref and _last_result is not None:
        return _last_result

    e = env()
    fixture = objects["controller_fixture"]

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    arm_vels = None
    if e is not None:
        try:
            from parts import _to_numpy
            base = e.unwrapped
            arm_vels = {}
            for arm in ("left", "right"):
                robot = base.scene[f"{arm}_robot"]
                lv = _to_numpy(robot.data.root_link_lin_vel_w)[0]
                av = _to_numpy(robot.data.root_link_ang_vel_w)[0]
                arm_vels[arm] = (lv, av)
        except Exception:
            pass

    result = _SCORER.update(
        fixture.pos,
        fixture.quat_xyzw,
        fixture.sliders,
        left_open and right_open,
        arm_vels,
    )

    st = _SCORER.logic.state
    stowed = (0.0, 0.0, g.LAMP_OFF_Z)

    if e is not None:
        # ARMED lamp
        write_pose(
            e,
            "lamp_armed",
            (g.ARMED_LAMP_POS[0], g.ARMED_LAMP_POS[1], g.LAMP_ON_Z) if st.armed else stowed,
        )
        # FAULT lamp
        write_pose(
            e,
            "lamp_fault",
            (g.FAULT_LAMP_POS[0], g.FAULT_LAMP_POS[1], g.LAMP_ON_Z) if st.fault_lamp else stowed,
        )
        # PROGRESS lamps
        for i, y_pos in enumerate(g.PROGRESS_LAMP_Y):
            is_lit = (i < st.progress_count)
            write_pose(
                e,
                f"lamp_progress_{i}",
                (g.PROGRESS_LAMP_X, y_pos, g.LAMP_ON_Z) if is_lit else stowed,
            )

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


def plans():
    return demo_plan.plans()
