"""Yam-arena task file: Compress and lock a spring-loaded cartridge.

Instruction:
Compress the spring plunger all the way in with one hand and slide the lock bar across with the other, then let go so the cartridge stays compressed.

Concept:
A spring-loaded cartridge (battery tray, magazine) has to be compressed and locked with a bar;
one hand cannot do both. If you let go before the bar is across, the spring throws it back.
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
from parts import Slider, env, gripper_open, on_reset, on_step, part, write_pose
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Compress the spring plunger all the way in with one hand and slide the lock bar across with the other, then let go so the cartridge stays compressed."


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


# 1. Sliders on the Cartridge Fixture
PLUNGER_SLIDER = Slider(
    "plunger_slider",
    rest=g.PLUNGER_LOCAL_REST,
    boxes=tuple(
        ((c[0] - g.BASE_POS[0], c[1] - g.BASE_POS[1], c[2] - g.BASE_POS[2]), size, colour)
        for c, size, colour in g.plunger_slider_world_boxes()
    ),
    travel=g.PLUNGER_TRAVEL,
    rate=g.PLUNGER_SPRING_K,
    damping=g.PLUNGER_DAMPING,
    mass=g.PLUNGER_MASS,
    axis=g.PLUNGER_AXIS,
)

LOCK_BAR_SLIDER = Slider(
    "lock_bar_slider",
    rest=g.LOCK_BAR_LOCAL_REST,
    boxes=tuple(
        ((c[0] - g.BASE_POS[0], c[1] - g.BASE_POS[1], c[2] - g.BASE_POS[2]), size, colour)
        for c, size, colour in g.lock_bar_slider_world_boxes()
    ),
    travel=g.LOCK_BAR_TRAVEL,
    rate=g.LOCK_BAR_RATE,
    damping=g.LOCK_BAR_DAMPING,
    mass=g.LOCK_BAR_MASS,
    axis=g.LOCK_BAR_AXIS,
)

# 2. Heavy Cartridge Base Fixture
CARTRIDGE_FIXTURE = world_part(
    "cartridge_fixture",
    pos=g.BASE_POS,
    yaw=0.0,
    boxes=tuple(g.fixture_world_boxes()),
    mass=g.FIXTURE_MASS,
    sliders=(PLUNGER_SLIDER, LOCK_BAR_SLIDER),
    color=g.BASE_GREY,
    friction=g.FIXTURE_FRICTION,
)

# 3. Status Lamp Marker (25 mm green disc)
STATUS_LAMP = part(
    "status_lamp",
    pos=g.LAMP_POS,
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), g.LAMP_SIZE, g.LAMP_GREEN),),
    collision=False,
)

OBJECTS = [
    CARTRIDGE_FIXTURE,
    STATUS_LAMP,
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
    fixture = objects["cartridge_fixture"]
    plunger_disp = float(fixture.sliders.get("plunger_slider", 0.0))
    bar_disp = float(fixture.sliders.get("lock_bar_slider", 0.0))

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    arm_tcps, arm_vels = _arm_info(e)

    result = _SCORER.update(
        plunger_disp=plunger_disp,
        bar_disp=bar_disp,
        grippers_open=left_open and right_open,
        arm_tcps=arm_tcps,
        arm_velocities=arm_vels,
    )

    if e is not None:
        lamp_z = g.LAMP_ON_Z if result.details.get("lamp_green") else g.LAMP_OFF_Z
        write_pose(e, "status_lamp", (g.LAMP_POS[0], g.LAMP_POS[1], lamp_z))

    _trace({
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        "plunger_disp": plunger_disp,
        "bar_disp": bar_disp,
        **result.details,
        "dbg_tcps": {a: [round(float(v), 3) for v in arm_tcps[a]] for a in (arm_tcps or {})},
        "dbg_left_tcp": [round(float(v), 4) for v in (arm_tcps.get("left") if arm_tcps else [0, 0, 0])],
        "dbg_right_tcp": [round(float(v), 4) for v in (arm_tcps.get("right") if arm_tcps else [0, 0, 0])],
        "dbg_plunger_knob_pos": [round(float(v), 4) for v in (g.PLUNGER_REST_X + plunger_disp, g.PLUNGER_REST_Y, g.PLUNGER_REST_Z)],
        "dbg_lock_bar_pos": [round(float(v), 4) for v in (g.LOCK_BAR_REST_X, g.LOCK_BAR_REST_Y + bar_disp, g.LOCK_BAR_REST_Z)],
        "dbg_plunger_disp_mm": round(plunger_disp * 1000.0, 2),
        "dbg_bar_disp_mm": round(bar_disp * 1000.0, 2),
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
