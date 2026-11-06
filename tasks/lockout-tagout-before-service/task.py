"""Yam-arena task file: Lockout-tagout procedure before servicing electrical enclosure.

Story:
Electrical maintenance safety standard:
Before servicing an enclosure, technicians must switch the main circuit breaker off,
insert a red lockout pin into the breaker lockout hole, press the TEST button to verify
power is dead (green lamp illuminates), and only then hold the spring-return service cover
open to access and press the blue service button.
Opening the cover while the breaker is live or without the safety pin seated triggers a FAULT.
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

INSTRUCTION = "Switch the breaker off, lock it with the red pin, press TEST, then hold the cover open and press the blue service button: never open the cover while the breaker is on."

# 1. Console fixture with sliders (breaker lever, test button, service cover, service button)
LEVER_LOCAL_REST = (g.LEVER_START_X - g.BASE_POS[0], g.LEVER_Y - g.BASE_POS[1], g.LEVER_Z - g.BASE_POS[2])
TEST_BTN_LOCAL_REST = (g.TEST_BTN_POS[0] - g.BASE_POS[0], g.TEST_BTN_POS[1] - g.BASE_POS[1], g.TEST_BTN_POS[2] - g.BASE_POS[2])
COVER_LOCAL_REST = (g.COVER_REST_X - g.BASE_POS[0], g.COVER_Y - g.BASE_POS[1], g.COVER_Z - g.BASE_POS[2])
COVER_HANDLE_LOCAL = (g.COVER_HANDLE_X - g.BASE_POS[0], g.COVER_Y - g.BASE_POS[1], g.COVER_HANDLE_Z - g.BASE_POS[2])
SERVICE_BTN_LOCAL_REST = (g.SERVICE_BTN_POS[0] - g.BASE_POS[0], g.SERVICE_BTN_POS[1] - g.BASE_POS[1], g.SERVICE_BTN_POS[2] - g.BASE_POS[2])

BREAKER_SLIDER = Slider(
    "breaker_slider",
    rest=LEVER_LOCAL_REST,
    boxes=((LEVER_LOCAL_REST, g.LEVER_HANDLE_SIZE, g.LEVER_ORANGE),),
    travel=g.LEVER_THROW,
    rate=0.02,         # almost no return spring: the breaker stays where it is pushed
    damping=10.0,
    mass=0.050,
    axis="+X",
)

TEST_BTN_SLIDER = Slider(
    "test_button_slider",
    rest=TEST_BTN_LOCAL_REST,
    boxes=((TEST_BTN_LOCAL_REST, g.TEST_BTN_SIZE, g.BUTTON_WHITE),),
    travel=g.TEST_BTN_TRAVEL,
    rate=100.0,
    damping=5.0,
    mass=0.010,
    axis="-Z",
)

COVER_SLIDER = Slider(
    "cover_slider",
    rest=COVER_LOCAL_REST,
    boxes=(
        (COVER_LOCAL_REST, g.COVER_SIZE, g.COVER_STEEL),
        (COVER_HANDLE_LOCAL, g.COVER_HANDLE_SIZE, g.PANEL_GREY),
    ),
    travel=g.COVER_TRAVEL,
    rate=g.COVER_SPRING_K,
    damping=5.0,
    mass=0.100,
    axis="-X",
)

SERVICE_BTN_SLIDER = Slider(
    "service_button_slider",
    rest=SERVICE_BTN_LOCAL_REST,
    boxes=((SERVICE_BTN_LOCAL_REST, g.SERVICE_BTN_SIZE, g.BUTTON_BLUE),),
    travel=g.SERVICE_BTN_TRAVEL,
    rate=100.0,
    damping=5.0,
    mass=0.010,
    axis="-Z",
)

def world_part(name, pos, boxes=(), cylinders=(), **kw):
    """Like part(), but the shapes are given at their WORLD positions: shift them into the body frame here."""
    px, py, pz = pos

    def shift(c):
        return (c[0] - px, c[1] - py, c[2] - pz)

    return part(name, pos, boxes=tuple((shift(c), s, col) for c, s, col in boxes),
                cylinders=tuple((shift(c), *rest) for c, *rest in cylinders), **kw)


CONSOLE_FIXTURE = world_part(
    "console_fixture",
    pos=g.BASE_POS,
    yaw=0.0,
    boxes=tuple(g.base_world_boxes()),
    mass=10.0,
    sliders=(BREAKER_SLIDER, TEST_BTN_SLIDER, COVER_SLIDER, SERVICE_BTN_SLIDER),
    color=g.DARK_GREY,
    friction=1.0,
)

# 2. Red Lock Pin
RED_LOCK_PIN = part(
    "red_lock_pin",
    pos=g.PIN_STAND_POS,
    yaw=0.0,
    boxes=(((0.0, 0.0, 0.0), (g.PIN_RADIUS * 2, g.PIN_RADIUS * 2, g.PIN_HEIGHT), g.PIN_RED),),
    cylinders=(((0.0, 0.0, 0.0), g.PIN_RADIUS, g.PIN_HEIGHT, g.PIN_RED, "Z"),),
    mass=0.035,
    friction=0.8,
)

# 3. Indicator Lamps (display markers)
LIVE_LAMP = part(
    "live_lamp",
    pos=g.LAMP_LIVE_POS,
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), g.LAMP_SIZE, g.LAMP_AMBER),),
    collision=False,
)

DEAD_LAMP = part(
    "dead_lamp",
    pos=g.LAMP_DEAD_POS,
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), g.LAMP_SIZE, g.LAMP_GREEN),),
    collision=False,
)

OBJECTS = [
    CONSOLE_FIXTURE,
    RED_LOCK_PIN,
    LIVE_LAMP,
    DEAD_LAMP,
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
    console = objects["console_fixture"]
    breaker_disp = float(console.sliders.get("breaker_slider", 0.0))
    test_btn_disp = float(console.sliders.get("test_button_slider", 0.0))
    cover_disp = abs(float(console.sliders.get("cover_slider", 0.0)))  # the cover slides in -X, so its joint value is negative
    service_btn_disp = float(console.sliders.get("service_button_slider", 0.0))

    pin_pos = objects["red_lock_pin"].pos

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    arm_tcps, arm_vels = _arm_info(e)

    result = _SCORER.update(
        breaker_disp=breaker_disp,
        pin_pos=pin_pos,
        test_btn_disp=test_btn_disp,
        cover_disp=cover_disp,
        service_btn_disp=service_btn_disp,
        grippers_open=left_open and right_open,
        arm_tcps=arm_tcps,
        arm_velocities=arm_vels,
    )

    if e is not None:
        live_z = g.LAMP_ON_Z if result.details.get("live_lamp_lit") else g.LAMP_OFF_Z
        dead_z = g.LAMP_ON_Z if result.details.get("dead_lamp_lit") else g.LAMP_OFF_Z
        write_pose(e, "live_lamp", (g.LAMP_LIVE_POS[0], g.LAMP_LIVE_POS[1], live_z))
        write_pose(e, "dead_lamp", (g.LAMP_DEAD_POS[0], g.LAMP_DEAD_POS[1], dead_z))

    _trace({
        "score": result.score,
        "success": result.success,
        "breaker_disp": breaker_disp,
        "cover_disp": cover_disp,
        **result.details,
        "dbg_pin": [round(float(v), 4) for v in pin_pos],
        "dbg_test": round(test_btn_disp, 4),
        "dbg_service": round(service_btn_disp, 4),
        "dbg_tcps": {a: [round(float(v), 3) for v in arm_tcps[a]] for a in (arm_tcps or {})},
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

