"""On the station table, in front of the arms: a grey post on a base plate carrying a spring hook. The hook is a
square steel pin with a small upturned tip, pointing toward the robot from a carriage that slides down the post's
front face against a spring. Beside the carriage is a yellow drag pointer: the hook pushes it down and it stays at the
lowest point the hook has reached. Next to the pointer is a scale strip, white at the top and red lower down. To the
left, standing on the table, is a grey weight: a block with a rigid square ring on top. Only the weight is loose.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SUCCESS_HOLD_S = 4.0
TOP = TABLE_TOP[SCENE]
G = 9.81

GREY = (0.55, 0.56, 0.58)
STEEL = (0.75, 0.76, 0.78)
DARK = (0.25, 0.25, 0.27)
YELLOW = (0.95, 0.80, 0.10)
WHITE = (0.95, 0.95, 0.95)
RED = (0.85, 0.12, 0.12)

# -- the spring hook -----------------------------------------------------------------------------------------------
SPRING = 150.0  # N/m
DAMPING = 0.6  # N s/m: lightly damped (damping ratio about 0.04 with the weight on)
TRAVEL = 0.07  # m
HOOK_MASS = 0.02  # kg
PIN_Z = TOP + 0.17  # top face of the pin, hook unloaded
PIN_W = 0.010  # square pin (m)
PIN_LEN = 0.060
TIP_H = 0.006  # upturned tip above the pin's top face
FACE_X = 0.365  # post front face
CARRIAGE_T = 0.008
PIN_ROOT_X = FACE_X - CARRIAGE_T  # 0.357
PIN_TIP_X = PIN_ROOT_X - PIN_LEN  # 0.297
POST_DEPTH, POST_W = 0.030, 0.078
POST_Y = 0.019  # post centre y; the pin is at y = 0
POST_TOP = PIN_Z + 0.03
POINTER_Y = (0.020, 0.035)
STRIP_Y = (0.040, 0.055)
STRIP_PROUD = 0.003  # the strip stands 3 mm off the post face; the pointer 5 mm
ZERO_Z = PIN_Z + 0.010  # the pointer's top edge at zero
RED_FROM = 0.037  # m of extension where the red band starts (1.4 x the weight's static extension)
STRIP_END = 0.075
POINTER_SPRING, POINTER_MASS = 400.0, 0.01

# -- the weight ----------------------------------------------------------------------------------------------------
W_MASS = 0.4
BODY = (0.05, 0.05, 0.06)
RING_IN_W, RING_IN_H, BAR = 0.030, 0.028, 0.005
W_START = (0.25, 0.12, TOP + BODY[2] / 2)
STATIC_EXT = W_MASS * G / SPRING  # 26.2 mm
HOOK_REST = HOOK_MASS * G / SPRING  # the unloaded hook's own sag, 1.3 mm: the scale's zero
STILL_SPEED = 0.03  # m/s: the hung weight keeps a sub-millimetre contact jitter in the simulator
LOADED_BAND = 0.002  # hook extension within this of STATIC_EXT: the hook carries the whole weight
HANG_X = PIN_ROOT_X - 0.048  # where the demo hangs the ring on the pin, 0.309

POST = parts.Part(
    "spring_hook",
    pos=(0.38, POST_Y, TOP),
    shapes=[
        parts.box((0.12, 0.12, 0.010), at=(0.0, 0.0, 0.005), color=DARK),
        parts.box((POST_DEPTH, POST_W, POST_TOP - TOP - 0.01), at=(0.0, 0.0, (POST_TOP - TOP + 0.01) / 2), color=GREY),
        parts.box((STRIP_PROUD + 0.001, STRIP_Y[1] - STRIP_Y[0], RED_FROM), color=WHITE,
                  at=(FACE_X - 0.38 - (STRIP_PROUD - 0.001) / 2, sum(STRIP_Y) / 2 - POST_Y, ZERO_Z - TOP - RED_FROM / 2)),
        parts.box((STRIP_PROUD + 0.001, STRIP_Y[1] - STRIP_Y[0], STRIP_END - RED_FROM), color=RED,
                  at=(FACE_X - 0.38 - (STRIP_PROUD - 0.001) / 2, sum(STRIP_Y) / 2 - POST_Y, ZERO_Z - TOP - (RED_FROM + STRIP_END) / 2)),
    ],
    joints=[
        parts.Slider(
            "hook", axis=(0.0, 0.0, -1.0), travel=TRAVEL, mass=HOOK_MASS, spring=SPRING, damping=DAMPING,
            shapes=[
                parts.box((CARRIAGE_T, 0.034, 0.040), at=(FACE_X - CARRIAGE_T / 2 - 0.38, -POST_Y, PIN_Z - 0.005 - TOP),
                          color=DARK),
                parts.box((PIN_LEN, PIN_W, PIN_W), at=((PIN_ROOT_X + PIN_TIP_X) / 2 - 0.38, -POST_Y,
                                                       PIN_Z - PIN_W / 2 - TOP), color=STEEL),
                parts.box((0.005, PIN_W, TIP_H), at=(PIN_TIP_X + 0.0025 - 0.38, -POST_Y, PIN_Z + TIP_H / 2 - TOP),
                          color=STEEL),
            ]),
        parts.Slider(
            "pointer", axis=(0.0, 0.0, -1.0), travel=STRIP_END, mass=POINTER_MASS, spring=POINTER_SPRING,
            damping=4.0,
            shapes=[parts.box((0.005, POINTER_Y[1] - POINTER_Y[0], 0.008), color=YELLOW,
                              at=(FACE_X - 0.0025 - 0.38, sum(POINTER_Y) / 2 - POST_Y, ZERO_Z - 0.004 - TOP))]),
    ],
)

_RING_Z0 = BODY[2] / 2  # ring opening from the body top (weight frame, origin at the body centre)
WEIGHT = parts.Part(
    "weight",
    pos=W_START,
    mass=W_MASS,
    shapes=[
        parts.box(BODY, color=GREY),
        *[parts.box((BAR, BAR, RING_IN_H + BAR), color=DARK,
                    at=(0.0, s * (RING_IN_W + BAR) / 2, _RING_Z0 + (RING_IN_H + BAR) / 2)) for s in (-1, 1)],
        parts.box((BAR, RING_IN_W + 2 * BAR, BAR), color=DARK, at=(0.0, 0.0, _RING_Z0 + RING_IN_H + BAR / 2)),
    ],
)

OBJECTS = [POST, WEIGHT]

# -- scoring -------------------------------------------------------------------------------------------------------
# The record is kept over the whole trial: the largest hook extension seen at a control step (20 Hz), which the
# drag pointer then shows. It is cleared only by a scene reset
# (on_reset), which the robot cannot cause; offline it starts empty.
_PEAK = [0.0]


def _ext(objects) -> float:
    """Hook extension below its unloaded rest (m)."""
    return float(objects["spring_hook"].joints["hook"]) - HOOK_REST


def _record(objects) -> float:
    _PEAK[0] = max(_PEAK[0], _ext(objects))
    return _PEAK[0]


@parts.on_reset
def _clear(sim):
    _PEAK[0] = 0.0
    _STILL["steps"] = 0


@parts.on_step
def _drag_pointer(sim):
    peak = _record(sim.states())
    sim.set_target("spring_hook", "pointer", peak - POINTER_MASS * G / POINTER_SPRING)


def _quat_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def on_hook(objects) -> bool:
    """The pin passes through the ring's opening: where the pin's centre line crosses the ring's plane, it lies
    inside the opening, and the pin reaches that plane (the ring is between the tip and the carriage)."""
    w, h = objects["weight"], objects["spring_hook"]
    # the pin's centre line in the world: the hook only moves straight down
    drop = float(h.joints["hook"])
    y0, zc = 0.0, PIN_Z - PIN_W / 2 - drop
    r = _quat_matrix(w.quat_xyzw)
    a, b = np.array([PIN_TIP_X, y0, zc]), np.array([PIN_ROOT_X, y0, zc])
    la, lb = r.T @ (a - w.pos), r.T @ (b - w.pos)
    if not (la[0] < 0.0 < lb[0] or lb[0] < 0.0 < la[0]):
        return False
    p = la + (lb - la) * (0.0 - la[0]) / (lb[0] - la[0])
    return bool(abs(p[1]) < RING_IN_W / 2 and _RING_Z0 < p[2] < _RING_Z0 + RING_IN_H)


def hanging_released(objects) -> bool:
    """On the hook, off the table (bottom at least 2 cm up), still, and the hook carrying the whole weight."""
    w = objects["weight"]
    return bool(
        on_hook(objects)
        and w.low[2] >= TOP + 0.02 - 1e-6
        and w.resting(max_speed=STILL_SPEED, max_spin=1.0)
        and abs(_ext(objects) - STATIC_EXT) <= LOADED_BAND + 1e-6
    )


# "Still over the last 1.5 s": in the simulator, the hung-and-released test has held at every control step for the
# last 30 steps (1.5 s at 20 Hz). Counted by on_step; offline (near_misses.py, no simulator) the window is not applied.
STILL_STEPS = 30
_STILL = {"steps": 0, "sim": False}


@parts.on_step
def _count_still(sim):
    _STILL["sim"] = True
    _STILL["steps"] = _STILL["steps"] + 1 if hanging_released(sim.states()) else 0


def _still_long_enough() -> bool:
    return not _STILL["sim"] or _STILL["steps"] >= STILL_STEPS


def success(objects) -> bool:
    peak = _record(objects)
    return hanging_released(objects) and _still_long_enough() and peak < RED_FROM - 1e-6


def score(objects) -> float:
    """1.0 hung and let go with the peak short of the red band; else 0 (pass/fail), including a shock-loaded hook."""
    peak = _record(objects)
    if not (hanging_released(objects) and _still_long_enough()):
        return 0.0
    return 1.0 if peak < RED_FROM - 1e-6 else 0.0


# -- demo: left arm --------------------------------------------------------------------------------------------------
# Grip the body near its top (fingers along y, clear of the ring), carry it in front of the pin's tip with the ring's
# top bar 6 mm above the tip, slide the ring over the pin, then lower slowly well past contact so the hook takes the
# weight, wait, open and lift away. TCP is 10 mm below the body top.
GRIP_DOWN = 0.010
# Measured in a trace: once lifted, the carried weight sits 20 mm to the robot's right of the commanded TCP and 4 mm
# lower. The demo commands the TCP that much the other way from the pin onward.
COMP_Y, COMP_Z = 0.020, 0.004


def _tcp_z_for_ring_bar(bar_bottom_z):
    """TCP height that puts the ring's top-bar underside at bar_bottom_z."""
    return bar_bottom_z - (BODY[2] / 2 + RING_IN_H) + BODY[2] / 2 - GRIP_DOWN


def demo():
    x0, y0, z0 = W_START
    grip_z = z0 + BODY[2] / 2 - GRIP_DOWN
    thread_z = _tcp_z_for_ring_bar(PIN_Z + TIP_H + 0.006)
    contact_z = _tcp_z_for_ring_bar(PIN_Z)
    travel_z = thread_z + 0.01
    front_x = PIN_TIP_X - 0.042  # fingers (34 mm ahead of the TCP) clear of the tip while moving across
    plan = [
        Waypoint(1.5, (x0, y0, travel_z), FINGER_OPEN, "above the weight"),
        Waypoint(1.5, (x0, y0, grip_z), FINGER_OPEN, "descend"),
        Waypoint(0.8, (x0, y0, grip_z), FINGER_CLOSED, "close"),
        Waypoint(1.2, (x0, y0, travel_z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (front_x, COMP_Y, thread_z + COMP_Z), FINGER_CLOSED, "in front of the tip"),
        Waypoint(2.0, (HANG_X, COMP_Y, thread_z + COMP_Z), FINGER_CLOSED, "ring over the pin"),
    ]
    # Held well short of the static stretch (about 20 mm of 26 mm), so the gripped hold is outside the scored band;
    # the last 6 mm are taken up when the fingers open. The arm gives way about 1.5 mm under the hook's push.
    push_z = contact_z - STATIC_EXT + 0.005
    plan += [
        Waypoint(1.0, (HANG_X, COMP_Y, contact_z + 0.002 + COMP_Z), FINGER_CLOSED, "to just above contact"),
        Waypoint(4.0, (HANG_X, COMP_Y, push_z), FINGER_CLOSED, "lower slowly onto the hook"),
        Waypoint(1.5, (HANG_X, COMP_Y, push_z), FINGER_CLOSED, "let the hook settle"),
        Waypoint(0.8, (HANG_X, COMP_Y, push_z), FINGER_OPEN, "open"),
    ]
    plan += [
        Waypoint(0.8, (HANG_X, COMP_Y, plan[-1].pos[2]), FINGER_OPEN, "wait open"),
        Waypoint(1.5, (0.20, COMP_Y, plan[-1].pos[2] + 0.01), FINGER_OPEN, "back out toward the robot"),
        Waypoint(1.0, (0.20, 0.12, TOP + 0.15), FINGER_OPEN, "clear"),
    ]
    return {"left": plan}
