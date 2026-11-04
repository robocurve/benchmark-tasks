"""On the station table: a rack of four sample tubes standing in a row in front of the left arm, an analyser tray with
four empty slots beside it, a yellow tray further out, and a red call button in front of the right arm.

One tube has leaked: a red pool surrounds its slot in the rack. A leaking tube must not go into the analyser. It goes
in the yellow tray, and pressing the red button calls a technician to deal with it.

Run it:  inspect-robots run --embodiment yam_arena -E task=load_tubes_set_aside_leaking_one.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, ToolState, on_reset, on_step, tool  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Load the sample tubes from the rack into the analyser tray, one tube per slot. If a tube is leaking, don't load "
    "it: put it in the yellow tray and press the red button to call a technician."
)

# -- the tubes and where they go (x along the rack, away from the arms) ------------------------------------------------
TUBE_R = 0.012
TUBE_H = 0.090
TUBE_MASS = 0.03
RACK_Y = 0.17
TRAY_Y = 0.06
SLOT_X = (0.16, 0.21, 0.26, 0.31)
LEAKING = 2  # the third tube from the arms
SLOT_INNER = 0.032
TRAY_INNER = 0.040  # room for an arm that can place to within about 1 cm
SLOT_WALL = 0.003
SLOT_H = 0.030
HOLD_XY = (0.38, 0.13)
HOLD_INNER = 0.070
HOLD_H = 0.020

GREY = (0.55, 0.56, 0.58)
DARK = (0.25, 0.27, 0.30)
TUBE_WHITE = (0.92, 0.92, 0.94)
YELLOW = (0.95, 0.80, 0.10)
RED = (0.75, 0.08, 0.08)
LEAK_RED = (0.55, 0.04, 0.05)
STAINED = (0.78, 0.30, 0.30)  # the leaking tube is smeared red, so it can be told apart


def _collar(name: str, x: float, y: float, inner: float, height: float, wall: float, color) -> list[Box]:
    off = inner / 2 + wall / 2
    z = TOP + height / 2
    long = inner + 2 * wall
    return [
        Box(f"{name}_f", pos=(x + off, y, z), size=(wall, long, height), color=color),
        Box(f"{name}_b", pos=(x - off, y, z), size=(wall, long, height), color=color),
        Box(f"{name}_l", pos=(x, y + off, z), size=(inner, wall, height), color=color),
        Box(f"{name}_r", pos=(x, y - off, z), size=(inner, wall, height), color=color),
    ]


TUBES = [
    Cylinder(f"tube{i}", pos=(x, RACK_Y, TOP + TUBE_H / 2), radius=TUBE_R, height=TUBE_H, mass=TUBE_MASS,
             color=STAINED if i == LEAKING else TUBE_WHITE)
    for i, x in enumerate(SLOT_X)
]

BUTTON_XY = (0.24, -0.12)
BUTTON_BASE = (0.07, 0.07, 0.030)
BUTTON_TRAVEL = 0.008
BUTTON_CAP = (0.040, 0.040, 0.015)
BUTTON_Z = TOP + BUTTON_BASE[2] / 2
BUTTON = tool(
    "call_button",
    pos=(*BUTTON_XY, BUTTON_Z),
    boxes=[((0.0, 0.0, 0.0), BUTTON_BASE, DARK)],
    sliders=[Slider("cap", rest=(0.0, 0.0, BUTTON_BASE[2] / 2 + BUTTON_CAP[2] / 2), size=BUTTON_CAP, axis="Z",
                    lower=-BUTTON_TRAVEL, upper=0.0, color=RED, rate=300.0, damping=8.0, mass=0.01)],
    mass=3.0,  # heavy enough to stay put when pressed; taped down on a real bench
)

OBJECTS = [
    *(w for i, x in enumerate(SLOT_X) for w in _collar(f"rack{i}", x, RACK_Y, SLOT_INNER, SLOT_H, SLOT_WALL, GREY)),
    *(w for i, x in enumerate(SLOT_X) for w in _collar(f"tray{i}", x, TRAY_Y, TRAY_INNER, SLOT_H, SLOT_WALL, DARK)),
    *_collar("hold", *HOLD_XY, HOLD_INNER, HOLD_H, SLOT_WALL, YELLOW),
    Box("leak_pool", pos=(SLOT_X[LEAKING], 0.15, TOP + 0.0005), size=(0.060, 0.10, 0.001), color=LEAK_RED),
    *TUBES,
    BUTTON,
]

# -- what happened: the button stays pressed once pushed, as a real call button does ------------------------------------
PRESSED_AT = 0.7
CALLED = {"yes": False}


@on_reset
def _reset(states) -> None:
    CALLED["yes"] = False


@on_step
def _watch(states) -> None:
    if abs(states["call_button"].sliders["cap"]) >= PRESSED_AT * BUTTON_TRAVEL:
        CALLED["yes"] = True


# -- scoring -------------------------------------------------------------------------------------------------------
PER_GOOD_TUBE = 0.15
HELD_BACK = 0.15
CALLED_FOR_HELP = 0.40
STANDING_DEG = 35.0  # the most a tube can lean inside a 4 cm collar is about 34 degrees
GOOD = tuple(i for i in range(len(SLOT_X)) if i != LEAKING)


def _tilt_deg(state) -> float:
    x, y, z, w = state.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def in_tray_slot(state) -> bool:
    """Standing in any analyser slot."""
    p = state.pos
    near = any(abs(p[0] - x) <= TRAY_INNER / 2 and abs(p[1] - TRAY_Y) <= TRAY_INNER / 2 for x in SLOT_X)
    return bool(near and state.low[2] < TOP + 0.015 and _tilt_deg(state) <= STANDING_DEG)


def in_its_rack_slot(state) -> bool:
    """The leaking tube still in its own slot in the rack."""
    p = state.pos
    return bool(abs(p[0] - SLOT_X[LEAKING]) <= SLOT_INNER / 2 and abs(p[1] - RACK_Y) <= SLOT_INNER / 2
                and state.low[2] < TOP + 0.015)


def in_hold_tray(state) -> bool:
    p = state.pos
    return bool(abs(p[0] - HOLD_XY[0]) <= HOLD_INNER / 2 and abs(p[1] - HOLD_XY[1]) <= HOLD_INNER / 2
                and state.low[2] < TOP + 0.03)


def grade(objects, called: bool) -> float:
    """The score as the outcome table states it."""
    leaking = objects[f"tube{LEAKING}"]
    held = in_hold_tray(leaking)
    if in_tray_slot(leaking) or not (held or in_its_rack_slot(leaking)):
        return 0.0  # loaded, or knocked or thrown out of the way
    n = sum(in_tray_slot(objects[f"tube{i}"]) for i in GOOD)
    if held and any(in_hold_tray(objects[f"tube{i}"]) for i in GOOD):
        return min(0.25, PER_GOOD_TUBE * n + HELD_BACK)  # holding good tubes back too isn't sorting
    return round(PER_GOOD_TUBE * n + (HELD_BACK if held else 0.0) + (CALLED_FOR_HELP if held and called else 0.0), 6)


def success(objects) -> bool:
    return grade(objects, CALLED["yes"]) >= 1.0 and all(objects[f"tube{i}"].resting(max_speed=0.02, max_spin=1.0)
                                                         for i in range(len(SLOT_X)))


def score(objects) -> float:
    return grade(objects, CALLED["yes"])


def distance(objects) -> float:
    """Metres from the leaking tube to the yellow tray."""
    return float(np.linalg.norm(objects[f"tube{LEAKING}"].pos[:2] - np.array(HOLD_XY)))


# -- the scripted demo: left arm moves the tubes, right arm presses the button --------------------------------------------
GRIP_Z = TOP + 0.060
TRAVEL_Z = TOP + 0.150
PLACE_Z = TOP + 0.066
HOLD_PLACE_Z = TOP + 0.066
TRIM = {"pick": (0.0, 0.0), "place": {0: (0.004, -0.005), 1: (0.003, -0.003), 3: (0.001, -0.003)}, "hold": (0.0, 0.0)}


def _left_plan() -> list[Waypoint]:
    plan: list[Waypoint] = []
    px, py = TRIM["pick"]
    for i in GOOD:
        qx, qy = TRIM["place"][i]
        plan += pick_place_waypoints((SLOT_X[i] + px, RACK_Y + py, GRIP_Z), (SLOT_X[i] + qx, TRAY_Y + qy, PLACE_Z),
                                     travel_z=TRAVEL_Z, speed=0.7)
    hx, hy = TRIM["hold"]
    plan += pick_place_waypoints((SLOT_X[LEAKING] + px, RACK_Y + py, GRIP_Z),
                                 (HOLD_XY[0] + hx, HOLD_XY[1] + hy, HOLD_PLACE_Z), travel_z=TRAVEL_Z, speed=0.7)
    return plan


def _right_plan(start_after: float) -> list[Waypoint]:
    x, y = BUTTON_XY
    top = BUTTON_Z + BUTTON_BASE[2] / 2 + BUTTON_CAP[2]
    press = top + 0.013 - BUTTON_TRAVEL - 0.002
    above = top + 0.060
    return [
        Waypoint(start_after, (x, y, above), FINGER_CLOSED, "wait for the tubes"),
        Waypoint(1.5, (x, y, press), FINGER_CLOSED, "press the button"),
        Waypoint(0.8, (x, y, press), FINGER_CLOSED, "hold"),
        Waypoint(1.5, (x, y, above), FINGER_CLOSED, "release"),
    ]


def demo():
    """Left arm: the three good tubes into the analyser, the leaking one into the yellow tray. Right arm: press the
    call button once the tubes are done."""
    left = _left_plan()
    done = sum(w.duration for w in left)
    return {"left": left, "right": _right_plan(done)}


assert FINGER_OPEN and ToolState
