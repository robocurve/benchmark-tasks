"""On the station table, in front of the left arm: a main line valve, taped down. Its body is a brass block on a dark
bracket as long as its top plate, with a short yellow pipe running through it from the robot's left to its right. A
dark handle stands on a steel guide plate along the body's top and slides along the plate, away from the arms. It
starts back, at OPEN; pushed all the way forward against the dark stop at the plate's far end, it is SHUT. Beside the
handle's path, on both sides, the plate carries three 2 cm bands of matt tape, white, black and white, the first
starting level with the handle's leading face. The far edges of the bands, 2, 4 and 6 cm ahead of the leading face at
the start, are the marks; the last is at the stop. The handle stays where it is put.

Beyond the valve, further from the arms, two lamps stand on dark posts: on the left a lamp marked FLOW, lit green, and
on the right a lamp marked ALARM, grey.

Run it:  inspect-robots run --embodiment yam_arena -E task=<this file> --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys
from collections import deque

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, ToolState, on_reset, on_step, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, HZ, LAMP_GREEN, LAMP_OFF, LAMP_RED, STEEL, WHITE, YELLOW, Clock, Lamp, distance_xy,
)
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Shut the main line valve slowly: close it one mark at a time, taking at least four seconds from each mark to the "
    "next, until the handle is at the shut stop."
)

BRASS = (0.72, 0.58, 0.28)
BLACK = (0.08, 0.08, 0.09)  # the middle band: darker than the handle, so its edges read on video

# -- the valve, in its own frame: origin at the body's centre, x away from the arms ------------------------------------
VALVE_XY = (0.26, 0.16)
BODY = (0.060, 0.050, 0.050)  # brass block, x y z (m)
PLATE = (0.100, 0.040, 0.004)  # the guide plate along the body's top, 2 cm over each end
BRACKET = (PLATE[0], 0.040, 0.020)  # under the body, down to the table, as long as the plate: a wide base to push on
VALVE_Z = TOP + BRACKET[2] + BODY[2] / 2
PIPE_SECTION = 0.030
PIPE_LEN = 0.130  # along y through the body: 4 cm out of each side
SURFACE = BODY[2] / 2 + PLATE[2]  # the plate's top: the handle slides on it and the finger tips pass over it
HANDLE = (0.030, 0.020, 0.035)  # x, y, z: its top 1.7 cm above the TCP during a push, below the fingers' wider part
HANDLE_TRAVEL = 0.060  # rest is OPEN, the stop at full travel is SHUT
HANDLE_REST_X = -HANDLE_TRAVEL / 2  # the handle's centre runs from -3 cm to +3 cm, centred over the body
HANDLE_BACK = HANDLE_REST_X - HANDLE[0] / 2  # its near face at rest, where the fingers push
HANDLE_FRONT = HANDLE_REST_X + HANDLE[0] / 2  # its leading face at rest, which the bands are measured from
MARKS_AT = (0.020, 0.040, 0.060)  # m of travel: the leading face at each band's far edge
BAND_COLOURS = (WHITE, BLACK, WHITE)
BAND_W = 0.009  # across: beside the handle's path, out to the plate's edge
BAND_T = 0.001
BAND_Y = HANDLE[1] / 2 + 0.001 + BAND_W / 2
STOP = (0.005, HANDLE[1], 0.012)  # across the plate's far end, where the leading face lands at full travel


def _bands() -> list:
    """Three 2 cm bands on each side of the handle's path, from the leading face at rest to the stop."""
    out = []
    for k, colour in enumerate(BAND_COLOURS):
        x0 = HANDLE_FRONT + (MARKS_AT[k - 1] if k else 0.0)
        x1 = HANDLE_FRONT + MARKS_AT[k]
        for side in (1, -1):
            out.append((((x0 + x1) / 2, side * BAND_Y, SURFACE + BAND_T / 2), (x1 - x0, BAND_W, BAND_T), colour))
    return out


MAIN_VALVE = tool(
    "main_valve",
    pos=(*VALVE_XY, VALVE_Z),
    boxes=[
        ((0.0, 0.0, 0.0), BODY, BRASS),
        ((0.0, 0.0, -BODY[2] / 2 - BRACKET[2] / 2), BRACKET, DARK),
        ((0.0, 0.0, 0.0), (PIPE_SECTION, PIPE_LEN, PIPE_SECTION), YELLOW),
        ((0.0, 0.0, BODY[2] / 2 + PLATE[2] / 2), PLATE, STEEL),
        ((HANDLE_FRONT + HANDLE_TRAVEL + STOP[0] / 2, 0.0, SURFACE + STOP[2] / 2), STOP, DARK),
        *_bands(),
    ],
    sliders=[
        # No spring: the handle stays where it is pushed, as a valve stays where it is set.
        Slider("handle", rest=(HANDLE_REST_X, 0.0, SURFACE + HANDLE[2] / 2), size=HANDLE, axis="X", lower=0.0,
               upper=HANDLE_TRAVEL, color=DARK, rate=0.0, damping=8.0, mass=0.02),
    ],
    mass=3.0,  # heavy enough to stay put when pushed; taped down on a real bench
)

# -- the lamps: separate fixed boxes on posts, so they can be recoloured -----------------------------------------------
FLOW_XY = (0.36, 0.22)
SURGE_XY = (0.36, 0.10)
LAMP_POST = (0.030, 0.030, 0.060)  # low enough to stay 4 cm under the wrist camera during the last push
LAMP_SIZE = (0.040, 0.040, 0.030)
LAMP_TOP = TOP + LAMP_POST[2] + LAMP_SIZE[2]
FLOW = Lamp("flow_lamp", {"flow": LAMP_GREEN, "off": LAMP_OFF}, "flow")
SURGE = Lamp("surge_lamp", {"off": LAMP_OFF, "surge": LAMP_RED}, "off")

OBJECTS = [
    Box("flow_lamp_post", pos=(*FLOW_XY, TOP + LAMP_POST[2] / 2), size=LAMP_POST, color=DARK),
    FLOW.box((*FLOW_XY, TOP + LAMP_POST[2] + LAMP_SIZE[2] / 2), LAMP_SIZE),
    Box("surge_lamp_post", pos=(*SURGE_XY, TOP + LAMP_POST[2] / 2), size=LAMP_POST, color=DARK),
    SURGE.box((*SURGE_XY, TOP + LAMP_POST[2] + LAMP_SIZE[2] / 2), LAMP_SIZE),
    MAIN_VALVE,
]

# -- what happened: the handle's crossings and its pace, checked every step --------------------------------------------
READ_EARLY = 0.003  # a mark counts as reached this far before the band edge: what a person can call from video
MARK_1 = MARKS_AT[0] - READ_EARLY  # 1.7 cm of travel: the first mark reached
MARK_2 = MARKS_AT[1] - READ_EARLY  # 3.7 cm: the second mark reached
SHUT_AT = 0.055  # at the stop, within 5 mm
LEVELS = (MARK_1, MARK_2, SHUT_AT)
PAUSE_S = 4.0  # the least time from one level to the next; from open to the first mark is free
PAUSE_STEPS = round(PAUSE_S * HZ)
RATE_CAP = 0.026  # m: past the first mark, the handle advances no more than this in any 4.0 s
REARM = 0.002  # a level is crossed again only after the handle has gone back more than this below it
CLOCK = Clock()
# Per level: ready to cross, and the step it was last crossed. "recent": travel past the first mark over the last 4 s.
PACE = {"armed": [True] * 3, "up": [None] * 3, "gaps": [], "recent": deque(maxlen=PAUSE_STEPS), "surged": False,
        "why": []}


def travel(state: ToolState) -> float:
    """How far the handle has been pushed from OPEN (m), read in the valve's own frame."""
    return float(state.sliders["handle"])


def past_first_mark(h: float) -> float:
    """The travel the pace cap counts: only what lies beyond the first mark."""
    return max(h - MARK_1, 0.0)


def _surge(why: str) -> None:
    PACE["surged"] = True
    if why not in PACE["why"]:
        PACE["why"].append(why)


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    FLOW.reset()
    SURGE.reset()
    h = travel(states["main_valve"]) if "main_valve" in states else 0.0
    PACE.update(armed=[h < level for level in LEVELS], up=[None if h < level else 0 for level in LEVELS], gaps=[],
                recent=deque([past_first_mark(h)], maxlen=PAUSE_STEPS), surged=False, why=[])


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    h = travel(states["main_valve"])
    for i, level in enumerate(LEVELS):  # upwards, so a handle that jumps two levels in one step crosses both, 0 s apart
        if PACE["armed"][i] and h >= level:
            PACE["armed"][i], PACE["up"][i] = False, CLOCK.step
            if i:  # timed from the latest crossing of the level below
                gap = CLOCK.step - PACE["up"][i - 1]
                PACE["gaps"].append((level, gap / HZ))
                if gap < PAUSE_STEPS:
                    _surge("interval")
        elif not PACE["armed"][i] and h < level - REARM:
            PACE["armed"][i] = True  # pulled back below it: the next crossing is timed afresh
    beyond = past_first_mark(h)
    if beyond - min(PACE["recent"]) > RATE_CAP + 1e-9:  # against the furthest back it was in the last 4 s
        _surge("rate")
    PACE["recent"].append(beyond)
    if PACE["surged"]:
        SURGE.set("surge", CLOCK.step)  # latched for the rest of the trial
    FLOW.set("off" if h >= SHUT_AT else "flow", CLOCK.step)


# -- scoring: the end position, with the surge remembered from the whole trial -----------------------------------------
PART_CLOSED = 0.5  # the first mark reached, not shut, no surge
MOVED = 0.030  # a valve body shoved this far, or
TILT_DEG = 20.0  # tipped this far, has been wrecked


def in_place(state: ToolState) -> bool:
    return bool(distance_xy(state, VALVE_XY) <= MOVED and state.tilt_deg() <= TILT_DEG)


def grade(objects, surged: bool) -> float:
    """The score as the outcome table states it."""
    valve = objects["main_valve"]
    if surged or not in_place(valve):
        return 0.0
    h = travel(valve)
    if h >= SHUT_AT:
        return 1.0
    return PART_CLOSED if h >= MARK_1 else 0.0


def success(objects) -> bool:
    return grade(objects, PACE["surged"]) >= 1.0 and objects["main_valve"].resting(max_speed=0.02, max_spin=1.0)


def score(objects) -> float:
    return grade(objects, PACE["surged"])


def distance(objects) -> float:
    """Metres of handle travel left to the shut stop."""
    return max(SHUT_AT - travel(objects["main_valve"]), 0.0)


# -- the scripted demo (left arm): three pushes, a wait parked to the side after each of the first two -----------------
HOVER_Z = TOP + 0.150
PUSH_Z = VALVE_Z + SURFACE + 0.005 + 0.013  # TCP height: the finger tips 5 mm over the guide plate
# Half the closed fingers' extent along x against height above the TCP (m), from the gripper URDF's collision hulls:
# 1.2 cm at the TCP, widening about 0.39 cm per cm, so the handle's top meets them further forward than the pads.
FINGER_TAPER = ((0.000, 0.0120), (0.025, 0.0217), (0.032, 0.0245), (0.055, 0.0334))
HANDLE_TOP_ABOVE_TCP = VALVE_Z + SURFACE + HANDLE[2] - PUSH_Z


def finger_half_x(above_tcp: float) -> float:
    zs, xs = zip(*FINGER_TAPER, strict=True)
    return float(np.interp(above_tcp, zs, xs))


PUSH_FACE = finger_half_x(HANDLE_TOP_ABOVE_TCP)  # the furthest forward the fingers reach over the handle's height
PUSH_BACK = 0.030  # the fingers come down this far behind the handle
OVERSHOOT = 0.006  # commanded past the stop on the last push, so the handle is shut if the arm falls a little short
BACK_OFF = 0.020  # off the handle along the push axis before lifting
STAGES = (0.024, 0.044, HANDLE_TRAVEL)  # where each push leaves the handle: 4 mm past each band edge, then the stop
WAIT_S = 5.5  # parked at each of the first two marks
PARK_Y = VALVE_XY[1] + 0.150  # 15 cm to the left arm's own side, clear of the top camera's view of the bands
TRIM = {"push": (0.0, 0.0)}


def handle_back_x(at: float) -> float:
    """World x of the handle's near face with the handle ``at`` m along its travel."""
    return VALVE_XY[0] + HANDLE_BACK + at


def push_x(start: float, to: float, overshoot: float = 0.0) -> tuple[float, float]:
    """TCP x at the start and end of a push that moves the handle from ``start`` to ``to`` (m of travel)."""
    tx = TRIM["push"][0]
    return handle_back_x(start) + tx - PUSH_BACK - PUSH_FACE, handle_back_x(to) + tx - PUSH_FACE + overshoot


def _push(start: float, to: float, what: str, overshoot: float = 0.0) -> list[Waypoint]:
    s, e = push_x(start, to, overshoot)
    y = VALVE_XY[1] + TRIM["push"][1]
    return [
        Waypoint(1.5, (s, y, HOVER_Z), FINGER_CLOSED, "over the handle"),
        Waypoint(1.2, (s, y, PUSH_Z), FINGER_CLOSED, "down behind the handle"),
        Waypoint(0.5, (s, y, PUSH_Z), FINGER_CLOSED, "settle"),
        Waypoint(2.0, (e, y, PUSH_Z), FINGER_CLOSED, f"push {what}"),
        Waypoint(0.8, (e, y, PUSH_Z), FINGER_CLOSED, "hold"),
        Waypoint(0.5, (e - BACK_OFF, y, PUSH_Z), FINGER_CLOSED, "back off"),
        Waypoint(1.0, (e - BACK_OFF, y, HOVER_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.5, (e - BACK_OFF, PARK_Y, HOVER_Z), FINGER_CLOSED, "park to its side"),
    ]


def _wait(plan: list[Waypoint], what: str) -> Waypoint:
    return Waypoint(WAIT_S, plan[-1].pos, FINGER_CLOSED, f"wait at {what}")


def until(plan: list[Waypoint], label: str) -> float:
    """Seconds from the start of ``plan`` to the end of its first waypoint labelled ``label``."""
    t = 0.0
    for w in plan:
        t += w.duration
        if w.label == label:
            return t
    raise KeyError(label)


def demo():
    """Left arm: push the handle just past the first mark, park and wait; just past the second mark, park and wait;
    then on to the stop, and park. The right arm stays at its ready pose."""
    plan = _push(0.0, STAGES[0], "to the first mark")
    plan.append(_wait(plan, "the first mark"))
    plan += _push(STAGES[0], STAGES[1], "to the second mark")
    plan.append(_wait(plan, "the second mark"))
    plan += _push(STAGES[1], STAGES[2], "to the stop", OVERSHOOT)
    return {"left": plan}
