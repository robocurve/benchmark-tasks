"""On the station table, in front of the left arm: a main line valve, taped down. Its body is a brass block on a dark
bracket, with a short yellow pipe running through it from the robot's left to its right. A dark handle stands on a
steel guide plate along the body's top and slides along it, away from the arms. It starts back, at OPEN; pushed all the
way forward against the dark stop at the end of the slot, it is SHUT. White marks on both sides of the slot stand 2, 4
and 6 cm ahead of the handle's leading face at the start; the last pair is at the stop. The handle stays where it is
put.

Beyond the valve, further from the arms, two lamps stand on dark posts: on the left a lamp marked FLOW, lit green, and
on the right a lamp marked ALARM, grey.

Run it:  inspect-robots run --embodiment yam_arena -E task=<this file> --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

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
    "Shut the main line valve slowly: close it one mark at a time, pausing at least four seconds at each mark, until "
    "the handle is at the shut stop."
)

BRASS = (0.72, 0.58, 0.28)

# -- the valve, in its own frame: origin at the body's centre, x away from the arms ------------------------------------
VALVE_XY = (0.26, 0.16)
BODY = (0.060, 0.050, 0.050)  # brass block, x y z (m)
BRACKET = (0.050, 0.040, 0.020)  # under the body, down to the table
VALVE_Z = TOP + BRACKET[2] + BODY[2] / 2
PIPE_SECTION = 0.030
PIPE_LEN = 0.130  # along y through the body: 4 cm out of each side
PLATE = (0.100, 0.040, 0.004)  # the guide plate along the body's top, 2 cm over each end
SURFACE = BODY[2] / 2 + PLATE[2]  # the plate's top: the handle slides on it and the finger tips pass over it
HANDLE = (0.030, 0.020, 0.050)  # x, y, z: tall enough for the fingers to push on
HANDLE_TRAVEL = 0.060  # rest is OPEN, the stop at full travel is SHUT
HANDLE_REST_X = -HANDLE_TRAVEL / 2  # the handle's centre runs from -3 cm to +3 cm, centred over the body
HANDLE_BACK = HANDLE_REST_X - HANDLE[0] / 2  # its near face at rest, where the fingers push
HANDLE_FRONT = HANDLE_REST_X + HANDLE[0] / 2  # its leading face at rest, which the marks are measured from
MARKS_AT = (0.020, 0.040, 0.060)  # m of travel: the leading face reaches each mark
MARK = (0.002, 0.009, 0.001)  # a white strip
MARK_Y = HANDLE[1] / 2 + 0.001 + MARK[1] / 2  # beside the slot, on both sides of the handle
STOP = (0.005, HANDLE[1], 0.012)  # across the slot's end, where the leading face lands at full travel

MAIN_VALVE = tool(
    "main_valve",
    pos=(*VALVE_XY, VALVE_Z),
    boxes=[
        ((0.0, 0.0, 0.0), BODY, BRASS),
        ((0.0, 0.0, -BODY[2] / 2 - BRACKET[2] / 2), BRACKET, DARK),
        ((0.0, 0.0, 0.0), (PIPE_SECTION, PIPE_LEN, PIPE_SECTION), YELLOW),
        ((0.0, 0.0, BODY[2] / 2 + PLATE[2] / 2), PLATE, STEEL),
        ((HANDLE_FRONT + HANDLE_TRAVEL + STOP[0] / 2, 0.0, SURFACE + STOP[2] / 2), STOP, DARK),
        *(((HANDLE_FRONT + at, side * MARK_Y, SURFACE + MARK[2] / 2), MARK, WHITE)
          for at in MARKS_AT for side in (1, -1)),
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

# -- what happened: the handle's crossings, timed every step -----------------------------------------------------------
MARK_1 = 0.020  # m of travel: the first mark is crossed
MARK_2 = 0.040  # the second mark is crossed
SHUT_AT = 0.055  # at the stop, within 5 mm
LEVELS = (MARK_1, MARK_2, SHUT_AT)
PAUSE_S = 4.0  # the least time from one crossing to the next; from open to the first mark is free
PAUSE_STEPS = round(PAUSE_S * HZ)
REARM = 0.002  # a mark is crossed again only after the handle has gone back more than this below it
CLOCK = Clock()
PACE = {"armed": [True] * 3, "up": [None] * 3, "gaps": [], "surged": False}  # per level: ready to cross, step crossed


def travel(state: ToolState) -> float:
    """How far the handle has been pushed from OPEN (m), read in the valve's own frame."""
    return float(state.sliders["handle"])


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    FLOW.reset()
    SURGE.reset()
    h = travel(states["main_valve"]) if "main_valve" in states else 0.0
    PACE.update(armed=[h < level for level in LEVELS], up=[None if h < level else 0 for level in LEVELS], gaps=[],
                surged=False)


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    h = travel(states["main_valve"])
    for i, level in enumerate(LEVELS):  # upwards, so a handle that jumps two levels in one step crosses both
        if PACE["armed"][i] and h >= level:
            PACE["armed"][i], PACE["up"][i] = False, CLOCK.step
            if i:  # timed from the latest crossing of the level below
                gap = CLOCK.step - PACE["up"][i - 1]
                PACE["gaps"].append((level, gap / HZ))
                if gap < PAUSE_STEPS:
                    PACE["surged"] = True
        elif not PACE["armed"][i] and h < level - REARM:
            PACE["armed"][i] = True  # pulled back below it: the next crossing is timed afresh
    if PACE["surged"]:
        SURGE.set("surge", CLOCK.step)  # latched for the rest of the trial
    FLOW.set("off" if h >= SHUT_AT else "flow", CLOCK.step)


# -- scoring: the end position, with the surge remembered from the whole trial -----------------------------------------
PART_CLOSED = 0.5  # past the first mark, not shut, no surge
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
PUSH_BACK = 0.030  # the fingers come down this far behind the handle
FINGER_X = 0.020  # half the closed fingers' extent along x from the TCP up to 4 cm above it, where they meet the handle
OVERSHOOT = 0.006  # commanded past the stop on the last push, so the handle is shut if the arm falls a little short
BACK_OFF = 0.020  # off the handle along the push axis before lifting
STAGES = (0.024, 0.044, HANDLE_TRAVEL)  # where each push leaves the handle: 4 mm past each mark, then the stop
WAIT_S = 5.5  # parked at each of the first two marks
PARK_Y = VALVE_XY[1] + 0.150  # 15 cm to the left arm's own side, clear of the top camera's view of the marks
TRIM = {"push": (0.0, 0.0)}


def handle_back_x(at: float) -> float:
    """World x of the handle's near face with the handle ``at`` m along its travel."""
    return VALVE_XY[0] + HANDLE_BACK + at


def push_x(start: float, to: float, overshoot: float = 0.0) -> tuple[float, float]:
    """TCP x at the start and end of a push that moves the handle from ``start`` to ``to`` (m of travel)."""
    tx = TRIM["push"][0]
    return handle_back_x(start) + tx - PUSH_BACK - FINGER_X, handle_back_x(to) + tx - FINGER_X + overshoot


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
