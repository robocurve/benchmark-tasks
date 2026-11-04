"""On the station table, in front of the left arm: a white feed tray holding three cubes, two red and one blue, and two
open bins beyond it, a red one and a blue one. In front of the right arm: a white tank, open at the top, with a strip
of eight level segments up its front face that lights from the bottom as the tank fills, and a brass inlet valve block
nearer the arm with a dark handle that slides along a slot in the block's top. At the near end of the slot (toward
the arm) the inlet is open; pushed to the far end (toward the tank) it is closed. A grey lamp on a short base stands
beyond the tank, and a dark mat lies around the tank's foot.

Run it:  inspect-robots run --embodiment yam_arena -E task=close_the_inlet_when_the_high_level_alarm_comes_on.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from benchkit import Slider, on_reset, on_step, recolor, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, HZ, LAMP_OFF, LAMP_RED, RED, STRIP_DARK, WATER_BLUE, WHITE, Clock, Lamp, Level, distance_xy, over, pushed,
)
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Sort the cubes from the feed tray into the bins by colour: red cubes in the red bin, the blue cube in the blue "
    "bin. The tank beside you is filling. If its high-level light turns red, close the inlet valve before it "
    "overflows, then carry on sorting."
)

BLUE = (0.20, 0.35, 0.65)
BRASS = (0.72, 0.56, 0.22)
COLOUR = {"red": RED, "blue": BLUE}

# -- the sorting job: a feed tray of cubes in front of the left arm, two bins beyond it --------------------------------
CUBE = 0.035
CUBE_MASS = 0.04
TRAY_XY = (0.18, 0.11)
TRAY_SIZE = (0.16, 0.10, 0.004)
# The reds stand 11 cm apart with the blue between and beyond them, so a gripper opened over any cube clears the others.
CUBES = {"cube0": ("red", (0.125, 0.085)), "cube1": ("red", (0.235, 0.085)), "cube2": ("blue", (0.18, 0.14))}
BIN_XY = {"red": (0.32, 0.10), "blue": (0.32, 0.22)}
BIN_INNER = {"red": (0.09, 0.12), "blue": (0.09, 0.09)}  # the red bin is long enough for two cubes side by side
BIN_H = 0.030
BIN_WALL = 0.004
IN_BIN_BOTTOM = TOP + 0.015  # a cube counts with its underside under this: on the bin's floor, not on another cube

# -- the tank, its level strip, the alarm lamp and the spill mat, in front of the right arm ----------------------------
TANK_XY = (0.30, -0.14)
TANK_INNER = 0.12
TANK_WALL = 0.006
TANK_H = 0.16
TANK_FACE_X = TANK_XY[0] - TANK_INNER / 2 - TANK_WALL  # the front wall's outer face, toward the arms
SEGMENTS = 8
SEGMENT = (0.004, 0.080, 0.016)  # one level segment, on the front face
LEVEL = Level("tank_level", SEGMENTS)
LAMP_XY = (0.38, -0.26)
LAMP_BASE = (0.056, 0.056, 0.020)
LAMP_BOX = (0.050, 0.050, 0.030)
LAMP = Lamp("alarm_lamp", {"off": LAMP_OFF, "red": LAMP_RED}, "off")
MAT = 0.17
MAT_DRY = STRIP_DARK
MAT_WET = WATER_BLUE

# -- the inlet valve: a brass block with a handle that slides along it, in front of the right arm ----------------------
VALVE_XY = (0.17, -0.22)  # 6 cm in front of the tank's wall, so the gripper clears it during the push
BLOCK = (0.050, 0.080, 0.040)
HANDLE = (0.022, 0.012, 0.036)  # the grip: 3.6 cm tall above the block, for the closed fingers to push on
HANDLE_TRAVEL = 0.030
HANDLE_REST_Y = -HANDLE_TRAVEL / 2  # open at the near end of the slot (toward the arm); +y to the far end closes
SLOT = (0.006, HANDLE_TRAVEL + HANDLE[1] + 0.004, 0.001)
HANDLE_Z = BLOCK[2] / 2 + SLOT[2] + 0.0005 + HANDLE[2] / 2
VALVE_Z = TOP + BLOCK[2] / 2
CLOSED_AT = 0.7  # of the handle's travel: 2.1 cm of 3
VALVE_UPRIGHT_DEG = 20.0  # a valve knocked over further than this reads as open: its handle means nothing

INLET_VALVE = tool(
    "inlet_valve",
    pos=(*VALVE_XY, VALVE_Z),
    boxes=[
        ((0.0, 0.0, 0.0), BLOCK, BRASS),
        ((0.0, 0.0, BLOCK[2] / 2 + SLOT[2] / 2), SLOT, DARK),
    ],
    sliders=[
        # No spring: the handle stays where it is pushed. When it crossed 70% of its travel is tracked step by step.
        Slider("handle", rest=(0.0, HANDLE_REST_Y, HANDLE_Z), size=HANDLE, axis="Y", lower=0.0, upper=HANDLE_TRAVEL,
               color=DARK, rate=0.0, damping=8.0, mass=0.02),
    ],
    mass=3.0,  # heavy enough to stay put when pushed; taped down on a real bench
)


def _walls(name: str, xy, inner, height: float, wall: float, color) -> list[Box]:
    """Four fixed walls around a rectangular opening ``inner`` (x, y), standing on the table."""
    x, y = xy
    ix, iy = inner
    z = TOP + height / 2
    return [
        Box(f"{name}_f", pos=(x + ix / 2 + wall / 2, y, z), size=(wall, iy + 2 * wall, height), color=color),
        Box(f"{name}_b", pos=(x - ix / 2 - wall / 2, y, z), size=(wall, iy + 2 * wall, height), color=color),
        Box(f"{name}_l", pos=(x, y + iy / 2 + wall / 2, z), size=(ix, wall, height), color=color),
        Box(f"{name}_r", pos=(x, y - iy / 2 - wall / 2, z), size=(ix, wall, height), color=color),
    ]


OBJECTS = [
    Box("feed_tray", pos=(*TRAY_XY, TOP + TRAY_SIZE[2] / 2), size=TRAY_SIZE, color=WHITE),
    *(Box(name, pos=(*xy, TOP + TRAY_SIZE[2] + CUBE / 2), size=(CUBE,) * 3, mass=CUBE_MASS, color=COLOUR[colour])
      for name, (colour, xy) in CUBES.items()),
    *(w for colour in ("red", "blue")
      for w in _walls(f"{colour}_bin", BIN_XY[colour], BIN_INNER[colour], BIN_H, BIN_WALL, COLOUR[colour])),
    Box("spill_mat", pos=(*TANK_XY, TOP + 0.0005), size=(MAT, MAT, 0.001), color=MAT_DRY),
    *_walls("tank", TANK_XY, (TANK_INNER, TANK_INNER), TANK_H, TANK_WALL, WHITE),
    Box("tank_floor", pos=(*TANK_XY, TOP + 0.003), size=(TANK_INNER, TANK_INNER, 0.004), color=WATER_BLUE),
    *LEVEL.boxes((TANK_FACE_X - SEGMENT[0] / 2 - 0.001, TANK_XY[1]), TOP + 0.010, SEGMENT, lit=2),
    Box("lamp_base", pos=(*LAMP_XY, TOP + LAMP_BASE[2] / 2), size=LAMP_BASE, color=DARK),
    LAMP.box((*LAMP_XY, TOP + LAMP_BASE[2] + LAMP_BOX[2] / 2), LAMP_BOX),
    INLET_VALVE,
]

# -- the process: the tank fills while the inlet is open; the lamp, strip and mat show what the sim can't --------------
START_ML = 200.0
INFLOW_ML_PER_S = 20.0  # one 100 mL segment every 5 s
SEGMENT_ML = 100.0
FULL_ML = SEGMENTS * SEGMENT_ML  # 800 mL: above this the tank overflows
ALARM_ML = 600.0  # the high-level alarm
ALARM_S = (ALARM_ML - START_ML) / INFLOW_ML_PER_S  # 20 s after the start if the inlet stays open
OVERFLOW_S = (FULL_ML - START_ML) / INFLOW_ML_PER_S  # 30 s
CLOCK = Clock()
STATE = {"ml": START_ML, "alarm_at": None, "closed_at": None, "overflow_at": None}  # times in control steps


def inlet_closed(valve) -> bool:
    """The handle past 70% of its travel on a valve still standing."""
    return pushed(valve, "handle", HANDLE_TRAVEL, CLOSED_AT) and valve.tilt_deg() <= VALVE_UPRIGHT_DEG


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    STATE.update(ml=START_ML, alarm_at=None, closed_at=None, overflow_at=None)
    LAMP.reset()
    LEVEL.show(START_ML / FULL_ML)
    recolor("spill_mat", MAT_DRY)


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    closed = inlet_closed(states["inlet_valve"])
    if closed and STATE["closed_at"] is None:
        STATE["closed_at"] = CLOCK.step
    if not closed:
        STATE["ml"] += INFLOW_ML_PER_S / HZ
    if STATE["alarm_at"] is None and STATE["ml"] >= ALARM_ML:
        STATE["alarm_at"] = CLOCK.step
        LAMP.set("red", CLOCK.step)
    if STATE["overflow_at"] is None and STATE["ml"] > FULL_ML:
        STATE["overflow_at"] = CLOCK.step
        recolor("spill_mat", MAT_WET)
    LEVEL.show(STATE["ml"] / FULL_ML)


# -- scoring -------------------------------------------------------------------------------------------------------
PER_CUBE = 0.2
CLOSED_IN_TIME = 0.4
EARLY_CAP = 0.3  # the inlet closed before the alarm: the batch was stopped early, whatever the sorting


def in_bin(cube, colour: str) -> bool:
    """On the floor of the ``colour`` bin: centre inside its walls, underside under 1.5 cm up."""
    ix, iy = BIN_INNER[colour]
    return bool(over(cube, BIN_XY[colour], (ix / 2, iy / 2)) and cube.low[2] < IN_BIN_BOTTOM)


def sorted_cubes(objects) -> int:
    """How many cubes stand in the bin of their own colour."""
    return sum(in_bin(objects[name], colour) for name, (colour, _) in CUBES.items())


def closed_early(state) -> bool:
    """The first close came before the alarm (or the alarm never came, because the inlet was shut first)."""
    return state["closed_at"] is not None and (state["alarm_at"] is None or state["closed_at"] < state["alarm_at"])


def grade(objects, state) -> float:
    """The score as the outcome table states it."""
    if state["overflow_at"] is not None:
        return 0.0
    base = PER_CUBE * sorted_cubes(objects)
    if state["closed_at"] is None:
        return round(base, 6)  # the inlet is still open and the tank still filling
    if closed_early(state):
        return round(min(base, EARLY_CAP), 6)
    return round(base + (CLOSED_IN_TIME if inlet_closed(objects["inlet_valve"]) else 0.0), 6)


def success(objects) -> bool:
    return grade(objects, STATE) >= 1.0 and all(objects[n].resting(max_speed=0.02, max_spin=1.0) for n in CUBES)


def score(objects) -> float:
    return grade(objects, STATE)


def distance(objects) -> float:
    """Metres from the farthest unsorted cube to its bin, plus the handle's travel left once the alarm is on."""
    apart = max((distance_xy(objects[n], BIN_XY[c]) for n, (c, _) in CUBES.items() if not in_bin(objects[n], c)),
                default=0.0)
    handle = objects["inlet_valve"].sliders["handle"]
    left = max(HANDLE_TRAVEL - handle, 0.0) if STATE["alarm_at"] is not None else 0.0
    return float(apart + left)


# -- the scripted demo: left arm sorts the cubes, right arm waits for the alarm and closes the inlet -------------------
GRIP_Z = TOP + TRAY_SIZE[2] + 0.022
TRAVEL_Z = TOP + 0.150
PLACE_Z = TOP + BIN_H + 0.013 + 0.009  # the pad tips 9 mm above the rim when the fingers open: the cube drops 3 cm
DROP_XY = {"cube0": (0.32, 0.075), "cube1": (0.32, 0.125), "cube2": (0.32, 0.22)}  # the reds side by side
SORT_SPEED = 1.0
PUSH_Z = TOP + BLOCK[2] + 0.005 + 0.013  # the pad tips 5 mm above the block, against the handle
HOVER_Z = TOP + 0.150
FINGER_Y = 0.0065  # half the closed fingers' width along y, near the tips
OVERSHOOT = 0.006  # commanded past the stop, so the handle still closes if the arm falls a little short
HANDLE_FACE_OPEN_Y = VALVE_XY[1] + HANDLE_REST_Y - HANDLE[1] / 2
HANDLE_FACE_SHUT_Y = HANDLE_FACE_OPEN_Y + HANDLE_TRAVEL
PUSH_START_Y = HANDLE_FACE_OPEN_Y - 0.030
PUSH_END_Y = HANDLE_FACE_SHUT_Y - FINGER_Y + OVERSHOOT
PUSH_AT_S = 22.0  # the alarm comes on at 20 s; the push starts 2 s later and takes 4 s
HOVER_S, DOWN_S, SETTLE_S, PUSH_S = 1.5, 1.5, 0.5, 4.0
TRIM = {"pick": (0.0, 0.0), "drop": (0.0, 0.0), "push": (0.0, 0.0)}


def _left_plan() -> list[Waypoint]:
    plan: list[Waypoint] = []
    px, py = TRIM["pick"]
    qx, qy = TRIM["drop"]
    for name, (_, (x, y)) in CUBES.items():
        dx, dy = DROP_XY[name]
        plan += pick_place_waypoints((x + px, y + py, GRIP_Z), (dx + qx, dy + qy, PLACE_Z), travel_z=TRAVEL_Z,
                                     speed=SORT_SPEED)
    return plan


def _right_plan() -> list[Waypoint]:
    tx, ty = TRIM["push"]
    x = VALVE_XY[0] + tx
    start, end = PUSH_START_Y + ty, PUSH_END_Y + ty
    return [
        Waypoint(HOVER_S, (x, start, HOVER_Z), FINGER_CLOSED, "to hover"),
        Waypoint(PUSH_AT_S - HOVER_S - DOWN_S - SETTLE_S, (x, start, HOVER_Z), FINGER_CLOSED, "wait for the alarm"),
        Waypoint(DOWN_S, (x, start, PUSH_Z), FINGER_CLOSED, "down behind the handle"),
        Waypoint(SETTLE_S, (x, start, PUSH_Z), FINGER_CLOSED, "settle"),
        Waypoint(PUSH_S, (x, end, PUSH_Z), FINGER_CLOSED, "push the inlet closed"),
        Waypoint(0.8, (x, end, PUSH_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.5, (x, end, HOVER_Z), FINGER_CLOSED, "lift off"),
        Waypoint(1.5, (x, start, HOVER_Z), FINGER_CLOSED, "return"),
    ]


def demo():
    """Left arm: the two red cubes into the red bin, then the blue cube into the blue bin. Right arm: hover behind the
    inlet handle, and once the lamp has come on (20 s) push the handle to the closed end, then go back up."""
    return {"left": _left_plan(), "right": _right_plan()}
