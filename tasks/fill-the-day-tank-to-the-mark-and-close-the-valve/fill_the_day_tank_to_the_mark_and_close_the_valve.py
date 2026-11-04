"""On the station table, in front of the right arm: a white open-topped day tank, taped down, standing on a dark
mat. Up the tank's front face runs a strip of eight level segments that light blue from the bottom as the tank
fills; two are lit at the start. A red line crosses the strip at the top of the sixth segment.

Nearer the arm, and further to the robot's right, stands a brass transfer valve: a block with a dark handle on top
that slides along the block. Pushed towards the robot's right the handle opens the valve; pushed back towards the
robot's left it closes it. It starts closed.

Run it:  inspect-robots run --embodiment yam_arena -E task=fill_the_day_tank_to_the_mark_and_close_the_valve.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from benchkit import Slider, ToolState, on_reset, on_step, tool  # noqa: E402
from plantkit import DARK, HZ, RED, WATER_BLUE, WHITE, Clock, Lamp, Level, at_rest, pushed  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 130.0  # longer than the run limit: the valve can be reopened after the mark, so the end state decides
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Top up the day tank: open the transfer valve, watch the level, and close the valve when the level reaches the "
    "red mark. Don't let it overflow."
)

BRASS = (0.78, 0.60, 0.22)
MAT_DARK = (0.15, 0.16, 0.18)

# -- the day tank, its level strip and the red mark --------------------------------------------------------------------
TANK_XY = (0.32, -0.09)
TANK_INNER = 0.120
TANK_WALL = 0.006
TANK_H = 0.160
TANK_OUTER = TANK_INNER + 2 * TANK_WALL
MAT_SIZE = (0.160, 0.160, 0.001)
TANK_FLOOR_Z = TOP + MAT_SIZE[2]  # the tank stands on the mat
TANK_RIM_Z = TANK_FLOOR_Z + TANK_H
TANK_FRONT_X = TANK_XY[0] - TANK_OUTER / 2  # the face towards the arms, which carries the strip
TANK_RIGHT_Y = TANK_XY[1] - TANK_OUTER / 2  # the face towards the right arm's base
SEGMENTS = 8
SEG_SIZE = (0.004, 0.100, 0.016)  # thickness (x), width (y), height (z)
SEG_GAP = 0.002
SEG_PITCH = SEG_SIZE[2] + SEG_GAP  # m of strip per segment
STRIP_Z0 = TOP + 0.010  # the bottom of the lowest segment
STRIP_X = TANK_FRONT_X - SEG_SIZE[0] / 2
MARK_SEGMENT = 6  # the red mark sits on top of this segment
MARK_SIZE = (0.005, 0.100, 0.003)
MARK_Z = STRIP_Z0 + MARK_SEGMENT * SEG_PITCH - SEG_GAP + MARK_SIZE[2] / 2  # underside on the sixth segment's top
MARK_X = STRIP_X - SEG_SIZE[0] / 2 - MARK_SIZE[0] / 2  # proud of the strip

ML_PER_SEGMENT = 100.0
TANK_ML = SEGMENTS * ML_PER_SEGMENT  # 800 mL: the rim
START_ML = 200.0  # a quarter full: two segments lit
MARK_ML = MARK_SEGMENT * ML_PER_SEGMENT  # 600 mL
FILL_ML_PER_S = 50.0  # while the valve is open: one segment every 2 s


def _tank_walls() -> list[Box]:
    """Four fixed white walls and a floor, standing on the mat."""
    x, y = TANK_XY
    off = TANK_INNER / 2 + TANK_WALL / 2
    z = TANK_FLOOR_Z + TANK_H / 2
    return [
        Box("tank_floor", pos=(x, y, TANK_FLOOR_Z + 0.001), size=(TANK_INNER, TANK_INNER, 0.002), color=WHITE),
        Box("tank_wall_far", pos=(x + off, y, z), size=(TANK_WALL, TANK_OUTER, TANK_H), color=WHITE),
        Box("tank_wall_near", pos=(x - off, y, z), size=(TANK_WALL, TANK_OUTER, TANK_H), color=WHITE),
        Box("tank_wall_left", pos=(x, y + off, z), size=(TANK_INNER, TANK_WALL, TANK_H), color=WHITE),
        Box("tank_wall_right", pos=(x, y - off, z), size=(TANK_INNER, TANK_WALL, TANK_H), color=WHITE),
    ]


LEVEL = Level("tank_level", SEGMENTS)
MAT = Lamp("spill_mat", {"dry": MAT_DARK, "wet": WATER_BLUE}, "dry")

# -- the transfer valve: a block with a handle that slides along y and stays where it is pushed ---------------------------
VALVE_XY = (0.20, -0.24)
VALVE_BLOCK = (0.050, 0.050, 0.050)
VALVE_Z = TOP + VALVE_BLOCK[2] / 2
BLOCK_TOP_Z = VALVE_Z + VALVE_BLOCK[2] / 2
HANDLE = (0.030, 0.020, 0.050)
HANDLE_TRAVEL = 0.030  # m towards -y (the robot's right): the rest stop is closed, full travel is open
VALVE = tool(
    "transfer_valve",
    pos=(*VALVE_XY, VALVE_Z),
    boxes=[((0.0, 0.0, 0.0), VALVE_BLOCK, BRASS)],
    sliders=[
        # No spring: the handle stays where it is pushed, as a real valve does. The end state is what's scored.
        Slider("handle", rest=(0.0, 0.0, VALVE_BLOCK[2] / 2 + HANDLE[2] / 2), size=HANDLE, axis="Y",
               lower=-HANDLE_TRAVEL, upper=0.0, color=DARK, rate=0.0, damping=8.0, mass=0.02),
    ],
    mass=3.0,  # heavy enough to stay put when pushed; taped down on a real bench
)

OBJECTS = [
    MAT.box((*TANK_XY, TOP + MAT_SIZE[2] / 2), MAT_SIZE),
    *_tank_walls(),
    *LEVEL.boxes((STRIP_X, TANK_XY[1]), STRIP_Z0, SEG_SIZE, lit=int(START_ML / ML_PER_SEGMENT)),
    Box("full_mark", pos=(MARK_X, TANK_XY[1], MARK_Z), size=MARK_SIZE, color=RED),
    VALVE,
]

# -- what happened: the level climbs while the valve is open, and an overflow is for good ------------------------------
OPEN_AT = 0.7  # of the handle's travel: liquid flows from here
CLOSED_AT = 0.3  # back within this of the closed stop counts as closed
CLOCK = Clock()
TANK = {"ml": START_ML, "overflowed": False}


def valve_open(valve: ToolState) -> bool:
    return pushed(valve, "handle", HANDLE_TRAVEL, OPEN_AT)


def valve_closed(valve: ToolState) -> bool:
    return at_rest(valve, "handle", HANDLE_TRAVEL, CLOSED_AT)


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    TANK["ml"], TANK["overflowed"] = START_ML, False
    LEVEL.show(START_ML / TANK_ML)
    MAT.reset()


@on_step
def _fill(states) -> None:
    """Liquid runs in at 50 mL/s while the valve is open; past the rim it runs over the mat, which stays wet."""
    CLOCK.tick()
    if valve_open(states["transfer_valve"]):
        TANK["ml"] += FILL_ML_PER_S / HZ
        if TANK["ml"] > TANK_ML:
            TANK["ml"] = TANK_ML
            TANK["overflowed"] = True
            MAT.set("wet", CLOCK.step)
    LEVEL.show(TANK["ml"] / TANK_ML)


# -- scoring: the end state, read with the overflow latch -------------------------------------------------------------
MARK_BAND = (500.0, 650.0)  # mL, inclusive: at the mark
EARLY_BAND = (350.0, 500.0)  # mL, up to the mark band: stopped early
OVER_BAND = (650.0, 800.0)  # mL, above the mark band up to the rim: overshot
AT_MARK = 1.0
STOPPED_EARLY = 0.5
OVERSHOT = 0.3
LEFT_OPEN_AT_MARK = 0.3
LEFT_OPEN = 0.1
UPRIGHT_DEG = 20.0


def level_band(ml: float) -> str | None:
    """Which row of the outcome table the level falls in, or None under 350 mL."""
    if MARK_BAND[0] <= ml <= MARK_BAND[1]:
        return "mark"
    if EARLY_BAND[0] <= ml < MARK_BAND[0]:
        return "early"
    if MARK_BAND[1] < ml <= OVER_BAND[1]:
        return "over"
    return None


def grade(objects, ml: float, overflowed: bool) -> float:
    """The score as the outcome table states it."""
    valve = objects["transfer_valve"]
    band = level_band(ml)
    if overflowed or band is None or valve.tilt_deg() > UPRIGHT_DEG:
        return 0.0  # ran over, never got going, or the valve was knocked over
    if not valve_closed(valve):
        return LEFT_OPEN_AT_MARK if band == "mark" else LEFT_OPEN
    return {"mark": AT_MARK, "early": STOPPED_EARLY, "over": OVERSHOT}[band]


def success(objects) -> bool:
    return (grade(objects, TANK["ml"], TANK["overflowed"]) >= 1.0
            and objects["transfer_valve"].resting(max_speed=0.02, max_spin=1.0))


def score(objects) -> float:
    return grade(objects, TANK["ml"], TANK["overflowed"])


def distance(objects) -> float:
    """Metres still to go: until the level reaches the mark band, the handle's travel left to open, the strip height
    still to light up to the band, and the handle's travel back to closed; from then on, the handle's travel left to
    close."""
    h = min(abs(objects["transfer_valve"].sliders["handle"]), HANDLE_TRAVEL)
    short = MARK_BAND[0] - TANK["ml"]
    if short > 0:
        return (HANDLE_TRAVEL - h) + short / ML_PER_SEGMENT * SEG_PITCH + HANDLE_TRAVEL
    return h


# -- the scripted demo: the right arm pushes the handle open, waits, and pushes it closed from the other side -----------
# The closed fingers are 13 mm wide for the 3 cm above their pad tips and widen above that, so they push the handle
# 2 cm up its 5 cm height, where the handle's top still meets the narrow part.
PUSH_ABOVE_BLOCK = 0.020  # the pad tips travel this far above the block's top
PUSH_Z = BLOCK_TOP_Z + PUSH_ABOVE_BLOCK + 0.013  # TCP: the pad tips sit 13 mm below it
HOVER_Z = TOP + 0.185
FINGER_Y = 0.0085  # half the closed fingers' width along y, over the 3 cm above the pad tips (URDF collision meshes)
HOUSING_HALF_X = 0.034  # half the gripper housing's width across the fingers, 6 to 10 cm above the pad tips
WRIST_CAM_AHEAD_X = 0.070  # the wrist camera's body sits this far ahead of the TCP (+x), 7.5 cm above the pad tips
WRIST_CAM_HALF = 0.025
OVERSHOOT = 0.006  # commanded past the stop, so the handle still gets there if the arm falls a little short
BEHIND = 0.030  # the fingers come down this far from the handle's face
HANDLE_HALF_Y = HANDLE[1] / 2
CLOSED_LEFT_FACE_Y = VALVE_XY[1] + HANDLE_HALF_Y  # the handle's +y face with the valve closed
OPEN_RIGHT_FACE_Y = VALVE_XY[1] - HANDLE_TRAVEL - HANDLE_HALF_Y  # its -y face with the valve open
OPEN_START_Y = CLOSED_LEFT_FACE_Y + BEHIND
OPEN_END_Y = CLOSED_LEFT_FACE_Y - HANDLE_TRAVEL + FINGER_Y - OVERSHOOT
CLOSE_START_Y = OPEN_RIGHT_FACE_Y - BEHIND
CLOSE_END_Y = OPEN_RIGHT_FACE_Y + HANDLE_TRAVEL - FINGER_Y + OVERSHOOT
PUSH_S = 2.0  # each push: about 30 mm/s
WATCH_S = 1.3  # hover with the valve open; with the pushes' own time it passes liquid for 7.5 s (575 mL)
TRIM = {"push": (0.0, 0.0)}


def _right_plan() -> list[Waypoint]:
    tx, ty = TRIM["push"]
    x = VALVE_XY[0] + tx
    return [
        Waypoint(1.5, (x, OPEN_START_Y + ty, HOVER_Z), FINGER_CLOSED, "over the valve"),
        Waypoint(1.5, (x, OPEN_START_Y + ty, PUSH_Z), FINGER_CLOSED, "down beside the handle"),
        Waypoint(0.5, (x, OPEN_START_Y + ty, PUSH_Z), FINGER_CLOSED, "settle"),
        Waypoint(PUSH_S, (x, OPEN_END_Y + ty, PUSH_Z), FINGER_CLOSED, "push the valve open"),
        Waypoint(0.5, (x, OPEN_END_Y + ty, PUSH_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.0, (x, OPEN_END_Y + ty, HOVER_Z), FINGER_CLOSED, "lift off"),
        Waypoint(1.0, (x, CLOSE_START_Y + ty, HOVER_Z), FINGER_CLOSED, "round to the far side"),
        Waypoint(WATCH_S, (x, CLOSE_START_Y + ty, HOVER_Z), FINGER_CLOSED, "watch the level"),
        Waypoint(1.5, (x, CLOSE_START_Y + ty, PUSH_Z), FINGER_CLOSED, "down beside the handle"),
        Waypoint(0.5, (x, CLOSE_START_Y + ty, PUSH_Z), FINGER_CLOSED, "settle"),
        Waypoint(PUSH_S, (x, CLOSE_END_Y + ty, PUSH_Z), FINGER_CLOSED, "push the valve closed"),
        Waypoint(0.8, (x, CLOSE_END_Y + ty, PUSH_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.5, (x, CLOSE_END_Y + ty, HOVER_Z), FINGER_CLOSED, "lift off"),
    ]


def demo():
    """Right arm: push the handle open from its left, hover while the level climbs, push it closed from its right
    and lift away. The left arm holds its ready pose."""
    return {"right": _right_plan()}
