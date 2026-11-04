"""On the station table: a closed light-grey tank, taped down on the midline, with two dark valve handles that slide
sideways 3 cm and stay where they are put. The vent valve's handle stands on the top plate, on the robots' left of
centre, and opens when pushed toward the midline. The drain valve's handle stands on a dark valve block that steps out
from the foot of the face toward the arms, on the robots' right of centre, and also opens when pushed toward the
midline. Both start shut. A grey spout leads out of the block's front over a grey sump collar on the table. Up the left
of the same face runs the level gauge, a column of eight segments; all eight are lit blue.

Run it:  inspect-robots run --embodiment yam_arena -E task=open_the_vent_before_draining_the_tank.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from benchkit import Slider, ToolState, on_reset, on_step, recolor, tool  # noqa: E402
from plantkit import DARK, GREY, HZ, Clock, Level, distance_xy, pushed  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Drain the tank into the sump: open the vent valve on top first, then open the drain valve at the bottom, and let "
    "it run until the level reads empty."
)

TANK_GREY = (0.80, 0.81, 0.83)
HANDLE_DARK = (0.12, 0.13, 0.15)
COLLAPSED_RED = (0.45, 0.05, 0.05)

# -- the tank, in its own frame: origin at the body's centre, x away from the arms, y to the robots' left ------------
TANK_XY = (0.27, 0.00)
TANK_SIZE = (0.12, 0.12, 0.10)
TANK_H = TANK_SIZE[2]
FRONT_X = TANK_XY[0] - TANK_SIZE[0] / 2  # the face toward the arms (world x)
HANDLE = (0.020, 0.030, 0.040)  # each valve handle: x, y, z
TRAVEL = 0.030  # a handle slides this far from shut to open
OPEN_AT = 0.7  # of its travel: a valve counts as open from here, shut below it

VENT_Y = 0.030  # the vent handle's centre at rest (shut); it slides -y, toward the midline, to open
VENT_REST = (0.0, VENT_Y, TANK_H / 2 + HANDLE[2] / 2)

# The drain valve block steps out from the foot of the front face. A handle flush with the face can't be pushed by a
# straight-down gripper (the finger bodies are 7 cm across and sit 4 cm above the TCP), so the handle stands on the
# block's front edge, 3.5 cm proud of the face, high enough that the finger bodies pass over the tank's top.
BLOCK = (0.045, 0.085, 0.050)
BLOCK_X = -TANK_SIZE[0] / 2 - BLOCK[0] / 2
BLOCK_Y = -0.0175  # from y -0.06 to y 0.025: under the drain handle's whole travel and the spout
BLOCK_Z = -TANK_H / 2 + BLOCK[2] / 2
BLOCK_TOP = BLOCK[2]  # above the table
DRAIN_X = BLOCK_X - BLOCK[0] / 2 + HANDLE[0] / 2  # standing at the block's front edge
DRAIN_Y = -0.030  # at rest (shut); it slides +y, toward the midline, to open
DRAIN_REST = (DRAIN_X, DRAIN_Y, -TANK_H / 2 + BLOCK[2] + HANDLE[2] / 2)

SPOUT = (0.040, 0.020, 0.012)
SPOUT_Y = 0.015  # beside the open drain handle, not under it
SPOUT_BOTTOM = 0.035  # above the table: 5 mm over the sump's walls
SPOUT_REST = (BLOCK_X - BLOCK[0] / 2 - SPOUT[0] / 2, SPOUT_Y, -TANK_H / 2 + SPOUT_BOTTOM + SPOUT[2] / 2)

TANK = tool(
    "tank",
    pos=(*TANK_XY, TOP + TANK_H / 2),
    boxes=[
        ((0.0, 0.0, 0.0), TANK_SIZE, TANK_GREY),
        ((BLOCK_X, BLOCK_Y, BLOCK_Z), BLOCK, DARK),
        (SPOUT_REST, SPOUT, GREY),
    ],
    sliders=[
        # No springs: a valve handle stays where it is pushed. The step hook reads both every control step.
        Slider("vent", rest=VENT_REST, size=HANDLE, axis="Y", lower=-TRAVEL, upper=0.0, color=HANDLE_DARK, rate=0.0,
               damping=8.0, mass=0.02),
        Slider("drain", rest=DRAIN_REST, size=HANDLE, axis="Y", lower=0.0, upper=TRAVEL, color=HANDLE_DARK, rate=0.0,
               damping=8.0, mass=0.02),
    ],
    mass=3.0,  # heavy enough to stay put when pushed; taped down on a real bench
)

# -- the sump and the level gauge (fixed boxes: the gauge's segments are recoloured by name) -----------------------
SUMP_XY = (0.105, SPOUT_Y)  # its far wall stops 2 cm short of the valve block; the spout's mouth overhangs it
SUMP_INNER = 0.070
SUMP_WALL = 0.003
SUMP_H = 0.030

SEGMENTS = 8
SEG = (0.004, 0.030, 0.009)  # one gauge segment: x, y, z
SEG_GAP = 0.002
GAUGE_H = SEGMENTS * SEG[2] + (SEGMENTS - 1) * SEG_GAP
GAUGE_Y = 0.045  # up the left of the front face, beside the valve block
GAUGE_X = FRONT_X - 0.0015 - SEG[0] / 2  # 1.5 mm in front of the face: the loose tank never touches a fixed box
GAUGE_Z0 = TOP + 0.008
LEVEL = Level("tank_level", SEGMENTS)


def _collar(name: str, xy, inner: float, height: float, wall: float, color) -> list[Box]:
    """Four fixed walls around a square opening, standing on the table."""
    x, y = xy
    off = inner / 2 + wall / 2
    z = TOP + height / 2
    long = inner + 2 * wall
    return [
        Box(f"{name}_f", pos=(x + off, y, z), size=(wall, long, height), color=color),
        Box(f"{name}_b", pos=(x - off, y, z), size=(wall, long, height), color=color),
        Box(f"{name}_l", pos=(x, y + off, z), size=(inner, wall, height), color=color),
        Box(f"{name}_r", pos=(x, y - off, z), size=(inner, wall, height), color=color),
    ]


OBJECTS = [
    *_collar("sump", SUMP_XY, SUMP_INNER, SUMP_H, SUMP_WALL, GREY),
    *LEVEL.boxes((GAUGE_X, GAUGE_Y), GAUGE_Z0, SEG, lit=SEGMENTS, gap=SEG_GAP),
    TANK,
]

# -- what happens over time: the collapse latches, the level falls while both valves are open ---------------------
SEGMENT_S = 1.5  # one segment of level per this many seconds with both valves open
SEGMENT_STEPS = round(SEGMENT_S * HZ)
DRAIN_S = SEGMENTS * SEGMENT_S  # 12 s from full to empty
DRAIN_STEPS = SEGMENTS * SEGMENT_STEPS


def vent_open(tank: ToolState) -> bool:
    return pushed(tank, "vent", TRAVEL, OPEN_AT)


def drain_open(tank: ToolState) -> bool:
    return pushed(tank, "drain", TRAVEL, OPEN_AT)


class Trial:
    """What the step hook tracks: the clock, whether the tank has collapsed, and how long it has drained."""

    def __init__(self) -> None:
        self.clock = Clock()
        self.collapsed = False
        self.drained_steps = 0
        self.painted = False  # the tank was painted red for a collapse: paint it grey again at the next reset

    def reset(self) -> None:
        self.clock.reset()
        self.collapsed = False
        self.drained_steps = 0
        if self.painted:
            self.painted = False
            recolor("tank", TANK_GREY)

    def collapse(self) -> None:
        if not self.collapsed:
            self.collapsed = True
            self.painted = True
            recolor("tank", COLLAPSED_RED)

    @property
    def drained(self) -> float:
        """Seconds both valves have been open, with the tank intact."""
        return self.drained_steps / HZ

    @property
    def empty(self) -> bool:
        return self.drained_steps >= DRAIN_STEPS

    def segments_lit(self) -> int:
        """One segment goes dark at each 1.5 s of draining; the last at 12 s."""
        return math.ceil((DRAIN_STEPS - self.drained_steps) / SEGMENT_STEPS)


TRIAL = Trial()


@on_reset
def _reset(states) -> None:
    TRIAL.reset()
    LEVEL.show(1.0)


@on_step
def _watch(states) -> None:
    tank = states["tank"]
    TRIAL.clock.tick()
    vent, drain = vent_open(tank), drain_open(tank)
    if drain and not vent:
        TRIAL.collapse()  # draining a shut tank pulls a vacuum: permanent, whatever happens next
    elif vent and drain and not TRIAL.collapsed and not TRIAL.empty:
        TRIAL.drained_steps += 1
    LEVEL.show(TRIAL.segments_lit() / SEGMENTS)


# -- scoring: the end state, with the collapse and the draining tracked above --------------------------------------
DRAINING = 0.5
VENTED = 0.2
MOVED_M = 0.030
TIPPED_DEG = 20.0


def tank_in_place(tank: ToolState) -> bool:
    """Still where it was taped down, and standing."""
    return distance_xy(tank, TANK_XY) <= MOVED_M and tank.tilt_deg() <= TIPPED_DEG


def grade(objects, trial: Trial) -> float:
    """The score as the outcome table states it."""
    tank = objects["tank"]
    if trial.collapsed or not tank_in_place(tank):
        return 0.0
    if trial.empty:
        return 1.0
    if trial.drained_steps > 0 or (vent_open(tank) and drain_open(tank)):
        return DRAINING
    if vent_open(tank):
        return VENTED
    return 0.0


def success(objects) -> bool:
    return grade(objects, TRIAL) >= 1.0 and objects["tank"].resting(max_speed=0.02, max_spin=1.0)


def score(objects) -> float:
    return grade(objects, TRIAL)


def distance(objects) -> float:
    """Metres of valve travel still to push, plus the height of gauge still lit."""
    tank = objects["tank"]
    left = sum(TRAVEL - min(abs(tank.sliders[name]), TRAVEL) for name in ("vent", "drain"))
    return float(left + GAUGE_H * TRIAL.segments_lit() / SEGMENTS)


# -- the scripted demo: left arm opens the vent; the right arm opens the drain once it is, then waits for the level ----
FINGER_Y = 0.0065  # half the closed fingers' width along y, near the tips
OVERSHOOT = 0.006  # commanded past the stop, so a handle still opens if the arm falls a little short
BEHIND = 0.030  # the fingers come down this far behind a handle's face
VENT_PUSH_Z = TOP + TANK_H + 0.005 + 0.013  # finger tips 5 mm above the top plate
DRAIN_PUSH_Z = TOP + BLOCK_TOP + 0.005 + 0.013  # finger tips 5 mm above the valve block
LEFT_HOVER_Z = TOP + 0.185
RIGHT_HOVER_Z = TOP + 0.150
VENT_X = TANK_XY[0]
VENT_FACE_SHUT = TANK_XY[1] + VENT_Y + HANDLE[1] / 2  # the face the left arm pushes on, at rest
VENT_FACE_OPEN = VENT_FACE_SHUT - TRAVEL
VENT_START_Y = VENT_FACE_SHUT + BEHIND
VENT_END_Y = VENT_FACE_OPEN + FINGER_Y - OVERSHOOT
DRAIN_WORLD_X = TANK_XY[0] + DRAIN_X
DRAIN_FACE_SHUT = TANK_XY[1] + DRAIN_Y - HANDLE[1] / 2  # the face the right arm pushes on, at rest
DRAIN_FACE_OPEN = DRAIN_FACE_SHUT + TRAVEL
DRAIN_START_Y = DRAIN_FACE_SHUT - BEHIND
DRAIN_END_Y = DRAIN_FACE_OPEN - FINGER_Y + OVERSHOOT
LEFT_ASIDE = (0.22, 0.16)  # out of the top camera's view of the tank while it drains
RIGHT_ASIDE = (0.22, -0.16)
RIGHT_STARTS_AFTER = 6.0  # the vent reads open at about 5.7 s
DRAIN_WAIT = 14.0  # longer than the 12 s drain plus the success hold
TRIM = {"vent": (0.0, 0.0), "drain": (0.0, 0.0)}


def _left_plan() -> list[Waypoint]:
    tx, ty = TRIM["vent"]
    x, start, end = VENT_X + tx, VENT_START_Y + ty, VENT_END_Y + ty
    return [
        Waypoint(1.5, (x, start, LEFT_HOVER_Z), FINGER_CLOSED, "over the vent handle"),
        Waypoint(1.5, (x, start, VENT_PUSH_Z), FINGER_CLOSED, "down behind the handle"),
        Waypoint(0.5, (x, start, VENT_PUSH_Z), FINGER_CLOSED, "settle"),
        Waypoint(3.0, (x, end, VENT_PUSH_Z), FINGER_CLOSED, "push the vent open"),
        Waypoint(0.8, (x, end, VENT_PUSH_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.5, (x, end, LEFT_HOVER_Z), FINGER_CLOSED, "lift off"),
        Waypoint(2.0, (*LEFT_ASIDE, LEFT_HOVER_Z), FINGER_CLOSED, "clear of the tank"),
    ]


def _right_plan(start_after: float) -> list[Waypoint]:
    tx, ty = TRIM["drain"]
    x, start, end = DRAIN_WORLD_X + tx, DRAIN_START_Y + ty, DRAIN_END_Y + ty
    return [
        Waypoint(start_after, (x, start, RIGHT_HOVER_Z), FINGER_CLOSED, "wait for the vent"),
        Waypoint(1.5, (x, start, DRAIN_PUSH_Z), FINGER_CLOSED, "down behind the handle"),
        Waypoint(0.5, (x, start, DRAIN_PUSH_Z), FINGER_CLOSED, "settle"),
        Waypoint(3.0, (x, end, DRAIN_PUSH_Z), FINGER_CLOSED, "push the drain open"),
        Waypoint(0.8, (x, end, DRAIN_PUSH_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.5, (x, end, RIGHT_HOVER_Z), FINGER_CLOSED, "lift off"),
        Waypoint(2.0, (*RIGHT_ASIDE, RIGHT_HOVER_Z), FINGER_CLOSED, "clear of the sump"),
        Waypoint(DRAIN_WAIT, (*RIGHT_ASIDE, RIGHT_HOVER_Z), FINGER_CLOSED, "let the tank drain"),
    ]


def demo():
    """Left arm: push the vent handle open and move aside. Right arm: wait until the vent is open, push the drain
    handle open, move aside, and wait for the level to read empty."""
    return {"left": _left_plan(), "right": _right_plan(RIGHT_STARTS_AFTER)}
