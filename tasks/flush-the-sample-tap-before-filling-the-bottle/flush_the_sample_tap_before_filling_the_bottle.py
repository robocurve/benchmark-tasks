"""On the station table: a steel sample tap standing on the midline, its spout turned to the robots' left, with a dark
lever handle 10 cm long sticking out towards the arms 9 cm above the table; the lever slides sideways across the pipe.
A grey waste cup stands under the spout's tip, and a small lamp at the tap's foot lights green while the tap runs. In
front of the left arm stands a clear square sample bottle with two blue lines round it and a dark handle post on its
left side; further to the left a white square mark is taped to the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=flush_the_sample_tap_before_filling_the_bottle.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, ToolState, on_reset, on_step, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, GREY, HZ, LAMP_GREEN, LAMP_OFF, STEEL, WHITE, Clock, Lamp, at_rest, gap_to, pushed, tool_points,
)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 130.0  # longer than the run limit: the tap could be opened again after the bottle is set down
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Take a water sample: open the tap and let it run to the waste cup for at least ten seconds, then hold the "
    "bottle under the tap and fill it to between the two lines, set the bottle on the mark, and close the tap."
)

CLEAR = (0.86, 0.89, 0.92)
LINE_BLUE = (0.15, 0.35, 0.80)

# -- the sample tap (tap frame: origin at the foot of the pipe, on the table, z up) ----------------------------------
TAP_XY = (0.30, 0.00)
PIPE = (0.03, 0.03, 0.20)
FLANGE = (0.08, 0.05, 0.01)
SPOUT = (0.03, 0.085, 0.03)  # 8.5 cm along the top of the pipe, from its right face to 7 cm left of its axis
SPOUT_Y = SPOUT[1] / 2 - PIPE[1] / 2  # the spout's centre: its root is flush with the pipe's right face
DRIP = (0.0, SPOUT_Y + SPOUT[1] / 2, PIPE[2] - SPOUT[2])  # the underside of the spout's tip, where the stream falls
HANDLE = (0.10, 0.02, 0.035)
HANDLE_Z = 0.09  # the handle's centre above the table
HANDLE_REST = (-(PIPE[0] / 2 + HANDLE[0] / 2), 0.0, HANDLE_Z)  # its root against the pipe's front face
LEVER_TRAVEL = 0.03  # pushed this far to the robots' right (-y) the tap runs; its rest stop is shut
OPEN_AT = 0.7  # of the travel: at or past it the tap runs
CLOSED_AT = 0.3  # back within it the tap is shut

SAMPLE_TAP = tool(
    "sample_tap",
    pos=(*TAP_XY, TOP),
    boxes=[
        ((0.0, 0.0, PIPE[2] / 2), PIPE, STEEL),
        ((0.0, 0.0, FLANGE[2] / 2), FLANGE, DARK),
        ((0.0, SPOUT_Y, PIPE[2] - SPOUT[2] / 2), SPOUT, STEEL),
    ],
    sliders=[
        # No spring: the lever stays where it is pushed, as a quarter-turn handle does. Its rest stop is shut.
        Slider("lever", rest=HANDLE_REST, size=HANDLE, axis="Y", lower=-LEVER_TRAVEL, upper=0.0, color=DARK,
               rate=0.0, damping=8.0, mass=0.02),
    ],
    mass=3.0,  # heavy enough to stay put when pushed; taped down on a real bench
    points=[("drip", DRIP)],
)
DRIP_WORLD = (TAP_XY[0] + DRIP[0], TAP_XY[1] + DRIP[1], TOP + DRIP[2])
TAP_STANDS = 20.0  # degrees: a tap leaning more has been knocked over
TAP_MOVED = 0.02  # m: a tap whose foot is further than this from its spot has been shoved

# -- the waste cup under the spout, and the flow lamp at the tap's foot ----------------------------------------------
CUP_XY = DRIP_WORLD[:2]
CUP_INNER = 0.07
CUP_WALL = 0.003
CUP_H = 0.04
RIM_Z = TOP + CUP_H
LAMP = Lamp("flow_lamp", {"off": LAMP_OFF, "green": LAMP_GREEN}, "off")
LAMP_POS = (TAP_XY[0], TAP_XY[1] - 0.05, TOP + 0.015)
LAMP_SIZE = (0.02, 0.02, 0.03)


def _collar(name: str, xy, inner: float, height: float, wall: float, color) -> list[Box]:
    """Four fixed walls round a square opening, standing on the table."""
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


# -- the sample bottle in its holder (bottle frame: origin at the bottle's centre, z up) -----------------------------
BOTTLE_XY = (0.20, 0.16)
BOTTLE_SIZE = (0.05, 0.05, 0.09)
BOTTLE_WALL = 0.003  # walls and floor
BOTTLE_MASS = 0.08  # bottle and holder together
LINE_Z = (0.05, 0.07)  # the two fill lines above the bottle's bottom
LINE_H = 0.003  # each line stands 1 mm proud of the wall
MOUTH = (0.0, 0.0, BOTTLE_SIZE[2] / 2)
_INNER = (BOTTLE_SIZE[0] - 2 * BOTTLE_WALL, BOTTLE_SIZE[1] - 2 * BOTTLE_WALL)  # the opening, 4.4 x 4.4 cm
POST = (0.03, 0.02, 0.06)  # the handle post the gripper holds
POST_Y = 0.075  # its centre, left of the bottle's centre: far enough that the gripper's body clears the spout
POST_Z = BOTTLE_SIZE[2] / 2 - POST[2] / 2  # its top level with the bottle's top
BAR_Y0 = BOTTLE_SIZE[1] / 2 - BOTTLE_WALL  # the bar starts inside the bottle's left wall ...
BAR_Y1 = POST_Y - POST[1] / 2 + 0.005  # ... and ends 5 mm into the post
BAR = (0.015, BAR_Y1 - BAR_Y0, 0.012)
BAR_Z = -0.009  # its centre: mid-height, 3.0 to 4.2 cm above the bottle's bottom
GRIP = (0.0, POST_Y, 0.02)  # the gripper's tool point when holding the post: its finger tips 2.8 cm down the post


def _lines() -> list:
    out = []
    for z in LINE_Z:
        zl = z - BOTTLE_SIZE[2] / 2
        off = BOTTLE_SIZE[0] / 2 + 0.0005
        out += [
            ((off, 0.0, zl), (0.001, BOTTLE_SIZE[1] + 0.002, LINE_H), LINE_BLUE),
            ((-off, 0.0, zl), (0.001, BOTTLE_SIZE[1] + 0.002, LINE_H), LINE_BLUE),
            ((0.0, off, zl), (BOTTLE_SIZE[0], 0.001, LINE_H), LINE_BLUE),
            ((0.0, -off, zl), (BOTTLE_SIZE[0], 0.001, LINE_H), LINE_BLUE),
        ]
    return out


BOTTLE = tool(
    "bottle",
    pos=(*BOTTLE_XY, TOP + BOTTLE_SIZE[2] / 2),
    boxes=[
        ((0.0, 0.0, -BOTTLE_SIZE[2] / 2 + BOTTLE_WALL / 2), (_INNER[0], _INNER[1], BOTTLE_WALL), CLEAR),
        ((BOTTLE_SIZE[0] / 2 - BOTTLE_WALL / 2, 0.0, 0.0), (BOTTLE_WALL, BOTTLE_SIZE[1], BOTTLE_SIZE[2]), CLEAR),
        ((-BOTTLE_SIZE[0] / 2 + BOTTLE_WALL / 2, 0.0, 0.0), (BOTTLE_WALL, BOTTLE_SIZE[1], BOTTLE_SIZE[2]), CLEAR),
        ((0.0, BOTTLE_SIZE[1] / 2 - BOTTLE_WALL / 2, 0.0), (_INNER[0], BOTTLE_WALL, BOTTLE_SIZE[2]), CLEAR),
        ((0.0, -BOTTLE_SIZE[1] / 2 + BOTTLE_WALL / 2, 0.0), (_INNER[0], BOTTLE_WALL, BOTTLE_SIZE[2]), CLEAR),
        *_lines(),
        ((0.0, (BAR_Y0 + BAR_Y1) / 2, BAR_Z), BAR, DARK),
        ((0.0, POST_Y, POST_Z), POST, DARK),
    ],
    mass=BOTTLE_MASS,
    points=[("mouth", MOUTH), ("grip", GRIP)],
)

# -- the mark where the filled bottle goes --------------------------------------------------------------------------
MARK_XY = (0.18, 0.26)
MARK = (0.07, 0.07, 0.003)

OBJECTS = [
    SAMPLE_TAP,
    *_collar("waste_cup", CUP_XY, CUP_INNER, CUP_H, CUP_WALL, GREY),
    LAMP.box(LAMP_POS, LAMP_SIZE),
    BOTTLE,
    Box("mark", pos=(*MARK_XY, TOP + MARK[2] / 2), size=MARK, color=WHITE),
]

# -- the water: a ledger in millilitres, since the simulator has no liquid -------------------------------------------
FLOW_ML_S = 15.0  # while the lever is open
FLUSH_ML = 150.0  # to waste before the first drop enters the bottle: 10 s of flow


def _ml(height: float) -> float:
    """Millilitres in the bottle when the water stands ``height`` above its bottom (the floor is 3 mm thick)."""
    return round((height - BOTTLE_WALL) * _INNER[0] * _INNER[1] * 1e6)


FILL_RANGE = (_ml(LINE_Z[0]), _ml(LINE_Z[1]))  # between the two lines: 91 to 130 mL
CAPACITY_ML = _ml(BOTTLE_SIZE[2])  # at the brim, 168 mL; past it the bottle overflows
CATCH_XY = 0.02  # the mouth within this of the drip point, seen from above, for the water to count
SPILL_TILT = 30.0  # degrees: a bottle leaning past this empties
SPILL_CAP_ML = 30.0


def over_cup(point) -> bool:
    """Seen from above, the point is inside the waste cup's opening."""
    return bool(abs(point[0] - CUP_XY[0]) <= CUP_INNER / 2 and abs(point[1] - CUP_XY[1]) <= CUP_INNER / 2)


def tap_standing(tap: ToolState) -> bool:
    """On its spot and upright: neither shoved along the bench nor knocked over."""
    return bool(tap.tilt_deg() <= TAP_STANDS and float(np.linalg.norm(tap.pos[:2] - np.array(TAP_XY))) <= TAP_MOVED)


def stream_in_mouth(tap: ToolState, bottle: ToolState) -> bool:
    """The falling stream enters the bottle's opening: where the vertical line through the drip point crosses the
    plane of the mouth (tilted with the bottle) it is inside the 4.4 cm opening, with the mouth below the drip point,
    above the cup's rim, and the bottle not lying down. This is what wets the bottle, however badly it is held."""
    drip, mouth = tap.point("drip"), bottle.point("mouth")
    if not (RIM_Z < mouth[2] < drip[2]) or bottle.tilt_deg() >= 90.0:
        return False
    to_bottle = bottle.rotation().T
    p = to_bottle @ (drip - mouth)  # the drip point in the bottle's frame, from the mouth's centre
    d = to_bottle @ np.array([0.0, 0.0, -1.0])  # the stream's direction in that frame; d[2] is -cos(tilt), not 0
    crossing = p - (p[2] / d[2]) * d  # where the stream meets the mouth's plane, local z = 0
    return bool(abs(crossing[0]) <= _INNER[0] / 2 and abs(crossing[1]) <= _INNER[1] / 2)


def catching(tap: ToolState, bottle: ToolState) -> bool:
    """The stream is credited to the bottle: it enters the opening, the mouth is within 2 cm of the drip point seen
    from above, the bottle leans under 30 degrees, and the tap stands on its spot with its spout over the cup."""
    drip, mouth = tap.point("drip"), bottle.point("mouth")
    return bool(stream_in_mouth(tap, bottle) and np.linalg.norm(mouth[:2] - drip[:2]) <= CATCH_XY
                and bottle.tilt_deg() <= SPILL_TILT and over_cup(drip) and tap_standing(tap))


class Ledger:
    """Millilitres in the waste cup, in the bottle and on the bench, and whether the line was flushed before the bottle
    took its first drop."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.waste = 0.0
        self.bottle = 0.0
        self.spilled = 0.0
        self.waste_at_first_drop: float | None = None  # mL already run to waste when the stream first wet the bottle
        self.open = False

    @property
    def flushed(self) -> bool:
        before = self.waste if self.waste_at_first_drop is None else self.waste_at_first_drop
        return before >= FLUSH_ML

    def _lose(self, ml: float, mouth) -> None:
        """Water leaving the bottle lands in the cup when its mouth is over the opening, otherwise on the bench."""
        if over_cup(mouth):
            self.waste += ml
        else:
            self.spilled += ml

    def update(self, tap: ToolState, bottle: ToolState) -> None:
        """One control step."""
        mouth = bottle.point("mouth")
        if self.bottle > 0.0 and bottle.tilt_deg() > SPILL_TILT:
            self._lose(self.bottle, mouth)
            self.bottle = 0.0
        self.open = pushed(tap, "lever", LEVER_TRAVEL, OPEN_AT)
        if not self.open:
            return
        ml = FLOW_ML_S / HZ
        if self.waste_at_first_drop is None and stream_in_mouth(tap, bottle):
            self.waste_at_first_drop = self.waste
        if catching(tap, bottle):
            self.bottle += ml
            if self.bottle > CAPACITY_ML:
                self._lose(self.bottle - CAPACITY_ML, mouth)
                self.bottle = CAPACITY_ML
        elif over_cup(tap.point("drip")) and tap_standing(tap):
            self.waste += ml
        else:
            self.spilled += ml


LEDGER = Ledger()
CLOCK = Clock()


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    LEDGER.reset()
    LAMP.reset()


@on_step
def _track(states) -> None:
    CLOCK.tick()
    LEDGER.update(states["sample_tap"], states["bottle"])
    LAMP.set("green" if LEDGER.open else "off", CLOCK.step)


# -- scoring -------------------------------------------------------------------------------------------------------
ON_MARK_WITHIN = 0.03  # m, the bottle's centre from the mark's centre
UPRIGHT_DEG = 15.0
STANDING_BELOW = TOP + 0.02  # the bottle's bottom at most this high: standing on the mark, not on something else
HELD_WITHIN = 0.01  # m: a gripper's tool point this close to the bottle or its holder is still holding it
FULL = 1.0
UNFINISHED = 0.7  # flushed and filled, tap shut, but the bottle not set on the mark
FLUSHED_ONLY = 0.3  # flushed and shut, the bottle not filled (or overfilled, which earns no fill credit)
TAP_OPEN_CAP = 0.25  # flushed and filled but the tap left running; anything less with the tap running is 0
SPILL_CAP = 0.25  # more than 30 mL on the bench
NO_FLUSH = 0.2  # filled, set down and shut without the flush


def tap_closed(tap: ToolState) -> bool:
    return at_rest(tap, "lever", LEVER_TRAVEL, CLOSED_AT)


def filled(ledger: Ledger) -> bool:
    return FILL_RANGE[0] <= ledger.bottle <= FILL_RANGE[1]


def overfilled(ledger: Ledger) -> bool:
    return ledger.bottle > FILL_RANGE[1]


def on_mark(bottle: ToolState) -> bool:
    """Standing upright on the mark."""
    return bool(np.linalg.norm(bottle.pos[:2] - np.array(MARK_XY)) <= ON_MARK_WITHIN
                and bottle.low[2] < STANDING_BELOW and bottle.tilt_deg() < UPRIGHT_DEG)


def held(bottle: ToolState) -> bool:
    """A gripper's tool point is at the bottle or its holder. Empty outside the simulator, so never held there."""
    return any(gap_to(p, bottle) <= HELD_WITHIN for p in tool_points())


def grade(ledger: Ledger, tap: ToolState, bottle: ToolState, holding: bool) -> float:
    """The score as the outcome table states it."""
    if not tap_standing(tap):
        return 0.0
    flushed, shut, good = ledger.flushed, tap_closed(tap), filled(ledger)
    placed = on_mark(bottle) and not holding
    if flushed and good and placed:
        score = FULL
    elif flushed and good:
        score = UNFINISHED
    elif flushed:
        score = FLUSHED_ONLY
    elif good and placed:
        score = NO_FLUSH
    else:
        score = 0.0
    if not shut:
        score = min(score, TAP_OPEN_CAP) if flushed and good else 0.0
    if overfilled(ledger):
        score = min(score, FLUSHED_ONLY)
    if ledger.spilled > SPILL_CAP_ML:
        score = min(score, SPILL_CAP)
    return round(score, 6)


def success(objects) -> bool:
    bottle = objects["bottle"]
    return grade(LEDGER, objects["sample_tap"], bottle, held(bottle)) >= 1.0 and bottle.resting()


def score(objects) -> float:
    bottle = objects["bottle"]
    return grade(LEDGER, objects["sample_tap"], bottle, held(bottle))


def distance(objects) -> float:
    """Metres from the bottle's mouth to the drip point until the bottle is filled, then from the bottle to the mark."""
    tap, bottle = objects["sample_tap"], objects["bottle"]
    if LEDGER.bottle < FILL_RANGE[0]:
        return float(np.linalg.norm(bottle.point("mouth") - tap.point("drip")))
    return float(np.linalg.norm(bottle.pos[:2] - np.array(MARK_XY)))


# -- the scripted demo: left arm opens the tap, flushes, fills and sets the bottle down; right arm shuts the tap -----
# The lever is pushed with the gripper turned 90 degrees, so the wrist camera (7 cm off the tool point) swings to the
# side instead of into the pipe, and the pads' long side leads the push. Each arm's wrist turns one way only.
PUSH_YAW = {"left": -math.pi / 2, "right": math.pi / 2}
FINGER_PUSH = 0.020  # the pads' half-length along the push, with the gripper turned
OVERSHOOT = 0.006  # commanded past the stop, so the lever still reaches it if the arm falls a little short
HOVER_Z = TOP + 0.165  # carrying height: the bottle's bottom 10 cm up, 6 cm over the cup's rim, never over the lever
PUSH_X = 0.21  # 2.5 cm in from the handle's tip (the pads 1.5 cm): the gripper's body stays 2.4 cm short of the pipe
PUSH_Z = TOP + HANDLE_Z - HANDLE[2] / 2 + 0.005 + 0.013  # finger tips 5 mm above the handle's bottom edge
HANDLE_SHUT_Y = TAP_XY[1] + HANDLE[1] / 2  # the handle's left face when shut
HANDLE_OPEN_Y = HANDLE_SHUT_Y - LEVER_TRAVEL
LEVER_WAY_Y = 0.16  # the left arm turns here and comes in over the lever along x = PUSH_X, well clear of the spout
OPEN_START_Y = HANDLE_SHUT_Y + FINGER_PUSH + 0.025  # the pads 2.5 cm behind the shut handle
OPEN_END_Y = HANDLE_OPEN_Y + FINGER_PUSH - OVERSHOOT  # the tool point 6 mm right of the midline at the open stop
SHUT_START_Y = TAP_XY[1] - HANDLE[1] / 2 - LEVER_TRAVEL - FINGER_PUSH - 0.025  # 2.5 cm beyond the open handle
SHUT_END_Y = TAP_XY[1] - HANDLE[1] / 2 - FINGER_PUSH + OVERSHOOT
GRIP_OPEN = -0.020  # finger target: pads 4 cm apart, 1 cm either side of the post and 2 cm clear of the bottle's rim
GRIP_Z = TOP + BOTTLE_SIZE[2] / 2 + GRIP[2]  # the tool point on the post while the bottle stands on the table
PLACE_Z = GRIP_Z + 0.005
POUR_Z = RIM_Z + 0.02 + BOTTLE_SIZE[2] / 2 + GRIP[2]  # bottle bottom 2 cm above the rim, mouth 2 cm under the spout
POUR_TCP = (CUP_XY[0], CUP_XY[1] + GRIP[1])  # the tool point when the mouth is under the spout's tip
APPROACH_Y = CUP_XY[1] + BOTTLE_SIZE[1] / 2 + 0.02 + GRIP[1]  # the bottle's right face 2 cm past the spout's tip
FILL_HOLD_S = 6.0  # with the slide in and out under the stream, about 7.4 s of flow: 110 mL, between the lines
RIGHT_WAIT = (PUSH_X, -0.20, TOP + 0.15)  # clear of the left arm's camera, which swings right of the midline
TRIM = {"lever": (0.0, 0.0), "bottle": (0.0, 0.0), "pour": (0.0, 0.0), "mark": (0.0, 0.0)}


def _left_plan() -> list[Waypoint]:
    yaw = PUSH_YAW["left"]
    lx = PUSH_X + TRIM["lever"][0]
    ty = TRIM["lever"][1]
    bx, by = BOTTLE_XY[0] + TRIM["bottle"][0], BOTTLE_XY[1] + GRIP[1] + TRIM["bottle"][1]
    px, py = POUR_TCP[0] + TRIM["pour"][0], POUR_TCP[1] + TRIM["pour"][1]
    ay = APPROACH_Y + TRIM["pour"][1]
    mx, my = MARK_XY[0] + TRIM["mark"][0], MARK_XY[1] + GRIP[1] + TRIM["mark"][1]
    return [
        Waypoint(2.0, (lx, LEVER_WAY_Y, HOVER_Z), FINGER_CLOSED, "in from the left, turned", yaw),
        Waypoint(1.0, (lx, OPEN_START_Y + ty, HOVER_Z), FINGER_CLOSED, "over the lever", yaw),
        Waypoint(1.5, (lx, OPEN_START_Y + ty, PUSH_Z), FINGER_CLOSED, "down beside the lever", yaw),
        Waypoint(0.5, (lx, OPEN_START_Y + ty, PUSH_Z), FINGER_CLOSED, "settle", yaw),
        Waypoint(2.0, (lx, OPEN_END_Y + ty, PUSH_Z), FINGER_CLOSED, "push the lever open", yaw),
        Waypoint(0.5, (lx, OPEN_END_Y + ty, PUSH_Z), FINGER_CLOSED, "hold", yaw),
        Waypoint(1.5, (lx, OPEN_END_Y + ty, HOVER_Z), FINGER_CLOSED, "lift off", yaw),
        Waypoint(2.0, (bx, by, HOVER_Z), GRIP_OPEN, "over the bottle's handle", yaw),
        Waypoint(1.0, (bx, by, HOVER_Z), GRIP_OPEN, "turn back, let the tap run"),
        Waypoint(1.5, (bx, by, GRIP_Z), GRIP_OPEN, "down to the handle"),
        Waypoint(0.8, (bx, by, GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (bx, by, HOVER_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (px, ay, HOVER_Z), FINGER_CLOSED, "carry to beside the spout"),
        Waypoint(1.5, (px, ay, POUR_Z), FINGER_CLOSED, "lower beside the spout"),
        Waypoint(1.5, (px, py, POUR_Z), FINGER_CLOSED, "slide the mouth under the spout"),
        Waypoint(FILL_HOLD_S, (px, py, POUR_Z), FINGER_CLOSED, "fill"),
        Waypoint(1.5, (px, ay, POUR_Z), FINGER_CLOSED, "slide out"),
        Waypoint(1.5, (px, ay, HOVER_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (mx, my, HOVER_Z), FINGER_CLOSED, "carry to the mark"),
        Waypoint(1.5, (mx, my, PLACE_Z), FINGER_CLOSED, "set down"),
        Waypoint(0.6, (mx, my, PLACE_Z), GRIP_OPEN, "release"),
        Waypoint(1.0, (mx, my, HOVER_Z), GRIP_OPEN, "retreat"),
    ]


def _right_plan(start_after: float) -> list[Waypoint]:
    yaw = PUSH_YAW["right"]
    x = PUSH_X + TRIM["lever"][0]
    ty = TRIM["lever"][1]
    return [
        Waypoint(start_after, RIGHT_WAIT, FINGER_CLOSED, "wait for the bottle to be set down", yaw),
        Waypoint(1.5, (x, SHUT_START_Y + ty, HOVER_Z), FINGER_CLOSED, "over the lever", yaw),
        Waypoint(1.5, (x, SHUT_START_Y + ty, PUSH_Z), FINGER_CLOSED, "down beside the lever", yaw),
        Waypoint(0.5, (x, SHUT_START_Y + ty, PUSH_Z), FINGER_CLOSED, "settle", yaw),
        Waypoint(2.0, (x, SHUT_END_Y + ty, PUSH_Z), FINGER_CLOSED, "push the lever shut", yaw),
        Waypoint(0.5, (x, SHUT_END_Y + ty, PUSH_Z), FINGER_CLOSED, "hold", yaw),
        Waypoint(1.5, (x, SHUT_END_Y + ty, HOVER_Z), FINGER_CLOSED, "lift off", yaw),
        Waypoint(1.5, RIGHT_WAIT, FINGER_CLOSED, "return", yaw),
    ]


def demo():
    """Left arm: push the lever open, let the tap run, take the bottle by its handle post, slide its mouth under the
    spout, fill it, set it on the mark and let go. Right arm: once the bottle is down, push the lever shut."""
    left = _left_plan()
    return {"left": left, "right": _right_plan(sum(w.duration for w in left))}


assert FINGER_OPEN
