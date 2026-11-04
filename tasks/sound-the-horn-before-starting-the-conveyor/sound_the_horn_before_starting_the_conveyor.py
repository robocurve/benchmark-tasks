"""On the station table: two push buttons on dark bases, one in front of each arm. The button in front of the left arm
has a yellow cap and works the conveyor's horn; the button in front of the right arm has a green cap and starts the
conveyor. A lamp stands on a short dark post beyond each button, out to that arm's side. The horn lamp, beyond the
yellow button, is grey and lights amber while the yellow cap is held in. The run lamp, beyond the green button, is grey
now and lights green while the conveyor runs. A short dark conveyor with a grey tote on it lies on the midline at the
back of the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=sound_the_horn_before_starting_the_conveyor.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from benchkit import Slider, ToolState, on_reset, on_step, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, GREEN, GREY, HZ, LAMP_AMBER, LAMP_GREEN, LAMP_OFF, LAMP_RED, YELLOW, Clock, Lamp, at_rest, pushed,
)
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Start the conveyor: press and hold the yellow horn button so the warning sounds for at least three seconds, then "
    "press the green start button."
)

BELT_BLACK = (0.16, 0.17, 0.19)

# -- the two push buttons, one in front of each arm: 32 cm apart, so each arm presses its own --------------------------
BUTTON_X = 0.22
BUTTON_Y = 0.16  # each side of the midline: the horn on the left arm's side, start on the right arm's
BUTTON_BASE = (0.07, 0.07, 0.030)
BUTTON_CAP = (0.040, 0.040, 0.015)
BUTTON_TRAVEL = 0.008
BUTTON_Z = TOP + BUTTON_BASE[2] / 2
CAP_REST = (0.0, 0.0, BUTTON_BASE[2] / 2 + BUTTON_CAP[2] / 2)  # the cap's centre at rest (tool frame)
CAP_TOP_Z = TOP + BUTTON_BASE[2] + BUTTON_CAP[2]  # the cap's top at rest (world)


def _button(name: str, y: float, cap_colour):
    return tool(
        name,
        pos=(BUTTON_X, y, BUTTON_Z),
        boxes=[((0.0, 0.0, 0.0), BUTTON_BASE, DARK)],
        sliders=[Slider("cap", rest=CAP_REST, size=BUTTON_CAP, axis="Z", lower=-BUTTON_TRAVEL, upper=0.0,
                        color=cap_colour, rate=300.0, damping=8.0, mass=0.01)],
        mass=3.0,  # heavy enough to stay put when pressed; taped down on a real bench
    )


HORN_BUTTON = _button("horn_button", BUTTON_Y, YELLOW)
START_BUTTON = _button("start_button", -BUTTON_Y, GREEN)
BUTTONS = ("horn_button", "start_button")

# -- the lamps, each on a short post beyond its button and out to its arm's side ---------------------------------------
# The wrist camera rides about 7 cm forward of the TCP and 6 cm or more above the pad tips. Pressing a cap, it passes
# over the near edge of the lamp beyond it, so post and lamp stand only 6 cm tall: 3.5 cm under the camera.
LAMP_X = 0.32
LAMP_Y = 0.22  # each side of the midline
POST = (0.030, 0.030, 0.030)
LAMP_SIZE = (0.050, 0.050, 0.030)
LAMP_Z = TOP + POST[2] + LAMP_SIZE[2] / 2
LAMP_TOP_Z = TOP + POST[2] + LAMP_SIZE[2]
HORN_LAMP = Lamp("horn_lamp", {"off": LAMP_OFF, "sounding": LAMP_AMBER}, "off")
RUN_LAMP = Lamp("run_lamp", {"off": LAMP_OFF, "run": LAMP_GREEN, "fault": LAMP_RED}, "off")

# -- the conveyor: a fixed belt with a tote on it, there to be seen; nothing on it moves -------------------------------
CONVEYOR_XY = (0.36, 0.0)
BELT = (0.20, 0.06, 0.040)  # x 26 to 46 cm on the midline, 4 cm tall
TOTE = (0.050, 0.050, 0.040)
TOTE_X = 0.34

OBJECTS = [
    HORN_BUTTON,
    START_BUTTON,
    Box("horn_post", pos=(LAMP_X, LAMP_Y, TOP + POST[2] / 2), size=POST, color=DARK),
    HORN_LAMP.box((LAMP_X, LAMP_Y, LAMP_Z), LAMP_SIZE),
    Box("run_post", pos=(LAMP_X, -LAMP_Y, TOP + POST[2] / 2), size=POST, color=DARK),
    RUN_LAMP.box((LAMP_X, -LAMP_Y, LAMP_Z), LAMP_SIZE),
    Box("conveyor_belt", pos=(*CONVEYOR_XY, TOP + BELT[2] / 2), size=BELT, color=BELT_BLACK),
    Box("tote", pos=(TOTE_X, CONVEYOR_XY[1], TOP + BELT[2] + TOTE[2] / 2), size=TOTE, color=GREY),
]

# -- the conveyor's starter, watched every step ------------------------------------------------------------------------
PUSHED_AT = 0.7  # of the cap's travel: 5.6 mm in sounds the horn, or presses start
RELEASE_AT = 0.3  # the start cap back out this far before its next press counts
PRESS_DEPTH = PUSHED_AT * BUTTON_TRAVEL
HORN_S = 3.0  # the horn held in this long, without a break, is a warning
WINDOW_S = 10.0  # a warning that ended no longer ago than this still covers a start press
HORN_STEPS = int(round(HORN_S * HZ))
WINDOW_STEPS = int(round(WINDOW_S * HZ))
CLOCK = Clock()


class Starter:
    """The conveyor's starter. ``held`` counts consecutive steps with the horn cap in; any step with it out ends the
    hold. ``warned_at`` is the step the latest hold of HORN_S or more ended. The first start press runs the conveyor if
    the horn is still held in after HORN_S, or a hold that long ended no more than WINDOW_S before; otherwise it is a
    fault, for the rest of the trial. Once the conveyor runs or has faulted, later presses change nothing."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.state = "idle"  # idle, running or fault
        self.held = 0  # steps the horn cap has been in, this hold
        self.longest = 0  # the longest hold so far (steps), the one in progress included
        self.warned_at = None  # the step the latest hold of HORN_STEPS or more ended
        self.pressed = False  # the start cap is in (with hysteresis)
        self.presses = 0
        self.started_at = None  # the step the conveyor started, or faulted
        self.fault = ""

    def warned(self, step: int) -> bool:
        """The horn has sounded long enough, and recently: held in for HORN_S and still in, or a hold that long ended
        no more than WINDOW_S ago."""
        return self.held >= HORN_STEPS or (self.warned_at is not None and step - self.warned_at <= WINDOW_STEPS)

    def update(self, horn: ToolState, start: ToolState, step: int) -> None:
        if pushed(horn, "cap", BUTTON_TRAVEL, PUSHED_AT):
            self.held += 1
            self.longest = max(self.longest, self.held)
            HORN_LAMP.set("sounding", step)
        else:
            if self.held >= HORN_STEPS:
                self.warned_at = step
            self.held = 0
            HORN_LAMP.set("off", step)
        down = pushed(start, "cap", BUTTON_TRAVEL, PUSHED_AT)
        edge = down and not self.pressed
        if down:
            self.pressed = True
        elif at_rest(start, "cap", BUTTON_TRAVEL, RELEASE_AT):
            self.pressed = False
        if not edge:
            return
        self.presses += 1
        if self.state != "idle":
            return
        self.started_at = step
        if self.warned(step):
            self.state = "running"
            RUN_LAMP.set("run", step)
        else:
            self.state = "fault"
            self.fault = "start pressed with no horn" if self.longest == 0 else "start pressed without a fresh 3 s horn"
            RUN_LAMP.set("fault", step)


STARTER = Starter()


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    STARTER.reset()
    HORN_LAMP.reset()
    RUN_LAMP.reset()


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    STARTER.update(states["horn_button"], states["start_button"], CLOCK.step)


# -- scoring: the starter's verdict, with both buttons still standing on the table -------------------------------------
RUNNING = 1.0
HORN_ONLY = 0.3  # the horn sounded long enough, start never pressed
UPRIGHT_DEG = 20.0
ON_TABLE_WITHIN = 0.010  # m: a button's base this close to the height it stands at is on the table


def standing(button: ToolState) -> bool:
    """Upright on the table: not knocked over, lifted, or pushed off the edge. Read from the base's own centre."""
    return bool(button.tilt_deg() <= UPRIGHT_DEG and abs(button.pos[2] - BUTTON_Z) <= ON_TABLE_WITHIN)


def grade(objects, state: str, horn_s: float) -> float:
    """The score as the outcome table states it. ``state`` is the starter's (idle, running or fault); ``horn_s`` is the
    longest the horn was held in without a break, the hold in progress included."""
    if state == "fault" or not all(standing(objects[n]) for n in BUTTONS):
        return 0.0
    if state == "running":
        return RUNNING
    return HORN_ONLY if horn_s >= HORN_S else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def score(objects) -> float:
    return grade(objects, STARTER.state, STARTER.longest / HZ)


def distance(objects) -> float:
    """Metres of cap travel still to go to a press: the horn's until it has sounded long enough and recently, plus the
    start button's until the conveyor runs."""
    if STARTER.state == "running":
        return 0.0

    def to_press(name: str) -> float:
        return max(PRESS_DEPTH - abs(objects[name].sliders["cap"]), 0.0)

    horn = 0.0 if STARTER.warned(CLOCK.step) else to_press("horn_button")
    return horn + to_press("start_button")


# -- the scripted demo: the left arm sounds the horn and parks, then the right arm presses start -----------------------
PRESS_Z = CAP_TOP_Z + PAD_TIP_PAST_TCP - BUTTON_TRAVEL - 0.002  # the pad tips 2 mm past the cap's full travel
ABOVE_Z = CAP_TOP_Z + 0.060
PARK_DY = 0.15  # each arm parks this far out to its own side, clear of the top camera's view of the buttons and lamps
TO_BUTTON_S = 1.5
HORN_DOWN_S = 1.0  # the horn cap goes in past 70% about 2.3 s into the trial
HORN_HOLD_S = 3.6  # at full press; with the way down and up, the cap is in past 70% for about 4 s
START_AFTER_S = 7.0  # the right arm hovers over the start button until then, after the horn has stopped
START_DOWN_S = 1.5
START_HOLD_S = 0.8
LIFT_S = 1.5
PARK_S = 1.5


def _left_plan() -> list[Waypoint]:
    x, y = BUTTON_X, BUTTON_Y
    return [
        Waypoint(TO_BUTTON_S, (x, y, ABOVE_Z), FINGER_CLOSED, "over the horn button"),
        Waypoint(HORN_DOWN_S, (x, y, PRESS_Z), FINGER_CLOSED, "press the horn"),
        Waypoint(HORN_HOLD_S, (x, y, PRESS_Z), FINGER_CLOSED, "hold the horn"),
        Waypoint(LIFT_S, (x, y, ABOVE_Z), FINGER_CLOSED, "release"),
        Waypoint(PARK_S, (x, y + PARK_DY, ABOVE_Z), FINGER_CLOSED, "park"),
    ]


def _right_plan(start_after: float) -> list[Waypoint]:
    x, y = BUTTON_X, -BUTTON_Y
    return [
        Waypoint(start_after, (x, y, ABOVE_Z), FINGER_CLOSED, "wait for the horn"),
        Waypoint(START_DOWN_S, (x, y, PRESS_Z), FINGER_CLOSED, "press start"),
        Waypoint(START_HOLD_S, (x, y, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(LIFT_S, (x, y, ABOVE_Z), FINGER_CLOSED, "release"),
        Waypoint(PARK_S, (x, y - PARK_DY, ABOVE_Z), FINGER_CLOSED, "park"),
    ]


def demo():
    """Left arm: down onto the yellow button, hold it in for 3.6 s, lift off and park to its side. Right arm: hover over
    the green button until 7 s in, by when the horn has sounded and stopped, then press it, lift off and park."""
    return {"left": _left_plan(), "right": _right_plan(START_AFTER_S)}
