"""On the station table: the local controls of two pumps that feed one header, and the header's flow lamp.

- In front of the left arm, pump A's controls: a push button with a red cap, A's stop button, and behind it, on a
  post with a white "A" label, A's running lamp.
- In front of the right arm, pump B's controls: a push button with a green cap, B's start button, and behind it, on
  a post with a white "B" label, B's running lamp.
- On the midline at the back, on a taller post: the header flow lamp.

Both buttons are spring-loaded: a cap pressed in comes back up when it is let go. A running lamp is green while its
pump runs and grey while it is stopped. The flow lamp shows the header's flow; it is green. Pump A is running and
pump B is stopped.

Run it:  inspect-robots run --embodiment yam_arena -E task=start_the_standby_pump_before_stopping_the_duty_pump.py
         --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, Tool, ToolState, on_reset, on_step, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, GREEN, HZ, LAMP_GREEN, LAMP_OFF, LAMP_RED, RED, WHITE, Clock, Lamp, pushed,
)
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Change over from pump A to pump B without interrupting the flow: press pump B's green start button and wait "
    "until B's running light is green, then press pump A's red stop button."
)

# -- the two push buttons: A's stop in front of the left arm, B's start in front of the right --------------------------
BUTTON_BASE = (0.07, 0.07, 0.030)  # m
BUTTON_CAP = (0.040, 0.040, 0.015)
BUTTON_TRAVEL = 0.008  # m the cap presses in
BUTTON_Z = TOP + BUTTON_BASE[2] / 2
CAP_REST = (0.0, 0.0, BUTTON_BASE[2] / 2 + BUTTON_CAP[2] / 2)  # the cap's centre at rest (tool frame)
CAP_TOP_Z = TOP + BUTTON_BASE[2] + BUTTON_CAP[2]  # the cap's top at rest (world)
A_STOP_XY = (0.22, 0.16)
B_START_XY = (0.22, -0.16)


def _button(name: str, xy, cap_colour) -> Tool:
    return tool(
        name,
        pos=(*xy, BUTTON_Z),
        boxes=[((0.0, 0.0, 0.0), BUTTON_BASE, DARK)],
        sliders=[Slider("cap", rest=CAP_REST, size=BUTTON_CAP, axis="Z", lower=-BUTTON_TRAVEL, upper=0.0,
                        color=cap_colour, rate=300.0, damping=8.0, mass=0.01)],
        mass=3.0,  # heavy enough to stay put when pressed; taped down on a real bench
    )


A_STOP = _button("a_stop", A_STOP_XY, RED)
B_START = _button("b_start", B_START_XY, GREEN)
BUTTONS = {"a_stop": A_STOP, "b_start": B_START}

# -- the lamps: fixed boxes on posts, recoloured as the pumps start and stop -------------------------------------------
# Each running lamp stands 11 cm behind and 6 cm outboard of its button: while the fingers are on the cap, the wrist
# camera (7 cm ahead of the TCP) stays over 4.4 cm from the lamp head and the gripper housing 4 cm.
POST = (0.02, 0.02, 0.10)  # m
RUN_HEAD = (0.04, 0.04, 0.03)
LABEL = (0.002, 0.030, 0.025)  # a white plate on the post's face towards the arms: it reads "A" or "B" on a real bench
LABEL_UP = 0.065  # m: the label's centre above the table
A_LAMP_XY = (0.33, 0.22)
B_LAMP_XY = (0.33, -0.22)
FLOW_XY = (0.38, 0.0)
FLOW_POST = (0.02, 0.02, 0.14)
FLOW_HEAD = (0.05, 0.05, 0.03)

A_LAMP = Lamp("a_lamp", {"run": LAMP_GREEN, "off": LAMP_OFF}, "run")
B_LAMP = Lamp("b_lamp", {"run": LAMP_GREEN, "off": LAMP_OFF}, "off")
FLOW_LAMP = Lamp("flow_lamp", {"flow": LAMP_GREEN, "lost": LAMP_RED}, "flow")


def _on_a_post(lamp: Lamp, post_name: str, xy, post, head) -> list[Box]:
    x, y = xy
    return [Box(post_name, pos=(x, y, TOP + post[2] / 2), size=post, color=DARK),
            lamp.box((x, y, TOP + post[2] + head[2] / 2), head)]


def _label(name: str, xy) -> Box:
    x, y = xy
    return Box(name, pos=(x - POST[0] / 2 - LABEL[0] / 2, y, TOP + LABEL_UP), size=LABEL, color=WHITE)


OBJECTS = [
    A_STOP,
    B_START,
    *_on_a_post(A_LAMP, "a_post", A_LAMP_XY, POST, RUN_HEAD),
    _label("a_label", A_LAMP_XY),
    *_on_a_post(B_LAMP, "b_post", B_LAMP_XY, POST, RUN_HEAD),
    _label("b_label", B_LAMP_XY),
    *_on_a_post(FLOW_LAMP, "flow_post", FLOW_XY, FLOW_POST, FLOW_HEAD),
]

# -- the pumps: B comes up 6 s after its start goes in, A stops at once; neither running loses the flow ----------------
PRESSED_AT = 0.7  # of the cap's travel: 5.6 mm in counts as pressed
START_S = 6.0  # s from B's start going in to B running (its lamp green): its run-up to speed and pressure
START_STEPS = round(START_S * HZ)


class Pumps:
    """What the pumps have done this trial, in steps since the reset. Everything latches: B has no stop button, A no
    start button, and lost flow can't be taken back."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.a_running = True
        self.a_stopped_at = None
        self.b_started_at = None  # the step B's start first went in
        self.b_running = False
        self.b_running_at = None
        self.outage = False  # a step with neither pump running: the header lost its flow
        self.outage_at = None

    @property
    def b_started(self) -> bool:
        return self.b_started_at is not None

    def step(self, step: int, b_pressed: bool, a_pressed: bool) -> None:
        if b_pressed and not self.b_started:
            self.b_started_at = step
        if self.b_started and not self.b_running and step - self.b_started_at >= START_STEPS:
            self.b_running, self.b_running_at = True, step
        if a_pressed and self.a_running:
            self.a_running, self.a_stopped_at = False, step
        if not (self.a_running or self.b_running) and not self.outage:
            self.outage, self.outage_at = True, step


PUMPS = Pumps()
CLOCK = Clock()


def pressed(button: ToolState) -> bool:
    return pushed(button, "cap", BUTTON_TRAVEL, PRESSED_AT)


def _show() -> None:
    A_LAMP.set("run" if PUMPS.a_running else "off", CLOCK.step)
    B_LAMP.set("run" if PUMPS.b_running else "off", CLOCK.step)
    FLOW_LAMP.set("lost" if PUMPS.outage else "flow", CLOCK.step)


@on_reset
def _reset(states) -> None:
    PUMPS.reset()
    CLOCK.reset()
    for lamp in (A_LAMP, B_LAMP, FLOW_LAMP):
        lamp.reset()


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    PUMPS.step(CLOCK.step, pressed(states["b_start"]), pressed(states["a_stop"]))
    _show()


# -- scoring: the end state, and whether the header ever lost its flow -------------------------------------------------
CHANGEOVER_STARTED = 0.5  # B started, A still running: the header is fed, the changeover half done
MOVED_M = 0.03
TIPPED_DEG = 20.0


def in_place(button: ToolState, spec: Tool) -> bool:
    """A button where it was taped down: within 3 cm of its start and leaning under 20 degrees."""
    return bool(np.linalg.norm(button.pos - np.array(spec.pos)) <= MOVED_M and button.tilt_deg() <= TIPPED_DEG)


def grade(objects, pumps: Pumps) -> float:
    """The score as the outcome table states it."""
    if pumps.outage or not all(in_place(objects[n], spec) for n, spec in BUTTONS.items()):
        return 0.0
    if not pumps.b_started:
        return 0.0  # nothing done
    if pumps.a_running:
        return CHANGEOVER_STARTED  # B running, or still coming up at the end
    return 1.0  # A stopped with the flow never lost, so B was already running


def success(objects) -> bool:
    return grade(objects, PUMPS) >= 1.0 and all(objects[n].resting() for n in BUTTONS)


def score(objects) -> float:
    return grade(objects, PUMPS)


def distance(objects) -> float:
    """Metres of cap travel still to go along the changeover: B's start, then A's stop. Once the flow is lost, the full
    distance: nothing left on the table makes up for it."""
    if PUMPS.outage:
        return 2 * PRESSED_AT * BUTTON_TRAVEL

    def to_go(name: str, done: bool) -> float:
        return 0.0 if done else max(PRESSED_AT * BUTTON_TRAVEL - abs(objects[name].sliders["cap"]), 0.0)

    return to_go("b_start", PUMPS.b_started) + to_go("a_stop", not PUMPS.a_running)


# -- the scripted demo: the right arm presses B's start; the left presses A's stop once B's lamp is green --------------
PRESS_Z = CAP_TOP_Z + PAD_TIP_PAST_TCP - BUTTON_TRAVEL - 0.002  # the pad tips 2 mm past the cap's full travel
HOVER_Z = TOP + 0.160
PARK_DY = 0.15  # m: after its press each arm parks this far to its own side, out of the top camera's view of the lamps
TO_BUTTON_S, DESCEND_S, HOLD_S, LIFT_S, PARK_S = 1.5, 1.5, 0.8, 1.5, 1.5
A_DESCENT_AT = 10.0  # s: the left arm leaves its hover over A's stop; B's lamp turns green about 8.8 s in


def _press(xy, button: str, side: float) -> list[Waypoint]:
    """Down onto the cap, hold it in, lift straight up, then park to the arm's own side (+1 left, -1 right)."""
    x, y = xy
    return [
        Waypoint(DESCEND_S, (x, y, PRESS_Z), FINGER_CLOSED, f"press {button}"),
        Waypoint(HOLD_S, (x, y, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(LIFT_S, (x, y, HOVER_Z), FINGER_CLOSED, "lift off"),
        Waypoint(PARK_S, (x, y + side * PARK_DY, HOVER_Z), FINGER_CLOSED, "park to its side"),
    ]


def _right_plan() -> list[Waypoint]:
    return [Waypoint(TO_BUTTON_S, (*B_START_XY, HOVER_Z), FINGER_CLOSED, "over B's start button"),
            *_press(B_START_XY, "B's start", -1.0)]


def _left_plan(descend_at: float = A_DESCENT_AT) -> list[Waypoint]:
    over = (*A_STOP_XY, HOVER_Z)
    plan = [Waypoint(TO_BUTTON_S, over, FINGER_CLOSED, "over A's stop button")]
    if descend_at > TO_BUTTON_S:
        plan.append(Waypoint(descend_at - TO_BUTTON_S, over, FINGER_CLOSED, "wait for B's running lamp"))
    return plan + _press(A_STOP_XY, "A's stop", 1.0)


def demo():
    """Right arm: press B's start, lift off and park to its side. Left arm: hover over A's stop until B's running lamp
    has been green for over a second, then press A's stop, lift off and park to its side."""
    return {"left": _left_plan(), "right": _right_plan()}
