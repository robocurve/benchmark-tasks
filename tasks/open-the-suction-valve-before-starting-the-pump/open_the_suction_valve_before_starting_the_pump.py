"""On the station table: a transfer pump, a dark block taped down on the midline, with a flow lamp on top that is
grey now. A yellow suction line runs from the pump's left side (the robots' left) to a brass suction valve standing on
a steel bracket in front of the left arm. A dark handle stands on top of the valve and slides 3 cm along the table,
toward and away from the arms, staying where it is put. It starts at the near end of its travel, nearest the arms,
where the valve is shut; pushed 3 cm away from the arms, it opens the valve. A green start button on a dark base is in
front of the right arm; its cap presses in 8 mm and springs back.

Run it:  inspect-robots run --embodiment yam_arena -E task=open_the_suction_valve_before_starting_the_pump.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from benchkit import Slider, ToolState, on_reset, on_step, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, GREEN, HZ, LAMP_GREEN, LAMP_OFF, LAMP_RED, STEEL, YELLOW, Clock, Lamp, at_rest, distance_xy, pushed,
    slider_fraction,
)
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 130.0  # longer than the run limit: the valve could still be shut after the light turns green
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Put the transfer pump into service: open the suction valve on the inlet line, then press the green start button, "
    "and leave it running with the flow light green."
)

BRASS = (0.78, 0.60, 0.22)
HANDLE_DARK = (0.12, 0.13, 0.15)

# -- the pump and its flow lamp (fixed boxes: the lamp is recoloured by name) ------------------------------------------
PUMP_XY = (0.32, 0.00)
PUMP_SIZE = (0.10, 0.10, 0.08)  # x, y, z (m)
LAMP_SIZE = (0.05, 0.05, 0.03)
LAMP = Lamp("flow_lamp", {"off": LAMP_OFF, "flow": LAMP_GREEN, "dry": LAMP_RED}, "off")

# -- the suction valve, in its own frame: origin at the brass block's centre, x away from the arms ---------------------
VALVE_XY = (0.30, 0.16)
BLOCK = 0.050  # the brass valve body, a cube (m)
BRACKET = (0.090, 0.070, 0.020)  # the steel bracket it stands on: wide along the push, so a push can't tip it
HANDLE = (0.030, 0.020, 0.045)  # x, y, z: the pads meet 4 cm of it, and the wrist camera passes 2 cm over it
TRAVEL = 0.030  # the handle slides this far from shut to open (m)
HANDLE_REST_X = -0.015  # shut: overhangs the block's near face by 5 mm; open, its far face by 5 mm
VALVE_Z = TOP + BRACKET[2] + BLOCK / 2  # the block's centre: 4.5 cm up, level with the suction line
BLOCK_TOP = BRACKET[2] + BLOCK  # above the table (m): the surface the handle stands on

VALVE = tool(
    "suction_valve",
    pos=(*VALVE_XY, VALVE_Z),
    boxes=[
        ((0.0, 0.0, 0.0), (BLOCK,) * 3, BRASS),
        ((0.0, 0.0, -BLOCK / 2 - BRACKET[2] / 2), BRACKET, STEEL),
    ],
    sliders=[
        # No spring: the handle stays where it is pushed. It rests shut and is pushed +x to open.
        Slider("handle", rest=(HANDLE_REST_X, 0.0, BLOCK / 2 + HANDLE[2] / 2), size=HANDLE, axis="X", lower=0.0,
               upper=TRAVEL, color=HANDLE_DARK, rate=0.0, damping=8.0, mass=0.02),
    ],
    mass=3.0,  # heavy enough to stay put when pushed; taped down on a real bench
)

# -- the suction line: fixed, from the pump's +y face to 3 mm short of the loose valve body ----------------------------
LINE = 0.030  # square section (m)
LINE_GAP = 0.003  # the fixed pipe never touches the loose valve
LINE_Y0 = PUMP_XY[1] + PUMP_SIZE[1] / 2
LINE_Y1 = VALVE_XY[1] - BLOCK / 2 - LINE_GAP
LINE_Z = VALVE_Z  # 4.5 cm up, into the middle of the valve's face

# -- the start button --------------------------------------------------------------------------------------------------
BUTTON_XY = (0.22, -0.14)
BUTTON_BASE = (0.07, 0.07, 0.030)
BUTTON_TRAVEL = 0.008
BUTTON_CAP = (0.040, 0.040, 0.015)
BUTTON_Z = TOP + BUTTON_BASE[2] / 2
START_BUTTON = tool(
    "start_button",
    pos=(*BUTTON_XY, BUTTON_Z),
    boxes=[((0.0, 0.0, 0.0), BUTTON_BASE, DARK)],
    sliders=[Slider("cap", rest=(0.0, 0.0, BUTTON_BASE[2] / 2 + BUTTON_CAP[2] / 2), size=BUTTON_CAP, axis="Z",
                    lower=-BUTTON_TRAVEL, upper=0.0, color=GREEN, rate=300.0, damping=8.0, mass=0.01)],
    mass=3.0,  # heavy enough to stay put when pressed; taped down on a real bench
)

OBJECTS = [
    Box("pump", pos=(*PUMP_XY, TOP + PUMP_SIZE[2] / 2), size=PUMP_SIZE, color=DARK),
    LAMP.box((*PUMP_XY, TOP + PUMP_SIZE[2] + LAMP_SIZE[2] / 2), LAMP_SIZE),
    Box("suction_line", pos=(VALVE_XY[0], (LINE_Y0 + LINE_Y1) / 2, LINE_Z), size=(LINE, LINE_Y1 - LINE_Y0, LINE),
        color=YELLOW),
    VALVE,
    START_BUTTON,
]

# -- reading the valve and the button ----------------------------------------------------------------------------------
OPEN_AT = 0.7  # of the handle's travel (2.1 cm): the valve counts as open from here
SHUT_BELOW = 0.3  # under this (0.9 cm) a running pump's suction is shut
PRESS_AT = 0.7  # of the cap's travel (5.6 mm): a press
RELEASE_AT = 0.3  # back out this far (2.4 mm): released, so the next press counts
MOVED_M = 0.030  # a fixture moved further than this from its tape: the valve tears the suction line off
TIPPED_DEG = 20.0


def valve_open(valve: ToolState) -> bool:
    return pushed(valve, "handle", TRAVEL, OPEN_AT)


def valve_shut(valve: ToolState) -> bool:
    return slider_fraction(valve, "handle", TRAVEL) < SHUT_BELOW


def in_place(state: ToolState, xy) -> bool:
    """Still where it was taped down, and standing."""
    return distance_xy(state, xy) <= MOVED_M and state.tilt_deg() <= TIPPED_DEG


# -- the pump's starter, watched every step ----------------------------------------------------------------------------
FLOW_S = 2.0  # from a start with the suction open to the green light
FLOW_STEPS = round(FLOW_S * HZ)


class Pump:
    """Stopped until a start press. A press with the suction valve under 70 % open runs the pump dry; a press with it
    open starts it, and the flow light turns green 2.0 s later. The valve back under 30 % open while the pump runs
    also runs it dry. A dry run is permanent and shows red. Once running, the start button does nothing more. The
    valve or the button knocked out of place at any step is remembered too: it scores 0 even if put back."""

    def __init__(self) -> None:
        self.clock = Clock()
        self.reset()

    def reset(self) -> None:
        self.clock.reset()
        self.state = "stopped"  # stopped, running or dry_run
        self.started = 0  # the step it started
        self.flowing = False  # running with the flow light green
        self.pressed = False
        self.presses = 0
        self.why = ""
        self.knocked = ""  # which fixture was moved or tipped, if one was

    def run_dry(self, why: str) -> None:
        self.state, self.flowing, self.why = "dry_run", False, why
        LAMP.set("dry", self.clock.step)

    def update(self, valve: ToolState, button: ToolState) -> None:
        step = self.clock.step
        if not self.knocked:
            if not in_place(valve, VALVE_XY):
                self.knocked = "suction valve moved or tipped"
            elif not in_place(button, BUTTON_XY):
                self.knocked = "start button moved or tipped"
        down = pushed(button, "cap", BUTTON_TRAVEL, PRESS_AT)
        edge = down and not self.pressed
        if down:
            self.pressed = True
        elif at_rest(button, "cap", BUTTON_TRAVEL, RELEASE_AT):
            self.pressed = False
        if edge:
            self.presses += 1
        if self.state == "dry_run":
            return
        if edge and self.state == "stopped":
            if not valve_open(valve):
                self.run_dry("started with the suction valve shut")
                return
            self.state, self.started = "running", step
        if self.state == "running":
            if valve_shut(valve):
                self.run_dry("suction valve shut while running")
            elif not self.flowing and step - self.started >= FLOW_STEPS:
                self.flowing = True
                LAMP.set("flow", step)


PUMP = Pump()


@on_reset
def _reset(states) -> None:
    PUMP.reset()
    LAMP.reset()


@on_step
def _watch(states) -> None:
    PUMP.clock.tick()
    PUMP.update(states["suction_valve"], states["start_button"])


# -- scoring: the end state, with the starter's verdict ----------------------------------------------------------------
IN_SERVICE = 1.0
STARTED = 0.5  # started properly in the last 2 s of the run, the light not green yet
LINED_UP = 0.3
OUT_OF_REACH = 1.0  # m: what distance() reports once the goal can't be reached any more


def grade(objects, pump: Pump) -> float:
    """The score as the outcome table states it."""
    valve = objects["suction_valve"]
    if pump.state == "dry_run" or pump.knocked:
        return 0.0
    if not (in_place(valve, VALVE_XY) and in_place(objects["start_button"], BUTTON_XY)):
        return 0.0
    if pump.state == "running":
        if not valve_open(valve):
            return 0.0  # running on a suction eased back under 70 %: no better than a dry run
        return IN_SERVICE if pump.flowing else STARTED
    return LINED_UP if valve_open(valve) else 0.0


def success(objects) -> bool:
    return grade(objects, PUMP) >= 1.0 and objects["suction_valve"].resting(max_speed=0.02, max_spin=1.0)


def score(objects) -> float:
    return grade(objects, PUMP)


def distance(objects) -> float:
    """Metres of the valve handle's travel still to push, plus the cap's travel until the pump has been started; 1 m
    once the pump has run dry or a fixture was knocked out of place, when the goal is out of reach."""
    if PUMP.state == "dry_run" or PUMP.knocked:
        return OUT_OF_REACH
    left = TRAVEL - min(abs(objects["suction_valve"].sliders["handle"]), TRAVEL)
    return float(left + (BUTTON_TRAVEL if PUMP.state == "stopped" else 0.0))


# -- the scripted demo: the left arm opens the valve; the right arm presses start once it is open ----------------------
FINGER_X = 0.020  # half the closed pads' span along x: the push face for a push along x
OVERSHOOT = 0.006  # commanded past the stop, so the valve still opens fully if the arm falls a little short
BEHIND = 0.030  # the fingers come down this far behind the handle's face
BACK_OFF = 0.015  # after the push the fingers back away from the handle before lifting
PUSH_Z = TOP + BLOCK_TOP + 0.005 + 0.013  # finger tips 5 mm above the brass block, against the handle
LEFT_HOVER_Z = TOP + 0.185
RIGHT_HOVER_Z = TOP + 0.150
FACE_SHUT_X = VALVE_XY[0] + HANDLE_REST_X - HANDLE[0] / 2  # the handle's near face (world x), shut
FACE_OPEN_X = FACE_SHUT_X + TRAVEL
PUSH_START_X = FACE_SHUT_X - BEHIND - FINGER_X
PUSH_END_X = FACE_OPEN_X - FINGER_X + OVERSHOOT
CAP_TOP_Z = BUTTON_Z + BUTTON_BASE[2] / 2 + BUTTON_CAP[2]
PRESS_Z = CAP_TOP_Z + 0.013 - BUTTON_TRAVEL - 0.002  # the tips 2 mm past the cap's full travel
ABOVE_CAP_Z = CAP_TOP_Z + 0.060
LEFT_ASIDE = (0.20, 0.29)  # out of the top camera's view of the valve and the lamp
RIGHT_ASIDE = (0.20, -0.29)
PUSH_S = 2.5
PRESS_S, PRESS_HOLD_S = 1.5, 0.8
RIGHT_STARTS_AFTER = 6.0  # the valve reads fully open at about 5.6 s and the push ends at 6.0 s
FLOW_WAIT_S = 2.0  # parked, with the light green
TRIM = {"valve": (0.0, 0.0), "button": (0.0, 0.0)}


def _left_plan() -> list[Waypoint]:
    tx, ty = TRIM["valve"]
    y, start, end = VALVE_XY[1] + ty, PUSH_START_X + tx, PUSH_END_X + tx
    return [
        Waypoint(1.5, (start, y, LEFT_HOVER_Z), FINGER_CLOSED, "over the valve handle"),
        Waypoint(1.5, (start, y, PUSH_Z), FINGER_CLOSED, "down behind the handle"),
        Waypoint(0.5, (start, y, PUSH_Z), FINGER_CLOSED, "settle"),
        Waypoint(PUSH_S, (end, y, PUSH_Z), FINGER_CLOSED, "push the valve open"),
        Waypoint(0.8, (end, y, PUSH_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.0, (end - BACK_OFF, y, PUSH_Z), FINGER_CLOSED, "back off the handle"),
        Waypoint(1.5, (end - BACK_OFF, y, LEFT_HOVER_Z), FINGER_CLOSED, "lift off"),
        Waypoint(2.0, (*LEFT_ASIDE, LEFT_HOVER_Z), FINGER_CLOSED, "clear of the valve"),
    ]


def _right_plan(start_after: float) -> list[Waypoint]:
    tx, ty = TRIM["button"]
    x, y = BUTTON_XY[0] + tx, BUTTON_XY[1] + ty
    return [
        Waypoint(start_after, (x, y, ABOVE_CAP_Z), FINGER_CLOSED, "wait for the valve"),
        Waypoint(PRESS_S, (x, y, PRESS_Z), FINGER_CLOSED, "press start"),
        Waypoint(PRESS_HOLD_S, (x, y, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.5, (x, y, ABOVE_CAP_Z), FINGER_CLOSED, "release"),
        Waypoint(2.0, (*RIGHT_ASIDE, RIGHT_HOVER_Z), FINGER_CLOSED, "clear of the button"),
        Waypoint(FLOW_WAIT_S, (*RIGHT_ASIDE, RIGHT_HOVER_Z), FINGER_CLOSED, "watch the flow light"),
    ]


def demo():
    """Left arm: push the suction valve's handle open and move aside. Right arm: hover over the start button until the
    valve is open, press it, move aside, and wait for the flow light."""
    return {"left": _left_plan(), "right": _right_plan(RIGHT_STARTS_AFTER)}
