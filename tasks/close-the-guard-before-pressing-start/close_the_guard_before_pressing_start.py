"""On the station table: a small light grey machine with an open top, taped down between the arms a little to the
left arm's side. A pale blue guard panel slides across the machine's top along the robot's left-right direction; it
stands open beside the machine on the left arm's side, resting on a grey stand, with a grey handle bar across its far
end. A cycle lamp on a dark base stands behind the machine; it is grey now and lights green when the machine finishes
a cycle. A green start button on a dark base is in front of the right arm. A blue 4 cm cube, the blank, sits on a low
steel pallet in front of the left arm, and a yellow square output tray is further to the left, nearer the left arm.

Run it:  inspect-robots run --embodiment yam_arena -E task=close_the_guard_before_pressing_start.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, ToolState, on_reset, on_step, recolor, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, GREY, HZ, LAMP_GREEN, LAMP_OFF, LAMP_RED, STEEL, YELLOW, Clock, Lamp, at_rest, distance_xy, over, pushed,
)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Load the blank into the machine, slide the guard shut, and press the green start button. When the cycle light "
    "turns green, slide the guard open and put the finished part in the output tray."
)

LIGHT_GREY = (0.78, 0.79, 0.81)
PALE_BLUE = (0.70, 0.82, 0.92)
GREEN_CAP = (0.15, 0.65, 0.25)
BLANK_BLUE = (0.20, 0.35, 0.65)
PART_ORANGE = (0.92, 0.45, 0.08)

# -- the machine, in its own frame: origin at its centre, y along the guard's travel -----------------------------------
MACHINE_XY = (0.30, 0.08)
BOX_SIZE = (0.14, 0.14, 0.06)  # 6 cm walls: the wrist camera, 7 cm ahead of the TCP, clears the rim at every pick
WALL = 0.008
FLOOR = 0.006
INNER = BOX_SIZE[0] - 2 * WALL  # 12.4 cm square opening
RIM = BOX_SIZE[2] / 2  # above the machine's centre
FLOOR_TOP = -RIM + FLOOR  # the floor's top face, machine frame

GUARD_T = 0.006
GUARD_GAP = 0.001
GUARD_TRAVEL = BOX_SIZE[1]  # open, it clears the whole opening; shut, it covers it
GUARD_Z = RIM + GUARD_GAP + GUARD_T / 2
GUARD_SHUT_WITHIN = 0.010  # m of the shut stop
HANDLE = (0.030, 0.015, 0.030)
HANDLE_Y = BOX_SIZE[1] / 2 - HANDLE[1] / 2  # on the guard's far (+y) end, where the left arm can push both faces
STAND_Y = 0.19  # under the open guard's far end
STAND = (BOX_SIZE[0], 0.020, BOX_SIZE[2])

MACHINE = tool(
    "machine",
    pos=(*MACHINE_XY, TOP + BOX_SIZE[2] / 2),
    boxes=[
        ((0.0, 0.0, -RIM + FLOOR / 2), (INNER, INNER, FLOOR), DARK),
        ((BOX_SIZE[0] / 2 - WALL / 2, 0.0, 0.0), (WALL, BOX_SIZE[1], BOX_SIZE[2]), LIGHT_GREY),
        ((-BOX_SIZE[0] / 2 + WALL / 2, 0.0, 0.0), (WALL, BOX_SIZE[1], BOX_SIZE[2]), LIGHT_GREY),
        ((0.0, BOX_SIZE[1] / 2 - WALL / 2, 0.0), (INNER, WALL, BOX_SIZE[2]), LIGHT_GREY),
        ((0.0, -BOX_SIZE[1] / 2 + WALL / 2, 0.0), (INNER, WALL, BOX_SIZE[2]), LIGHT_GREY),
        ((0.0, STAND_Y, 0.0), STAND, GREY),
    ],
    sliders=[
        # No spring: the guard stays where it is pushed. It rests OPEN (+y) and is pushed -y to shut.
        Slider("guard", rest=(0.0, GUARD_TRAVEL, GUARD_Z), size=(BOX_SIZE[0], BOX_SIZE[1], GUARD_T), axis="Y",
               lower=-GUARD_TRAVEL, upper=0.0, color=PALE_BLUE, rate=0.0, damping=8.0, mass=0.10,
               parts=(((0.0, HANDLE_Y, GUARD_T / 2 + HANDLE[2] / 2), HANDLE, GREY),)),
    ],
    mass=3.0,  # heavy enough to stay put when pushed; taped down on a real bench
)

# -- the lamp, the start button, the blank on its pallet, and the tray ----------------------------------------------
LAMP_XY = (0.415, 0.08)
LAMP_BASE = (0.07, 0.07, 0.020)
LAMP_SIZE = (0.06, 0.06, 0.030)
LAMP = Lamp("cycle_lamp", {"off": LAMP_OFF, "done": LAMP_GREEN, "fault": LAMP_RED}, "off")

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
                    lower=-BUTTON_TRAVEL, upper=0.0, color=GREEN_CAP, rate=300.0, damping=8.0, mass=0.01)],
    mass=3.0,
)

BLANK = 0.040
BLANK_MASS = 0.05
BLANK_XY = (0.17, 0.12)
PALLET_H = 0.015  # the blank starts this high: at the pick, the wrist camera then clears the machine's rim by 2 cm
PALLET_SIZE = (0.060, 0.060, PALLET_H)

TRAY_XY = (0.18, 0.24)
TRAY_INNER = 0.080
TRAY_H = 0.020
TRAY_WALL = 0.003


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


OBJECTS = [
    MACHINE,
    Box("lamp_base", pos=(*LAMP_XY, TOP + LAMP_BASE[2] / 2), size=LAMP_BASE, color=DARK),
    LAMP.box((*LAMP_XY, TOP + LAMP_BASE[2] + LAMP_SIZE[2] / 2), LAMP_SIZE),
    START_BUTTON,
    Box("pallet", pos=(*BLANK_XY, TOP + PALLET_H / 2), size=PALLET_SIZE, color=STEEL),
    Box("blank", pos=(*BLANK_XY, TOP + PALLET_H + BLANK / 2), size=(BLANK,) * 3, mass=BLANK_MASS, color=BLANK_BLUE),
    *_collar("tray", *TRAY_XY, TRAY_INNER, TRAY_H, TRAY_WALL, YELLOW),
]

# -- where things are -----------------------------------------------------------------------------------------------
ON_FLOOR_WITHIN = 0.010  # m: the blank's bottom this close to the machine's floor counts as loaded
TRAY_BOTTOM_WITHIN = 0.015  # m above the table
FLAT_DEG = 35.0  # a cube resting on any face leans less than this; on an edge it leans 45
UPRIGHT_DEG = 20.0  # the machine


def guard_shut(machine: ToolState) -> bool:
    """Within 1 cm of the shut stop."""
    return bool(machine.sliders["guard"] <= -(GUARD_TRAVEL - GUARD_SHUT_WITHIN))


def in_machine(blank, machine: ToolState) -> bool:
    """The blank's centre inside the walls and its bottom on the floor of a machine still standing. Read in the
    machine's own frame, so shoving the machine along changes nothing."""
    x, y, _ = machine.rotation().T @ (blank.pos - machine.pos)
    floor_z = machine.to_world((0.0, 0.0, FLOOR_TOP))[2]
    return bool(abs(x) <= INNER / 2 and abs(y) <= INNER / 2 and blank.low[2] < floor_z + ON_FLOOR_WITHIN
                and machine.tilt_deg() <= UPRIGHT_DEG)


def flat_deg(state) -> float:
    """How far a cube's nearest face is from level: 0 resting on any face, 45 balanced on an edge."""
    x, y, z, w = state.quat_xyzw
    up = (abs(2 * (x * z - w * y)), abs(2 * (y * z + w * x)), abs(1 - 2 * (x * x + y * y)))
    return float(np.degrees(np.arccos(np.clip(max(up), -1.0, 1.0))))


def in_tray(part) -> bool:
    """Resting in the output tray: centre inside the collar, bottom on the table, flat on a face."""
    return bool(over(part, TRAY_XY, (TRAY_INNER / 2, TRAY_INNER / 2)) and part.low[2] < TOP + TRAY_BOTTOM_WITHIN
                and flat_deg(part) <= FLAT_DEG)


# -- the machine's controller, watched every step ----------------------------------------------------------------------
PRESS_AT = 0.7  # of the cap's travel: a press (5.6 mm: the microswitch under the cap on a real bench)
RELEASE_AT = 0.3  # back out this far: released, so the next press counts
CYCLE_S = 3.0
CYCLE_STEPS = int(round(CYCLE_S * HZ))
CLOCK = Clock()


class Cycle:
    """idle until a start press with the guard shut and the blank inside, then running for CYCLE_S, then done. A press
    with the guard open, or the guard leaving its shut position while running, is a fault for the rest of the trial.
    A press with the guard shut and nothing inside does nothing."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.state = "idle"
        self.started = 0  # the step the cycle began
        self.pressed = False
        self.presses = 0
        self.fault = ""

    def trip(self, why: str, step: int) -> None:
        self.state, self.fault = "fault", why
        LAMP.set("fault", step)

    def update(self, machine: ToolState, blank, button: ToolState, step: int) -> None:
        down = pushed(button, "cap", BUTTON_TRAVEL, PRESS_AT)
        edge = down and not self.pressed
        if down:
            self.pressed = True
        elif at_rest(button, "cap", BUTTON_TRAVEL, RELEASE_AT):
            self.pressed = False
        if self.state == "fault":
            return
        shut = guard_shut(machine)
        if edge:
            self.presses += 1
            if not shut:
                self.trip("start pressed with the guard open", step)
                return
            if self.state == "idle" and in_machine(blank, machine):
                self.state, self.started = "running", step
        if self.state == "running":
            if not shut:
                self.trip("guard opened mid-cycle", step)
            elif step - self.started >= CYCLE_STEPS:
                self.state = "done"
                LAMP.set("done", step)
                recolor("blank", PART_ORANGE)


CYCLE = Cycle()


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    CYCLE.reset()
    LAMP.reset()
    recolor("blank", BLANK_BLUE)


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    CYCLE.update(states["machine"], states["blank"], states["start_button"], CLOCK.step)


# -- scoring: the end state, with the controller's verdict -----------------------------------------------------------
DONE_IN_TRAY = 1.0
DONE_ELSEWHERE = 0.5
LOADED_SHUT_NO_CYCLE = 0.3
LOADED_NO_CYCLE = 0.2


def grade(objects, state: str) -> float:
    """The score as the outcome table states it. ``state`` is the controller's: idle, running, done or fault."""
    machine, part = objects["machine"], objects["blank"]
    if state == "fault" or machine.tilt_deg() > UPRIGHT_DEG:
        return 0.0
    if state == "done":
        return DONE_IN_TRAY if in_tray(part) else DONE_ELSEWHERE
    if in_machine(part, machine):
        return LOADED_SHUT_NO_CYCLE if guard_shut(machine) else LOADED_NO_CYCLE
    return 0.0


def success(objects) -> bool:
    return grade(objects, CYCLE.state) >= 1.0 and objects["blank"].resting(max_speed=0.02, max_spin=1.0)


def score(objects) -> float:
    return grade(objects, CYCLE.state)


def distance(objects) -> float:
    """Metres from the blank to the machine plus what's left of the guard's travel to shut; once the cycle is done,
    from the part to the tray."""
    machine, part = objects["machine"], objects["blank"]
    if CYCLE.state == "done":
        return distance_xy(part, TRAY_XY)
    return distance_xy(part, MACHINE_XY) + max(GUARD_TRAVEL + machine.sliders["guard"], 0.0)


# -- the scripted demo: the left arm loads, shuts, opens and unloads; the right arm presses start ---------------------
GRIP_Z = TOP + PALLET_H + 0.024  # TCP on the 4 cm cube standing on its pallet
GRIP_IN_Z = TOP + FLOOR + 0.024  # the same cube standing on the machine's floor
PLACE_IN_Z = GRIP_IN_Z + 0.003
TRAY_PLACE_Z = TOP + 0.024 + 0.020  # let go 2 cm up: the pad tips and the wrist camera stay clear of the tray and guard
TRAVEL_Z = TOP + 0.185
HOVER_Z = TOP + 0.185
FINGER_IN = -0.030  # 60 mm between the pads inside the machine: 10 mm round the cube, 22 mm from the walls
PUSH_Z = TOP + BOX_SIZE[2] + GUARD_GAP + GUARD_T + 0.005 + 0.013  # finger tips 5 mm above the guard, on the handle
FINGER_Y = 0.0065  # half the closed fingers' width along y, near the tips
OVERSHOOT = 0.006  # commanded past the stop, so the guard still shuts (or opens) if the arm falls a little short
HANDLE_OPEN_Y = MACHINE_XY[1] + GUARD_TRAVEL + HANDLE_Y  # the handle's centre (world y) with the guard open
HANDLE_SHUT_Y = HANDLE_OPEN_Y - GUARD_TRAVEL
SHUT_START_Y = HANDLE_OPEN_Y + HANDLE[1] / 2 + 0.030
SHUT_END_Y = HANDLE_SHUT_Y + HANDLE[1] / 2 + FINGER_Y - OVERSHOOT
OPEN_START_Y = HANDLE_SHUT_Y - HANDLE[1] / 2 - 0.030
OPEN_END_Y = HANDLE_OPEN_Y - HANDLE[1] / 2 - FINGER_Y + OVERSHOOT
BUTTON_TOP_Z = BUTTON_Z + BUTTON_BASE[2] / 2 + BUTTON_CAP[2]
PRESS_Z = BUTTON_TOP_Z + 0.013 - BUTTON_TRAVEL - 0.002
BUTTON_ABOVE_Z = BUTTON_TOP_Z + 0.060
PARK_ASIDE = 0.15  # each arm waits this far to its own side of where it works, out of the top camera's view
LEFT_PARK = (MACHINE_XY[0], OPEN_START_Y + PARK_ASIDE, HOVER_Z)
RIGHT_PARK = (BUTTON_XY[0], BUTTON_XY[1] - PARK_ASIDE, BUTTON_ABOVE_Z)
BUTTON_APPROACH_S, PRESS_S, PRESS_HOLD_S, PRESS_LIFT_S = 1.5, 1.5, 0.8, 1.5
PARK_S = 1.5
AFTER_PRESS_S = 3.5  # the left arm leaves its park this long after the right arm has lifted off the button
CYCLE_WAIT_S = BUTTON_APPROACH_S + PRESS_S + PRESS_HOLD_S + PRESS_LIFT_S + AFTER_PRESS_S - PARK_S
SPEED = 0.7
TRIM = {"pick": (0.0, 0.0), "load": (0.0, 0.0), "push": (0.0, 0.0), "tray": (0.0, 0.0)}


def _push(start_y: float, end_y: float, label: str) -> list[Waypoint]:
    x = MACHINE_XY[0] + TRIM["push"][0]
    start, end = start_y + TRIM["push"][1], end_y + TRIM["push"][1]
    return [
        Waypoint(1.5, (x, start, HOVER_Z), FINGER_CLOSED, "over the handle"),
        Waypoint(1.5, (x, start, PUSH_Z), FINGER_CLOSED, "down behind the handle"),
        Waypoint(0.5, (x, start, PUSH_Z), FINGER_CLOSED, "settle"),
        Waypoint(4.0, (x, end, PUSH_Z), FINGER_CLOSED, label),
        Waypoint(0.8, (x, end, PUSH_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.5, (x, end, HOVER_Z), FINGER_CLOSED, "lift off"),
    ]


def _left_plan() -> list[Waypoint]:
    k = 1.0 / SPEED
    bx, by = BLANK_XY[0] + TRIM["pick"][0], BLANK_XY[1] + TRIM["pick"][1]
    mx, my = MACHINE_XY[0] + TRIM["load"][0], MACHINE_XY[1] + TRIM["load"][1]
    tx, ty = TRAY_XY[0] + TRIM["tray"][0], TRAY_XY[1] + TRIM["tray"][1]
    return [
        # Load: the cube is lowered onto the machine's floor, not dropped, and the fingers open only to 60 mm inside.
        Waypoint(1.5 * k, (bx, by, TRAVEL_Z), FINGER_OPEN, "to the blank"),
        Waypoint(1.5 * k, (bx, by, GRIP_Z), FINGER_OPEN, "descend"),
        Waypoint(0.8 * k, (bx, by, GRIP_Z), FINGER_CLOSED, "close"),
        Waypoint(1.2 * k, (bx, by, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.8 * k, (mx, my, TRAVEL_Z), FINGER_CLOSED, "carry over the machine"),
        Waypoint(1.5 * k, (mx, my, PLACE_IN_Z), FINGER_CLOSED, "lower in"),
        Waypoint(0.6 * k, (mx, my, PLACE_IN_Z), FINGER_IN, "release"),
        Waypoint(1.2 * k, (mx, my, TRAVEL_Z), FINGER_IN, "retreat"),
        # Shut the guard, then park to the left, out of the top camera's view, while the right arm presses start.
        *_push(SHUT_START_Y, SHUT_END_Y, "push the guard shut"),
        Waypoint(PARK_S, LEFT_PARK, FINGER_CLOSED, "park"),
        Waypoint(CYCLE_WAIT_S, LEFT_PARK, FINGER_CLOSED, "wait for the cycle"),
        *_push(OPEN_START_Y, OPEN_END_Y, "push the guard open"),
        # Unload.
        Waypoint(1.5 * k, (mx, my, TRAVEL_Z), FINGER_IN, "over the part"),
        Waypoint(1.5 * k, (mx, my, GRIP_IN_Z), FINGER_IN, "down to the part"),
        Waypoint(0.8 * k, (mx, my, GRIP_IN_Z), FINGER_CLOSED, "close"),
        Waypoint(1.2 * k, (mx, my, TRAVEL_Z), FINGER_CLOSED, "lift out"),
        Waypoint(1.8 * k, (tx, ty, TRAVEL_Z), FINGER_CLOSED, "carry to the tray"),
        Waypoint(1.2 * k, (tx, ty, TRAY_PLACE_Z), FINGER_CLOSED, "lower"),
        Waypoint(0.6 * k, (tx, ty, TRAY_PLACE_Z), FINGER_OPEN, "release"),
        Waypoint(1.0 * k, (tx, ty, TRAVEL_Z), FINGER_OPEN, "retreat"),
    ]


def shut_done_at(left: list[Waypoint]) -> float:
    """Seconds into the demo when the left arm has lifted off the shut guard and starts for its park."""
    return sum(w.duration for w in left[:left.index(next(w for w in left if w.label == "park"))])


def _right_plan(start_after: float) -> list[Waypoint]:
    x, y = BUTTON_XY
    return [
        Waypoint(start_after, RIGHT_PARK, FINGER_CLOSED, "wait for the guard"),
        Waypoint(BUTTON_APPROACH_S, (x, y, BUTTON_ABOVE_Z), FINGER_CLOSED, "over the button"),
        Waypoint(PRESS_S, (x, y, PRESS_Z), FINGER_CLOSED, "press start"),
        Waypoint(PRESS_HOLD_S, (x, y, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(PRESS_LIFT_S, (x, y, BUTTON_ABOVE_Z), FINGER_CLOSED, "release"),
        Waypoint(PARK_S, RIGHT_PARK, FINGER_CLOSED, "park"),
    ]


def demo():
    """Left arm: the blank into the machine, the guard shut, then (after the cycle) the guard open and the part into
    the tray. Right arm: from its park, press start once the guard is shut, then park again."""
    left = _left_plan()
    return {"left": left, "right": _right_plan(shut_done_at(left))}
