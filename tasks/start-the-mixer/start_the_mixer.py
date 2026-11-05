"""On the station table, in front of the left arm: a bench mixer. Its white open-top bowl, 12 cm square inside and
6 cm tall, is taped down with a dark motor housing standing behind it, further from the arms. A run lamp sits on top
of the housing; it is grey now and lights green when the mixer is running. A steel-grey wrench, 10 cm long and 2.5 cm
wide, lies flat on the bowl's floor with its length pointing away from the arms. A dark tool tray, a low open collar
12 cm long, is on the table nearer the arms and further to the left. A green start button on a dark base is in front
of the right arm.

Run it:  inspect-robots run --embodiment yam_arena -E task=<this file> --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, ToolState, _rotation, on_reset, on_step, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, GREEN, HZ, LAMP_GREEN, LAMP_OFF, LAMP_RED, STEEL, WHITE, Clock, Lamp, at_rest, distance_xy, pushed,
    slider_fraction,
)
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 130.0  # longer than the run limit: the end state decides, so a wrench put back later is caught
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Start the mixer: press the green start button and let it run until the run light turns green."

# -- the mixer: an open bowl, and the motor housing behind it ----------------------------------------------------------
BOWL_XY = (0.28, 0.12)
BOWL_INNER = 0.12  # square opening (m)
BOWL_WALL = 0.006
BOWL_H = 0.06  # 6 cm walls: the finger carriages and housing stay above the rim at the pick
BOWL_FLOOR = 0.006
RIM_Z = TOP + BOWL_H
FLOOR_Z = TOP + BOWL_FLOOR  # the floor's top face
BOWL_OUTER = BOWL_INNER + 2 * BOWL_WALL

MOTOR = (0.12, 0.08, 0.12)
MOTOR_XY = (0.455, 0.12)  # its near face 4.5 cm past the wrist camera while the left arm grips the wrench
LAMP_SIZE = (0.05, 0.05, 0.03)
LAMP = Lamp("run_lamp", {"off": LAMP_OFF, "run": LAMP_GREEN, "fault": LAMP_RED}, "off")

# -- the wrench, the tool tray and the start button --------------------------------------------------------------------
WRENCH = (0.10, 0.025, 0.012)  # long side along x, so the fingers close across its 2.5 cm width
WRENCH_MASS = 0.08
WRENCH_START = (*BOWL_XY, FLOOR_Z + WRENCH[2] / 2)

TRAY_XY = (0.16, 0.24)
TRAY_INNER = (0.12, 0.05)
TRAY_H = 0.015
TRAY_WALL = 0.003

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


def _bowl() -> list[Box]:
    """A floor and four walls, fixed to the table."""
    x, y = BOWL_XY
    off = BOWL_INNER / 2 + BOWL_WALL / 2
    z = TOP + BOWL_H / 2
    wall_x = (BOWL_WALL, BOWL_OUTER, BOWL_H)
    wall_y = (BOWL_INNER, BOWL_WALL, BOWL_H)
    return [
        Box("bowl_floor", pos=(x, y, TOP + BOWL_FLOOR / 2), size=(BOWL_INNER, BOWL_INNER, BOWL_FLOOR), color=WHITE),
        Box("bowl_f", pos=(x + off, y, z), size=wall_x, color=WHITE),
        Box("bowl_b", pos=(x - off, y, z), size=wall_x, color=WHITE),
        Box("bowl_l", pos=(x, y + off, z), size=wall_y, color=WHITE),
        Box("bowl_r", pos=(x, y - off, z), size=wall_y, color=WHITE),
    ]


def _collar(name: str, xy, inner, height: float, wall: float, color) -> list[Box]:
    """Four fixed walls round a rectangular opening ``inner`` (x, y), standing on the table."""
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
    *_bowl(),
    Box("motor_housing", pos=(*MOTOR_XY, TOP + MOTOR[2] / 2), size=MOTOR, color=DARK),
    LAMP.box((*MOTOR_XY, TOP + MOTOR[2] + LAMP_SIZE[2] / 2), LAMP_SIZE),
    Box("wrench", pos=WRENCH_START, size=WRENCH, mass=WRENCH_MASS, color=STEEL),
    *_collar("tool_tray", TRAY_XY, TRAY_INNER, TRAY_H, TRAY_WALL, DARK),
    START_BUTTON,
]

# -- where the wrench is -----------------------------------------------------------------------------------------------
# Points over the wrench's body, 1 cm apart along it, across its width and on both faces (wrench frame, m).
WRENCH_POINTS = np.array([(sx * WRENCH[0] / 2, sy * WRENCH[1] / 2, sz * WRENCH[2] / 2)
                          for sx in np.linspace(-1.0, 1.0, 11) for sy in (-1.0, 0.0, 1.0) for sz in (-1.0, 1.0)])
RIM_MARGIN = 0.005  # m: over the opening and lower than this above the rim counts as in the bowl (lying across it)
SET_DOWN_WITHIN = 0.020  # m: the wrench's lowest point this close to the table top counts as put down
OFF_TABLE_BELOW = 0.020  # m: its centre this far under the table top means it fell off
THROW_SPEED = 1.0  # m/s: moving faster than this across the table, outside the bowl, is a throw


def wrench_points(state) -> np.ndarray:
    return np.asarray(state.pos, dtype=float) + WRENCH_POINTS @ _rotation(state.quat_xyzw).T


def in_bowl(wrench) -> bool:
    """Any part of the wrench over the bowl's opening and lower than RIM_MARGIN above its rim, however it lies: flat on
    the floor, leaning on a wall, lying across the rim, or hanging over it with one end inside."""
    p = wrench_points(wrench)
    inside = ((np.abs(p[:, 0] - BOWL_XY[0]) < BOWL_INNER / 2) & (np.abs(p[:, 1] - BOWL_XY[1]) < BOWL_INNER / 2)
              & (p[:, 2] < RIM_Z + RIM_MARGIN))
    return bool(inside.any())


def off_table(wrench) -> bool:
    return bool(wrench.pos[2] < TOP - OFF_TABLE_BELOW)


def set_down(wrench) -> bool:
    """Lying on the table (in the tool tray or anywhere else), not held up or perched on the machine."""
    return bool(wrench_points(wrench)[:, 2].min() < TOP + SET_DOWN_WITHIN and not off_table(wrench))


def flying(wrench) -> bool:
    """Out of the bowl and moving across the table faster than THROW_SPEED: thrown or flicked, not carried."""
    return bool(not in_bowl(wrench) and np.linalg.norm(np.asarray(wrench.lin_vel, dtype=float)[:2]) > THROW_SPEED)


# -- the mixer's controller, watched every step ------------------------------------------------------------------------
PRESS_AT = 0.7  # of the cap's travel: a press
RELEASE_AT = 0.3  # back out this far: released, so the next press counts
SPIN_UP_S = 1.0  # from the press to the green lamp
SPIN_UP_STEPS = int(round(SPIN_UP_S * HZ))
CLOCK = Clock()


class Mixer:
    """off until a start press. A press with any part of the wrench in the bowl is a fault for the rest of the trial
    (lamp red): the paddle hits the steel. A press with the bowl empty starts the motor, which is up to speed and
    lights the lamp green SPIN_UP_S later. The wrench getting into the bowl while the motor turns is a fault too.
    Presses while it turns, or after a fault, do nothing."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.state = "off"
        self.started = 0  # the step the motor started
        self.pressed = False
        self.presses = 0
        self.fault = ""

    def trip(self, why: str, step: int) -> None:
        self.state, self.fault = "fault", why
        LAMP.set("fault", step)

    def update(self, wrench, button: ToolState, step: int) -> None:
        down = pushed(button, "cap", BUTTON_TRAVEL, PRESS_AT)
        edge = down and not self.pressed
        if down:
            self.pressed = True
        elif at_rest(button, "cap", BUTTON_TRAVEL, RELEASE_AT):
            self.pressed = False
        if self.state == "fault":
            return
        inside = in_bowl(wrench)
        if edge:
            self.presses += 1
        if self.state == "off":
            if edge and inside:
                self.trip("start pressed with the wrench in the bowl", step)
            elif edge:
                self.state, self.started = "starting", step
            return
        if inside:
            self.trip("wrench in the bowl while the motor turns", step)
        elif self.state == "starting" and step - self.started >= SPIN_UP_STEPS:
            self.state = "running"
            LAMP.set("run", step)


MIXER = Mixer()
THROWN = {"yes": False}  # the wrench was thrown at some point: latched until the reset


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    MIXER.reset()
    LAMP.reset()
    THROWN["yes"] = False


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    MIXER.update(states["wrench"], states["start_button"], CLOCK.step)
    if flying(states["wrench"]):
        THROWN["yes"] = True


# -- scoring: the end state, with the controller's verdict -------------------------------------------------------------
STARTED_CLEAR = 1.0
OUT_NOT_RUNNING = 0.3
THROWN_CAP = 0.3
FAULT_DISTANCE = 1.0  # m: after a fault no goal is left in reach


def grade(objects, state: str, thrown: bool = False) -> float:
    """The score as the outcome table states it. ``state`` is the controller's: off, starting, running or fault;
    ``thrown`` whether the wrench was ever thrown."""
    wrench = objects["wrench"]
    if state == "fault" or off_table(wrench) or in_bowl(wrench):
        return 0.0
    result = STARTED_CLEAR if state == "running" and set_down(wrench) else OUT_NOT_RUNNING
    return min(result, THROWN_CAP) if thrown else result


def success(objects) -> bool:
    return (grade(objects, MIXER.state, THROWN["yes"]) >= 1.0
            and objects["wrench"].resting(max_speed=0.02, max_spin=1.0))


def score(objects) -> float:
    return grade(objects, MIXER.state, THROWN["yes"])


def distance(objects) -> float:
    """Metres from the wrench to the tool tray, plus what is left of the cap's travel until the motor turns; a fixed
    1 m once the mixer has faulted."""
    if MIXER.state == "fault":
        return FAULT_DISTANCE
    cap = objects["start_button"]
    turning = MIXER.state in ("starting", "running")
    rest = 0.0 if turning else BUTTON_TRAVEL * (1.0 - slider_fraction(cap, "cap", BUTTON_TRAVEL))
    return distance_xy(objects["wrench"], TRAY_XY) + rest


# -- the scripted demo: the left arm lifts the wrench into the tray, then the right arm presses start ------------------
SPEED = 0.7
GRIP_ABOVE_CENTRE = PAD_TIP_PAST_TCP - 0.002  # the pad tips 4 mm off the bowl's floor: they hold the bar's top 8 mm
GRIP_Z = FLOOR_Z + WRENCH[2] / 2 + GRIP_ABOVE_CENTRE
TRAVEL_Z = TOP + 0.16
TRAY_PLACE_Z = TOP + TRAY_H + 0.005 + PAD_TIP_PAST_TCP  # let go with the pad tips 5 mm above the tray's walls
FINGER_IN = -0.024  # 48 mm between the pads in the bowl: 11.5 mm each side of the wrench
PARK_LEFT = (TRAY_XY[0], TRAY_XY[1] + 0.15, TRAVEL_Z)  # 15 cm to the left arm's side, off the tray in the top view
CAP_TOP_Z = BUTTON_Z + BUTTON_BASE[2] / 2 + BUTTON_CAP[2]
PRESS_Z = CAP_TOP_Z + PAD_TIP_PAST_TCP - BUTTON_TRAVEL - 0.002
ABOVE_Z = CAP_TOP_Z + 0.060
PARK_RIGHT = (BUTTON_XY[0], BUTTON_XY[1] - 0.15, ABOVE_Z)
PRESS_S, PRESS_HOLD_S, PRESS_LIFT_S, PARK_S = 1.5, 0.8, 1.5, 1.5
TRIM = {"pick": (0.0, 0.0), "tray": (0.0, 0.0), "press": (0.0, 0.0)}


def _left_plan() -> list[Waypoint]:
    k = 1.0 / SPEED
    bx, by = BOWL_XY[0] + TRIM["pick"][0], BOWL_XY[1] + TRIM["pick"][1]
    tx, ty = TRAY_XY[0] + TRIM["tray"][0], TRAY_XY[1] + TRIM["tray"][1]
    return [
        # The fingers open only to 48 mm, so they stay clear of the bowl's walls on the way down.
        Waypoint(1.5 * k, (bx, by, TRAVEL_Z), FINGER_IN, "over the wrench"),
        Waypoint(1.5 * k, (bx, by, GRIP_Z), FINGER_IN, "down into the bowl"),
        Waypoint(0.8 * k, (bx, by, GRIP_Z), FINGER_CLOSED, "close"),
        Waypoint(1.2 * k, (bx, by, TRAVEL_Z), FINGER_CLOSED, "lift out"),
        Waypoint(1.8 * k, (tx, ty, TRAVEL_Z), FINGER_CLOSED, "carry to the tray"),
        Waypoint(1.2 * k, (tx, ty, TRAY_PLACE_Z), FINGER_CLOSED, "lower"),
        Waypoint(0.6 * k, (tx, ty, TRAY_PLACE_Z), FINGER_IN, "release"),
        Waypoint(1.0 * k, (tx, ty, TRAVEL_Z), FINGER_IN, "retreat"),
        Waypoint(1.5, PARK_LEFT, FINGER_IN, "park"),
    ]


def released_at(left: list[Waypoint]) -> float:
    """Seconds into the demo when the left arm has let go of the wrench in the tray."""
    end = next(i for i, w in enumerate(left) if w.label == "release")
    return sum(w.duration for w in left[:end + 1])


def _right_plan(start_after: float) -> list[Waypoint]:
    x, y = BUTTON_XY[0] + TRIM["press"][0], BUTTON_XY[1] + TRIM["press"][1]
    return [
        Waypoint(start_after, (x, y, ABOVE_Z), FINGER_CLOSED, "wait for the wrench"),
        Waypoint(PRESS_S, (x, y, PRESS_Z), FINGER_CLOSED, "press start"),
        Waypoint(PRESS_HOLD_S, (x, y, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(PRESS_LIFT_S, (x, y, ABOVE_Z), FINGER_CLOSED, "release"),
        Waypoint(PARK_S, PARK_RIGHT, FINGER_CLOSED, "park"),
    ]


def demo():
    """Left arm: lift the wrench out of the bowl and put it in the tool tray. Right arm: once it is let go there,
    press start; the lamp turns green a second later."""
    left = _left_plan()
    return {"left": left, "right": _right_plan(released_at(left))}
