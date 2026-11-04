"""On the station table: a small press. Its dark steel frame stands on the midline at the back, with a square cycle
lamp on top. A steel die plate lies on the table in front of the frame, and a blue part stands on the plate's centre.
Two green push buttons stand in front of the arms, one on the left and one on the right, 44 cm apart. A yellow output
tray sits in front of the left arm, between the left button and the die plate, and a spare steel-grey die block stands
close in front of the left arm, beside the left button.

The press runs one cycle while both buttons are held down together. The cycle lamp is grey until a cycle is complete,
then green, and the formed part turns orange.

Run it:  inspect-robots run --embodiment yam_arena -E task=two_hand_press_cycle.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, ToolState, on_reset, on_step, recolor, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, GREEN, GREY, HZ, LAMP_GREEN, LAMP_OFF, STEEL, YELLOW, Clock, Lamp, distance_xy, over, pushed,
)
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Run the press: hold both green buttons down at the same time until the cycle light turns green, then take the "
    "finished part out of the die and put it in the output tray."
)

BLUE = (0.20, 0.35, 0.65)
FORMED = (0.92, 0.45, 0.08)  # the part once the press has formed it

# -- the press: frame, cycle lamp, die plate and the part standing on it -------------------------------------------
# The frame stands 6 cm behind the die plate: pointing down over the part, the gripper body reaches 3.3 cm and the
# wrist camera 7 cm towards the frame, so the body clears the frame's face by 7.8 cm and the camera by 4 cm.
FRAME_XY = (0.45, 0.0)
FRAME_SIZE = (0.12, 0.14, 0.16)
LAMP_SIZE = (0.06, 0.06, 0.03)
DIE_XY = (0.28, 0.0)
DIE_SIZE = (0.10, 0.10, 0.006)
DIE_TOP = TOP + DIE_SIZE[2]
PART = 0.040  # the part and the spare block are 4 cm cubes
PART_MASS = 0.05  # kg

LAMP = Lamp("cycle_lamp", {"off": LAMP_OFF, "done": LAMP_GREEN}, "off")

# -- the two push buttons, 44 cm apart: an open gripper spans 9.4 cm, so one gripper can never press both -----------
BUTTON_X = 0.22  # m in front of the arm bases
BUTTON_Y = 0.22  # each side of the midline
BUTTON_BASE = (0.07, 0.07, 0.030)
BUTTON_CAP = (0.040, 0.040, 0.015)
BUTTON_TRAVEL = 0.008  # m: the cap's full travel into its base
BUTTON_Z = TOP + BUTTON_BASE[2] / 2
CAP_REST = (0.0, 0.0, BUTTON_BASE[2] / 2 + BUTTON_CAP[2] / 2)  # the cap's centre at rest (tool frame)
CAP_SEAT = (0.0, 0.0, BUTTON_BASE[2] / 2)  # the base's top, which the cap sits on (tool frame)
CAP_TOP_Z = TOP + BUTTON_BASE[2] + BUTTON_CAP[2]  # the cap's top at rest (world)


def _button(name: str, y: float):
    return tool(
        name,
        pos=(BUTTON_X, y, BUTTON_Z),
        boxes=[((0.0, 0.0, 0.0), BUTTON_BASE, DARK)],
        sliders=[Slider("cap", rest=CAP_REST, size=BUTTON_CAP, axis="Z", lower=-BUTTON_TRAVEL, upper=0.0, color=GREEN,
                        rate=300.0, damping=8.0, mass=0.01)],
        mass=3.0,  # heavy enough to stay put when pressed; taped down on a real bench
    )


BUTTONS = {"left_button": _button("left_button", BUTTON_Y), "right_button": _button("right_button", -BUTTON_Y)}

# -- the output tray and the spare die block -------------------------------------------------------------------------
TRAY_XY = (0.18, 0.12)
TRAY_INNER = 0.080
TRAY_WALL = 0.003
TRAY_H = 0.020
SPARE_XY = (0.14, 0.26)
SPARE_MASS = 0.20  # on a cap its weight is 6.5 mm of the spring's 8 mm: it holds a button down by itself
LOOSE = ("part", "spare_block", *BUTTONS)


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
    Box("press_frame", pos=(*FRAME_XY, TOP + FRAME_SIZE[2] / 2), size=FRAME_SIZE, color=DARK),
    LAMP.box((*FRAME_XY, TOP + FRAME_SIZE[2] + LAMP_SIZE[2] / 2), LAMP_SIZE),
    Box("die_plate", pos=(*DIE_XY, TOP + DIE_SIZE[2] / 2), size=DIE_SIZE, color=STEEL),
    Box("part", pos=(*DIE_XY, DIE_TOP + PART / 2), size=(PART,) * 3, mass=PART_MASS, color=BLUE),
    *_collar("tray", *TRAY_XY, TRAY_INNER, TRAY_H, TRAY_WALL, YELLOW),
    *BUTTONS.values(),
    Box("spare_block", pos=(*SPARE_XY, TOP + PART / 2), size=(PART,) * 3, mass=SPARE_MASS, color=GREY),
]

# -- what happened, step by step: the cycle timer, a reach into the die, a prop on a button --------------------------
CYCLE_S = 3.0  # both caps held in this long completes a cycle
CYCLE_STEPS = round(CYCLE_S * HZ)
PUSHED_AT = 0.7  # of the cap's travel: 5.6 mm in counts as pressed
DIE_WITHIN = 0.020  # m: the part's centre this close to the die centre is in the die
DIE_LIFT = 0.010  # m: the part's bottom this far above the plate is out of it
PROP_WITHIN = 0.020  # m: an object's centre this close to a pushed cap's centre, resting above the cap's seat, props it
PROP_ABOVE = 0.002  # m above the seat, so an object on the base's ledge beside the cap does not count


class Press:
    """What the press has seen this trial. ``held`` counts consecutive steps with both caps pushed; the flags latch."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.held = 0
        self.done = False
        self.done_at = None  # the step the cycle completed
        self.reached_in = False  # the part left the die while the buttons were held, before the cycle completed
        self.propped = False  # a loose object held a cap down


PRESS = Press()
CLOCK = Clock()


def cap_centre(button: ToolState) -> np.ndarray:
    """The cap's centre in the world, wherever the button has been pushed to."""
    return button.to_world((CAP_REST[0], CAP_REST[1], CAP_REST[2] + button.sliders["cap"]))


def in_die(part) -> bool:
    return bool(distance_xy(part, DIE_XY) <= DIE_WITHIN and part.low[2] <= DIE_TOP + DIE_LIFT)


def propped(states, name: str) -> bool:
    """Something other than a gripper holds ``name``'s cap in: a loose object sits over the pushed cap, above
    its seat."""
    button = states[name]
    if not pushed(button, "cap", BUTTON_TRAVEL, PUSHED_AT):
        return False
    centre, seat = cap_centre(button), button.to_world(CAP_SEAT)
    return any(distance_xy(states[n], centre[:2]) <= PROP_WITHIN and states[n].low[2] > seat[2] + PROP_ABOVE
               for n in LOOSE if n != name)


@on_reset
def _reset(states) -> None:
    PRESS.reset()
    CLOCK.reset()
    LAMP.reset()
    recolor("part", BLUE)


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    both = all(pushed(states[n], "cap", BUTTON_TRAVEL, PUSHED_AT) for n in BUTTONS)
    PRESS.held = PRESS.held + 1 if both else 0
    if PRESS.held and not PRESS.done:
        if not in_die(states["part"]):
            PRESS.reached_in = True  # the part moved while the ram was coming down
        if PRESS.held >= CYCLE_STEPS:
            PRESS.done, PRESS.done_at = True, CLOCK.step
            LAMP.set("done", CLOCK.step)
            recolor("part", FORMED)
    if any(propped(states, n) for n in BUTTONS):
        PRESS.propped = True


# -- scoring -------------------------------------------------------------------------------------------------------
CYCLE_ONLY = 0.4  # the cycle run safely, the part not in the tray
IN_TRAY_BELOW = TOP + 0.015  # the part's bottom under this is down in the tray, not on its wall


def in_tray(part) -> bool:
    """Resting in the output tray: centre inside the collar, bottom on or near its floor."""
    return bool(over(part, TRAY_XY, (TRAY_INNER / 2, TRAY_INNER / 2)) and part.low[2] < IN_TRAY_BELOW)


def grade(objects, press: Press) -> float:
    """The score as the outcome table states it."""
    if not press.done or press.reached_in or press.propped:
        return 0.0
    return 1.0 if in_tray(objects["part"]) else CYCLE_ONLY


def success(objects) -> bool:
    return grade(objects, PRESS) >= 1.0 and objects["part"].resting(max_speed=0.02, max_spin=1.0)


def score(objects) -> float:
    return grade(objects, PRESS)


DIE_TO_TRAY = float(np.linalg.norm(np.subtract(TRAY_XY, DIE_XY)))  # the formed part's run, 15.6 cm


def distance(objects) -> float:
    """Metres to go. Until the cycle is done: the part's distance from the die centre (it belongs in the die), what
    is left of both caps' travel to the pressed point, and the die-to-tray run. After it: the part's distance from
    the tray."""
    part = objects["part"]
    if PRESS.done:
        return distance_xy(part, TRAY_XY)
    to_press = sum(max(PUSHED_AT * BUTTON_TRAVEL - abs(objects[n].sliders["cap"]), 0.0) for n in BUTTONS)
    return distance_xy(part, DIE_XY) + to_press + DIE_TO_TRAY


# -- the scripted demo: both arms press together, then the left arm moves the part -----------------------------------
PRESS_Z = CAP_TOP_Z + PAD_TIP_PAST_TCP - BUTTON_TRAVEL - 0.002  # the pad tips 2 mm past the cap's full travel
ABOVE_Z = CAP_TOP_Z + 0.060
TO_BUTTON_S, DESCEND_S, HOLD_S, LIFT_S = 1.5, 1.5, 3.6, 1.5
LIFT_OFF_S = TO_BUTTON_S + DESCEND_S + HOLD_S  # when both arms begin to lift off their buttons
GRIP_Z = DIE_TOP + 0.024
TRAVEL_Z = TOP + 0.150
PLACE_Z = TOP + 0.040  # the pad tips 7 mm above the tray's rim when the fingers open; the part drops 1.6 cm
TRIM = {"pick": (0.0, 0.0), "place": (0.0, 0.0)}


def _press_plan(y: float) -> list[Waypoint]:
    x = BUTTON_X
    return [
        Waypoint(TO_BUTTON_S, (x, y, ABOVE_Z), FINGER_CLOSED, "over the button"),
        Waypoint(DESCEND_S, (x, y, PRESS_Z), FINGER_CLOSED, "press"),
        Waypoint(HOLD_S, (x, y, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(LIFT_S, (x, y, ABOVE_Z), FINGER_CLOSED, "lift off"),
    ]


def _left_plan() -> list[Waypoint]:
    px, py = TRIM["pick"]
    qx, qy = TRIM["place"]
    return _press_plan(BUTTON_Y) + pick_place_waypoints(
        (DIE_XY[0] + px, DIE_XY[1] + py, GRIP_Z), (TRAY_XY[0] + qx, TRAY_XY[1] + qy, PLACE_Z),
        travel_z=TRAVEL_Z, speed=0.7)


def demo():
    """Both arms: down onto their buttons together, hold through the cycle, lift off. Then the left arm takes the
    part from the die to the tray; the right arm stays over its button."""
    return {"left": _left_plan(), "right": _press_plan(-BUTTON_Y)}
