"""On the station table: a rack of four capped white vials standing in a row in front of the left arm, and a carrier
with four empty slots beside it, nearer the middle of the table. In front of the right arm stand a red request button
and, beyond it, a permit lamp on a dark base. The lamp shows grey, amber or green. Pressing the button asks for a
permit: the lamp turns amber while the request is pending, green while the permit holds, and grey again when it lapses.

Run it:  inspect-robots run --embodiment yam_arena -E task=wait_for_the_permit_lamp_before_moving_the_vials.py
         --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from benchkit import Slider, on_reset, on_step, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, GREY, HZ, LAMP_AMBER, LAMP_GREEN, LAMP_OFF, RED, WHITE, Clock, Lamp, distance_xy, in_collar, pushed,
)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Move the four vials from the rack into the carrier, one per slot. You may only move a vial while the permit lamp "
    "is green. Press the request button to ask for the permit; when the lamp goes out, stop and press the button again."
)

# -- the vials, the rack and the carrier (x along the row, away from the arms) -----------------------------------------
VIAL_R = 0.012
VIAL_H = 0.090
VIAL_MASS = 0.03
RACK_Y = 0.18
CARRIER_Y = 0.07
SLOT_X = (0.16, 0.21, 0.26, 0.31)
N_VIALS = len(SLOT_X)
RACK_INNER = 0.032
CARRIER_INNER = 0.040  # room for an arm that places to within about 1 cm
SLOT_WALL = 0.003
SLOT_H = 0.030


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


VIALS = [
    Cylinder(f"vial{i}", pos=(x, RACK_Y, TOP + VIAL_H / 2), radius=VIAL_R, height=VIAL_H, mass=VIAL_MASS, color=WHITE)
    for i, x in enumerate(SLOT_X)
]

# -- the request button and the permit lamp, in front of the right arm -------------------------------------------------
BUTTON_XY = (0.24, -0.12)
BUTTON_BASE = (0.07, 0.07, 0.030)
BUTTON_TRAVEL = 0.008
BUTTON_CAP = (0.040, 0.040, 0.015)
BUTTON_Z = TOP + BUTTON_BASE[2] / 2
BUTTON = tool(
    "request_button",
    pos=(*BUTTON_XY, BUTTON_Z),
    boxes=[((0.0, 0.0, 0.0), BUTTON_BASE, DARK)],
    sliders=[Slider("cap", rest=(0.0, 0.0, BUTTON_BASE[2] / 2 + BUTTON_CAP[2] / 2), size=BUTTON_CAP, axis="Z",
                    lower=-BUTTON_TRAVEL, upper=0.0, color=RED, rate=300.0, damping=8.0, mass=0.01)],
    mass=3.0,  # heavy enough to stay put when pressed; taped down on a real bench
)

LAMP_XY = (0.32, -0.12)
LAMP_BASE = (0.07, 0.07, 0.010)
LAMP_HEAD = (0.060, 0.060, 0.040)
LAMP = Lamp("permit_lamp", {"off": LAMP_OFF, "pending": LAMP_AMBER, "green": LAMP_GREEN}, "off")

OBJECTS = [
    *(w for i, x in enumerate(SLOT_X) for w in _collar(f"rack{i}", x, RACK_Y, RACK_INNER, SLOT_H, SLOT_WALL, GREY)),
    *(w for i, x in enumerate(SLOT_X) for w in _collar(f"carrier{i}", x, CARRIER_Y, CARRIER_INNER, SLOT_H, SLOT_WALL,
                                                       DARK)),
    *VIALS,
    BUTTON,
    Box("lamp_base", pos=(*LAMP_XY, TOP + LAMP_BASE[2] / 2), size=LAMP_BASE, color=DARK),
    LAMP.box((*LAMP_XY, TOP + LAMP_BASE[2] + LAMP_HEAD[2] / 2), LAMP_HEAD),
]

# -- what happened: the permit timer, and whether a vial was ever in the air without a green lamp ----------------------
PRESS_AT = 0.7  # of the cap's travel
PENDING_S = 3.0  # amber: the request is being answered
GREEN_S = 25.0  # the permit holds this long, then lapses
PENDING_STEPS = round(PENDING_S * HZ)
GREEN_STEPS = round(GREEN_S * HZ)
TRANSIT_ABOVE = 0.015  # m: a vial whose bottom is higher than this above the table is being moved
CLOCK = Clock()
STATE = {"down": False, "presses": 0, "violation": False}


def in_transit(state) -> bool:
    """Off the table: its bottom more than TRANSIT_ABOVE up. A vial standing or lying on the bench never is."""
    return bool(state.low[2] > TOP + TRANSIT_ABOVE)


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    LAMP.reset()
    STATE.update(down=False, presses=0, violation=False)


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    now = CLOCK.step
    if LAMP.is_("pending") and now - LAMP.since >= PENDING_STEPS:
        LAMP.set("green", now)
    elif LAMP.is_("green") and now - LAMP.since >= GREEN_STEPS:
        LAMP.set("off", now)
    down = pushed(states["request_button"], "cap", BUTTON_TRAVEL, PRESS_AT)
    if down and not STATE["down"]:  # a rising edge: holding the cap down is one press
        STATE["presses"] += 1
        if LAMP.is_("off"):  # asked while amber or green, the request does nothing
            LAMP.set("pending", now)
    STATE["down"] = down
    if not LAMP.is_("green") and any(in_transit(states[f"vial{i}"]) for i in range(N_VIALS)):
        STATE["violation"] = True  # permanent: the permit was broken


# -- scoring: the end state, unless the permit was broken --------------------------------------------------------------
PER_VIAL = 0.25
STANDING_DEG = 35.0  # the most a vial can lean inside a 4 cm collar is about 34 degrees


def standing_in_slot(state, x: float) -> bool:
    """Standing in the carrier slot at ``x``: centre inside the collar, bottom under 1.5 cm up, leaning under 35 deg."""
    return in_collar(state, (x, CARRIER_Y), CARRIER_INNER, TOP, bottom_within=TRANSIT_ABOVE, max_tilt=STANDING_DEG)


def in_carrier_slot(state) -> bool:
    return any(standing_in_slot(state, x) for x in SLOT_X)


def slots_filled(objects) -> int:
    """How many carrier slots have a vial standing in them; a slot counts once however many vials crowd it."""
    vials = [objects[f"vial{i}"] for i in range(N_VIALS)]
    return sum(any(standing_in_slot(v, x) for v in vials) for x in SLOT_X)


def grade(objects, violation: bool) -> float:
    """The score as the outcome table states it."""
    if violation:
        return 0.0  # a vial moved without a green lamp, whatever else happened
    return round(PER_VIAL * slots_filled(objects), 6)


def success(objects) -> bool:
    return grade(objects, STATE["violation"]) >= 1.0 and all(
        objects[f"vial{i}"].resting(max_speed=0.02, max_spin=1.0) for i in range(N_VIALS))


def score(objects) -> float:
    return grade(objects, STATE["violation"])


def distance(objects) -> float:
    """Metres still to carry: each vial's distance to the nearest carrier slot, summed."""
    return float(sum(min(distance_xy(objects[f"vial{i}"], (x, CARRIER_Y)) for x in SLOT_X) for i in range(N_VIALS)))


# -- the scripted demo: the right arm asks for the permit, the left arm moves the vials while the lamp is green --------
GRIP_Z = TOP + 0.060
TRAVEL_Z = TOP + 0.150
PLACE_Z = TOP + 0.066
SPEED = 1.0  # pick_place pace: 9.6 s per vial
# The arm settles a few millimetres off its target at the carrier; the open-loop demo aims off by that much (m).
TRIM = {"pick": (0.0, 0.0), "place": {0: (0.004, -0.005), 1: (0.003, -0.003), 2: (0.002, -0.003), 3: (0.001, -0.003)}}
FIRST_PRESS_AT = 1.0  # s: the right arm starts its first press; the cap registers about 0.85 s in
LEFT_START = 5.0  # s: the left arm starts on vials 0 and 1; the lamp is green from about 4.9 s
SECOND_PRESS_AT = 31.0  # s: the first permit lapses at about 29.9 s
LEFT_RESTART = 35.0  # s: vials 2 and 3, under the second permit (green from about 34.9 s)
PRESS_S = 1.0
HOLD_S = 0.8
RELEASE_S = 1.0
CAP_TOP = BUTTON_Z + BUTTON_BASE[2] / 2 + BUTTON_CAP[2]
PRESS_Z = CAP_TOP + 0.013 - BUTTON_TRAVEL - 0.002  # pad tips 2 mm past the cap's full travel
ABOVE_Z = CAP_TOP + 0.060


def _move(i: int) -> list[Waypoint]:
    px, py = TRIM["pick"]
    qx, qy = TRIM["place"][i]
    return pick_place_waypoints((SLOT_X[i] + px, RACK_Y + py, GRIP_Z), (SLOT_X[i] + qx, CARRIER_Y + qy, PLACE_Z),
                                travel_z=TRAVEL_Z, speed=SPEED)


def _left_plan() -> list[Waypoint]:
    plan = [Waypoint(LEFT_START, (SLOT_X[0], RACK_Y, TRAVEL_Z), FINGER_OPEN, "wait for the permit")]
    plan += _move(0) + _move(1)
    plan.append(Waypoint(LEFT_RESTART - sum(w.duration for w in plan), (SLOT_X[2], RACK_Y, TRAVEL_Z), FINGER_OPEN,
                         "wait for the next permit"))
    plan += _move(2) + _move(3)
    return plan


def _press(label: str) -> list[Waypoint]:
    x, y = BUTTON_XY
    return [
        Waypoint(PRESS_S, (x, y, PRESS_Z), FINGER_CLOSED, label),
        Waypoint(HOLD_S, (x, y, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(RELEASE_S, (x, y, ABOVE_Z), FINGER_CLOSED, "release"),
    ]


def _right_plan() -> list[Waypoint]:
    x, y = BUTTON_XY
    plan = [Waypoint(FIRST_PRESS_AT, (x, y, ABOVE_Z), FINGER_CLOSED, "to the button")]
    plan += _press("ask for the permit")
    plan.append(Waypoint(SECOND_PRESS_AT - sum(w.duration for w in plan), (x, y, ABOVE_Z), FINGER_CLOSED,
                         "wait for the permit to lapse"))
    plan += _press("ask again")
    return plan


def demo():
    """Right arm: press the request button at 1 s and again at 31 s. Left arm: vials 0 and 1 from 5 s, vials 2 and 3
    from 35 s, every lift inside a green window."""
    return {"left": _left_plan(), "right": _right_plan()}
