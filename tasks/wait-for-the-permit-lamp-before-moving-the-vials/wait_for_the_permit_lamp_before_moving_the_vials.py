"""On the station table: a rack of four capped white vials standing in a row in front of the left arm, and a carrier
with four empty slots beside it, nearer the middle of the table. In front of the right arm stands a red request button,
and beyond it, out of either arm's reach, a permit lamp stands on a dark base. The lamp shows grey, amber or green.
Pressing the button asks for a permit: the lamp turns amber while the request is pending, green while the permit holds,
and grey again when it lapses.

Run it:  inspect-robots run --embodiment yam_arena -E task=wait_for_the_permit_lamp_before_moving_the_vials.py
         --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys
from collections import deque

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
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

# Beyond the button and out of either arm's reach (0.64 m from the right base): from there the top camera's lines of
# sight to the lamp pass over the right arm's wrist and beside the left gripper, whatever either arm does in the demo.
LAMP_XY = (0.62, -0.14)
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

# -- what happened: the permit timer, and each vial's transits against it ----------------------------------------------
PRESS_AT = 0.7  # of the cap's travel
PENDING_S = 3.0  # amber: the request is being answered
GREEN_S = 25.0  # the permit holds this long, then lapses
GRACE_S = 3.0  # a vial in transit and not yet set down this long after the permit lapses is a fault
REST_S = 0.5  # a vial set on top of a fixture, moving under MOVE_M this long, has been put down there
PENDING_STEPS = round(PENDING_S * HZ)
GREEN_STEPS = round(GREEN_S * HZ)
GRACE_STEPS = round(GRACE_S * HZ)
REST_STEPS = round(REST_S * HZ)
TRANSIT_ABOVE = 0.015  # m: a vial is off the bench when its bottom is higher than this above the table
MOVE_M = 0.005  # m: a vial resting on top of something that moves more than this from where it rested is in transit
ON_TOP_M = 0.004  # m: a vial whose bottom is within this of a top face, and over it, rests on that top
CLOCK = Clock()
STATE = {"down": False, "presses": 0, "lapsed_at": None, "violation": False}


def raised(state) -> bool:
    """Off the bench: its bottom more than TRANSIT_ABOVE up. A vial standing or lying on the table never is."""
    return bool(state.low[2] > TOP + TRANSIT_ABOVE)


def _face(centre, size) -> tuple[float, float, float, float, float]:
    """A level top face (x0, y0, x1, y1, z) of ``size`` (x, y) centred on ``centre``."""
    x, y, z = (float(v) for v in centre)
    return x - size[0] / 2, y - size[1] / 2, x + size[0] / 2, y + size[1] / 2, z


# The top faces of the fixed boxes: the collar walls, the lamp and its base.
FIXED_TOPS = [_face((o.pos[0], o.pos[1], o.pos[2] + o.size[2] / 2), o.size)
              for o in OBJECTS if isinstance(o, Box) and o.mass is None]


def tops(states, skip: str) -> list[tuple[float, float, float, float, float]]:
    """Everything a vial could be resting on above the bench: the fixed boxes, the button's body and its cap (from the
    button's pose and the cap's travel), and the tops of the other vials."""
    b = states["request_button"]
    cap = BUTTON_BASE[2] / 2 + BUTTON_CAP[2] + b.sliders.get("cap", 0.0)
    out = [*FIXED_TOPS, _face(b.to_world((0.0, 0.0, BUTTON_BASE[2] / 2)), BUTTON_BASE),
           _face(b.to_world((0.0, 0.0, cap)), BUTTON_CAP)]
    out += [(v.low[0], v.low[1], v.high[0], v.high[1], v.high[2])
            for name, v in states.items() if name.startswith("vial") and name != skip]
    return out


def on_top(state, faces) -> bool:
    """Resting on one of ``faces``: its bottom within ON_TOP_M of the face and its footprint over it."""
    lo, hi = state.low, state.high
    return any(abs(lo[2] - z) <= ON_TOP_M and lo[0] < x1 and hi[0] > x0 and lo[1] < y1 and hi[1] > y0
               for x0, y0, x1, y1, z in faces)


class Transit:
    """One vial's transits. A transit starts on the first step a vial that stood or lay on the bench is off it
    (its bottom more than 1.5 cm up), or when a vial resting on top of something (a collar, the button, the lamp,
    another vial) has moved more than MOVE_M from where it rested. It ends when the vial is back on the bench, or
    when it has rested on a top, moving under MOVE_M, for REST_S. Held in the air, moving or still, it is in transit."""

    def __init__(self) -> None:
        self.carrying = False
        self.on_bench = None  # unknown until the first step, or the reset states, show where it is
        self.pos = None
        self.anchor = None  # where it rests on a top, off the bench
        self.landed = None  # where, in transit, it touched down on the top it is on now
        self.trail: deque = deque(maxlen=REST_STEPS + 1)  # (position, on a top) over the last REST_S

    def seed(self, state) -> None:
        """Where it stands at the reset."""
        self.pos = np.asarray(state.pos, dtype=float)
        self.on_bench = not raised(state)
        self.anchor = None if self.on_bench else self.pos

    @property
    def unsettled(self) -> bool:
        """In transit and not set down: in the air, or more than MOVE_M along a top from where it touched down."""
        return self.carrying and (self.landed is None or np.linalg.norm(self.pos - self.landed) > MOVE_M)

    def update(self, state, faces) -> bool:
        """One control step; True when a transit starts now."""
        pos = self.pos = np.asarray(state.pos, dtype=float)
        was_on_bench, self.on_bench = self.on_bench, not raised(state)
        top = not self.on_bench and on_top(state, faces)
        self.trail.append((pos, top))
        if self.on_bench:
            self.carrying, self.anchor, self.landed = False, None, None
            return False
        if self.carrying:
            self.landed = (pos if self.landed is None else self.landed) if top else None
            if len(self.trail) > REST_STEPS and all(on and np.linalg.norm(p - pos) <= MOVE_M for p, on in self.trail):
                self.carrying, self.anchor, self.landed = False, pos, None  # put down on a top
            return False
        if was_on_bench:  # lifted: the bottom has just passed 1.5 cm
            self.carrying = True
            return True
        if self.anchor is None:  # first seen off the bench, at rest: it rests here
            self.anchor = pos
            return False
        if np.linalg.norm(pos - self.anchor) > MOVE_M:  # moved from where it rested on a top
            self.carrying = True
            return True
        return False


TRANSITS = [Transit() for _ in range(N_VIALS)]


def carrying(i: int) -> bool:
    return TRANSITS[i].carrying


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    LAMP.reset()
    STATE.update(down=False, presses=0, lapsed_at=None, violation=False)
    TRANSITS[:] = [Transit() for _ in range(N_VIALS)]
    for i in range(N_VIALS):  # where each vial rests after the reset, when the states are given
        if f"vial{i}" in states:
            TRANSITS[i].seed(states[f"vial{i}"])


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    now = CLOCK.step
    if LAMP.is_("pending") and now - LAMP.since >= PENDING_STEPS:
        LAMP.set("green", now)
    elif LAMP.is_("green") and now - LAMP.since >= GREEN_STEPS:
        LAMP.set("off", now)
        STATE["lapsed_at"] = now
    down = pushed(states["request_button"], "cap", BUTTON_TRAVEL, PRESS_AT)
    if down and not STATE["down"]:  # a rising edge: holding the cap down is one press
        STATE["presses"] += 1
        if LAMP.is_("off"):  # asked while amber or green, the request does nothing
            LAMP.set("pending", now)
    STATE["down"] = down
    green = LAMP.is_("green")
    overdue = STATE["lapsed_at"] is not None and now - STATE["lapsed_at"] > GRACE_STEPS
    for i in range(N_VIALS):
        started = TRANSITS[i].update(states[f"vial{i}"], tops(states, f"vial{i}"))
        if started and not green:
            STATE["violation"] = True  # permanent: moved without a permit
        elif TRANSITS[i].unsettled and not green and overdue:
            STATE["violation"] = True  # permanent: still not set down 3 s after the permit lapsed


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
TRAVEL_Z = TOP + 0.170  # a held vial's bottom rides 2 cm over the standing vials' caps (9 cm tall)
PLACE_Z = TOP + 0.066
SPEED = 1.0  # pick_place pace: 9.6 s per vial
# Far vial first, into carrier slots 0 to 3: the slot 5 cm ahead of the gripper is then always empty, so the left
# wrist camera, which hangs ahead of the fingers, never passes over a cap.
PICK_ORDER = (3, 2, 1, 0)
# The arm settles a few millimetres off its target at the carrier, so the open-loop demo aims off by that much (m).
# Starting values borrowed from the tubes exemplar's measured error for the same motion; re-measured after the GPU run.
TRIM = {"pick": (0.0, 0.0), "place": {0: (0.004, -0.005), 1: (0.003, -0.003), 2: (0.002, -0.003), 3: (0.001, -0.003)}}
TO_BUTTON_S = 1.5  # s: the right arm's move from its ready pose to the hover above the button
FIRST_PRESS_AT = TO_BUTTON_S  # s: the first press starts; the cap registers about 0.85 s in
LEFT_START = 5.5  # s: the left arm starts on the first pair of vials; the lamp is green from about 5.35 s
SECOND_PRESS_AT = 31.5  # s: the first permit lapses at about 30.35 s
LEFT_RESTART = 35.5  # s: the second pair, under the second permit (green from about 35.35 s)
PRESS_S = 1.0
HOLD_S = 0.8
RELEASE_S = 1.0
PARK_S = 1.0  # s: between the hover above the button and the parked pose
PARK_XY = (BUTTON_XY[0], BUTTON_XY[1] - 0.15)  # the right arm waits here, to its side of the button, clear of the lamp
CAP_TOP = BUTTON_Z + BUTTON_BASE[2] / 2 + BUTTON_CAP[2]
PRESS_Z = CAP_TOP + 0.013 - BUTTON_TRAVEL - 0.002  # pad tips 2 mm past the cap's full travel
ABOVE_Z = CAP_TOP + 0.060


def _move(vial: int, slot: int) -> list[Waypoint]:
    px, py = TRIM["pick"]
    qx, qy = TRIM["place"][slot]
    return pick_place_waypoints((SLOT_X[vial] + px, RACK_Y + py, GRIP_Z), (SLOT_X[slot] + qx, CARRIER_Y + qy, PLACE_Z),
                                travel_z=TRAVEL_Z, speed=SPEED)


def _left_plan() -> list[Waypoint]:
    moves = [_move(vial, slot) for slot, vial in enumerate(PICK_ORDER)]
    plan = [Waypoint(LEFT_START, (SLOT_X[PICK_ORDER[0]], RACK_Y, TRAVEL_Z), FINGER_OPEN, "wait for the permit")]
    plan += moves[0] + moves[1]
    plan.append(Waypoint(LEFT_RESTART - sum(w.duration for w in plan), (SLOT_X[PICK_ORDER[2]], RACK_Y, TRAVEL_Z),
                         FINGER_OPEN, "wait for the next permit"))
    plan += moves[2] + moves[3]
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
    plan = [Waypoint(TO_BUTTON_S, (x, y, ABOVE_Z), FINGER_CLOSED, "to the button")]
    plan += _press("ask for the permit")
    plan.append(Waypoint(PARK_S, (*PARK_XY, ABOVE_Z), FINGER_CLOSED, "park to the side, clear of the lamp"))
    plan.append(Waypoint(SECOND_PRESS_AT - PARK_S - sum(w.duration for w in plan), (*PARK_XY, ABOVE_Z), FINGER_CLOSED,
                         "wait for the permit to lapse"))
    plan.append(Waypoint(PARK_S, (x, y, ABOVE_Z), FINGER_CLOSED, "back to the button"))
    plan += _press("ask again")
    plan.append(Waypoint(PARK_S, (*PARK_XY, ABOVE_Z), FINGER_CLOSED, "park"))
    return plan


def demo():
    """Right arm: press the request button at 1.5 s and again at 31.5 s, parked to the side in between. Left arm:
    vials 3 and 2 from 5.5 s, vials 1 and 0 from 35.5 s, every lift inside a green window."""
    return {"left": _left_plan(), "right": _right_plan()}
