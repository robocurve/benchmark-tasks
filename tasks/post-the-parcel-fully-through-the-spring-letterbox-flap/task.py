"""On the station table, in front of the left arm: a locked drop box (the kind used for after-hours returns and key
drops). Its front is a cream panel 20 mm thick standing across the table, with a 200 mm wide, 60 mm tall slot whose
floor is 140 mm above the table. A brass flap hangs over the slot on the outside, hinged along its top edge; it
swings inward when something pushes it, and is pulled shut by its own weight and a spring. Behind the panel, the
box is closed (grey sides, back, roof and a smooth floor board). A brown packet, 160 x 60 x 25 mm, lies flat on the
table in front of the box, its long side pointing at the slot.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). Heights below are above the table top.
YC = 0.12  # the slot's centre line
X_OUT = 0.36  # the door's outer face (toward the robot)
DOOR_T = 0.020
X_IN = X_OUT + DOOR_T  # the door's inner face
SLOT_W, SLOT_Z0, SLOT_Z1 = 0.200, 0.140, 0.200  # slot width; floor and ceiling heights
DOOR_W, DOOR_H = 0.360, 0.260
HALL_D = 0.220  # box depth behind the door, inside
HALL_FLOOR = 0.005  # a smooth floor board on the table inside the box
WALL = 0.010

CREAM, BRASS, GREY, BROWN = (0.93, 0.89, 0.78), (0.78, 0.62, 0.25), (0.55, 0.56, 0.58), (0.60, 0.42, 0.25)

# Flap: a plate on the outer face, hinged along a line 4 mm above the slot ceiling; it covers the slot when shut.
FLAP_T, FLAP_W, FLAP_H = 0.003, 0.210, 0.066
PIVOT_Z = SLOT_Z1 + 0.004
FLAP_SHUT_GAP = 0.005  # m: the flap's lower edge at most this far inside the door face counts as shut
FLAP_SHUT = math.asin(FLAP_SHUT_GAP / FLAP_H)  # rad, about 0.076


def _b(size, centre, color):
    """A box given in door coordinates: x from the outer face, y from the slot centre, z above the table."""
    return parts.box(size, at=centre, color=color)


_side_w = (DOOR_W - SLOT_W) / 2
_door = [
    _b((DOOR_T, DOOR_W, SLOT_Z0), (DOOR_T / 2, 0.0, SLOT_Z0 / 2), CREAM),  # below the slot
    _b((DOOR_T, DOOR_W, DOOR_H - SLOT_Z1), (DOOR_T / 2, 0.0, (SLOT_Z1 + DOOR_H) / 2), CREAM),  # above it
    *(_b((DOOR_T, _side_w, SLOT_Z1 - SLOT_Z0), (DOOR_T / 2, s * (SLOT_W + _side_w) / 2, (SLOT_Z0 + SLOT_Z1) / 2), CREAM)
      for s in (1, -1)),  # beside it
]
_hall = [
    *(_b((HALL_D, WALL, DOOR_H), (DOOR_T + HALL_D / 2, s * (DOOR_W - WALL) / 2, DOOR_H / 2), GREY) for s in (1, -1)),
    _b((WALL, DOOR_W, DOOR_H), (DOOR_T + HALL_D + WALL / 2, 0.0, DOOR_H / 2), GREY),  # back
    _b((HALL_D, DOOR_W - 2 * WALL, HALL_FLOOR), (DOOR_T + HALL_D / 2, 0.0, HALL_FLOOR / 2), GREY),  # floor
    _b((HALL_D + WALL, DOOR_W, WALL), (DOOR_T + (HALL_D + WALL) / 2, 0.0, DOOR_H + WALL / 2), GREY),  # roof
]
FLAP = parts.Hinge(
    "flap",
    shapes=[_b((FLAP_T, FLAP_W, FLAP_H), (-FLAP_T / 2, 0.0, PIVOT_Z - FLAP_H / 2), BRASS)],
    pivot=(-FLAP_T / 2, 0.0, PIVOT_Z),
    axis=(0.0, -1.0, 0.0),  # positive swings the lower edge inward (+x)
    lower=0.0,  # shut against the door face
    upper=1.45,
    mass=0.05,
    spring=0.005,  # N m/rad, plus the flap's weight
    damping=0.005,
)
LETTERBOX = parts.Part(
    "letterbox", pos=(X_OUT, YC, TOP), shapes=[*_door, *_hall], joints=[FLAP], friction=(0.3, 0.25)
)

PACKET_SIZE = (0.160, 0.060, 0.025)
# Cardboard: lower friction than the kit's rubber-like default.
PACKET = parts.Part("packet", pos=(0.22, YC, TOP + PACKET_SIZE[2] / 2), shapes=[parts.box(PACKET_SIZE, color=BROWN)],
                    mass=0.10, friction=(0.4, 0.3))

OBJECTS = [LETTERBOX, PACKET]



def _inside(p) -> bool:
    """The whole packet inside the box: behind the door's inner face, in front of the back wall, between the
    side walls and under the roof (bounding box, which is never smaller than the packet)."""
    half = DOOR_W / 2 - WALL
    return bool(
        p.low[0] >= X_IN - 1e-6
        and p.high[0] <= X_IN + HALL_D + 1e-6
        and p.low[1] >= YC - half - 1e-6
        and p.high[1] <= YC + half + 1e-6
        and p.high[2] <= TOP + DOOR_H + 1e-6
    )


def _flap_shut(objects) -> bool:
    return objects["letterbox"].joints["flap"] <= FLAP_SHUT + 1e-6


def success(objects):
    p = objects["packet"]
    return bool(_inside(p) and _flap_shut(objects) and p.resting())


def score(objects):
    """1.0 inside with the flap shut, at rest; else 0."""
    return 1.0 if success(objects) else 0.0


# -- demo: the left arm grips the packet's trailing end, loads it level into the slot until the hand meets the door,
# and lets go; the packet's centre is then past the slot's inner edge and it tips into the box.
TIP = 0.013  # pad tips below the TCP
PAD_BACK = 0.013  # the pads reach about 13 mm behind the TCP (x) at the packet's height
HAND_FRONT = 0.034  # the finger's widest part reaches 34 mm ahead of the TCP, above the slot
GRIP_Z = TOP + 0.003 + TIP  # pad tips 3 mm above the table
SAG_Z = 0.004  # the left arm settles about 4 mm low while carrying
BOTTOM = SLOT_Z0 + (SLOT_Z1 - SLOT_Z0 - PACKET_SIZE[2]) / 2  # packet underside centred in the slot: 157.5 mm
CARRY_Z = TOP + BOTTOM + 0.003 + TIP + SAG_Z
TRAVEL_Z = TOP + 0.19


def demo():
    rear = PACKET.pos[0] - PACKET_SIZE[0] / 2
    gx = rear + PAD_BACK  # pads flush with the trailing end
    line_up = X_OUT - 0.010 - (PACKET_SIZE[0] - PAD_BACK)  # leading end 10 mm short of the flap
    home = X_OUT - HAND_FRONT - 0.002  # hand 2 mm short of the door face
    y = YC
    left = [
        Waypoint(1.5, (gx, y, TRAVEL_Z), FINGER_OPEN, "over the trailing end"),
        Waypoint(1.5, (gx, y, GRIP_Z), FINGER_OPEN, "descend"),
        Waypoint(0.8, (gx, y, GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (gx, y, CARRY_Z), FINGER_CLOSED, "lift to slot height"),
        Waypoint(1.5, (line_up, y, CARRY_Z), FINGER_CLOSED, "line up"),
        Waypoint(3.0, (home, y, CARRY_Z), FINGER_CLOSED, "post it"),
        Waypoint(0.6, (home, y, CARRY_Z), FINGER_OPEN, "let go"),
        Waypoint(1.2, (home - 0.08, y, CARRY_Z), FINGER_OPEN, "back off"),
        Waypoint(1.0, (home - 0.08, y, TRAVEL_Z), FINGER_OPEN, "up"),
    ]
    return {"left": left}
