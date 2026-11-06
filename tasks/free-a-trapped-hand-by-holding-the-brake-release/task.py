"""On the station table, in front of the arms and a little to the robot's left of centre: a small stopped robot arm
and a fixture. The robot's dark grey base carries a joint with a vertical axis; its orange arm (a 20 cm bar with a
black knob near its tip) lies along the fixture's front face, pointing to the robot's right. A skin-coloured
training hand (a palm block standing against the fixture, with a forearm block lying on the table toward the
robot) is caught between the arm and the fixture: the palm sits in a shallow recess in the fixture face, and the
forearm passes under the arm. The joint has a holding brake: a pin under a toothed ring on top of the joint locks
it. The black button on top of the base, beside the ring, lowers the pin while it is held down; let go, and a spring
pushes the pin back up into the next gap between the teeth. A green mat lies on the table to the robot's right.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). Heights below are above the table top.
DARK = (0.22, 0.23, 0.25)
ORANGE = (0.95, 0.45, 0.05)
BLACK = (0.05, 0.05, 0.05)
STEEL = (0.62, 0.64, 0.67)
FIXTURE_GREY = (0.55, 0.56, 0.58)
SKIN = (0.88, 0.67, 0.53)
GREEN = (0.15, 0.62, 0.25)

# -- the stopped robot -------------------------------------------------------------------------------------------
PIVOT_XY = (0.293, 0.06)  # the joint axis (vertical)
ARM_LEN = 0.20  # from the axis to the arm's tip, toward -y
ARM_W, ARM_Z0, ARM_Z1 = 0.040, 0.030, 0.070  # bar width (x) and height span
HUB_R, HUB_TOP = 0.030, 0.105
RING_R, RING_Z0, RING_Z1 = 0.055, 0.105, 0.110  # the brake ring (a disc) on top of the hub
TOOTH_R, TOOTH_AT, TOOTH_Z0 = 0.003, 0.050, 0.095  # teeth hang from the ring's underside, 24 of them, 15 deg apart
N_TEETH = 24
BASE_TOP = 0.090
KNOB_DY, KNOB_R, KNOB_H = -0.180, 0.012, 0.035  # the knob on the arm's top, 18 cm from the axis

# The brake pin rises from the base top into the gap between two teeth, on the robot's left of the axis.
PIN_DY = TOOTH_AT  # pin at +y of the axis, at the teeth radius
PIN_W, PIN_TOP = 0.005, 0.103
BUTTON_DY, BUTTON = 0.090, (0.030, 0.030, 0.020)  # button head on the base top, 9 cm left of the axis
TRAVEL = 0.020
BRAKE_SPRING = 40.0  # N/m



def _robot():
    px, py = PIVOT_XY
    base_shapes = [
        parts.box((0.10, 0.12, BASE_TOP), at=(0.0, 0.06, BASE_TOP / 2), color=DARK),  # y 0.06..0.18 (world)
    ]
    teeth = []
    for k in range(N_TEETH):
        a = math.radians(90.0 + 7.5 + 15.0 * k)  # the pin (at 90 deg) sits halfway between two teeth
        teeth.append(parts.cylinder(TOOTH_R, RING_Z0 - TOOTH_Z0,
                                    at=(TOOTH_AT * math.cos(a), TOOTH_AT * math.sin(a), (TOOTH_Z0 + RING_Z0) / 2),
                                    color=STEEL, axis="z"))
    arm_shapes = [
        parts.box((ARM_W, ARM_LEN, ARM_Z1 - ARM_Z0), at=(0.0, -ARM_LEN / 2, (ARM_Z0 + ARM_Z1) / 2), color=ORANGE),
        parts.cylinder(HUB_R, HUB_TOP - ARM_Z0, at=(0.0, 0.0, (ARM_Z0 + HUB_TOP) / 2), color=ORANGE, axis="z"),
        parts.cylinder(RING_R, RING_Z1 - RING_Z0, at=(0.0, 0.0, (RING_Z0 + RING_Z1) / 2), color=STEEL, axis="z"),
        *teeth,
        parts.cylinder(KNOB_R, KNOB_H, at=(0.0, KNOB_DY, ARM_Z1 + KNOB_H / 2), color=BLACK, axis="z"),
    ]
    pin_bottom = 0.070
    brake_shapes = [
        parts.box((PIN_W, PIN_W, PIN_TOP - pin_bottom), at=(0.0, PIN_DY, (PIN_TOP + pin_bottom) / 2), color=STEEL),
        parts.box((0.012, BUTTON_DY - PIN_DY + 0.012, 0.008), at=(0.0, (PIN_DY + BUTTON_DY) / 2, pin_bottom + 0.004),
                  color=BLACK),  # link from the button to the pin, inside the base
        parts.box(BUTTON, at=(0.0, BUTTON_DY, BASE_TOP + BUTTON[2] / 2), color=BLACK),
    ]
    return parts.Part(
        "robot", pos=(px, py, TOP), shapes=base_shapes,
        joints=[
            parts.Hinge("arm", shapes=arm_shapes, pivot=(0.0, 0.0, ARM_Z0), axis=(0.0, 0.0, 1.0),
                        lower=-1.6, upper=0.0, mass=0.3, spring=0.0, damping=0.05),
            parts.Slider("brake", shapes=brake_shapes, axis=(0.0, 0.0, -1.0), travel=TRAVEL, mass=0.01,
                         spring=BRAKE_SPRING, damping=1.0),
        ],
    )


ROBOT = _robot()

# -- the fixture the hand is caught against --------------------------------------------------------------------------
FACE_X = 0.346  # fixture front face: 2 mm behind the palm
RECESS_X = 0.330  # front of the two recess side blocks
PALM = (0.030, 0.070, 0.050)
PALM_Y = (-0.110, -0.040)
HAND_Y = (PALM_Y[0] + PALM_Y[1]) / 2
RECESS_W = 0.076  # 3 mm wider than the palm on each side
RECESS_SIDE_Y = RECESS_W / 2 + 0.004  # centre of each 8 mm recess side block
FIXTURE = parts.Part(
    "fixture", pos=(FACE_X, HAND_Y, TOP),
    shapes=[
        parts.box((0.03, 0.16, 0.080), at=(0.015, 0.0, 0.040), color=FIXTURE_GREY),  # x 0.346..0.376
        *(parts.box((FACE_X - RECESS_X, 0.008, 0.080), at=(-(FACE_X - RECESS_X) / 2, side * RECESS_SIDE_Y, 0.040),
                    color=FIXTURE_GREY) for side in (-1, 1)),  # the recess's two side blocks
    ],
)

# -- the training hand (loose): palm standing in the recess, forearm lying on the table under the arm ---------------
ARM_FRONT_X = PIVOT_XY[0] + ARM_W / 2  # 0.313
PALM_X0 = ARM_FRONT_X + 0.001  # 1 mm from the arm and 2 mm from the fixture face
FOREARM = (0.100, 0.040, 0.025)
HAND = parts.Part(
    "hand", pos=(PALM_X0, HAND_Y, TOP), mass=0.08,
    shapes=[
        parts.box(PALM, at=(PALM[0] / 2, 0.0, PALM[2] / 2), color=SKIN),
        parts.box(FOREARM, at=(-FOREARM[0] / 2, 0.0, FOREARM[2] / 2), color=SKIN),
    ],
)

# -- the mat -----------------------------------------------------------------------------------------------------
MAT_XY, MAT = (0.18, -0.21), (0.20, 0.12, 0.002)
MAT_BOX = Box("mat", pos=(*MAT_XY, TOP + MAT[2] / 2), size=MAT, color=GREEN)

OBJECTS = [ROBOT, FIXTURE, HAND, MAT_BOX]
DEMO_TOUCHES = ("robot",)  # the left fingers press the button flush with the base top

EPS = 1e-6
ON_TOL = 0.001  # hand bottom within 1 mm of the mat's top
MAX_TILT = math.radians(10.0)


def _tilt(state) -> float:
    x, y, z, w = state.quat_xyzw
    up_z = 1 - 2 * (x * x + y * y)  # the hand's own z axis, world z component
    return math.acos(max(-1.0, min(1.0, up_z)))


def _on_mat(objects) -> bool:
    """The hand lies flat on the mat: its bottom within 1 mm of the mat's top, leaning under 10 deg, the centre of
    its outline seen from above inside the mat, and still."""
    h = objects["hand"]
    cx, cy = (h.low[0] + h.high[0]) / 2, (h.low[1] + h.high[1]) / 2
    mat_top = TOP + MAT[2]
    return bool(
        abs(h.low[2] - mat_top) <= ON_TOL + EPS
        and _tilt(h) < MAX_TILT
        and abs(cx - MAT_XY[0]) <= MAT[0] / 2 + EPS
        and abs(cy - MAT_XY[1]) <= MAT[1] / 2 + EPS
        and h.resting()
    )


def success(objects):
    return _on_mat(objects)


def distance(objects):
    h = objects["hand"]
    c = (h.low + h.high) / 2
    return float(np.hypot(c[0] - MAT_XY[0], c[1] - MAT_XY[1]))


def _out_of_recess(objects) -> bool:
    """The hand is entirely out of the recess (every part of it nearer the robot than the recess side blocks' front,
    x 33.0 cm) and resting on the table or the mat: its bottom within 1 mm of either surface, and still."""
    h = objects["hand"]
    on_surface = any(abs(h.low[2] - z) <= ON_TOL + EPS for z in (TOP, TOP + MAT[2]))
    return bool(h.high[0] < RECESS_X - EPS and on_surface and h.resting())


def score(objects):
    """1.0 hand standing on the mat as it started, and still; 0.5 the hand entirely out of the recess and resting on
    the table or mat; else 0."""
    if _on_mat(objects):
        return 1.0
    if _out_of_recess(objects):
        return 0.5
    return 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
TRAVEL_Z = TOP + 0.16
SWING = math.radians(75.0)  # a lock position: the pin drops into a gap when the button is let go
KNOB_YAW = 0.9  # the grip on the round knob starts turned this far; it turns with the arm


def _knob(theta: float):
    px, py = PIVOT_XY
    return (px - KNOB_DY * math.sin(theta), py + KNOB_DY * math.cos(theta))


def demo():
    """Left arm: presses the button with closed fingers, about 26 mm past first contact, and holds it while the right
    arm swings the arm away; then lets go. Right arm: grips the knob, follows the knob's arc toward the robot, lets
    go, then grips the forearm in front of where the arm was, lifts the hand straight up out of the recess, carries
    it to the mat and releases it 7 mm up."""
    bx, by = PIVOT_XY[0], PIVOT_XY[1] + BUTTON_DY
    touch = TOP + BASE_TOP + BUTTON[2] + TIP  # TCP height at which the tips meet the button
    left = [
        Waypoint(1.5, (bx, by, TRAVEL_Z), FINGER_CLOSED, "over the button"),
        Waypoint(1.0, (bx, by, touch + 0.015), FINGER_CLOSED, "down to the button"),
        Waypoint(1.5, (bx, by, touch - 0.026), FINGER_CLOSED, "press the button"),
        Waypoint(9.5, (bx, by, touch - 0.026), FINGER_CLOSED, "hold it while the arm swings"),
        Waypoint(1.0, (bx, by, touch + 0.03), FINGER_CLOSED, "let go"),
        Waypoint(1.0, (bx - 0.05, by + 0.05, TRAVEL_Z), FINGER_CLOSED, "up and away"),
    ]
    kz = TOP + ARM_Z1 + 0.008 + TIP  # finger tips 8 mm above the arm's top, beside the knob
    kx, ky = _knob(0.0)
    steps = 8
    arc = [Waypoint(0.8, (*_knob(-SWING * k / steps), kz), FINGER_CLOSED, f"swing {k}", KNOB_YAW - SWING * k / steps)
           for k in range(1, steps + 1)]
    fx, fy = PALM_X0 - 0.032, HAND_Y  # forearm grip near the palm, over the hand's centre of mass
    gz = TOP + 0.003 + TIP  # finger tips 3 mm above the table
    mx, my = MAT_XY[0] + 0.003, MAT_XY[1]
    right = [
        Waypoint(1.5, (kx - 0.04, ky - 0.06, TRAVEL_Z), FINGER_OPEN, "aside"),
        Waypoint(2.5, (kx, ky, TRAVEL_Z), FINGER_OPEN, "over the knob", KNOB_YAW),
        Waypoint(1.0, (kx, ky, kz), FINGER_OPEN, "down to the knob", KNOB_YAW),
        Waypoint(0.8, (kx, ky, kz), FINGER_CLOSED, "grip the knob", KNOB_YAW),
        *arc,
        Waypoint(0.6, (*_knob(-SWING), kz), FINGER_OPEN, "let go of the knob", KNOB_YAW - SWING),
        Waypoint(1.0, (*_knob(-SWING), TRAVEL_Z), FINGER_OPEN, "up", KNOB_YAW - SWING),
        Waypoint(1.5, (fx, fy, TRAVEL_Z), FINGER_OPEN, "over the forearm"),
        Waypoint(1.2, (fx, fy, gz), FINGER_OPEN, "down to the forearm"),
        Waypoint(0.8, (fx, fy, gz), FINGER_CLOSED, "grip the forearm"),
        Waypoint(2.0, (fx, fy, gz + 0.07), FINGER_CLOSED, "lift the hand out"),
        Waypoint(1.8, (mx, my, gz + 0.07), FINGER_CLOSED, "carry to the mat"),
        Waypoint(1.2, (mx, my, gz + MAT[2] + 0.007), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (mx, my, gz + MAT[2] + 0.007), FINGER_OPEN, "release"),
        Waypoint(1.0, (mx, my, TRAVEL_Z), FINGER_OPEN, "up"),
    ]
    return {"left": left, "right": right}
