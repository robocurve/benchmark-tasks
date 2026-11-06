"""Geometry, masses and kinematics of the dividing head. NumPy only (no Isaac).

Units: metres, kilograms, radians unless a name ends in _DEG; quaternions xyzw. Station frame: x forward from the
line between the arm bases, y left, z up; TOP is the table top. Heights below are above the table top.

The head: an opaque dark-blue housing on the table. On its top, at CRANK_XY, a horizontal index plate with two hole
circles (20 holes inner, 21 holes outer) and a vertical crank axis. The crank arm carries a slide (the plunger's
radial setting, a damped prismatic joint between the two circles' radii), the plunger pin (a vertical prismatic
joint with a spring pushing it down) and the knob sleeve (spinning freely on the pin, so a gripper can keep its yaw
while it circles). Pocket holes are the gaps between raised "lands", so the seated pin locks the crank by contact.
The spindle (vertical, at SPINDLE_XY) carries the workpiece disc; it is kinematic and follows crank angle / 40 in code,
one way (the worm drive is not back-drivable and is not modelled).

Angles: crank angle = the rotor's yaw (the crank arm's direction, rotor local +x). Hole k of an N-hole circle sits at
ZERO_DEG - k * 360 / N (k counted clockwise from above, the direction the demonstration turns).
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena.task_file.TABLE_TOP["station"])

# -- layout ------------------------------------------------------------------------------------------
CRANK_XY = np.array([0.28, 0.17])  # crank axis, in front of the left arm
SPINDLE_XY = np.array([0.28, 0.00])  # spindle axis, on the centre line
ZERO_DEG = 0.0  # world direction of the zero line (both circles' hole 0); the crank starts here

# -- index plate -----------------------------------------------------------------------------------
RATIO = 40  # crank turns per spindle turn
CIRCLES = {20: 0.042, 21: 0.056}  # hole count -> circle radius (m)
INNER_N, OUTER_N = 20, 21
START_N = INNER_N  # the plunger starts in hole 0 of the 20-hole circle
PLATE_R = 0.070
PLATE_Z0 = 0.050  # plate bottom (housing top)
PLATE_BASE_T = 0.006  # solid plate under the pockets: pocket floor at PLATE_Z0 + PLATE_BASE_T
POCKET_DEPTH = 0.005
PLATE_TOP = PLATE_Z0 + PLATE_BASE_T + POCKET_DEPTH  # 0.061: the plate's working face
POCKET_W = 0.007  # pocket (hole) size, tangential and radial
BAND_SEGMENTS = 36  # the raised annulus between the circles, as convex segments

# -- crank, slide, pin, knob ----------------------------------------------------------------------------
PIN_R = 0.003  # 6 mm pin in a 7 mm pocket: 0.5 mm slack each side
PIN_DEPTH = 0.004  # seated pin bottom below the plate face (the prismatic joint's lower limit)
PIN_LIFT_MAX = 0.010  # upper limit of the pin joint (pin bottom 6 mm above the face)
PIN_LIFT = 0.007  # how far the demonstration lifts it (pin bottom 3 mm above the face)
SPRING_K = 150.0  # N/m, spring pushing the pin down
SPRING_PRELOAD = 0.010  # spring target this far below the seated limit: 1.5 N seated, 2.55 N at PIN_LIFT
SPRING_DAMPING = 3.0  # N s/m
SLIDE_DAMPING = 5.0  # N s/m
SLIDE_K = 40.0  # N/m: a light spring pulls the plunger in toward the 20-hole circle (0.4 N there, 0.96 N at the 21)
SLIDE_PRELOAD = 0.010  # its target this far inside the inner limit; a pin seated in a 21-hole pocket holds it out
ARM_Z = (0.067, 0.073)  # crank arm (visual): bottom and top heights above the table (the plate face is at 0.061)
HUB_R = 0.012
KNOB_R, KNOB_H = 0.010, 0.036
KNOB_Z0 = 0.080  # knob sleeve bottom when the pin is seated
KNOB_DZ = KNOB_Z0 + KNOB_H / 2 - PLATE_TOP  # knob centre above the pin origin

# -- spindle and workpiece ----------------------------------------------------------------------------
NOSE_R, NOSE_Z = 0.030, (PLATE_Z0, 0.062)  # spindle nose on the housing top
WORK_R, WORK_T = 0.050, 0.016  # workpiece disc (aluminium), 100 mm across, 16 mm thick
WORK_Z0 = NOSE_Z[1]  # disc bottom
POINTER_DEG = 0.0  # fixed pointer just outside the disc rim, in this world direction; the line starts on it

# -- housing (kinematic, opaque) --------------------------------------------------------------------
HOUSE_X = (-0.075, 0.075)  # relative to CRANK_XY[0]
HOUSE_Y = (SPINDLE_XY[1] - CRANK_XY[1] - 0.065, 0.078)  # relative to CRANK_XY[1]

# -- masses (declared: several shapes are visual only) ------------------------------------------------------
MASS = {"rotor": 0.25, "slide": 0.02, "pin": 0.015, "knob": 0.02}
INERTIA = {  # principal moments about each body's centre of mass (kg m^2)
    "rotor": (2.0e-4, 2.0e-4, 3.0e-4),
    "slide": (2.0e-6, 2.0e-6, 2.0e-6),
    "pin": (2.0e-6, 2.0e-6, 2.0e-6),
    "knob": (3.0e-6, 3.0e-6, 1.0e-6),
}
FRICTION = {"head": (0.6, 0.5), "pin": (0.3, 0.25), "knob": (1.0, 0.9), "work": (0.6, 0.5)}
CRANK_DAMPING = 0.02 * math.pi / 180.0  # N m per (deg/s): 0.02 N m s/rad bearing drag

# -- target --------------------------------------------------------------------------------------------
DIVISIONS = 7
TARGET_DEG = 360.0 / DIVISIONS  # 51.43 degrees of spindle
TARGET_TURNS = RATIO / DIVISIONS  # 5 15/21 crank turns
TOL_DEG = 0.2  # one hole on the 21-hole circle moves the spindle 0.43 degrees
TURNS_BAND_DEG = (5 * 360.0 / RATIO - TOL_DEG, 6 * 360.0 / RATIO - TOL_DEG)  # 5 whole turns: [44.8, 53.8)
CIRCLE_TOL_M = 0.0015  # the pin is on a circle when its radius is within this of the circle's
SEATED_MAX_M = 0.0015  # pin origin at most this above its seated limit (its bottom >= 2.5 mm into a pocket)


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def yaw_quat(a: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def yaw_of(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def tilt_of(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))


def hole_deg(n: int, k: float) -> float:
    """World direction of hole k on the n-hole circle (k counted clockwise from the zero line)."""
    return ZERO_DEG - k * 360.0 / n


def plate_point(r: float, a_deg: float, z: float) -> np.ndarray:
    a = math.radians(a_deg)
    return np.array([CRANK_XY[0] + r * math.cos(a), CRANK_XY[1] + r * math.sin(a), TOP + z])


def crank_poses(crank_deg: float, radius: float, lift: float = 0.0) -> dict[str, np.ndarray]:
    """Poses (xyz + quat xyzw) of rotor, slide, pin and knob for a crank angle, plunger radius and pin lift."""
    q = yaw_quat(math.radians(crank_deg))
    pin = plate_point(radius, crank_deg, PLATE_TOP + lift)
    return {
        "rotor": np.array([*plate_point(0.0, 0.0, PLATE_TOP), *q]),
        "slide": np.array([*plate_point(radius, crank_deg, PLATE_TOP), *q]),
        "pin": np.array([*pin, *q]),
        "knob": np.array([*(pin + [0.0, 0.0, KNOB_DZ]), *q]),
    }


def knob_center(crank_deg: float, radius: float, lift: float = 0.0) -> np.ndarray:
    return crank_poses(crank_deg, radius, lift)["knob"][:3]


def start_poses() -> dict[str, np.ndarray]:
    poses = crank_poses(ZERO_DEG, CIRCLES[START_N])
    poses["head"] = np.array([CRANK_XY[0], CRANK_XY[1], TOP, 0.0, 0.0, 0.0, 1.0])
    poses["work"] = work_pose(0.0)
    return poses


def work_pose(spindle_deg: float) -> np.ndarray:
    """Workpiece pose for a spindle rotation (deg) from the start; its painted line starts at POINTER_DEG."""
    return np.array([SPINDLE_XY[0], SPINDLE_XY[1], TOP + WORK_Z0,
                     *yaw_quat(math.radians(POINTER_DEG + spindle_deg))])


def check_layout(p: dict[str, np.ndarray]) -> dict[str, float]:
    """Joint-anchor mismatches (m) of the crank chain: all ~0 when the joints hold."""
    rot = p["rotor"]
    yaw = yaw_of(rot[3:])
    u = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    axis = np.array([CRANK_XY[0], CRANK_XY[1], TOP + PLATE_TOP])
    slide = p["slide"][:3] - rot[:3]
    off_arm = float(np.linalg.norm(slide - np.dot(slide, u) * u))  # the slide off the arm's line
    pin = p["pin"][:3] - p["slide"][:3]
    knob = p["knob"][:3] - p["pin"][:3] - np.array([0.0, 0.0, KNOB_DZ])
    return {"crank_axis_m": float(np.linalg.norm(rot[:3] - axis)), "slide_off_arm_m": off_arm,
            "pin_off_slide_m": float(np.linalg.norm(pin[:2])), "knob_off_pin_m": float(np.linalg.norm(knob))}
