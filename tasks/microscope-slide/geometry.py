"""Dimensions and layout of the microscope-slide task.

Station frame: metres from the midpoint between the arm bases, x forward (away from the arms), y to the robot's left,
z up. Microscope frame ("M"): origin on the optical axis at the stage's top surface (stage at its centred readings),
x toward the microscope's back (the station's +x), y to its left (the station's +y), z up.

The microscope is an AmScope B120C (binocular, two-layer mechanical stage 116 x 125 mm, 70 x 25 mm X-Y travel, metal
template with a spring caliper and lever, forward-facing quadruple nosepiece, 45 mm parfocal objectives). Sizes marked
(est.) are estimates from the maker's specifications and comparable instruments; the README lists the ones measured on a
unit before the real trials.

The slide is a ground-edge colour-frosted glass slide, 75 x 25 x 1 mm, frosted (label) end 20 mm, with a 22 mm square
cover glass centred on its top face. Slide frame: origin at the slide's centre (mid-thickness), x along its length
toward the label end, y across its width, z out of the coverslip face.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475                        # the station's table top

# -- the microscope (AmScope B120C) -------------------------------------------------------------------------------------
AXIS_X, AXIS_Y = 0.360, 0.000       # the optical axis (set by the reach check, dev/reach2.py)
STAGE_H = 0.190                     # stage top above the table, stage fully lowered (est.; CX23 drawing: 190 mm)
STAGE_Z = TOP + STAGE_H             # world height of the stage's top surface
# the stage plates (moving upper layer, fixed lower layer), M frame
STAGE_X = (-0.0625, 0.0625)         # 125 mm deep (front -x .. back +x)
STAGE_Y = (-0.058, 0.058)           # 116 mm wide
STAGE_UPPER_T = 0.004               # the upper (moving) plate, its top at z = 0
STAGE_LOWER_T = 0.008               # the lower (fixed) plate under it
APERTURE_R = 0.012                  # the light hole (visual only)
# the template (on the upper plate): back strip and fixed jaw (left), 1.5 mm steel
TEMPLATE_T = 0.0015
BACK_EDGE_X = 0.0125                # the back strip's front face: a seated slide's back edge
JAW_Y = 0.0375                      # the jaw's right face: a seated slide's left end
BACK_STRIP = ((BACK_EDGE_X, 0.0275), (-0.042, 0.050))      # x range, y range
JAW = ((-0.016, BACK_EDGE_X), (JAW_Y, 0.050))
# the caliper: a curved arm on a vertical pivot, swinging in the stage plane; its tip presses the slide's right end
PIVOT = (0.022, -0.050)             # M frame (est.)
ARM_Z = (0.0002, 0.0018)            # the arm's thickness band above the upper plate
ARM_A = ((-0.004, 0.022), (-0.0525, -0.0475))   # from the hub forward (closed pose, M frame)
ARM_TIP = ((-0.006, 0.002), (-0.0500, -0.0375))  # the tip finger, its +y face on a seated slide's right end
HUB_R = 0.004
LEVER_LINK = ((-0.018 + 0.022, 0.022), (-0.066, -0.050))   # visual/collider link from the hub to the knob
KNOB = (0.004, -0.066)              # the lever knob's axis, closed (M frame)
KNOB_R = 0.004
KNOB_Z = (0.0018, 0.014)
CAL_OPEN = math.radians(40.0)       # fully open (joint upper limit)
CAL_SHUT = math.radians(-6.0)       # closing stop with no slide (joint lower limit)
CAL_PRELOAD = math.radians(-60.0)   # the spring drive's target (a preloaded torsion spring)
TIP_FORCE = 1.5                     # N at the tip on a seated slide (est.; measured with a kitchen scale)
TIP_ARM = 0.022 + 0.002             # pivot to the tip face's line of action (x offset), m
CAL_K = TIP_FORCE * TIP_ARM / (0.0 - CAL_PRELOAD)        # N m / rad
CAL_FREE_SPEED = 0.40               # m/s: dropped lever, tip speed (est.; the README's 240 fps measurement sets it)
CAL_MASS = 0.012                    # kg (est.)
# a dropped lever closes damping-limited: tip speed = r * torque / damping  (torque at ~20 deg open)
CAL_R_TIP = math.hypot(ARM_TIP[0][0] - PIVOT[0], ARM_TIP[1][1] - PIVOT[1])
CAL_D = CAL_K * (math.radians(20.0) - CAL_PRELOAD) * CAL_R_TIP / CAL_FREE_SPEED
# the stage carriage: two prismatic joints (Y knob: along x; X knob: along y), Coulomb-like breakaway
STAGE_BREAK = 5.0                   # N (est.; measured with a kitchen scale against the holder)
STAGE_K = 1.0e5                     # N/m: the carriage held by a stiff spring to a rest point (stick) ...
STAGE_DAMP = 300.0                  # N s/m
                                    # ... that task.py drags along when the spring force passes STAGE_BREAK (slip)
STAGE_TRAVEL = ((-0.0125, 0.0125), (-0.035, 0.035))      # Y knob (x), X knob (y)
# the fixed frame (M frame boxes: (x range, y range, z range)), est.
BASE = ((-0.075, 0.125), (-0.085, 0.085), (-STAGE_H, -STAGE_H + 0.040))
PILLAR = ((0.078, 0.125), (-0.032, 0.032), (-STAGE_H + 0.040, 0.150))
BRACKET = ((0.020, 0.078), (-0.030, 0.030), (-0.045, -STAGE_UPPER_T - STAGE_LOWER_T))
CONDENSER = (0.018, (-0.060, -STAGE_UPPER_T - STAGE_LOWER_T))      # radius, z range, on the axis
XY_KNOBS = ((-0.030, -0.052), 0.011, (-0.075, -0.014))             # (x, y), radius, z range (right side, below)
FOCUS_KNOBS = ((0.100, -0.110), 0.022, (0.032, 0.062))             # (x, z), radius, |y| range (both sides)
LIMB = ((-0.030, 0.125), (-0.030, 0.030), (0.098, 0.150))          # the arm's front, over the nosepiece
HEAD = ((-0.055, 0.060), (-0.038, 0.038), (0.150, 0.205))
EYEPIECES = (0.016, 0.032, (-0.045, 0.195), math.radians(45.0), 0.115)   # radius, |y|, start (x, z), tilt, length
# the nosepiece and objectives (forward-facing: the idle objectives lean back toward the limb), est.
OBJ4_TIP = 0.045                    # the 4x objective's tip above the stage top, stage fully lowered (est.: 4-5 cm)
OBJ4_R = (0.0075, 0.0105)           # its radius at the tip and at the shoulder
OBJ4_LEN = 0.022                    # tip to shoulder
TURRET_C = (0.016, 0.0, OBJ4_TIP + OBJ4_LEN + 0.012)   # the turret's centre, M frame
TURRET_R, TURRET_T = 0.036, 0.022
TURRET_TILT = math.radians(18.0)    # its axis tilted back from vertical (top toward +x)
OBJ_OTHERS = (                      # (shoulder position offset from the 4x's, tilt from vertical, azimuth, length)
    (0.030, math.radians(28.0), 0.0, 0.040),
    (0.022, math.radians(22.0), math.radians(55.0), 0.036),
    (0.022, math.radians(22.0), math.radians(-55.0), 0.030),
)
OBJ_OTHER_R = 0.0105

# -- the slide ---------------------------------------------------------------------------------------------------------
SLIDE_L, SLIDE_W, SLIDE_T = 0.075, 0.025, 0.001
LABEL_L = 0.020                     # the frosted label end
COVER_S, COVER_T = 0.022, 0.0002    # the cover glass (0.13-0.17 mm; 0.2 mm collider)
SLIDE_MASS = 0.0047
GLASS_MU = 0.3                      # glass on painted metal (est.)
PAD_MU = 0.9                        # glass on the finger pads (est.)

# -- the bench slide holder (Heathrow Scientific HS15981C), est. slot geometry -------------------------------------------
HOLDER_X, HOLDER_Y = 0.300, 0.220   # its centre (the middle slot)
HOLDER_SIZE = (0.152, 0.102, 0.020)
SLOTS, SLOT_PITCH = 15, 0.0095
SLOT_W, SLOT_GAP = 0.0265, 0.0016   # slot length (across y) and gap (x)
SLOT_DEPTH = 0.015                  # along the slot's axis
SLOT_LEAN = math.radians(20.0)      # leaning back (top away from the coverslip side)
HOLDER_YAW = math.radians(90.0)     # turned so the slots run along x: the slide's coverslip faces -y (the microscope);
                                    # the 15 slots stack along y (set by the reach check: a top-down pick the wrist can
                                    # also turn flat; dev/plan_check3.py)

# -- scoring constants (spec "How it is scored") -------------------------------------------------------------------------
FACE_COS = math.cos(math.radians(45.0))   # a gripper contact normal within 45 deg of the face normal is a face contact
FACE_INSET = 0.0003                 # ... if its point lies on the face more than this inside the outline (not the arris)
TOUCH_F = 1e-3                      # N: a contact carrying force counts as a touch
DROP = 0.010                        # free fall
STAGE_MOVED = 0.001                 # carriage travel from its start
V_MAX = 0.020                       # m/s, mean tip speed over its last 5 mm before contact
LAST_MM = 0.005
TIP_STILL = 0.00005                 # the caliper tip moving less than this per step (1 mm/s) is still
FLAT_TOL = 0.0003                   # corners within 0.3 mm of the stage plate
SEAT_TOL = 0.0003                   # back edge and jaw within 0.3 mm
UNDER_TOL = 0.008                   # coverslip centre within 8 mm of the optical axis
CLEAR = 0.030                       # grippers 3 cm or more from slide, arm and lever
REST_SPEED = 0.002                  # m/s: "still"
HOLD_STEPS = 20                     # the final 1 s at 20 Hz


def to_world(p_m) -> np.ndarray:
    """M-frame point -> station frame (stage at its centred readings)."""
    p = np.asarray(p_m, float)
    return p + np.array([AXIS_X, AXIS_Y, STAGE_Z])


def caliper_point(p_closed_m, phi: float, carriage=(0.0, 0.0)) -> np.ndarray:
    """A point of the caliper (M frame, given in its closed pose phi = 0) at joint angle phi (CCW = open), on the
    carriage offset (dx, dy)."""
    c, s = math.cos(phi), math.sin(phi)
    v = np.asarray(p_closed_m[:2], float) - PIVOT
    out = np.array([PIVOT[0] + c * v[0] - s * v[1] + carriage[0], PIVOT[1] + s * v[0] + c * v[1] + carriage[1]])
    return np.r_[out, p_closed_m[2]] if len(p_closed_m) > 2 else out


def slide_corners() -> np.ndarray:
    """The slide's four bottom corners, slide frame."""
    hl, hw, ht = SLIDE_L / 2, SLIDE_W / 2, SLIDE_T / 2
    return np.array([[sx * hl, sy * hw, -ht] for sx in (-1, 1) for sy in (-1, 1)])


def holder_slot_floor() -> np.ndarray:
    """The middle slot's floor centre, world (the slide's bottom end rests on it); the holder's origin."""
    return np.array([HOLDER_X, HOLDER_Y, TOP + HOLDER_SIZE[2] - SLOT_DEPTH * math.cos(SLOT_LEAN)])


def holder_rot() -> np.ndarray:
    c, s = math.cos(HOLDER_YAW), math.sin(HOLDER_YAW)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def slide_start():
    """The slide's start pose in the middle slot: standing label end up, leaning back SLOT_LEAN, coverslip facing the
    holder's local -x (world -y). Returns (position, rotation matrix): slide x (to the label end), z (coverslip)."""
    a = SLOT_LEAN
    Rh = holder_rot()
    ex = Rh @ np.array([math.sin(a), 0.0, math.cos(a)])   # along the slide, to the label end
    ez = Rh @ np.array([-math.cos(a), 0.0, math.sin(a)])  # coverslip normal: toward local -x, tilted up
    ey = np.cross(ez, ex)
    R = np.column_stack([ex, ey, ez])
    floor = holder_slot_floor()
    # bottom end on the floor, the back face (-ez) against the slot's back wall (the slot centred on the floor point)
    pos = floor + ex * (SLIDE_L / 2 + 0.0002) + ez * (-(SLOT_GAP / 2 - SLIDE_T / 2))
    return pos, R
