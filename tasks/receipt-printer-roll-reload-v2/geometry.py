"""Geometry for receipt_printer_roll_reload_v2 (metres; x forward and y left from the arm midpoint; table top = bench).

Frames: z of a part is its height above the table top. The printer origin is the centre of its base on the table; the
cover is a hinged lid joint at the back top edge of the printer body (x = +0.060, z = 0.070 in the printer frame). The roll
origin is the bottom of the paper cylinder on its axis: the cylinder axis runs along the part's y, the paper tail runs
along the part's +x and lies 3 mm under the cylinder (z -0.003 to 0). The stand origin is the centre of its plate on the table.
"""

from __future__ import annotations

import math

from mechkit import Shape, box

TABLE_TOP = 0.7475

COLOR_BODY = (0.17, 0.18, 0.20)        # charcoal ABS
COLOR_COVER = (0.10, 0.11, 0.13)       # darker ABS
COLOR_RIB = (0.22, 0.23, 0.25)
COLOR_WINDOW = (0.55, 0.70, 0.78)      # clear window
COLOR_TAB = (0.12, 0.34, 0.85)         # blue lift lip
COLOR_BEZEL = (0.05, 0.05, 0.06)
COLOR_BUTTON = (0.62, 0.64, 0.67)
COLOR_PAPER = (0.95, 0.94, 0.90)
COLOR_CORE = (0.62, 0.46, 0.28)        # cardboard core
COLOR_TAPE = (0.90, 0.45, 0.10)        # tape on the paper's free end
COLOR_STAND = (0.12, 0.12, 0.13)       # rubber
COLOR_LAMP_RED = (0.95, 0.05, 0.05)
COLOR_LAMP_GREEN = (0.05, 0.85, 0.20)
COLOR_LAMP_AMBER = (1.0, 0.62, 0.0)

# -- printer: fixed (25 kg stand-in for a bolted-down printer), 12 x 20 x 7 cm, front face at x 0.155, rear face at 0.275 ---
PRINTER_NAME = "rp_printer"
PRINTER_XY = (0.215, 0.0)
PRINTER_SIZE = (0.120, 0.200, 0.070)
PRINTER_MASS_KG = 25.0
PRINTER_FRICTION = 1.2
FRONT_FACE_X = PRINTER_XY[0] - PRINTER_SIZE[0] / 2        # 0.155
BAY_FLOOR_Z = 0.005                                         # top of the base slab: the bay floor and the chute top
WALL_TOP_Z = PRINTER_SIZE[2]                               # the cover rests on the walls' tops at z 0.070
# Bay facts used by the scorer and the tests (absolute x, y; z above the table).
BAY_X_RANGE = (0.1725, 0.2475)          # interior between the front posts and the rear wall: 7.5 cm (60 mm roll + 7.5 mm per side)
CRADLE_HALF_WIDTH_M = 0.034            # the roll's channel between the shoulders: 6.8 cm (58 mm roll + 5 mm per side)
FINGER_RECESS_HALF_WIDTH_M = 0.065     # the bay opens to 13 cm above the shoulders, for the gripper pads
NOTCH_HALF_WIDTH_M = 0.045             # the exit notch in the front wall: 9 cm (58 mm paper + 16 mm per side), open to the cover
SHOULDER_TOP_Z = 0.020
CHUTE_X_RANGE = (0.095, FRONT_FACE_X)
_SIDE = PRINTER_SIZE[1] / 2            # 0.100


def _mirror_y(centre_x, y_lo, y_hi, z_c, size_x, size_z, colour):
    """The same box on both sides of y = 0: (centre, size, colour) for y in [y_lo, y_hi] and [-y_hi, -y_lo]."""
    yc, sy = (y_lo + y_hi) / 2, y_hi - y_lo
    return (box((centre_x, yc, z_c), (size_x, sy, size_z), colour), box((centre_x, -yc, z_c), (size_x, sy, size_z), colour))


# Printer-frame shapes (for Mech).
PRINTER_SHAPES = (
    box((0.0, 0.0, 0.0025), (0.120, 0.200, 0.005), COLOR_BODY),                              # base slab
    *_mirror_y(-0.050, NOTCH_HALF_WIDTH_M, _SIDE, 0.0375, 0.020, 0.065, COLOR_BODY),        # front posts
    *_mirror_y(0.0, FINGER_RECESS_HALF_WIDTH_M, _SIDE, 0.0375, 0.120, 0.065, COLOR_BODY),  # side walls
    box((0.0475, 0.0, 0.0375), (0.025, 0.200, 0.065), COLOR_BODY),                          # rear wall
    *_mirror_y(-0.0025, CRADLE_HALF_WIDTH_M, FINGER_RECESS_HALF_WIDTH_M, 0.0125, 0.075, 0.015, COLOR_BODY),  # shoulders
    box((-0.090, 0.0, 0.0025), (0.060, 0.100, 0.005), COLOR_BODY),                             # chute apron
    box((-0.062, 0.065, 0.050), (0.004, 0.020, 0.020), COLOR_BEZEL),                          # lamp bezel
    box((-0.063, -0.065, 0.050), (0.006, 0.016, 0.016), COLOR_BUTTON),                        # power button
)

# Lamp (display only, VisualizationMarkers): centre in the printer frame, size.
LAMP_CENTRE = (-0.0655, 0.065, 0.050)
LAMP_SIZE = (0.003, 0.016, 0.016)

# -- cover: ONE hinged lid at the back top edge of the printer body ---------------------------------------------------
COVER_NAME = "rp_cover"
COVER_MASS_KG = 0.15
COVER_SPRING_N_M_PER_RAD = 0.08        # small spring toward closed (<= 10 N m/rad equivalent that an arm can hold)
COVER_DAMPING = 0.2
COVER_LOW_RAD = 0.0                    # rest position = closed flat (0 rad)
COVER_HIGH_RAD = math.radians(85.0)    # stops: positive angle lifts the front upward
HINGE_AT = (0.060, 0.0, 0.070)         # hinge axis at the back top edge in printer frame (world x 0.275, z TABLE_TOP + 0.070)

# Cover shapes in the joint frame (hinge origin (0, 0, 0); rest has lid extending along -x).
COVER_PLATE = box((-0.055, 0.0, 0.004), (0.110, 0.200, 0.008), COLOR_COVER)
COVER_RIBS = (
    box((-0.090, 0.0, 0.009), (0.006, 0.160, 0.002), COLOR_RIB),
    box((-0.070, 0.0, 0.009), (0.006, 0.160, 0.002), COLOR_RIB),
    box((-0.050, 0.0, 0.009), (0.006, 0.160, 0.002), COLOR_RIB),
)
COVER_WINDOW = box((-0.050, 0.0, 0.009), (0.050, 0.080, 0.002), COLOR_WINDOW)
# Front lip on the left side (y 0.13), lifted by the left arm to open the lid: tall grippable tab (>= 12 mm) with top hook cap
COVER_EAR = box((-0.055, 0.115, 0.004), (0.110, 0.030, 0.008), COLOR_COVER)
COVER_STEM = box((-0.100, 0.130, 0.016), (0.020, 0.008, 0.016), COLOR_TAB)
COVER_CAP = box((-0.100, 0.130, 0.026), (0.024, 0.024, 0.004), COLOR_TAB)
LIP_LOCAL_AT = (-0.100, 0.130, 0.018)
LIP_SIZE = (0.024, 0.024, 0.020)
COVER_SHAPES = (COVER_PLATE, COVER_EAR, *COVER_RIBS, COVER_WINDOW, COVER_STEM, COVER_CAP)

# Angles:
BAY_OPEN_DEG = 50.0                    # bay is open once the lid angle >= 50 degrees
COVER_CLOSED_TOL_DEG = 3.0             # cover closed within 3 degrees of rest (0 degrees)
HOLD_OPEN_DEG = 85.0                   # demo opens and holds lid around 85 degrees

# -- roll: loose Mech body, 58 mm wide, 60 mm across, 25 mm core, 90 g, thermal paper ----------------------------------
ROLL_NAME = "rp_roll"
PAPER_ROLL_MASS_KG = 0.09
ROLL_FRICTION = 0.5
ROLL_RADIUS_M = 0.030
ROLL_WIDTH_M = 0.058
ROLL_CORE_RADIUS_M = 0.0125
ROLL_CORE_WIDTH_M = 0.059
TAIL_LEN_M = 0.12                      # from the roll's axis to the free end
TAIL_T_M = 0.003
ROLL_CENTRE_Z = ROLL_RADIUS_M          # axis height above the part origin
# Cylinder along local y (tilt rotates local z onto y).
_S = math.sqrt(0.5)
_QUAT_CYL_Y = (_S, -_S, 0.0, 0.0)
ROLL_CYL = Shape("cyl", (0.0, 0.0, ROLL_CENTRE_Z), (ROLL_RADIUS_M, ROLL_RADIUS_M, ROLL_WIDTH_M), COLOR_PAPER, _QUAT_CYL_Y)
ROLL_CORE = Shape("cyl", (0.0, 0.0, ROLL_CENTRE_Z), (ROLL_CORE_RADIUS_M, ROLL_CORE_RADIUS_M, ROLL_CORE_WIDTH_M), COLOR_CORE, _QUAT_CYL_Y)
ROLL_TAIL = box((TAIL_LEN_M / 2, 0.0, -TAIL_T_M / 2), (TAIL_LEN_M, ROLL_WIDTH_M, TAIL_T_M), COLOR_PAPER)
ROLL_SHAPES = (ROLL_CYL, ROLL_CORE, ROLL_TAIL)

# The free end's orange tape is a display marker (a 15 mm square, 0.6 mm thick, on the tail's top face) that follows the tail.
TAPE_MARKER_SIZE = (0.015, 0.015, 0.0006)
TAPE_MARKER_LOCAL = (TAIL_LEN_M - 0.0075, 0.0, 0.0004)
TAIL_TIP_LOCAL = (TAIL_LEN_M, 0.0, -TAIL_T_M / 2)
ROLL_AXIS_LOCAL = (0.0, 1.0, 0.0)
ROLL_UP_LOCAL = (0.0, 0.0, 1.0)

# -- stand: rubber plate 9 x 14 x 1 cm with ONE chock 8.5 mm behind the roll (the tail runs the other way) --------------------
STAND_NAME = "rp_stand"
STAND_XY = (0.10, -0.14)
STAND_MASS_KG = 25.0
STAND_FRICTION = 1.2
STAND_PLATE = box((0.0, 0.0, 0.005), (0.09, 0.14, 0.01), COLOR_STAND)
STAND_CHOCK = box((0.0, 0.0465, 0.021), (0.04, 0.016, 0.022), COLOR_STAND)         # inner face 38.5 mm from roll axis
STAND_SHAPES = (STAND_PLATE, STAND_CHOCK)
STAND_TOP_Z = 0.01
STAND_HALF = (0.045, 0.07)

# Roll at start: axis along world x, free end running toward -y (yaw -90 deg), on the plate with the tail under it.
ROLL_START_YAW = -math.pi / 2
ROLL_START_XY = STAND_XY
ROLL_START_ORIGIN_Z = STAND_TOP_Z + TAIL_T_M                                    # 0.013
ROLL_START_CENTRE_Z = ROLL_START_ORIGIN_Z + ROLL_CENTRE_Z                       # 0.043

# Correct seating: roll across the bay (axis along y), free end toward -x through the notch, tail lying on bay floor.
SEAT_XY = (0.210, 0.0)                                                          # middle of bay's x range
SEAT_ORIGIN_Z = BAY_FLOOR_Z + TAIL_T_M                                          # 0.008
SEAT_CENTRE_Z = SEAT_ORIGIN_Z + ROLL_CENTRE_Z                                   # 0.038
SEAT_TOL_X_M = 0.012
SEAT_TOL_Y_M = 0.008
SEAT_TOL_Z_M = 0.003
SEAT_MAX_AXIS_DEG = 5.0                # roll axis within this angle of the y line
SEAT_MAX_TILT_DEG = 5.0                # roll up direction within this angle of vertical
FEED_TIP_MAX_X = FRONT_FACE_X - 0.020  # free end at least 2 cm outside the front face (0.135)
FEED_TIP_MAX_ABS_Y = 0.020
FEED_TIP_MAX_Z = 0.030                 # free end lies on the chute, not hanging up
STAND_MAX_TILT_DEG = 10.0
STAND_Z_BAND_M = (-0.010, 0.012)       # roll centre z relative to ROLL_START_CENTRE_Z when back on stand

# Latched facts and thresholds.
LIFT_MIN_M = 0.03
OPEN_HOLD_STEPS = 20                   # 1 s at 20 Hz
FIXTURE_MOVE_M = 0.005
RELEASE_MIN_DIST_M = 0.03
FINGERS_OPEN_FRACTION = 0.9
STILL_SPEED_M_S = 0.01
STILL_WINDOW_STEPS = 20
STILL_MOTION_M = 0.001
SUCCESS_HOLD_TIME_S = 1.0
PAD_BELOW_TCP_M = 0.013
TCP_BELOW_LINK_M = 0.134
