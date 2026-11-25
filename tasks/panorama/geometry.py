"""Dimensions and layout of the phone-panorama task.

Station frame (the robot's): metres from the midpoint between the arm bases, x forward (away from the arms), y to the
robot's left, z up. TOP is the table top's world height; names ending _Z are world heights; "heading" is an angle in
degrees about z from +x (counterclockwise seen from above).

The phone is an Apple iPhone 12 at the size of Apple's dimensional drawing (2020-10-16). Its frame: origin at the
body's centre; x out of the screen (the screen normal), z along the long side toward the top edge, y = z x x, so
seen from the front +y is the viewer's right. The main (wide) camera is at the back's top corner on the +y side (the
side button's edge); the volume buttons are on the -y edge. The camera looks along -x; its heading is the screen's
heading plus 180 degrees.

The three boards stand on a fan round the cross on the mat: each board's printed centre line, its rod and the cross's
centre lie on one line (the board's sight line). Seen from a lens above the cross, the rod hides the line; seen from a
lens off to the side of a sight line, the near rod and the far line separate (parallax).

Values marked "estimate" are not published by a maker or fixed by the build.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475                        # the table top (world z)
PHYSICS_HZ = 240
CONTROL_HZ = 20
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)

# -- the phone (free): Apple iPhone 12 ------------------------------------------------------------------------------
PH, PW, PT = 0.14671, 0.07152, 0.00739    # length (z), width (y), thickness (x)
PHONE_MASS = 0.164
CORNER_R = 0.0105                   # the outline's corner radius in front view (drawn as a chamfer-free box in sim)
LENS_FROM_TOP = 0.01273             # wide lens centre from the top edge and from the camera-side edge (+y)
LENS_FROM_SIDE = 0.01273
LENS_D = 0.01210                    # lens glass
LENS_PROUD = 0.00151                # back glass to rear camera glass
PLATEAU_TOP, PLATEAU_SIDE = 0.03630, 0.03413   # the camera plateau from the top edge and from the +y edge
PLATEAU_PROUD = 0.00073             # back glass to camera plateau
ULTRA_FROM_TOP = 0.02701            # the ultra-wide lens (same column)
FLASH = (0.01987, 0.02528)          # flash centre: from the top, from the +y edge
# button bands (from the top edge): volume up and down on the -y edge, the side button on the +y edge
VOL_UP = (0.03931 - 0.00565, 0.03931 + 0.00565)
VOL_DOWN = (0.05331 - 0.00565, 0.05331 + 0.00565)
SIDE_BTN = (0.04631 - 0.00890, 0.04631 + 0.00890)
DISPLAY_INSET = 0.00347             # housing to the display's active area, all round
# the on-screen shutter (portrait Camera app): centre on the centre line, 21 mm above the bottom edge (estimate from
# a 1170 x 2532 screenshot at 0.0552 mm/px), about 12 mm across
SHUTTER_UP = 0.021
SHUTTER_D = 0.012
# the camera corner the robot must keep off during a capture: every outer face of the phone (back, top edge, side
# edge) within CORNER_TOP of the top edge and CORNER_SIDE of the +y edge (the plateau, the back glass round it, and the
# top and side edges along it)
CORNER_TOP, CORNER_SIDE = 0.037, 0.035


def lens_local() -> np.ndarray:
    """The wide lens's centre on the back glass, phone frame."""
    return np.array([-PT / 2, PW / 2 - LENS_FROM_SIDE, PH / 2 - LENS_FROM_TOP])


def shutter_local() -> np.ndarray:
    """The on-screen shutter's centre, on the glass, phone frame."""
    return np.array([PT / 2, 0.0, -PH / 2 + SHUTTER_UP])


# -- the desk stand (fixed): an aluminium phone stand, Lamicall type ------------------------------------------------
STAND_XY = (0.340, 0.250)           # the base's centre
STAND_YAW = 0.0                     # the phone in it faces +x (screen heading 0)
RECLINE = 20.0                      # degrees back from vertical
BASE_L, BASE_W, BASE_T = 0.090, 0.080, 0.006       # base plate (x, y, thickness)
SEAT_UP = 0.030                     # the phone's bottom edge rests this high (on the seat)
LIP_H = 0.008                       # the lip in front of the bottom edge stands this high above the seat
BACK_L = 0.100                      # the reclined back plate's length up its slope
PLATE_T = 0.004                     # plate thickness
SLOT = 0.012                        # the seat's depth from the back plate to the lip (room for a phone in a case)
# the phone in the stand: its bottom edge's mid-line on the seat, its back on the back plate
STAND_SEAT_X = STAND_XY[0] + 0.012  # the bottom edge's back corner line (x) on the seat


def phone_in_stand():
    """(position of the phone's centre, rotation) resting in the stand, table frame (z above the table): bottom edge
    on the seat, back on the plate."""
    a = math.radians(RECLINE)
    R = rot_z(math.radians(STAND_YAW)) @ rot_y(-a)        # long axis leans back toward -x, screen up 20 degrees
    up, n = R[:, 2], R[:, 0]
    # the back-bottom edge (local x -PT/2, z -PH/2) sits on the seat at STAND_SEAT_X (table frame: z above the
    # table), 0.3 mm off the plate and the seat so that it settles onto them
    corner = np.array([STAND_SEAT_X, STAND_XY[1], SEAT_UP])
    centre = corner + R @ np.array([PT / 2 + 0.0003, 0.0, PH / 2 + 0.0003])
    return centre, R


# -- the boards, rods and cross (fixed) -------------------------------------------------------------------------------
CROSS = (0.300, 0.120)              # the white tape cross's centre on the mat
LENS_H = 0.185                      # the stripe's centre and the ticks' midpoint above the table: the lens height
BEARINGS = (108.0, 43.0, -22.0)     # each board's sight line, from the cross (a 130 degree fan)
BOARD_R = 0.450                     # the sheet's face from the cross's centre
ROD_R = 0.300                       # the rod's axis from the cross's centre
SHEET_W, SHEET_H = 0.210, 0.297     # A4 portrait
SHEET_Z0 = 0.025                    # the sheet's bottom edge above the table (in its T-base holder)
LINE_W = 0.005                      # the printed centre line
TICKS = (0.155, 0.215)              # the two ticks across the line, above the table: the judged span between them
TICK_L = 0.030
STRIPE_H = 0.020                    # a coloured horizontal stripe across the sheet, centred at LENS_H
DIGIT_H = 0.060                     # the board's number, upper left, on a coloured square
HOLDER_W, HOLDER_D, HOLDER_H = 0.230, 0.080, 0.012  # clear acrylic T-base: foot (width along the sheet, depth, height)
HOLDER_BACK = 0.004                 # the acrylic pocket in front of and behind the sheet
ROD_D = 0.006                       # straight 6 mm round rod
ROD_LEN = 0.300
ROD_BLOCK = (0.050, 0.050, 0.018)   # drilled wooden block, taped down
MAT = (0.300, 0.400, 0.003)         # black cloth mouse pad: x, y, thickness
MAT_XY = (0.330, 0.170)
CROSS_ARM, CROSS_W = 0.020, 0.003   # white tape cross: two 20 x 3 mm strips
BOARD_COLOURS = ((200, 40, 40), (40, 150, 60), (40, 80, 200))


def bearing_vec(b_deg: float) -> np.ndarray:
    b = math.radians(b_deg)
    return np.array([math.cos(b), math.sin(b)])


def board_xy(i: int) -> np.ndarray:
    return np.asarray(CROSS) + BOARD_R * bearing_vec(BEARINGS[i])


def rod_xy(i: int) -> np.ndarray:
    return np.asarray(CROSS) + ROD_R * bearing_vec(BEARINGS[i])


def board_half_angle() -> float:
    """Degrees either side of a board's sight line that its sheet spans, seen from the cross."""
    return math.degrees(math.atan2(SHEET_W / 2, BOARD_R))


# -- the camera and the capture (the app is not simulated: these rules stand in for it) ----------------------------
FRAME_W, FRAME_H = 52.0, 60.0       # portrait Pano frame, degrees (26 mm-equivalent lens on a 4:3 sensor; estimate)
SLOW_DPS = 40.0                     # "Slow down": the heading turns faster than this (deg/s) over SLOW_WINDOW_S
SLOW_WINDOW_S = 0.25                # (estimate)
DRIFT_DEG = 5.0                     # off the centre line: the view drifts more than this, tilt plus height change seen
DRIFT_DIST = 0.45                   # at the boards' distance (estimate)
LEVEL_PITCH, LEVEL_ROLL = 3.0, 4.0  # level: optical axis within 3 degrees of horizontal, roll within 4 (estimate)
BACK_DEG = 5.0                      # turning back more than this ends the capture unsaved
AUTO_END_DEG = 240.0                # the app ends and saves a capture at this sweep
TAP_MAX_S = 0.5                     # a tap: an end face on the glass this long or less,
TAP_R = 0.006                       #   the touch staying within this of the shutter's centre throughout
TOUCH_GAP = 0.0005                  # an end face this close to the glass touches it
PRESS_N = 3.0                       # a button press: this force or more on a button band (estimate)
ALIGN_T = 0.006                     # aligned: the lens this close to the board's sight line (= align_threshold())
SEEN_RANGE = 1.5                    # robot parts this close in the panorama's view show in it
STILL_S = 1.0                       # set down: still this long
CLEAR = 0.030                       # set down: both grippers this far from the phone
FALL = 0.050                        # a fall: the phone drops this far with no gripper on it

# The line between the ticks shows when the rod no longer covers it by a sliver 1 mm wide or more (on the sheet).
# Seen from a lens d off the sight line, the line moves across the rod by d (1/ROD_R - 1/BOARD_R) in angle. It shows
# once that exceeds the rod's half-width angle less the line's half-width angle plus the 1 mm sliver:
#   d >= (ROD_D / 2 / ROD_R - LINE_W / 2 / BOARD_R + SLIVER / BOARD_R) / (1 / ROD_R - 1 / BOARD_R)
SLIVER = 0.001


def align_threshold(rod_d: float = ROD_D, sliver: float = SLIVER) -> float:
    k = 1 / ROD_R - 1 / BOARD_R
    return (rod_d / 2 / ROD_R - LINE_W / 2 / BOARD_R + sliver / BOARD_R) / k


# -- the robot's hands ----------------------------------------------------------------------------------------------
TIP = 0.013                         # fingertips past the tool point
PAD_W = 0.0264                      # the fingertip section's width (yam_arena collision hulls, 13 mm either side)
GAP_MM = 94.0                       # the pads' gap at grip 1.0
HAND_LINKS = ("gripper", "tip_left", "tip_right")
ROBOT_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")


# -- helpers ----------------------------------------------------------------------------------------------------------
def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def wrap(a: float) -> float:
    """Degrees into [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def phone_upright(screen_heading_deg: float) -> np.ndarray:
    """The phone's rotation standing upright, its screen facing ``screen_heading_deg``."""
    return rot_z(math.radians(screen_heading_deg))


def camera_axes(R: np.ndarray):
    """(optical axis, image right, image up) in the world for a phone at rotation R: the camera looks along -x, the
    way a person holding the phone looks; image right is the phone's +y (their right) and image up its +z."""
    return -R[:, 0], R[:, 1], R[:, 2]


def heading_of(v) -> float:
    return math.degrees(math.atan2(v[1], v[0]))
