"""Every dimension of the hole-punch scene, in one place (m, station frame relative to the env origin: x forward from
the arm bases, y to the robot's left, z up). NumPy only: the scorer, the demo and the assets all read it."""

import numpy as np

TABLE = 0.7475
BED = TABLE + 0.020           # the punch bed and the foam stand, 20 mm above the table
BED_STEP = 0.0002             # the bed sits 0.2 mm below the stand, so the card steps down onto it and never snags
T = 0.0015                    # card thickness in the model (real 300 g/m2 card stock is about 0.35 mm)
CARD_X, CARD_Y = 0.105, 0.148  # card depth (toward the punch) and width
CARD_M = 0.0047

STOP_X = 0.282                # back stop face: the card's leading edge stops here
PIN_X = STOP_X - 0.012        # punches 12 mm in from the edge (ISO 838)
PIN_Y = 0.040                 # punches at y -40 and +40 mm (80 mm apart)
PIN_R = 0.0015                # punch tip radius: it lands on the pre-scored 6 mm disc to within 1.5 mm
SLUG_R = 0.0030               # the 6 mm hole
CUT = 0.0066                  # the square opening in the card that the disc sits in
DIE = 0.008                   # die holes in the bed
SLUG_M = 0.0005
TOL = 0.0015                  # a hole counts within 1.5 mm of its mark

BED_X0 = 0.245                # the punch base runs x 0.245 .. 0.300
BED_X1 = 0.300
STAND_X0, STAND_X1 = 0.095, 0.245  # foam stand, level with the bed
STAND_Y = (-0.115, 0.078)  # under the card's whole width; 2 mm short of the lever handle's path (y 0.080)

CEIL_Z0 = BED + 0.0060        # throat: 6 mm from the bed to the housing underside
CEIL_Z1 = BED + 0.0130
CEIL_X0 = 0.262               # housing front face

HEAD_REST_TIP = CEIL_Z0       # punch tips at rest, flush with the throat's ceiling
STROKE = 0.009                # head travel: tips end 6 mm below the bed, inside the dies
PIN_LEN = 0.019
BAR_Z0 = HEAD_REST_TIP + PIN_LEN  # head crossbar 6 mm thick
BAR_X = (PIN_X - 0.004, PIN_X + 0.004)
BAR_Y = (-0.050, 0.100)
HEAD_M = 0.15
SPRING_K = 500.0              # N/m; preloaded 10 mm against the top stop
SPRING_PRELOAD = 0.010
SPRING_DAMP = 20.0

LEVER_PIVOT = np.array([0.295, 0.090, BAR_Z0 + 0.006 + 0.004])  # pivot axis along y, behind the punches
LEVER_FRONT_X = 0.195
LEVER_Y = (0.080, 0.100)  # clear of the card (|y| <= 74 mm) as the handle end comes down
LEVER_THICK = 0.006
LEVER_M = 0.10
LEVER_LIMIT_DEG = 24.0
KNOB_X = (0.195, 0.225)       # red handle knob on the lever's end, widened toward +y for a finger to press
KNOB_Y = (0.080, 0.125)
PRESS_POINT = np.array([0.208, 0.1025])  # where a finger presses the knob

CARD_START = np.array([STOP_X - 0.040 - CARD_X / 2, -0.015, BED + T / 2])  # edge 40 mm short, 15 mm off centre
CARD_HOME = np.array([STOP_X - CARD_X / 2, 0.0, BED + T / 2])


def slug_offset(k: int) -> np.ndarray:
    """Disc k's centre in the card's frame (card frame at its centre; its leading edge at local +x)."""
    return np.array([CARD_X / 2 - 0.012, (-1, 1)[k] * PIN_Y, 0.0])


def pin_xy(k: int) -> np.ndarray:
    return np.array([PIN_X, (-1, 1)[k] * PIN_Y])

# closed fingertips, gripper straight down at yaw 0 (from the stock URDF's collision meshes): +-7.7 mm in x, +-4.0 mm in y
TIP_HALF_X, TIP_HALF_Y = 0.0077, 0.0040
