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
PIN_R = 0.0030                # 6 mm punch tips, the hole's size (ISO 838)
SLUG_R = 0.0030               # the 6 mm hole
CUT = 0.0066                  # the square opening the card shows around each disc
# the card's collider leaves a wider opening (visual card stock fills the ring between), so a 6 mm tip up to 1.5 mm
# off its disc passes the card and pushes the disc; further off, it lands on the card stock and is blocked
CUT_CLEAR = 2 * (PIN_R + 0.0015) + 0.0006
DIE = 0.010                   # die holes in the bed: a disc up to 1.5 mm off the punch's axis still drops through
# The tips and discs collide as square prisms of about the same area as the round 6 mm shapes they show (5.4 mm tips,
# 6 mm discs): PhysX's flat cylinder-on-cylinder contact let a tip sink through a disc at under 20 N. Each disc is
# given 10 g (a real 6 mm card disc weighs about 10 mg) so the solver keeps the punch from passing through it; its
# weight, 0.1 N, is small next to the 30 N resistance.
TIP_SIDE = 0.0054
SLUG_SIDE = 2 * SLUG_R
SLUG_M = 0.010
TOL = 0.0015                  # a hole counts within 1.5 mm of its mark
# Cutting resistance. Each disc sits in the card on a slide along the card's normal, held by a force drive that pushes
# back with up to CUT_FORCE_N (it saturates after CUT_FORCE_N / CUT_STIFF = 0.15 mm). The disc is cut free only once
# the punch has pushed it the card's full thickness and its tip is PAST below the card's underside (scoring.py).
# 30 N per hole: paperboard's in-plane shear strength is about the geometric mean of its MD and CD tensile strengths
# (Fellers), about 16 N/mm for board of this weight, so a flat 6 mm punch cutting its whole 18.9 mm rim at once would
# need about 300 N. Desk punch tips are shear-ground (they enter point first), and the die-design rule for a
# shear-ground punch scales the flat force by p t / s: real thickness t 0.35 mm, penetration at fracture p 0.4-0.5,
# shear depth s 1.5-2 mm, so 0.07-0.12 x 300 N = 20-35 N. 30 N per hole: 60 N for the pair, about 17 N at the knob
# through the lever's 3.5:1 ratio. Measured in Isaac Sim with a straight-down force on the head (card on its marks):
# 65 N does not cut, 70 N cuts both (60 N for the discs plus the return spring); one disc pushed alone holds at
# 28 N and gives way at 33 N. Straight down on the knob: 20 N does not cut, 30 N does. A 50 mm press overshoot (policy.PRESS_EXTRA) stalled the demo short of the cut.
CUT_FORCE_N = 30.0
CUT_STIFF = 2.0e5             # N/m
CUT_DAMP = 20.0               # N s/m
PAST = 0.0005                 # tip at least 0.5 mm below the card's underside (0.3 mm below the bed's top face)
SLIDE_MIN = -(T + 0.004)      # the disc's slide: up to 5.5 mm down, never up out of the card

BED_X0 = 0.245                # the punch base runs x 0.245 .. 0.300
BED_X1 = 0.300
STAND_X0, STAND_X1 = 0.095, 0.245  # foam stand, level with the bed
# under the card's whole width at any start, and under the demo's fingertips beside the card's right edge (down to
# y -0.140); 2 mm short of the lever handle's path (y 0.080)
STAND_Y = (-0.145, 0.078)

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

# The card's start, drawn per seed: nominal centre x 18.5, y -2.0 cm (leading edge 44.5 mm short of the stop), then
# x +-0.8 cm, y +-1.2 cm and yaw +-10 deg, uniform. At the extremes the card stays on the stand (x >= 10.0 cm,
# y -11.4 to 7.4 cm), clear of the lever's path (y >= 8.0 cm) and 4 mm short of the housing's front face (x 26.2 cm).
CARD_START = np.array([0.185, -0.020, BED + T / 2])
START_DX, START_DY, START_YAW_DEG = 0.008, 0.012, 10.0
CARD_HOME = np.array([STOP_X - CARD_X / 2, 0.0, BED + T / 2])


def yaw_quat(yaw: float) -> np.ndarray:
    """Quaternion (x, y, z, w) of a turn ``yaw`` (rad) about z."""
    return np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])


def card_start(seed=None) -> tuple[np.ndarray, float]:
    """The card's start centre (xyz) and yaw (rad) for a trial seed; the nominal start when seed is None."""
    if seed is None:
        return CARD_START.copy(), 0.0
    rng = np.random.default_rng(int(seed))
    dx, dy = rng.uniform(-START_DX, START_DX), rng.uniform(-START_DY, START_DY)
    yaw = float(np.radians(rng.uniform(-START_YAW_DEG, START_YAW_DEG)))
    return CARD_START + np.array([dx, dy, 0.0]), yaw


def slug_offset(k: int) -> np.ndarray:
    """Disc k's centre in the card's frame (card frame at its centre; its leading edge at local +x)."""
    return np.array([CARD_X / 2 - 0.012, (-1, 1)[k] * PIN_Y, 0.0])


def pin_xy(k: int) -> np.ndarray:
    return np.array([PIN_X, (-1, 1)[k] * PIN_Y])

# closed fingertips, gripper straight down at yaw 0 (from the stock URDF's collision meshes): +-7.7 mm in x, +-4.0 mm in y
TIP_HALF_X, TIP_HALF_Y = 0.0077, 0.0040
