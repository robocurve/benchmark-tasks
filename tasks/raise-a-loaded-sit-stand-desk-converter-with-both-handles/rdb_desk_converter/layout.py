"""Every number of the sit-stand desk converter, its load and its place on the table, with its status.

Frame: the station's tabletop frame. x forward from the line through the two arm bases, y to the robot's left,
z up from the table top, metres. The converter's keyboard tray faces the robot, so the converter's "front"
(the user's side) points toward -x, and the work surface moves toward -x as it rises.

Status words used below: "published" (read on the maker's page or in a review), "derived" (computed from published
figures), "estimate" (no published figure; how to measure it is in the README).

NumPy only.
"""

import math

import numpy as np

IN = .0254

# -- the product: VariDesk Pro Plus 30 (Vari) ---------------------------------------------------------------------
WIDTH = 30 * IN                     # 0.762 work surface width                       published (vari.com spec table)
DEPTH = 29.75 * IN                  # 0.756 surface depth, both tiers                published (vari.com spec table)
BASE_W = BASE_D = 24 * IN           # 0.610 base footprint                           published (vari.com dimension captions)
TOTAL_DEPTH_RAISED = 41.75 * IN     # 1.060 total depth (raised)                     published caption; reading it as
                                    #   "back of base to front of tray at the top setting" is ours (derived)
H_LOW, H_HIGH = 4.5 * IN, 17.5 * IN  # 0.114, 0.445 work surface height range      published (spec sheet via review)
SETTINGS = 11                       # height settings                                published
PRODUCT_MASS = 21.04                # 46.39 lb                                       published
RATED_LOAD = 15.9                   # 35 lb                                          published
TRAY_DROP = 3.5 * IN                # 0.089 keyboard tray below the work surface     published (review)
TRAY_D = 12.5 * IN                  # 0.318 keyboard tray depth (middle)             published (review)
UPPER_D = DEPTH - TRAY_D            # 0.438 upper tier depth                         derived
SLAB_T = .019                       # work surface board thickness                  estimate
TRAY_T = .016                       # keyboard tray board thickness                 estimate

# -- the lift: a parallelogram ("rowing lift"); the top translates on a circle of radius L ----------------------
# Rise 0.331 m (published height range). Forward travel over the whole rise: 0.305 m, derived from the two published
# depths (29.75 in surface, 41.75 in total raised) on the assumption that at the lowest setting the back of the
# surface is over the back of the base. First setting: arm 5 degrees above level (estimate). Settings equally
# spaced in arm angle (the lock's notches are holes on a circle round the arm's upper pivot; estimate).
RISE = H_HIGH - H_LOW
FORWARD = TOTAL_DEPTH_RAISED - DEPTH
THETA_1 = math.radians(5.)


def _solve_arc():
    lo, hi = math.radians(40.), math.radians(120.)
    for _ in range(80):
        mid = (lo + hi) / 2
        ratio = (math.cos(THETA_1) - math.cos(mid)) / (math.sin(mid) - math.sin(THETA_1))
        lo, hi = (mid, hi) if ratio < FORWARD / RISE else (lo, mid)
    t11 = (lo + hi) / 2
    return t11, RISE / (math.sin(t11) - math.sin(THETA_1))


THETA_11, ARM_L = _solve_arc()       # about 79.4 deg and 0.369 m                   derived
STEP = (THETA_11 - THETA_1) / (SETTINGS - 1)
START_SETTING, GOAL_SETTING = 4, 9


def theta(setting):
    """Arm angle above level (rad) at a setting 1..11 (may be fractional)."""
    return THETA_1 + (setting - 1) * STEP


def setting_of(angle):
    return 1 + (angle - THETA_1) / STEP


# -- top frame ---------------------------------------------------------------------------------------------------
# Origin at the middle of the upper tier's work surface (its top face), axes parallel to the station's.
SLAB = ((-UPPER_D / 2, UPPER_D / 2), (-WIDTH / 2, WIDTH / 2), (-SLAB_T, 0.))
TRAY = ((-UPPER_D / 2 - TRAY_D, -UPPER_D / 2), (-WIDTH / 2, WIDTH / 2), (-TRAY_DROP - TRAY_T, -TRAY_DROP))
HANGER_Y = .20                      # the tray hangs from two brackets this far either side of the middle   estimate
UPPER_PIVOT = np.array((.02, 0., -.045))   # the primary arms' upper pivot in the top frame                estimate
SURFACE_ABOVE_PIVOT = -UPPER_PIVOT[2]

# Handles: one under each side edge of the upper tier, near its front corner, with the release paddle below the
# edge. Sizes and place are estimates from product photographs (measure with a ruler: see README).
HANDLE_X = -UPPER_D / 2 + .045      # paddle centre, 45 mm behind the upper tier's front edge             estimate
# How deep a hand can go: with the jaws open, the YAM finger roots (61.5 mm behind the fingertip centre, measured on
# the URDF collision hulls) span the board's thickness, so the fingertip centre can be at most 61.5 mm behind the
# board's front edge. Isaac run 2 had the grasp 80 mm in and stopped 18.5 mm short, pads never on the paddle.
FINGER_ROOT = .0615
GRASP_DEPTH = .048                  # fingertip centre behind the front edge at the grasp (13.5 mm to spare)
HANDLE_Y = WIDTH / 2 - .029         # 0.352 paddle centre from the middle                                  estimate
PADDLE_HALF = (.035, .0185, .005)   # paddle plate: 70 mm along the edge, 37 mm deep, 10 mm thick          estimate
PADDLE_GAP = .020                   # paddle's top to the board's underside, released                      estimate
PADDLE_TRAVEL = .016                # squeeze travel                                                        estimate
PADDLE_FORCE = (3.3, 12.0)          # N to start moving it, N at full squeeze (spring)                      estimate
PADDLE_TOP_REST = -SLAB_T - PADDLE_GAP

# Lock: per side, a sprung pin on the paddle's latch drops into one of 11 notches (holes in the published product;
# radial slots here) on a toothed quadrant fixed to that side's primary arm, round the arm's upper pivot. The
# latch moves straight up and down in the top, so squeezing a paddle lifts only its own side's pin out.
NOTCH_R = (.093, .118)              # radial extent of the teeth round the upper pivot                     estimate
PIN_R = (.097, .107)                # pin's radial extent when engaged (10 mm into the slot)              estimate
PIN_TANGENT = .0055                 # pin width along the arc                                               estimate
TOOTH_TANGENT = .0050               # tooth width (slots about 8.4 mm wide at mid radius)                estimate
LOCK_Y = (.260, .272)               # lock plane: |y| band of teeth and pins                                estimate
assert PIN_R[0] - PADDLE_TRAVEL < NOTCH_R[0] - .001          # a squeezed pin clears the teeth
assert GRASP_DEPTH < FINGER_ROOT - .008 and abs(GRASP_DEPTH - (HANDLE_X + UPPER_D / 2)) < PADDLE_HALF[0] - .02

# -- masses (kg) and centres; the product total is published, the split is an estimate ---------------------------
BASE_MASS = 6.0
TOP_MASS = 12.2                     # boards, frame, tray, brackets, handles
ARM_MASS = 1.9                      # two primary arms and their cross tube (centre at mid-arm)
REAR_ARM_MASS = .5                  # two secondary arms
LATCH_MASS = .15                    # each
assert abs(BASE_MASS + TOP_MASS + ARM_MASS + REAR_ARM_MASS + 2 * LATCH_MASS - PRODUCT_MASS) < .3
TOP_COM = np.array((-.08, 0., -.03))     # in the top frame (the tray pulls it forward)                  estimate

# -- the load (task definition) -------------------------------------------------------------------------------------
MONITOR_MASS = 4.0                  # a 24 in office monitor on its own stand (typical listed 3.5-4.5 kg)
MONITOR_BASE = (.22, .18, .015)     # stand foot, full sizes
MONITOR_PANEL = (.045, .540, .325)  # panel depth, width, height (24 in, 16:9, with bezel)
MONITOR_PANEL_BOTTOM = .110         # panel's lower edge above the foot's underside
MONITOR_AT = np.array((.095, 0.))   # foot centre, top frame (behind the handles, centred)
KEYBOARD_MASS = .60
KEYBOARD_SIZE = (.135, .440, .025)  # depth, width, height (full size wired)
KEYBOARD_AT = np.array((-UPPER_D / 2 - TRAY_D / 2 - .005, 0.))
LOAD_MASS = MONITOR_MASS + KEYBOARD_MASS

# -- the counterbalance: a torsion spring on the lift's DOF, k (theta_t - theta) ---------------------------------
# No lift-force figure is published. Reviews: "lifts with little effort thanks to spring assistance" but
# "lowering it takes some muscle, especially to get it in the bottom-most slot" (PCWorld); the weight is largely
# offset by the spring "depending on how heavy your monitor is" (Impulse Gamer). The spring here is set so that
# with this task's load the two hands together must push up along the path by 3 N at setting 4 and 2 N at setting 10
# (up to 11.3 N in between, see net_push), while at the lowest settings the spring is stronger than the weight.
# Estimate; measure with a luggage scale at each handle (README).
SPRING_FIT = ((4, 3.0), (10, 2.0))   # (setting, net push N)
G = 9.81


def gravity_torque(angle):
    """Torque (N m) the weight of everything that moves exerts on the lift's DOF, at arm angle ``angle``."""
    moving = TOP_MASS + 2 * LATCH_MASS + LOAD_MASS
    return G * math.cos(angle) * ARM_L * (moving + (ARM_MASS + REAR_ARM_MASS) / 2)


def _spring():
    (sa, fa), (sb, fb) = SPRING_FIT
    a, b = theta(sa), theta(sb)
    ta, tb = gravity_torque(a) - fa * ARM_L, gravity_torque(b) - fb * ARM_L
    k = (ta - tb) / (b - a)
    return k, a + ta / k                     # stiffness (N m/rad), angle where the spring torque is zero


SPRING_K, SPRING_ZERO = _spring()
LIFT_DAMPING = 3.0                  # N m s/rad, about 0.6 N along the path at 3 cm/s                      estimate


def spring_torque(angle):
    return SPRING_K * (SPRING_ZERO - angle)


def net_push(angle):
    """Force (N, both hands together) needed along the path to hold the loaded top still: + up, - down."""
    return (gravity_torque(angle) - spring_torque(angle)) / ARM_L


# -- placement on the table ---------------------------------------------------------------------------------------
GRASP_IN_TOP = {side: np.array((-UPPER_D / 2 + GRASP_DEPTH, sign * HANDLE_Y, (PADDLE_TOP_REST + PADDLE_TRAVEL - 2 * PADDLE_HALF[2]) / 2))
                for side, sign in (("left", 1.), ("right", -1.))}
HANDLE_X_AT_START = .468            # the grasp points at setting 4 (slot centre), station x; same converter place as run 2
ARC_DIR = lambda a: np.array((math.cos(a), 0., math.sin(a)))     # noqa: E731
_T4_X = HANDLE_X_AT_START - GRASP_IN_TOP["left"][0]
PIVOT = np.array((_T4_X + UPPER_PIVOT[0] - ARM_L * math.cos(theta(START_SETTING)), 0.,
                  H_LOW - SURFACE_ABOVE_PIVOT - ARM_L * math.sin(THETA_1)))   # lift's lower pivot C, station frame


def upper_pivot(angle):
    return PIVOT + ARM_L * ARC_DIR(angle)


def top_origin(angle):
    """The top frame's origin (middle of the work surface) in the station frame."""
    return upper_pivot(angle) - UPPER_PIVOT


def surface_height(angle):
    return float(top_origin(angle)[2])


def grasp_point(side, angle):
    return top_origin(angle) + GRASP_IN_TOP[side]


# Base: back edge under the surface's back edge at setting 1.
BASE_BACK_X = float(top_origin(THETA_1)[0] + UPPER_D / 2)
BASE_X = (BASE_BACK_X - BASE_D, BASE_BACK_X)
BASE_RAIL_Y = (.300, .340)          # the base's two side rails (flat steel), |y|                         estimate
BASE_RAIL_H = .012
REAR_OFFSET = np.array((-.030, 0., .055))   # secondary arm's pivots relative to the primary's (both ends)  estimate
ARM_DISPLAY_Y = .300                # |y| of the arms                                                       estimate
LIFT_LIMITS = (theta(3) - .3 * STEP, THETA_11 + math.radians(1.))  # model's travel (the real unit goes to setting 1)

# Green card: stands on the table right of the converter; its top edge is at the work surface's height at setting 9.
CARD_SIZE = (.003, .060, round(surface_height(theta(GOAL_SETTING)), 4))
CARD_AT = np.array((.80, -.42))


def _rect(centre, angle, half_radial, half_tangent):
    u = np.array((math.cos(angle), math.sin(angle)))
    v = np.array((-math.sin(angle), math.cos(angle)))
    return np.array([centre + a * half_radial * u + b * half_tangent * v for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])


def _gap2d(a, b):
    """Separation of two convex quadrilaterals (x-z plane) along their best edge normal (negative: overlap)."""
    best = -1e9
    for poly in (a, b):
        for i in range(4):
            e = poly[(i + 1) % 4] - poly[i]
            n = np.array((-e[1], e[0])) / np.linalg.norm(e)
            pa, pb = a @ n, b @ n
            best = max(best, pb.min() - pa.max(), pa.min() - pb.max())
    return float(best)


def pin_gap(offset):
    """Smallest gap (m) between an engaged pin and the teeth of its quadrant when the arm angle is ``offset`` (rad)
    from a slot's centre. Pure geometry in the lock plane (x-z), the same boxes as the asset."""
    r_tooth, r_pin = (NOTCH_R[0] + NOTCH_R[1]) / 2, (PIN_R[0] + PIN_R[1]) / 2
    teeth = [_rect(r_tooth * np.array((math.cos(a), math.sin(a))), a, (NOTCH_R[1] - NOTCH_R[0]) / 2, TOOTH_TANGENT / 2)
             for a in (-math.pi / 2 + STEP / 2, -math.pi / 2 - STEP / 2)]
    a = -math.pi / 2 - offset
    pin = _rect(r_pin * np.array((math.cos(a), math.sin(a))), a, (PIN_R[1] - PIN_R[0]) / 2, PIN_TANGENT / 2)
    return min(_gap2d(pin, t) for t in teeth)


def _seat_play():
    lo, hi = 0., -STEP / 2
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if pin_gap(mid) > 0. else (lo, mid)
    return -lo


# Slot play: with both pins in a notch the arm can turn this far either side of the slot's centre before a pin's side
# meets a tooth (0.0117 rad, 0.67 deg). The loaded top (net weight down from setting 4 to 10) rests on the pins'
# lower contact, SEAT_PLAY below the slot's centre: setting k seated reads k - SEAT_SETTINGS (about k - 0.089).
# Isaac run 1 settled at lift -0.0117 rad from the slot centre, exactly this.
SEAT_PLAY = _seat_play()
SEAT_SETTINGS = SEAT_PLAY / STEP


def seated(setting):
    """Arm angle of the loaded top resting in notch ``setting``."""
    return theta(setting) - SEAT_PLAY


START_ANGLE = seated(START_SETTING) + 5e-5   # authored here: the seated pose, pins 5 micrometres off the teeth


def notch_angles():
    """Angle (rad, about the upper pivot, in the arm's own frame as authored) of each slot's centre line.

    The pin points straight down from the upper pivot in the top frame. The arm turns by +theta relative to the top
    (the top stays level), so slot k lies at -90 deg - (theta_k - START_ANGLE) in the arm's authored frame, measured
    in the x-z plane from +x toward +z. At the start the pin is therefore SEAT_PLAY off the centre of slot 4, resting
    on its lower contact.
    """
    return [-math.pi / 2 - (theta(k) - START_ANGLE) for k in range(1, SETTINGS + 1)]


def tooth_angles():
    return [-math.pi / 2 - (theta(k + .5) - START_ANGLE) for k in range(0, SETTINGS + 1)]


def rot_xz(a):
    """Rotation by ``a`` in the x-z plane (from +x toward +z): a rotation about -y."""
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, 0., -s), (0., 1., 0.), (s, 0., c)))
