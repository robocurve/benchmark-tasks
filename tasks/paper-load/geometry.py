"""Dimensions and layout of the paper-refill task.

Floor bay (no table): the station's arms on steel pedestals, mounts 76.25 cm above the floor. World frame: metres from
the midpoint between the arm mounts, x forward, y to the robot's left, z up. TOP is the side table's top (world z);
names ending _H are heights above it.

The printer is a Canon PIXMA TS3720 all-in-one (outline, mass and Canon's loading steps from its TS3700-series
manual) on an IKEA LACK side table (55 x 55 cm, 45 cm tall), front toward the arms. Positions inside the printer
(the rear tray's slot, right side and guide, the paper support, the keys) are estimates.

The paper is five sheets of 20 lb (75 g/m2) Letter copy paper, 279.4 x 215.9 mm, 4.5 g each, simulated as one limp
stack: N_STRIPS rigid strips across the stack's short side (each the stack's full 215.9 mm wide), joined by sprung
hinges, so held by a short edge it hangs and droops as paper does; its sheets never separate. The stack's own
coordinates: ``a`` along its long side from the edge of strip 0 (0 .. 279.4 mm), ``b`` along the strips (-107.95 ..
+107.95 mm).
"""

from __future__ import annotations

import math

import numpy as np

FZ = -0.0125                        # the floor (world z), yam_arena station.FLOOR_Z
STAND_H = 0.45                      # IKEA LACK side table: 45 cm tall
TOP = FZ + STAND_H                  # the table's top (world z): the printer stands on it
PHYSICS_HZ = 240
CONTROL_HZ = 20
LACK = (0.55, 0.55, 0.05)           # top 55 x 55, 5 thick; legs 5 x 5
LACK_X0 = 0.125                     # its near edge (just beyond the pedestals' feet, which reach x 0.12)
LACK_Y = -0.05                      # centred on the printer

# -- the printer (Canon PIXMA TS3720; outline and mass from Canon, the rest estimates) ----------------------------------
# The parts are laid out for a front face at x 0.170 and shifted by DX, so the front stands at x 0.10.
DX = 0.100 - 0.170
PR_X = (0.170 + DX, 0.497 + DX)     # 327 deep, front face toward the robot
PR_Y = (-0.2675, 0.1675)            # 435 wide, centred on y -0.050
PR_H = 0.145
PR_MASS = 4.0
FRAME_H = 0.1165
GLASS_H = 0.115
GLASS_X = (0.215 + DX, 0.435 + DX)
GLASS_Y = (-0.225, 0.075)
CORNER = (GLASS_X[1], GLASS_Y[1])
COVER_X = (0.185 + DX, 0.455 + DX)  # the document cover (shut throughout)
COVER_Y = (-0.255, 0.085)
COVER_T = 0.0175
COVER_TOP_H = 0.135
COVER_LOW_H = COVER_TOP_H - COVER_T
HINGE = (0.455 + DX, COVER_TOP_H)
COVER_MASS = 0.35
COVER_STOP = math.radians(100.0)
COVER_DAMP = 0.02
LIP_X = (0.170 + DX, 0.205 + DX)
LIP_FLOOR_H = 0.085
PANEL_X = (0.176 + DX, 0.400 + DX)
PANEL_Y = (0.092, 0.161)
BTN = 0.013
BTN_PROUD = 0.002
BTN_TRAVEL = 0.0015
BTN_PRESSED = 0.0008
BTN_RELEASED = 0.0003
BTN_K = 2500.0                      # N/m: 2 N to press it 0.8 mm (estimate)
BLACK_XY = (0.200 + DX, 0.140)
COLOR_XY = (0.200 + DX, 0.114)
OK_XY = (0.262 + DX, 0.1145)        # OK, beside Paper Select on the panel (estimate)
BTN_TOP_H = COVER_TOP_H + BTN_PROUD
TRAY_X = (0.060 + DX, 0.170 + DX)   # the paper output tray, pulled out
TRAY_Y = (-0.135, 0.035)
TRAY_H = 0.030

# the rear tray (behind the document cover's hinge): a slot along y, its right end the right side
REAR_X = (0.455 + DX, PR_X[1])      # the rear housing
SLOT_X = (0.458 + DX, 0.480 + DX)   # the slot the paper stands in (22 mm)
SLOT_FLOOR_H = 0.112                # its floor
SLOT_Y = (-0.160, 0.135)            # from the right side (y -0.160) past the guide's widest
RIGHT_Y = SLOT_Y[0]
# the paper support: a plate on a hinge along the housing's back; shut it lies forward over the rear tray, raised it
# leans back 20 degrees from upright on its stop, and the paper leans on it
SUPPORT_HINGE = (0.482 + DX, 0.150)
SUPPORT_L, SUPPORT_T = 0.110, 0.003
SUPPORT_Y = (-0.170, 0.080)
SUPPORT_STOP = math.radians(110.0)
SUPPORT_MASS = 0.05
SUPPORT_DAMP = 0.02
SUPPORT_OPEN_DEG = 95.0             # "raised": past upright
SUPPORT_START = SUPPORT_STOP        # raised at the start, as when a printer runs out of paper mid-job
# the paper guide: a slider in the slot, its face toward the paper; widest at GUIDE_OPEN_Y
GUIDE_OPEN_Y = 0.100
GUIDE_T = 0.008
GUIDE_H = (SLOT_FLOOR_H + 0.001, 0.158)    # its raised grip stands 13 mm above the housing (Canon's figure)
GUIDE_GRIP_H = 0.012
GUIDE_TRAVEL = 0.052                # it can close to y 0.048 (below the paper's edge at 0.0559)
GUIDE_DAMP = 3.0                    # N s/m: it stays where it is pushed
GUIDE_MASS = 0.01

# -- the paper: five sheets of 20 lb Letter as one limp stack -------------------------------------------------------------
N_SHEETS = 5
SHEET_L = 0.2159                    # along b (y at the start), the strips' length
SHEET_W = 0.2794                    # along a (x at the start)
N_STRIPS = 9
STRIP_W = SHEET_W / N_STRIPS        # 31.0 mm
COLL_T = 0.002                      # each strip's collider (the stack is drawn 0.5 mm thick in its middle)
DRAW_T = 0.0005
REAL_MASS = 0.0045 * N_SHEETS       # 22.5 g
SCALE = 8.0                         # mass and stiffness both x8
STRIP_MASS = REAL_MASS * SCALE / N_STRIPS
BEND_S = 0.20e-3 * N_SHEETS         # five sheets' bending stiffnesses add (they slide on one another), N m per m
HINGE_K = SCALE * BEND_S * SHEET_L / STRIP_W
HINGE_C = 5.0e-4
HINGE_LIMIT = math.radians(170.0)
FOLDED_DEG = 150.0
SHEET_MU = 1.0
PRINTER_MU = 0.25

# the start: portrait on the shut document cover, its near short edge 40 mm over the cover's front edge
START_A0_X = COVER_X[0] - 0.040
START_Y = RIGHT_Y + 0.015 + SHEET_L / 2   # its right edge 15 mm left of the rear tray's right side
START_Z = TOP + COVER_TOP_H + COLL_T / 2 + 0.0002

# -- scoring -----------------------------------------------------------------------------------------------------------
SEATED_TOL = 0.003                  # loaded: the stack's lowest edge within 3 mm of the slot's floor ...
RIGHT_TOL = 0.0005                  # ... and its right edge within 0.5 mm of the right side
GUIDE_TOL = 0.0005                  # snug: the guide within 0.5 mm of the stack's left edge
UPRIGHT_MIN = math.radians(50.0)    # in the tray: the stack's lowest RISE_SPAN rising at least 50 degrees from level
RISE_SPAN = 3 * STRIP_W
TOUCH = 0.002
FINAL_S = 1.0


def strip_centre_a(i: int) -> float:
    return (i + 0.5) * STRIP_W


def start_strip_pos(i: int) -> np.ndarray:
    return np.array([START_A0_X + strip_centre_a(i), START_Y, START_Z])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def loaded_strip_poses(dy: float = 0.0, lift: float = 0.0, lean_deg: float = 20.0) -> list:
    """Strip poses (centre, rotation) for the stack standing in the slot, leaning back ``lean_deg`` on the open
    support, strip 0 at the top, its right edge ``dy`` from the right side and its bottom edge ``lift`` above the
    slot's floor (for tests). Strip frame: x along a (down the stack), y along b (+y), z its face."""
    lean = math.radians(lean_deg)
    down = np.array([-math.sin(lean), 0.0, -math.cos(lean)])         # from the top edge to the bottom edge
    yv = np.array([0.0, 1.0, 0.0])
    R = np.column_stack([down, yv, np.cross(down, yv)])
    bottom = np.array([SLOT_X[0] + 0.009, RIGHT_Y + dy + SHEET_L / 2, TOP + SLOT_FLOOR_H + COLL_T / 2 + lift])
    top = bottom - SHEET_W * down
    return [(top + strip_centre_a(i) * down, R) for i in range(N_STRIPS)]
