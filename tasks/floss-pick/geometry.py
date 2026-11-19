"""Dimensions and layout of the floss-pick task.

Station frame: metres from the midpoint between the arm bases, x forward (away from the arms), y to the robot's left,
z up. Head frame ("H"): millimetres from the contact point between the two lower central incisors (31 and 41), station
axes (x toward the back of the mouth, y to the robot's left, z up). The head faces the robot, so the patient's left
(31) is at -y and the patient's right (41) at +y.

The typodont's teeth are modelled at standard (Wheeler) adult crown sizes: lower central incisor crown 5.4 mm
mesio-distal at the contact (3.5 mm at the neck), 9 mm long, 2.7 mm labio-lingual at the contact. The contact sits
1.5 mm below the incisal edges and the interdental papilla 2.5 mm below the contact. Each central incisor is moved
0.3 mm away from the contact, a 0.6 mm gap, so the 0.5 mm floss fits once the task's contact rule lets it in.

The pick frame ("P"): millimetres; origin at the floss's midpoint, x along the floss toward the back arm (the handle's
tip end), z from the floss toward the handle, y = z x x. In the work pose (arms pointing down, handle out of the
mouth toward the robot) P is aligned with the station axes.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475                        # the station's table top (m)

# -- the target ---------------------------------------------------------------------------------------------------------
CONTACT = np.array([0.300, 0.0, TOP + 0.140])     # the 31/41 contact (station, m)


def to_world(p_mm) -> np.ndarray:
    """Head-frame mm -> station m."""
    return CONTACT + np.asarray(p_mm, float) / 1000.0


def to_head(p_m) -> np.ndarray:
    """Station m -> head-frame mm."""
    return (np.asarray(p_m, float) - CONTACT) * 1000.0


# -- the lower central incisors (H frame, mm; 41 at +y, 31 its mirror) ---------------------------------------------------
GAP_HALF = 0.3                      # each incisor moved 0.3 mm away from the contact: a 0.6 mm gap
B0 = 2.7                            # half the crown's mesio-distal width at the contact (5.4 mm)
Y_C = GAP_HALF + B0                 # 41's crown axis
Z_EDGE = 1.5                        # the incisal edges, 1.5 mm above the contact
Z_RND, R_RND = 0.65, 0.8            # the mesio-incisal corner: rounded (radius 0.8) above z 0.65 (the incisal notch)
TAPER, TAPER_U0 = 0.14, 0.6         # below the contact the mesial side recedes 0.14 mm per mm (onset over 0.6 mm)
A_EDGE, A_CERV, A_SPAN, A_POW = 0.55, 3.0, 8.5, 1.8    # half the labio-lingual thickness: 0.55 at the edge -> 3.0
X_SHIFT = 0.35                      # the crown's axis moves lingually as it thickens (the labial face nearly upright)
N_SE = 3.5                          # superellipse exponent of the crown's cross-section (flat proximal sides)
Z_NECK = -7.5                       # the labial cemento-enamel junction (crown 9 mm long)
Z_ROOT = -16.0                      # the modelled root's end (inside the gum)
ZONE = (-0.5, 0.5)                  # the contact zone: the floss's centre line passes it between these heights
XS = 0.5                            # ... over the contact area, |x| <= XS
YS = 0.35                           # ... between the teeth, |y| <= YS
PAP_Z, PAP_K = -2.5, 0.8            # the papilla's crest: z = PAP_Z - PAP_K x^2 (2.5 mm below the contact)


def a_of(z):
    """Half the crown's labio-lingual thickness (a wedge thickening from the edge, finite slope everywhere)."""
    t = np.clip((Z_EDGE - np.asarray(z, float)) / A_SPAN, 0.0, 1.0)
    return A_EDGE + (A_CERV - A_EDGE) * (1.0 - (1.0 - t) ** A_POW)


A0 = float(a_of(0.0))


def xc_of(z):
    return X_SHIFT * (a_of(z) - A0)


def b_of(z):
    z = np.asarray(z, float)
    b = np.full(z.shape, B0)
    top = z > Z_RND
    dz = np.minimum(z[top] - Z_RND, 0.95 * R_RND)         # the corner's arc stops short of horizontal (finite slope)
    b[top] = B0 - (R_RND - np.sqrt(R_RND ** 2 - dz ** 2))
    low = z < ZONE[0]
    u = ZONE[0] - z[low]
    f = np.where(u < TAPER_U0, u * u / (2 * TAPER_U0), u - TAPER_U0 / 2)
    b[low] = np.maximum(B0 - TAPER * f, 1.0)
    return b


def phi41(p) -> np.ndarray:
    """The crown's implicit function at points p (N, 3), H mm: 0 on 41's side surface, < 0 inside (a normalised
    superellipse radius minus 1); the incisal edge is a separate plane (see tooth_distance)."""
    p = np.asarray(p, float)
    z = p[:, 2]
    a, b = a_of(z), b_of(z)
    X = np.abs(p[:, 0] - xc_of(z)) / a
    Y = np.abs(p[:, 1] - Y_C) / b
    return (X ** N_SE + Y ** N_SE) ** (1.0 / N_SE) - 1.0


def mesial_y41(x, z) -> np.ndarray:
    """41's mesial (facing) surface: y as a function of x and z (|x - xc| < a)."""
    x, z = np.broadcast_arrays(np.asarray(x, float), np.asarray(z, float))
    a, b = a_of(z), b_of(z)
    u = np.clip(np.abs(x - xc_of(z)) / a, 0.0, 1.0)
    return Y_C - b * (1.0 - u ** N_SE) ** (1.0 / N_SE)


def pap_z(x):
    return PAP_Z - PAP_K * np.asarray(x, float) ** 2


# -- the wipe band: 2 mm wide, centred under the contact, from the contact's lower edge to 1 mm above the gum ----------
BAND_W = 2.0
BAND_GUM_CLEAR = 1.0
BAND_COLS, BAND_ROWS = 10, 4        # the band's grid of cells: 10 columns of 0.2 mm, 4 rows of a quarter of its local
                                    # height (between the groove at the contact's lower edge and the groove 1 mm above
                                    # the gum line); a side is wiped when 80% of its 40 cells are (32)


def band41():
    """The band's cells on 41's facing side, 4 sample points a cell (2 x 2 at the cell's quarter points): points
    (M, 3), H mm, their outward normals, and each point's cell index. A cell counts as wiped when at least 3 of its 4
    points are, as on the photo a cell counts as bare when bare resin covers at least three quarters of it."""
    pts, cell = [], []
    cw = BAND_W / BAND_COLS
    for i in range(BAND_COLS):
        x0 = -BAND_W / 2 + i * cw
        for xs in (x0 + 0.25 * cw, x0 + 0.75 * cw):
            z_top, z_lo = ZONE[0], float(pap_z(xs)) + BAND_GUM_CLEAR
            rh = (z_top - z_lo) / BAND_ROWS
            for j in range(BAND_ROWS):
                for zs in (z_top - (j + 0.25) * rh, z_top - (j + 0.75) * rh):
                    pts.append((xs, float(mesial_y41(xs, zs)), zs))
                    cell.append(i * BAND_ROWS + j)
    pts = np.array(pts)
    h = 1e-4
    grad = np.stack([(phi41(pts + h * e) - phi41(pts - h * e)) / (2 * h) for e in np.eye(3)], axis=1)
    return pts, grad / np.linalg.norm(grad, axis=1, keepdims=True), np.array(cell)


import os as _os0

# -- the floss pick (P frame, mm) ---------------------------------------------------------------------------------------
SPAN = 15.0                         # floss between the arms' tips
ARM_L = 10.0                        # the arms, from the handle's lower edge to the floss
ARM_T = 2.0                         # arm cross-section (x and y)
ARM_BELOW = 0.8                     # the arm's tip below the floss
HANDLE_W, HANDLE_T = 10.0, 6.0      # the handle: 10 mm deep in the pick's plane, 6 mm thick (a thick ribbed grip)
PICK_L = 75.0
X_TIP = SPAN / 2 + ARM_T            # the handle's tip end (the back arm's outer face)
X_REAR = X_TIP - PICK_L             # the handle's rear end
Z_HANDLE = (ARM_L, ARM_L + HANDLE_W)
GRIP_L = 26.0                       # the pads' length along the handle
GRIP_X = X_REAR + GRIP_L / 2        # the pads over the handle's rear 26 mm
GRIP_C = np.array([GRIP_X, 0.0, ARM_L + HANDLE_W / 2])   # the tool point when held
PICK_MASS = float(_os0.environ.get("FP_PICK_G", "5.0")) / 1000.0   # kg
ANCHOR_A = np.array([-SPAN / 2, 0.0, 0.0])   # the front (labial) arm's floss anchor
ANCHOR_B = np.array([SPAN / 2, 0.0, 0.0])    # the back (lingual) arm's
PICK_OVERHANG = 0.030                        # m: the handle's rear end over the board's front edge
PICK_START = np.array([0.235 - PICK_OVERHANG + PICK_L / 2000, -0.115])   # the handle's middle (station x, y), on the board

# -- the floss (task-code string) --------------------------------------------------------------------------------------
FLOSS_R = 0.25                      # 0.5 mm floss (sim)
T0 = float(_os0.environ.get("FP_T0", "5.0"))   # N, pre-tension
K_AX = 4.7                          # N/mm: tension rise per mm of stretch (floss and arms)
N_NODES = 31
import os as _os
F_STATIC_DN = float(_os.environ.get("FP_F_STATIC", "8.0"))   # N: the contact's hold, straight down
F_W_DN, F_F_DN = 0.0, F_STATIC_DN                             # the friction-direction law (3/8 of the hold while
                                                              # sawing +-2 mm at 2 Hz)
F_STATIC_UP = float(_os.environ.get("FP_F_UP", "3.0"))       # ... and from below: a one-way hold (no creep while
F_W_UP, F_F_UP = 0.0, F_STATIC_UP                             # sliding; it lets go only at this pull)
VS_DEADBAND = 1.0                   # mm/s: slower sliding along the floss counts as none
VS_WINDOW = 0.1                     # s: sliding speed = the floss's travel along itself over this window

# -- scoring constants --------------------------------------------------------------------------------------------------
SNAP_MM, SNAP_S = 1.0, 0.025        # diagnostic only: the floss's descent over 25 ms before each touch of the gum
STRIP_W, STRIP_L = 0.4, 6.0         # mm: the pressure film strip lying flat along the papilla under the contact (y, x)
F_MARK = 2.0                        # N: the floss pressing the strip this hard marks it: a snap (estimate)
PRESS_N, PRESS_S = 3.0, 0.5         # 3 N or more on the head for 0.5 s, at any time in the trial
F_WIPE = 0.5                        # N: the floss pressing a tooth this hard wipes it (estimate)
WIPE_TOL = 0.1                      # mm: a band point within this of the floss's surface ...
WIPE_SPEED = 1.0                    # mm/s: ... while the floss slides over it at least this fast
WIPED = 0.80                        # a side is wiped at 80% of its band points
FLOSS_FAIL = 22.0                   # N: the floss's load (pull-out or break)
HEAD_MOVED = 0.005                  # m (the head is fixed in the sim)
HOLD_STEPS = 20                     # the final 1 s at 20 Hz

# -- the lower lip (H frame, mm) ----------------------------------------------------------------------------------------
LIP_TOP = Z_EDGE - 3.0              # the lip covers the labial faces up to 3 mm below the incisal edges
LIP_BOTTOM = Z_NECK                 # ... from the gum line
LIP_T = 10.0                        # thick
LIP_LEN = 40.0                      # along the arch
LIP_GAP = 0.15                      # its back face this far in front of the labial faces at rest
LIP_K = 300.0                       # N/m (0.3 N/mm)
LIP_PRELOAD = 0.3                   # N against the teeth at rest
LIP_TRAVEL = 0.010                  # m forward
LIP_MASS = 0.030                    # kg
LIP_DAMP = 2.0                      # N s/m
LIP_MU = 0.6

# -- the head, board, scale, display and phone (station, m) --------------------------------------------------------------
SCALE_X, SCALE_Y = (0.235, 0.535), (-0.150, 0.150)
SCALE_H = 0.040                     # platform top above the table
BOARD_T = 0.018
BOARD_TOP = SCALE_H + BOARD_T       # above the table
DISPLAY = ((0.245, 0.290), (0.090, 0.140), (BOARD_TOP, BOARD_TOP + 0.030))   # the tethered display (x, y, z ranges)
PHONE_C = (0.300, 0.350, 0.140)     # the phone's lens (x, y, height above the table)
UPPER_EDGE_Z = Z_EDGE + 48.0        # the upper incisal edges (H mm)
UPPER_X = -6.0                      # the upper incisors' labial offset from the lower (H mm, the jaw open)
MOUTH_W = 52.0                      # the mask's mouth opening

# -- the arches (H frame, mm): tooth centres along a curve, s = arc length from the midline ----------------------------
# Lower teeth from the midline: (name, mesio-distal width, half labio-lingual at the neck, crown kind)
LOWER_TEETH = (("1", 5.4, 3.0, "incisor"), ("2", 5.9, 3.1, "incisor"), ("3", 6.9, 3.8, "canine"),
               ("4", 7.0, 3.8, "premolar"), ("5", 7.1, 4.0, "premolar"), ("6", 11.2, 5.2, "molar"),
               ("7", 10.5, 5.0, "molar"))
UPPER_TEETH = (("1", 8.6, 3.5, "incisor"), ("2", 6.6, 3.0, "incisor"), ("3", 7.6, 4.0, "canine"),
               ("4", 7.1, 4.5, "premolar"), ("5", 6.6, 4.5, "premolar"), ("6", 10.4, 5.5, "molar"),
               ("7", 9.8, 5.3, "molar"))
ARCH_THETA_MAX = math.radians(70.0)     # the arch's heading turns from along y (front) to nearly along x (molars)
ARCH_S0, ARCH_W = 4.0, 9.0


def arch_theta(s):
    s = np.abs(np.asarray(s, float))
    u = np.maximum(s - ARCH_S0, 0.0) / ARCH_W
    return ARCH_THETA_MAX * (1.0 - np.exp(-u * u))


_S = np.linspace(0.0, 80.0, 8001)
_TH = arch_theta(_S)
_AX = np.concatenate([[0.0], np.cumsum(np.sin(_TH[1:]) * np.diff(_S))])
_AY = np.concatenate([[0.0], np.cumsum(np.cos(_TH[1:]) * np.diff(_S))])


def arch(s, scale: float = 1.0):
    """The arch's point and lingual normal at arc length s (signed: + toward +y), and the heading. ``scale`` widens
    the arch (the upper one)."""
    s = np.asarray(s, float)
    a = np.abs(s) / scale
    x = np.interp(a, _S, _AX) * scale
    y = np.interp(a, _S, _AY) * scale * np.sign(s)
    th = arch_theta(a)
    sg = np.where(s < 0, -1.0, 1.0)
    normal = np.stack([np.cos(th), -np.sin(th) * sg, np.zeros_like(th)], -1)
    return np.stack([x, y, np.zeros_like(x)], -1), normal, th


def tooth_layout(teeth=LOWER_TEETH, start: float = GAP_HALF, scale: float = 1.0):
    """[(name, side, s_centre, width, neck, kind)] for both sides, side +1 at +y."""
    out = []
    for side in (1, -1):
        s = start
        for name, w, neck, kind in teeth:
            out.append((name, side, side * (s + w / 2), w, neck, kind))
            s += w
    return out


def contacts(teeth=LOWER_TEETH, start: float = GAP_HALF):
    """Arc lengths of the interproximal contacts (the papillae), both sides and the midline."""
    out = [0.0]
    s = start
    for _, w, _, _ in teeth[:-1]:
        s += w
        out += [s, -s]
    return np.array(out)
