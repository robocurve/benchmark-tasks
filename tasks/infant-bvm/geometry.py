"""Sizes, layout and rule constants of the infant-bvm task.

Station frame: metres from the midpoint between the arm bases, x forward (away from the arms), y to the robot's left,
z up. Heights in comments are above the table top unless said otherwise.

The manikin is a Laerdal Little Baby QCPR (published 650 x 260 x 140 mm, 1.8 kg, "a three-month-old infant"). Its
sim body is built by build_manikin.py from the CC0 MakeHuman base mesh with MakeHuman's own baby targets (1/3 each
ethnicity, both sexes), its arms lowered 25 degrees and scaled 1.08 so it is 650 long and 140 high. Face positions,
the neck hinge and every stiffness are estimates (est.).

The bag-valve mask is an Ambu SPUR II Infant (bag 190 x 71 mm, 70 g; 40 cmH2O valve) with an Ambu infant anatomical
face mask (55 x 47 x 25 mm, 14 g), on the patient valve's swivel. Mask frame ("M"): origin at the middle of the
cushion's contact line, z out of the face (up the stem), x toward the mask's wide (chin) end.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475                        # the station's table top

# -- mat and manikin ----------------------------------------------------------------------------------------------------
MAT_C = (0.500, 0.0)                # rubber non-slip mat, 700 x 500 x 3 mm, taped down
MAT_SIZE = (0.700, 0.500, 0.003)
MAT_TOP = TOP + MAT_SIZE[2]
CROWN_X = 0.180                     # the manikin lies face up on y 0, head toward the robot, crown at x 180
MH_SCALE = 1.08                     # MakeHuman baby (602 mm) -> 650 mm long, 140 mm high
ARM_DROP = math.radians(25.0)       # the MakeHuman A-pose arms lowered toward the sides (260 mm wide) ...
ARM_BACK = math.radians(12.0)       # ... and back, to lie along them
TORSO_MASS = 1.55                   # kg (1.8 kg published, less the head)
HEAD_MASS = 0.25                    # kg (est.)
MAT_MU = 0.8                        # manikin vinyl on the rubber mat (est.)
SKIN_MU = 0.9                       # cushion / finger pad on the face (est.)

# the neck: a hinge about y through the centre of the occiput's curve; the occiput (a sphere about the hinge) rests on
# the mat, so a press on the face goes into the mat and only a turning moment tilts the head
OCCIPUT_PRELOAD = 0.0005            # the occiput's sphere reaches 0.5 mm below the mat at rest, so it takes the head's weight
NECK_BREAK = 0.5                    # N m: the neck's friction (est.), held in task code as stick-slip
NECK_K = 20.0                       # N m/rad: the stiff "stick" spring of that friction (task code drags its rest angle)
NECK_D = 0.5                        # N m s/rad
NECK_LIMITS = (-20.0, 30.0)         # deg from neutral: flexed (-) .. extended (+)
AIRWAY = (-5.0, 15.0)               # deg: the airway is open from 5 deg flexed to 15 deg back (est.)

# -- the chest and its flag --------------------------------------------------------------------------------------------
RISE_PER_ML = 0.25e-3               # m of chest (flag dot) rise per mL in the lungs (est.)
LUNG_CAP_ML = 60.0                  # the manikin's lung bag holds at most this (rise 15 mm)
LUNG_TAU = 0.5                      # s: exhaling once the bag is released or the seal is broken
FLAG_H, FLAG_W, FLAG_T = 0.030, 0.020, 0.0008   # stiff white card, upright, face toward +y
DOT_D, DOT_UP = 0.010, 0.020        # 10 mm black dot, centred 20 mm above the skin

# -- the bag-valve mask (mask frame M, metres) -------------------------------------------------------------------------
MASK_L, MASK_W, MASK_H = 0.055, 0.047, 0.025
NOSE_R, CHIN_R = 0.012, 0.0235      # the outline is the hull of two circles: narrow (nose) end, wide (chin) end
NOSE_CX = -MASK_L / 2 + NOSE_R
CHIN_CX = MASK_L / 2 - CHIN_R
CUSHION_IN = 0.005                  # the cushion's contact line runs this far inside the outline
N_SEG = 12                          # the air cushion as 12 sprung rim segments
SEG_R = 0.0035                      # each a capsule along the rim ...
SEG_LEN = 0.0045                    # ... its cylinder this long (the 12 stay clear of each other)
SEG_TRAVEL = 0.004                  # m of compression travel
SEG_K = 400.0                       # N/m (0.4 N/mm, est.)
SEG_PRELOAD = 0.1                   # N
SEG_D = 0.3                         # N s/m
SEG_NOM = 0.0020                    # the rim is shaped to the face: centred and pressed, each segment is in 2 mm
DOME_TOP = 0.0205                   # the dome's top (thumb rest) above the contact line's plane
DOME_CAP_FROM = 0.0165              # the dome's collider: its cap above this height (the nose tip stays 4 mm below)
STEM_R, STEM_H = 0.011, 0.020       # 22 mm stem on the dome
SWIVEL_R, SWIVEL_H = 0.011, 0.018   # the swivel connector (patient port) between the stem and the valve housing
VALVE_R, VALVE_L = 0.014, 0.050     # the patient valve housing (28 x 50), along the bag's axis on the stem
VALVE_Z = DOME_TOP + STEM_H + SWIVEL_H + VALVE_R    # its axis above M's origin
PORT_R, PORT_L = 0.015, 0.008       # the expiratory port ring at its far (+x) end
BAG_L, BAG_R = 0.190, 0.0355        # Ambu SPUR II Infant, 190 x 71 mm
BAG_U0 = VALVE_L / 2                # the bag runs from the valve's end (u = distance from the mask axis along -x)
BAG_MID = BAG_U0 + BAG_L / 2        # its middle, 120 mm from the mask's axis
BAG_NECK_R = 0.012
WALL_L = 0.100                      # 4 squeezable walls, each a 90-degree shell, 100 mm long, at the middle
BAG_FLAT = 0.003                    # the bag's underside rests flattened 3 mm (a soft bag on a table)
WALL_FLAT = 0.024                   # each wall's crown is flat this wide, as a pad flattens a soft bag ...
WALL_DEPTH = 0.015                  # ... its collider a block this deep under the crown
WALL_T = 0.002
WALL_TRAVEL = 0.030
WALL_K = 800.0                      # N/m (0.8 N/mm, est.)
WALL_PRELOAD = 0.5                  # N
WALL_D = 2.0                        # N s/m
SQUEEZE_LEN = 0.060                 # bag volume: pi/4 (d_lr^2 + d_tb^2) x 60 mm (squeeze depths d)
BAG_VOL_CAP_ML = 150.0              # the bag's stroke volume
MASS_DOME, MASS_SEG = 0.002, 0.001    # mask 14 g with its cushion
MASS_VALVE = 0.026
MASS_BAG_CORE, MASS_WALL = 0.030, 0.010     # bag 70 g
SWIVEL_D = 0.002                    # N m s/rad: the swivel's light drag ...
SWIVEL_BREAK = 0.02                 # N m: ... and its friction (est.), stick-slip in task code
SWIVEL_K = 1.0                      # N m/rad: the stick spring
BVM_MU = 0.9                        # PVC / bag on the finger pads (est.)

# start: mask rim down on the mat at the manikin's left, narrow (nose) end toward the robot, mask centre at x 300, y 190;
# the bag points toward the robot and its far end rests on the table
BVM_START_XY = (0.300, 0.190)

# -- rules -------------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.006                    # mask centre within 6 mm of the face's mask point
SEG_IN = 0.0010                     # a segment is pressed: in 1.0 mm or more ...
SEG_TOUCH = 0.0005                  # ... and its tip within 0.5 mm of the face
EYE_R = 0.009                       # eye zones
EYE_LATCH_S = 1.0
BREATH_START = 0.005                # an episode starts at a 5 mm rise ...
ONSET = 0.0005                      # ... its onset is the last time the rise left 0.5 mm ...
BREATH_END = 0.002                  # ... and it ends when the rise is back within 2 mm
BREATH_MAX = 0.012                  # peaks over 12 mm are too big
PEAK_WINDOW = (0.5, 2.0)            # s from the onset to the peak
SEATED_025 = 2.0                    # s: seated this long (continuously) without a breath scores 0.25
HELD_OFF_GAP = 0.030                # held off: the mask 30 mm or more from the face ...
HELD_STILL = 0.002                  # ... the BVM moving under 2 mm per step, held in a gripper, for 1 s
HOLD_S = 1.0
MOVED = 0.020                       # the manikin moving 20 mm from its outline ...
BACK_LIFT = 0.005                   # ... or its back lifting 5 mm off the mat: 0 for the run
DROP_FALL = 0.030                   # the BVM falling 30 mm ungripped onto the manikin: 0
TOUCH_F = 0.02                      # N: a contact carrying this much force counts as a touch


def mask_outline(n: int = 96, inset: float = 0.0) -> np.ndarray:
    """Points round the mask's outline (or a line ``inset`` inside it) in M's x-y plane, counter-clockwise from the
    chin end: the hull of the nose and chin circles."""
    r1, r2 = NOSE_R - inset, CHIN_R - inset
    c1, c2 = np.array([NOSE_CX, 0.0]), np.array([CHIN_CX, 0.0])
    d = c2[0] - c1[0]
    phi = math.asin((r2 - r1) / d)            # tangent lines' tilt
    # the chin arc from -(90+phi) to +(90+phi) deg, the nose arc on the other side
    a2 = np.linspace(-(math.pi / 2 + phi), math.pi / 2 + phi, n // 2)
    a1 = np.linspace(math.pi / 2 + phi, 3 * math.pi / 2 - phi, n // 2)
    P = np.vstack([c2 + r2 * np.c_[np.cos(a2), np.sin(a2)], c1 + r1 * np.c_[np.cos(a1), np.sin(a1)]])
    return P


def contact_points(k: int = N_SEG) -> tuple[np.ndarray, np.ndarray]:
    """k points equally spaced along the cushion's contact line (M x-y), and the unit tangents there."""
    P = mask_outline(720, CUSHION_IN)
    seg = np.linalg.norm(np.roll(P, -1, 0) - P, axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    total = s[-1]
    out, tan = [], []
    for i in range(k):
        t = total * i / k
        j = int(np.searchsorted(s, t, side="right") - 1)
        f = (t - s[j]) / max(seg[j], 1e-12)
        a, b = P[j], P[(j + 1) % len(P)]
        out.append(a + f * (b - a))
        tan.append((b - a) / np.linalg.norm(b - a))
    return np.array(out), np.array(tan)


def bag_volume_ml(d_lr: float, d_tb: float) -> float:
    """The bag's squeezed-out volume (mL) from its squeeze depths across (left + right walls) and up-down (m)."""
    v = math.pi / 4 * (d_lr ** 2 + d_tb ** 2) * SQUEEZE_LEN * 1e6
    return min(v, BAG_VOL_CAP_ML)


def bvm_start(face_h, n_beta: int = 401):
    """The bag-valve mask's start pose as it settles: mask rim down on the mat at BVM_START_XY, narrow end toward the
    robot, the bag toward the robot resting on the mat/table. Returns (M origin position, rotation matrix)."""
    cp, _ = contact_points()
    tips = np.c_[cp, np.asarray(face_h) - SEG_NOM]
    us = np.linspace(BAG_U0 + 0.035, BAG_U0 + BAG_L, 40)
    w0, w1 = BAG_MID - WALL_L / 2, BAG_MID + WALL_L / 2
    rad = np.interp(us, [BAG_U0 + 0.012, w0, w1, BAG_U0 + BAG_L], [BAG_NECK_R, BAG_R, BAG_R, 0.020])
    bag = np.c_[-us, np.zeros_like(us), VALVE_Z - np.minimum(rad, BAG_R - BAG_FLAT)]
    x0, y0 = BVM_START_XY

    def support(x):
        lo, hi = MAT_C[0] - MAT_SIZE[0] / 2, MAT_C[0] + MAT_SIZE[0] / 2
        return np.where((x >= lo) & (x <= hi), MAT_TOP, TOP)

    best = None
    for beta in np.linspace(-0.4, 0.2, n_beta):
        c, s = math.cos(beta), math.sin(beta)
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        pt, pb = tips @ R.T, bag @ R.T
        h = max((support(x0 + pt[:, 0]) - pt[:, 2]).max(), (support(x0 + pb[:, 0]) - pb[:, 2]).max())
        gap_t = (h + pt[:, 2] - support(x0 + pt[:, 0])).min()
        gap_b = (h + pb[:, 2] - support(x0 + pb[:, 0])).min()
        key = max(gap_t, gap_b)
        if best is None or key < best[0]:
            best = (key, np.array([x0, y0, h]), R)
    return best[1], best[2]
