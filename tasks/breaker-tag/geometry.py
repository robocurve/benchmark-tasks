"""Dimensions and layout of the tagged-breaker task.

Station frame (the robot's): metres from the midpoint between the arm bases, x forward (away from the arms), y to the
robot's left, z up. TOP is the table top's world height; heights named *_H or in the panel tables are above TOP.

A 2 x 4 ft sheet of 3/4 in plywood stands on the table on its long edge as a wall, its face at x 0.430. A Square D
QO816L100DS load center (8 spaces, surface cover with door) is screwed to it, door open. Its eight single-pole
breakers sit in two columns of four; each handle pivots about a vertical axis and leans toward the middle (ON) or
away from it (OFF). Spaces 2, 3 and 4 are OFF. A Master Lock 497A DANGER / DO NOT OPERATE tag is tied to space 4's
handle with one of its own nylon ties and hangs over spaces 6 and 8, below it; space 2 is directly above it.

Handle joints: q = 0 straight out, +HANDLE_STOP = ON (tip toward the middle), -HANDLE_STOP = OFF.

Sizes marked "estimate" are not published by a maker; they are the simulation's values.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475                        # the table top (world z)
PHYSICS_HZ = 240

# -- the wall (fixed) ---------------------------------------------------------------------------------------------------
WALL_X = 0.430                      # its face
WALL_T = 0.019                      # 3/4 in plywood
WALL_W, WALL_H = 1.220, 0.610       # 4 ft wide, 2 ft tall, on its long edge
SILL = (0.038, 0.089)               # a 2x4 laid flat behind it (height, depth)
BRACKETS_Y = (-0.450, 0.0, 0.450)   # three steel shelf brackets behind it

# -- the load center (fixed) ------------------------------------------------------------------------------------------
PANEL_Y = 0.090                     # the load center's centre line (y)
BOX_X = (0.335, WALL_X)             # the steel box, 95 deep
BOX_W, BOX_Z = 0.226, (0.060, 0.379)    # 226 wide, 319 tall, bottom 60 above the table
FRONT_X = 0.330                     # the dead front's face (the cover, 5 thick)
COVER_W, COVER_Z = 0.232, (0.057, 0.382)
DOOR_W, DOOR_Z = 0.180, (0.100, 0.338)  # the door, hinged on the cover's right (-y) edge, open flat (180 degrees)
DOOR_HINGE = (0.315, PANEL_Y - COVER_W / 2)
DOOR_T = 0.0015

# -- the breakers -------------------------------------------------------------------------------------------------------
COL_Y = {"L": PANEL_Y + 0.038, "R": PANEL_Y - 0.038}   # left column 1, 3, 5, 7; right column 2, 4, 6, 8 (top to bottom)
ROWS = (0.2386, 0.2195, 0.2005, 0.1814)     # handle centres above the table, 3/4 in (19.05) pitch
PIVOT_X = 0.340                     # each handle's vertical pivot, 10 behind the dead front (estimate)
HANDLE_R = 0.022                    # pivot to tip: 12 proud of the dead front (estimate)
HANDLE_T, HANDLE_HT = 0.009, 0.010  # 9 thick (y), 10 tall (estimate)
HANDLE_IN = 0.008                   # the collider runs from the tip to 8 from the pivot (behind the dead front)
HANDLE_STOP = math.radians(25.0)    # leans 25 degrees either way
HANDLE_MASS = 0.005
START_ON = {1: True, 2: False, 3: False, 4: False, 5: True, 6: True, 7: True, 8: True}
TAGGED = 4
TARGETS = (2, 3)                    # space 2 directly above the tagged handle, space 3 in the left column
OTHERS = (1, 5, 6, 7, 8)

# over-centre spring (estimate): 10 N at the tip to push ON, 6 N to push OFF, switching at midway. A drive pulls the
# handle toward a target past the stop it rests on; the target switches to the other side when the handle passes
# midway. Torque at midway = K * (stop + preload).
F_ON, F_OFF = 10.0, 6.0             # N at the tip (estimate)
K_HANDLE = F_ON * HANDLE_R / (HANDLE_STOP + math.radians(30.0))       # N m/rad
PRE_OFF = math.radians(30.0)
PRE_ON = F_OFF * HANDLE_R / K_HANDLE - HANDLE_STOP                     # 8 degrees
D_HANDLE = 0.0003


def space(n: int):
    """(column, row index, y, z above the table) of breaker space n (1..8)."""
    col = "L" if n % 2 else "R"
    row = (n - 1) // 2
    return col, row, COL_Y[col], ROWS[row]


def on_sign(n: int) -> float:
    """The handle joint's axis is +z (left column) or -z (right column), so that + is always toward the panel's
    middle."""
    return 1.0 if n % 2 else -1.0


def handle_tip(n: int, q: float) -> np.ndarray:
    """The tip's centre (table frame, z above the table) at joint angle q."""
    _, _, y, z = space(n)
    a = on_sign(n) * q                                 # about +z
    return np.array([PIVOT_X - HANDLE_R * math.cos(a), y - HANDLE_R * math.sin(a), z])


# -- the tag and its tie ------------------------------------------------------------------------------------------------
TAG_W, TAG_L, TAG_T = 0.079, 0.146, 0.0015    # Master Lock 497A: 3-1/8 x 5-3/4 in, laminated polyester
TAG_HOLE = 0.013                    # the grommet's centre below the top edge (estimate)
GROMMET_OD = 0.019                  # (estimate)
TAG_MASS = 0.008                    # (estimate)
TIE_W = 0.0036                      # the nylon tie's width
LOOP_S0 = 0.0025                    # the loop's centre this far out from the dead front along the handle, at the start
LOOP_OFF = 0.011                    # ... past the handle's tip: the tag is off (estimate: 11 along the leaning handle)
LOOP_MAX = 0.016                    # the slide's end (beyond the tip)
LOOP_BASE = 0.011                   # the dead front, along the handle's axis from the pivot (10 / cos 25)
LOOP_F = 5.0                        # N outward slides the loop (estimate)
LOOP_D = 2.0e4                      # N s/m: a damping drive capped at LOOP_F (sticks below it)
TIE_L = 0.014                       # the stem's axis to the grommet's centre (the loop's slack)
TIE_TILT0 = math.radians(50.0)      # the tie's start: swung out over space 4's tip
SWING_D = 2.0e-4                    # N m s/rad on the tag's hinges (estimate)
TIE_D = 5.0e-5
DUMMY_MASS = 2.0e-4                 # the tie's links (0.2 g each)

# -- frictions (estimate) ---------------------------------------------------------------------------------------------
MU_HANDLE = (0.4, 0.35)
MU_TAG = (0.4, 0.35)
MU_STEEL = (0.3, 0.25)

# -- scoring ----------------------------------------------------------------------------------------------------------
RELEASE_GAP = 0.020                 # released: no gripper part within this of any handle, the tag or the panel
FINAL_S = 1.0
STILL_DEG = 0.5                     # (reporting) handles still: joint speed below this per control step

# -- the trial camera (fixed): a phone on a gooseneck clamped to the wall's top edge, to the right of the panel ------
TRIAL_CAM = (0.300, -0.100, 0.450)  # the phone's camera (its back), out of the top camera's view of the handles and tag
TRIAL_AIM = (PIVOT_X - HANDLE_R, COL_Y["R"], 0.222)    # aimed down at spaces 2 and 4 and the tie
PHONE = (0.150, 0.075, 0.009)       # the phone's size (long side across the view), estimate
CLAMP = (0.4395, -0.100, 0.625)     # a 40 x 40 x 30 clamp on the wall's top edge
NECK_ELBOW = (0.400, -0.100, 0.560) # the gooseneck's bend; 12 mm thick


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def box_verts(lo, hi) -> np.ndarray:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def tag_corners() -> np.ndarray:
    """The tag's 8 corners in its own frame: origin at the grommet's centre, x out of its back face (toward the panel),
    z up along its length."""
    return box_verts((-TAG_T / 2, -TAG_W / 2, TAG_HOLE - TAG_L), (TAG_T / 2, TAG_W / 2, TAG_HOLE))


def handle_corners(n: int, q: float) -> np.ndarray:
    """The handle collider's corners (table frame, z above the table) at joint angle q."""
    _, _, y, z = space(n)
    loc = box_verts((-HANDLE_R, -HANDLE_T / 2, -HANDLE_HT / 2), (-HANDLE_IN, HANDLE_T / 2, HANDLE_HT / 2))
    return np.array([PIVOT_X, y, z]) + loc @ rot_z(on_sign(n) * q).T


# -- the demo -----------------------------------------------------------------------------------------------------------
PUSH_IN = 0.005                     # the fingertips this far behind the handle's tip line (toward the dead front)
