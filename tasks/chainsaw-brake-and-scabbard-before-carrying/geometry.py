"""Geometry of the chainsaw scene on the yam_arena station table, shared by build_assets.py, task.py, logic.py and
demo.py. Pure Python (no Isaac Sim), so the task file and the unit tests can import it.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights are above the table top
(TOP) unless they say "world". Arm bases: left (0, +0.305), right (0, -0.305).

A generic 10-inch (25 cm) cordless chainsaw lies on its SIDE on the table, bar pointing to the robot's right (-y),
the engine housing at the left. Lying on its side puts the guide bar in a horizontal plane and the front hand guard's
hinge on a vertical axis. Pushing the hand guard forward (toward the bar tip) past a detent engages the chain brake. A
loose plastic bar cover (scabbard), 26.5 cm long, rests on a holder beyond the bar tip, its open end toward the bar;
it slides onto the bar.

The saw's origin (the root of its USD articulation) is the point on the table under the bar's root axis: the bar runs
from y = 0 (root) to y = -BAR_LEN (tip) along x = SAW_X.
"""

from __future__ import annotations

import math
import os

TOP = 0.7475                                    # yam_arena.task_file.TABLE_TOP["station"]

# -- layout (world) -------------------------------------------------------------------------------------------------
# TASK_VARIANT=A (default): the bar points to the robot's right (-y), the hand guard is worked by the left arm and the
# cover by the right arm. TASK_VARIANT=B is the mirror image across the table's centre line: the bar points to the
# robot's left (+y), the right arm works the guard and the left arm the cover. S is the bar's direction: world y of a
# point at saw-frame y is SAW_Y + S * y.
VARIANT = os.environ.get("TASK_VARIANT", "A").strip().upper() or "A"
if VARIANT not in ("A", "B"):
    raise ValueError("TASK_VARIANT must be A or B")
S = 1.0 if VARIANT == "A" else -1.0
GUARD_ARM, COVER_ARM = ("left", "right") if VARIANT == "A" else ("right", "left")
SAW_X, SAW_Y = 0.200, 0.020 * S                 # the bar's root axis, on the table
SAW_YAW = 0.0

# -- the saw (all in the saw's frame: origin under the bar root axis, x forward (the saw's "up" while in use), y along
#    the bar toward the housing, z up from the table; the saw lies on its flat motor side) ---------------------------------
SAW_MASS = 2.8                                  # kg, with its battery (cordless 10 in saws weigh 2.3 to 3.0 kg)
SAW_FRICTION = (0.9, 0.8)                       # rubber feet on the table

HOUSING_Y = (0.0, 0.200)                        # the motor housing, along the bar axis
HOUSING_X = (-0.040, 0.100)                     # and across it (x is the saw's up)
HOUSING_Z = (0.0, 0.085)                        # its thickness: the saw lies on this flat side
BATTERY_Y = (0.200, 0.262)                      # the battery pack behind it
BATTERY_X = (-0.030, 0.085)
BATTERY_Z = (0.005, 0.080)
COVER_Y = (0.000, 0.080)                        # the sprocket cover: a plate on the housing's top side (the bar side)
COVER_X = (-0.040, 0.055)
COVER_Z = (0.085, 0.100)

# the guide bar (a 25 cm bar): a flat plate on the sprocket cover, in the horizontal plane, with the chain round its rim
BAR_LEN = 0.250                                 # visible bar, from the sprocket cover's edge (y = 0) to the tip
BAR_INSIDE = 0.050                              # more bar under the cover (y from 0 to +0.05), hidden
BAR_WIDTH = 0.026                               # across the bar (x), 26 mm
BAR_PLATE_T = 0.0060                            # the steel plate
CHAIN_H = 0.0075                                # the chain's thickness (z) with the bar plate: bar + chain outline
CHAIN_OVER = 0.003                              # the chain's teeth stand this far past the plate edge (each side, x)
BAR_Z = 0.1000 + CHAIN_H / 2                    # the bar's mid-plane height above the table: 103.75 mm
BAR_OUT_HALF_W = BAR_WIDTH / 2 + CHAIN_OVER     # half the bar's outline width: 16 mm
BAR_TIP_R = BAR_OUT_HALF_W                      # the tip is round

# the front handle (a horizontal bar over the housing's top edge) and rear handle
FRONT_HANDLE = dict(x=0.128, y0=0.115, y1=0.190, r=0.0145, z=0.0425)
REAR_HANDLE = dict(x=0.100, y0=0.215, y1=0.262, r=0.0160, z=0.0425)

# -- the hand guard (chain brake): a plate that swings in the horizontal plane about a vertical hinge ---------------------
GUARD_PIVOT = (0.108, 0.052)                    # (x, y) in the saw's frame; the hinge sits 8 mm off the housing's top edge
GUARD_LEN = 0.070                               # the plate, from the hinge outward along +x at rest
GUARD_T = 0.012                                 # its thickness (y)
GUARD_H = 0.085                                 # its height (z), the housing's full thickness
GUARD_BAR_LEN = 0.014                           # the free end's grip bar (x length)
GUARD_BAR_T = 0.026                             # the grip bar's thickness (y): the gripper's jaws hold it
GUARD_MASS = 0.060
GUARD_REST = 0.0                                # rad: brake off, the guard stands back toward the front handle
GUARD_LIMITS = (-0.62, 0.12)                    # rad about +z; negative swings the free end toward the bar tip
BRAKE_DETENT = -0.30                            # past this (more negative) the guard snaps over centre to the brake stop
BRAKE_ON = -0.45                                # at or past this, the chain brake is engaged
BRAKE_STOP = -0.58                              # the engaged stop (the drive target once over the detent)
BRAKE_RELEASE = -0.12                           # brought back past this (less negative), the guard returns to rest
GUARD_K = 1.6                                   # N m / rad: the spring drive toward its current target
GUARD_DAMP = 0.08                               # N m s / rad

# -- the scabbard (bar cover): a plastic channel, open at one end -------------------------------------------------------------
SCAB_LEN = 0.265                                # outer length (y)
SCAB_WALL = 0.003
SCAB_IN_W = 2 * BAR_OUT_HALF_W + 0.014         # inner width: 7 mm clearance each side of the chain outline = 46 mm
SCAB_IN_H = CHAIN_H + 0.012                     # inner height: 6 mm clearance above and below the chain = 19.5 mm
SCAB_OUT_W = SCAB_IN_W + 2 * SCAB_WALL
SCAB_OUT_H = SCAB_IN_H + 2 * SCAB_WALL
SCAB_MASS = 0.140
SCAB_FRICTION = (0.5, 0.4)

# where the scabbard rests at the start: lying on a holder beyond the bar tip, open end (+y) 5 cm from the tip
SCAB_START_GAP = 0.050
SCAB_X = SAW_X
SCAB_OPEN_Y_START = SAW_Y - S * (BAR_LEN + SCAB_START_GAP)      # the open end's world y
SCAB_CENTER_Y_START = SCAB_OPEN_Y_START - S * SCAB_LEN / 2
SCAB_YAW = 0.0 if VARIANT == "A" else math.pi                   # its open end faces the bar
SCAB_CENTER_Z = BAR_Z                                           # its channel's mid-height equals the bar's
HOLDER_W, HOLDER_L = 0.040, 0.100               # the holder block under the scabbard (narrower than it, so the pads clear)
HOLDER_Y = SCAB_CENTER_Y_START - S * 0.030      # its centre, towards the scabbard's closed half
HOLDER_TOP = SCAB_CENTER_Z - SCAB_OUT_H / 2     # world height above the table of its top face

# -- scoring thresholds ---------------------------------------------------------------------------------------------------
SAW_MOVE_LIMIT = 0.030                          # m: the saw pushed this far from where it started is a failure
SAW_TILT_LIMIT = math.radians(10)               # the saw tipped past this is a failure
COVER_FULL = 0.95                               # the bar's visible length that must be covered
COVER_DX, COVER_DZ = 0.012, 0.010               # the scabbard's centre off the bar's axis / mid-plane (m)
COVER_TILT = math.radians(8)
COVER_YAW = math.radians(6)
HOLD_S = 2.0
CHAIN_ZONE = dict(half_w=BAR_OUT_HALF_W + 0.012, dz=0.020)      # a gripper tool point inside this is "touching the chain"
