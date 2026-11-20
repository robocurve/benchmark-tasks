"""Dimensions and layout of the mouse-bait task.

Station frame (the robot's): metres from the midpoint between the arm bases, x forward (away from the arms), y to the
robot's left, z up. Heights below are above the table top (TOP) unless a name says _Z (world height).

The bait station is a TOMCAT Rodent Station (Bell Laboratories 33473; listed 241 x 222 x 114 mm). Its frame: origin
at the middle of its floor's underside, x toward the wall (its depth, 222), y along its length (241, toward the robot's
left), z up. The lid is hinged along its left end (+y) and opens toward the robot's left; its frame has its origin on
the hinge axis, and closed it lies along -y from there.

The bait blocks are DETEX BLOX (Bell 993000; listed 51 x 25 x 19 mm, 20 g), standing in for Tomcat Bait Chunx. Their
frame: origin at the centre, x along the length, z through the broad face (the hole's axis).
Sizes marked "estimate" are not published by the makers.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475                        # the station's table top (world z)
FLOOR_Z = -0.0125                   # the floor under the table (world z): the bench top is 760 mm above it
PHYSICS_HZ = 240                    # PhysX rate (yam_arena's default)

# -- the wall (a pegboard panel at the back of the bench) -----------------------------------------------------------------
WALL_X = 0.480                      # its front face
WALL_T = 0.006
WALL_W = 0.900                      # along y, centred on y 0
WALL_H = 0.450
DROP_X = (WALL_X - 0.020, WALL_X)   # the droppings strip at its foot
DROP_Y = (-0.200, 0.200)
N_DROPPINGS = 20

# -- the bait station -------------------------------------------------------------------------------------------------
ST_X, ST_Y = 0.280, 0.000           # its centre at the start (a taped outline)
ST_D = 0.222                        # depth (along x)
ST_L = 0.241                        # length (along y)
ST_H = 0.114                        # closed height
BASE_H = 0.100                      # the base's walls; the lid's skirt sits on them
FLOOR_T = 0.003
SIDE_T = 0.003                      # wall thickness
LID_T = 0.003                       # the lid's top plate
LID_SKIRT = ST_H - BASE_H           # 14 mm: the lid is a shallow cap, its skirt over the walls
BASE_MASS = 0.330                   # 450 g in all (estimate)
LID_MASS = 0.120
HALF_D, HALF_L = ST_D / 2, ST_L / 2
INNER_D, INNER_L = HALF_D - SIDE_T, HALF_L - SIDE_T     # interior half-sizes
ENTRANCE_W, ENTRANCE_H = 0.050, 0.050                   # a mouse entrance in each end wall, at the floor
BAFFLE_Y = HALF_L - 0.040           # an inner baffle 40 mm in from each end (estimate) ...
BAFFLE_H = 0.060                    # ... 60 tall ...
BAFFLE_X = (-INNER_D, 0.040)        # ... from the front wall, leaving a passage at the back
# the four bait-securing rods, fitted upright in the floor
ROD_R = 0.002                       # 4 across (estimate)
ROD_H = 0.060                       # above the floor (estimate)
ROD_TIP = FLOOR_T + ROD_H           # 63 mm above the station's underside
RODS = [(sx * 0.045, sy * 0.035) for sx in (-1, 1) for sy in (-1, 1)]   # 90 apart along x, 70 along y (estimate)
# the lid's hinge: along the left end's top edge
HINGE_Y, HINGE_Z = HALF_L, BASE_H
LID_OPEN = math.radians(100.0)      # its rest angle and stop: 10 degrees past vertical (estimate)
LID_DAMP = 0.06                     # N m s/rad, the hinge's damping: it falls shut in about 1 s (estimate)

# -- the two key locks at the lid's free end (task code) ---------------------------------------------------------------
LOCK_XS = (-0.080, 0.080)           # the two locks, front and back, at the free end (station frame x)
LOCK_F = 10.0                       # N per lock to press it shut (estimate): 20 N at the free end between them
LOCK_AT = math.radians(0.4)         # pressed this close to shut, the locks catch ...
LOCK_DWELL = 0.10                   # ... held there this long (s): a lid let fall shut does not stay there
CATCH = math.radians(1.5)           # over its last 1.5 degrees shut the catches push the lid back up
CATCH_F = 2 * LOCK_F                # the push at the free end at LOCK_AT, rising linearly from 0 at CATCH
CATCH_K = CATCH_F * ST_L / (CATCH - LOCK_AT)    # N m/rad: the catches as a spring in the hinge's drive (implicit)
CATCH_DAMP = 1.0                    # N m s/rad within the catches (they take up a falling lid's speed)

# -- the bait blocks -----------------------------------------------------------------------------------------------------
BLK_L, BLK_W, BLK_T = 0.051, 0.025, 0.019
BLK_MASS = 0.020
HOLE = 0.007                        # a square hole through the broad face, 7 across (estimate)
BLOCKS = {"block_a": (0.190, -0.260), "block_b": (0.265, -0.260)}   # marks; lying flat, long side along x

# -- fixed props -----------------------------------------------------------------------------------------------------------
PAIL_R, PAIL_H = 0.085, 0.180       # (estimate)
PAIL_X, PAIL_Y = 0.390, -0.440
TRUCK_L, TRUCK_W, TRUCK_H = 0.254, 0.191, 0.171     # Green Toys Dump Truck, listed 10 x 7.5 x 6.75 in
TRUCK_X, TRUCK_Y = WALL_X - 0.015 - TRUCK_W / 2, DROP_Y[1] + 0.010 + TRUCK_L / 2   # against the wall, at the strip's end
CUP_R, CUP_H = 0.0375, 0.160        # (estimate)
CUP_X, CUP_Y = 0.180, 0.240

# -- scoring -----------------------------------------------------------------------------------------------------------------
ON_ROD_XY = 0.003                   # the hole's axis within this of a rod's axis ...
ON_ROD_TILT = math.radians(20.0)    # ... the block tilted at most this ...
ON_ROD_DEPTH = 0.005                # ... and its underside at least this far below the rod's tip
WALL_GAP = 0.020                    # along the wall: both back corners within this of the wall's face ...
WALL_TILT = math.radians(10.0)      # ... the station upright within this
TOUCH = 0.002                       # a gripper (collision hulls) within this of the station or a block is touching it
MARK_TOL = 0.020                    # the safety flag: a block within this of its mark, lying flat, is still on its mark
FINAL_S = 1.0                       # the final window: no touching
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)
OFF_BENCH_Z = -0.020                # a centre this far below the table top has left the bench


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def lid_rot(angle: float) -> np.ndarray:
    """The lid frame's rotation in the station frame, opened ``angle`` rad (about -x: the free end rises)."""
    return rot_x(-angle)


def lid_to_station(p, angle: float) -> np.ndarray:
    """A point in the lid frame, in the station frame, the lid opened ``angle`` rad."""
    return np.array([0.0, HINGE_Y, HINGE_Z]) + lid_rot(angle) @ np.asarray(p, float)


FREE_END = np.array([0.0, -ST_L, LID_SKIRT])     # the lid's free end, top, in the lid frame
