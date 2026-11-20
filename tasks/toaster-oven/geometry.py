"""Dimensions and layout of the toaster-oven task.

Station frame: metres from the midpoint between the arm bases, x forward (away from the arms), y to the robot's left,
z up. Heights below are above the table top (TOP) unless a name says _Z (world height).

The oven is a BLACK+DECKER TO1313SBD 4-slice toaster oven: listed 15.5 x 11.2 x 8.3 in (394 wide, 284 deep, 211 tall),
5.0 lb, with its own bake pan and slide rack. Everything inside it (cavity, door, rack, pan, knob sizes) is not
published; those values are marked (est.) and the submitted text calls them estimates.

Oven frame: the oven's front face is the plane x = OVEN_X (the door's outer face when shut); y and z as the station's.
Door angle: 0 shut, positive opening toward the robot (the joint itself turns about +y, so q = -angle).
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475                        # the station's table top (world z)
PHYSICS_HZ = 240                    # yam_arena's default

# -- the oven (outer size as listed) -------------------------------------------------------------------------------------
OVEN_W, OVEN_D, OVEN_H = 0.394, 0.284, 0.211
OVEN_MASS = 2.3
OVEN_X = 0.380                      # front face
OVEN_Y0 = -0.020                    # its right-hand end (the control column)
OVEN_Y1 = OVEN_Y0 + OVEN_W          # 0.374
OVEN_X1 = OVEN_X + OVEN_D           # back
FOOT_H = 0.012                      # (est.)
COLUMN_W = 0.090                    # the control column at the right-hand end (est.)
COLUMN_Y1 = OVEN_Y0 + COLUMN_W
# cavity (est.)
CAV_Y0, CAV_Y1 = COLUMN_Y1 + 0.017, OVEN_Y1 - 0.017     # 270 wide
CAV_YC = (CAV_Y0 + CAV_Y1) / 2
DOOR_T = 0.016                      # the door's thickness (est.)
CAV_X0 = OVEN_X + DOOR_T            # the opening behind the shut door
CAV_X1 = CAV_X0 + 0.240             # 240 deep (est.)
CAV_Z0, CAV_Z1 = 0.050, 0.185       # floor and ceiling (est.)
WALL = 0.010                        # body walls (collision boxes)
ELEM_Z_TOP, ELEM_Z_BOT = 0.160, 0.072   # heating element rods (est.)
ELEM_R = 0.004
# door: bottom-hinged, opens toward the robot onto a stop (est.)
DOOR_W, DOOR_H = 0.290, 0.140
HINGE_X, HINGE_Z = OVEN_X + DOOR_T / 2, CAV_Z0         # the hinge axis (along y) at the door's bottom edge
DOOR_OPEN = math.radians(80.0)      # the open stop, 10 degrees above level
DOOR_MASS = 0.45                    # (est.)
BAR_R = 0.006                       # handle bar, 12 across
BAR_L = 0.220
BAR_UP = 0.128                      # its axis this far up the door from the hinge (12 below the top edge)
BAR_PROUD = 0.028                   # its axis this far proud of the door's outer face
WINDOW = (0.230, 0.090)             # the glass (y, up the door)
WINDOW_UP = 0.022                   # its bottom edge this far up the door from the hinge
# door spring (task code): pulls it shut below DOOR_TOGGLE; gravity lets it drop onto its stop above (est.)
DOOR_SPRING = 0.20                  # N m toward shut (constant-force spring)
DOOR_DETENT = 0.15                  # N m more over the last DETENT_ANG before shut
DETENT_ANG = math.radians(4.0)
DOOR_DAMP = 0.30                    # N m s/rad
# rack: slides out along -x to a stop (est.)
RACK_W, RACK_D = 0.260, 0.226
RACK_Z = 0.100                      # wire centres
WIRE_R = 0.002
RACK_X0 = CAV_X0 + 0.008            # the front wire's centre, rack home
RACK_OUT = 0.150                    # drawn out this far to its stop
RACK_MASS = 0.25
RACK_DAMP = 3.0                     # N s/m
# bake pan: free, on the rack (est.)
PAN_W, PAN_D, PAN_H, PAN_T = 0.250, 0.195, 0.020, 0.003
PAN_MASS = 0.30
PAN_X0 = RACK_X0 + 0.016            # its front wall's outer face, rack home
PAN_Z = RACK_Z + WIRE_R             # its underside
# knobs on the control column (est.): Toast/Timer is the bottom one
KNOB_Y = OVEN_Y0 + COLUMN_W / 2
KNOB_R = 0.0175                     # 35 across
KNOB_DEPTH = 0.018                  # proud of the front face
KNOB_Z = (0.070, 0.118, 0.166)      # Toast/Timer, function, temperature
RIDGE = (0.008, 0.030, 0.012)       # the grip ridge across the knob's face: thick, long, proud (est.)
KNOB_MASS = 0.02
TIMER_DRAG = 0.15                   # N m to turn the Toast/Timer knob (task code, stick-slip)
TIMER_K = 3.0                       # N m/rad: its stick spring
TIMER_DAMP = 0.005
TIMER_MAX = math.radians(300.0)     # 30 minutes (est.)
TIMER_DETENT = 0.05                 # N m holding OFF within DETENT of it
ON_ANGLE = math.radians(15.0)       # on: 15 degrees or more off OFF

# power cord (visual): out of the back near the control column, straight back across the table and over its far edge
CORD_Y = OVEN_Y0 + 0.030
CORD_R = 0.0035
TABLE_X1 = 0.910                    # the station table's far edge

# -- the paper plate: AJM Packaging Green Label 9 in uncoated (listed 9 in) ------------------------------------------
PLATE_R = 0.1143                    # 9 in
PLATE_FLAT_R = 0.088                # flat bottom (est.)
PLATE_RIM_H = 0.018                 # rim rise at the edge (est.)
PLATE_T = 0.002                     # the sim's rigid disc (the real paper is thinner)
PLATE_MASS = 0.010                  # (est.)
PLATE_FLUTES = 60
PLATE_X, PLATE_Y = 0.130, -0.130    # start, its centre

# -- the bagel: Thomas' Plain Pre-Sliced, 95 g; two halves, cut side up (est. sizes) ---------------------------------
BAGEL_R = 0.050                     # 100 across
HOLE_R = 0.015                      # 30 hole
HALF_T = 0.020
HALF_MASS = 0.0475
HALF_SEGS = 24
HALF_GAP = 0.012                    # between the two halves on the plate
HALF_X = (PLATE_X - BAGEL_R - HALF_GAP / 2, PLATE_X + BAGEL_R + HALF_GAP / 2)   # half_a (near), half_b (far)

# -- scoring constants ----------------------------------------------------------------------------------------------------
NEAR = 0.100                        # the plate this close to the oven (or in it) while the oven is on: 0
CLEAR = 0.030                       # grippers this far from the oven, pan and bagel at the end
SHUT_GAP = 0.005                    # door top edge within this of the frame: shut
SHUT_ANGLE = SHUT_GAP / DOOR_H
REST_TOL = 0.006                    # a half resting on the pan or rack: its lowest point within this of the surface
HOLD_STEPS = 20                     # 1 s at 20 Hz


def door_point(local, angle: float) -> np.ndarray:
    """A point on the door given in the shut door's frame offsets from the hinge (dx toward -x positive = outward is
    negative x, dz up), turned open by ``angle``; returns (x, z) relative to the table top."""
    dx, dz = local
    c, s = math.cos(angle), math.sin(angle)
    return np.array([HINGE_X + dx * c - dz * s, HINGE_Z + dx * s + dz * c])


def bar_centre(angle: float) -> np.ndarray:
    """The handle bar's axis (x, z) for a door angle."""
    return door_point((-(DOOR_T / 2 + BAR_PROUD), BAR_UP), angle)


def cavity_box():
    """The cavity as (lo, hi), table-top heights, from the door's plane to the back wall."""
    return np.array([OVEN_X, CAV_Y0, CAV_Z0]), np.array([CAV_X1, CAV_Y1, CAV_Z1])
