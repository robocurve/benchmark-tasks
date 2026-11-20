"""Dimensions and layout of the wheelchair task.

World frame (the station's): metres, x forward from the line through the two arm mounts, y to the robot's left, z up.
The floor is at world z FZ; heights named _H are above the floor.

Chair frame: origin on the floor below the midpoint between the rear-wheel hubs, +x the way the chair faces (its
front, with the casters), +y the chair's left, +z up. The chair is a Drive Medical Blue Streak BLS18FBD-SF (18 x 16 in
seat, seat 19.5 in above the floor, 24 in wide and 36 in tall open, 24 in rear wheels, 8 in casters, desk-length
flip-back arms, push-to-lock wheel locks), its swing-away footrests off. Sizes marked "estimate" are not published.
"""

from __future__ import annotations

import math

import numpy as np

FZ = -0.0125                        # yam_arena station.FLOOR_Z: the floor's world z
PHYSICS_HZ = 240
MOUNT_H = 0.75 - FZ                 # the arm mounts above the floor (76.25 cm)

# -- the chair (chair frame) -------------------------------------------------------------------------------------------
CHAIR_MASS = 17.0                   # without footrests (estimate; 41.25 lb listed with footrests)
R_WHEEL = 0.305                     # 24 in rear wheels
HUB_Y = 0.2735                      # the rear wheels' mid planes
TYRE_W = 0.025                      # 24 x 1 in solid tyre
TYRE_R = TYRE_W / 2
HANDRIM_R, HANDRIM_OUT, HANDRIM_TUBE = 0.270, 0.022, 0.0095   # 61.0 cm over the handrims
R_CASTER, CASTER_W = 0.1016, 0.030  # 8 in casters
CASTER_TRAIL = 0.055                # caster axle from its swivel (estimate)
W_RAIL = 0.2286                     # seat-rail centres, half spacing (18 in seat)
R_RAIL = 0.0125
R_TUBE = 0.011
RAIL_X = (0.02, 0.44)               # seat rail ends (42 cm)
RAIL_Z = 0.480                      # seat rail centre; sling top 49.5 cm
SEAT_TOP = 0.495
SLING_X = (0.03, 0.41)              # the sling seat starts 3 cm behind the rails' front ends
X_FRONT = 0.43                      # front posts and caster swivels
SWIVEL_Z = 0.205                    # fork crown
HOUSING_Z = 0.22
LOWER_Z = 0.14                      # lower side rails
X_CANE = -0.04                      # back canes
HANDLE_Z = 0.900                    # push-handle grips' axis (36 in tall overall)
HANDLE_X = (-0.21, -0.10)           # foam grips
BACK_Z = (0.53, 0.88)               # back upholstery
X_BRACE = 0.17                      # the X cross-brace's plane
ARM_PAD = dict(x=(-0.03, 0.22), y=(0.020, 0.070), z=(0.720, 0.760))   # desk arm pad, y outboard of the rail
GUARD = dict(x=(-0.05, 0.19), y=0.024, z=(0.50, 0.70), t=0.004)       # clothing guard
# wheel lock: a lever on the side frame ahead of the rear tyre, pivoting about y; pushed forward over centre it presses
# its shoe into the tyre (estimates: the lever's geometry)
KNOB_R = 0.012                      # 2.4 cm knob ...
KNOB_LEN = 0.030                    # ... 3 cm long, its axis along y
KNOB_Y = W_RAIL + 0.030             # 3 cm outboard of the seat rail
PIVOT = np.array([0.325, KNOB_Y, 0.300])
KNOB_OFF = np.array([0.300, KNOB_Y, 0.440])      # 44 cm up, 30 cm ahead of the axle
LEVER_L = float(np.hypot(*(KNOB_OFF - PIVOT)[[0, 2]]))
TH_OFF = math.atan2(KNOB_OFF[0] - PIVOT[0], KNOB_OFF[2] - PIVOT[2])   # lever angle from vertical, + forward
TRAVEL = 0.080                      # the knob goes 8 cm forward
TH_ON = math.asin((KNOB_OFF[0] + TRAVEL - PIVOT[0]) / LEVER_L)
TH_C = math.radians(12.0)           # over centre here
TH_BAND = math.radians(1.0)         # pulled back this far past centre, it lets go again
LOCK_F = 20.0                       # N at the knob, along its path, to push it over centre (estimate)
SNAP_F = 8.0                        # N at the knob, toward on, just past centre
OFF_PRELOAD = 0.05                  # rad: the off spring's preload past its stop
ON_PRELOAD = 0.10
K_OFF = LOCK_F * LEVER_L / (TH_C - TH_OFF + OFF_PRELOAD)              # N m/rad
K_ON = SNAP_F * LEVER_L / (TH_ON + ON_PRELOAD - TH_C)
LEVER_MASS = 0.12
SHOE_GAP = 0.004                    # shoe to tyre with the lever off

# -- masses (kg), scaled to CHAIR_MASS -----------------------------------------------------------------------------------
WHEEL_MASS = 2.0
FORK_MASS = 0.20
STRUT_MASS = 0.05                   # the caster stem, sliding in its housing on the spring
CASTER_MASS = 0.35
FRAME_MASS = CHAIR_MASS - 2 * (WHEEL_MASS + FORK_MASS + STRUT_MASS + CASTER_MASS + LEVER_MASS)
FRAME_COM = np.array([0.175, 0.0, 0.40])        # (estimate)
_COM_X = (FRAME_MASS * FRAME_COM[0] + 2 * (FORK_MASS + STRUT_MASS) * X_FRONT
          + 2 * CASTER_MASS * (X_FRONT + CASTER_TRAIL) + 2 * LEVER_MASS * PIVOT[0]) / CHAIR_MASS
WHEEL_DAMP = 0.02                   # N m s/rad: bearing drag
SWIVEL_DAMP = 0.01
LEVER_DAMP = 0.01
SPRING_K = 15000.0                  # N/m: each caster's stem on a stiff spring in its housing, so the rigid frame stands on
SPRING_D = 300.0                    # all four wheels (estimate); N s/m
SPRING_TRAVEL = 0.010               # m each way
SPRING_PRELOAD = CHAIR_MASS * 9.81 * _COM_X / (X_FRONT + CASTER_TRAIL) / 2 / SPRING_K   # m, at the start
AXLE_LOCK_K = 2000.0                # N m/rad: the shoe's hold on a locked wheel
AXLE_LOCK_D = 20.0
AXLE_LOCK_MAX = 60.0                # N m
TYRE_MU = 0.55                      # tyre on vinyl (estimate)
GRIP_MU = (1.0, 0.9)                # foam grips, vinyl pads, powder coat (averaged with the robot's pads)

# -- layout (world) ---------------------------------------------------------------------------------------------------
START_M = np.array([0.60, 0.105])   # hub midpoint at the start
START_YAW = math.pi / 2             # facing +y, away from the bed
REF_M = np.array([0.75, -0.045])    # the reference pose: alongside, facing the bed's foot
REF_YAW = math.pi
TURN_C = np.array([0.75, 0.105])    # the turn from start to reference is a rotation about this point

BED_EDGE_Y = -0.45                  # the mattress's near long edge
BED_X = (-0.665, 1.240)             # foot end, head end
BED_W = 0.965                       # 38 in
BED_FRAME_H = 0.356                 # 14 in platform frame
MATT_T = 0.152                      # 6 in mattress
MATT_TOP_H = BED_FRAME_H + MATT_T   # 50.8
PILLOW = dict(x=(0.96, 1.21), w=0.62, t=0.07)
HIP_X = 0.31                        # the hip tick on the mattress edge
LEE_Y = -0.75                       # Mrs. Lee's centreline
LEE_HEIGHT = 1.78
PEDESTALS = {"left": (0.0, 0.305), "right": (0.0, -0.305)}
PEDESTAL, PEDESTAL_FOOT = (0.10, 0.10), (0.24, 0.24, 0.015)

# -- scoring --------------------------------------------------------------------------------------------------------
ALONG = dict(gap=0.25, heading=math.radians(10.0), hip=0.20)
NEARLY = dict(gap=0.40, heading=math.radians(30.0), hip=0.40)
HOLD_F = 20.0                       # N on the push handle, back then forward (2 kg on a luggage scale)
HOLD_TOL = 0.010                    # a held wheel's hub stays within 1 cm of the tick plumb under it
RAMP_S, KEEP_S = 0.5, 1.0           # each pull and push: ramp to 20 N, keep it
TIP = math.radians(10.0)
TOUCH = 0.002                       # m: closer than this counts as touching
READY_TOL = math.radians(3.0)
STILL_S = 1.0
STILL_V = 0.05                      # rad/s
RUN_LIMIT = 3600


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def knob_at(theta: float, side: float) -> np.ndarray:
    """The knob's centre (chair frame) at lever angle theta; side +1 (chair's left) or -1."""
    p = PIVOT + LEVER_L * np.array([math.sin(theta), 0.0, math.cos(theta)])
    return p * np.array([1.0, side, 1.0])


def turn_pose(phi: float) -> tuple[np.ndarray, float]:
    """The planned chair pose phi into the turn: hub midpoint (world xy) and yaw."""
    return TURN_C + rot_z(phi)[:2, :2] @ (START_M - TURN_C), START_YAW + phi


def chair_to_world(p, m_xy, yaw) -> np.ndarray:
    p = np.asarray(p, float)
    return np.r_[np.asarray(m_xy, float), FZ] + rot_z(yaw) @ p


# grip points (chair frame) the scripted demo uses: (x, y, z of the tool point); side +1 left, -1 right
def rail_front(side: float) -> np.ndarray:
    return np.array([0.420, side * W_RAIL, RAIL_Z + 0.004])


def arm_front(side: float) -> np.ndarray:
    return np.array([0.195, side * (W_RAIL + 0.045), ARM_PAD["z"][1] - 0.012])


def handle_grip(side: float) -> np.ndarray:
    return np.array([-0.165, side * W_RAIL, HANDLE_Z])


def strap_point() -> np.ndarray:
    """The held test's pull point: the middle of a strap looped between the two push handles' grips."""
    return np.array([-0.165, 0.0, HANDLE_Z])
