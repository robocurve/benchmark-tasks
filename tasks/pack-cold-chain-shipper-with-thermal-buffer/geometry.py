"""Geometry, physical dimensions, and station layout for pack-cold-chain-shipper-with-thermal-buffer.

World frame (station frame):
- X forward from arm bases (sagittal axis)
- Y lateral (Y > 0 is robot left, Y < 0 is robot right)
- Z vertical up, table top elevation at z = 0.7475 m (TABLE_Z)

Contents:
- Insulated Shipper Base: Cavity 172 mm (Y) x 150 mm (X) x 70 mm deep centered at (0.280, 0.000).
- Refrigerant Gel Brick: hard-shell gel brick, 152 mm (Y) x 76 mm (X) x 25 mm (Z), mass 0.33 kg.
- Thermal Buffer Barrier Pad: corrugated pad, 150 mm (Y) x 74 mm (X) x 15 mm (Z), mass 0.04 kg.
- Medication Payload Carton: freeze-sensitive drug pack, 130 mm (Y) x 70 mm (X) x 25 mm (Z), mass 0.18 kg.
- Staging Pads: Left pad at (0.220, +0.220), Right pad at (0.220, -0.220), Front pad at (0.115, 0.000).
- Shipper Lid: rigid foam-style slab 170 mm (X) x 192 mm (Y) x 8 mm with a 40 x 30 x 22 mm grip tab, mass 0.12 kg. It
  rests on the shipper rim and is held down by two spring slide latches, one on each Y end of the shipper. Layout A
  stages the lid beside the left pad (y = +0.420), layout B beside the right pad (y = -0.420).

Grasping: every part is pinched across its SHORT side (X, at most 76 mm) with the gripper yawed so the
fingers close along world X (GRIP_YAW_LEFT = -pi/2, GRIP_YAW_RIGHT = +pi/2). The pad tips sit 13 mm below
the commanded TCP point, so the commanded TCP height for a part resting on a surface at height z0 is z0 + GRIP_DZ.
"""

from __future__ import annotations

import math
import numpy as np

# Table surface elevation
TABLE_Z = 0.7475
TOP = TABLE_Z

# Control and dwell parameters
CONTROL_HZ = 20.0
HOLD_DWELL_S = 3.0
HOLD_STEPS = int(HOLD_DWELL_S * CONTROL_HZ)  # 60 steps at 20 Hz
DEMO_TIME_SCALE = 1.8

# Kinematic and audit tolerances
POS_TOL_MM = 3.0
ROT_TOL_DEG = 3.0
LIMIT_MARGIN_RAD = 0.05
TILT_TOL_DEG = 4.0
VEL_TOL = 0.05

# ---------------------------------------------------------------------------------------------
# Gripper model (measured on the YAM arms in yam_arena)
# ---------------------------------------------------------------------------------------------
# Finger joint value f (m): the pads are 2 * |f| apart. 0 = closed, -0.04695 = fully open (94 mm).
FINGER_WIDE = -0.04695
FINGER_OPEN = -0.04695
FINGER_CLOSED = 0.0
FINGER_GRIP = -0.020       # commanded 40 mm pad gap: well below every part width, so the pads squeeze the part
FINGER_RELEASE = -0.045    # commanded 90 mm pad gap: at least 7 mm of daylight per side around every part
FINGER_GRIP_TAB = -0.004   # commanded 8 mm pad gap for the 40 mm lid tab (the same 30+ mm of squeeze as the other parts)

GRIPPER_MAX_OPEN = 0.094        # m, pad gap with the fingers fully open
MAX_PINCH_WIDTH = 0.080         # m, widest part we pinch (>= 7 mm per side: arms land 3 to 6 mm off)
PAD_TIP_PAST_TCP = 0.013        # m, pad tips are this far BELOW the commanded TCP point
PAD_GAP = 0.0045                # m, pad tips ride this far above the surface a part rests on
GRIP_DZ = PAD_TIP_PAST_TCP + PAD_GAP   # 0.0175 m: commanded TCP height above the bottom of a pinched part
PLACE_DROP = 0.002              # m, a part is released 2 mm above its seat and drops onto it
MIN_ENGAGE = 0.008              # m, pad tips must engage at least this much of the part's side height
FINGER_THICK = 0.012            # m, finger body thickness along the closing axis
FINGER_WIDTH = 0.024            # m, finger body width across the closing axis
FINGER_BODY_LEN = 0.040         # m, finger body extends this far up from the pad tip
FINGER_FREE_PER_SIDE = 0.015    # m, free space needed on each side of a pinched part for the fingers

# Fingers close along world X (yaw = 0 would close along world Y). Yaw +pi/2 and -pi/2 pinch identically;
# each arm takes the one that keeps joint6 well inside its limit over the shipper and the front pad.
GRIP_YAW_LEFT = -math.pi / 2.0
GRIP_YAW_RIGHT = math.pi / 2.0

# Arm home / standby postures (Z elevation above table)
HOME_L = (0.180, 0.220, 0.855)
HOME_R = (0.180, -0.220, 0.855)
HOVER_Z = 0.855
TRANSIT_Z = 0.905   # carried parts ride 15 mm or more above the shipper rim and the latch heads (a loaded arm sags about 10 mm)
LID_TRANSIT_Z = 0.910   # the carried lid rides 15 mm or more above the latch heads
RETURN_Z = 0.905        # after a release the empty gripper lifts here, so its fingers clear the latch posts on the way home


def grip_tcp_z(rest_z: float) -> float:
    """Commanded TCP height that puts the pad tips PAD_GAP above a surface at height ``rest_z``."""
    return rest_z + GRIP_DZ


def pad_tip_z(tcp_z: float) -> float:
    return tcp_z - PAD_TIP_PAST_TCP


# Staging pads (thin flat guide plates resting on table)
PAD_THICKNESS = 0.004  # 4 mm
PAD_SIZE_X = 0.110
PAD_SIZE_Y = 0.180
PAD_SURFACE_Z = TABLE_Z + PAD_THICKNESS  # 0.7515 m

PAD_LEFT_X = 0.220
PAD_LEFT_Y = 0.220
PAD_LEFT_Z = TABLE_Z + PAD_THICKNESS / 2.0

PAD_RIGHT_X = 0.220
PAD_RIGHT_Y = -0.220
PAD_RIGHT_Z = TABLE_Z + PAD_THICKNESS / 2.0

PAD_FRONT_X = 0.115
PAD_FRONT_Y = 0.000
PAD_FRONT_Z = TABLE_Z + PAD_THICKNESS / 2.0

SPAWN_GAP = 0.0005  # parts spawn this far above their pad so nothing starts in contact or overlap

# Shipper Base Container (Insulated Expanded Foam Shipper)
SHIPPER_X = 0.280
SHIPPER_Y = 0.000
CAVITY_LENGTH_Y = 0.172  # 172 mm cavity along Y (long side of every part)
CAVITY_WIDTH_X = 0.150   # 150 mm cavity along X (the pinched axis: the finger hulls flare to 24 mm thick above the pads)
CAVITY_DEPTH_Z = 0.070   # 70 mm deep cavity

# Standard aliases for consistency
CAVITY_LENGTH = CAVITY_LENGTH_Y
CAVITY_WIDTH = CAVITY_WIDTH_X
CAVITY_DEPTH = CAVITY_DEPTH_Z

SHIPPER_FLOOR_THICKNESS = 0.010  # 10 mm bottom floor
SHIPPER_WALL_THICKNESS = 0.010   # 10 mm insulated walls

CAVITY_FLOOR_Z = TABLE_Z + SHIPPER_FLOOR_THICKNESS  # 0.7575 m
CAVITY_RIM_Z = CAVITY_FLOOR_Z + CAVITY_DEPTH_Z      # 0.8275 m

SHIPPER_EXT_X = CAVITY_WIDTH_X + 2.0 * SHIPPER_WALL_THICKNESS   # 0.170 m
SHIPPER_EXT_Y = CAVITY_LENGTH_Y + 2.0 * SHIPPER_WALL_THICKNESS  # 0.192 m
SHIPPER_TOTAL_HEIGHT = SHIPPER_FLOOR_THICKNESS + CAVITY_DEPTH_Z  # 0.080 m
SHIPPER_MASS = 2.5                                               # kg
SHIPPER_FRICTION = 0.3   # low enough that a lid's end edge slides over the round latch head instead of sticking to it

# Gel brick: hard-shell refrigerant brick, about 6 x 3 x 1 in
BRICK_LENGTH_Y = 0.152  # 152 mm long side (along Y)
BRICK_WIDTH_X = 0.076   # 76 mm pinched side (along X)
BRICK_HEIGHT_Z = 0.025  # 25 mm thickness
BRICK_MASS = 0.33       # kg (about 295 cm3 of gel plus shell)
BRICK_FRICTION = 1.0

BRICK_LENGTH = BRICK_LENGTH_Y
BRICK_WIDTH = BRICK_WIDTH_X
BRICK_HEIGHT = BRICK_HEIGHT_Z

BRICK_CLEARANCE_X = (CAVITY_WIDTH_X - BRICK_WIDTH_X) / 2.0     # 0.037 m (37 mm per side)
BRICK_CLEARANCE_Y = (CAVITY_LENGTH_Y - BRICK_LENGTH_Y) / 2.0   # 0.010 m (10 mm per side)

# Target position when placed flat inside shipper cavity (Layer 1: Bottom)
BRICK_TARGET_X = SHIPPER_X
BRICK_TARGET_Y = SHIPPER_Y
BRICK_TARGET_Z = CAVITY_FLOOR_Z + BRICK_HEIGHT_Z / 2.0  # 0.7575 + 0.0125 = 0.7700 m
BRICK_TOP_SEATED_Z = CAVITY_FLOOR_Z + BRICK_HEIGHT_Z    # 0.7825 m

# Corrugated thermal buffer barrier pad
BUFFER_LENGTH_Y = 0.150  # 150 mm long side (along Y)
BUFFER_WIDTH_X = 0.074   # 74 mm pinched side (along X)
BUFFER_HEIGHT_Z = 0.015  # 15 mm thickness
BUFFER_MASS = 0.04       # kg
BUFFER_FRICTION = 1.0

BUFFER_LENGTH = BUFFER_LENGTH_Y
BUFFER_WIDTH = BUFFER_WIDTH_X
BUFFER_HEIGHT = BUFFER_HEIGHT_Z

BUFFER_CLEARANCE_X = (CAVITY_WIDTH_X - BUFFER_WIDTH_X) / 2.0     # 0.038 m (38 mm per side)
BUFFER_CLEARANCE_Y = (CAVITY_LENGTH_Y - BUFFER_LENGTH_Y) / 2.0   # 0.011 m (11 mm per side)

# Target position when seated over gel brick inside shipper (Layer 2: Middle)
BUFFER_TARGET_X = SHIPPER_X
BUFFER_TARGET_Y = SHIPPER_Y
BUFFER_TARGET_Z = BRICK_TOP_SEATED_Z + BUFFER_HEIGHT_Z / 2.0  # 0.7825 + 0.0075 = 0.7900 m
BUFFER_TOP_SEATED_Z = BRICK_TOP_SEATED_Z + BUFFER_HEIGHT_Z    # 0.7975 m

# Medication payload carton (about 130 x 70 x 25 mm, a small vial or pen carton)
CARTON_LENGTH_Y = 0.130  # 130 mm long side (along Y)
CARTON_WIDTH_X = 0.070   # 70 mm pinched side (along X)
CARTON_HEIGHT_Z = 0.025  # 25 mm thickness
CARTON_MASS = 0.18       # kg
CARTON_FRICTION = 1.0

CARTON_LENGTH = CARTON_LENGTH_Y
CARTON_WIDTH = CARTON_WIDTH_X
CARTON_HEIGHT = CARTON_HEIGHT_Z

CARTON_CLEARANCE_X = (CAVITY_WIDTH_X - CARTON_WIDTH_X) / 2.0     # 0.040 m (40 mm per side)
CARTON_CLEARANCE_Y = (CAVITY_LENGTH_Y - CARTON_LENGTH_Y) / 2.0   # 0.021 m (21 mm per side)

# Target position when seated over buffer pad inside shipper (Layer 3: Top)
CARTON_TARGET_X = SHIPPER_X
CARTON_TARGET_Y = SHIPPER_Y
CARTON_TARGET_Z = BUFFER_TOP_SEATED_Z + CARTON_HEIGHT_Z / 2.0  # 0.7975 + 0.0125 = 0.8100 m
CARTON_TOP_SEATED_Z = BUFFER_TOP_SEATED_Z + CARTON_HEIGHT_Z    # 0.8225 m (5 mm below the 0.8275 rim)

# Staging heights on table pads (part centre; a 0.5 mm spawn gap above the pad plate)
BRICK_STAGING_Z = PAD_SURFACE_Z + SPAWN_GAP + BRICK_HEIGHT_Z / 2.0    # 0.7645 m
BUFFER_STAGING_Z = PAD_SURFACE_Z + SPAWN_GAP + BUFFER_HEIGHT_Z / 2.0  # 0.7595 m
CARTON_STAGING_Z = PAD_SURFACE_Z + SPAWN_GAP + CARTON_HEIGHT_Z / 2.0  # 0.7645 m

# Freezing hazard contact threshold:
# Carton centre minus brick centre is 0.040 m with the buffer pad between them and 0.025 m when the
# carton sits directly on the brick. Direct thermal contact threshold is set midway at 0.0325 m.
CONTACT_FREEZE_DIST_Z = 0.0325  # m

# Scoring placement tolerances
SEATED_TOL_XY = 0.016  # +-16 mm center tolerance within cavity
SEATED_TOL_Z = 0.012   # +-12 mm vertical seating tolerance

# Pick / place commanded TCP heights (derived, single source of truth)
PICK_TCP_Z = grip_tcp_z(PAD_SURFACE_Z + SPAWN_GAP)                 # every part rests on a pad plate
BRICK_PLACE_TCP_Z = grip_tcp_z(CAVITY_FLOOR_Z) + PLACE_DROP        # part bottom on the cavity floor
BUFFER_PLACE_TCP_Z = grip_tcp_z(BRICK_TOP_SEATED_Z) + PLACE_DROP   # on top of the brick
CARTON_PLACE_TCP_Z = grip_tcp_z(BUFFER_TOP_SEATED_Z) + PLACE_DROP  # on top of the buffer
CARTON_DIRECT_PLACE_TCP_Z = grip_tcp_z(BRICK_TOP_SEATED_Z) + PLACE_DROP  # ablation: straight onto the brick

# ---------------------------------------------------------------------------------------------
# Shipper lid and the two spring slide latches (the closing mechanism)
# ---------------------------------------------------------------------------------------------
# The lid is one rigid slab the size of the shipper's outer footprint, so it sits on the 10 mm rim all round. A grip
# tab stands on its top face. Two latches sit outside the shipper's two Y ends. Each is a rigid slider on a prismatic
# joint pushed back by a spring: a vertical post outside the end wall carries a round head (a rod along X) that hangs
# over the lid's end edge when the lid is seated. Pressing the lid down, its lower end edge rides over the round head
# and pushes the slider outward; once the lid is seated the spring returns the head over the lid's top face.
LID_LENGTH_Y = SHIPPER_EXT_Y   # 0.192 m
LID_WIDTH_X = SHIPPER_EXT_X    # 0.170 m
LID_THICK_Z = 0.008            # 8 mm slab
LID_MASS = 0.12                # kg
LID_FRICTION = 0.3
LID_TAB_X = 0.040              # 40 mm: the pinched width, along the closing axis
LID_TAB_Y = 0.030              # 30 mm: wider than the 24 mm finger pad
LID_TAB_Z = 0.022              # 22 mm tall
LID_PAD_GAP = 0.005            # m, pad tips ride this far above the lid's top face when it is pinched by the tab
LID_GRIP_DZ = PAD_TIP_PAST_TCP + LID_PAD_GAP   # 0.018 m: commanded TCP height above the top face of the lid

LID_SEAT_BOTTOM_Z = CAVITY_RIM_Z                       # 0.8275 m: the lid's underside rests on the rim
LID_SEATED_Z = LID_SEAT_BOTTOM_Z + LID_THICK_Z / 2.0   # 0.8315 m, lid centre when seated
LID_TOP_SEATED_Z = LID_SEAT_BOTTOM_Z + LID_THICK_Z     # 0.8355 m
LID_TAB_TOP_SEATED_Z = LID_TOP_SEATED_Z + LID_TAB_Z    # 0.8575 m

# Latch geometry, given in the shipper's frame (origin at the shipper centre on the table, z up from the table)
LATCH_RADIUS = 0.007           # m, the round head is a 14 mm rod along X
LATCH_OVERLAP = 0.004          # m, the head hangs this far over the lid's end edge when the lid is seated
LATCH_GAP_ABOVE_LID = 0.0015   # m, a seated lid's top face is this far under the head
LATCH_WIDTH_X = 0.030          # m, the head rod and post are 30 mm wide (X)
LATCH_TRAVEL = 0.008           # m, the slider moves up to 8 mm outward
LATCH_RATE = 500.0             # N/m (0.5 N/mm): the spring that pushes the head back over the lid
LATCH_DAMPING = 5.0            # N s/m
LATCH_MASS = 0.01              # kg per slider
LATCH_TIP_Y = LID_LENGTH_Y / 2.0 - LATCH_OVERLAP         # 0.092 m: inner point of a head at rest (distance from the centreline)
LATCH_CENTRE_Y = LATCH_TIP_Y + LATCH_RADIUS              # 0.099 m: head rod axis
LATCH_CENTRE_Z = (LID_SEAT_BOTTOM_Z - TABLE_Z) + LID_THICK_Z + LATCH_GAP_ABOVE_LID + LATCH_RADIUS  # 0.0965 m above the table
LATCH_POST_Y0 = LID_LENGTH_Y / 2.0 + 0.004               # post starts 4 mm outside the shipper's end face
LATCH_POST_Y1 = LATCH_CENTRE_Y + LATCH_RADIUS            # and ends level with the head's outer side
LATCH_POST_Z0 = 0.060                                    # m above the table (the post reaches down beside the end wall)
LATCH_HEAD_TOP_Z = TABLE_Z + LATCH_CENTRE_Z + LATCH_RADIUS   # 0.851 m world height of the head tops
# Where the lid's lower end edge rests on a round head when the lid is laid on the latches without pressing: the edge is
# LATCH_RADIUS - LATCH_OVERLAP = 3 mm outside the head axis, so it sits on the rod's circle sqrt(r^2 - d^2) above the axis
LID_ON_LATCH_BOTTOM_Z = TABLE_Z + LATCH_CENTRE_Z + math.sqrt(LATCH_RADIUS**2 - (LATCH_RADIUS - LATCH_OVERLAP) ** 2)
LID_ON_LATCH_TOP_Z = LID_ON_LATCH_BOTTOM_Z + LID_THICK_Z

# Lid scoring tolerances
LID_SEAT_TOL_Z = 0.0025        # lid centre at most 2.5 mm above its seat (the latch head leaves 1.5 mm of play)
LID_SEAT_TOL_XY = 0.012        # lid centred on the shipper within 12 mm
LID_TILT_TOL_DEG = 3.0
LATCH_MIN_OVERLAP = 0.0005     # a latch is engaged once its head hangs at least 0.5 mm over the lid's end edge

# Lid staging: a plate beside the carton's pad (layout A, left) or the brick's pad (layout B, right)
LID_PAD_X = 0.300
LID_PAD_Y_A = 0.420
LID_PAD_Y_B = -0.420
LID_PAD_SIZE_X = LID_WIDTH_X + 0.006
LID_PAD_SIZE_Y = LID_LENGTH_Y + 0.006
LID_STAGING_Z = PAD_SURFACE_Z + SPAWN_GAP + LID_THICK_Z / 2.0   # 0.7560 m (lid centre on its plate)
LID_TOP_STAGED_Z = PAD_SURFACE_Z + SPAWN_GAP + LID_THICK_Z      # 0.7600 m

# Lid commanded TCP heights (the pad tips ride 5 mm above the lid's top face)
LID_PICK_TCP_Z = LID_TOP_STAGED_Z + LID_GRIP_DZ                 # 0.778 m
LID_SEAT_TCP_Z = LID_TOP_SEATED_Z + LID_GRIP_DZ                 # 0.8535 m: lid on the rim
LID_ON_LATCH_TCP_Z = LID_ON_LATCH_TOP_Z + LID_GRIP_DZ           # 0.876 m: lid resting on the latch heads
LID_PRESS_OVERSHOOT = 0.020                                     # m, commanded below the seat: the soft arm gives and the rim stops the lid
LID_PRESS_TCP_Z = LID_SEAT_TCP_Z - LID_PRESS_OVERSHOOT

# Visual colors (sRGB 0..1)
COLOR_SHIPPER = (0.92, 0.92, 0.94)        # Insulated white cooler box
COLOR_PAD = (0.28, 0.32, 0.38)            # Dark slate staging pad
COLOR_BRICK = (0.10, 0.45, 0.82)          # Sub-zero refrigerant gel blue
COLOR_BUFFER = (0.72, 0.54, 0.38)         # Corrugated kraft brown pad
COLOR_CARTON = (0.96, 0.96, 0.97)         # Medical pharmaceutical carton
COLOR_LABEL_INSULIN = (0.12, 0.32, 0.85)  # Insulin blue freeze-sensitive stripe
COLOR_LABEL_HEPARIN = (0.95, 0.58, 0.10)  # Heparin amber thermal warning stripe
COLOR_ICON_WHITE = (0.92, 0.95, 1.00)     # Pictogram white
COLOR_ICON_DARK = (0.20, 0.18, 0.16)      # Hazard dark icon
COLOR_LID = (0.80, 0.85, 0.90)            # Foam shipper lid
COLOR_LATCH = (1.00, 0.50, 0.00)          # Orange spring latches and lid grip tab
