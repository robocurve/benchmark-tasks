"""Geometry and physical station layout for interlocked-drawer-transfer. Pure Python.

World frame: x forward, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Contents:
- Cabinet block at x = 0.395 m (front face at x = 0.320 m):
  150 (x) x 400 (y) x 80 (z) mm rigid fixture enclosure.
  Houses two side-by-side open-top drawer bays:
    Left drawer bay centered at y = +0.120 m.
    Right drawer bay centered at y = -0.120 m.
- Two sliding drawers:
  Each 110 (y) x 130 (x) x 60 (z) mm interior cavity.
  10 mm walls, 6 mm floor -> 150 (x) x 130 (y) x 66 (z) mm outer envelope.
  Prismatic slide along -X (toward robot), 110 mm travel (0.110 m).
  Friction-held (near-zero spring rate 0.02 N/m, damping 10.0 N s/m).
  Handle: 20 (x) x 45 (y) x 30 (z) mm centered on drawer front face.
- White block:
  40 x 40 x 40 mm cube, mass 0.050 kg, friction 1.5.
  Starts at rear of left drawer floor (x = 0.430 m, y = +0.120 m).
  Covered by cabinet rear top plate until left drawer is pulled open >= 70 mm.
- Staging pad:
  60 x 60 x 4 mm marked pad at cabinet top front-centre (x = 0.350 m, y = 0.000 m).
- Indicator lamps on a crossbar above the cabinet (x = 0.450 m):
  Green "OK" lamp: 25 mm marker at (0.450, +0.035 m).
  Red "TIP" lamp: 25 mm marker at (0.450, -0.035 m).
"""

from __future__ import annotations

TOP = 0.7475

# Colours (sRGB)
DARK_GREY = (0.20, 0.20, 0.22)
CABINET_GREY = (0.36, 0.36, 0.40)
DRAWER_INNER = (0.52, 0.52, 0.55)
HANDLE_ORANGE = (0.90, 0.45, 0.10)
WHITE_BLOCK = (0.95, 0.95, 0.95)
STAGING_PAD_COLOR = (0.85, 0.70, 0.20)
LAMP_OK_GREEN = (0.15, 0.85, 0.20)
LAMP_TIP_RED = (0.90, 0.15, 0.15)
LAMP_BAR_COLOR = (0.28, 0.28, 0.30)

# Cabinet
CABINET_FRONT_X = 0.290
CABINET_BACK_X = 0.440
CABINET_SIZE_X = round(CABINET_BACK_X - CABINET_FRONT_X, 4)      # 0.150 m
CABINET_POS_X = round((CABINET_FRONT_X + CABINET_BACK_X) / 2, 4)  # 0.365 m
CABINET_WIDTH = 0.400                                   # y in [-0.200, +0.200]
CABINET_HEIGHT = 0.080
CABINET_POS = (CABINET_POS_X, 0.000, TOP + CABINET_HEIGHT / 2)
CABINET_TOP_Z = TOP + CABINET_HEIGHT                    # 0.8275 m

# Drawers
DRAWER_INNER_X = 0.130
DRAWER_INNER_Y = 0.110
DRAWER_INNER_Z = 0.060
DRAWER_WALL_T = 0.010
DRAWER_FLOOR_T = 0.006

DRAWER_SIZE = (
    DRAWER_INNER_X + 2 * DRAWER_WALL_T,  # 0.150 m
    DRAWER_INNER_Y + 2 * DRAWER_WALL_T,  # 0.130 m
    DRAWER_INNER_Z + DRAWER_FLOOR_T,     # 0.066 m
)
DRAWER_TRAVEL = 0.110      # 110 mm travel toward robot (-X)
DRAWER_SPRING_K = 0.02     # Pure friction hold (rate ~0 per V7c lesson to prevent creep)
DRAWER_DAMPING = 20.0
DRAWER_MASS = 0.350

DRAWER_LEFT_Y = +0.120
DRAWER_RIGHT_Y = -0.120

DRAWER_FLOOR_BOTTOM_Z = TOP + 0.008
DRAWER_FLOOR_TOP_Z = DRAWER_FLOOR_BOTTOM_Z + DRAWER_FLOOR_T  # 0.7615 m
DRAWER_RIM_Z = DRAWER_FLOOR_TOP_Z + DRAWER_INNER_Z            # 0.8215 m

DRAWER_REST_X = CABINET_FRONT_X + DRAWER_SIZE[0] / 2          # 0.395 m
DRAWER_REST_Z = DRAWER_FLOOR_BOTTOM_Z + DRAWER_SIZE[2] / 2    # 0.7885 m

# Handles on drawer front faces
HANDLE_SIZE = (0.020, 0.045, 0.030)
HANDLE_REST_X = CABINET_FRONT_X - HANDLE_SIZE[0] / 2          # 0.310 m
HANDLE_REST_Z = DRAWER_REST_Z

# White block
BLOCK_SIZE = (0.040, 0.040, 0.040)
BLOCK_MASS = 0.050
BLOCK_FRICTION = 1.5

BLOCK_REST_Z = DRAWER_FLOOR_TOP_Z + BLOCK_SIZE[2] / 2         # 0.7815 m
BLOCK_START_X = CABINET_FRONT_X + 0.110                       # 0.430 m (rear of left drawer)
BLOCK_START_POS = (BLOCK_START_X, DRAWER_LEFT_Y, BLOCK_REST_Z)

# Staging pad at cabinet top front-centre
STAGING_PAD_SIZE = (0.060, 0.060, 0.004)
STAGING_PAD_POS = (0.310, 0.000, CABINET_TOP_Z + STAGING_PAD_SIZE[2] / 2)
STAGING_SPOT_Z = CABINET_TOP_Z + STAGING_PAD_SIZE[2] + BLOCK_SIZE[2] / 2  # 0.8515 m
STAGING_SPOT = (0.310, 0.000, STAGING_SPOT_Z)

# Lamps on crossbar above cabinet
LAMP_BAR_POS = (0.420, 0.000, CABINET_TOP_Z + 0.015)
LAMP_BAR_SIZE = (0.030, 0.160, 0.008)

LAMP_SIZE = (0.025, 0.025, 0.006)
LAMP_OK_POS = (0.420, +0.035, CABINET_TOP_Z + 0.015 + 0.004 + LAMP_SIZE[2] / 2)
LAMP_TIP_POS = (0.420, -0.035, CABINET_TOP_Z + 0.015 + 0.004 + LAMP_SIZE[2] / 2)
LAMP_ON_Z = LAMP_OK_POS[2]
LAMP_OFF_Z = LAMP_ON_Z - 0.050

# Thresholds & Rules
DRAWER_OPEN_THRESHOLD = 0.015    # > 15 mm displacement counts as OPEN
DRAWER_CLOSED_THRESHOLD = 0.005  # <= 5 mm displacement counts as CLOSED
DRAWER_OPENED_GOAL = 0.070       # >= 70 mm displacement for scoring milestone
TIP_FAULT_TIME = 0.200           # >= 0.2 s simultaneous open counts as TIP FAULT
SUCCESS_HOLD_S = 1.0

# Arm Grasp and Navigation Heights
HOVER_Z = 0.8600           # ~0.860 m (clears cabinet top and staging pad with generous kinematic headroom)
GRASP_HANDLE_Z = HANDLE_REST_Z            # ~0.7885 m
GRASP_BLOCK_DRAWER_Z = BLOCK_REST_Z       # ~0.7815 m
LIFT_BLOCK_CLEAR_Z = DRAWER_RIM_Z + 0.035 # ~0.8565 m (clear of drawer rim)
CARRY_BLOCK_Z = HOVER_Z

PAD_TO_TCP = 0.013
MAX_ARM_REACH = 0.50
LEFT_BASE = (0.0, +0.305)
RIGHT_BASE = (0.0, -0.305)

# Cabinet shell construction constants
COVER_FRONT_X = 0.370
CABINET_FLOOR_T = 0.008
CABINET_BACK_T = 0.012
CABINET_OUTER_WALL_T = 0.015
CABINET_DIVIDER_W = 0.080


def cabinet_world_boxes() -> list[tuple]:
    """Open-front cabinet shell in world coordinates."""
    x0 = CABINET_FRONT_X
    x1 = CABINET_BACK_X
    hw = CABINET_WIDTH / 2
    h = CABINET_HEIGHT
    zf = TOP + CABINET_FLOOR_T / 2
    wall_h = h - CABINET_FLOOR_T
    wall_z = TOP + CABINET_FLOOR_T + wall_h / 2

    # Cover plate over back of drawers (x >= COVER_FRONT_X)
    cov_len = x1 - COVER_FRONT_X
    cov_cx = (COVER_FRONT_X + x1) / 2
    cov_t = 0.006
    cov_z = TOP + h - cov_t / 2

    # Center staging plate between drawers (x in [x0, COVER_FRONT_X], y in [-0.040, +0.040])
    stg_len = COVER_FRONT_X - x0
    stg_cx = (x0 + COVER_FRONT_X) / 2
    stg_w = 0.080

    boxes = [
        # Floor plate
        (((x0 + x1) / 2, 0.0, zf), (x1 - x0, CABINET_WIDTH, CABINET_FLOOR_T), CABINET_GREY),
        # Back wall
        ((x1 - CABINET_BACK_T / 2, 0.0, wall_z), (CABINET_BACK_T, CABINET_WIDTH, wall_h), CABINET_GREY),
        # Left outer side wall
        (((x0 + x1) / 2, +(hw - CABINET_OUTER_WALL_T / 2), wall_z), (x1 - x0, CABINET_OUTER_WALL_T, wall_h), CABINET_GREY),
        # Right outer side wall
        (((x0 + x1) / 2, -(hw - CABINET_OUTER_WALL_T / 2), wall_z), (x1 - x0, CABINET_OUTER_WALL_T, wall_h), CABINET_GREY),
        # Center divider structure between bays
        (((x0 + x1) / 2, 0.0, wall_z), (x1 - x0, CABINET_DIVIDER_W, wall_h), CABINET_GREY),
        # Rear top cover plate over bays (x >= COVER_FRONT_X)
        ((cov_cx, 0.0, cov_z), (cov_len, CABINET_WIDTH, cov_t), CABINET_GREY),
        # Front-centre top plate (under staging pad)
        ((stg_cx, 0.0, cov_z), (stg_len, stg_w, cov_t), CABINET_GREY),
        # Lamp mounting bar
        (LAMP_BAR_POS, LAMP_BAR_SIZE, LAMP_BAR_COLOR),
    ]
    return boxes


def drawer_world_boxes(center_y: float) -> list[tuple]:
    """Drawer box in world coordinates (closed rest position)."""
    x0 = DRAWER_REST_X - DRAWER_SIZE[0] / 2
    x1 = DRAWER_REST_X + DRAWER_SIZE[0] / 2
    hw = DRAWER_SIZE[1] / 2
    zb = DRAWER_FLOOR_BOTTOM_Z
    floor_t = DRAWER_FLOOR_T
    wall_t = DRAWER_WALL_T
    wall_h = DRAWER_SIZE[2]
    wall_z = zb + wall_h / 2
    side_len = x1 - x0 - 2 * wall_t

    return [
        # Drawer floor
        (((x0 + x1) / 2, center_y, zb + floor_t / 2), (x1 - x0, DRAWER_SIZE[1], floor_t), DRAWER_INNER),
        # Front wall
        ((x0 + wall_t / 2, center_y, wall_z), (wall_t, DRAWER_SIZE[1], wall_h), DRAWER_INNER),
        # Back wall
        ((x1 - wall_t / 2, center_y, wall_z), (wall_t, DRAWER_SIZE[1], wall_h), DRAWER_INNER),
        # Left wall
        (((x0 + x1) / 2, center_y + (hw - wall_t / 2), wall_z), (side_len, wall_t, wall_h), DRAWER_INNER),
        # Right wall
        (((x0 + x1) / 2, center_y - (hw - wall_t / 2), wall_z), (side_len, wall_t, wall_h), DRAWER_INNER),
    ]
