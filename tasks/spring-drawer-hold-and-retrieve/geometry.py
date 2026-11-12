"""Geometry and physical station layout for spring-drawer-hold-and-retrieve. Pure Python.

World frame: x forward, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Contents:
- Cabinet block at x = 0.415, y = 0.000:
  220 (x) x 160 (y) x 80 (z) mm dark grey enclosure.
  Front face at x = 0.305 m.
  Open-front shell: floor, back wall, two side walls and a cover plate over the back (x >= 0.380 m).
- Spring-return drawer:
  190 (x) x 120 (y) x 30 (z) mm open-top box with 10 mm walls and a 6 mm floor.
  Slides along X toward robot (-X direction), travel 140 mm (0.140 m).
  Spring rate: 18 N/m (max force <= 2.5 N at 140 mm).
  Front handle: 25 (x) x 60 (y) x 35 (z) mm on drawer front face.
- White block:
  40 mm cube, mass 0.10 kg, starts inside covered back part of drawer.
  Only reachable when drawer is out >= 90 mm.
- Green shelf on right at (0.300, -0.280):
  130 x 130 x 10 mm.
- Open-position cyan lamp: 25 mm marker on the cover plate at (0.450, 0.000).
  Lights when drawer displacement >= 90 mm.
"""

from __future__ import annotations

TOP = 0.7475

# Colours (sRGB)
DARK_GREY = (0.22, 0.22, 0.24)
CABINET_GREY = (0.35, 0.35, 0.38)
DRAWER_INNER = (0.50, 0.50, 0.52)
HANDLE_ORANGE = (0.90, 0.45, 0.10)
WHITE_BLOCK = (0.95, 0.95, 0.95)
GREEN_SHELF = (0.15, 0.75, 0.25)
LAMP_CYAN = (0.15, 0.85, 0.85)

# Cabinet
CABINET_SIZE = (0.220, 0.160, 0.080)
CABINET_FRONT_X = 0.305
CABINET_POS = (CABINET_FRONT_X + CABINET_SIZE[0] / 2, 0.000, TOP + CABINET_SIZE[2] / 2)
CABINET_TOP_Z = TOP + CABINET_SIZE[2]

# Drawer
DRAWER_SIZE = (0.190, 0.120, 0.030)
DRAWER_TRAVEL = 0.140      # 140 mm travel toward robot (-X)
DRAWER_SPRING_K = 18.0     # 18 N/m -> 2.52 N max force
DRAWER_DAMPING = 5.0
DRAWER_MASS = 0.400

# Drawer local rest position in cabinet
DRAWER_REST_Z = TOP + 0.010 + DRAWER_SIZE[2] / 2
DRAWER_REST_X = CABINET_FRONT_X + DRAWER_SIZE[0] / 2

# Handle on drawer front face
HANDLE_SIZE = (0.025, 0.060, 0.035)
HANDLE_REST_X = CABINET_FRONT_X - HANDLE_SIZE[0] / 2
HANDLE_REST_Z = DRAWER_REST_Z

# White block
BLOCK_SIZE = (0.040, 0.040, 0.040)
BLOCK_MASS = 0.100
# Starts at back of drawer floor (covered by cabinet top plate)
BLOCK_REST_X = CABINET_FRONT_X + 0.145
DRAWER_FLOOR_BOTTOM_Z = TOP + 0.011           # 1 mm above the cabinet floor
DRAWER_FLOOR_T = 0.006
DRAWER_WALL_T = 0.010
DRAWER_FLOOR_TOP_Z = DRAWER_FLOOR_BOTTOM_Z + DRAWER_FLOOR_T
BLOCK_REST_Z = DRAWER_FLOOR_TOP_Z + BLOCK_SIZE[2] / 2
BLOCK_START_POS = (BLOCK_REST_X, 0.000, BLOCK_REST_Z)

# Thresholds
DRAWER_OPEN_THRESHOLD = 0.090   # 90 mm out lights cyan lamp
DRAWER_CLOSED_THRESHOLD = 0.005 # 5 mm out counts as closed
PINCH_TRAVEL_THRESHOLD = 0.030  # drawer < 30 mm out while the block is still in the cabinet mouth zone = PINCH
PINCH_ZONE_DEPTH = 0.060        # the mouth zone: block centre within 60 mm behind the cabinet front face

# Green shelf on right at (0.300, -0.280)
SHELF_SIZE = (0.130, 0.130, 0.010)
SHELF_POS = (0.300, -0.280, TOP + SHELF_SIZE[2] / 2)
SHELF_TOP_Z = TOP + SHELF_SIZE[2]
SHELF_SPOT = (0.300, -0.280, SHELF_TOP_Z + BLOCK_SIZE[2] / 2)

# Open position lamp (marker) on cabinet top
LAMP_SIZE = (0.025, 0.025, 0.004)
LAMP_POS = (0.450, 0.000, CABINET_TOP_Z + LAMP_SIZE[2] / 2)   # on the cover plate
LAMP_ON_Z = LAMP_POS[2]
LAMP_OFF_Z = LAMP_POS[2] - 0.050

# Arm Grasp and Navigation Heights
HOVER_Z = TOP + 0.110           # ~0.8575 m (clears cabinet top and drawer)
GRASP_HANDLE_Z = HANDLE_REST_Z  # ~0.7725 m
GRASP_BLOCK_Z = BLOCK_REST_Z    # ~0.7775 m

PAD_TO_TCP = 0.013
MAX_ARM_REACH = 0.50
LEFT_BASE = (0.0, +0.305)
RIGHT_BASE = (0.0, -0.305)



# Cover plate over the back of the cabinet: hides the block until the drawer is out about 90 mm
COVER_FRONT_X = 0.380
COVER_BOTTOM_Z = TOP + 0.060
CABINET_WALL_T = 0.019
CABINET_FLOOR_T = 0.010
CABINET_BACK_T = 0.015


def cabinet_world_boxes() -> list[tuple]:
    """Open-front cabinet shell in world coordinates: floor, back wall, two side walls and the cover plate."""
    x0 = CABINET_FRONT_X
    x1 = CABINET_FRONT_X + CABINET_SIZE[0]
    hw = CABINET_SIZE[1] / 2
    h = CABINET_SIZE[2]
    side_len = x1 - CABINET_BACK_T - x0
    side_y = hw - CABINET_WALL_T / 2
    cover_len = x1 - COVER_FRONT_X
    return [
        (((x0 + x1) / 2, 0.0, TOP + CABINET_FLOOR_T / 2), (x1 - x0, CABINET_SIZE[1], CABINET_FLOOR_T), CABINET_GREY),
        ((x1 - CABINET_BACK_T / 2, 0.0, TOP + h / 2), (CABINET_BACK_T, CABINET_SIZE[1], h), CABINET_GREY),
        ((x0 + side_len / 2, +side_y, TOP + CABINET_FLOOR_T + (h - CABINET_FLOOR_T) / 2), (side_len, CABINET_WALL_T, h - CABINET_FLOOR_T), CABINET_GREY),
        ((x0 + side_len / 2, -side_y, TOP + CABINET_FLOOR_T + (h - CABINET_FLOOR_T) / 2), (side_len, CABINET_WALL_T, h - CABINET_FLOOR_T), CABINET_GREY),
        ((x1 - cover_len / 2, 0.0, (COVER_BOTTOM_Z + TOP + h) / 2), (cover_len, CABINET_SIZE[1], TOP + h - COVER_BOTTOM_Z), CABINET_GREY),
    ]


def drawer_world_boxes() -> list[tuple]:
    """The drawer as an open-top box (world coordinates, closed position): floor, front, back and two side walls."""
    x0 = DRAWER_REST_X - DRAWER_SIZE[0] / 2
    x1 = DRAWER_REST_X + DRAWER_SIZE[0] / 2
    hw = DRAWER_SIZE[1] / 2
    top = TOP + 0.010 + DRAWER_SIZE[2] + 0.001
    zb = DRAWER_FLOOR_BOTTOM_Z
    wall_h = top - zb
    inner = x1 - x0 - 2 * DRAWER_WALL_T
    return [
        (((x0 + x1) / 2, 0.0, zb + DRAWER_FLOOR_T / 2), (x1 - x0, DRAWER_SIZE[1], DRAWER_FLOOR_T), DRAWER_INNER),
        ((x0 + DRAWER_WALL_T / 2, 0.0, zb + wall_h / 2), (DRAWER_WALL_T, DRAWER_SIZE[1], wall_h), DRAWER_INNER),
        ((x1 - DRAWER_WALL_T / 2, 0.0, zb + wall_h / 2), (DRAWER_WALL_T, DRAWER_SIZE[1], wall_h), DRAWER_INNER),
        (((x0 + x1) / 2, +(hw - DRAWER_WALL_T / 2), zb + DRAWER_FLOOR_T + (wall_h - DRAWER_FLOOR_T) / 2), (inner, DRAWER_WALL_T, wall_h - DRAWER_FLOOR_T), DRAWER_INNER),
        (((x0 + x1) / 2, -(hw - DRAWER_WALL_T / 2), zb + DRAWER_FLOOR_T + (wall_h - DRAWER_FLOOR_T) / 2), (inner, DRAWER_WALL_T, wall_h - DRAWER_FLOOR_T), DRAWER_INNER),
    ]
