"""Geometry and physical station layout for weigh-and-reject-heavy-block. Pure Python.

World frame: x forward, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Contents:
- Three grey 40 mm cubes on supply rack at left (pitch 100 mm along x):
  Block 0 (near, 0.150, +0.300): 0.10 kg (light)
  Block 1 (middle, 0.250, +0.300): 0.30 kg (heavy)
  Block 2 (far, 0.350, +0.300): 0.10 kg (light)
- Supply rack base: 300 x 80 x 10 mm at (0.250, +0.300)
- Spring scale: 140 x 140 x 20 mm base at (0.300, 0.000) with 100 x 100 x 8 mm platform
  on a Z-axis prismatic spring slider (travel 12 mm, k = 600 N/m).
- Readout meter: 50 x 160 x 15 mm housing at (0.450, 0.000) with scale markings,
  needle marker moving along Y with x10 gain, red band beyond 32 mm.
- Green shelf (for light blocks): 260 x 90 x 10 mm at (0.300, +0.120).
- Red reject tray (for heavy block): 130 x 130 x 20 mm (15 mm walls) at (0.300, -0.300).
"""

from __future__ import annotations

TOP = 0.7475  # table top height

# Colours (sRGB)
DARK_GREY = (0.22, 0.22, 0.24)
GREY_BLOCK = (0.55, 0.55, 0.57)
GREEN_SHELF = (0.15, 0.65, 0.25)
RED_TRAY = (0.75, 0.15, 0.15)
SCALE_HOUSING = (0.35, 0.35, 0.38)
SCALE_PLATFORM = (0.70, 0.70, 0.72)
NEEDLE_COLOUR = (0.95, 0.95, 0.10)
MARKER_WHITE = (0.90, 0.90, 0.90)
BAND_RED = (0.85, 0.15, 0.15)
BAND_GREEN = (0.20, 0.75, 0.25)

# Blocks
BLOCK_SIZE = (0.040, 0.040, 0.040)
BLOCK_NAMES = ("block_0", "block_1", "block_2")
HEAVY_INDEX = 1  # block_1 (middle position) is the heavy block
BLOCK_MASSES = {
    "block_0": 0.10,  # light
    "block_1": 0.30,  # heavy
    "block_2": 0.10,  # light
}

# Supply rack
SUPPLY_RACK_SIZE = (0.300, 0.080, 0.010)
SUPPLY_RACK_POS = (0.250, +0.300, TOP + SUPPLY_RACK_SIZE[2] / 2)
SUPPLY_TOP_Z = TOP + SUPPLY_RACK_SIZE[2]

BLOCK_SUPPLY_POS: dict[str, tuple[float, float, float]] = {
    "block_0": (0.150, +0.300, SUPPLY_TOP_Z + BLOCK_SIZE[2] / 2),
    "block_1": (0.250, +0.300, SUPPLY_TOP_Z + BLOCK_SIZE[2] / 2),
    "block_2": (0.350, +0.300, SUPPLY_TOP_Z + BLOCK_SIZE[2] / 2),
}

# Scale
SCALE_BASE_SIZE = (0.140, 0.140, 0.020)
SCALE_BASE_POS = (0.300, 0.000, TOP + SCALE_BASE_SIZE[2] / 2)
SCALE_BASE_TOP_Z = TOP + SCALE_BASE_SIZE[2]

SCALE_PLATFORM_SIZE = (0.100, 0.100, 0.008)
SCALE_TRAVEL = 0.012      # 12 mm downward travel
SCALE_SPRING_K = 600.0    # 600 N/m
SCALE_DAMPING = 12.0      # settles < 1.5 s
SCALE_PLATFORM_MASS = 0.050

# Platform rest height: elevated so when compressed up to 12 mm it stays flush/above housing
SCALE_PLATFORM_REST_Z = SCALE_BASE_TOP_Z + SCALE_TRAVEL + SCALE_PLATFORM_SIZE[2] / 2
SCALE_PLATFORM_TOP_REST_Z = SCALE_BASE_TOP_Z + SCALE_TRAVEL + SCALE_PLATFORM_SIZE[2]
SCALE_PLATFORM_LOCAL_REST = (0.0, 0.0, SCALE_PLATFORM_REST_Z - SCALE_BASE_POS[2])

# Scale center in world
SCALE_CENTER_XY = (0.300, 0.000)

# Predicted resting sags (m):
SAG_PLATFORM_EMPTY = (SCALE_PLATFORM_MASS * 9.81) / SCALE_SPRING_K          # ~0.8 mm
SAG_LIGHT_BLOCK = (BLOCK_MASSES["block_0"] * 9.81) / SCALE_SPRING_K         # ~1.6 mm
SAG_HEAVY_BLOCK = (BLOCK_MASSES["block_1"] * 9.81) / SCALE_SPRING_K         # ~4.9 mm

# Readout meter bar at (0.450, 0.000)
METER_SIZE = (0.050, 0.160, 0.015)
METER_POS = (0.450, 0.000, TOP + METER_SIZE[2] / 2)
METER_TOP_Z = TOP + METER_SIZE[2]
NEEDLE_SIZE = (0.012, 0.004, 0.008)
NEEDLE_Z = METER_TOP_Z + NEEDLE_SIZE[2] / 2
READOUT_GAIN = 10.0  # 1 mm sag -> 10 mm needle travel
# Threshold for heavy band on readout: 32 mm
READOUT_HEAVY_THRESHOLD_M = 0.032

# Green shelf on left for light blocks
SHELF_SIZE = (0.260, 0.090, 0.010)
SHELF_POS = (0.300, +0.140, TOP + SHELF_SIZE[2] / 2)
SHELF_TOP_Z = TOP + SHELF_SIZE[2]

# Targets on green shelf for light blocks
SHELF_SPOTS = {
    "block_0": (0.240, +0.140, SHELF_TOP_Z + BLOCK_SIZE[2] / 2),
    "block_2": (0.360, +0.140, SHELF_TOP_Z + BLOCK_SIZE[2] / 2),
}

# Red reject tray on right for heavy block
TRAY_SIZE = (0.130, 0.130, 0.020)
TRAY_POS = (0.300, -0.300, TOP + TRAY_SIZE[2] / 2)
TRAY_TOP_Z = TOP + TRAY_SIZE[2]
TRAY_SPOT = (0.300, -0.300, TOP + 0.005 + BLOCK_SIZE[2] / 2)

# Grasp and arm navigation heights
HOVER_Z = TOP + 0.085                       # ~0.8325 m (clears 40 mm block tops at 0.7975 m)
GRASP_Z = SUPPLY_TOP_Z + BLOCK_SIZE[2] / 2  # TCP aligned with block center
SCALE_HOVER_Z = 0.880  # fingertips must clear a 40 mm block (top 0.8275) before any sideways move
SCALE_PLACE_Z = SCALE_PLATFORM_TOP_REST_Z + BLOCK_SIZE[2] / 2

HOME_L = (0.200, +0.280, HOVER_Z)
HOME_R = (0.200, -0.280, HOVER_Z)
PAD_TO_TCP = 0.013
