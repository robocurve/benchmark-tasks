"""Geometry and constants for sliding-tile-sort-three-tiles."""

from __future__ import annotations

import os
import numpy as np

# Table surface height
TABLE_Z = 0.7475

# Frame geometry: 2x2 cells, pitch 76 mm, rim 4 mm tall, 6 mm thick
CELL_SIZE = 0.076
TILE_SIZE = 0.064
TILE_HEIGHT = 0.015
RIM_HEIGHT = 0.004
RIM_THICK = 0.006

FRAME_POS = (0.280, 0.000, TABLE_Z + RIM_HEIGHT / 2.0)
FRAME_INNER = 2 * CELL_SIZE  # 0.152 m
FRAME_OUTER = FRAME_INNER + 2 * RIM_THICK  # 0.164 m

# 4 cell centers in world frame
# Cell 0: (+x, +y), Cell 1: (+x, -y), Cell 2: (-x, +y), Cell 3: (-x, -y)
CELL_0 = (0.280 + CELL_SIZE / 2.0, +CELL_SIZE / 2.0)  # (0.318, +0.038)
CELL_1 = (0.280 + CELL_SIZE / 2.0, -CELL_SIZE / 2.0)  # (0.318, -0.038)
CELL_2 = (0.280 - CELL_SIZE / 2.0, +CELL_SIZE / 2.0)  # (0.242, +0.038)
CELL_3 = (0.280 - CELL_SIZE / 2.0, -CELL_SIZE / 2.0)  # (0.242, -0.038)

CELLS = (CELL_0, CELL_1, CELL_2, CELL_3)

# Initial placement: Yellow in 0, Empty in 1, Orange in 2, Blue in 3
TILE_REST_Z = TABLE_Z + TILE_HEIGHT / 2.0  # 0.7550 m
TILE_YELLOW_START = (CELL_0[0], CELL_0[1], TILE_REST_Z)
TILE_ORANGE_START = (CELL_2[0], CELL_2[1], TILE_REST_Z)
TILE_BLUE_START = (CELL_3[0], CELL_3[1], TILE_REST_Z)

# Goal mapping: Cell 0 -> Blue, Cell 1 -> Orange, Cell 2 -> Yellow, Cell 3 -> Empty
GOAL_CELL_BLUE = 0
GOAL_CELL_ORANGE = 1
GOAL_CELL_YELLOW = 2
GOAL_CELL_EMPTY = 3

# Tile physics
TILE_MASS = 0.04
TILE_FRICTION = 0.5
TILE_DIMS = (TILE_SIZE, TILE_SIZE, TILE_HEIGHT)

# Goal card: visual plate beside frame
CARD_POS = (0.280, -0.160, TABLE_Z + 0.001)
CARD_SIZE = (0.080, 0.080, 0.002)

# Tolerances and zero-gate limits
CELL_MATCH_TOL_XY = 0.025
TILE_MAX_Z = TABLE_Z + TILE_HEIGHT + 0.012  # 0.7745 m (lifted > 12 mm = escaped)
FRAME_MAX_XY = (0.280, 0.000, 0.090)        # within 90 mm of frame center
HOLD_TIME_S = 2.0

# Demo control parameters
DEMO_TIME_SCALE = float(os.environ.get("DEMO_TIME_SCALE", "1.8"))
DEMO_HOVER_Z = float(os.environ.get("DEMO_HOVER_Z", "0.850"))
DEMO_PUSH_Z = TABLE_Z + 0.0090              # 0.7565 m (mid-height, clears 4 mm rim)
DEMO_PUSH_OVERSHOOT = float(os.environ.get("DEMO_PUSH_OVERSHOOT", "0.020"))
DEMO_PUSH_HOLD_S = float(os.environ.get("DEMO_PUSH_HOLD_S", "1.0"))
DEMO_FINAL_HOLD_S = float(os.environ.get("DEMO_FINAL_HOLD_S", "2.6"))

# Finger values
FINGER_OPEN = -0.04695
FINGER_CLOSED = -0.0050
FINGER_GRASP = -0.0300

# Colors (RGB)
COLOR_TILE_BLUE = (0.20, 0.40, 0.85)
COLOR_TILE_ORANGE = (0.90, 0.50, 0.10)
COLOR_TILE_YELLOW = (0.90, 0.80, 0.15)
COLOR_FRAME = (0.45, 0.45, 0.45)
COLOR_CARD = (0.92, 0.92, 0.92)

