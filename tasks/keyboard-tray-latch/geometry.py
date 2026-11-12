from __future__ import annotations

"""Geometric constants for keyboard_tray_latch (T22)."""

TABLE_TOP = 0.7475
SUCCESS_HOLD_TIME_S = 1.0

COLOR_DESK = (0.85, 0.80, 0.70)    # Beige desk (#D9CCB3)
COLOR_TRAY = (0.50, 0.52, 0.55)    # Grey tray (#80858C)
COLOR_LATCH = (0.60, 0.25, 0.75)   # Purple latch (#9940BF)

# Fixed beige desk (25 kg, friction 1.2) at x=0.30, y=0.0
DESK_NAME = "computer_desk"
DESK_MASS_KG = 25.0
DESK_FRICTION = 1.2
DESK_POS_WORLD = (0.30, 0.0, TABLE_TOP)

# Desk body 200 x 360 x 60 mm with under-desk guide rails
DESK_BOXES = [
    # Desk top: 200 x 360 x 20 mm
    ((0.0, 0.0, 0.050), (0.200, 0.360, 0.020), COLOR_DESK),
    # Left rail: 200 x 20 x 40 mm
    ((0.0, 0.170, 0.020), (0.200, 0.020, 0.040), COLOR_DESK),
    # Right rail: 200 x 20 x 40 mm
    ((0.0, -0.170, 0.020), (0.200, 0.020, 0.040), COLOR_DESK),
]

# Keyboard tray: 200 x 300 x 20 mm on +X slide (travel 280 mm)
TRAY_DIMS_M = (0.200, 0.300, 0.020)

# Purple spring latch slider at catch (travel 8 mm along Z)
LATCH_DIMS_M = (0.020, 0.020, 0.020)

GATE_DESK_DRIFT_MAX_M = 0.005
