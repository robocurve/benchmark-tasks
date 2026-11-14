"""Geometry and physical specifications for air_fryer_basket_release_button_v2."""

from __future__ import annotations

import math
import numpy as np

# Table surface in yam_arena
TABLE_TOP = 0.7475

# Colours (sRGB 0..1)
COLOR_HOUSING = (0.369, 0.384, 0.408)    # #5E6268 heat-tinted grey
COLOR_DRAWER = (0.784, 0.267, 0.122)     # #C8441F orange painted steel
COLOR_BASKET = (0.824, 0.271, 0.118)     # #D2451E orange fry basket
COLOR_HANDLE = (0.120, 0.120, 0.140)     # #1E1E24 cool-touch black rubber
COLOR_BUTTON = (0.231, 0.435, 0.839)     # #3B6FD6 blue ABS release button
COLOR_MAT = (0.478, 0.180, 0.180)        # #7A2E2E dark red silicone mat
COLOR_AMBER = (0.950, 0.550, 0.100)      # amber glow tiles and heating lamp

# Mat definition (fixed on table beside drawer)
MAT_CENTRE = (0.414, 0.300, TABLE_TOP)   # a 30 x 30 cm silicone trivet mat on the left of the appliance; its -y edge (y 0.15) clears the housing board (y 0.11)
MAT_SIZE = (0.300, 0.300, 0.010)
MAT_X_MIN = 0.264
MAT_X_MAX = 0.564
MAT_Y_MIN = 0.150
MAT_Y_MAX = 0.450
MAT_TOP_Z = TABLE_TOP + 0.010
MAT_Z_TOL = 0.005
TILT_MAX_DEG = 5.0

# Housing definition (fixed, mass=None)
HOUSING_POS = (0.650, -0.020, TABLE_TOP + 0.030)   # on a 3 cm stand so the handle and release button are within the arm's low reach
HOUSING_MASS = 25.0
HOUSING_FRICTION = 1.2

# Heat-resistant board the appliance stands on (part of the fixed housing, not hot): fills the 3 cm under the housing
STAND_BOX = ((0.000, 0.000, -0.015), (0.260, 0.260, 0.030))
COLOR_STAND = (0.690, 0.545, 0.369)     # #B08B5E bamboo board

# 5 Housing collision boxes (all hot: true) - relative to housing origin
HOUSING_BOXES = [
    ((0.000, 0.000, 0.002), (0.260, 0.260, 0.004), True),    # floor
    ((0.000, 0.000, 0.185), (0.260, 0.260, 0.190), True),    # top & panel
    ((0.122, 0.000, 0.047), (0.016, 0.260, 0.086), True),    # back wall
    ((0.000, 0.1235, 0.047), (0.260, 0.013, 0.086), True),   # left wall
    ((0.000, -0.1235, 0.047), (0.260, 0.013, 0.086), True),  # right wall
]

# Drawer slide definition (joint on housing)
DRAWER_AT = (-0.235, 0.000, 0.006)   # relative to housing origin (pulled out at start; 15 mm of the drawer's rear wall stays inside the housing)
DRAWER_MASS = 0.8
DRAWER_TRAVEL_M = 0.225
DRAWER_RATE_N_M = 0.0     # not sprung: the drawer stays where it is left (damping only)
DRAWER_DAMPING_NS_M = 30.0

# 9 Drawer collision boxes (all hot: true) - relative to drawer origin
DRAWER_BOXES = [
    ((0.000, 0.000, 0.003), (0.240, 0.224, 0.006), True),     # floor
    ((0.000, 0.109, 0.041), (0.240, 0.006, 0.070), True),     # left wall
    ((0.000, -0.109, 0.041), (0.240, 0.006, 0.070), True),    # right wall
    ((0.117, 0.000, 0.041), (0.006, 0.224, 0.070), True),     # back wall
    ((-0.117, 0.000, 0.019), (0.006, 0.224, 0.026), True),   # front bottom strip
    ((-0.117, 0.106, 0.0565), (0.006, 0.012, 0.041), True),   # front left remnant
    ((-0.117, -0.106, 0.0565), (0.006, 0.012, 0.041), True),  # front right remnant
    ((-0.117, 0.095, 0.058), (0.006, 0.010, 0.008), True),    # catch ledge left
    ((-0.117, -0.095, 0.058), (0.006, 0.010, 0.008), True),   # catch ledge right
]

# Basket definition (loose Mech)
BASKET_START_POS = (0.414, -0.020, TABLE_TOP + 0.047)
BASKET_MASS = 0.30       # an empty steel basket with a rubber-coated handle (real 0.25 to 0.45 kg)
BASKET_FRICTION = 1.2    # one value for the whole basket: stands for the rubber-coated handle (rubber on a gripper finger is about 1.0 to 1.5)

# 5 Basket body collision boxes (all hot: true) - relative to basket origin
BASKET_BODY_BOXES = [
    ((-0.024, 0.000, 0.000), (0.178, 0.204, 0.010), True),   # floor (with 5 mm feet stand-in); x -0.113 to 0.065
    ((-0.024, 0.100, 0.035), (0.178, 0.004, 0.060), True),   # left wall
    ((-0.024, -0.100, 0.035), (0.178, 0.004, 0.060), True),  # right wall
    ((0.063, 0.000, 0.035), (0.004, 0.204, 0.060), True),    # back wall: 49 mm play to the drawer's back wall so the basket can pitch as it is lifted by its front handle (measured on the GPU: 17 mm jammed it at 11 deg)
    ((-0.111, 0.000, 0.035), (0.004, 0.155, 0.060), True),   # front wall with a 20 mm slot at each side for the catch lugs
]

# 3 Handle collision boxes (touchable, hot: false) - relative to basket origin
BASKET_HANDLE_BOXES = [
    ((-0.143, 0.065, 0.0355), (0.060, 0.014, 0.025), False),   # left post, 6 cm long (gripped across y by the lifting hand; room for a finger body each side)
    ((-0.143, -0.065, 0.0355), (0.060, 0.014, 0.025), False),  # right post
    ((-0.1855, 0.01875, 0.0355), (0.025, 0.1375, 0.025), False),  # cool-touch bar, main part (y -0.050 to 0.0875)
    ((-0.1855, -0.08375, 0.0355), (0.025, 0.0075, 0.025), False), # bar end cap (y -0.0875 to -0.080); the 30 mm slot between them holds the release button
]

# Basket floor half extents for corner checks
BASKET_FLOOR_HALF_X = 0.113   # front half (handle side)
BASKET_FLOOR_BACK_X = 0.065   # back edge of the floor (the basket is 17.8 cm long)
BASKET_FLOOR_HALF_Y = 0.102

# Release button definition (slide joint on basket)
BUTTON_AT = (-0.2015, -0.065, 0.028)    # relative to basket origin
BUTTON_MASS = 0.04
BUTTON_TRAVEL_M = 0.014
BUTTON_RATE_N_M = 8.0
BUTTON_DAMPING_NS_M = 5.0
BUTTON_CLEARANCE_DISPLACEMENT_M = 0.008 # the lugs clear the ledges' rear edge after 7 mm of travel; 8 mm is the scoring threshold
BYPASS_BAND_BUTTON_M = 0.004            # while the basket passes the ledges (lift 8 to 14 mm) the button must be at least this far in (a deliberate press, 30 % of the stroke; measured on the GPU: a held press reads 5.6 to 14 mm during the lift)
BUTTON_OUT_THRESHOLD_M = 0.0014         # >= 90% extended (<= 1.4 mm displaced of the 14 mm stroke)

# 3 Button collision boxes (touchable, hot: false) - relative to button joint origin
BUTTON_BOXES = [
    ((0.000, 0.000, 0.005), (0.005, 0.025, 0.025), False),    # the push button: 2.5 x 2.5 cm plate, front face 6 mm in front of the bar front face
    ((0.0055, 0.000, 0.005), (0.006, 0.012, 0.012), False),   # its stem, in the slot of the bar (6 mm clear of the bar's rear face at a full 14 mm stroke)
    ((0.0875, 0.155, 0.004), (0.012, 0.010, 0.010), False),   # catch lug left (y +0.09 on the basket): under its ledge at rest, 7 mm past its rear edge at full travel
    ((0.0875, -0.025, 0.004), (0.012, 0.010, 0.010), False),  # catch lug right (y -0.09 on the basket)
]

# Mat box (touchable, hot: false) - relative to mat origin
MAT_BOXES = [
    ((0.000, 0.000, 0.005), (0.300, 0.300, 0.010), False),
]

# Contact scoring dimensions
PAD_BOX_SIZE = (0.012, 0.012, 0.012)   # 12 mm cube
PAD_BOX_HALF = (0.006, 0.006, 0.006)
HOT_INFLATION_M = 0.002                # 2 mm inflation of hot boxes
RELEASE_MIN_DIST_M = 0.030             # 3 cm clearance from basket/handle/button
STILL_MOTION_M = 0.001                 # 1 mm peak-to-peak motion
STILL_WINDOW_STEPS = 20                # 1.0 s at 20 Hz
BASKET_CLEARED_DRAWER_Z = TABLE_TOP + 0.120  # 12 cm above table (basket origin; the floor underside is 11.5 cm up)
BASKET_LIFT_MIN_M = 0.020              # 20 mm lift above start

# TCP and ready poses
TCP_BELOW_LINK_M = 0.134  # yam_arena TCP_OFFSET: the tool point sits 0.134 m below the gripper link (measured on the GPU)
R_READY = (0.100, -0.220, TABLE_TOP + 0.120)
L_READY = (0.100, 0.140, TABLE_TOP + 0.120)

# Display markers (VisualizationMarkers, _Tiles pattern)
VENT_GLOW_TILES_LOCAL = [
    (0.040, 0.040, 0.281),
    (0.040, -0.040, 0.281),
    (-0.040, 0.040, 0.281),
    (-0.040, -0.040, 0.281),
]
LAMP_TILE_LOCAL = (-0.080, 0.000, 0.281)
