"""Geometry and physical specifications for the air fryer fry-basket task.

Source appliance: the 5.3 qt Air Fryer, model CM1708 (owner's manual for QVC item K46295). Its fry basket has a handle
shaped like an inverted L: a short arm out from the basket rim and a vertical grip hanging from the arm's outer end. The
Basket Release Button sits on top of the arm, where the thumb rests, under a Sliding Button Guard that slides forward to
expose it. Pressing the button lets the fry basket lift out of the outer basket; the manual says to do this only with the
fry basket and outer basket "resting on a level, heat-resistant surface".

Frames: x forward (away from the robot), y left, z up. Basket-frame positions are metres from the basket origin (the
centre of its floor), with the handle on the -x side, toward the robot.
"""

from __future__ import annotations

# Table surface in yam_arena
TABLE_TOP = 0.7475

# Colours (sRGB 0..1)
COLOR_HOUSING = (0.369, 0.384, 0.408)    # #5E6268 heat-tinted grey
COLOR_PAN = (0.784, 0.267, 0.122)        # #C8441F outer basket, orange painted steel
COLOR_BASKET = (0.824, 0.271, 0.118)     # #D2451E fry basket
COLOR_HANDLE = (0.120, 0.120, 0.140)     # #1E1E24 cool-touch black plastic handle
COLOR_BUTTON = (0.231, 0.435, 0.839)     # #3B6FD6 blue release button (lever)
COLOR_GUARD = (0.780, 0.860, 0.900)      # #C7DBE6 clear sliding button guard (shown pale blue-grey)
COLOR_MAT = (0.478, 0.180, 0.180)        # #7A2E2E dark red silicone mat
COLOR_TRIVET = (0.690, 0.545, 0.369)     # #B08B5E bamboo trivet board
COLOR_AMBER = (0.950, 0.550, 0.100)      # amber glow tiles and heating lamp

# Mat for the fry basket (fixed on the bench, left of the appliance)
MAT_CENTRE = (0.414, 0.335, TABLE_TOP)   # a 30 x 30 cm silicone mat; its -y edge (y 0.185) is 2 cm clear of the trivet (y 0.165)
MAT_SIZE = (0.300, 0.300, 0.010)
MAT_X_MIN = 0.264
MAT_X_MAX = 0.564
MAT_Y_MIN = 0.185
MAT_Y_MAX = 0.485
MAT_TOP_Z = TABLE_TOP + 0.010
MAT_Z_TOL = 0.005
TILT_MAX_DEG = 5.0

# The appliance body (fixed, hot), its drawer bay empty: the outer basket has been pulled out onto the trivet
HOUSING_POS = (0.700, 0.000, TABLE_TOP + 0.030)
HOUSING_FRICTION = 1.2
STAND_BOX = ((0.000, 0.000, -0.015), (0.260, 0.260, 0.030))   # 3 cm board under the appliance (not hot)

# 5 Housing collision boxes (all hot: true) - relative to housing origin
HOUSING_BOXES = [
    ((0.000, 0.000, 0.002), (0.260, 0.260, 0.004), True),    # floor
    ((0.000, 0.000, 0.185), (0.260, 0.260, 0.190), True),    # top & panel
    ((0.122, 0.000, 0.047), (0.016, 0.260, 0.086), True),    # back wall
    ((0.000, 0.1235, 0.047), (0.260, 0.013, 0.086), True),   # left wall
    ((0.000, -0.1235, 0.047), (0.260, 0.013, 0.086), True),  # right wall
]

# Heat-resistant trivet (fixed, not hot) the outer basket rests on, as the manual requires before the guard is moved
TRIVET_POS = (0.350, 0.035, TABLE_TOP)
TRIVET_BOXES = [((0.000, 0.000, 0.005), (0.260, 0.260, 0.010), False)]
TRIVET_TOP_Z = TABLE_TOP + 0.010

# Outer basket (loose, hot): a steel pan with a catch lip on the outside of its front wall
PAN_START_POS = (0.350, 0.035, TABLE_TOP + 0.010)   # origin = underside of its floor, resting on the trivet
PAN_MASS = 0.60
PAN_FRICTION = 1.0
PAN_WALL_TOP = 0.066                                 # wall top above the pan origin
PAN_BOXES = [
    ((0.000, 0.000, 0.003), (0.240, 0.224, 0.006), True),     # floor
    ((0.000, 0.109, 0.036), (0.240, 0.006, 0.060), True),     # left wall
    ((0.000, -0.109, 0.036), (0.240, 0.006, 0.060), True),    # right wall
    ((0.117, 0.000, 0.036), (0.006, 0.212, 0.060), True),     # back wall
    ((-0.117, 0.000, 0.036), (0.006, 0.212, 0.060), True),    # front wall (x -0.120 to -0.114)
    ((-0.1225, 0.000, 0.049), (0.005, 0.024, 0.008), True),   # catch lip under the handle: x -0.125 to -0.120, z 4.5 to 5.3 cm
]

# Fry basket (loose): the basket origin sits 11 mm above the pan origin (on the pan floor)
BASKET_START_POS = (0.350, 0.035, TABLE_TOP + 0.021)
BASKET_ABOVE_PAN_M = 0.011
BASKET_MASS = 0.25       # an empty steel basket with a plastic handle (real 0.25 to 0.45 kg)
BASKET_FRICTION = 1.5    # one value for the whole basket: stands for the handle's rubber-coated grip

# 5 Basket body collision boxes (all hot: true) - relative to basket origin
BASKET_BODY_BOXES = [
    ((-0.024, 0.000, 0.000), (0.178, 0.204, 0.010), True),   # floor; x -0.113 to 0.065
    ((-0.024, 0.100, 0.035), (0.178, 0.004, 0.060), True),   # left wall
    ((-0.024, -0.100, 0.035), (0.178, 0.004, 0.060), True),  # right wall
    ((0.063, 0.000, 0.035), (0.004, 0.204, 0.060), True),    # back wall: 49 mm play to the pan's back wall so the basket can pitch as it is lifted by its front handle
    ((-0.111, 0.000, 0.035), (0.004, 0.204, 0.060), True),   # front wall (x -0.113 to -0.109), 1 mm inside the pan's front wall
]

# 6 Handle collision boxes (cool-touch, hot: false) - relative to basket origin. Inverted L: an arm out from the rim and
# a vertical grip hanging from its outer end; the arm's inner part is a slot between two rails that holds the release lever.
BASKET_HANDLE_BOXES = [
    ((-0.111, 0.010, 0.0775), (0.004, 0.008, 0.025), False),   # left half of the mount on the basket front wall (z 6.5 to 9.0 cm; the lever passes between the halves)
    ((-0.124, 0.010, 0.080), (0.022, 0.008, 0.020), False),    # left rail of the slot (x -0.135 to -0.113, z 7.0 to 9.0 cm), 6 mm above the pan's front wall
    ((-0.124, -0.010, 0.080), (0.022, 0.008, 0.020), False),   # right rail
    ((-0.15875, 0.000, 0.080), (0.0475, 0.024, 0.020), False), # solid outer arm (x -0.1825 to -0.135)
    ((-0.170, 0.000, 0.030), (0.025, 0.024, 0.080), False),    # vertical grip, 8 cm (x -0.1825 to -0.1575, z -1.0 to 7.0 cm): the lifting hand closes across it
    ((-0.111, -0.010, 0.0775), (0.004, 0.008, 0.025), False),  # right half of the mount
]
GRIP_CENTRE = (-0.170, 0.000, 0.030)

# Basket floor extents for corner checks
BASKET_FLOOR_HALF_X = 0.113   # front half (handle side)
BASKET_FLOOR_BACK_X = 0.065   # back edge of the floor (the basket is 17.8 cm long)
BASKET_FLOOR_HALF_Y = 0.102

# Basket Release Button: a sprung lever pivoting across the slot (hinge about y). Its tooth hangs straight down from the
# pivot outside the pan's front wall, and its foot points inward under the catch lip, so the outer basket's weight on the
# foot passes through the pivot and cannot turn the lever (measured on the GPU: with the foot 2.3 cm inboard of the pivot,
# a carried outer basket pried the lever open). The thumb pad sits 3 cm inboard of the pivot, over the basket rim, where
# the thumb rests; pressing it down swings the foot out from under the lip.
LEVER_AT = (-0.125, 0.000, 0.082)       # pivot, relative to basket origin
LEVER_MASS = 0.015
LEVER_TRAVEL_RAD = 0.28                 # end stop: the pad moves down 8 mm (a thumb button's 4 to 8 mm)
LEVER_RATE_NM_RAD = 0.35                # spring back to rest: 1.3 N on the pad releases it, 2.9 N holds the stop
LEVER_DAMPING = 0.02
LEVER_RELEASE_RAD = 0.13                # the foot clears the lip once the lever turns 0.09 rad, by 1.5 mm at 0.12 (measured on the GPU: a 1 mm clearance still caught); 0.13 (pad 4 mm down) is the "pressed" threshold
LEVER_FRICTION = 0.2                    # smooth plastic hook on the steel lip
LEVER_OUT_RAD = 0.02                    # back at rest (<= 7 % of the stroke)
# Lever boxes (hot: false) - relative to the pivot, at rest
LEVER_BOXES = [
    ((0.012, 0.000, 0.000), (0.036, 0.008, 0.008), False),     # beam in the slot, through the mount and over the rim to the pad
    ((0.030, 0.000, 0.014), (0.008, 0.008, 0.020), False),     # stem up to the pad
    ((0.030, 0.000, 0.027), (0.020, 0.022, 0.006), False),     # thumb pad 2.0 x 2.2 cm (basket x -0.105 to -0.085, z 10.6 to 11.2 cm)
    ((-0.005, 0.000, -0.0295), (0.004, 0.008, 0.051), False),  # tooth hanging straight down outside the lip (basket x -0.132 to -0.128, z 2.7 to 7.8 cm)
    ((-0.00125, 0.000, -0.052), (0.0115, 0.008, 0.006), False),# foot under the catch lip: 4.5 mm overlap, 1 mm below it, centred under the pivot
]
PAD_CENTRE = (-0.095, 0.000, 0.109)     # basket frame, at rest

# Sliding Button Guard: a clear cap over the pad that slides forward (+x, toward the basket) to expose it; its 6 cm
# travel keeps its tab clear of a pressing finger (1.5 to 4 cm wide; measured on the GPU: at 4.5 cm the finger, drifting
# 9 mm forward, rested on the tab at its stop and pinned the basket down)
GUARD_AT = (0.000, 0.000, 0.000)
GUARD_MASS = 0.008
GUARD_TRAVEL_M = 0.060
GUARD_DAMPING_NS_M = 3.0                # no spring: it stays where it is pushed
GUARD_OPEN_M = 0.030                    # pad uncovered beyond 3 cm
# Guard boxes (hot: false) - relative to the basket origin at rest (the joint origin is the basket origin)
GUARD_BOXES = [
    ((-0.095, 0.000, 0.116), (0.030, 0.028, 0.004), False),    # cap plate over the pad (x -0.110 to -0.080, z 11.4 to 11.8 cm)
    ((-0.112, 0.000, 0.122), (0.004, 0.020, 0.016), False),    # ridged thumb tab at its back edge (x -0.114 to -0.110, z 11.4 to 13.0 cm): above the pad top (measured on the GPU: a tab reaching below it jammed on the pad)
]
GUARD_TAB_BACK_X = -0.114

# Mat box (touchable, hot: false) - relative to mat origin
MAT_BOXES = [
    ((0.000, 0.000, 0.005), (0.300, 0.300, 0.010), False),
]

# Contact scoring dimensions
PAD_BOX_SIZE = (0.012, 0.012, 0.012)   # 12 mm cube
PAD_BOX_HALF = (0.006, 0.006, 0.006)
HOT_INFLATION_M = 0.002                # 2 mm inflation of hot boxes
RELEASE_MIN_DIST_M = 0.030             # 3 cm clearance from basket, handle, lever and guard
STILL_MOTION_M = 0.001                 # 1 mm peak-to-peak motion
STILL_WINDOW_STEPS = 20                # 1.0 s at 20 Hz
BASKET_CLEARED_PAN_M = 0.070           # basket risen 7 cm out of the pan: its floor clears the pan walls by 1 cm
BASKET_LIFT_MIN_M = 0.020              # 20 mm lift above the pan
UNLATCHED_LIFT_M = 0.012               # basket 12 mm up out of the pan without the lever ever pressed: latch defeated
PAN_HOME_XY_M = 0.015                  # outer basket still on its trivet: within 1.5 cm of its start
PAN_HOME_Z_M = 0.005
PAN_LIFTED_M = 0.010                   # outer basket 1 cm or more off the trivet

# TCP and ready poses
TCP_BELOW_LINK_M = 0.134  # yam_arena TCP_OFFSET: the tool point sits 0.134 m below the gripper link (measured on the GPU)
R_READY = (0.100, -0.220, TABLE_TOP + 0.120)
L_READY = (0.100, 0.140, TABLE_TOP + 0.120)

# Display markers (VisualizationMarkers): heat cues on top of the appliance
VENT_GLOW_TILES_LOCAL = [
    (0.040, 0.040, 0.281),
    (0.040, -0.040, 0.281),
    (-0.040, 0.040, 0.281),
    (-0.040, -0.040, 0.281),
]
LAMP_TILE_LOCAL = (-0.080, 0.000, 0.281)
