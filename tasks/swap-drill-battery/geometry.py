"""Geometry of the drill-battery scene, shared by build_assets.py, task.py and demo.py. Pure Python.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres.

An 18 V cordless drill-driver lies on its side on the table, its handle toward the robot and its chuck pointing to
the robot's right. Its flat battery is on the foot of the handle. Its slide pack locks with a spring catch, released
by a button on the pack's top face, ahead of the tool's foot. A charged pack of the same kind lies on its side
26 cm to the right, in line with the drill's rails.

Every asset is built in a frame with the world's axes (no rotation at spawn). In the battery's frame the origin is
the pack's centre, +y its back (the end that goes onto the tool first), -y its front (button end), +x its top face
(which slides against the tool's foot), z across its width (vertical as it lies). The drill's frame has its origin
where a seated pack's centre is.
"""

from __future__ import annotations

TOP = 0.7475

# -- the pack: 115 x 50 x 76 mm, 0.33 kg (a slim 1.5 Ah 18 V slide pack) ------------------------------------------------
# (measured: a 69 mm pinch, button to base on a 62 mm pack, stalled the open fingers on the pack's top; the YAM
# fingers take 4-6.5 cm at full depth)
L, H, W = 0.115, 0.050, 0.076          # along y (length), x (height, top face +x), z (width)
PACK_MASS = 0.31
LATCH_MASS = 0.02
WALL_T = 0.012                         # the rail walls along the top face's two long edges (z bands)
WALL_H = 0.014                         # they stand this far above the top face
LIP_T = 0.004                          # each wall's inward lip, at its top
LIP_IN = 0.007                         # how far it reaches in

# latch: a slider moving along x; its tooth stands up out of the top face, its button sits ahead of the walls
TOOTH = (0.005, 0.004, 0.016)         # square-faced block: height above the top face (x), length (y), width (z)
BUTTON = (0.006, 0.020, 0.026)         # stands 6 mm proud of the top face
BUTTON_Y = -0.045
BUTTON_Z = 0.022                       # its centre across the width (pack frame): the upper half, where the pads reach
TRAVEL = 0.0065                        # button pressed flush: the tooth is below the top face
LATCH_K = 500.0                        # N/m (sim): ~2.8 N to press it flush, about a real pack's button

# -- the drill's foot: a T-rail the pack's walls and lips slide along, with a notch the tooth drops into --------------
CLR = 0.002                            # play all round (a pack lined up open loop to 2 mm still finds the rail)
FOOT_LEN = 0.060                       # along y, from the back end (L/2) forward (measured: with 7.5 cm the open
                                       # fingers, wider along y than their pads, landed on the foot beside the button)
FOOT_Y0 = L / 2 - FOOT_LEN             # front end of the rail (pack frame, seated)
FLANGE_X = (H / 2 + CLR, H / 2 + WALL_H - LIP_T - CLR)       # the T's flange: under the lips
FLANGE_Z = W / 2 - WALL_T - CLR                              # half width
STEM_X = (FLANGE_X[1], H / 2 + WALL_H + CLR)
STEM_Z = W / 2 - WALL_T - LIP_IN - CLR
FRONT_PIECE = 0.012                    # flange ahead of the notch
NOTCH = 0.012
WALL_Y0 = FOOT_Y0 - 0.005              # the pack's walls run from here to its back end; ahead of it the top is flat
TOOTH_Y = FOOT_Y0 + FRONT_PIECE + 0.001 + 0.002   # the tooth block's centre (pack frame): 1 mm behind the notch's
                                       # front edge; its 45-degree ramp runs on behind it (+y), inside the notch
FOOT_BLOCK_X = (STEM_X[1], H / 2 + 0.035)
STOP_T = 0.011                         # behind the back end: what a fully home pack butts against

# -- the drill body (visual and mass; it rests on its foot and its motor housing) ------------------------------------
DRILL_MASS = 1.3                       # a bare 18 V drill-driver
HANDLE_X = (FOOT_BLOCK_X[1], H / 2 + 0.145)
HANDLE_Y = (0.005, 0.055)
HANDLE_T = 0.035
MOTOR_X = (HANDLE_X[1], H / 2 + 0.215)
MOTOR_Y = (-0.130, 0.065)
MOTOR_T = 0.068
CHUCK_R, CHUCK_LEN = 0.021, 0.070

# -- layout -----------------------------------------------------------------------------------------------------------
SEAT = (0.26, 0.00)                    # world x, y of a seated pack's centre (and the drill's origin)
SPARE_DY = -0.26                       # the charged pack starts this far along the rail axis, ahead of the drill
Z_C = TOP + W / 2                      # world z of a pack's centre lying on its side

# -- scoring ----------------------------------------------------------------------------------------------------------
OFF = FOOT_LEN + 0.005                 # a pack this far or more forward of home is clear of the rail
ENGAGED = 0.030                        # on the rail by at least 3 cm (pack within FOOT_LEN - 3 cm of home)
HOME_TOL = 0.002                       # home: within 2 mm of the stop
ALIGN_TOL = 0.005                      # on the rail line: within 5 mm across (x and z, drill frame)
LATCHED = 0.0015                       # the catch up: pressed less than 1.5 mm
CLEAR = 0.010                          # released: no fingertip within 1 cm of the pack
STILL_S = 1.0
