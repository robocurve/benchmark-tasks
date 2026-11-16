"""Geometry of the rebar-cap scene on the yam_arena station table, shared by build_assets.py, task.py, logic.py and
demo.py. Pure Python (no Isaac Sim), so the task file and the unit tests can import it.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights are above the table top
(TOP) unless they say "world". Arm bases: left (0, +0.305), right (0, -0.305).

A footing form (a 30 mm plywood floor) stands on the table with five vertical #4 reinforcing bars (12.7 mm round
steel) sticking up 70 mm out of it in a row along y. Two stubs already wear a red plastic safety cap; three are bare.
Three more caps stand on a tray of staging pads in front of the form. A safety cap is a steel-plate-topped cap with a raised grip boss
with a blind bore, a flared mouth that guides it onto the bar and two spring-loaded gripping fins that hold it on: it is
pressed down until the bar is at least 32 mm up the bore.
"""

from __future__ import annotations

import math

TOP = 0.7475                                    # yam_arena.task_file.TABLE_TOP["station"]

# -- layout (world) -------------------------------------------------------------------------------------------------
# One fixed layout: the capped stubs are the two outer ones (y = -0.24 and +0.24) and the bare ones the middle three
# (y = -0.12, 0 and +0.12).

# -- the footing form: a plywood floor with the bars standing in it ---------------------------------------------------
FORM_X = 0.300                                  # the row of bars, world x
FORM_SIZE = (0.100, 0.600, 0.020)               # x, y, z of the plywood floor (a 20 mm board, narrow enough for open fingers at the pads)
FORM_TOP = TOP + FORM_SIZE[2]                   # the floor's top face, world z
BAR_R = 0.00635                                 # #4 bar: 12.7 mm round
BAR_H = 0.060                                   # stub height above the floor
BAR_TOP = FORM_TOP + BAR_H                      # world z of the bar tips
STUB_Y = (-0.240, -0.120, 0.0, 0.120, 0.240)    # world y of the five stubs (120 mm apart: the plates are 101.6 mm wide)
PRE_CAPPED = (0, 4)                             # indices of the stubs that already wear a cap
BARE = tuple(i for i in range(5) if i not in PRE_CAPPED)

# -- the safety cap (frame: origin on the bore axis at the cap's mouth, z up) -----------------------------------------
# An impalement-rated cap in the style of a steel-plate-topped cap: a square steel plate on a polymer body whose bore is
# lined with two spring-loaded gripping fins that grip the bar once it is pushed in (see the README for the real part).
CAP_PLATE = 0.1016                              # the steel top plate: 4.0 in square (8 CCR 344.90(d)(1) asks for at least 4 x 4 in)
CAP_PLATE_T = 0.0015                            # ... 1.5 mm thick
CAP_BODY = 0.064                                # the polymer body below it: a 64 mm square block
CAP_BODY_H = 0.040                              # ... 40 mm tall (mouth to the plate's underside)
CAP_BOSS = 0.060                                # a raised polymer grip boss on the plate: a 60 mm square ...
CAP_BOSS_H = 0.024                              # ... 24 mm tall (the gripper pinches it; the plate is wider than the open fingers)
CAP_H = CAP_BODY_H + CAP_PLATE_T + CAP_BOSS_H   # 65.5 mm overall
CAP_BORE_Y = 0.0150                             # the bore's width across y: 15 mm (the bar is 12.7 mm: 1.15 mm of play a side)
CAP_BORE_DEPTH = 0.036                          # the blind bore's depth (to its roof)
CAP_FUNNEL_H = 0.021                            # the flared mouth: wide at the mouth, down to the bore at this height (45 degrees)
CAP_FUNNEL_HALF = 0.028                         # half-width of the mouth's opening (56 mm across)
CAP_MASS = 0.20                                 # kg: the Dayton PC110's 0.37 lb (0.168 kg, its data sheet) plus a ~0.03 kg printed grip adapter
CAP_FRICTION = (0.45, 0.4)                      # polymer on steel (the jaws' friction on the bar averages with the bar's 0.8)
# the two gripping fins (jaws): each slides along x on a spring, pressed in toward the bar
JAW_Z = (0.021, 0.033)                          # height of a fin's flat face above the mouth (the bar passes through it)
JAW_T = 0.006                                   # a fin's thickness along x
JAW_REST_GAP = 0.0107                           # the gap between the fins' inner faces at rest: 10.7 mm, 2 mm under the bar
JAW_TRAVEL = 0.0095                             # how far a fin can be pushed outward (9.5 mm)
JAW_K = 4000.0                                  # N/m: each fin's spring (4 N on the bar at the 1 mm the bar pushes it)
JAW_DAMP = 20.0                                 # N s/m
JAW_MASS = 0.005                                # kg each
JAW_ENGAGED = 0.0006                            # each fin has been pushed out at least this far (0.6 mm) by the bar
SEATED_Z = BAR_TOP - CAP_BORE_DEPTH + 0.0002    # world z of a fully seated cap's mouth (0.2 mm of air under the roof)

# -- staging: three loose caps stand on pads in front of the form -----------------------------------------------------
PAD_X = 0.175
PAD_SIZE = (0.070, 0.070, 0.004)
CAP_START_Y = tuple(STUB_Y[i] for i in BARE)    # world y of the three loose caps: in front of the bare stubs
CAP_START_Z = TOP + PAD_SIZE[2]                 # a loose cap stands on its mouth on a pad

# -- scoring --------------------------------------------------------------------------------------------------------------
SEAT_INSIDE_MIN = 0.032                         # a cap counts as seated when the bar is at least 32 mm up the 36 mm bore
SEAT_OFFSET_MAX = 0.003                         # ... its bore axis within 3 mm of the bar's, horizontally
SEAT_TILT_MAX = math.radians(8.0)               # ... and its axis within 8 degrees of vertical
HOLD_S = 3.0
CLEAR_DIST = 0.150                              # an arm is clear when its tool point is 15 cm from every stub axis ...
CLEAR_Z = TOP + 0.250                           # ... or higher than this above the table

MOUTH_TO_PAD_TIP = CAP_BODY_H + CAP_PLATE_T + 0.007    # demo: the pad tips reach this far above the mouth (7 mm over the plate, holding the boss)
