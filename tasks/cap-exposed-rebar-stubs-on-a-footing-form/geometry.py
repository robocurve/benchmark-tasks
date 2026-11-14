"""Geometry of the rebar-cap scene on the yam_arena station table, shared by build_assets.py, task.py, logic.py and
demo.py. Pure Python (no Isaac Sim), so the task file and the unit tests can import it.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights are above the table top
(TOP) unless they say "world". Arm bases: left (0, +0.305), right (0, -0.305).

A footing form (a 30 mm plywood floor) stands on the table with five vertical #4 reinforcing bars (12.7 mm round
steel) sticking up 70 mm out of it in a row along y. Two stubs already wear a red plastic safety cap; three are bare.
Three more caps stand on a tray of staging pads in front of the form. A safety cap is a square-section plastic cap with
a blind bore and a flared mouth that guides it onto the bar: it is pressed down until the bar is 30 mm inside it.
"""

from __future__ import annotations

import math

TOP = 0.7475                                    # yam_arena.task_file.TABLE_TOP["station"]

# -- layout (world) -------------------------------------------------------------------------------------------------
# One fixed layout: the capped stubs are the two outer ones (y = -0.22 and +0.22) and the bare ones the middle three
# (y = -0.11, 0 and +0.11).

# -- the footing form: a plywood floor with the bars standing in it ---------------------------------------------------
FORM_X = 0.300                                  # the row of bars, world x
FORM_SIZE = (0.100, 0.600, 0.030)               # x, y, z of the plywood floor (a 30 mm board, narrow enough for open fingers at the pads)
FORM_TOP = TOP + FORM_SIZE[2]                   # the floor's top face, world z
BAR_R = 0.00635                                 # #4 bar: 12.7 mm round
BAR_H = 0.070                                   # stub height above the floor
BAR_TOP = FORM_TOP + BAR_H                      # world z of the bar tips
STUB_Y = (-0.220, -0.110, 0.0, 0.110, 0.220)    # world y of the five stubs
PRE_CAPPED = (0, 4)                             # indices of the stubs that already wear a cap
BARE = tuple(i for i in range(5) if i not in PRE_CAPPED)

# -- the safety cap (frame: origin on the bore axis at the cap's mouth, z up) -----------------------------------------
CAP_BORE = 0.014                                # square bore, 14 mm across (the bar is 12.7 mm: 0.65 mm each side)
CAP_BORE_DEPTH = 0.030                          # the blind bore's depth
CAP_STEM = 0.064                                # the stem's outer width (square)
CAP_STEM_H = 0.028                              # the stem's height (mouth to the head's underside)
CAP_HEAD = 0.070                                # the head: a 70 mm square ...
CAP_HEAD_H = 0.016                              # ... 16 mm tall
CAP_H = CAP_STEM_H + CAP_HEAD_H                 # 44 mm overall
CAP_FUNNEL_H = 0.021                            # the flared mouth: wide at the mouth, down to the bore at this height (45 degrees)
CAP_FUNNEL_HALF = 0.028                         # half-width of the mouth's opening (56 mm across)
CAP_MASS = 0.050                                # kg (a 2.5 in rebar cap weighs 30 to 60 g)
CAP_FRICTION = (0.45, 0.4)                      # moulded plastic on steel (the pads hold it by pressure)
SEATED_Z = BAR_TOP - CAP_BORE_DEPTH + 0.0002    # world z of a fully seated cap's mouth (0.2 mm of air under the roof)

# -- staging: three loose caps stand on pads in front of the form -----------------------------------------------------
PAD_X = 0.175
PAD_SIZE = (0.070, 0.070, 0.004)
CAP_START_Y = tuple(STUB_Y[i] for i in BARE)    # world y of the three loose caps: in front of the bare stubs
CAP_START_Z = TOP + PAD_SIZE[2]                 # a loose cap stands on its mouth on a pad

# -- scoring --------------------------------------------------------------------------------------------------------------
SEAT_INSIDE_MIN = 0.026                         # a cap counts as seated when the bar is at least 26 mm inside the bore
SEAT_OFFSET_MAX = 0.003                         # ... its bore axis within 3 mm of the bar's, horizontally
SEAT_TILT_MAX = math.radians(8.0)               # ... and its axis within 8 degrees of vertical
HOLD_S = 3.0
CLEAR_DIST = 0.150                              # an arm is clear when its tool point is 15 cm from every stub axis ...
CLEAR_Z = TOP + 0.250                           # ... or higher than this above the table

MOUTH_TO_PAD_TIP = 0.022                        # demo: the pads reach this far above the mouth (6 mm under the head, holding its sides)
