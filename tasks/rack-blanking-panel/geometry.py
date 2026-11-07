"""Dimensions and layout of the rack task: a 19-inch two-post open-frame rack (8U) clamped to the table, its front
facing the arms, and a 1U tool-less blanking panel standing in a slotted stand in front of it.

Station frame: metres from the midpoint between the arm bases, x forward, y to the robot's left, z up; heights here
are above the table top (TOP), and task.py adds TOP.
"""

from __future__ import annotations

TOP = 0.7475                      # the station's table top

# -- the rack (EIA-310: 1U = 44.45 mm, three holes per U at 6.35, 22.225 and 38.1 mm from the unit's bottom) ----------
U = 0.04445
HOLE_OFF = (0.00635, 0.022225, 0.0381)
NU = 8                            # units, U1 at the bottom
RX = 0.42                         # the posts' front face
PY = 0.2325                       # hole centres, either side (465 mm apart)
RZ0 = 0.030                       # U1's bottom edge above the table (the rack stands on 2 cm feet)
HOLE = 0.0095                     # square holes
POST_IN, POST_OUT = 0.2225, 0.2565   # each post's front face, |y|
POST_D = 0.016                    # how deep the holes go
POST_BACK = 0.030                 # solid post behind the holes
BAR_H = 0.025                     # the top bar joining the posts
TARGET_U = 6
INSTALLED = (("server", 1, 2), ("switch", 3, 1), ("patch", 8, 1))   # (kind, lowest unit, units); U4 to U7 are empty


def hole_z(h: int) -> float:
    """Height of hole h (1 to 3 NU, counted from the bottom)."""
    return RZ0 + (h - 1) // 3 * U + HOLE_OFF[(h - 1) % 3]


def middle_hole(u: int) -> int:
    return 3 * (u - 1) + 2


TARGET_HOLE = middle_hole(TARGET_U)   # 17
TOP_Z = RZ0 + NU * U              # top of U8

# -- the blanking panel (origin: the middle of its back face, which seats against the posts) --------------------------
PANEL_W = 0.4826                  # 19 inches
PANEL_H = 0.0430                  # 1U less 1.45 mm, so a panel resting on its clips clears the next units
PANEL_T = 0.006
PANEL_MASS = 0.12
PEG = 0.008                       # the clip on each end: 8 mm square (chamfered to 5 mm), 12 mm out of the back
PEG_L = 0.012
SAG = (HOLE - PEG) / 2            # a hanging panel rests 0.75 mm below the hole's centre

# -- the stand: two slotted wooden uprights holding the panel upright, face to the arms -------------------------------
STAND_X = 0.300                   # the panel's back face, standing in the stand
STAND_Y = 0.170                   # the uprights' centres, either side
STAND_W = 0.040                   # along y
STAND_D = 0.032                   # along x
STAND_H = 0.160
SLOT = 0.008                      # slot width (x) and depth below the top: 26.5 mm
SLOT_BOTTOM = 0.1335
PANEL_START_Z = SLOT_BOTTOM + PANEL_H / 2   # the panel's middle, standing in the stand

# -- scoring ------------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.002                  # an end is pushed home when its back is no more than 2 mm in front of its post ...
BEHIND_TOL = 0.001                # ... and no more than 1 mm behind its front face
HOLE_TOL = 0.003                  # a clip is level with a hole when within 3 mm of its centre's height
CLIP_TOL = (HOLE - PEG) / 2 + 0.001   # and in it when within 1.75 mm of its centre, across and up (it sags 0.75 mm)
OUT_TOL = 0.015                   # the 0.5 for one end home: the other end's back no more than 15 mm out
CLEAR = 0.01                      # let go: every part of both arms at least 1 cm from the panel
