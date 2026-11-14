"""Dimensions and layout of the tourniquet task: a training thigh lying across the table with a windlass tourniquet
(C-A-T style) already strapped round it, and a pressure gauge on a bladder under the band.

Station frame: metres from the midpoint between the arm bases, x forward, y to the robot's left, z up; heights here
are above the table top (TOP), and task.py adds TOP. The windlass rod's angle THETA is the direction of its "+x" end,
measured about the vertical from +x (away from the arms) toward +y (the hip). The band runs round the thigh in the
x-z plane, so on top of the thigh it runs along x: THETA 0 or 180 is the rod lying along the band.
"""

from __future__ import annotations

import math

TOP = 0.7475                      # the station's table top

# -- the training thigh: a firm foam cylinder lying along y, hip end on the robot's left (+y) -----------------------------
LEG_X = 0.280
LEG_R = 0.0635                    # 12.7 cm across
LEG_L = 0.40
LEG_Z = LEG_R + 0.002             # its axis: resting on two chocks, 2 mm off the table
CHOCK_Y = (0.15, -0.15)
CHOCK = (0.03, 0.04, 0.035)       # wooden chocks: x, y, height
CHOCK_IN = math.sqrt(LEG_R ** 2 - (LEG_Z - CHOCK[2]) ** 2)   # their inner faces this far from the axis (5.6 cm)

# -- the tourniquet: band and bladder round the thigh, the windlass on top ---------------------------------------------
BAND_Y = 0.0
BAND_W = 0.038                    # the C-A-T's band, 1.5 in
BAND_T = 0.006                    # strap and the bladder under it
BAND_R = LEG_R + BAND_T           # the band's outer radius
BLADDER_W = 0.050                 # the pressure bladder under the band: 5 cm wide, 42 cm long, wrapped once round the
BLADDER_L = 0.42                  # thigh (12.7 cm across, so 39.9 cm round) with 2 cm of overlap
BLADDER_T = 0.0025
BAND_TOP = LEG_Z + BAND_R
HUB = (0.030, 0.022, 0.0075)      # the twisted strap under the rod's middle (visual)
ROD_R = 0.006
ROD_L = 0.150
ROD_Z = BAND_TOP + HUB[2] + ROD_R  # the rod's axis, free: it turns about the vertical and tips about its middle
ROD_MASS = 0.03

# -- the clip: the tourniquet's own windlass clip, on the band beside the windlass, toward the arms ---------------------
# It has a C-shaped hook on each edge of the band, so the rod can be locked after turning either way. A hook is a short
# channel 0.5 mm wider than the rod each side, between two walls whose lips reach 1 mm into it near the top: the opening
# (11 mm) is narrower than the rod (12 mm), so the rod is pushed down past the lips, which then close over it above its
# centre line. Above the lips a lead-in slopes into the channel. The hooks sit on the band partway down the thigh's
# curve, so a seated rod lies along the band, a little to one side, its end tipped down into the hook.
HOOK_R = 0.050                    # from the rod's middle to a hook's middle, along the seated rod
SEAT_TILT = math.radians(21)      # a seated rod tips down this far
PHI = math.radians(24)            # and lies this far to the side of the band's line, seen from above
HOOK_L = 0.010                    # each hook's length along the rod
CLIP_GAP = 0.0005                 # clear either side of the rod in the channel (13 mm wide)
PRONG_T = 0.0015                  # each wall's thickness
LIP_IN = 0.001                    # each lip reaches this far into the channel: an 11 mm opening for the 12 mm rod
LIP_Z = (0.007, 0.008)            # the lips' underside and top, above the hook's floor: over the seated rod's centre line
WALL_H = 0.009                    # the walls' height above the floor (the lead-in is the top 1 mm)
FLOOR_T = 0.003
HOOK_XY = HOOK_R * math.cos(SEAT_TILT) * math.cos(PHI), HOOK_R * math.cos(SEAT_TILT) * math.sin(PHI)
#                                 # a hook's middle, from the windlass seen from above: 4.3 cm toward the arms, 1.9 cm
#                                 # either side of the band's line (the band's edges)
TILT_MAX = math.radians(30)       # the strap lets either end of the rod be pushed down this far
THETA0 = math.pi - PHI - math.radians(30)   # the rod starts 30 degrees short of where its end lines up with the clip's
#                                             hook on the hip side (+y), one end toward the arms and the hip

# -- mechanics: the band unwinds the rod unless something holds it; twisting either way tightens -----------------------------
K_TWIST = 0.06                    # N m per rad of twist
TWIST_DAMPING = 0.03              # N m s / rad
TILT_K = 0.3                      # N m / rad, the strap holding the rod level
TILT_D = 0.02
P_SNUG = 60.0                     # mmHg under the band as strapped
P_PER_DEG = 1.0                   # mmHg per degree of twist
GREEN = 250.0                     # the gauge's green arc starts here
GAUGE_MAX = 300.0
LOCK_TILT = SEAT_TILT - 0.0005 / (HOOK_R + HOOK_L / 2)   # the rod counts as locked when it lines up with a hook and
#                                                         # tips down to within 0.5 mm of the floor at its outer end
UNLOCK_TILT = SEAT_TILT - 0.002 / HOOK_R   # and comes out when lifted until its centre line is 2 mm up, above the lips
LOCK_ALIGN = math.radians(1.5)    # seen from above, the rod lies within this of a hook's line (the channel
#                                 # itself lets it lie within 0.6 degrees)


def pressure(twist_deg: float) -> float:
    return P_SNUG + P_PER_DEG * abs(twist_deg)


def hook_angles() -> tuple[float, float]:
    """The directions (seen from above) of a seated rod's end in each hook: hip side (+y) and knee side (-y)."""
    return math.pi - PHI, math.pi + PHI


# -- the gauge: an aneroid dial on a stand beyond the thigh, tilted up toward the arms ----------------------------------------
GAUGE_XY = (0.430, -0.150)
GAUGE_Z = 0.120                   # the dial's centre
GAUGE_TILT = math.radians(50)     # the dial faces this far up from facing the arms
DIAL_R = 0.060                    # 12 cm across
NEEDLE_SWEEP = math.radians(270)  # 0 to GAUGE_MAX


def needle_angle(p: float) -> float:
    """Clockwise from straight up, seen on the dial: 0 at -135 degrees, GAUGE_MAX at +135 (a stop holds it there)."""
    return -NEEDLE_SWEEP / 2 + NEEDLE_SWEEP * min(max(p, 0.0), GAUGE_MAX) / GAUGE_MAX


# -- scoring ---------------------------------------------------------------------------------------------------------------
CLEAR = 0.01                      # let go: every part of both arms at least 1 cm from the rod and the clip
HELD_LEVEL = math.radians(5)      # a held rod counts only within this of level (not pressed down onto the band)
TOUCH = 0.003                     # touching: part of the robot within 3 mm of the band, bladder, thigh or gauge
