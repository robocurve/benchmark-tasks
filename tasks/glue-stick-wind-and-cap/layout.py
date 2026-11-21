"""Every size, mass, friction, force and place of the glue-stick scene, in one module. The task file and the scorer
read it. Frame: x forward from the arm bases, y left, z up, metres; the table top is z = TOP. The stick lies along y,
its knob toward +y (the left arm), its glue end toward -y (the right arm).

Sources (opened 2026-11-20):
- Pritt Stick 43 g: "Dimensions: 31x31x113mm", "Weight: 59g" (one stick, with cap).
  https://www.officestationery.co.uk/product/pritt-stick-glue-stick-43g-pack-of-24-1564148-hk1035-HK1035/
  The split of that length into knob, barrel, neck and cap, the 6 mm of travel per turn, the cap's 8 N click and the
  knob's turning friction are estimates (the maker does not publish them).
"""

from __future__ import annotations

TOP = 0.7475  # the station's table top

# -- the stick: barrel (origin at the shoulder's centre, where the cap's rim seats), knob, glue column --------------
BARREL_R, BARREL_L = 0.0150, 0.078  # 30 mm barrel, from the shoulder (y = 0) to the knob (y = +78 mm)
NECK_R, NECK_L = 0.0130, 0.016  # 26 mm neck the cap slides over, from the shoulder to the rim (y = -16 mm)
KNOB_R, KNOB_L = 0.0135, 0.012  # 27 mm ribbed knob, 12 mm long, beyond the barrel's end
GLUE_R, GLUE_L = 0.0105, 0.070  # 21 mm glue column, 70 mm long (inside the neck's 2 mm wall)
GLUE_OUT0 = 0.012  # at the start the glue's tip stands 12 mm past the rim
GLUE_MIN, GLUE_MAX = -0.020, 0.025  # how far out the screw can carry the tip (below / above the rim)
PITCH = 0.006  # m of glue travel per turn of the knob (estimate)
WIND_OUT = -1.0  # turning the knob clockwise seen from its end (negative about +y) winds the glue out
KNOB_DAMP = 5.0e-4  # N m per deg/s: about 0.05 N m to turn the knob at 100 deg/s (estimate)
BARREL_MASS, KNOB_MASS, GLUE_MASS = 0.008, 0.004, 0.043  # with the cap's 4 g: the listed 59 g; 43 g is the glue
MU = (0.9, 0.7)  # polypropylene on the laminate table / on the pads (estimate)

# -- the cap: a cup, origin at its rim's centre, open toward +y --------------------------------------------------------
CAP_R, CAP_WALL, CAP_DEPTH, CAP_END = 0.0155, 0.0015, 0.019, 0.002  # 31 mm across, 21 mm long
CAP_IN_R = CAP_R - CAP_WALL  # 14 mm: 1 mm round the neck
CAP_MASS = 0.004
CAP_STAVES = 16
# seated, the cap's inner end is CAP_DEPTH - NECK_L = 3 mm past the rim: glue standing out more than that stops it
FIT_ENGAGE = 0.006  # the snap fit catches the cap's rim on the last 6 mm before the shoulder ...
FIT_PUSH = 8.0  # ... resists it there with 8 N (a gripper gives 50 N per finger) ...
FIT_CLICK = 0.0007  # ... clicks home within 0.7 mm of the shoulder ...
FIT_HOLD = 15.0  # ... and then holds it seated against up to 15 N

# -- the start (fixed) ----------------------------------------------------------------------------------------------------
X = 0.36  # the stick's axis, cm from the arm bases' line (see the reach check in the README)
STICK_Y = -0.035  # the shoulder's place on y: the stick spans y -63 mm (glue tip) to +55 mm (knob end)
AXIS_Z = TOP + BARREL_R  # lying on the table
CAP_Y = -0.110  # the cap's rim; it lies on its side, open end toward the stick, its axis on the stick's axis line
CAP_X = X

# -- scoring thresholds (the scoring text, number for number) -----------------------------------------------------
SEATED_GAP = 0.001  # cap rim within 1 mm of the shoulder
SEATED_OFF = 0.0015  # and on the axis (the joint holds it there once caught)
SEATED_TILT = 5.0
TOUCH = 0.0005  # the cap's inner end within 0.5 mm of the glue's tip: pressed on the glue
ON_TABLE = 0.004  # the stick's lowest point within 4 mm of the table
HANDS_CLEAR = 0.030  # every finger point at least 3 cm from the stick and the cap at the end
POINTS = {"full": 1.0, "wound": 0.5}

# a round barrel on a simulated table rolls away from the smallest nudge (PhysX has no rolling resistance); a real
# stick lying on a desk stays put. Angular damping (1/s) stands in for that resistance.
ROLL_DAMP = 5.0
