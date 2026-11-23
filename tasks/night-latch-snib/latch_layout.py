"""Every size, mass, spring and position of the night latch scene, in one place. The task file and the scorer read
from here. Metres, kilograms, newtons. Frame: x forward from the midpoint between the arm bases, y to the robot's
left, z up; heights are above the table top.

Sources (opened 2026-11-22):
  [Y]  Yale P77 Traditional Nightlatch, 60 mm backset, maker's page: 550 g with cylinder and staple; "a deadlocking snib
       function that allows the latch to be held back or securely deadlocked from the inside".
       https://www.yalehome.co.uk/77-traditional-nightlatch-60mm
  [F]  Yale P77 fitting instructions (60 mm from the door's edge to the knob and cylinder axis, doors up to 57 mm).
       https://media.wickes.co.uk/is/content/wickes/A3392_109885_TECH_0
  [D]  Dimension drawing and flat photo of the Yale 77 on a lock shop's listing: case 66 mm tall, fixing screws 67 mm and
       51 mm apart, bolt 25 mm tall and 16 mm thick, standing 13 mm out.
       https://www.lockshop-warehouse.co.uk/acatalog/yale-77-traditional-style-nightlatch-reversed-outward-opening.html
  [P]  Measured on [D]'s flat photo, scaled by the 67 mm screw spacing (+-2 mm): the remaining outside sizes.
  est. Estimates: everything inside the case (the latch was not opened), springs, masses of single parts.
"""

# -- where things stand -------------------------------------------------------------------------------------------------
CASE_FACE_X = 0.400  # the case's front face (the robot's side)
KNOB_Y = -0.040  # the knob's axis
LATCH_H = 0.150  # the knob's axis above the table top

# -- the door section (a cut from a 44 mm solid-core door, painted white) ---------------------------------------------
DOOR_T = 0.044  # standard UK external door thickness
DOOR_W = 0.300  # along y, ending at the latch edge
DOOR_H = 0.300  # standing on the table
BRACKET = (0.075, 0.040, 0.075)  # two steel angle brackets screw it to the table on its far side (x, y, z)

# -- the latch case [D][P] ----------------------------------------------------------------------------------------------
BACKSET = 0.060  # door edge to the knob's axis [F]
CASE_W = 0.093  # along y: from the door's edge to 33 mm past the knob's axis [P]
CASE_H = 0.066  # [D]
CASE_BODY_H = 0.051  # the raised middle of the case [P]
CASE_D = 0.025  # how far the raised case stands off the door (est.)
CASE_FLANGE_D = 0.013  # the screw flanges above and below it [D]

# -- the knob [P] -------------------------------------------------------------------------------------------------------
KNOB_W = 0.025  # across the oval (along y at rest)
KNOB_L = 0.038  # along the oval (upright at rest)
KNOB_HEAD_T = 0.012  # thickness of the oval head (est. from the listing's angled photo)
KNOB_STEM_R, KNOB_STEM_L = 0.0075, 0.010  # the neck between case and head (est., same photo)
KNOB_MASS = 0.060  # est. (die-cast)
KNOB_STOP_DEG = 72.0  # the knob's stop (est.); its top turns away from the door's edge
KNOB_SPRING = 0.0005  # N m per degree: a light spring that returns the knob when the bolt is held back (est.)
KNOB_DAMPING = 0.00005  # N m per degree/s

# -- the bolt [D] -------------------------------------------------------------------------------------------------------
THROW = 0.013  # how far the bolt stands out of the door's edge at rest
BOLT_H, BOLT_T = 0.025, 0.016
BOLT_OVERTRAVEL = 0.001  # it can be drawn 1 mm past flush (est.)
BOLT_MASS = 0.120  # est. (brass)
BOLT_K = 400.0  # N/m (est.)
BOLT_PRELOAD = 0.012  # m of spring compression with the bolt out: 4.8 N at rest, 10.4 N drawn back (est.)

# -- inside the case (est.) -----------------------------------------------------------------------------------------------
CAM_R = 0.0125  # the knob's cam pin, its radius from the knob's axis
CAM_PIN_R = 0.0025
CAM_START_DEG = 35.0  # the pin starts this far toward the door's edge from straight up
CAM_PLAY = 0.0004  # gap between the pin and the bolt's tail at rest
LAYER = (0.004, 0.014)  # all inside parts lie between these distances from the door's face

# -- the snib [P] ---------------------------------------------------------------------------------------------------------
SNIB_FROM_KNOB = 0.036  # the button's centre, toward the door's edge from the knob's axis
SNIB_R = 0.006  # a round button 12 mm across
SNIB_PROUD = 0.006  # standing this far off the case (est. from the angled photo)
SNIB_TRAVEL = 0.006  # down from its free end to its hold end
SNIB_FREE_Z = 0.003  # the button's centre above the knob's axis at its free end; 3 mm below it at its hold end
SNIB_MASS = 0.010  # est.
SNIB_DETENT_K = 300.0  # N/m: the detent seats the button at the nearer end of its slot, 0.9 N to move it (est.)
STOP_W, STOP_H = 0.005, 0.006  # the snib's stop inside the case: along the bolt, and in height
NOTCH_W = 0.007  # each of the two notches in the bolt's tail (one for bolt out, one for bolt drawn back)

# -- what the scorer reads (a ruler at the door's edge, a photo of the case) ------------------------------------------------
BACK_MAX_OUT = 0.003  # the bolt counts as drawn back when it stands at most this far out of the door's edge (13 mm at rest; the snib holds it about 1 mm out)
SNIB_HOLD_MIN = 0.004  # the button counts as at its hold end at least this far below its free end
CLEAR_M = 0.050  # both grippers at least this far from the latch
HOLD_S = 1.0  # for this long, at the end of the run

# colours
WHITE, CASE_C, BRASS, STEEL, DARK = (0.93, 0.93, 0.91), (0.47, 0.49, 0.40), (0.83, 0.68, 0.30), (0.6, 0.6, 0.62), (0.08, 0.08, 0.08)
