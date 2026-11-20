"""Every size and place in the scene, in metres, and the stated rules' numbers. Pure Python (the scorer's tests need
only the standard library).

Tabletop frame: x forward from the line through the two arm bases, y to the robot's left, z up from the table top.
``[E]`` marks an estimate to be measured on the real items (TECHNICAL.md says how); ``[S]`` a simulation choice
explained in the fidelity note.

The plate's own frame: origin at its middle, x along its length toward its top (unspotted) edge, y across it, z out
of the silica face. At the start it lies flat on its bar, silica up, its top edge away from the robot, so its frame
is the table's, moved.
"""

import math

# -- the TLC plate: MilliporeSigma 1.15327 (silica gel 60 F254 on glass, 2.5 x 7.5 cm, 250 um layer) -----------------
PLATE_LEN = .075
PLATE_WIDE = .025
GLASS_T = .0020                  # [E] glass thickness (the maker gives the layer, not the glass)
LAYER_T = .00025                 # silica gel layer, maker's nominal
PLATE_T = GLASS_T + LAYER_T
PLATE_MASS = .0097               # [E] 2 mm glass at 2.5 g/cm3 plus the layer
BASELINE_UP = .010               # the pencil baseline, this far above the bottom edge (Nichols: about 1 cm)
LANE_Y = (.006, 0., -.006)       # ticks 1, 2, 3 across the plate from its centre line, 1 on the robot's left
TICK_LEN = .002                  # each tick is a 2 mm pencil stroke across the baseline
SPOT_MAX_D = .003                # a printed 3 mm circle: a spot must fit inside it
LANE_HALF = .003                 # a spot counts toward lane k when its centre is within this of tick k's crossing
BARE_MAX = .001                  # bare glass longer than this in a lane is a gouge (scored)

# -- the riser: 3 in (76.2 mm) of 1/2 in (12.7 mm) square aluminium bar under the plate's centre line ------------------
BAR = (.0762, .0127, .0127)      # length (x), width (y), height (z)
PLATE_X, PLATE_Y = .262, .020    # the plate's (and the bar's) middle on the table
PLATE_Z = BAR[2] + PLATE_T / 2   # the plate's middle above the table at the start
FACE_Z = BAR[2] + PLATE_T        # the silica face's height at the start

# -- spotters: borosilicate capillaries 1.5 mm OD x 0.86 mm ID x 100 mm, standing in a foam rack -----------------------
CAP_LEN = .100
CAP_OD = .0015                   # [S] square section 1.5 x 1.5 mm in simulation (a round tube rolls between flat pads)
CAP_ID = .00086
CAP_MASS = .002                  # [S] real about 0.26 g; heavier for contact stability (fidelity note)
RACK = (.030, .110, .025)        # capillary rack block: x, y, height
RACK_X, RACK_Y = .165, .150
HOLE = .005                      # square holes 5 mm across ...
HOLE_DEPTH = .020                # ... 20 mm deep (5 mm of block under each)
HOLE_Y = (.190, .150, .110)      # spotters 1, 2, 3 (1 on the robot's left)

# -- samples: 4 mL glass vials (15 x 45 mm) of dyed water, 1 mL each, in a vial block -----------------------------------
VIAL_OD = .015
VIAL_ID = .0125
VIAL_H = .045
VIAL_NECK_ID = .0085             # the screw-thread neck's opening, its top 7 mm
VIAL_NECK_H = .007
VIAL_FLOOR = .002
SAMPLE_ML = 1.
LIQUID_Z = VIAL_FLOOR + SAMPLE_ML * 1e-6 / (math.pi * (VIAL_ID / 2) ** 2)    # the sample's surface, 10.1 mm up
VIAL_X = .235
VIAL_Y = (.180, .140, .100)      # samples 1, 2, 3 (1 on the robot's left)
VIAL_BLOCK = (.030, .120, .020)  # x, y, height; holes 16 mm across
VIAL_BLOCK_Y = .140
DYES = ("red", "blue", "green")  # food colouring: sample 1 red, 2 blue, 3 green

# -- the developing chamber: Sigma-Aldrich Z146226 rectangular TLC developing tank with its lid -------------------------
TANK = (.083, .121, .108)        # outside: front-to-back (x), side to side (y), height; maker's W, L, H
TANK_WALL = .003                 # [E] glass walls ...
TANK_FLOOR = .004                # [E] ... and floor
TANK_IN = (TANK[0] - 2 * TANK_WALL, TANK[1] - 2 * TANK_WALL)          # inside: 77 x 115 mm
TANK_X, TANK_Y = .250, -.160     # the tank's middle on the table; its long sides face the robot
TANK_MASS = .400                 # [E] glass
SOLVENT_DEPTH = .005             # water, 5 mm deep
SOLVENT_Z = TANK_FLOOR + SOLVENT_DEPTH
SOLVENT_ML = TANK_IN[0] * TANK_IN[1] * SOLVENT_DEPTH * 1e6               # 44 mL
LID = (.087, .125, .004)         # [E] the tank's flat glass lid: front-to-back, side to side, thickness
LID_MASS = .105                  # [E]
LID_X, LID_Y = .135, -.250       # lying flat on the bench in front of the tank

# -- the stated rules ---------------------------------------------------------------------------------------------------
CONTROL_HZ = 20
DIP_MIN = .002                   # a spotter fills when its tip is at least this far below a sample's surface ...
FILL_S = .25                     # ... for this long, inside the vial's bore
TOUCH_MIN_N = .05                # a spot needs the tip pressing the silica within this force band ...
TOUCH_MAX_N = 1.0                # [E] ... (above it the tip gouges the layer)
DWELL_S = .1                     # ... for at least this long (a brush or a bounce leaves nothing)
SPOT_D0 = .0010                  # [E] a spot's diameter after the dwell ...
SPOT_GROWTH = 1.6e-5             # [E] ... grows as sqrt(D0^2 + growth x seconds in contact): 3.0 mm after 0.5 s
GOUGE_N = TOUCH_MAX_N            # pressing harder than this for GOUGE_S scrapes the layer off: bare glass 1.5 mm
GOUGE_S = .1                     # (a single-step spike as the tip lands is an impact of a few ms, not a press)
GOUGE_LEN = .0015
SLIDE_MAX = .001                 # sliding the tip more than this while pressing scrapes a track that long
BREAK_N = 3.0                    # [E] a spotter breaks above this force from anything but the grippers' pads
DROP_TILT_DEG = 60.              # a spotter lying further than this from upright with no gripper holding it is dropped
HELD_N = 1.0                     # a spotter counts as held while the pads squeeze it with at least this in total
MERGE = .001                     # a touch within this of an earlier spot of the same sample grows that spot
LEAN_MAX_DEG = 35.               # the plate leans at most this far from vertical: the baseline stays 3 mm over 5 mm of
                                 # solvent up to 37 degrees (10 mm x cos 37 = 8 mm)
BASELINE_CLEAR = .003            # the baseline at least this far above the solvent's surface
LID_CENTRE = .006                # the lid on: its middle within this of the tank's middle ...
LID_TILT_DEG = 4.                # ... level within this, square to the tank within LID_YAW_DEG, on the rim
LID_YAW_DEG = 8.
CLEAR_M = .050                   # 1.0 needs both grippers' centre points this far from the tank and its lid
TANK_TIP_DEG = 10.               # 0 if the tank tips past this ...
TANK_SHIFT = .005                # ... or is pushed this far (a hand or the plate shoving it)
END_S = 1.                       # the trial ends 1 s after 1.0 is reached


def tick_point(k):
    """Tick k's crossing with the baseline, plate frame."""
    return (-PLATE_LEN / 2 + BASELINE_UP, LANE_Y[k], PLATE_T / 2)


def spot_diameter(seconds):
    return math.sqrt(SPOT_D0 ** 2 + SPOT_GROWTH * max(0., seconds))


def seconds_for(diameter):
    return max(0., (diameter ** 2 - SPOT_D0 ** 2) / SPOT_GROWTH)


def hole_point(k):
    """The bottom of spotter k's rack hole, on the table."""
    return (RACK_X, HOLE_Y[k], RACK[2] - HOLE_DEPTH)


def vial_point(k):
    """The middle of sample k's surface."""
    return (VIAL_X, VIAL_Y[k], LIQUID_Z)
