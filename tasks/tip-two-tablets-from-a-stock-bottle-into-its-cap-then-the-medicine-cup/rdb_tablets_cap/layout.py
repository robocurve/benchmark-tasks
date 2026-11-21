"""Every dimension of the bottle, its cap, the tablets and the medicine cup, in metres and kilograms.

PAGE = read on a product or label page (listed in the README); EST = estimate, to be measured on the bought
item; MODEL = a simulation choice that differs from the real object and is disclosed in task.json.
"""

import math

# -- stock bottle: 120 cc white HDPE packer, 38-400 neck -------------------------------------------------
BOTTLE_R = .02525            # PAGE: 1.989 in across
BOTTLE_H = .0864             # PAGE: 3.402 in tall
BOTTLE_MASS = .01363         # PAGE: 13.63 g
BODY_H = .062                # EST: where the shoulder starts
SHOULDER_H = .071            # EST: where the neck starts
NECK_R_OUT = .0176           # EST: 38-400 finish under the thread
NECK_R_IN = .0155            # EST: 31 mm bore
BOTTLE_WALL = .004           # MODEL: 4 mm walls (real about 1 mm); the outside is true, the inside 6 mm narrower
BOTTLE_FLOOR = .003          # MODEL
BODY_R_IN = BOTTLE_R - BOTTLE_WALL
SIDES = 16                   # MODEL: every round wall is 16 flat staves

# -- its cap: 38-400 white polypropylene push-down-and-turn child-resistant cap -----------------------------
CAP_R = .0223                # PAGE: 44.6 mm across
CAP_H = .0172                # PAGE: 17.2 mm tall
CAP_MASS = .006              # EST
CAP_R_IN = .018              # EST: 36 mm across inside
CAP_FLOOR = .004             # EST: inside 13.2 mm deep
CAP_DEPTH = CAP_H - CAP_FLOOR

# Two opposed bayonet lugs run under neck ledges. A spring collar seats them
# against the ledges. Depressing the cap clears the counterclockwise stop.
CLOSED_Z = BOTTLE_H + CAP_FLOOR + .004     # top of cap, in bottle coordinates
COLLAR_Z = CLOSED_Z - CAP_H                 # z of the sprung rim contacting the cap skirt
COLLAR_TRAVEL = .004
COLLAR_MASS = .001
COLLAR_STIFFNESS = 110.                     # N/m; model parameters, not product measurements
LUG_TOP = .0828
LUG_THICKNESS = .0014
LEDGE_Z = LUG_TOP + .0002
STOP_Z = LUG_TOP - .0017
LUG_HALF_ANGLE = 5.                         # degrees
WINDOW_START = 29.
WINDOW_END = 70.

# -- tablets: round brown film-coated 200 mg ibuprofen tablets ----------------------------------------------
TABLET_R = .005              # PAGE: 10 mm, round, brown
TABLET_T = .005              # EST: thickness at the middle
TABLET_EDGE = .0026          # EST: height of the straight edge band
TABLET_MASS = .00033         # EST
N_TABLETS = 30
DOSE = 2

# -- medicine cup: 1 oz graduated polypropylene cup ---------------------------------------------------------
CUP_R_TOP = .0238            # PAGE: 1 7/8 in across the top
CUP_R_BOTTOM = .0143         # PAGE: 1 1/8 in across the bottom
CUP_H = .0349                # PAGE: 1 3/8 in tall
CUP_MASS = .0011             # PAGE (upper bound): case shipping weight / 5,000
CUP_WALL = .0008             # EST (real); MODEL: collision walls are 2.5 mm thick, outside the true inner surface
CUP_FLOOR = .002             # MODEL
CUP_MODEL_WALL = .0025

FRICTION = .35               # EST: coated tablet on plastic, plastic on the table
GRIP_FRICTION = .8           # EST: gripper pads on ribbed plastic


def cup_inner_radius(z):
    """Inside radius of the cup at height z above its base."""
    z = min(max(z, 0.), CUP_H)
    return (CUP_R_BOTTOM - CUP_WALL) + (CUP_R_TOP - CUP_R_BOTTOM) * z / CUP_H


def tablet_start(i):
    """Start position of tablet i in the bottle frame (origin on the table under the bottle's axis): flat
    layers of seven, each layer turned 30 degrees from the one below."""
    layer, k = divmod(i, 7)
    z = BOTTLE_FLOOR + TABLET_T / 2 + .0005 + layer * (TABLET_T + .0006)
    if k == 6:
        return (0., 0., z)
    a = math.radians(60 * k + 30 * layer)
    return (.014 * math.cos(a), .014 * math.sin(a), z)
