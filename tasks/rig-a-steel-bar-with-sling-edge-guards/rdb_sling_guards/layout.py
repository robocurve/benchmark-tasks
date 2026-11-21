"""Every dimension, mass, force and placement of the fixed scene, each marked as published (with its source), derived,
or an ESTIMATE. Tabletop frame: x forward from the line through the two arm bases, y to the robot's left, z up from
the table top; metres, kilograms, newtons, seconds. Pure Python: the scorer's tests need no NumPy.
"""

import math

IN = 0.0254

# -- the load: a 4 x 4 in cold-finished steel square on two 2x4 blocks ----------------------------------------------
BAR_S = 4 * IN                      # 101.6 mm square, sharp (as-drawn) edges; a common bar size
BAR_L = 12 * IN                     # 304.8 mm long
STEEL_DENSITY = 7850.
BAR_MASS = STEEL_DENSITY * BAR_S * BAR_S * BAR_L      # derived: 24.7 kg (the bar is fixed in the scene)
XS = .250                           # the sling passes under the bar at x = 250 mm (chosen for reach: check 1)
BLOCK_H, BLOCK_W, BLOCK_L = 1.5 * IN, 3.5 * IN, .300  # 2x4 laid flat (nominal 1.5 x 3.5 in), 300 mm long, across
Z_BOT = BLOCK_H                     # the bar's underside, 38.1 mm up
Z_TOP = BLOCK_H + BAR_S             # its top face, 139.7 mm up
HALF = BAR_S / 2
BAR_X = (XS - BAR_L / 2, XS + BAR_L / 2)
BLOCK_GAP = .015                    # each block's outer side this far in from the bar's end
BLOCK_X = ((BAR_X[0] + BLOCK_GAP, BAR_X[0] + BLOCK_GAP + BLOCK_W), (BAR_X[1] - BLOCK_GAP - BLOCK_W, BAR_X[1] - BLOCK_GAP))

# -- the sling: 2 in two-ply nylon web, triangle fittings both ends (Type 2), 3 ft bearing point to bearing point ----
# (HSI TT2-802-03 class: 2 in wide, 3 ft, 6,400 lb vertical rating; fittings about 0.45 kg each). A 2 ft sling was too
# short for this basket with the guards on: with the first eye on the hook, the web had to be pulled up off the table
# and tight round the bar before the second eye could reach its horn (native runs full5 and fullb).
SLING_W = 2 * IN
BEARING_L = 36 * IN                 # 914 mm, apex bar to apex bar when straight
WEB_T = .0045                       # web collider thickness; a two-ply 2 in web is about 3-4 mm. ESTIMATE
FIT_H = .055                        # fitting: base bar (web loop) to apex bar (bearing point). ESTIMATE
FIT_BASE_HALF, FIT_APEX_HALF = .032, .020      # half lengths of the base and apex bars. ESTIMATE
FIT_BAR_R, FIT_SIDE_R = .007, .006  # 14 mm bars (13-16 mm on 2 in triangles), 12 mm sides. ESTIMATE
FIT_MASS = .45                      # 1-1.1 lb per triangle (Web-Trap ST-2, Lift-All Unilink catalogue figures)
WEB_L = BEARING_L - 2 * FIT_H       # derived: 804 mm of web between the base bars
N_LINKS = 53
LINK_L = WEB_L / N_LINKS            # derived: 15.2 mm links (the probe: 15 mm links lie flatter round the bar
                                    # than 22.7 mm ones)
LINK_MASS = .44 / 33                # 13.3 g a link, 0.71 kg of web in all; a real 3 ft web is about 0.12 kg. Raised for
                                    # steady contact against 0.45 kg fittings (probe). ESTIMATE, stated in the fidelity note
WEB_EI = 2.3e-4                     # N m^2 bending stiffness of the two-ply web. ESTIMATE
HINGE_K = WEB_EI / LINK_L           # derived: N m/rad per hinge
HINGE_D = .002                      # N m s/rad. ESTIMATE
HINGE_ARMATURE = 1e-5
HINGE_LIMIT_DEG = 130.
WEB_FRICTION = .40                  # nylon web on steel and on the guards' PETG. ESTIMATE

# -- edge guards: printed PETG angle, 3 in long, with four recessed NdFeB discs ------------------------------------
# No published magnetic sling protector is short enough for the YAM's 94 mm grip (the smallest LiftGuard is 6 in), so
# the guard is a printed copy of the type, shortened to 3 in (cad/edge_guard.stl).
GUARD_LEN = 3 * IN                  # 76.2 mm along the edge: the 2 in web plus 12.7 mm each side
GUARD_FL = .045                     # flange width from the inside corner
GUARD_T = .003                      # flange thickness
GUARD_NOSE_R = .0045                # the rounded, thickened bearing nose along the outside of the corner
TAB_T = .004                        # a grip tab across each end of the guard (outside the web's width): this thick ...
TAB_FROM, TAB_TO = .004, .024       # ... from 4 to 24 mm out from the inside corner along the outside bisector ...
TAB_W = .012                        # ... and 12 mm wide; and a lobe on the same end walls, out from the side flange's
LOBE_OUT, LOBE_DOWN = .020, .022    # outside face (20 mm) along its first 22 mm from the corner. The hand pinches the
                                    # guard's two end walls (tab and lobe): from above-outside at a top edge (on the
                                    # tab), from above at a bottom edge (on the lobe)
PETG_DENSITY = 1270.                # kg/m^3, typical PETG filament datasheet value
MAGNET = "K&J Magnetics D81-N52 (1/2 in x 1/16 in NdFeB disc, grade N52)"
MAGNET_D, MAGNET_T = .0127, .0015875
MAGNET_PULL_N = 3.84 * 4.4482216    # 17.1 N (3.84 lb), case 1 (one magnet on a thick steel plate): the figure used.
                                    # K&J lists the N42 disc of this size (D81) at 3.1 lb; the N52 grade pulls about
                                    # (1.44 / 1.31)^2 = 1.2 times as hard (remanence squared), about 3.7 lb. ESTIMATE
MAGNET_MASS = 7500. * math.pi * (MAGNET_D / 2) ** 2 * MAGNET_T     # derived: 1.5 g
MAG_RECESS = .0005                  # the discs sit 0.5 mm below the inside faces (they cannot scratch the bar)
MAG_G0 = .0010                      # gap at which one disc's pull falls to a quarter. ESTIMATE, fitted to the shape
                                    # of the maker's pull-versus-distance curves for thin discs
MAG_RANGE = .015                    # beyond this gap the pull is ignored
MAG_POS = ((-.019, .0225), (.019, .0225))   # each flange: (along the edge from the middle, out from the corner)
GUARD_FRICTION = .45                # PETG on steel. ESTIMATE
GUARD_VOLUME = (GUARD_LEN * (2 * GUARD_FL - GUARD_T) * GUARD_T
                + GUARD_LEN * (math.pi * GUARD_NOSE_R ** 2 * .75)
                + 2 * TAB_T * ((TAB_TO - TAB_FROM) * TAB_W + LOBE_OUT * LOBE_DOWN))   # derived, approximate
GUARD_MASS = PETG_DENSITY * GUARD_VOLUME + 4 * MAGNET_MASS              # derived: about 0.043 kg

# where each guard ends up: (y, z) of the bar's edge, the outward normal of the horizontal face (+1 top, -1 bottom)
# and of the side face (+1 left, -1 right)
EDGES = {"TL": ((HALF, Z_TOP), 1., 1.), "BL": ((HALF, Z_BOT), -1., 1.),
         "TR": ((-HALF, Z_TOP), 1., -1.), "BR": ((-HALF, Z_BOT), -1., -1.)}
SEAT_TOL = .003                     # seated: inside corner within 3 mm of the edge ...
SEAT_ANGLE_DEG = 10.                # ... the guard turned less than this from its seat ...
COVER_MARGIN = .005                 # ... and it covers the web's width with at least this to spare each side

# -- trays: two printed trays on the table, two guards in each ------------------------------------------------------
# Each guard waits in the pose from which the hand can pinch its tabs and carry it to its edge: the top-edge guard
# stands as a tent (corner and tabs up, flanges resting on their free edges), on the tray's inner side; the bottom-edge
# guard sits as it will sit on its bottom edge (one flange flat, the other standing), on a block, with its corner at
# the block's outer edge so that its tabs hang clear.
TRAY_XY = {"left": (.130, .215), "right": (.130, -.215)}
TRAY_SIZE = (.110, .150, .008)      # floor 8 mm thick
TRAY_WALL = (.004, .012)            # wall thickness and height above the floor
TENT_Y = -.037                      # the tent's corner from the tray's middle (+ outward, away from the robot's middle)
BLOCK_CORNER_Y = .047               # the bottom-edge guard's inside corner from the tray's middle (outward)
GUARD_BLOCK_H = .021                # the block it sits on: this tall ...
GUARD_BLOCK = (.080, .043)          # ... this long (along x) and wide (inward from 1 mm inside the corner)

# -- the hook: Carl Stahl CondorLift eye double hook, 1,000 kg (material no. 25302030015001, maker's no. 68580011): two
# forged safety latches; "for synthetic slings"; both horns must be loaded equally (one sling eye on each horn).
# Published dimensions (carlstahl.com): A 175 overall width, B 80 bowl centre to bowl centre, C 55 x D 75 eye inside,
# E2 150 overall height, F 19 eye wall, G 26 bowl floor wall, R 32 bowl inside radius, X 27 opening, X1 20 thickness.
HOOK_Z = .200                       # the bowls' inside bottoms at rest, 200 mm above the table (chosen for
                                    # reach: probe/hang_plan.py, and room for the guards: probe/bl_grid.py)
RAM_B = .080                        # B
BOWL_R_IN = .032                    # R
HOOK_T = .020                       # X1: the forging's thickness
HOOK_R = HOOK_T / 2                 # capsule radius of the horns
RC = BOWL_R_IN + HOOK_R             # derived: the horns' centreline radius
HOOK_MASS = 1.75                    # not published. ESTIMATE from the outline (about 220 cm^3 of steel)
TIP_DEG = 25.                       # the horn tips rise to this angle above the bowl centre's level (drawing)
EYE_TOP = .0975                     # the eye's outside top above the bowl centres' midpoint (E2 - R - G = 92 mm; drawn
                                    # 97.5 to keep the eye's 75 mm inside height)
LATCH_MASS = .03                    # ESTIMATE
LATCH_K = .06                       # N m/rad latch spring. ESTIMATE
LATCH_PRELOAD = .03                 # N m against the tip. ESTIMATE
LATCH_CLOSED_DEG = 5.               # a latch counts as closed within 5 degrees of its stop
APEX_OFF_PLANE = FIT_APEX_HALF - FIT_SIDE_R  # 14 mm: an apex bar centred further than this off the hook's plane does
                                    # not have the horn inside its triangle (beside the hook, not on it)
LATCH_PIVOT = (.034, .032)          # (|y|, z) of each latch's pivot from the bowl centres' midpoint (drawing)
LATCH_END = (.0686, .0208)          # (|y|, z) of its free end, against the tip's inside face (the latch engages the
                                    # tip; a latch reaching to the tip's middle swept the bowl's floor and a fitting
                                    # resting there held it open in a native run)
PEND_L = .30                        # the hoist's load chain and hook above the double hook's eye. ESTIMATE
HOIST_MASS, GIMBAL_MASS = .50, .05  # the hoist's moving parts in the scene (its body and the chain's top)
LIFT_K = 3000.                      # N/m: the hoist's position drive (the operator's lift)
CHAIN_MASS = .30                    # ESTIMATE
SWING_DAMPING = .30                 # N m s/rad at the chain's top. ESTIMATE
HOIST_PRELOAD = ((HOIST_MASS + GIMBAL_MASS + HOOK_MASS + CHAIN_MASS + 2 * LATCH_MASS) * 9.81 / LIFT_K)   # derived:
                                    # the drive's target at rest, so that the hook hangs at HOOK_Z under its own weight
FRICTION = {"steel": .35, "web": WEB_FRICTION, "guard": GUARD_FRICTION, "wood": .50, "table": .50, "hook": .30}

# -- the test the operator runs once the hitch is made and the hands are clear -------------------------------------
CLEAR_S = 1.0                       # the robot clear of the sling, hook and guards for this long before the test
CLEAR_DIST = .080                   # clear: both tool points at least this far from the hook's centre (midway
                                    # between its bowls) and from both apex bars, and no arm contact with the hook,
                                    # the fittings, the web or the guards
LIFT_RATE = .030                    # the hoist takes up the slack at 30 mm/s ...
PULL_N = 50.                        # ... until the hook carries 50 N, held (force control) ...
HOLD_S = 2.0                        # ... for 2 s
HOLD_MIN_N = 40.                    # if the hook's load falls below this during the hold, the hoist lifts again and
                                    # the hold starts over (in a native run the web snagged on a bare corner, the pull
                                    # read 55 N for one step and fell back to the 11 N it read before the lift)
TEST_LIMIT_S = 30.                  # a test that has not completed its hold this long after the lift began fails
EDGE_ZONE = .015                    # contact within 15 mm of an edge counts as that edge's
BARE_LIMIT_N = .5                   # web on bare steel near an edge: any sample above this fails the test
PULL_KI = 4e-4                      # m/s per newton: the hoist's force control while holding
