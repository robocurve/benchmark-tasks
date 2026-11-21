"""A fingerstick trainer lies palm up on a board clamped to the table: a palm block with an index finger, a thumb and a
ring finger, its fingertips lowered; the ring finger rides on a light spring and gives way when pressed unless it is
held. A BD Microtainer contact-activated lancet (21 G x 1.8 mm) stands tip down in a stand whose socket holds its
protective tab, and a BD sharps container stands beside. Do a fingerstick as BD directs. Done means: the lancet's tab
was twisted off in the stand before the lancet came out of it; the ring finger was held still by a gripper closed on it
from the lancet's first touch until after it fired; the lancet fired square to the skin on a side of the ring
fingertip's pad, without sliding; and the used lancet is inside the sharps container, let go.
"""
import math
import pathlib
import random
import sys
from dataclasses import dataclass

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, hull, mechanism, on_place, on_reset, on_step, run_seed  # noqa: E402
from yam_arena import station  # noqa: E402,F401
from yam_arena import ik  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
RUN_LIMIT_STEPS = 1200           # 60 s at 20 Hz. WHY: the slowest planned path (stem k00) ends at 44.4 s
TOP = TABLE_TOP[SCENE]
GRAVITY = 9.81                   # m/s^2 (yam_arena's)

# -- the lancet (BD Microtainer contact-activated lancet 366577, 21 G x 1.8 mm: a loose tool of two links) -------------
# Lancet frame: origin at the shield's face (shield out), z up the lancet's axis toward its rear.
L_W, L_T = 0.013, 0.010          # the housing's width and thickness [EST: BD publishes no body size; to be measured on
#                                  the real lancet with calipers before the trials, README Real trials check 3]
L_LEN = 0.037                    # the housing's length [EST, as above]
SHIELD_R = 0.0035                # the contact shield's radius [EST]
SHIELD_OUT = 0.003               # the shield's face stands this far past the housing's front [EST]
SHIELD_TRAVEL = 0.0022           # the shield's travel into the housing to its stop [EST]
K_SHIELD = 4000.0                # N/m: the shield's spring (the drive spring's load path, lumped) [CHOICE]
F_PRE = 1.0                      # N: the shield's preload against its stop [CHOICE]
F_FIRE = 8.0                     # N on the shield fires it [EST: inside the 4 - 15 N activation force of the Owen Mumford
#                                  Unistik Touch contact-activated lancet, FDA 510(k) K222111; BD publishes no value]
F_FIRE_MAX_REAL = 15.0           # N: the real BD lancet is used only if it fires at or under this (README Real trials)
Q_TOUCH = 5e-5                   # m: the shield pushed this far off its stop is touching something (under it the
#                                  model reads no force on the shield)
F_TOUCH = F_PRE + K_SHIELD * Q_TOUCH   # the shield's force once it is off its stop (a placement's start, below)
TOUCH_D = 0.0003                 # m: the shield's face this close to the ring finger's skin (or the shield off its stop
#                                  with the face on it) is the touch, which starts the hold-and-still window. WHY a
#                                  distance and not a force: a real trial reads the touch as the first side-camera frame
#                                  where the face meets the finger, and the camera sees contact, not force. In the t00
#                                  trace the face sat 0.03 to 0.29 mm off the skin for 0.5 s before the shield left its
#                                  stop [RUN]
LANCET_G = 4.0                   # g: the lancet's mass [EST], for the setup text
LANCET_KG = (0.0200, 0.0020)     # sim masses, housing and shield [CHOICE: heavier than the real lancet, for PhysX].
#                                  WHY 2 g on the shield: at 0.5 g (600x lighter than the hand) PhysX let it sink 2.9 mm
#                                  into the fingertip at 3.8 N, so it never fired [RUN]. WHY 20 g on the housing: at 2 g
#                                  the right's 19 N a pad grip (1.2 friction) let the housing slide 1 to 2.6 mm across
#                                  and up the pads when lifted out of the stand and at 4.6 to 6 N of press, far under
#                                  friction's 45 N (PhysX, 2026-11-20); a light body's solver slip, our inference. At
#                                  20 g it held to 0.1 mm [RUN]. Its weight, 0.2 N, is 2% of the press

# -- the stand (its socket holds the tab; the tab's frangible post is a sprung turn) -----------------------------------
STAND = (0.340, -0.200)          # the stand's centre (world x, y): ahead of the trainer's fingertips, on the right arm's
#                                  side [CHOICE: tools/grippers.py]
STAND_H = 0.025
STAND_HALF = 0.020
CUP_D = 0.008                    # the socket's depth above its floor (the tab)
CUP_CLEAR = 0.0004               # each side between the socket and the housing
CUP_WALL = 0.0025
TAB_H = 0.002                    # the tab cap's height in the socket's floor
T_TAB = 0.030                    # N m: the tab's break torque [EST: under the "< 4.9 N-cm" cap twist-off torque of the
#                                  Facet 28G Universal Lancet, FDA 510(k) K221433; BD publishes no value]
T_TAB_MAX_REAL = 0.049           # N m: the real tab is used only if it breaks at or under this (README Real trials)
TAB_BREAK = math.radians(45.0)   # the tab breaks off at this turn, either way [EST]
K_TAB = T_TAB / TAB_BREAK        # N m/rad: the turn's spring, so its torque reaches T_TAB at the break
TAB_STOP = math.radians(100)     # the socket turns this far either way. WHY both ways: BD's step 6 figure shows a twist
#                                  (two curved arrows) but no direction, and a frangible tab breaks either way [EST]
LIFT_OUT = 0.010                 # m: the lancet risen more than this is out of the stand
OUT_XY = 0.004                   # m: or moved more than this across from where it stood
L_START = (STAND[0], STAND[1], TOP + STAND_H + TAB_H + 0.0005)

# -- the sharps container (BD 305635, 1 qt phlebotomy collector with needle ports) -------------------------------------
SHARPS = (0.220, -0.360)         # its centre (world x, y): in front of the right arm, outside the trainer's thumb
#                                  [CHOICE: tools/grippers.py and tools/clearance.py]
SH_L, SH_W, SH_H = 0.1245, 0.1092, 0.1651   # [SOURCE: BD sharps catalog, 305635 "4.9 x 4.3 x 6.5" in, L x W x H]
SH_OPEN = (0.0635, 0.0305)       # the lid's opening [SOURCE: same catalog, "2.5 × 1.2 open (305635)" in]
SH_WALL = 0.003                  # [EST]

# -- the trainer (a base clamped to the table, a palm block on a post, and a ring finger on a sprung slide) -------------
# Sizes from benchmark-tasks #1837 (merged; ANSUR II median adult male, hand length 193 mm, breadth 88 mm). Hand frame
# (u, v, w): u along the fingers toward the wrist (0 at the index fingertip), v across (thumb +v), w out of the palm.
FINGERS = {  # name: (v, apex u, knuckle u, distal radius, proximal radius). WHY no middle or little finger: a gripper
    #          closed round the ring finger needs its pads 7.8 to 14.2 mm either side of the finger's axis, where the
    #          neighbours lie (their surfaces 12.2 and 12.7 mm off it); the trainer leaves them off [CHOICE]
    "index": (0.000, 0.000, 0.095, 0.0082, 0.0088),
    "ring": (-0.045, -0.002, 0.097, 0.0073, 0.0078),
}
DIP = 0.0266                     # the distal joint's distance from the apex (#1837, index)
PALM_U, PALM_V, PALM_W = (0.126, 0.176), (-0.073, 0.013), 0.013   # WHY from 0.126: the slide's web stands behind the
#                                  ring's knuckle (WEB_X), and the block starts 8 mm behind it [CHOICE]
THUMB = ((0.042, 0.058, 0.0095), (0.016, 0.165, 0.012))    # (v, u, radius) at its tip and its base (#1837)
PITCH = math.radians(-20.0)      # the hand palm up, its fingers sloping down 20 deg from the knuckles to the tips. WHY:
#                                  palm up is BD's puncture figure and step order (README Sources, an inference); the
#                                  slope down follows BD's "hand in a downward position" (step 4), and the offline
#                                  search passes there as well as level (README Design) [RUN]
BASE_KG = 3.0                    # the base link's mass (fixed to the world: in the real rig a board held to the table by
#                                  two C-clamps) [CHOICE]
APEX = (FINGERS["ring"][1], FINGERS["ring"][0])             # (u, v) of the ring's apex: the hand mechanism's origin
HZ = 0.100                       # the ring's apex (on its axis) over the table. WHY not 140: with the fingertips
#                                  lowered, the right's pose 45 mm back from the site was out of its reach at 140 at
#                                  some layout corners (up to 35 mm) [RUN: tools/ikcheck.py]
RING_KG = 0.025                  # the ring finger with what moves with it [EST: the PETG print, about 15 g at 40% infill;
#                                  the guide's block, 8 g (its datasheet); four M2 screws, about 1 g]
K_RING = 100.0                   # N/m: the slide's return spring [CHOICE]
RING_PRELOAD = 1.0               # N: the spring holds the finger up on its stop with this to spare over its weight
#                                  [CHOICE]. WHY light: an unheld finger is pushed away by a lancet [inference]; a
#                                  hospital procedure for this lancet says "Grasp the finger" (README Sources). The
#                                  slide lumps a real finger's give (its knuckle, wrist and arm) into one vertical slide
#                                  behind the knuckle; the finger is rigid on it, so its tip gives the same [CHOICE]
DROP_STOP = 0.010                # m: the slide's travel down to its lower stop (the web's foot on the stop post)
DROP_MAX = 0.002                 # m: the finger pushed more than this down its slide has not been held still [CHOICE]
STEADY_AFTER = 0.5               # s: the finger must stay still from first touch until this long after the click.
#                                  WHY after: a fast jab can fire before a light finger moves, which then gives way [EST:
#                                  the 1-D impact estimate in README Model]
HAND_YAW = math.radians(180.0)   # the hand's turn on the table: the fingers point +x, away from the arm bases, the
#                                  wrist toward them, the thumb toward the right arm (-y)
HAND0 = (0.260, -0.120)          # the ring's apex over the table (world x, y) before the seed's shift and turn [RUN:
#                                  offline search, README Design]

# -- the site (BD VS7688-1: "Identify the desired puncture site (shaded area)"; "middle or ring finger is preferable";
# "The fifth finger must not be punctured") -----------------------------------------------------------------------------
SITE_FINGERS = ("ring",)
SITE_S = (0.008, 0.015)          # m back from the apex [EST: the shaded crescents of the BD figure lie on the distal pad;
#                                  CHOICE: from 8 mm, past the pad's radius, so the skin's normal has no along-finger lean]
SITE_ANG = (math.radians(15), math.radians(60))   # from the pad's middle line, either side [EST: the BD figure's two
#                                  crescents lie on the pad's two sides, off its middle]
ON_SKIN = 0.0015                 # m: the shield's face within this of a finger's surface is on it
TILT_MAX = math.radians(10)      # the lancet's axis within this of the skin's local normal is square [CHOICE]
SLIDE_MAX = 0.0010               # m: the face moving more on the skin from first touch to firing is a slide [CHOICE]

MATERIALS = {"default": (1.2, 1.0),
             "plastic": (0.5, 0.4),
             "housing": (1.2, 1.0),       # the lancet's housing where the gripper holds it. WHY not "plastic": at 0.5/0.4
             #                              the right's grip let the housing slide up its pads at 5 to 6 N in PhysX, under
             #                              the 8 N fire, so the press slid on the skin (t00 seeds 126748, 84277; c00) and
             #                              never fired in the stand (f00, 5.3 N peak) [RUN, 2026-11-20]. 1.2/1.0 is
             #                              yam_arena's GRIPPY_FRICTION for grasped objects; BD's housing has moulded grip
             #                              ribs, and check 7 requires the real grip to hold 15 N [CHOICE]
             "skin": (0.5, 0.4),          # #1837: hand friction 0.5/0.4
             "trainer": (1.0, 0.9),       # the ring finger's silicone skin [CHOICE: silicone on a gripper pad]
             "foam": (1.0, 0.9),
             "rubber": (1.0, 0.9)}
WHITE, PINK, SKIN = (0.93, 0.93, 0.93), (0.90, 0.45, 0.60), (0.86, 0.70, 0.60)
RED, DARK, FOAM, GREY = (0.78, 0.10, 0.10), (0.10, 0.10, 0.11), (0.35, 0.40, 0.50), (0.55, 0.55, 0.57)


def _box(lo, hi, color, material="plastic", collide=True, mass=0.0):
    lo, hi = np.minimum(lo, hi).astype(float), np.maximum(lo, hi).astype(float)
    return Shape("box", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 collide=collide, mass=mass)


def _cyl(c, r, h, color, axis="Z", material="plastic", collide=True):
    return Shape("cylinder", tuple(float(v) for v in c), (float(r), float(h)), color, axis=axis, material=material,
                 collide=collide)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


R_HAND = _ry(PITCH)              # the hand frame in the mechanism's: x toward the wrist tipped down by PITCH
AXIS = R_HAND @ np.array([1.0, 0.0, 0.0])   # the fingers' axis (apex to knuckle) in the mechanism frame
NORMAL = R_HAND @ np.array([0.0, 0.0, 1.0])  # out of the palm (the pads' middle line) in the mechanism frame


def hand_to_mech(p):
    """A hand-frame point (u, v, w) in the hand mechanism's frame (origin at the ring's apex, z up, x horizontal
    toward the wrist): the hand turned by PITCH about y, so its fingers slope down toward their tips."""
    u, v, w = (float(t) for t in p)
    return R_HAND @ np.array([u - APEX[0], v - APEX[1], w])


def mech_to_hand(p):
    q = R_HAND.T @ np.asarray(p, float)
    return np.array([q[0] + APEX[0], q[1] + APEX[1], q[2]])


def _rod_hull(a, ra, b, rb, color, material="skin", n=16):
    """A tapered rod from a to b (hand frame) as a hull of two rings, put in the mechanism frame."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = (b - a) / np.linalg.norm(b - a)
    e1 = np.cross(d, [0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.cross(d, [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(d, e1)
    # WHY 1/cos(pi/n): the ring's flats, not its corners, lie on the radius, so the rod is no thinner than the finger
    k = 1.0 / math.cos(math.pi / n)
    pts = [tuple(hand_to_mech(c + k * r * (math.cos(2 * math.pi * i / n) * e1 + math.sin(2 * math.pi * i / n) * e2)))
           for c, r in ((a, ra), (b, rb)) for i in range(n)]
    return hull(pts, color=color, material=material)


def _ball(c, r, color, material="skin"):
    """A sphere (a capsule with no straight part) at hand-frame point c."""
    return Shape("capsule", tuple(float(t) for t in hand_to_mech(c)), (float(r), 1e-5), color, axis="X",
                 material=material)


KNUCKLE_S = FINGERS["ring"][2] - APEX[0]           # 0.099 m: the ring's knuckle back from its apex, along its axis
WEB_X = (0.106, 0.112)           # the slide's web (mechanism x), fused under the end of the ring's printed base. WHY
#                                  behind the knuckle: the holding pads, closed round the finger from the side, reach
#                                  16 mm along it either side of the hold, to 102 mm (HOLD_AT 86 mm); the shelled tips,
#                                  below the finger, pass the web's front 5.5 mm off [RUN: tools/grippers.py]
RING_END_S = (WEB_X[1] - FINGERS["ring"][4] * abs(math.sin(PITCH))) / math.cos(PITCH)   # the ring's printed base
#                                  runs on past its knuckle over the web. WHY short of WEB_X[1] on its axis: its end is
#                                  square to the pitched axis, so its far edge reaches WEB_X[1], 3 mm clear of the rail


def _finger_parts(name, material="skin"):
    """A finger in the mechanism frame: a ball at its tip, its distal and proximal segments as rods, and a ball at its
    knuckle (the ring runs on to the web instead)."""
    v, u0, u1, rd, rp = FINGERS[name]
    parts = (_ball((u0 + rd, v, 0.0), rd, SKIN, material),
             _rod_hull((u0 + rd, v, 0.0), rd, (u0 + DIP, v, 0.0), rd, SKIN, material),
             _rod_hull((u0 + DIP, v, 0.0), rp, ((RING_END_S + APEX[0]) if name == "ring" else u1, v, 0.0), rp, SKIN,
                       material))
    return parts if name == "ring" else parts + (_ball((u1, v, 0.0), rp, SKIN, material),)


def hand_shapes():
    """The trainer's fixed parts in the mechanism frame: the index finger, the palm block and the thumb."""
    out = list(_finger_parts("index"))
    palm = [tuple(hand_to_mech((u, v, w))) for w in (-PALM_W, PALM_W) for u, v in
            ((PALM_U[0], PALM_V[0]), (PALM_U[1], PALM_V[0]), (PALM_U[1], PALM_V[1]), (PALM_U[0], PALM_V[1]))]
    out.append(hull(palm, color=SKIN, material="skin"))
    # WHY a rod from the index's knuckle into the palm block: the block starts 31 mm behind the knuckle, and the rig's
    # index finger is held up by this rod (rig.scad)
    v, _, u1, _, rp = FINGERS["index"]
    out.append(_rod_hull((u1 - 0.004, v, 0.0), rp, (PALM_U[0] + 0.006, v, 0.0), rp, SKIN))
    (tv, tu, tr), (bv, bu, br) = THUMB
    out.append(_rod_hull((tu, tv, 0.0), tr, (bu, bv, 0.0), br, SKIN))
    return tuple(out)


HAND_SHAPES = hand_shapes()
RING_SHAPES = _finger_parts("ring", "trainer")
HOLD_SEG = (DIP, KNUCKLE_S)      # m along the ring's axis from its apex: where a holding TCP counts as on the ring
#                                  finger, from its distal joint to its knuckle (the real trial's inked lines)
HOLD_R = 0.015                   # m: and at most this far from the finger's axis
HOLD_ACROSS = 0.004              # m: and at most this far off it along the gripper's closing direction [CHOICE]. WHY:
#                                  closed on the 15.6 mm finger, both pads touch it only with the finger between them
HOLD_GAP_ON = 0.00790            # m: the half-gap the left reads closed on the ring finger [RUN: PhysX traces of the
#                                  demo, 2026-11-19/20: 7.84 to 7.97 mm (lf) from the touch until 0.5 s after the click
#                                  over four layouts; 0.1 mm over the finger's 7.8 mm radius, so the reading is not the
#                                  pads' distance from the axis]
HOLD_GAP = (HOLD_GAP_ON - 0.0015, HOLD_GAP_ON + 0.0003)   # m: from 1.5 mm closer (a squeeze, a thinner print) to
#                                  0.3 mm wider than that reading. WHY the cap near it: the reading closed on the finger
#                                  is its widest; wider, the pads are off the skin (review of e233a08: 8.6 and 8.7 mm
#                                  used to pass with the pads up to 0.9 mm off). WHY 0.3 and not 0.15: the readings
#                                  spread 0.07 mm either side of 7.90 across layouts, and 0.15 left 0.11 mm [RUN]
# the slide (rig.scad; README Real trials): a miniature linear guide behind the ring finger's knuckle. Its rail is
# screwed to the front of the palm block's post; its block carries a web printed with the finger; two springs on fixed
# pins push the web's foot up against collars at the pins' tops (the top stop), and the foot lands on a stop post 10 mm
# down (the lower stop). WHY a rolling guide and not pins in printed holes: the press is 100 mm ahead of the slide, and
# plain pins in bores jam at offsets over L / (2 mu) (16 mm at L 8 mm, mu 0.25); a preloaded ball block does not wedge,
# and its rated pitch moment covers the press (README Real trials, the slide). Mechanism frame, m.
RAIL = {"part": "LILY MGN7C1R40Z0CM", "rail_len": 0.040, "rail_w": 0.007, "rail_h": 0.0048, "block_l": 0.0225,
        "block_w": 0.017, "height": 0.008, "B": 0.012, "C": 0.008, "C0": 1240.0, "MR": 4.7, "MP": 2.84, "MY": 2.84}
#                                  [SOURCE: LILY Bearing's product page for MGN7C1R40Z0CM ("Interchangeable With:
#                                  HIWIN"), read 2026-11-19: rail 40 mm, WR 7, HR 4.8, block L 22.5, W 17, H 8, H1 1.5,
#                                  mounting holes B 12 x C 8 (M2 x 2.5), C0 1.24 kN, MR 4.7, MP 2.84, MY 2.84 N-m]
SLIDE_Z = -WEB_X[0] * math.tan(PITCH)               # the ring's axis over the web's front face (z from the apex)
WEB_HALF_Y = 0.005               # the web's half-width [CHOICE]
BLOCK_Z = (SLIDE_Z - 0.0415, SLIDE_Z - 0.019)       # the block at rest (22.5 mm). WHY its top 19 mm under the axis: at
#                                  the lower stop the finger's base, 7.8 mm round its axis, keeps clear of the rail
RAIL_Z = (BLOCK_Z[1] - RAIL["rail_len"], BLOCK_Z[1])   # the rail, its top level with the block's at rest
POST_X = (WEB_X[1] + RAIL["height"], 0.165)         # the palm block's post: the rail is screwed to its front face
POST_Y = (-0.020, 0.050)         # and across (the palm block spans -0.028 to 0.058) [CHOICE]
FOOT_Z = (BLOCK_Z[0], BLOCK_Z[0] + 0.005)           # the web's foot, across the spring pins
SPRING_Y = 0.015                 # the spring pins either side of the axis, at the web's middle in x [CHOICE]
GUIDE_TOP = FOOT_Z[0] - DROP_STOP                   # the lower stop post's top: the foot lands on it
SPRING_REST = 0.0185             # m: each slide spring's length at rest, seat to foot [CHOICE]
PILLAR_TOP = FOOT_Z[0] - SPRING_REST - 0.002        # the seat pillar's top, the pins pressed into it: the springs sit
#                                                     on two 1 mm shims there (rig.scad)
PILLAR_X, PILLAR_Y = 0.005, 0.019   # m: the pillar's half-size along x (round the pins) and across [CHOICE]
BOARD_T = 0.006                  # the base board's thickness (plywood) [CHOICE]
BOARD = (((0.080, 0.200), (-0.040, 0.110)), ((0.200, 0.420), (-0.020, 0.050)))   # its two parts (mechanism x, y): a
#                                  plate under the hand, and a bar running back between the arm bases, where two
#                                  C-clamps hold it to the table. WHY it ends at x 0.42: at every corner of the layout
#                                  box its corners stay 14 mm or more inside the table's rear edge (world x -0.19) and
#                                  13 mm or more clear of the top camera stand's foot (test_board_on_the_table) [CHOICE]


def _palm_bottom(x):
    """The palm block's underside height (mechanism z) at mechanism x."""
    s = (x + PALM_W * math.sin(PITCH)) / math.cos(PITCH)
    return -s * math.sin(PITCH) - PALM_W * math.cos(PITCH)


POST_TOP = max(_palm_bottom(POST_X[0]), _palm_bottom(POST_X[1]))   # WHY the higher end: the post runs up into the
#                                                                    sloped palm block over its whole top


def ring_target():
    """The slide spring's target (m above the stop): its force there is the preload plus the finger's weight."""
    return (RING_PRELOAD + RING_KG * GRAVITY) / K_RING


def support_shapes():
    """The base board on the table and the palm block's post on it (fixed, colliding)."""
    floor = -HZ
    board = tuple(_box((x[0], y[0], floor), (x[1], y[1], floor + BOARD_T), GREY) for x, y in BOARD)
    post = _box((POST_X[0], POST_Y[0], floor + BOARD_T), (POST_X[1], POST_Y[1], POST_TOP), GREY)
    return board + (post,)


def slide_fixed():
    """The slide's fixed parts on the base, drawn only: the rail on the post's front face, the lower stop post."""
    floor = -HZ
    xm = (WEB_X[0] + WEB_X[1]) / 2
    return (_box((POST_X[0] - RAIL["rail_h"], -RAIL["rail_w"] / 2, RAIL_Z[0]),
                 (POST_X[0], RAIL["rail_w"] / 2, RAIL_Z[1]), DARK, collide=False),
            _box((xm - PILLAR_X, -PILLAR_Y, floor + BOARD_T), (xm + PILLAR_X, PILLAR_Y, PILLAR_TOP), GREY,
                 collide=False),
            _box((WEB_X[0], -0.003, PILLAR_TOP), (WEB_X[1], 0.003, GUIDE_TOP), GREY, collide=False))


def ring_moving():
    """The slide's parts that move with the ring finger, drawn only: the web, its foot and the guide's block."""
    x0, x1 = WEB_X
    return (_box((x0, -WEB_HALF_Y, FOOT_Z[1]), (x1, WEB_HALF_Y, SLIDE_Z), SKIN, collide=False),
            _box((x0, -SPRING_Y - 0.0035, FOOT_Z[0]), (x1, SPRING_Y + 0.0035, FOOT_Z[1]), SKIN, collide=False),
            _box((x1, -RAIL["block_w"] / 2, BLOCK_Z[0]), (x1 + RAIL["height"] - 0.0015, RAIL["block_w"] / 2,
                 BLOCK_Z[1]), DARK, collide=False))   # the block (H1 1.5 mm clear of the rail's foot)


def hand_parts():
    """The base (the board, the palm block's post, the palm block, the index finger and the thumb; the slide's rail and
    lower stop, drawn), fixed to the world, and the ring finger on a vertical slide behind its knuckle (its web, foot
    and block drawn): a spring holds it up on its top stop with RING_PRELOAD to spare; pressed down, it gives way unless
    a gripper holds it. The joint and its stops carry the finger, as the linear guide and the springs do in the rig."""
    base = support_shapes() + slide_fixed() + HAND_SHAPES
    ring = Link("ring", RING_KG, RING_SHAPES + ring_moving())
    joint = Joint("ring", "prismatic", "ring", (WEB_X[0], 0.0, SLIDE_Z), "Z", -DROP_STOP, 0.0, stiffness=K_RING,
                  damping=0.5, target=ring_target())
    return Link("hand_base", BASE_KG, base), (ring,), (joint,)


def lancet_parts():
    """The lancet tip down: the housing (with its rear cap) and the contact shield on its spring."""
    base = (_box((-L_W / 2, -L_T / 2, SHIELD_OUT), (L_W / 2, L_T / 2, SHIELD_OUT + L_LEN), PINK, material="housing"),
            _box((-L_W / 2 + 0.001, -L_T / 2 - 0.0003, SHIELD_OUT + 0.012), (L_W / 2 - 0.001, L_T / 2 + 0.0003,
                 SHIELD_OUT + 0.022), WHITE, collide=False))   # the grip indentations, drawn
    shield = Link("shield", LANCET_KG[1], (_cyl((0.0, 0.0, 0.004), SHIELD_R, 0.008, WHITE),), com=(0.0, 0.0, 0.004))
    joint = Joint("shield", "prismatic", "shield", (0.0, 0.0, 0.0), "Z", 0.0, SHIELD_TRAVEL, stiffness=K_SHIELD,
                  damping=2.0, target=-F_PRE / K_SHIELD)
    return Link("lancet", LANCET_KG[0], base, com=(0.0, 0.0, SHIELD_OUT + L_LEN / 2)), (shield,), (joint,)


def bench_parts():
    """Fixed, in world x, y (z from the table): the stand with its turning socket (the tab in its floor), and the sharps
    container (walls, floor, and a lid with its opening)."""
    sx, sy, h = STAND[0], STAND[1], STAND_H
    s = [_box((sx - STAND_HALF, sy - STAND_HALF, 0.0), (sx + STAND_HALF, sy + STAND_HALF, h), DARK)]
    cx, cy, w = SHARPS[0], SHARPS[1], SH_WALL
    x0, x1, y0, y1 = cx - SH_L / 2, cx + SH_L / 2, cy - SH_W / 2, cy + SH_W / 2
    s += [_box((x0, y0, 0.0), (x1, y1, w), RED),
          _box((x0, y0, 0.0), (x0 + w, y1, SH_H), RED), _box((x1 - w, y0, 0.0), (x1, y1, SH_H), RED),
          _box((x0, y0, 0.0), (x1, y0 + w, SH_H), RED), _box((x0, y1 - w, 0.0), (x1, y1, SH_H), RED)]
    ox, oy = SH_OPEN[0] / 2, SH_OPEN[1] / 2
    lid = SH_H - w
    s += [_box((x0, y0, lid), (cx - ox, y1, SH_H), RED), _box((cx + ox, y0, lid), (x1, y1, SH_H), RED),
          _box((cx - ox, y0, lid), (cx + ox, cy - oy, SH_H), RED), _box((cx - ox, cy + oy, lid), (cx + ox, y1, SH_H), RED)]
    base = Link("bench", 5.0, tuple(s))
    ix, iy, top = L_W / 2 + CUP_CLEAR, L_T / 2 + CUP_CLEAR, h + TAB_H + CUP_D
    cup = (_box((sx - ix, sy - iy, h), (sx + ix, sy + iy, h + TAB_H), WHITE),                       # the tab cap
           _box((sx - ix - CUP_WALL, sy - iy - CUP_WALL, h), (sx - ix, sy + iy + CUP_WALL, top), GREY),
           _box((sx + ix, sy - iy - CUP_WALL, h), (sx + ix + CUP_WALL, sy + iy + CUP_WALL, top), GREY),
           _box((sx - ix, sy - iy - CUP_WALL, h), (sx + ix, sy - iy, top), GREY),
           _box((sx - ix, sy + iy, h), (sx + ix, sy + iy + CUP_WALL, top), GREY))
    socket = Link("socket", 0.004, cup)
    joint = Joint("tab", "revolute", "socket", (sx, sy, h), "Z", -TAB_STOP, TAB_STOP, stiffness=K_TAB,
                  damping=0.002)
    return base, (socket,), (joint,)


_HB, _HL, _HJ = hand_parts()
# WHY fixed (no free root): the real base is clamped to the table, so the trainer cannot be dragged or tipped; the
# seed's layout moves the fixed root before each trial settles (place_layout)
HAND = mechanism("hand", (HAND0[0], HAND0[1], TOP + HZ), _HB, _HL, _HJ, MATERIALS, yaw=HAND_YAW, self_collide=False)
_LB, _LL, _LJ = lancet_parts()
LANCET = mechanism("lancet", L_START, _LB, _LL, _LJ, MATERIALS, self_collide=False, free=True)
_BB, _BL, _BJ = bench_parts()
BENCH = mechanism("bench", (0.0, 0.0, TOP), _BB, _BL, _BJ, MATERIALS, self_collide=False)
OBJECTS = [BENCH, HAND, LANCET]

# -- the layout by seed: the hand's base shifted and turned on the table, drawn continuously from the trial's seed ---------
SHIFT_MAX = 0.012                # m: the base's shift in x and in y, each uniform in +-SHIFT_MAX
TURN_C = 0.140                   # m behind the ring's apex, level along the hand: the point the turn is about (the
#                                  palm's middle)
TURN_MAX = math.radians(6.0)     # its turn, uniform in +-TURN_MAX. WHY these bounds: both arms'
#                                  reach (tools/ikcheck.py) and the sharps clearance (tools/clearance.py) pass at all
#                                  eight corners of this box


def shift_turn(seed):
    """(dx, dy, yaw) of the hand's base for a trial. An int seed draws each uniform from random.Random(seed) (WHY
    Python's: its random() sequence for an int seed is stable across Python versions, numpy's Generator promises no
    such thing); None gives no shift; a (dx, dy, yaw) tuple is used as given (the offline tools' bound corners)."""
    if seed is None:
        return (0.0, 0.0, 0.0)
    if isinstance(seed, tuple):
        return tuple(float(v) for v in seed)
    r = random.Random(int(seed))
    u = lambda a: a * (2.0 * r.random() - 1.0)  # noqa: E731
    return (u(SHIFT_MAX), u(SHIFT_MAX), u(TURN_MAX))


def corners():
    """The eight corners of the layout box, for the offline reach and clearance checks and the tests."""
    return [(sx * SHIFT_MAX, sy * SHIFT_MAX, st * TURN_MAX) for sx in (-1, 1) for sy in (-1, 1) for st in (-1, 1)]


def layout(seed):
    """The hand mechanism's pose (its origin, the ring's apex, and its yaw): shifted (dx, dy), then turned about the
    point TURN_C behind the apex on the hand's axis."""
    dx, dy, turn = shift_turn(seed)
    yaw = HAND_YAW + turn
    cx, cy = HAND0[0] + TURN_C * math.cos(HAND_YAW), HAND0[1] + TURN_C * math.sin(HAND_YAW)
    return {"hand": (cx - TURN_C * math.cos(yaw) + dx, cy - TURN_C * math.sin(yaw) + dy, TOP + HZ, yaw)}


def _quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rot3(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _np(x):
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=np.float64)


def _torch_of(x):
    return getattr(x, "torch", x)


def _write_root(base, name, xyz, yaw):
    """Put an articulation's root at a world pose, still (the hand's fixed root is moved this way before each trial
    settles; it does not move after)."""
    import torch
    asset = base.scene[name]
    dev = _torch_of(asset.data.root_link_pose_w).device
    o = _torch_of(base.scene.env_origins)[0]
    pose = torch.tensor([[*xyz, *_quat(yaw)]], dtype=torch.float32, device=dev)
    pose[:, :3] += o
    zero = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    if hasattr(asset, "write_root_pose_to_sim_index"):  # Isaac Lab 3.0
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=zero)
    else:
        asset.write_root_state_to_sim(torch.cat([pose, zero], dim=1))
    asset.write_data_to_sim()


@on_place
def place_layout(env):
    """The trial's layout: the hand's base shifted and turned by seed (before the reset settles)."""
    x, y, z, yaw = layout(run_seed())["hand"]
    _write_root(env.unwrapped, "hand", (x, y, z), yaw)


# -- geometry the model and the scorer share ----------------------------------------------------------------------------
def shield_force(q):
    """The push on the shield (N) from its travel: off its stop, the preload plus the spring; on it, 0."""
    return F_PRE + K_SHIELD * q if q > Q_TOUCH else 0.0


def face_w(l_pos, l_quat, q=0.0):
    """The shield's face centre (world), the shield pushed in by q."""
    return np.asarray(l_pos, float) + _rot3(l_quat) @ np.array([0.0, 0.0, q])


def tcp_over_lancet():
    """The TCP over the lancet's frame when held: pad tips 18 mm up the housing (clear of the socket's rim)."""
    return PAD_TIP_L + TIP


def ring_drop(ring_q):
    """How far (m) the ring finger has been pushed down its slide (ring_q, the slide's position, is 0 on its top stop
    and negative down)."""
    return max(0.0, -float(ring_q))


TIP_MARK = 0.0115                # m back from the ring's apex: the fingertip's mark the real trial reads "still" at, on
#                                  the finger's axis beside the site (README Real trials) [CHOICE: the site's middle]


def tip_height(ring_pos, ring_quat, base_pos, base_quat):
    """The fingertip mark's height (m) in the base's frame. WHY: the real trial reads "still" at the fingertip as well
    as at the web (a slide that pitched would drop the tip more), so the sim reads the same point."""
    p = _rot3(ring_quat) @ (TIP_MARK * AXIS) + np.asarray(ring_pos, float) - np.asarray(base_pos, float)
    return float((_rot3(base_quat).T @ p)[2])


def site_of(p_mech):
    """Where a point (the ring link's frame, the mechanism's frame at rest) touches the trainer: the nearest finger's
    distal pad as (finger, s from its apex, ang (the angle round) from the pad's middle line, the skin's inward normal
    there in the mechanism frame, distance off the surface), or (part, ...) for the palm, or None off the hand."""
    h = mech_to_hand(p_mech)
    best = None
    for name, (v, u0, u1, rd, rp) in FINGERS.items():
        end = RING_END_S + APEX[0] if name == "ring" else u1
        for seg, (a, b, r) in (("pad", (u0 + rd, u0 + DIP, rd)), ("finger", (u0 + DIP, end, rp))):
            u = min(max(h[0], a), b)
            rho = np.array([h[0] - u, h[1] - v, h[2]])
            d = float(np.linalg.norm(rho)) - r
            if best is None or d < best[4]:
                n_in = -(R_HAND @ (rho / max(np.linalg.norm(rho), 1e-9)))
                ang = math.atan2(float(rho[1]), float(rho[2]))
                best = (name if seg == "pad" else name + " (not the fingertip)", float(h[0] - u0), ang, n_in, d)
    if best[4] <= ON_SKIN:
        return best
    for part, lo, hi in (("palm", (PALM_U[0], PALM_V[0], -PALM_W), (PALM_U[1], PALM_V[1], PALM_W)),):
        if all(lo[i] - ON_SKIN <= h[i] <= hi[i] + ON_SKIN for i in range(3)):
            return (part, 0.0, 0.0, np.array([0.0, 0.0, -1.0]), 0.0)
    return None


def _on_ring(site):
    """The shield's face on the ring finger's skin (its pad or the segment behind it)."""
    return site is not None and site[0].startswith("ring")


def site_ok(site):
    """(ok, why not): either side of the ring fingertip's pad, SITE_ANG round from its middle line."""
    if site is None:
        return False, "not on the hand"
    name, s, ang = site[0], site[1], abs(site[2])
    if name not in SITE_FINGERS:
        return False, f"on the {name}, not the side of the ring fingertip"
    if not (SITE_S[0] <= s <= SITE_S[1]):
        return False, f"{1000 * s:.1f} mm from the {name}'s apex, off the pad"
    if ang < SITE_ANG[0]:
        return False, f"on the middle of the {name}'s pad, not its side"
    if ang > SITE_ANG[1]:
        return False, f"too far round the {name}'s side ({math.degrees(ang):.0f} deg)"
    return True, ""


def tilt_of(l_quat, hand_quat, n_in_mech):
    """The angle (rad) between the lancet's pointing direction and the skin's inward normal, both in the world."""
    point = -_rot3(l_quat)[:, 2]
    n = _rot3(hand_quat) @ np.asarray(n_in_mech, float)
    return math.acos(max(-1.0, min(1.0, float(point @ n))))


def in_sharps(l_pos):
    """The lancet's frame inside the container: within its walls and under its lid."""
    p = np.asarray(l_pos, float)
    return bool(abs(p[0] - SHARPS[0]) <= SH_L / 2 - SH_WALL and abs(p[1] - SHARPS[1]) <= SH_W / 2 - SH_WALL
                and TOP + SH_WALL - 0.002 <= p[2] <= TOP + SH_H - SH_WALL)


def in_socket(l_pos):
    """The lancet's frame still in the stand. Leaving it latches the model's "out" (advance), so a lancet put back
    in the stand still counts as out: let go there, it is a drop."""
    p = np.asarray(l_pos, float)
    return bool(math.hypot(p[0] - L_START[0], p[1] - L_START[1]) <= OUT_XY and p[2] - L_START[2] <= LIFT_OUT)


def grip_point(l_pos, l_quat):
    return np.asarray(l_pos, float) + _rot3(l_quat) @ np.array([0.0, 0.0, tcp_over_lancet()])


HELD_GAP = (min(L_W, L_T) / 2 - 0.002, max(L_W, L_T) / 2 + 0.0005)
# m: a finger half-gap in this range is closed on the housing. WHY the top is the wider side: a grasp across the 13 mm
# width is as valid as one across the 10 mm thickness, and the thickness alone read it as let go, so as a drop. WHY a
# bottom: a gripper closed on nothing (its half-gap at the 1 mm the demo commands) is not holding the lancet; closed
# on the 10 mm side the sim read 3.9 to 4.8 mm [RUN, trace of the right finger, 2026-11-17].


HELD_AX = (-0.010, SHIELD_OUT + L_LEN + 0.013)   # m along the lancet from its face: where a holding TCP can sit, from a
#                                  grasp with the pad tips 23 mm past the face to one with the tips at its rear end (the
#                                  tips are 13 mm under the TCP)
HELD_R = 0.015                   # m: and this far off the lancet's axis (the housing can sit anywhere across the pads)
RELEASE_R = 0.060                # m: a gripper not open whose TCP is nearer the lancet than this has not let it go


def tcp_off_axis(hand, l_pos, l_quat):
    """(distance off the lancet's axis segment HELD_AX, position along it) of a hand's TCP."""
    a = np.asarray(hand[0], float) - np.asarray(l_pos, float)
    ax = _rot3(l_quat)[:, 2]
    t = float(a @ ax)
    tc = min(max(t, HELD_AX[0]), HELD_AX[1])
    return float(np.linalg.norm(a - tc * ax)), t


def held_by(hand, l_pos, l_quat):
    """A hand closed on the lancet: its finger half-gap in HELD_GAP and its TCP within HELD_R of the lancet's axis
    segment HELD_AX. WHY the segment and not one grip point: a grasp slipped along the housing, or taken deeper or
    higher, still holds it."""
    if hand is None or hand[0] is None or hand[1] is None:
        return False
    gap = -float(hand[1])
    return bool(HELD_GAP[0] <= gap <= HELD_GAP[1] and tcp_off_axis(hand, l_pos, l_quat)[0] <= HELD_R)


def let_go(hand, l_pos, l_quat):
    """A hand has let the lancet go: its fingers open past HELD_GAP, or its TCP more than RELEASE_R from the lancet's
    axis segment. WHY on top of held_by: a gripper closed tight next to the lancet (under HELD_GAP, say on the shield)
    may still hold it."""
    if hand is None or hand[0] is None or hand[1] is None:
        return True
    return bool(-float(hand[1]) > HELD_GAP[1] or tcp_off_axis(hand, l_pos, l_quat)[0] > RELEASE_R)


def finger_held(hand, ring_pos, ring_quat):
    """A gripper closed on the ring finger: (TCP, finger reading, the gripper's quat). Its half-gap in HOLD_GAP (what
    the pads read closed on the finger), its TCP beside the finger's axis between the distal joint and the knuckle
    (HOLD_SEG), at most HOLD_ACROSS off it along the closing direction (the gripper's x axis) and HOLD_R from it, and
    its pad tips (TIP past the TCP along its approach, the gripper's z axis) past the axis. WHY a rule as well as the
    slide: a jab can fire before a light unheld finger moves, and the rule is what a real trial's camera sees.
    WHY each part: a gripper closed on air beside the finger reads a smaller gap, sits off the axis along its closing
    direction, or stops short of it with its tips."""
    if hand is None or len(hand) < 3 or any(v is None for v in hand[:3]):
        return False
    gap = -float(hand[1])
    if not HOLD_GAP[0] <= gap <= HOLD_GAP[1]:
        return False
    Rr = _rot3(ring_quat)
    tcp = np.asarray(hand[0], float)
    s = float((Rr.T @ (tcp - np.asarray(ring_pos, float))) @ AXIS)
    if not HOLD_SEG[0] <= s <= HOLD_SEG[1]:
        return False
    d = tcp - (np.asarray(ring_pos, float) + Rr @ (s * AXIS))
    Rg = _rot3(hand[2])
    across = abs(float(d @ Rg[:, 0]))
    back = -float(d @ Rg[:, 2])      # the axis this far past the TCP along the approach
    return bool(across <= HOLD_ACROSS and back < TIP and float(np.linalg.norm(d)) <= HOLD_R)


# -- the model -----------------------------------------------------------------------------------------------------------
def _fresh_model(seed=None):
    return {"t": 0.0, "tab_off": False, "out": False, "fault": None, "press": None, "max_drop": 0.0, "touch_t": None,
            "fire_t": None, "unheld_t": None, "tip0": None, "positioned": False, "fired": False, "fire": None, "free_t": 0.0, "q": 0.0, "ring_q": 0.0,
            "tab": 0.0,
            "right": None, "left": None, "steps": 0, "error": None, "layout": shift_turn(seed), "base0": None}


_M = _fresh_model()


def _latch(m, why):
    if not m["fault"]:
        m["fault"] = why


def advance(m, dt, l_pos, l_quat, q, tab, ring_q, ring_pos, ring_quat, right, left=None, base=None):
    """One step: the tab's break, the lancet out of the socket, the press on the skin (where, how square, slid or not,
    the ring finger held and still or not), the firing, and the used lancet's fate. A hand is (TCP, finger reading,
    gripper quat); base, the trainer's base pose (pos, quat), adds the fingertip mark's drop to the slide's."""
    m["t"] += dt
    m["q"], m["tab"], m["ring_q"] = float(q), float(tab), float(ring_q)
    m["right"], m["left"] = right, left
    if abs(tab) >= TAB_BREAK:
        m["tab_off"] = True
    if not m["out"] and not in_socket(l_pos):
        m["out"] = True
        if not m["tab_off"]:
            _latch(m, "lancet lifted out of the stand with its tab still on")
    f = shield_force(q)
    Rr = _rot3(ring_quat)
    face = face_w(l_pos, l_quat, q)
    p_mech = Rr.T @ (face - np.asarray(ring_pos, float))
    if m["tab_off"] and not m["out"] and not m["fired"] and f >= F_FIRE:
        # WHY: the real lancet, its tab off, fires against whatever its shield is pressed on, the stand's post too
        m["fired"], m["fire_t"] = True, m["t"]
        m["fire"] = {"site": None, "tilt": math.pi, "drop": 0.0, "slide": 0.0, "held": False}
        _latch(m, "fired in the stand (pressed down on the post after the twist)")
    site = site_of(p_mech)
    on_ring = _on_ring(site) and (site[4] <= TOUCH_D or f >= F_TOUCH)
    # WHY on the ring finger only: a touch on the table, the plate or another finger is not the fingerstick's start, and
    # a real trial reads the touch where the camera sees the shield meet the ring finger
    if m["out"] and m["tab_off"] and on_ring and m["touch_t"] is None:
        m["touch_t"] = m["t"]
    drop = ring_drop(ring_q)
    if base is not None:
        h = tip_height(ring_pos, ring_quat, base[0], base[1])
        if m["tip0"] is None:      # WHY the first step: the finger starts on its top stop
            m["tip0"] = h
        drop = max(drop, m["tip0"] - h)
    # WHY from first touch to STEADY_AFTER past the click, whatever the force: a jab fires before the finger moves
    window = m["touch_t"] is not None and (m["fire_t"] is None or m["t"] <= m["fire_t"] + STEADY_AFTER + 1e-9)
    if window:
        m["max_drop"] = max(m["max_drop"], drop)
        if m["max_drop"] > DROP_MAX:
            when = "within 0.5 s of the click" if m["fired"] else "under the press"
            _latch(m, f"the ring finger was not held still: it gave {1000 * m['max_drop']:.1f} mm {when}")
    # WHY every hand, the one held_by calls the lancet's too: the two half-gap windows overlap, so a gripper closed on
    # the finger near the press can read as holding the lancet as well; dropping it scored a real hold 0 (review of
    # e233a08). A hand that holds the lancet and not the finger is not finger_held anyway
    held = any(finger_held(h, ring_pos, ring_quat) for h in (right, left))
    if window and not held and m["unheld_t"] is None:
        m["unheld_t"] = m["t"]
    if m["out"] and m["tab_off"] and not m["fired"] and f >= F_TOUCH:
        if m["press"] is None:
            m["press"] = p_mech.copy()
        slide = float(np.linalg.norm(p_mech - m["press"]))
        if slide > SLIDE_MAX:
            _latch(m, f"the press slid {1000 * slide:.1f} mm on the skin")
        ok, why = site_ok(site)
        square = site is not None and tilt_of(l_quat, ring_quat, site[3]) <= TILT_MAX
        if ok and square and held and m["unheld_t"] is None and m["max_drop"] <= DROP_MAX:
            m["positioned"] = True
        if f >= F_FIRE:
            m["fired"], m["fire_t"] = True, m["t"]
            tilt = tilt_of(l_quat, ring_quat, site[3]) if site is not None else math.pi
            m["fire"] = {"site": None if site is None else site[:3], "tilt": tilt, "drop": m["max_drop"],
                         "slide": slide, "held": held}
            if not ok:
                _latch(m, "fired " + why)
            elif tilt > TILT_MAX:
                _latch(m, f"fired tilted {math.degrees(tilt):.0f} deg from square to the skin")
            elif not held:
                _latch(m, "fired with no gripper holding the ring finger")
    elif f < F_TOUCH:
        m["press"] = None
    # WHY at the click and not at the first unheld step: a finger let go of gives way, and "not held still" then says so
    if m["fired"] and m["unheld_t"] is not None:
        _latch(m, f"the ring finger was not held from first touch to {STEADY_AFTER:g} s after the click (let go "
                  f"{m['unheld_t'] - m['touch_t']:.2f} s after the touch)")
    if m["out"] and not in_sharps(l_pos) and not held_by(right, l_pos, l_quat) and not held_by(left, l_pos, l_quat):
        m["free_t"] += dt
        if m["free_t"] > DROP_S:
            _latch(m, "lancet dropped outside the sharps container")
    else:
        m["free_t"] = 0.0


DROP_S = 0.5                     # s loose outside the stand and the container is a drop (the fall into it takes 0.2 s)

_IDS = {}
_CLOCK = {"dt": None, "steps": 0}
TCP_OFFSET = (0.0, 0.0, 0.134)   # yam_arena.constants: the TCP along the gripper's approach axis


def _ids(asset, key, names):
    if key not in _IDS:
        all_names = list(asset.joint_names)
        _IDS[key] = {j: all_names.index(j) for j in names}
    return _IDS[key]


@on_reset
def reset_model():
    reset_memory(run_seed())


@on_step
def run_model(env):
    """Read the lancet and its shield, the socket's turn, the ring finger's slide and link, and both hands; advance the
    model."""
    try:
        base = env.unwrapped
        lan, hand, bench = base.scene["lancet"], base.scene["hand"], base.scene["bench"]
        origin = _np(base.scene.env_origins)[0]
        pl = _np(lan.data.root_link_pose_w)[0]
        q = float(_np(lan.data.joint_pos)[0][_ids(lan, "lancet", ("shield",))["shield"]])
        tab = float(_np(bench.data.joint_pos)[0][_ids(bench, "bench", ("tab",))["tab"]])
        ring_q = float(_np(hand.data.joint_pos)[0][_ids(hand, "hand", ("ring",))["ring"]])
        ph = _np(hand.data.body_link_pose_w)[0][list(hand.body_names).index("ring")]
        hands = []
        for side in ("right", "left"):
            rob = base.scene[f"{side}_robot"]
            g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
            qr = _np(rob.data.joint_pos)[0]
            hands.append((g[:3] - origin + _rot3(g[3:7]) @ np.array(TCP_OFFSET),
                          float(qr[list(rob.joint_names).index("joint7")]), g[3:7].copy()))
        if _M["base0"] is None:
            _M["base0"] = _np(hand.data.root_link_pose_w)[0][:3] - origin
        pb = _np(hand.data.root_link_pose_w)[0]
        _M["base"] = pb[:3] - origin
        advance(_M, float(base.step_dt), pl[:3] - origin, pl[3:7], q, tab, ring_q, ph[:3] - origin, ph[3:7], hands[0],
                hands[1], (pb[:3] - origin, pb[3:7]))
        _M["steps"] += 1
    except Exception as e:  # WHY: never stop the run; no model step means no firing and so no full score
        _M["error"] = "step " + repr(e)


@on_step
def read_clock(env):
    try:
        _CLOCK["dt"] = float(env.unwrapped.step_dt)
    except Exception:  # WHY: never stop the run; no step length means no hold, so no full score
        _CLOCK["dt"] = None
    _CLOCK["steps"] += 1


# -- scoring -------------------------------------------------------------------------------------------------------------
LOST_Z = TOP - 0.030
BASE_MOVED = 0.005               # m: the base this far from its layout pose is a fault
BASE_TURN = math.radians(2.0)    # its turn or tilt off its layout pose past this is a fault too [CHOICE]. WHY kept with
#                                  a clamped, fixed base: it checks the sim placed the base by seed (place_layout); a
#                                  base that never moved from its spawn pose fails here on the first step


def failure(objects, m):
    """Latched faults: the model's (tab on when lifted, a slide, a bad fire, a drop), the trainer's base off its
    layout pose, the lancet off the table, or the model not running."""
    if m["fault"]:
        return m["fault"]
    b = objects["hand"]
    hx, hy, hz, yaw0 = layout(m["layout"])["hand"]   # m["layout"] is the trial's (dx, dy, yaw)
    Rb = _rot3(b.quat_xyzw)
    turn = (math.atan2(float(Rb[1, 0]), float(Rb[0, 0])) - yaw0 + math.pi) % (2 * math.pi) - math.pi
    if (float(np.linalg.norm(np.asarray(b.pos, float) - np.array([hx, hy, hz]))) > BASE_MOVED or abs(turn) > BASE_TURN
            or math.acos(max(-1.0, min(1.0, float(Rb[2, 2])))) > BASE_TURN):
        return "trainer's base off its layout pose"
    if float(objects["lancet"].pos[2]) < LOST_Z:
        return "lancet lost"
    if m["error"]:
        return "model " + m["error"]
    if _CLOCK["steps"] - m["steps"] > 5:
        return "model not stepping"
    return None


def stage(objects, m):
    """0 nothing; 1 tab off and the lancet out; 2 and fired square on a good site, the ring finger held and still; 3 and
    the used lancet in the sharps container, let go by both hands (let_go). WHY no "placed" rung: a real trial reads
    the site and the square from the patch and the lean at the click, so a placement with no fire cannot be scored
    (m["positioned"] stays in the trace only)."""
    if m["fired"]:
        l = objects["lancet"]
        if (in_sharps(l.pos) and let_go(m["right"], l.pos, l.quat_xyzw)
                and let_go(m["left"], l.pos, l.quat_xyzw)):
            return 3
        return 2
    return 1 if (m["tab_off"] and m["out"]) else 0


def _fresh():
    return {"last": None, "fail": None, "now": None, "held": 0.0}


_MEM = _fresh()


def reset_memory(seed=None):
    _M.clear()
    _M.update(_fresh_model(seed))
    _MEM.clear()
    _MEM.update(_fresh())
    _CLOCK.update({"dt": None, "steps": 0})


def update(objects):
    """One control step: latch a failure and judge this step (same states dict for success() and score())."""
    if objects is _MEM["last"]:
        return _MEM
    _MEM["last"] = objects
    why = failure(objects, _M)
    if why and not _MEM["fail"]:
        _MEM["fail"] = why
    n = {"stage": stage(objects, _M)}
    n["done"] = bool(not _MEM["fail"] and n["stage"] == 3)
    _MEM["held"] = _MEM["held"] + (_CLOCK["dt"] or 0.0) if n["done"] else 0.0
    _MEM["now"] = n
    return _MEM


def success(objects):
    """Tab off, fired on a good site square, the ring finger held and still, without a slide, the used lancet let go in
    the sharps container (held 1 s by yam_arena), with no latched fault."""
    return update(objects)["now"]["done"]


LADDER = {0: 0.0, 1: 0.25, 2: 0.5}
HOLD_S = 1.0


def score(objects):
    """At the last step: 0 after a latched fault; success held 1 s 1.0; in the container, let go, not yet held 1 s 0.75;
    fired well 0.5; tab off and out 0.25."""
    m = update(objects)
    if m["fail"]:
        return 0.0
    n = m["now"]
    if n["stage"] == 3:
        return 1.0 if (n["done"] and m["held"] >= HOLD_S - 1e-9) else 0.75
    return LADDER[n["stage"]]


# -- demo ----------------------------------------------------------------------------------------------------------------
TIP = 0.013                      # PAD_TIP_PAST_TCP: the pad tips sit 13 mm past the TCP along its approach
PAD_TIP_L = 0.018                # the pad tips this far up the lancet from its frame (the socket's rim is at 10 mm)
OPEN_F = -0.040
GRIP_L = -(L_T / 2 - 0.004)      # 4 mm into the housing's 10 mm: 20 N a finger at 5000 N/m. WHY: at 2.5 mm the housing
#                                  slid up the pads at 7.9 N, under the 8 N fire (friction about 0.34) [RUN]; on the
#                                  0.5/0.4 plastic even 4 mm slid at 5 to 6 N, hence MATERIALS["housing"]
TWIST = math.radians(70)         # the demo turns the lancet this far in the socket (the tab breaks at 45 deg)
PRESS_DZ = 0.020                 # a stem's press aims this far past contact (the demo's, press_depth)
SITE_AIM = 0.0115                # the demo's site: 11.5 mm back from the ring's apex (the middle of 8 to 15)
SITE_TURN = math.radians(45.0)   # and 45 deg round from the pad's middle line, on the thumb's side (+v), so the lancet
#                                  comes in 48 deg from vertical [RUN: offline search, README Design]
PRE_PRESS = 0.045                # the right's TCP backs off this far along the lancet's axis before and after the press
HOLD_AT = 0.086                  # m back from the ring's apex: where the left holds it, 74.5 mm behind the press. WHY
#                                  not 90: with the fingertips lowered, the finger's underside leans toward the web, and
#                                  at 90 the left's shelled tips passed it 2.6 mm off [RUN: tools/grippers.py]
HOLD_BACK = 0.005                # the left's TCP this far short of the ring's axis along its approach: its pad tips 8 mm
#                                  past the axis, across the finger's widest
GRIP_R = -(FINGERS["ring"][4] - 0.005)   # 5 mm into the finger's 15.6 mm: 25 N a pad at 5000 N/m
OPEN_R = -0.020                  # the left's pads open to 20 mm either side of the finger (12 mm clear of its skin)
HOLD_UP = 0.025                  # the left lifts the finger against its top stop by aiming this far over it (about
#                                  12 N [EST: the arm's 0.5 N/mm]). WHY: the press's 10 N, 6.6 N of it straight down,
#                                  would sag an arm that only closed on the finger; pulled up on its stop, the finger
#                                  stays put until the press passes the pull
APPROACH_L = 0.030               # the left comes in along its approach from this far back
HOLD_TILT = math.radians(30.0)   # the left's approach tilted this far down from the palm's plane. WHY: level, its wrist
#                                  (joint4) is within 0.1 rad of its limit at the hold and reaches it at some layout
#                                  corners; at 30 deg every joint keeps 0.6 rad [RUN: IK at the eight corners,
#                                  2026-11-19]
RELEASE_Z = TOP + 0.205          # the TCP over the container's opening at release
STEADY = 0.8                     # s held over a target before each descent (the arm lags 2 to 10 mm on a descent [RUN:
#                                  a traced YAM descent])
PARK_L = (0.100, 0.080)          # the left's TCP parked this far back along its approach and up from its hold, in the
#                                  hold's tilt, while the right takes the lancet. WHY tilted: from a straight-down grip
#                                  its wrist has to turn over to reach the hold, and the IK stalls on joint6's limit
#                                  [RUN: 2026-11-19]
WAIT_R = (0.220, -0.280, TOP + 0.200)  # the right's TCP while the left takes and lets go of the finger [CHOICE:
#                                  tools/grippers.py; higher, joint4 meets its limit]
L_TAKE = 2.0 + 0.8 + 1.2 + 0.6 + 0.8   # s: the left from park to holding the finger up on its stop (plan)
L_FREE = 0.6 + 0.6 + 1.2 + 2.0   # s: the left from holding the finger to back at park (plan)


@dataclass
class TWaypoint(Waypoint):
    """A Waypoint with a full TCP rotation (rows of a 3 x 3 matrix) in place of the straight-down grip turned by yaw.
    Played by TiltPlan; the scripted policy is patched to use it (_replan_demo_after_reset). Demo only."""
    rot: tuple = None


def waypoint_rot(w):
    """A waypoint's TCP rotation: its own, or the straight-down grip turned by its yaw."""
    r = getattr(w, "rot", None)
    return np.asarray(r, float) if r is not None else grip_rotation(w.yaw)


class TiltPlan(ArmPlan):
    """yam_arena's ArmPlan (the same smoothstep in position and slerp in rotation, the same IK), with each waypoint's
    rotation from waypoint_rot, so a grip can tilt."""

    def target(self, t):
        prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
        for w in self.waypoints:
            goal, rot = np.asarray(w.pos, dtype=float), waypoint_rot(w)
            if t < w.duration:
                s = ik.smoothstep(t / w.duration)
                return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                        prev_finger + s * (w.finger - prev_finger), w.label)
            t -= w.duration
            prev_pos, prev_rot, prev_finger = goal, rot, w.finger
        return prev_pos, prev_rot, prev_finger, "done"


def _w(t, p, f, label, yaw=0.0, rot=None):
    return TWaypoint(float(t), tuple(float(v) for v in p), float(f), label, float(yaw),
                     None if rot is None else tuple(tuple(float(x) for x in r) for r in np.asarray(rot)))


class _Arm:
    def __init__(self):
        self.w, self.t = [], 0.0

    def add(self, dt, p, f, label, yaw=0.0, rot=None):
        self.w.append(_w(dt, p, f, label, yaw, rot))
        self.t += dt


def rot_from(a, c):
    """The TCP rotation with approach a (its z) and closing direction c (its x), as grip_rotation's columns are."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float) - a * float(np.dot(a, c))
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def hand_world(seed, p, direction=False):
    """A hand-frame point (or, with direction, a hand-frame direction) in the world for the trial's layout."""
    hx, hy, hz, yaw = layout(seed)["hand"]
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    if direction:
        return Rz @ (R_HAND @ np.asarray(p, float))
    return np.array([hx, hy, hz]) + Rz @ hand_to_mech(p)


PRESS_ROLL = -1                  # the right's fingers close along the ring finger at the press (0: across it).
#                                  WHY: a parallel grip holds a push on the lancet's face in two ways. A twist about the
#                                  line through both pads only meets the pads' torsional friction, small; one about any
#                                  other axis presses one pad's edge harder, held by the pad's normal force. Under the
#                                  push the wrist gives 0.27 deg/N about the TCP's axis across the finger, 0.09 deg/N
#                                  about the one along it [ARITH: J K^-1 J^T at every corner]. Closing across the
#                                  finger (the old pose) put the larger give on the pads' weak axis: at 7 to 8.4 N the
#                                  lancet rocked 2.5 mm in the grip and the face jumped 3 to 4 mm toward the apex (t00
#                                  corners 126748 and 100825, PhysX traces 2026-11-20) [RUN]; put back to 0 with the
#                                  other press changes, the face slid 2 mm and the press never reached 8 N at 2 of 3
#                                  seeds [RUN]. -1 and not +1: +1 takes joint 6 to 2.01 rad of its 2.09 [RUN: ikcheck]


def press_pose(seed, s=SITE_AIM, turn=SITE_TURN):
    """The demo's press: the site on the ring's pad (world), the lancet's pointing direction a (into the skin, along
    its inward normal), and the right's TCP rotation (its fingers closing along the finger, PRESS_ROLL)."""
    v, u0, _, rd, _ = FINGERS["ring"]
    out = np.array([0.0, math.sin(turn), math.cos(turn)])          # the skin's outward normal (hand frame)
    site = hand_world(seed, np.array([u0 + s, v, 0.0]) + rd * out)
    a = -hand_world(seed, out, direction=True)
    x = hand_world(seed, (1.0, 0.0, 0.0), direction=True)
    return site, a, rot_from(a, PRESS_ROLL * x if PRESS_ROLL else np.cross(x, a))


ARM_K = (80.0, 80.0, 80.0, 10.0, 10.0, 10.0)   # N m/rad: the YAM's joint stiffnesses (yam_arena.constants SHOULDER,
#                                  WRIST); the arm is a joint-space PD, so these set its compliance at the TCP
Q_PRESS = (0.1, 1.75, 1.27, -1.3, -0.82, 0.2)   # an IK start near the press pose at every corner [RUN: ikcheck]


def _press_give(seed, touch=None, rp=None):
    """The face's give per N of push on it along +a, at the press pose: its displacement (m/N) and the TCP's rotation
    (a world rotation vector, rad/N), from the arm's compliance J K^-1 J^T (K the joint stiffnesses). The skin pushes
    back along -a, so pressed with f N the face moves -f times these."""
    site, a, rot = press_pose(seed)
    if touch is None:
        touch = site - tcp_over_lancet() * a
    rot = rot if rp is None else rp
    base = station.BASES["right"]
    q, _, _ = ik.solve(touch, ik.matrix_to_quat_xyzw(rot), q_init=Q_PRESS, base_pos=base)
    lever = tcp_over_lancet()
    x0, r0 = ik.tcp_pose(q, base)
    f0, h = x0 + lever * r0[:, 2], 1e-6
    jac = np.zeros((6, 6))
    for i in range(6):
        dq = np.array(q, float)
        dq[i] += h
        x, r = ik.tcp_pose(dq, base)
        jac[:3, i] = (x + lever * r[:, 2] - f0) / h
        w = (r @ r0.T - np.eye(3)) / h
        jac[3:, i] = (w[2, 1], w[0, 2], w[1, 0])
    give = jac @ np.diag([1.0 / k for k in ARM_K]) @ jac[:3].T @ a
    return a, give[:3], give[3:]


def press_dir(seed, touch=None, rp=None):
    """The direction the right drives the press along so that, after its joints give under the press, the force on the
    skin is along the lancet's axis a: a plus the face's lateral compliance per N over its axial give per N (the arm's
    and the shield's). WHY: the arm's compliance at the lancet's face (J K^-1 J^T, K the joint stiffnesses) bends a push
    along a 0.5 to 0.8 of the way sideways [ARITH: 2026-11-20, every corner]. Pressed along a alone, the face is pinned
    on the skin while the arm pulls it sideways, so the lancet rocked in the grip and slid on the skin (t00 corners
    126748 and 100825, c00) [RUN]."""
    a, v, _ = _press_give(seed, touch, rp)
    axial = float(v @ a)
    return a + (v - axial * a) / (axial + 1.0 / K_SHIELD)


PRESS_F = (5.0, 9.0)             # N: the push the demo's load and press stages aim at. WHY 9: midway from the 8 N
#                                  fire to the shield bottoming at 9.8 N; past that the push goes into the finger after
#                                  the click, and the housing slid up the pads at 9 to 9.7 N (PhysX, 2026-11-20) [RUN]


def press_depth(seed, f, touch=None, rp=None):
    """How far past contact (along press_dir) the demo aims for a push of f N: the arm's axial give per N
    (_press_give, 1.14 to 1.55 mm/N over the layout's corners) and the shield's, in series. WHY per layout: one depth
    for all gave 8.0 to 9.6 N across the corners, and the softest would not fire with margin. Seed 0 predicts
    1.39 mm/N; PhysX settled at 5.8 N on 9 mm, 1.34 mm/N [RUN, 2026-11-20]."""
    a, v, _ = _press_give(seed, touch, rp)
    return f * float(v @ a) + max(0.0, f - F_PRE) / K_SHIELD


def press_rot(seed, rp, f, touch=None):
    """The right's TCP rotation commanded at a press stage that pushes f N: rp turned back by the turn the wrist gives
    under f, so that loaded the lancet stands at rp. WHY: the wrist gives 0.27 to 0.31 deg/N [ARITH: _press_give; RUN:
    the lancet's axis turned 2.4 to 2.6 deg from the touch to the click at 8 N, PhysX 2026-11-20], and at one layout
    that put the click 5.7 deg from square to the skin against the 10 deg limit [RUN]."""
    _, _, w = _press_give(seed, touch, rp)
    v = f * w    # WHY +: w is the turn per N of push along +a, and the skin pushes back along -a, so loaded the wrist
    #              turns by -f w; commanded ahead by +f w, it ends at rp [RUN: with -f w the lancet turned 4.1 to
    #              4.5 deg from the touch to the click, against 2.4 to 2.6 deg with no pre-turn, PhysX 2026-11-20]
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return rp
    k = v / th
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return (np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * K @ K) @ rp


def hold_pose(seed, s=HOLD_AT, tilt=None):
    """The demo's hold: the left's TCP (HOLD_BACK short of the ring's axis, s back from its apex), its approach a (from
    the hand's -v side, across the finger, tilted HOLD_TILT down from the palm's plane) and its TCP rotation (its pads
    closing across the finger, onto its palm and back sides)."""
    tilt = HOLD_TILT if tilt is None else tilt
    v, u0 = FINGERS["ring"][0], FINGERS["ring"][1]
    axis = hand_world(seed, (u0 + s, v, 0.0))
    a = hand_world(seed, (0.0, math.cos(tilt), -math.sin(tilt)), direction=True)
    c = np.cross(hand_world(seed, (1.0, 0.0, 0.0), direction=True), a)
    return axis - HOLD_BACK * a, a, rot_from(a, c)


def park_pose(seed):
    """The left's park (TCP, rotation): back along its hold's approach and up, in the hold's rotation."""
    g, a, r = hold_pose(seed)
    return g - PARK_L[0] * a + np.array([0.0, 0.0, PARK_L[1]]), r


def plan(delay=0.0, seed=None, twist=TWIST):
    """Both arms: the right takes the lancet, twists its tab off and lifts it out, and waits; the left comes in from
    the side, closes on the ring finger and holds it up on its stop; the right tilts the lancet onto the side of the
    fingertip's pad and presses along its axis until it fires, then backs out; the left lets go and parks; the right
    drops the lancet into the container."""
    R, L = _Arm(), _Arm()
    lg = np.array(L_START) + np.array([0.0, 0.0, tcp_over_lancet()])
    R.add(2.0 + delay, lg + [0, 0, 0.05], OPEN_F, "over the lancet")
    R.add(STEADY, lg + [0, 0, 0.05], OPEN_F, "steady over the lancet")
    R.add(1.5, lg, OPEN_F, "down to the lancet")
    R.add(0.6, lg, GRIP_L, "grip the housing")
    R.add(0.6, lg, GRIP_L, "settle")
    R.add(2.0, lg, GRIP_L, "twist the tab off", twist)
    R.add(0.5, lg, GRIP_L, "hold", twist)
    # WHY turned back before the lift: the socket turns on a spring (K_TAB), and lifted out at the twist's 62 to 70 deg
    # the socket sprang back while the housing was still in it and dragged the housing up to 2.6 mm across the pads,
    # so the press landed up to 19 deg round from its aim (PhysX, 2026-11-20) [RUN]
    R.add(1.0, lg, GRIP_L, "turn it back in the stand", 0.0)
    R.add(1.2, lg + [0, 0, 0.06], GRIP_L, "lift it out of the stand", 0.0)
    R.add(1.5, WAIT_R, GRIP_L, "out of the left's way", 0.0)   # WHY yaw 0 at the wait: from
    #                                  there the turn to the tilted press stays in reach; from the twist's 70 deg it left
    #                                  joint 4's range on the way (1 to 11 mm off at the corners) [RUN: tools/ikcheck.py]
    t_left = R.t
    site, a, rp = press_pose(seed)
    touch = site - tcp_over_lancet() * a
    R.add(L_TAKE + 0.2, WAIT_R, GRIP_L, "wait for the finger to be held", 0.0)
    R.add(2.5, touch - PRE_PRESS * a, GRIP_L, "tilt over the site", rot=rp)
    R.add(STEADY, touch - PRE_PRESS * a, GRIP_L, "steady over the site", rot=rp)
    # WHY slow from here: run at 1.5 s and 0.8 s, the arm's lag swung the face 2 to 8 deg round the finger on the way
    # in, so the press landed that far from square to the skin (PhysX, 2026-11-20) [RUN]
    R.add(2.5, touch - 0.004 * a, GRIP_L, "just off the site", rot=rp)
    R.add(1.0, touch - 0.004 * a, GRIP_L, "steady just off the site", rot=rp)
    R.add(1.6, touch, GRIP_L, "tip on the site", rot=rp)
    ad = press_dir(seed, touch, rp)
    # WHY in two stages with a settle: pressed straight through, the face slipped on the skin near the fire in PhysX
    # (0.8 mm at one seed) and the shield bottomed, so the push went on into the finger after the click (1.4 mm of drop
    # within 0.5 s) [RUN: PhysX, 2026-11-20]. Loaded to about 5 N and let settle, then pressed on to just past the
    # fire, the load builds quasi-statically and stops a few N past 8 N
    lever = tcp_over_lancet()
    for dur, f, label in ((1.0, PRESS_F[0], "load the press"), (0.5, PRESS_F[0], "settle the press"),
                          (1.0, PRESS_F[1], "press until it fires"), (0.3, PRESS_F[1], "hold the press")):
        rs = press_rot(seed, rp, f, touch)
        dz = press_depth(seed, f, touch, rp)
        # WHY the TCP moved with the turn: the face, not the TCP, is what aims at the site
        R.add(dur, touch + dz * ad + lever * (a - rs[:, 2]), GRIP_L, label, rot=rs)
    R.add(1.2, touch - PRE_PRESS * a, GRIP_L, "back off along the axis", rot=rp)
    R.add(2.0, WAIT_R, GRIP_L, "back out of the left's way", 0.0)
    t_free = R.t
    R.add(L_FREE, WAIT_R, GRIP_L, "wait for the left to let go", 0.0)
    over = (SHARPS[0], SHARPS[1], RELEASE_Z)
    R.add(2.0, over, GRIP_L, "over the sharps container", 0.0)
    R.add(STEADY, over, GRIP_L, "steady over the opening", 0.0)
    R.add(0.6, over, OPEN_F, "let go", 0.0)
    # WHY 15 mm up: 30 mm over the container is past the right arm's reach (10 mm short) [RUN: tools/ikcheck.py]
    R.add(1.5, (SHARPS[0], SHARPS[1], RELEASE_Z + 0.015), OPEN_F, "up clear", 0.0)
    # the left hand: wait parked while the right takes the lancet, hold the ring finger through the press and until the
    # right is back at its wait, then let go and park before the right goes to the container
    g, ah, rh = hold_pose(seed)
    park, _ = park_pose(seed)
    L.add(2.5, park, OPEN_F, "park", rot=rh)
    L.add(t_left - L.t, park, OPEN_F, "wait parked", rot=rh)
    L.add(2.0, g - APPROACH_L * ah, OPEN_R, "beside the ring finger", rot=rh)
    L.add(STEADY, g - APPROACH_L * ah, OPEN_R, "steady beside the ring finger", rot=rh)
    L.add(1.2, g, OPEN_R, "in round the ring finger", rot=rh)
    L.add(0.6, g, GRIP_R, "grip the ring finger", rot=rh)
    L.add(0.8, g + [0, 0, HOLD_UP], GRIP_R, "hold it up on its stop", rot=rh)
    L.add(t_free - L.t, g + [0, 0, HOLD_UP], GRIP_R, "keep holding through the press", rot=rh)
    L.add(0.6, g, GRIP_R, "ease off", rot=rh)
    L.add(0.6, g, OPEN_R, "let go of the ring finger", rot=rh)
    L.add(1.2, g - APPROACH_L * ah, OPEN_R, "back out", rot=rh)
    L.add(2.0, park, OPEN_F, "back to park", rot=rh)
    return R.w, L.w


def demo(delay=0.0):
    right, left = plan(delay, run_seed())
    return {"right": right, "left": left}


def _replan_demo_after_reset():
    """WHY: inspect-robots resets the policy before the embodiment (inspect_robots/rollout.py), so yam_scripted would
    call demo() while run_seed() still holds the previous trial's seed and plan for the wrong layout. This makes
    yam_scripted plan again on each trial's first action, after the reset has set the seed, and play each arm's plan
    as a TiltPlan, so the demo's tilted grips (TWaypoint.rot) are followed (yam_scripted's ArmPlan turns a grip only
    about the vertical). It changes only the scripted demo policy, never the scene or the score."""
    try:
        from yam_arena import scripted
    except ImportError:
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_lancet_replan", False):
        return
    reset, act = cls.reset, cls.act
    unset = object()

    def tilt(self):
        self._plans = {arm: TiltPlan(p.waypoints, p.base_pos) for arm, p in self._plans.items()}

    def patched_reset(self, scene):
        self._lancet_scene, self._lancet_seed = scene, unset
        reset(self, scene)
        tilt(self)

    def patched_act(self, observation):
        if getattr(self, "_lancet_seed", None) is unset:
            self._lancet_seed = run_seed()
            reset(self, self._lancet_scene)
            tilt(self)
        return act(self, observation)

    cls.reset, cls.act, cls._lancet_replan = patched_reset, patched_act, True


_replan_demo_after_reset()
