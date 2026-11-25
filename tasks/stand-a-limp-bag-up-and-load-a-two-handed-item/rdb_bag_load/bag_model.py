"""Every dimension, mass, spring and friction of the limp tote bag, the carton and the pack. NumPy only.

Metres, kilograms, newtons, radians. Tabletop frame: x forward from the line through the two arm bases, y to the
robot's left, z up from the table top. The bag's long side runs away from the robot (along x); its two large
walls, each with a handle, face the two arms: ``left`` at +y, ``right`` at -y. ``near`` and ``far`` are the two
narrow end walls.

Status of each number: ESTIMATE (to be measured on the bought object), DERIVED (follows from other numbers) or
NUMERICAL (chosen for a stable simulation, stated in the task's fidelity field). Nothing here was measured on a
real bag.
"""

import numpy as np

# --- where things stand -------------------------------------------------------------------------------------
BAG_XY = (.360, .070)               # middle of the bag's base board on its tape marks
CARTON_XY = (.220, .245)            # the carton stands to the robot's left of the bag's near end
PACK_XY = (.405, -.170)             # the pack stands to the robot's right of the bag, level with its place in it

# --- the bag ------------------------------------------------------------------------------------------------
BASE_X, BASE_Y, BASE_T = .310, .160, .004       # ESTIMATE base board 310 x 160; NUMERICAL 4 mm thick (real about 2)
BASE_MASS = .045                    # ESTIMATE
WALL_T = .002                       # NUMERICAL collider thickness of the fabric (real non-woven: about 0.5 mm)
RIM = .180                          # ESTIMATE height of the bag's rim above the table with the walls pulled straight
SEG = (RIM - BASE_T) / 2            # DERIVED each wall is two rigid panels this high, hinged to each other
SIDE_L = BASE_X - 3 * WALL_T        # DERIVED length of a large wall (left, right): it stands between the end walls
END_L = BASE_Y - 3 * WALL_T         # DERIVED length of an end wall (near, far): it stands between the large walls
SIDE_PANEL_MASS = .008              # NUMERICAL one panel of a large wall (90 g/m^2 fabric would be 3 g)
END_PANEL_MASS = .004               # NUMERICAL one panel of an end wall (fabric: 1.5 g)
WALLS = ("left", "right", "near", "far")
OUTWARD = {"left": (0., 1., 0.), "right": (0., -1., 0.), "near": (-1., 0., 0.), "far": (1., 0., 0.)}
HALF_LENGTH = {"left": SIDE_L / 2, "right": SIDE_L / 2, "near": END_L / 2, "far": END_L / 2}
PANEL_MASS = {"left": SIDE_PANEL_MASS, "right": SIDE_PANEL_MASS, "near": END_PANEL_MASS, "far": END_PANEL_MASS}
HINGE_OUT = {"left": BASE_Y / 2 - WALL_T / 2, "right": BASE_Y / 2 - WALL_T / 2,       # hinge line: this far out from
             "near": BASE_X / 2 - WALL_T / 2, "far": BASE_X / 2 - WALL_T / 2}         # the middle of the base

# A wall's two joints. ``lean``: the lower panel about the base's edge, 0 upright, positive inward. ``fold``: the
# upper panel relative to the lower one, 0 straight. Each wall folds one way only, along its one crease: a large
# wall's lower panel leans inward and its upper panel folds back over it (crease inside the bag); an end wall's
# lower panel leans outward and its upper panel folds back over it (crease outside). Either way the wall's top
# edge stays over its bottom edge as the wall comes down, which is what the sewn corners require. Stops keep a
# wall from folding the other way.
CREASE = {"left": 1., "right": 1., "near": -1., "far": -1.}             # +1: crease inside; -1: crease outside
LEAN_LIMIT = {w: tuple(sorted(np.radians((-8. * s, 92. * s)))) for w, s in CREASE.items()}
FOLD_LIMIT = {w: tuple(sorted(np.radians((3. * s, -176. * s)))) for w, s in CREASE.items()}
# ESTIMATE the creases remember being folded; the right wall's a little more, so that it goes down first
LEAN_REST = {"left": np.radians(20.), "right": np.radians(35.), "near": np.radians(-25.), "far": np.radians(-25.)}
FOLD_REST = {"left": np.radians(-40.), "right": np.radians(-70.), "near": np.radians(50.), "far": np.radians(50.)}
CREASE_K = .002                     # ESTIMATE N m/rad: a limp fabric crease (gravity on a wall is ten times this)
CREASE_DAMPING = .0010              # NUMERICAL N m s/rad
CREASE_ARMATURE = 1e-5              # NUMERICAL kg m^2

# Corner seams: the top corners of neighbouring walls are tied together by a short elastic tie, standing for the
# give of the fabric at a sewn corner. With no pull the corners meet; a wall that hangs lower than its neighbours
# is pulled up by them.
SEAM_SLACK = .002                   # NUMERICAL: with the walls straight the tied points are 1.7 mm apart, so the ties
                                    # are then just slack (tight ties hold the empty bag's walls up against each other)
SEAM_K = 1000.                      # ESTIMATE N/m
SEAM_DAMPING = 1.                   # NUMERICAL N s/m
SEAM_TORN_M = .030                  # ESTIMATE a corner pulled this far apart counts as tearing the bag (scorer): 28 N
SEAM_TORN_N = SEAM_K * (SEAM_TORN_M - SEAM_SLACK)
CORNERS = (("left", "near"), ("left", "far"), ("right", "near"), ("right", "far"))

# Handles: a flat loop on each large wall, hinged along the rim, free to swing. Rigid (a stiffened strap).
HANDLE_SPAN = .150                  # ESTIMATE outside width of the loop along the rim
HANDLE_DROP = .090                  # ESTIMATE from the rim to the far side of the grip
LEG_W, LEG_T = .016, .006           # ESTIMATE strap 16 mm wide; NUMERICAL 6 mm thick
GRIP_W, GRIP_T = .018, .010         # ESTIMATE padded grip 18 mm wide, 10 mm thick
HANDLE_MASS = .008                  # ESTIMATE
# 0: standing up in line with the wall; negative: swung outward. The stiff strap is sewn to the outside of the wall:
# it cannot fold in over the rim, and hanging down it stands 25 degrees off the wall (its grip 34 mm from it).
HANDLE_LIMIT = np.radians((-155., 10.))
HANDLE_DAMPING = .0004              # NUMERICAL N m s/rad
HANDLE_ARMATURE = 5e-6              # NUMERICAL
BAG_MASS = BASE_MASS + 4 * SIDE_PANEL_MASS + 4 * END_PANEL_MASS + 2 * HANDLE_MASS       # DERIVED 109 g

# --- the two items ------------------------------------------------------------------------------------------
CARTON = (.060, .132, .215)         # ESTIMATE full box of crackers: 60 thick (x) x 132 wide (y) x 215 high
CARTON_MASS = .280                  # ESTIMATE
PACK = (.200, .100, .200)           # ESTIMATE four toilet rolls in film, two across and two high: 200 x 100 x 200
PACK_MASS = .360                    # ESTIMATE
GRIPPER_OPEN = .094                 # the gripper's pads are this far apart when open
# Where the two items end up inside the bag (their middles, bag frame, standing on the base board): the carton
# across the near end, the pack filling the rest. Used by the demonstration and the physics checks, not the scorer.
CARTON_SLOT = (-BASE_X / 2 + WALL_T + .008 + CARTON[0] / 2, 0., BASE_T + CARTON[2] / 2)
PACK_SLOT = ((CARTON_SLOT[0] + CARTON[0] / 2 + BASE_X / 2 - WALL_T) / 2, 0., BASE_T + PACK[2] / 2)   # mid-way

# --- friction (ESTIMATES) -----------------------------------------------------------------------------------
FRICTION = {"fabric": .50, "board": .50, "carton": .45, "film": .90, "strap": .70}

# --- names --------------------------------------------------------------------------------------------------
LOWER = {w: f"{w}_lower" for w in WALLS}
UPPER = {w: f"{w}_upper" for w in WALLS}
HANDLES = {"left": "handle_left", "right": "handle_right"}
LINKS = ("base", *LOWER.values(), *UPPER.values(), *HANDLES.values())
ITEMS = ("carton", "pack")
LEAN = {w: f"{w}_lean" for w in WALLS}
FOLD = {w: f"{w}_fold" for w in WALLS}
SWING = {w: f"{w}_swing" for w in HANDLES}
JOINTS = (*LEAN.values(), *FOLD.values(), *SWING.values())


def rot_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array(((1., 0., 0.), (0., c, -s), (0., s, c)))


def wall_frame(wall):
    """Columns: along the hinge, outward, up (bag frame). A positive turn about the first leans the wall inward."""
    out = np.asarray(OUTWARD[wall], dtype=float)
    up = np.array((0., 0., 1.))
    return np.column_stack((np.cross(out, up), out, up))


def hinge_point(wall):
    """Middle of the wall's hinge line on the base board (bag frame: origin on the table under the base's middle)."""
    return np.asarray(OUTWARD[wall], dtype=float) * HINGE_OUT[wall] + (0., 0., BASE_T)


def wall_pose(wall, lean, fold, swing=0.):
    """Bag-frame poses for joint angles: dict of (position, rotation) for the wall's lower and upper panel and, for
    a large wall, its handle (origin on the rim), plus ``rim``: the two ends of the wall's top edge."""
    frame, hinge = wall_frame(wall), hinge_point(wall)
    low = frame @ rot_x(lean)
    crease = hinge + low @ (0., 0., SEG)
    high = frame @ rot_x(lean + fold)
    top = crease + high @ (0., 0., SEG)
    half = HALF_LENGTH[wall]
    out = {"lower": (hinge + low @ (0., 0., SEG / 2), low), "upper": (crease + high @ (0., 0., SEG / 2), high),
           "rim": (top - high[:, 0] * half, top + high[:, 0] * half)}
    if wall in HANDLES:
        out["handle"] = (top, frame @ rot_x(lean + fold + swing))
    return out


def grip_centre(handle_pos, handle_rot):
    """Middle of a handle's grip from the handle's pose."""
    return np.asarray(handle_pos, dtype=float) + np.asarray(handle_rot, dtype=float) @ (0., 0., HANDLE_DROP - GRIP_W / 2)


def corner_anchor(wall, other):
    """Where the seam to ``other`` is tied on ``wall``'s upper panel, in that panel's own frame."""
    along = wall_frame(wall)[:, 0] @ np.asarray(OUTWARD[other], dtype=float)      # +1 or -1: which end of the wall
    return np.array((along * HALF_LENGTH[wall], 0., SEG / 2))


def seam_lengths(panels):
    """Distance between the two tied points of each corner seam. ``panels``: name -> (position, rotation)."""
    out = {}
    for a, b in CORNERS:
        pa, ra = panels[UPPER[a]]
        pb, rb = panels[UPPER[b]]
        out[f"{a}_{b}"] = float(np.linalg.norm(np.asarray(pa) + np.asarray(ra) @ corner_anchor(a, b)
                                               - np.asarray(pb) - np.asarray(rb) @ corner_anchor(b, a)))
    return out


def seam_pull(length):
    """Tension in a corner seam of the given length, N."""
    return SEAM_K * max(0., float(length) - SEAM_SLACK)


def rim_height(wall, position, rotation):
    """Height above the table of the lower end of a wall's top edge, from its upper panel's tabletop-frame pose."""
    rotation = np.asarray(rotation, dtype=float)
    top = np.asarray(position, dtype=float) + rotation @ (0., 0., SEG / 2)
    return float(top[2] - abs(rotation[2, 0]) * HALF_LENGTH[wall])


# The start of every trial: the empty bag collapsed on its base board, every wall folded flat on itself, each
# handle lying on the table beside the bag. Joint angles in degrees found by letting
# the bag settle (tools/physx_probe.py settle) and rounded; see TECHNICAL.md.
START_DEG = {
    "near_lean": -79.5, "near_fold": 165.4, "far_lean": -79.5, "far_fold": 165.4,
    "right_lean": 89.4, "right_fold": -165.8, "left_lean": 85.4, "left_fold": -165.4,
    "left_swing": -23.8, "right_swing": -26.9,
}
START = {name: float(np.radians(value)) for name, value in START_DEG.items()}

assert abs(BAG_MASS - .109) < 1e-9 and abs(SEG - .088) < 1e-9
assert min(PACK) > GRIPPER_OPEN and CARTON[0] < GRIPPER_OPEN                  # the pack fits no single gripper
assert .030 < BASE_X - 2 * WALL_T - CARTON[0] - PACK[0] < .060                 # side by side, 3 to 6 cm to spare
assert PACK[1] < CARTON[1] < BASE_Y - 2 * WALL_T and CARTON[2] > RIM and PACK[2] > RIM
