"""Geometry and measurements for priming a steel plate with a block brush.

A bare steel plate lies on the bench; beside it, a paint tray of red-oxide primer and a block brush
lying bristles-down. Primer only goes on from a brush that has been dipped, measured in ml: the coat
builds up, 10 mm patch by patch, where the loaded bristle tips actually touch the plate, thin and
see-through where a long stroke has drained the tips, opaque where it has been laid on fully.
Frame: metres, x away from the arm bases, y left, z up; TOP is the bench.
"""
from __future__ import annotations

import math

import numpy as np

from simkit import Hinge, Part, Piece

TOP = 0.7475

STEEL = (0.42, 0.43, 0.45)          # bare mill-scale plate
PRIMER = (0.55, 0.16, 0.10)         # red-oxide primer
TRAY = (0.80, 0.80, 0.78)
WOOD = (0.62, 0.45, 0.28)
BRISTLE = (0.86, 0.80, 0.66)        # pale natural bristle, so a loaded brush shows

# -- plate and its coat ---------------------------------------------------------------------
PLATE_XY = (0.330, 0.000)
PLATE = (0.120, 0.070, 0.008)
PLATE_TOP = TOP + PLATE[2]
TILE = 0.010
NX, NY = int(round(PLATE[0] / TILE)), int(round(PLATE[1] / TILE))
TILES = NX * NY                     # 12 x 7 = 84 patches of coat


def tile_centres():
    """Tile centres in plate-local x, y."""
    return [((i - (NX - 1) / 2) * TILE, (j - (NY - 1) / 2) * TILE) for i in range(NX) for j in range(NY)]


# -- primer tray ------------------------------------------------------------------------------
TRAY_XY = (0.330, 0.175)
TRAY_IN = (0.070, 0.100, 0.016)     # inside, x by y by depth: shallow, so the handle clears the rim
TRAY_WALL = 0.004
PRIMER_DEPTH = 0.010
# -- primer, by volume (ml) ---------------------------------------------------------------------
# A 70 mm brush dipped half its bristle length picks up ~6 ml of primer; the bristles hold ~3.5 ml
# without running; the excess runs off the tips as drops of ~0.05 ml until wiped off on the rim.
# Brushed thin, primer goes on at ~0.01 ml per cm^2 (100 um wet): one dip coats several plates.
PICKUP_ML = 6.0
HOLD_ML = 3.5
DROP_ML = 0.05
RUN_OFF_PER_S = 0.6                  # fraction of the excess that runs off per second while held up
ML_PER_TILE = 0.01                   # a full coat on one 10 x 10 mm patch
# Only the primer at the bristle tips goes onto the plate; it is fed from the body of the brush more
# slowly than a long stroke lays it down, so a stroke thins out towards its end (steel shows through)
# and a second pass, once the tips have filled again, tops it up.
TIP_CAP_ML = 0.30                    # primer the tips carry when full
TIP_FEED_ML_S = 0.12                 # feed from the body of the brush to the tips
LAY_PER_STEP = 0.25                  # fraction of a full coat a full tip lays on a patch per 0.05 s of contact
FULL_COAT = 0.85                     # film fraction at which a patch is opaque: coated
RIM_TOP = TOP + TRAY_WALL + TRAY_IN[2]
WIPE_WALL_X = TRAY_XY[0] + TRAY_IN[0] / 2 + TRAY_WALL / 2   # the tray's far wall, where the brush is wiped off
DROP_R = 0.0023                      # a 0.05 ml drop
DROP_MASS = 0.00005
DROPS = 24
DROP_NAMES = tuple(f"drip{i:02d}" for i in range(DROPS))
SPLAT_R = 0.0045                     # a drop spread flat where it lands

# -- block brush ------------------------------------------------------------------------------
# A wooden stock with a round handle, and a head of bristle tufts set in it: 2 rows across its 16 mm
# thickness, 7 across its 70 mm width. Each tuft is two flexible segments: they bend back as the
# brush is drawn, splay when it is pressed, and spring straight when it is lifted.
BRISTLE_BLOCK = (0.016, 0.070, 0.020)        # the head's footprint and bristle length
TUFT_ROWS, TUFT_COLS = 2, 7
TUFT = (0.007, 0.009)                        # one tuft's cross-section, x by y
SEG_L = BRISTLE_BLOCK[2] / 2
SEG_MASS = 0.0012
TUFT_K = 3.0e-4                              # bending stiffness, N m per degree, per joint
TUFT_DAMP = 2.0e-6
TUFT_LIMIT = 40.0
STOCK = (0.024, 0.074, 0.016)
HANDLE_R = 0.010
HANDLE_L = 0.110
HANDLE_Z = BRISTLE_BLOCK[2] + 0.008          # handle axis above the bristle tips
GRIP_X = -0.036                      # close to the stock: the head is heavy, a grip far back lets it pivot down
BRUSH_MASS = 0.045
BRUSH_XY = (0.220, 0.200)
WET = (0.62, 0.20, 0.13)                     # bristles soaked with red-oxide


def tuft_names():
    return tuple(f"tuft{r}{c}_{k}" for r in range(TUFT_ROWS) for c in range(TUFT_COLS) for k in ("a", "b"))


TUFT_NAMES = tuft_names()


def tuft_xy(name):
    r, c = int(name[4]), int(name[5])
    return ((r - (TUFT_ROWS - 1) / 2) * BRISTLE_BLOCK[0] / TUFT_ROWS,
            (c - (TUFT_COLS - 1) / 2) * BRISTLE_BLOCK[1] / TUFT_COLS)


COAT_LAYERS = ((0.15, 0.30), (0.45, 0.62), (FULL_COAT, 1.0))   # (film fraction from which it shows, opacity)


def tile_piece(k, layer):
    """Index, among the plate's pieces, of tile k's drawing at a coat layer."""
    return layer * TILES + k


def plate_object() -> Part:
    """The plate, with its coat drawn as tiles just above its top (hidden until painted): each patch
    has a thin, a half and a full coat drawing, from see-through to opaque."""
    tiles = tuple(Piece("box", (TILE + 0.0004, TILE + 0.0004, 0.0006), (x, y, PLATE[2] / 2 + 0.0004 + 0.0002 * layer),
                        color=PRIMER, opacity=op, collide=False)
                  for layer, (_, op) in enumerate(COAT_LAYERS) for x, y in tile_centres())
    return Part("plate", pos=(PLATE_XY[0], PLATE_XY[1], TOP + PLATE[2] / 2), size=PLATE, exact_z=True, color=STEEL,
                friction=(0.5, 0.4), pieces=tiles)


def tray_object() -> Part:
    tx, ty = TRAY_XY
    lx, ly, d = TRAY_IN[0] + 2 * TRAY_WALL, TRAY_IN[1] + 2 * TRAY_WALL, TRAY_IN[2]
    t = TRAY_WALL
    zc = d / 2
    p = []
    for s in (-1, 1):
        p.append(Piece("box", (t, ly, d + t), (s * (lx - t) / 2, 0.0, zc)))
        p.append(Piece("box", (lx - 2 * t, t, d + t), (0.0, s * (ly - t) / 2, zc)))
    p.append(Piece("box", (TRAY_IN[0], TRAY_IN[1], PRIMER_DEPTH), (0.0, 0.0, t / 2 + PRIMER_DEPTH / 2), color=PRIMER,
                   collide=False))
    return Part("tray", pos=(tx, ty, TOP + t / 2), size=(lx, ly, t), exact_z=True, color=TRAY, friction=(0.6, 0.5),
                pieces=tuple(p))


def brush_object(xy=BRUSH_XY) -> Part:
    """The stock (root) with its round handle back toward the arms; the tufts are hinged to it."""
    p = [Piece("cylinder", (HANDLE_R, HANDLE_L, HANDLE_R), (-STOCK[0] / 2 - HANDLE_L / 2 + 0.004, 0.0, HANDLE_Z - stock_z()),
               axis="X", color=WOOD, friction=(1.6, 1.4)),
         Piece("box", (STOCK[0] + 0.001, STOCK[1] + 0.001, 0.005), (0.0, 0.0, -STOCK[2] / 2 + 0.0025),
               color=(0.55, 0.56, 0.58))]                                     # steel ferrule round the tuft roots
    return Part("brush", pos=(xy[0], xy[1], TOP + stock_z()), size=STOCK, exact_z=True, mass=BRUSH_MASS, color=WOOD,
                friction=(0.5, 0.4), restitution=0.0, contact_offset=0.0015, com=(-0.030, 0.0, HANDLE_Z - stock_z()),
                pieces=tuple(p))


def stock_z():
    """Stock centre above the bristle tips (bristles straight)."""
    return BRISTLE_BLOCK[2] + STOCK[2] / 2


def tuft_object(name, xy=BRUSH_XY) -> Part:
    """One bristle-tuft segment: 'a' set in the stock, 'b' the tip half hinged to it. Each bends about
    y (the way the brush is drawn) against a spring. A soaked tip is drawn over it while loaded."""
    dx, dy = tuft_xy(name)
    upper = name.endswith("a")
    top = TOP + BRISTLE_BLOCK[2] - (0.0 if upper else SEG_L)
    c = (xy[0] + dx, xy[1] + dy, top - SEG_L / 2)
    pivot = (c[0], c[1], top)
    parent, parent_pos = ("brush", (xy[0], xy[1], TOP + stock_z())) if upper else \
        (name[:-1] + "a", (c[0], c[1], top + SEG_L / 2))
    wet = Piece("box", (TUFT[0] + 0.0006, TUFT[1] + 0.0006, SEG_L * (0.6 if upper else 1.0) + 0.0004),
                (0.0, 0.0, -SEG_L * 0.2 if upper else 0.0), color=WET, collide=False)
    return Part(name, pos=c, size=(TUFT[0], TUFT[1], SEG_L), exact_z=True, mass=SEG_MASS, color=BRISTLE,
                friction=(0.4, 0.3), restitution=0.0, contact_offset=0.001, pieces=(wet,),
                hinge=Hinge(pivot=pivot, axis="Y", damping=TUFT_DAMP, lower_deg=-TUFT_LIMIT, upper_deg=TUFT_LIMIT,
                            stiffness=TUFT_K, parent=parent, parent_pos=parent_pos))


def splat_object() -> Part:
    """Where drops of primer land they spread flat: drawn discs, parked out of sight until used."""
    discs = tuple(Piece("cylinder", (SPLAT_R, 0.0006, SPLAT_R), (0.0, 0.0, -0.4 + 0.0), color=PRIMER, collide=False)
                  for _ in DROP_NAMES)
    return Part("splats", pos=(0.05, -0.60, TOP + 0.001), size=(0.002, 0.002, 0.002), exact_z=True, color=PRIMER,
                pieces=discs)


# -- measurements ----------------------------------------------------------------------------

def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def bristle_tips(brush_pose):
    """World centre of the bristle tips, the brush's rotation, and its tilt from upright (degrees)."""
    b = np.asarray(brush_pose, float)
    r = quat_matrix(b[3:7])
    tips = b[:3] + r @ np.array([0.0, 0.0, -stock_z()])
    return tips, r, math.degrees(math.acos(np.clip(r[2, 2], -1, 1)))


def in_primer(brush_pose) -> bool:
    tips, _, tilt = bristle_tips(brush_pose)
    return bool(tilt < 30 and abs(tips[0] - TRAY_XY[0]) < TRAY_IN[0] / 2 and abs(tips[1] - TRAY_XY[1]) < TRAY_IN[1] / 2
                and tips[2] < TOP + TRAY_WALL + PRIMER_DEPTH - 0.003)


def tuft_tip(pose):
    """Centre of a tip segment's end, and how far that segment leans from straight down (degrees)."""
    p = np.asarray(pose, float)
    r = quat_matrix(p[3:7])
    return p[:3] + r @ np.array([0.0, 0.0, -SEG_L / 2]), math.degrees(math.acos(np.clip(r[2, 2], -1, 1)))


def tiles_touched(tip):
    """Coat tiles a bristle tip on the plate's top is wetting."""
    if abs(tip[2] - PLATE_TOP) > 0.0025:
        return []
    reach = (TUFT[0] / 2 + 0.003, TUFT[1] / 2 + 0.003)
    return [k for k, (x, y) in enumerate(tile_centres())
            if abs(PLATE_XY[0] + x - tip[0]) < reach[0] and abs(PLATE_XY[1] + y - tip[1]) < reach[1]]


def tiles_under(brush_pose):
    """Indices of coat tiles the bristle tips are on now (tips on the plate's top, brush near flat)."""
    tips, r, tilt = bristle_tips(brush_pose)
    if tilt > 20 or abs(tips[2] - PLATE_TOP) > 0.004:
        return []
    out = []
    for k, (x, y) in enumerate(tile_centres()):
        c = np.array([PLATE_XY[0] + x, PLATE_XY[1] + y, PLATE_TOP])
        loc = r.T @ (c - tips)
        if abs(loc[0]) < BRISTLE_BLOCK[0] / 2 + 0.002 and abs(loc[1]) < BRISTLE_BLOCK[1] / 2:
            out.append(k)
    return out


def on_rim(brush_pose) -> bool:
    """Bristle tips drawn across the top of the tray's far wall (wiping off the excess)."""
    tips, _, tilt = bristle_tips(brush_pose)
    return bool(tilt < 30 and abs(tips[0] - WIPE_WALL_X) < 0.009 and abs(tips[1] - TRAY_XY[1]) < TRAY_IN[1] / 2
                and tips[2] < RIM_TOP + 0.004)


def over_tray(p) -> bool:
    return bool(abs(p[0] - TRAY_XY[0]) < TRAY_IN[0] / 2 + TRAY_WALL and abs(p[1] - TRAY_XY[1]) < TRAY_IN[1] / 2 + TRAY_WALL)


def drop_objects():
    return [Part(n, pos=parking(i), size=(DROP_R,) * 3, shape="sphere", exact_z=True, mass=DROP_MASS, color=PRIMER,
                 friction=(0.6, 0.5), restitution=0.0, contact_offset=0.001, max_depenetration=0.3)
            for i, n in enumerate(DROP_NAMES)]


def parking(i):
    return (0.10 + 0.012 * (i % 12), -0.55 - 0.012 * (i // 12), TOP - 0.30)


def tips_on_bench(brush_pose) -> bool:
    """Loaded bristles flat on the bench, off the plate and the tray (painting the bench)."""
    tips, _, tilt = bristle_tips(brush_pose)
    return bool(tilt < 30 and tips[2] < TOP + 0.004 and not on_plate(tips) and not over_tray(tips))


def tip_on_bench(tip) -> bool:
    """A bristle tip down on the bench itself, off the plate and the tray."""
    return bool(tip[2] < TOP + 0.003 and not on_plate(tip) and not over_tray(tip))


def grip_point(brush_pose):
    b = np.asarray(brush_pose, float)
    return b[:3] + quat_matrix(b[3:7]) @ np.array([GRIP_X, 0.0, HANDLE_Z - stock_z()])


def on_plate(p) -> bool:
    return bool(abs(p[0] - PLATE_XY[0]) < PLATE[0] / 2 + 0.01 and abs(p[1] - PLATE_XY[1]) < PLATE[1] / 2 + 0.01)
