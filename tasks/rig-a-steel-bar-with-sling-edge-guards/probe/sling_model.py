"""Probe model: a fixed 4 x 4 in square bar, a 2 in two-ply web sling with triangle fittings in a basket hitch,
four magnetic L-shaped edge guards and a damped, latched eye hook on a force-limited lift. No robot.

Frame: tabletop (x forward, y left, z up from the table top). Layout "X": the bar's long axis along x, the sling
wraps it in the y-z plane at x = XS, the legs at y = +-S/2, the hook's plane is y-z with its throat facing -y.
Every dimension is in metres; each is marked as from a source, derived, or an ESTIMATE.
"""

import math
import os

import numpy as np

IN = 0.0254
# -- bar and blocks ---------------------------------------------------------------------------------------------
BAR_S = 4 * IN                  # 4 x 4 in square, sharp edges (load named in the brief)
BAR_L = 12 * IN                 # 12 in long (robotics review's change)
XS = 0.25                       # sling plane (robotics review's change: x ~ 0.25)
BLOCK_H = 1.5 * IN              # 2x4 laid flat
Z_BOT, Z_TOP = BLOCK_H, BLOCK_H + BAR_S
HALF = BAR_S / 2
BAR_LO = np.array((XS - BAR_L / 2, -HALF, Z_BOT))
BAR_HI = np.array((XS + BAR_L / 2, HALF, Z_TOP))
BLOCKS = [((XS - BAR_L / 2 + .015, -.15, 0.), (XS - BAR_L / 2 + .015 + 3.5 * IN, .15, Z_BOT)),
          ((XS + BAR_L / 2 - .015 - 3.5 * IN, -.15, 0.), (XS + BAR_L / 2 - .015, .15, Z_BOT))]

# -- sling: HSI TT2-802-02 class, 2 in wide, two-ply nylon, triangle fittings, 2 ft bearing point to bearing point --
SLING_W = 2 * IN
WEB_T = .0045                   # collider thickness (brief: 4-5 mm; a two-ply 2 in web is about 3-4 mm, ESTIMATE)
FIT_H = .055                    # fitting: base bar (web loop) to apex bar (bearing point), ESTIMATE
FIT_BASE_HALF, FIT_APEX_HALF = .032, .020
FIT_BAR_R, FIT_SIDE_R = .007, .006   # 14 mm bar (robotics review: 13-16 mm)
FIT_MASS = .45                  # each fitting 1-1.1 lb (robotics review, preliminary)
BEARING_L = 24 * IN             # 610 mm
WEB_L = BEARING_L - 2 * FIT_H   # web between the base bars
N_LINKS = int(os.environ.get("SLING_N_LINKS", 22))   # 22: 22.7 mm links (brief: 20-25 mm)
LINK_L = WEB_L / N_LINKS
LINK_MASS = .44 / N_LINKS       # 0.44 kg in all (real web ~0.08 kg): raised for steady contact, ESTIMATE
HINGE_K = .010                  # N m/rad, web bending stiffness over one link, ESTIMATE
HINGE_D = .002                  # N m s/rad
HINGE_ARMATURE = 1e-5
HINGE_LIMIT_DEG = 130.

# -- edge guards: magnetic L-shaped corner protectors, 3 mm (Pfeifer / DICA class) ------------------------------
GUARD_LEN = .100                # along the edge: 51 mm web + 24.5 mm each side
GUARD_FL = .050                 # flange width from the corner, ESTIMATE
GUARD_T = .003
GUARD_MASS = .25                # ESTIMATE (Pfeifer's magnetic protectors are 0.8-1.0 kg for 40-65 mm webs)
MAG_F0 = 8.                     # N per magnet at zero gap, 4 magnets per guard, ESTIMATE
MAG_G0 = .002                   # m, gap at which the pull falls to a quarter
MAG_RANGE = .015
EDGES = {  # name: (corner point (y, z), outward normal of the horizontal face n1z, outward normal of the side n2y)
    "TL": ((HALF, Z_TOP), 1., 1.), "BL": ((HALF, Z_BOT), -1., 1.),
    "TR": ((-HALF, Z_TOP), 1., -1.), "BR": ((-HALF, Z_BOT), -1., -1.)}

# -- hook: 2 t carbon eye hook with spring latch (Crosby 320 class) -----------------------------------------------
HOOK_Z = .20                    # bowl bottom (inside) when hung (brief)
BOWL_R_IN = 1.63 * IN / 2       # catalogue F for the 2 t hook (which letter is the bowl is to be confirmed)
HOOK_R = .011                   # section radius, ESTIMATE
RC = BOWL_R_IN + HOOK_R
HOOK_O = np.array((XS, 0., HOOK_Z + BOWL_R_IN))     # bowl centre
THROAT = np.array((0., -1., 0.))                     # the opening faces the right arm
HOOK_MASS = .94                 # 2.07 lb (catalogue, 2 t)
LATCH_MASS = .02
LATCH_K = .06                   # N m/rad, ESTIMATE
LATCH_PRELOAD = .03             # N m against the tip, ESTIMATE (about 1 N at the latch's end)
PIVOT_V = .093                  # eye centre above the bowl centre: the pendulum's pivot
SWING_DAMPING = .05             # N m s/rad, ESTIMATE
CARRIAGE_MASS = .50             # shackle and the lifting point
LIFT_K, LIFT_D, LIFT_MAX_F = 3000., 400., 200.

# -- materials (static = dynamic friction) -----------------------------------------------------------------------
FRICTION = {"steel": .35, "web": .40, "guard": .50, "wood": .50, "table": .50, "hook": .30}


def hp(s, v):
    """A point in the hook's plane: s along the throat direction, v up, from the bowl centre."""
    return HOOK_O + s * THROAT + v * np.array((0., 0., 1.))


def hook_capsules():
    """(name, a, b, radius) in the world at the hung pose: the hook body's colliders."""
    caps = []
    angs = np.radians(np.linspace(180, 360, 9))
    pts = [hp(RC * math.cos(a), RC * math.sin(a)) for a in angs]
    for k, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
        caps.append((f"bowl_{k}", a, b, HOOK_R))
    caps.append(("tip", hp(RC, 0.), hp(RC - .004, .024), .008))
    caps.append(("shank_0", hp(-RC, 0.), hp(-.012, .035), .010))
    caps.append(("shank_1", hp(-.012, .035), hp(0., .075), .009))
    ring = [hp(.018 * math.cos(a), PIVOT_V + .018 * math.sin(a)) for a in np.radians(np.arange(0, 361, 45))]
    for k, (a, b) in enumerate(zip(ring[:-1], ring[1:])):
        caps.append((f"eye_{k}", a, b, .006))
    return caps


LATCH_PIVOT = hp(-.010, .030)
LATCH_END = hp(RC - .012, .027)       # resting against the inside of the tip


def guard_boxes(name):
    """The guard's two flanges as (centre, half extents) in the world, at its seat on the edge."""
    (ye, ze), n1z, n2y = EDGES[name]
    e = np.array((XS, ye, ze))
    n1, n2 = np.array((0., 0., n1z)), np.array((0., n2y, 0.))
    f1c = e + n1 * GUARD_T / 2 + n2 * (GUARD_T - GUARD_FL) / 2
    f1h = np.array((GUARD_LEN / 2, (GUARD_FL + GUARD_T) / 2, GUARD_T / 2))
    f2c = e + n2 * GUARD_T / 2 - n1 * GUARD_FL / 2
    f2h = np.array((GUARD_LEN / 2, GUARD_T / 2, GUARD_FL / 2))
    return e, n1, n2, (f1c, f1h), (f2c, f2h)


def guard_magnets(name):
    """Magnet points (world, at the seat) on each flange's inner face and the face normal each pulls toward."""
    e, n1, n2, _, _ = guard_boxes(name)
    out = []
    for dx in (-.030, .030):
        out.append((e + np.array((dx, 0., 0.)) - n2 * .025 + n1 * .0005, -n1))
        out.append((e + np.array((dx, 0., 0.)) - n1 * .025 + n2 * .0005, -n2))
    return out


def magnet_force(points):
    """Pull (N) on each magnet point toward the bar: F0 / (1 + g / g0)^2 within MAG_RANGE of its surface."""
    p = np.asarray(points, float)
    q = np.clip(p, BAR_LO, BAR_HI)
    d = p - q
    g = np.linalg.norm(d, axis=1)
    inside = g < 1e-6
    direction = np.where(inside[:, None], 0., -d / np.maximum(g, 1e-9)[:, None])
    if inside.any():                                   # on or in the surface: pull along the nearest face's normal
        lo, hi = p - BAR_LO, BAR_HI - p
        for i in np.nonzero(inside)[0]:
            k = int(np.argmin(np.concatenate((lo[i], hi[i]))))
            n = np.zeros(3)
            n[k % 3] = -1. if k < 3 else 1.
            direction[i] = -n
    mag = np.where(g < MAG_RANGE, MAG_F0 / (1. + g / MAG_G0) ** 2, 0.)
    return direction * mag[:, None], g


# -- the sling's starting shape ----------------------------------------------------------------------------------
APEX_Y = .009                   # the two apex bars side by side on the bowl's floor
APEX_Z = HOOK_O[2] - math.sqrt((BOWL_R_IN - FIT_BAR_R) ** 2 - APEX_Y ** 2)
OFF = GUARD_T + WEB_T / 2 + .0007     # web centreline outside the guards


def _corner(side, level):
    s = 1. if side == "L" else -1.
    z = Z_TOP + OFF / math.sqrt(2) if level == "T" else Z_BOT - OFF / math.sqrt(2)
    return np.array((s * (HALF + OFF / math.sqrt(2)), z))


def path_points(bulge):
    """Web centreline in (y, z), from fitting A's base bar (left) to fitting B's (right): over the top corner, down
    the side bowed out by ``bulge``, under the bar sagging to the table, and back up the other side."""
    pts = []
    for side, s in (("L", 1.), ("R", -1.)):
        apex = np.array((s * APEX_Y, APEX_Z))
        tl = _corner(side, "T")
        u = (tl - apex) / np.linalg.norm(tl - apex)
        base = apex + FIT_H * u
        bl = _corner(side, "B")
        mid = np.array((s * (HALF + OFF + bulge), (Z_TOP + Z_BOT) / 2))
        floor_y = s * .028
        seq = [base, tl, mid, bl, np.array((floor_y, WEB_T / 2 + .0006))]
        pts.append(seq)
    left, right = pts
    return left + right[::-1], left[0], right[0]


def _dense(poly, step=2e-4):
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.linalg.norm(b - a) / step))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)


def chord_walk(poly, length, count):
    """``count`` + 1 hinge points along ``poly`` with chord ``length`` between neighbours."""
    d = _dense(poly)
    pts = [d[0]]
    j = 0
    for _ in range(count):
        while j < len(d) - 1 and np.linalg.norm(d[j] - pts[-1]) < length:
            j += 1
        if np.linalg.norm(d[j] - pts[-1]) < length:
            return np.array(pts), False
        a, b = d[j - 1], d[j]                            # the crossing lies between a and b: solve |p - last| = L
        lo, hi = 0., 1.
        for _ in range(50):
            m = (lo + hi) / 2
            if np.linalg.norm(a + m * (b - a) - pts[-1]) < length:
                lo = m
            else:
                hi = m
        pts.append(a + lo * (b - a))
    return np.array(pts), True


def start_shape():
    """Hinge points H_0..H_N (y, z) with H_N on fitting B's base bar, found by bisection on the sides' bulge."""
    def residual(b):
        poly, _, end = path_points(b)
        pts, ok = chord_walk(poly, LINK_L, N_LINKS)
        return (_arc_left(poly, pts[-1]) if ok else -1.), pts, end

    lo, hi = 0., .12
    for _ in range(50):
        m = (lo + hi) / 2
        if residual(m)[0] > 0:
            hi = m
        else:
            lo = m
    r, pts, end = residual(hi)
    gap = float(np.linalg.norm(pts[-1] - end))
    pts = pts.copy()
    pts[-1] = end
    return pts, hi, gap


def _arc_left(poly, p):
    d = _dense(poly)
    k = int(np.argmin(np.linalg.norm(d - p, axis=1)))
    return float(np.linalg.norm(np.diff(d[k:], axis=0), axis=1).sum())


# -- body frames of the sling at the start (shared by the USD builder and the probe's reset) ----------------------
def _yz_frame(d2):
    d = np.array((0., d2[0], d2[1]))
    d /= np.linalg.norm(d)
    x = np.array((1., 0., 0.))
    return np.column_stack((x, d, np.cross(x, d)))


def start_bodies():
    """[(name, position, rotation)] for fitA, link_00..link_NN-1, fitB. Every body's local y runs along the web
    from fitting A toward fitting B (fitting A's apex is at local y = -FIT_H, fitting B's at +FIT_H), so every
    hinge is straight at zero."""
    H, _, _ = start_shape()
    apA, apB = np.array((APEX_Y, APEX_Z)), np.array((-APEX_Y, APEX_Z))
    out = [("fitA", np.array((XS, *H[0])), _yz_frame(H[0] - apA))]
    for i in range(N_LINKS):
        a, b = H[i], H[i + 1]
        out.append((f"link_{i:02d}", np.array((XS, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2)), _yz_frame(b - a)))
    out.append(("fitB", np.array((XS, *H[-1])), _yz_frame(apB - H[-1])))
    return out, H


def start_hinge_angles():
    """Bend (rad) at hinge_00..hinge_NN about +x, body i to body i+1."""
    bodies, _ = start_bodies()
    out = []
    for (_, _, R0), (_, _, R1) in zip(bodies[:-1], bodies[1:]):
        d0, d1 = R0[:, 1], R1[:, 1]
        out.append(math.atan2(d0[1] * d1[2] - d0[2] * d1[1], d0 @ d1))
    return out
