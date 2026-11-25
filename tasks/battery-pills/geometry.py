"""battery-pills geometry, pure NumPy (no Isaac). Every size comes from params.json (the card's numbers; the card's
estimates stay estimates, see SOURCED_VALUES.md).

Frames
- World (yam_arena station): x forward, y left, z up, metres; arm bases at (0, +-0.305); table top z 0.7475.
  The card frame is the world's x, y with heights above the table top: world z = TABLE_TOP + card z.
- Each body's own frame: origin on its axis at its lowest point (tray, dish, jar) or at its centre (tablet, cell),
  axes parallel to the world at rest.

Bodies as convex pieces (PhysX convex hulls; no concave collider anywhere):
- tray: floor slab + 4 walls (boxes).
- dish: a foot frustum (r 35 -> 38.2 over the 4 mm floor) + 32 wall segments (the 3 mm flared wall from the floor
  top at r 32 to the rim at r 57.5 inner / 60.5 outer, 32 high).
- jar: floor disc + 32 wall segments (straight cylinder, outer r 38 = the 76 mm top, inner r 35 = the Stowed radius).
- tablet: 24-sided prism, 9.5 across x 4.0; cell: 24-sided prism, 7.9 across x 3.6.
"""
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
P = json.loads((HERE / 'params.json').read_text())
MM = 1e-3
TABLE_TOP = P['rig']['table_top_world_z_mm'] * MM      # 0.7475 = yam_arena.task_file.TABLE_TOP['station']
TR, DI, JA, TB, CE, LA = P['tray'], P['dish'], P['holder'], P['tablets'], P['cell'], P['layout']

# -- sizes (mm) -----------------------------------------------------------------------------------------------
TRAY_X, TRAY_Y, TRAY_H = TR['outer_mm']['x'], TR['outer_mm']['y'], TR['outer_mm']['z']
TRAY_FLOOR = TR['floor_top_mm_estimate']             # floor top, estimate
TRAY_WALL = TR['wall_mm_estimate']                   # estimate
TRAY_MASS_G = TR['mass_g_estimate']                  # estimate

DISH_TOP_R = DI['top_diameter_mm'] / 2               # 60.5 outer rim radius
DISH_H = DI['height_mm']                              # 32
DISH_FLOOR_R = DI['floor_radius_mm']                  # 32 inner floor radius (estimate)
DISH_FLOOR_TOP = DI['floor_top_mm_estimate']          # 4
DISH_WALL = DI['wall_mm_estimate']                    # 3
DISH_IN_R = P['scoring']['dish_inner_radius_mm']      # 57.5 rim inner radius
DISH_MASS_G = DI['mass_g_upper']                      # 61 (upper bound from the listing's case weight)
DISH_SEGMENTS = 32

JAR_H = JA['height_mm']                               # 254
JAR_OUT_R = JA['top_diameter_mm'] / 2                 # 38 (top opening as the body: body width not published)
JAR_IN_R = JA['stowed_radius_mm']                     # 35 (estimate until G1)
JAR_FLOOR = 3.0                                       # floor thickness, estimate (no published value)
JAR_MASS_G = JA['mass_g_upper']                       # 91 (upper bound, shipping weight)
JAR_SEGMENTS = 32

TAB_R = TB['diameter_mm_estimate'] / 2                # 4.75
TAB_H = TB['thickness_mm_estimate']                   # 4.0
TAB_MASS_G = TB['mass_g_estimate']                    # 0.35
CELL_R = CE['diameter_mm'] / 2                        # 3.95
CELL_H = CE['height_mm']                              # 3.6
CELL_MASS_G = 0.53                                    # Rayovac 312 Zero Mercury datasheet: 'Approximate Weight: .53 grams'
                                                      # (SOURCED_VALUES.md; replaces the card's 0.8 g estimate)
PRISM_SIDES = 24

SLOT_R = LA['ring_radius_mm']                         # 18
N_TABLETS = TB['count']


# -- dish profile ----------------------------------------------------------------------------------------------
def dish_outer_r(z):
    """Outer surface radius (mm) at height z above the dish bottom (straight flare from r 35 at 0 to 60.5 at 32)."""
    r0 = DISH_FLOOR_R + DISH_WALL
    return r0 + (DISH_TOP_R - r0) * np.clip(np.asarray(z, float), 0, DISH_H) / DISH_H


def dish_inner_r(z):
    """Inner surface radius (mm) at height z (the card's inner wall from (32, 4) to (57.5, 32))."""
    z = np.clip(np.asarray(z, float), DISH_FLOOR_TOP, DISH_H)
    return DISH_FLOOR_R + (DISH_IN_R - DISH_FLOOR_R) * (z - DISH_FLOOR_TOP) / (DISH_H - DISH_FLOOR_TOP)


WALL_SLOPE_DEG = math.degrees(math.atan2(DISH_H, DISH_TOP_R - DISH_FLOOR_R - DISH_WALL))   # outer flare, ~51.5 deg


# -- convex pieces (body frame, mm -> m) -----------------------------------------------------------------------
def box_pieces(centre, size):
    c, h = np.asarray(centre, float), np.asarray(size, float) / 2
    return np.array([c + h * [sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * MM


def tray_pieces():
    """(name, 8 corner points in m) for the tray: floor slab and four walls. Body origin = tray centre at its bottom."""
    w, X, Y, H, F = TRAY_WALL, TRAY_X, TRAY_Y, TRAY_H, TRAY_FLOOR
    return [('floor', box_pieces((0, 0, F / 2), (X, Y, F))),
            ('wall_near', box_pieces((-(X - w) / 2, 0, H / 2), (w, Y, H))),
            ('wall_far', box_pieces(((X - w) / 2, 0, H / 2), (w, Y, H))),
            ('wall_left', box_pieces((0, (Y - w) / 2, H / 2), (X - 2 * w, w, H))),
            ('wall_right', box_pieces((0, -(Y - w) / 2, H / 2), (X - 2 * w, w, H)))]


def _ring_segment(r_in_lo, r_out_lo, z_lo, r_in_hi, r_out_hi, z_hi, a0, a1):
    pts = []
    for a in (a0, a1):
        c, s = math.cos(a), math.sin(a)
        for r, z in ((r_in_lo, z_lo), (r_out_lo, z_lo), (r_in_hi, z_hi), (r_out_hi, z_hi)):
            pts.append((r * c, r * s, z))
    return np.array(pts) * MM


def disc_points(r_lo, r_hi, z_lo, z_hi, n):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    lo = np.c_[r_lo * np.cos(a), r_lo * np.sin(a), np.full(n, z_lo)]
    hi = np.c_[r_hi * np.cos(a), r_hi * np.sin(a), np.full(n, z_hi)]
    return np.vstack([lo, hi]) * MM


def dish_pieces():
    """Foot frustum (the floor) + wall segments. The polygon's outer vertices are pushed out by 1/cos(pi/n) so its
    flats sit on the profile. Body origin = dish axis at its bottom."""
    n = DISH_SEGMENTS
    k = 1.0 / math.cos(math.pi / n)
    out = [('floor', disc_points(dish_outer_r(0) * k, dish_outer_r(DISH_FLOOR_TOP) * k, 0.0, DISH_FLOOR_TOP, n))]
    z_lo, z_hi = DISH_FLOOR_TOP, DISH_H
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        out.append((f'wall{i:02d}', _ring_segment(dish_inner_r(z_lo), dish_outer_r(z_lo) * k, z_lo,
                                                  dish_inner_r(z_hi), dish_outer_r(z_hi) * k, z_hi, a0, a1)))
    return out


def jar_pieces():
    n = JAR_SEGMENTS
    k = 1.0 / math.cos(math.pi / n)
    out = [('floor', disc_points(JAR_OUT_R * k, JAR_OUT_R * k, 0.0, JAR_FLOOR, n))]
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        out.append((f'wall{i:02d}', _ring_segment(JAR_IN_R, JAR_OUT_R * k, JAR_FLOOR, JAR_IN_R, JAR_OUT_R * k, JAR_H, a0, a1)))
    return out


def prism_points(r, h, n=PRISM_SIDES):
    """A flat cylinder (tablet, cell) as an n-sided prism, centred at the origin, flats on the true radius."""
    k = 1.0 / math.cos(math.pi / n)
    return disc_points(r * k, r * k, -h / 2, h / 2, n)


def convex_volume(pts):
    """Volume of the convex hull of ``pts`` (m^3): a few points, so the Delaunay-free tetra fan over the hull faces
    is replaced by a simple Monte-Carlo-free formula for our prisms / boxes / segments (all are 'extrusions')."""
    pts = np.asarray(pts, float)
    # every piece here is two (or more) congruent rings of points; use the divergence theorem on the convex hull
    # via scipy-free quickhull is overkill: the pieces are frusta / boxes / prisms with planar top and bottom.
    z = pts[:, 2]
    lo, hi = pts[np.isclose(z, z.min())], pts[np.isclose(z, z.max())]
    if len(lo) < 3 or len(hi) < 3:
        raise ValueError('piece has no planar top and bottom')

    def area(poly):
        c = poly[:, :2].mean(0)
        ang = np.arctan2(poly[:, 1] - c[1], poly[:, 0] - c[0])
        p = poly[np.argsort(ang), :2]
        x, y = p[:, 0], p[:, 1]
        return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    a0, a1 = area(lo), area(hi)
    hgt = z.max() - z.min()
    # prismatoid: V = h/6 (A0 + 4 Am + A1), Am from the mid-section of corresponding points (same ordering by angle)
    mid = (lo[np.argsort(np.arctan2(lo[:, 1] - lo[:, 1].mean(), lo[:, 0] - lo[:, 0].mean()))] +
           hi[np.argsort(np.arctan2(hi[:, 1] - hi[:, 1].mean(), hi[:, 0] - hi[:, 0].mean()))]) / 2 \
        if len(lo) == len(hi) else None
    am = area(mid) if mid is not None else (math.sqrt(a0 * a1))
    return hgt / 6 * (a0 + 4 * am + a1)


def body_volume(pieces):
    return sum(convex_volume(p) for _, p in pieces)


# -- per-trial draws ------------------------------------------------------------------------------------------
NOMINAL = dict(tray=(0.0, 0.0), dish=(0.0, 0.0), holder=(0.0, 0.0), phi=0.0, cell=3, empty=6)


def draws(seed):
    """One draw per trial, as the card's Randomisation. Seed 0 = the scorer's nominal draw (cell slot 3, empty 6)."""
    if seed == 0:
        return dict(NOMINAL)
    rng = np.random.default_rng(1000 + seed)
    u = lambda r: float(rng.uniform(r[0], r[1]))  # noqa: E731
    tray = (u(TR['draw_dx_mm']), u(TR['draw_dy_mm']))
    dish = (u(DI['draw_dx_mm']), u(DI['draw_dy_mm']))
    holder = (u(JA['draw_dx_mm']), u(JA['draw_dy_mm']))
    phi = u(LA['phi_draw_deg'])
    cell = int(rng.integers(0, 7))
    empty = int(rng.choice([k for k in range(7) if k != cell]))
    return dict(tray=tray, dish=dish, holder=holder, phi=phi, cell=cell, empty=empty)


def slots(cx, cy, phi):
    s = [(cx, cy)]
    for k in range(6):
        a = math.radians(phi + 60 * k)
        s.append((cx + SLOT_R * math.cos(a), cy + SLOT_R * math.sin(a)))
    return s


def layout(seed):
    """Start positions (card frame, mm): tray / dish / jar centres, cell and tablet centres (x, y, z above table)."""
    d = draws(seed)
    tray = (TR['start_centre_mm'][0] + d['tray'][0], TR['start_centre_mm'][1] + d['tray'][1])
    dish = (DI['start_centre_mm'][0] + d['dish'][0], DI['start_centre_mm'][1] + d['dish'][1])
    jar = (JA['start_centre_mm'][0] + d['holder'][0], JA['start_centre_mm'][1] + d['holder'][1])
    sl = slots(*dish, d['phi'])
    cell = (*sl[d['cell']], DISH_FLOOR_TOP + CELL_H / 2)
    tabs = [(*sl[j], DISH_FLOOR_TOP + TAB_H / 2) for j in range(7) if j not in (d['cell'], d['empty'])]
    return dict(draw=d, tray=tray, dish=dish, jar=jar, cell=cell, tablets=tabs, slots=sl)


def world(xy_or_xyz, z_card=None):
    """Card (mm) -> world (m)."""
    v = list(xy_or_xyz)
    z = v[2] if len(v) > 2 and z_card is None else (z_card or 0.0)
    return (v[0] * MM, v[1] * MM, TABLE_TOP + z * MM)


def card(p):
    """World (m) -> card (mm)."""
    return np.array([p[0] / MM, p[1] / MM, (p[2] - TABLE_TOP) / MM])


# -- rotations -------------------------------------------------------------------------------------------------
def quat_xyzw_to_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat_wxyz(r):
    r = np.asarray(r, float)
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return np.array([0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = np.zeros(4)
    q[0] = (r[k, j] - r[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (r[j, i] + r[i, j]) / s
    q[1 + k] = (r[k, i] + r[i, k]) / s
    return q


def yaw_deg(rot):
    return math.degrees(math.atan2(rot[1, 0], rot[0, 0]))


def tilt_deg(rot):
    return math.degrees(math.acos(max(-1.0, min(1.0, rot[2, 2]))))
