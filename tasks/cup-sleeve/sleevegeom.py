"""Geometry of the cup and the corrugated sleeve. NumPy only (no simulator), so the scorer's tests can use it.

Units: metres. The station's world frame: x away from the robot, y to its left, z up.

The cup: a Dart Solo 316W-type 16 oz single-wall paper hot cup (88.9 mm across the lip, 58.4 mm across the bottom,
134.6 mm tall) with a sip lid. The sleeve: a kraft E-flute corrugated universal sleeve, 64 mm tall and 1.5 mm thick,
whose cone matches the cup's taper (64.1 mm inside at its lower edge, 78.6 mm at its upper edge), so that on this cup
it wedges with its lower edge about 25 mm above the cup's base.

The sleeve is modelled as N_FACETS flat strips round a 16-sided pyramid frustum (each strip's middle line touches the
cone), hinged to each other along the generator lines on their inner faces: a closed loop. Flat-packed, the loop is
folded flat along two of those lines (the factory creases, generators 0 and 8): strips 0-7 form one layer, 8-15 the
other, lying on it. All hinge lines meet at the cone's apex, in the flat pack as in the opened sleeve.
"""

from __future__ import annotations

import math

import numpy as np

# -- the cup ------------------------------------------------------------------------------------------------------------
CUP_D_BOTTOM = 0.0584
CUP_D_LIP = 0.0889
CUP_H = 0.1346
CUP_SLOPE = (CUP_D_LIP - CUP_D_BOTTOM) / 2 / CUP_H  # radius gained per metre of height (0.113)
LID_D = 0.093
LID_H = 0.0185
CUP_MASS = 0.434  # 14 g cup, 4 g lid, 400 g water, 16 g in the lid's water film and rim estimate
CUP_COM_Z = 0.058


def cup_radius(h: float) -> float:
    """The cup wall's outer radius at height h above its base (m)."""
    return CUP_D_BOTTOM / 2 + CUP_SLOPE * float(np.clip(h, 0.0, CUP_H))


# -- the sleeve ---------------------------------------------------------------------------------------------------------
N_FACETS = 16
SLEEVE_H = 0.064  # height along the cup's axis
BOARD_T = 0.0015  # E-flute corrugated board
SLEEVE_MASS = 0.004
SEAT_H = 0.025  # where its lower edge sits on this cup when it fits (wedged), above the cup's base
CREASES = (0, 8)  # the generator lines folded flat in the flat pack
R_IN_BOTTOM = cup_radius(SEAT_H)  # 32.05 mm: inner radius at its lower edge
R_IN_TOP = cup_radius(SEAT_H + SLEEVE_H)  # 39.30 mm


def _frustum():
    """The opened sleeve's inner surface: the pyramid's corner radii at the lower and upper edges, the apex below the
    lower edge (distance along the axis), and the fan angle of one strip when laid flat."""
    c = math.cos(math.pi / N_FACETS)
    rb, rt = R_IN_BOTTOM / c, R_IN_TOP / c  # corner radii, so that each strip's middle line lies on the cone
    apex_below = rb * SLEEVE_H / (rt - rb)
    a = np.array((rb, 0.0, apex_below))
    phi = 2 * math.pi / N_FACETS
    b = np.array((rb * math.cos(phi), rb * math.sin(phi), apex_below))
    fan = math.acos(float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b))))
    return rb, rt, apex_below, fan


RB, RT, APEX_BELOW, FAN = _frustum()
GEN_B = math.hypot(RB, APEX_BELOW)  # apex to the lower edge along a hinge line
GEN_T = math.hypot(RT, APEX_BELOW + SLEEVE_H)
CREASE_SPAN_B = 2 * GEN_B * math.sin(4 * FAN)  # crease to crease, flat-packed, at the lower edge (about 0.101)
CREASE_SPAN_T = 2 * GEN_T * math.sin(4 * FAN)  # and at the upper edge (about 0.124)


def flat_generators(origin, gap: float = 0.00015):
    """The hinge lines of the flat-packed sleeve standing on its narrow edge, its layers in the plane x = origin.x (the
    plane's normal along x), the apex below: for generator j (0..16, 16 = 0), the (bottom point, top point) on the
    inner faces' common plane. ``origin`` is the point midway between the two creases' lower ends."""
    o = np.asarray(origin, float)
    out = []
    for j in range(N_FACETS + 1):
        psi = (j - 4) * FAN if j <= 8 else (12 - j) * FAN
        d = np.array((0.0, math.sin(psi), math.cos(psi)))
        apex = o - np.array((0.0, 0.0, GEN_B * math.cos(4 * FAN)))
        out.append((apex + GEN_B * d, apex + GEN_T * d))
    return out


def facet_layer(k: int) -> int:
    """+1: the layer on the +x side (strips 0-7); -1: the other."""
    return 1 if k < 8 else -1


def flat_facets(origin, gap: float = 0.00015):
    """Each strip's eight corners (inner face 4, outer face 4) in the flat pack, world frame."""
    g = flat_generators(origin)
    out = []
    for k in range(N_FACETS):
        s = facet_layer(k)
        (b0, t0), (b1, t1) = g[k], g[k + 1]
        inner = [b0, b1, t1, t0]
        n = np.array((s * 1.0, 0.0, 0.0))
        pts = [p + n * gap for p in inner] + [p + n * (gap + BOARD_T) for p in inner]
        out.append(np.array(pts))
    return out


def hinge_lines(origin):
    """Hinge k joins strip k-1 (body0; strip 15 for k = 0) to strip k (body1), along generator k: (point, unit axis
    from the lower edge up). Every axis points up its line, so that a turn about it has one sense for every hinge."""
    g = flat_generators(origin)
    out = []
    for k in range(N_FACETS):
        b, t = g[k]
        d = (t - b) / np.linalg.norm(t - b)
        out.append(((b + t) / 2, d))
    return out


# -- measurements on the running sleeve (strip poses in, numbers out) ---------------------------------------------------------

def rotmat(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def facet_points(local_pts, poses):
    """World corners of each strip: ``local_pts[k]`` (8 x 3, body frame), ``poses[k]`` = (pos, quat_xyzw)."""
    return [p + local_pts[k] @ rotmat(q).T for k, (p, q) in enumerate(poses)]


def facet_features(pts_world):
    """Per strip: its inner face's lower-edge middle, centre and inward... (lower-mid, upper-mid, centre)."""
    out = []
    for p in pts_world:
        lo = (p[0] + p[1]) / 2
        hi = (p[2] + p[3]) / 2
        out.append((lo, hi, (lo + hi) / 2))
    return out


def opened_size(pts_world) -> float:
    """How far open the sleeve is: the smaller of the crease-to-crease distance (generators 0 and 8, mid-height) and
    the distance between the middles of strips 4 and 11 (which lie on each other in the flat pack)."""
    f = facet_features(pts_world)
    g0 = (pts_world[0][0] + pts_world[0][3]) / 2
    g8 = (pts_world[8][0] + pts_world[8][3]) / 2
    across = np.linalg.norm(f[4][2] - f[11][2])
    return float(min(np.linalg.norm(g0 - g8), across))


def on_cup(pts_world, cup_pos, cup_quat) -> dict:
    """The sleeve against the cup, in the cup's frame: lower-edge heights above the cup's base (per strip), the gap
    between each strip's inner middle line and the cup wall, and whether the loop goes round the cup's axis."""
    R = rotmat(cup_quat)
    c = np.asarray(cup_pos, float)
    f = facet_features(pts_world)
    lows, gaps, ang = [], [], []
    for lo, hi, mid in f:
        for p, store in ((lo, lows),):
            store.append(float(((p - c) @ R)[2]))
        ml = (mid - c) @ R
        r = math.hypot(ml[0], ml[1])
        gaps.append(r - cup_radius(ml[2]) if 0.0 < ml[2] < CUP_H else 1.0)
        ang.append(math.atan2(ml[1], ml[0]))
    turn = 0.0
    for k in range(N_FACETS):
        d = ang[(k + 1) % N_FACETS] - ang[k]
        turn += (d + math.pi) % (2 * math.pi) - math.pi
    around = abs(round(turn / (2 * math.pi))) == 1
    radial = [math.hypot(*(((m - c) @ R)[:2])) for _, _, m in f]
    return {"low": lows, "gap": gaps, "around": bool(around and max(radial) < 0.06)}
