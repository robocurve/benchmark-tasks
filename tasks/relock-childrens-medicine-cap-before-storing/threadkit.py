"""threadkit: the bottle's threaded neck and its child-resistant cap as convex-hull colliders, so the cap goes down to
its seat only by turning it on a real thread.

The neck carries a single-start, right-hand male thread of one turn (a GPI "400" finish) and the cap a matching female
thread inside its bore. Each thread ridge is a helix of short convex hulls (one per SEG_DEG of turn) with a trapezoid
section (60-degree flanks, wider at the root than at the crest), each hull's end faces shared with its neighbors', so a
ridge slides along the other with no ledge at the seams. The ridge's height runs out to nothing over the last RUN_DEG
at both ends, as a molded thread starts. The cap's bore is a ring of convex wedges with a lead-in chamfer at its mouth;
its top is one convex disc. The neck's core is one convex cylinder with a chamfered tip, the shoulder one convex
frustum. Nothing holds the cap on but contact: no joint, spring, capture or snap.

Frames:
- neck frame: origin on the axis where the neck leaves the shoulder, z out along the axis toward the mouth;
- cap frame: origin at the middle of the cap's mouth (its open end), z along the axis toward the cap's closed top.
The cap is screwed fully down (seated) when the two frames share their axes and the cap's origin is at neck z SEAT_Z
(its inner top face then rests on the neck's lip). A right-hand thread: seen from above, turning the cap clockwise
screws it down.
"""

from __future__ import annotations

import math

import numpy as np

import yamkit as kit

# -- dimensions (m) ----------------------------------------------------------------------------------------------------
# The neck is the GPI/SPI 24-400 finish: thread outside diameter T 23.67 mm, neck outside diameter E 21.54 mm, finish
# height H 10.16 mm, lip to the thread's top S 1.17 mm (SGS neck finish library), 8 threads per inch (3.175 mm
# pitch; eBottles closure guide), one turn of thread ("400": one thread turn). Thread depth (T - E) / 2 = 1.065 mm.
PITCH = 0.0254 / 8
TAN30 = math.tan(math.radians(30.0))
R_CREST = 0.02367 / 2  # male crest radius (T)
R_ROOT = 0.02154 / 2  # male root radius (E): the neck's core
R_BORE = R_CREST + 0.0004  # the cap's bore: the female thread's root
R_FCREST = R_ROOT + 0.0003  # the female crest
HW_MALE, HW_FEMALE = 0.00045, 0.00052  # each ridge's half-width at its crest: 0.35 mm of axial play between flanks
NECK_L = 0.01016  # neck length, shoulder to lip (H)
TIP_R, TIP_CHAMFER = R_ROOT - 0.0006, 0.0006  # the lip's chamfer: radius at the lip, and its length
SHOULDER_L = 0.020  # the shoulder: a cone from the body's radius up to SHOULDER_TOP_R at the neck
SHOULDER_TOP_R = 0.0125
S_TOP = 0.00117  # lip to the top of the thread (S)
TURNS_MALE = 1.0
RUN_DEG = 45.0  # the ridge's height runs out over this many degrees at each end
HW_ROOT_M = HW_MALE + TAN30 * (R_CREST - R_ROOT)
Z_MALE = NECK_L - S_TOP - HW_ROOT_M - PITCH * TURNS_MALE  # the male ridge's first center height
SEG_DEG = 10.0  # degrees of turn per ridge hull
EMBED = 0.0002  # each ridge's root is sunk this far into its own part (no seam at the root)

# The cap: a 24-400 push-down-and-turn child-resistant cap (The Cary Company 68WCB2: 1.22 in outside diameter x 0.67 in
# tall, 30.99 x 17.02 mm). Its two shells are one rigid body here (the lugs between them are not modelled: README).
R_CAP = 0.03099 / 2
H_CAP = 0.01702
CAP_G = 4.0
H_IN = NECK_L - 0.0004  # depth of the bore, mouth to the inner top face: seated, the mouth stands 0.4 mm off the shoulder
MOUTH_R, LEAD_IN = R_BORE + 0.0010, 0.0012  # the bore's lead-in: radius at the mouth, and its depth
SEAT_Z = NECK_L - H_IN  # cap origin's neck-z when seated
HW_ROOT_F = HW_FEMALE + TAN30 * (R_BORE - R_FCREST)
FEMALE_LOW = LEAD_IN + HW_ROOT_F  # the female ridge starts past the lead-in ...
# ... at the first height where, seated, it sits half a pitch from the male ridge (the two ridges interleave)
Z_FEMALE = FEMALE_LOW + ((Z_MALE + PITCH / 2 - SEAT_Z - FEMALE_LOW) % PITCH)
TURNS_FEMALE = 1.0  # female ridge length in turns
N_SIDES = 32  # sides of the round hulls (a 32-gon's flats sit 0.5% inside its corners)


def cap_mass_props():
    """(center of mass (cap frame), principal moments (kg m^2)) of the cap taken as a thin-walled cup of the listed
    size and mass: its round side and its closed top share the mass by area."""
    m, R, H = CAP_G / 1000.0, R_CAP, H_CAP
    a_side, a_top = 2 * math.pi * R * H, math.pi * R * R
    ms, mt = m * a_side / (a_side + a_top), m * a_top / (a_side + a_top)
    zc = (ms * H / 2 + mt * H) / m
    izz = ms * R * R + mt * R * R / 2
    ixx = ms * (R * R / 2 + H * H / 12) + ms * (H / 2 - zc) ** 2 + mt * R * R / 4 + mt * (H - zc) ** 2
    return (0.0, 0.0, zc), (ixx, ixx, izz)


def _hull(points) -> kit.Collider:
    """A convex-hull collider of `points` (any frame; center = their centroid, triangles from their convex hull)."""
    from scipy.spatial import ConvexHull

    p = np.asarray(points, float)
    c = p.mean(axis=0)
    tri = ConvexHull(p).simplices
    rel = p - c
    # outward winding
    faces = []
    for t in tri:
        a, b, d = rel[t[0]], rel[t[1]], rel[t[2]]
        n = np.cross(b - a, d - a)
        faces.append(tuple(int(i) for i in (t if n @ ((a + b + d) / 3) >= 0 else t[::-1])))
    return kit.Collider(tuple(float(v) for v in c), hull=tuple(tuple(float(v) for v in q) for q in rel),
                        faces=tuple(faces))


def _ring(r, z, a0, a1):
    return [(r * math.cos(a), r * math.sin(a), z) for a in (a0, a1)]


def _polygon_r(r_flat: float, n: int = N_SIDES) -> float:
    """Corner radius of an n-gon whose flats stand at r_flat (so a bore built of flats is no tighter than r_flat)."""
    return r_flat / math.cos(math.pi / n)


def prism(r: float, z0: float, z1: float, n: int = N_SIDES) -> list:
    """Points of an n-gon prism (corners at r) about z, from z0 to z1."""
    a = [2 * math.pi * k / n for k in range(n)]
    return [(r * math.cos(t), r * math.sin(t), z) for z in (z0, z1) for t in a]


def frustum(r0: float, z0: float, r1: float, z1: float, n: int = N_SIDES) -> list:
    a = [2 * math.pi * k / n for k in range(n)]
    return [(r * math.cos(t), r * math.sin(t), z) for r, z in ((r0, z0), (r1, z1)) for t in a]


def _ridge(r_base, r_crest, hw_crest, z0, turns, run_deg=RUN_DEG):
    """Hull point sets of a right-hand helical ridge with 60-degree flanks: its crest (half-width hw_crest) at r_crest,
    widening toward r_base; its crest runs down to r_base over run_deg at each end. Centered on z0 + PITCH * angle /
    2 pi."""
    n = max(1, int(round(turns * 360.0 / SEG_DEG)))
    total = turns * 360.0
    out = []

    def hw(r):
        return hw_crest + TAN30 * abs(r_crest - r)

    def section(deg):
        th = math.radians(deg)
        zc = z0 + PITCH * deg / 360.0
        run = min(1.0, deg / run_deg, (total - deg) / run_deg) if run_deg > 0 else 1.0
        rt = r_base + (r_crest - r_base) * max(0.0, run)
        c, s = math.cos(th), math.sin(th)
        pts = [(r_base * c, r_base * s, zc - hw(r_base)), (r_base * c, r_base * s, zc + hw(r_base))]
        if abs(rt - r_base) > 1e-6:
            pts += [(rt * c, rt * s, zc - hw(rt)), (rt * c, rt * s, zc + hw(rt))]
        return pts

    for k in range(n):
        d0, d1 = k * total / n, (k + 1) * total / n
        pts = section(d0) + section(d1)
        if len(pts) >= 5:
            out.append(pts)
    return out


def male_ridge() -> list:
    return _ridge(R_ROOT - EMBED, R_CREST, HW_MALE, Z_MALE, TURNS_MALE)


def female_ridge() -> list:
    return _ridge(R_BORE + EMBED, R_FCREST, HW_FEMALE, Z_FEMALE, TURNS_FEMALE)


def neck_point_sets(body_r: float) -> list:
    """The neck's and shoulder's hull point sets, neck frame: shoulder cone, core with chamfered tip, male ridge."""
    sets = [frustum(body_r, -SHOULDER_L, SHOULDER_TOP_R, 0.0)]
    rr = _polygon_r(R_ROOT)
    sets.append(prism(rr, -0.001, NECK_L - TIP_CHAMFER))  # the core (64 points: within PhysX's hull limit)
    sets.append(frustum(rr, NECK_L - TIP_CHAMFER, TIP_R, NECK_L))  # the chamfered tip, its own hull
    sets += male_ridge()
    return sets


def cap_point_sets() -> list:
    """The cap's hull point sets, cap frame: N_SIDES bore wedges (lead-in at the mouth), the top disc, female ridge."""
    sets = []
    n = N_SIDES
    ri, rm, ro = _polygon_r(R_BORE, n), _polygon_r(MOUTH_R, n), R_CAP
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        pts = _ring(rm, 0.0, a0, a1) + _ring(ri, LEAD_IN, a0, a1) + _ring(ri, H_IN, a0, a1)
        pts += _ring(ro, 0.0, a0, a1) + _ring(ro, H_IN, a0, a1)
        sets.append(pts)
    sets.append(prism(R_CAP, H_IN, H_CAP))
    sets += female_ridge()
    return sets


def colliders(point_sets, to_frame=None) -> tuple:
    """Colliders from point sets, each point mapped by `to_frame` (a function of an (n, 3) array) if given."""
    out = []
    for pts in point_sets:
        p = np.asarray(pts, float)
        if to_frame is not None:
            p = to_frame(p)
        out.append(_hull(p))
    return tuple(out)


# -- reading the cap on the neck ----------------------------------------------------------------------------------------
def rotm(q_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat(R) -> tuple:
    """(x, y, z, w) of a rotation matrix (Shepperd's method: stable near half turns)."""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = 2 * math.sqrt(1 + t)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(max(1e-12, 1 + R[i, i] - R[j, j] - R[k, k]))
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def relative(neck_origin, R_neck, cap_origin, R_cap) -> dict:
    """The cap's pose in the neck frame: `rise` (m, how far the cap's origin is out along the axis from its seated
    height; 0 seated, positive unscrewed), `off` (m, the cap's axis from the neck's), `tilt` (degrees between the
    axes) and `turn` (degrees, the cap's turn about the neck axis)."""
    Rn = np.asarray(R_neck, float)
    p = Rn.T @ (np.asarray(cap_origin, float) - np.asarray(neck_origin, float))
    Rr = Rn.T @ np.asarray(R_cap, float)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(Rr[2, 2])))))
    return {"rise": float(p[2] - SEAT_Z), "off": float(math.hypot(p[0], p[1])), "tilt": tilt,
            "turn": math.degrees(math.atan2(float(Rr[1, 0]), float(Rr[0, 0]))), "p": p}
