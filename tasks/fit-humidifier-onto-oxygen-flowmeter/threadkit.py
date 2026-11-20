"""threadkit: the flowmeter's threaded oxygen outlet and the humidifier's wing nut as convex-hull colliders, so the
nut goes on only by turning it on a real thread (built the same way as capkit in the toothpaste task).

The outlet is a DISS 1240 oxygen connection: a 9/16-18 UNF male thread (9/16 in major diameter, 18 threads per inch,
single start, right hand). The thread's section is the Unified basic profile (60-degree flanks; fundamental triangle
H = 0.866 P; external thread depth 5H/8; crest flat P/8 outside and P/4 inside). Each thread ridge is a helix of
short convex hulls (one per SEG_DEG of turn), each hull's end faces shared with its neighbors', so a ridge slides
along the other with no ledge at the seams. The ridges run out to nothing over their last RUN_DEG at both ends, as a
cut thread starts. The nut's bore is a ring of convex wedges with a lead-in chamfer at its mouth. Nothing holds the
nut on but contact: no joint to the outlet, no spring, capture or snap.

Frames:
- outlet frame: origin on the axis at the flowmeter's flat underside (where the stub leaves it), z out along the stub
  (straight down in the scene);
- nut frame: origin at the middle of the nut's mouth (its open end, toward the flowmeter), z along the axis toward
  the nut's closed end (the humidifier's lid).
The nut is screwed fully on (seated) when the two frames share their axes and the nut's origin is at outlet z SEAT_Z
(its inner floor then rests on the stub's end). Right-hand thread: seen from below (the nut's closed end), turning the
nut clockwise screws it on; seen from above, counterclockwise (a positive turn about the world's up axis).
"""

from __future__ import annotations

import math

import numpy as np

import yamkit as kit

INCH = 0.0254
# -- the DISS 1240 thread, 9/16-18 UNF ---------------------------------------------------------------------------------
PITCH = INCH / 18.0  # 1.411 mm
_H = 0.8660254 * PITCH
TAN30 = math.tan(math.radians(30.0))
R_CREST = 0.5625 * INCH / 2  # male crest radius: 7.144 mm (14.29 mm major diameter)
R_ROOT = R_CREST - 5 * _H / 8  # male root radius (the stub's core): 6.380 mm
D2 = 2 * R_CREST - 0.6495 * PITCH  # basic pitch diameter: 13.37 mm
# Fit: the nut's root stands 0.25 mm off the male crest and its crest 0.20 mm off the male root.
R_BORE = R_CREST + 0.00025  # the nut's bore: the female thread's root
R_FCREST = R_ROOT + 0.00020  # the female crest
HW_MALE, HW_FEMALE = PITCH / 16, PITCH / 8  # each ridge's half-width at its crest (crest flats P/8 and P/4)
# The outlet stub: 8 mm of thread out of the hex body's underside, its end chamfered.
STUB_L = 0.0080
TIP_R, TIP_CHAMFER = R_ROOT - 0.0004, 0.0008
Z_MALE, TURNS_MALE = 0.0012, 4.0  # the male ridge: its first center height and its length in turns
SEG_DEG = 12.0  # degrees of turn per ridge hull: a hull's straight crest stands at most 0.04 mm inside the true helix
RUN_DEG = 45.0  # each ridge runs out over this many degrees at each end
EMBED = 0.0002  # each ridge's root is sunk this far into its own part
N_SIDES = 32  # sides of the round hulls

# -- the wing nut (Salter Labs 7600: "large, easy to grip quad style wing nut") -----------------------------------------
H_IN = 0.0075  # depth of the threaded bore, mouth to the inner floor
MOUTH_R, LEAD_IN = R_BORE + 0.0008, 0.0010  # the bore's lead-in chamfer: radius at the mouth, and its depth
HUB_R = 0.0110  # the nut's round hub, 22 mm across
NUT_H = 0.0130  # the hub's height, mouth to its closed end
WING_SPAN = 0.046  # across opposite wing tips
WING_T = 0.0035  # each wing's thickness
WING_H = 0.0120  # each wing's height along the axis (from the mouth)
N_WINGS = 4
SEAT_Z = STUB_L - H_IN  # the nut's origin at outlet z when seated: its mouth 0.5 mm short of the flowmeter's underside
FEMALE_LOW = LEAD_IN + HW_FEMALE + TAN30 * (R_BORE - R_FCREST)
Z_FEMALE = FEMALE_LOW + ((Z_MALE + PITCH / 2 - SEAT_Z - FEMALE_LOW) % PITCH)
TURNS_FEMALE = 2.75


def _hull(points) -> kit.Collider:
    """A convex-hull collider of `points` (center = their centroid, triangles from their convex hull)."""
    from scipy.spatial import ConvexHull

    p = np.asarray(points, float)
    c = p.mean(axis=0)
    tri = ConvexHull(p).simplices
    rel = p - c
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
    """Corner radius of an n-gon whose flats stand at r_flat."""
    return r_flat / math.cos(math.pi / n)


def prism(r: float, z0: float, z1: float, n: int = N_SIDES) -> list:
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


def stub_point_sets() -> list:
    """The outlet stub's hull point sets, outlet frame: core with chamfered tip, male ridge."""
    rr = _polygon_r(R_ROOT)
    sets = [prism(rr, -0.001, STUB_L - TIP_CHAMFER), frustum(rr, STUB_L - TIP_CHAMFER, TIP_R, STUB_L)]
    sets += male_ridge()
    return sets


def nut_point_sets() -> list:
    """The nut's hull point sets, nut frame: N_SIDES bore wedges (lead-in at the mouth), the closed end, the wings,
    the female ridge."""
    sets = []
    n = N_SIDES
    ri, rm, ro = _polygon_r(R_BORE, n), _polygon_r(MOUTH_R, n), HUB_R
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        pts = _ring(rm, 0.0, a0, a1) + _ring(ri, LEAD_IN, a0, a1) + _ring(ri, H_IN, a0, a1)
        pts += _ring(ro, 0.0, a0, a1) + _ring(ro, H_IN, a0, a1)
        sets.append(pts)
    sets.append(prism(HUB_R, H_IN, NUT_H))
    sets += wing_point_sets()
    sets += female_ridge()
    return sets


def wing_angles() -> list:
    """Each wing's direction (rad) about the nut's axis, in the nut frame."""
    return [2 * math.pi * k / N_WINGS for k in range(N_WINGS)]


def wing_point_sets() -> list:
    out = []
    for a in wing_angles():
        c, s = math.cos(a), math.sin(a)
        u, v = np.array([c, s, 0.0]), np.array([-s, c, 0.0])
        pts = []
        for r in (HUB_R - 0.001, WING_SPAN / 2):
            for w in (-WING_T / 2, WING_T / 2):
                for z in (0.0, WING_H):
                    pts.append(tuple(r * u + w * v + np.array([0.0, 0.0, z])))
        out.append(pts)
    return out


def colliders(point_sets, to_frame=None) -> tuple:
    out = []
    for pts in point_sets:
        p = np.asarray(pts, float)
        if to_frame is not None:
            p = to_frame(p)
        out.append(_hull(p))
    return tuple(out)


# -- reading the nut on the outlet ---------------------------------------------------------------------------------------
def rotm(q_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat(R) -> tuple:
    """(x, y, z, w) of a rotation matrix (Shepperd's method)."""
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


def relative(outlet_origin, R_outlet, nut_origin, R_nut) -> dict:
    """The nut's pose in the outlet frame: `rise` (m, how far the nut's origin is out along the axis from its seated
    height; 0 seated, positive further off), `off` (m, the nut's axis from the outlet's), `tilt` (degrees between the
    axes) and `turn` (degrees, the nut's turn about the outlet axis)."""
    Ro = np.asarray(R_outlet, float)
    p = Ro.T @ (np.asarray(nut_origin, float) - np.asarray(outlet_origin, float))
    Rr = Ro.T @ np.asarray(R_nut, float)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(Rr[2, 2])))))
    return {"rise": float(p[2] - SEAT_Z), "off": float(math.hypot(p[0], p[1])), "tilt": tilt,
            "turn": math.degrees(math.atan2(float(Rr[1, 0]), float(Rr[0, 0]))), "p": p}
