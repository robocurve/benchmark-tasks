"""The stud finder's magnets pulling on a hidden drywall screw: the force from the real geometry (NumPy only).

Model, in plain terms (README, "The magnet's pull"):
- Each of the finder's two magnets is a neodymium disc, 1/2 in across and 1/8 in thick (12.7 x 3.175 mm), grade
  N42 (remanence 1.30 T), magnetised through its thickness, its working face 1.0 mm behind the finder's back (a
  plastic skin). The maker does not publish the magnets' size or grade; these are stated assumptions.
- A uniformly magnetised disc's field is exactly the field of its two faces carrying magnetic surface charge
  +-Br/mu0. Each face is summed as rings (Gauss-Legendre over the radius), and each ring's field is closed form
  (complete elliptic integrals, computed by the arithmetic-geometric mean).
- The screw is soft steel. Its bugle head (8.3 mm across, #6 drywall screw) is a magnetic pole spread over its top
  face, induced by the magnet. The flux it can carry is capped by saturation of the shank's root (2.4 mm across,
  2.0 T); below that it carries twice the magnet's own flux through the face (a soft-steel face draws field lines
  in). The pole's sign follows the field, so the pull is always attractive.
- The force on the magnet is the reaction to the force on that pole: toward the wall, and in the wall's plane
  toward the screw's axis (it centres the magnet over the screw).

Numbers this gives (1.0 mm skin + head set 1.0 mm under the paint, so a 2.0 mm gap when the finder lies flat):
about 1.5 N of pull straight over the screw, falling to 0.6 N at 6 mm off its axis and to nothing past 9 mm; the
sideways pull toward the axis peaks at about 0.8 N, 6 mm off. ``pull(rho, gap)`` interpolates a table built from
the integrals on first use.
"""
import math

import numpy as np

MU0 = 4e-7 * math.pi
MAGNET_D, MAGNET_T, MAGNET_BR = 0.0127, 0.003175, 1.30      # N42 disc, 1/2 x 1/8 in
SKIN = 0.0010                                                # plastic between the magnet's face and the finder's back
HEAD_D = 0.0083                                              # #6 bugle head, across
ROOT_D, STEEL_BSAT = 0.0024, 2.0                             # #6 coarse-thread root diameter; low-carbon steel, T
DRAW = 2.0                                                   # flux drawn into an unsaturated soft-steel face, x own
RHO_MAX, GAP_MIN, GAP_MAX = 0.020, 0.0010, 0.025             # the table's range (m); no force outside it


def _agm_KE(m):
    """Complete elliptic integrals K(m), E(m) (parameter m = k^2, 0 <= m < 1) by the arithmetic-geometric mean."""
    m = np.asarray(m, float)
    a, b, c = np.ones_like(m), np.sqrt(1.0 - m), np.sqrt(m)
    total, p = 0.5 * c * c, 0.5
    for _ in range(12):
        a, b, c = (a + b) / 2.0, np.sqrt(a * b), (a - b) / 2.0
        p *= 2.0
        total = total + p * c * c
    K = math.pi / (2.0 * a)
    return K, K * (1.0 - total)


def ring_field(q, a, rho, z):
    """Field (B_rho, B_z), tesla, at (rho, z) from a ring of radius a in the plane z = 0 carrying total magnetic
    charge q (A m, so that B = mu0 q / (4 pi r^2) far away)."""
    rho, z = np.asarray(rho, float), np.asarray(z, float)
    s2 = (a + rho) ** 2 + z ** 2
    d2 = (a - rho) ** 2 + z ** 2
    m = np.clip(4.0 * a * rho / s2, 0.0, 1.0 - 1e-12)
    K, E = _agm_KE(m)
    c = MU0 * q / (4.0 * math.pi) * 2.0 / (math.pi * np.sqrt(s2))
    bz = c * z * E / d2
    with np.errstate(divide="ignore", invalid="ignore"):
        br = np.where(rho > 1e-9, c / (2.0 * rho) * (K - (a * a - rho * rho + z * z) / d2 * E), 0.0)
    return br, bz


def magnet_field(rho, z, d=MAGNET_D, t=MAGNET_T, br=MAGNET_BR, n=32):
    """(B_rho, B_z) of the disc magnet at (rho, z): its working face is the plane z = 0, its body is z in [0, t], and
    points in front of it (toward the screw) have z < 0. The working face carries charge -Br/mu0 per area."""
    x, w = np.polynomial.legendre.leggauss(n)
    R = d / 2.0
    r, wr = R * (x + 1) / 2.0, R * w / 2.0
    sigma = br / MU0
    rho, z = np.asarray(rho, float)[..., None], np.asarray(z, float)[..., None]
    out_r, out_z = 0.0, 0.0
    for z0, sign in ((0.0, -1.0), (t, 1.0)):
        q = sign * sigma * 2.0 * math.pi * r * wr          # each ring's charge
        b_r, b_z = ring_field(q, r, rho, z - z0)
        out_r, out_z = out_r + b_r.sum(-1), out_z + b_z.sum(-1)
    return out_r, out_z


def head_pull(rho, gap):
    """(pull toward the wall, sideways pull toward the screw's axis), newtons, on one magnet whose working face is
    `gap` (m) in front of the head's top face, its axis `rho` (m) off the screw's axis."""
    a = HEAD_D / 2.0
    xr, wr = np.polynomial.legendre.leggauss(6)
    r = a * (xr + 1) / 2.0
    wr = a * wr / 2.0
    nt = 16
    th = np.arange(nt) * 2.0 * math.pi / nt
    R, T = np.meshgrid(r, th, indexing="ij")
    W = (wr[:, None] * R * (2.0 * math.pi / nt)).ravel()
    px, py = rho + (R * np.cos(T)).ravel(), (R * np.sin(T)).ravel()
    prho = np.hypot(px, py)
    b_r, b_z = magnet_field(prho, np.full(prho.shape, -gap))
    with np.errstate(divide="ignore", invalid="ignore"):
        cos = np.where(prho > 1e-12, px / prho, 1.0)
    b_x = b_r * cos                                          # field component along the line magnet axis -> screw
    area = math.pi * a * a
    bz_avg, bx_avg = float((b_z * W).sum() / area), float((b_x * W).sum() / area)
    flux = abs(bz_avg) * area
    q_sat = STEEL_BSAT * math.pi * (ROOT_D / 2.0) ** 2 / MU0
    q = min(q_sat, DRAW * flux / MU0)                        # the head's pole strength, A m
    p = q * (1.0 if bz_avg >= 0 else -1.0)                   # its sign follows the field: always attracted
    return p * bz_avg, -p * bx_avg


_TABLE = {}
RHO_GRID = np.arange(0.0, RHO_MAX + 1e-9, 0.0005)
GAP_GRID = np.concatenate([np.arange(GAP_MIN, 0.006, 0.00025), np.arange(0.006, GAP_MAX + 1e-9, 0.001)])


def table():
    """The pull table on (RHO_GRID x GAP_GRID), built once."""
    if not _TABLE:
        fn = np.zeros((len(RHO_GRID), len(GAP_GRID)))
        fl = np.zeros_like(fn)
        for i, rho in enumerate(RHO_GRID):
            for j, g in enumerate(GAP_GRID):
                fn[i, j], fl[i, j] = head_pull(rho, g)
        _TABLE.update(normal=fn, lateral=fl)
    return _TABLE


def pull(rho, gap):
    """(pull toward the wall, sideways pull toward the screw's axis), newtons, interpolated from the table; zero
    outside it (more than 20 mm off the axis or more than 25 mm away)."""
    if not (0.0 <= rho <= RHO_MAX) or gap > GAP_MAX:
        return 0.0, 0.0
    t = table()
    g = min(max(gap, GAP_MIN), GAP_MAX)
    i = min(int(np.searchsorted(RHO_GRID, rho, side="right")) - 1, len(RHO_GRID) - 2)
    j = min(max(int(np.searchsorted(GAP_GRID, g, side="right")) - 1, 0), len(GAP_GRID) - 2)
    u = (rho - RHO_GRID[i]) / (RHO_GRID[i + 1] - RHO_GRID[i])
    v = (g - GAP_GRID[j]) / (GAP_GRID[j + 1] - GAP_GRID[j])

    def lerp(a):
        return float((1 - u) * (1 - v) * a[i, j] + u * (1 - v) * a[i + 1, j] + (1 - u) * v * a[i, j + 1]
                     + u * v * a[i + 1, j + 1])

    return lerp(t["normal"]), lerp(t["lateral"])


def magnet_force(face_centre, axis, head_centre):
    """World-frame force (N) on one magnet: its working face's centre and unit axis (pointing out of the finder's
    back), and the screw head's top-face centre. Zero if the head is not in front of the face."""
    face_centre, axis, head_centre = (np.asarray(v, float) for v in (face_centre, axis, head_centre))
    d = head_centre - face_centre
    gap = float(d @ axis)
    if gap <= 0.0:
        return np.zeros(3)
    side = d - gap * axis
    rho = float(np.linalg.norm(side))
    fn, fl = pull(rho, gap)
    out = fn * axis
    if rho > 1e-9:
        out = out + fl * side / rho
    return out
