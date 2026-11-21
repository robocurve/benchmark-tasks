"""The care-training manikin's head: its moulded vinyl surface as one triangle mesh, built from adult head measurements.

Frame (m): origin on the midline at the nasal root (the deepest point of the nasal root, the sellion, lies 0.8 mm in
front of it), x straight back from the face, y to the head's left, z up. The face looks toward -x. NumPy only.

Measurements (adult means, men and women weighted equally):
- ANSUR II (2012 US Army anthropometric survey, public data, 4,082 men and 1,986 women): head breadth 151 mm,
  head length 195 mm, bizygomatic breadth 138 mm, interpupillary breadth 63 mm, menton-sellion length (face length)
  118 mm, tragion to top of head 129 mm, ear length 62 mm, ear breadth 35 mm, ear protrusion 22 mm, neck
  circumference 364 mm.
- NIOSH head-and-face survey of US respirator users (Zhuang & Bradtmiller 2005, Table VI): nasal root breadth 16.5 mm,
  nose breadth 35 mm, nose protrusion 20.5 mm, subnasale-sellion length 50 mm, minimum frontal breadth 104 mm.

The surface is the zero level of a smooth union of ellipsoids, spheres and capsules (cranium, face and jaw, brow,
cheekbones, chin, lips) with the eye sockets carved out and a nose added: a wedge whose two flat sides meet at a
rounded ridge and widen toward the cheeks. It is meshed by casting rays from a point inside the head and keeping the
outermost crossing. The ears, eyes and the neck are separate shapes (in the task file).
"""

from __future__ import annotations

import math

import numpy as np

MM = 0.001

# -- landmarks (head frame) ---------------------------------------------------------------------------------------
GLABELLA = np.array([-6.0, 0.0, 12.0]) * MM
EYE_Y, EYE_Z = 31.5 * MM, -5.0 * MM  # pupils: half the interpupillary breadth across, 5 mm below the nasal root
VERTEX_Z = 111.0 * MM  # top of the head: tragion to top of head 129 mm, the tragion 18 mm below the sellion
MENTON_Z = -118.0 * MM  # chin: face length (menton-sellion) 118 mm
HEAD_BACK_X = GLABELLA[0] + 195.0 * MM  # head length 195 mm from the glabella

# -- the nose ----------------------------------------------------------------------------------------------------
RIDGE_TILT = math.radians(28.0)  # the dorsum leans 28 deg forward of vertical
SPLAY = math.radians(35.0)  # each side, seen from above, runs back 35 deg from the midline plane
RIDGE_R = 7.0 * MM  # the ridge is rounded to this radius
RIDGE_DIR = np.array([-math.sin(RIDGE_TILT), 0.0, -math.cos(RIDGE_TILT)])  # down the dorsum


def _flank_normal(side: float) -> np.ndarray:
    """Outward normal of the nose's left (side +1) or right (-1) flat side."""
    h = np.array([math.cos(SPLAY), side * math.sin(SPLAY), 0.0])
    n = np.cross(h, RIDGE_DIR) * side
    n = n / np.linalg.norm(n)
    return n if n[0] < 0 else -n


FLANK_N = {1.0: _flank_normal(1.0), -1.0: _flank_normal(-1.0)}
# the sharp line where the two flat sides would meet (set below so that the rounded ridge passes through the sellion)
_APEX0 = np.zeros(3)


def _inner_apex(apex0: np.ndarray) -> np.ndarray:
    nl, nr = FLANK_N[1.0], FLANK_N[-1.0]
    return apex0 - RIDGE_R * (nl + nr) / (1.0 + float(nl @ nr))


def _wedge_dist(p: np.ndarray, apex: np.ndarray) -> np.ndarray:
    """Signed distance to the sharp wedge {n_L.(p - apex) <= 0, n_R.(p - apex) <= 0} (a convex region)."""
    nl, nr = FLANK_N[1.0], FLANK_N[-1.0]
    q = p - apex
    dl, dr = q @ nl, q @ nr
    inside = np.maximum(dl, dr)
    # outside: the nearest point is on a face whose foot lies within the other half-space, or on the apex line
    foot_l = q - dl[..., None] * nl
    foot_r = q - dr[..., None] * nr
    ok_l = (dl > 0) & (foot_l @ nr <= 0)
    ok_r = (dr > 0) & (foot_r @ nl <= 0)
    along = q @ RIDGE_DIR
    line = np.linalg.norm(q - along[..., None] * RIDGE_DIR, axis=-1)
    out = np.where(ok_l, dl, np.where(ok_r, dr, line))
    return np.where(inside <= 0, inside, out)


def nose_sdf(p: np.ndarray, apex0: np.ndarray | None = None) -> np.ndarray:
    """The nose: the rounded wedge, cut at its base (subnasale level) and behind, with the tip and the alae."""
    a0 = _APEX0 if apex0 is None else apex0
    w = _wedge_dist(p, _inner_apex(a0)) - RIDGE_R
    w = _smax(w, p[..., 2] - 6.0 * MM, 4.0 * MM)  # nothing of the wedge above the brow line (it is inside the brow)
    base_n = np.array([0.17, 0.0, -0.985])  # the nose's base rises 10 deg toward the tip
    w = _smax(w, (p - np.array([-4.0, 0.0, -46.0]) * MM) @ base_n, 3.0 * MM)
    w = _smax(w, p[..., 0] - 16.0 * MM, 2.0 * MM)  # it ends inside the face
    w = _smax(w, -(p[..., 0] + 21.0 * MM), 4.0 * MM)  # the tip is rounded off (the lobule)
    tip = _sphere(p, np.array([-12.0, 0.0, -36.0]) * MM, 8.5 * MM)
    alae = np.minimum(_sphere(p, np.array([-3.0, 11.0, -39.0]) * MM, 7.0 * MM),
                      _sphere(p, np.array([-3.0, -11.0, -39.0]) * MM, 7.0 * MM))
    return _smin(_smin(w, tip, 5.0 * MM), alae, 4.0 * MM)


# -- primitives --------------------------------------------------------------------------------------------------
def _ellipsoid(p, c, r):
    q = (p - np.asarray(c)) / np.asarray(r)
    k0 = np.linalg.norm(q, axis=-1)
    k1 = np.linalg.norm(q / np.asarray(r), axis=-1)
    return k0 * (k0 - 1.0) / np.maximum(k1, 1e-12)


def _sphere(p, c, r):
    return np.linalg.norm(p - np.asarray(c), axis=-1) - r


def _capsule(p, a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    pa, ba = p - a, b - a
    h = np.clip((pa @ ba) / float(ba @ ba), 0.0, 1.0)
    return np.linalg.norm(pa - h[..., None] * ba, axis=-1) - r


def _smin(a, b, k):
    h = np.clip(0.5 + 0.5 * (b - a) / k, 0.0, 1.0)
    return b * (1.0 - h) + a * h - k * h * (1.0 - h)


def _smax(a, b, k):
    return -_smin(-a, -b, k)


def face_sdf(p: np.ndarray) -> np.ndarray:
    """The head without the nose: cranium, the sides over and below the ears, the face, a soft brow, cheeks, lips and
    chin, with the eye sockets carved out."""
    cranium = _ellipsoid(p, np.array([GLABELLA[0] + 97.5 * MM, 0.0, 30.0 * MM]),
                         np.array([97.5, 75.5, 81.0]) * MM)  # head length 195, head breadth 151, vertex at +111
    sides = _ellipsoid(p, np.array([85.0, 0.0, -25.0]) * MM, np.array([70.0, 68.0, 50.0]) * MM)  # around the ears
    face = _ellipsoid(p, np.array([70.0, 0.0, -60.0]) * MM, np.array([76.0, 62.0, 62.0]) * MM)  # mouth and jaw
    d = _smin(cranium, sides, 16.0 * MM)
    d = _smin(d, face, 12.0 * MM)
    d = _smin(d, _ellipsoid(p, np.array([6.0, 0.0, 13.0]) * MM, np.array([9.0, 40.0, 7.0]) * MM), 12.0 * MM)  # brow
    for s in (1.0, -1.0):
        d = _smin(d, _sphere(p, np.array([23.0, s * 40.0, -27.0]) * MM, 14.0 * MM), 18.0 * MM)  # cheeks
    d = _smin(d, _ellipsoid(p, np.array([0.0, 0.0, -63.0]) * MM, np.array([7.0, 20.0, 8.0]) * MM), 10.0 * MM)  # lips
    d = _smin(d, _ellipsoid(p, np.array([5.0, 0.0, -100.0]) * MM, np.array([11.0, 17.0, 14.0]) * MM),
              20.0 * MM)  # chin
    for s in (1.0, -1.0):  # the eye sockets
        socket = _ellipsoid(p, np.array([8.0, s * 32.0, -5.0]) * MM, np.array([12.0, 19.0, 13.0]) * MM)
        d = _smax(d, -socket, 6.0 * MM)
    return d


def head_sdf(p: np.ndarray) -> np.ndarray:
    return _smin(face_sdf(p), nose_sdf(p), 3.0 * MM)


def _fit_apex() -> np.ndarray:
    """Place the nose's sharp ridge line so that the rounded ridge passes through the sellion (the origin)."""
    lo, hi = -30.0 * MM, 30.0 * MM  # the apex line's x at z = 0
    for _ in range(60):
        mid = (lo + hi) / 2
        d = float(nose_sdf(np.zeros((1, 3)), np.array([mid, 0.0, 0.0]))[0])
        # d < 0: the origin is inside the nose (the apex is too far forward): move the apex back (larger x)
        if d < 0:
            lo = mid
        else:
            hi = mid
    return np.array([(lo + hi) / 2, 0.0, 0.0])


_APEX0 = _fit_apex()
APEX0 = _APEX0  # the sharp ridge line passes through this point along RIDGE_DIR


def flank_offset(p: np.ndarray, side: float) -> np.ndarray:
    """Signed distance (m) of points from the plane of the nose's flat side (+ outside the nose)."""
    return (np.asarray(p, float) - APEX0) @ FLANK_N[side]


def ridge_x(z: float) -> float:
    """The rounded ridge's front (x) at height z on the midline."""
    lo, hi = -60.0 * MM, 30.0 * MM
    for _ in range(50):
        mid = (lo + hi) / 2
        if float(head_sdf(np.array([[mid, 0.0, z]]))[0]) > 0:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


# -- the mesh ----------------------------------------------------------------------------------------------------
CENTRE = np.array([60.0, 0.0, -10.0]) * MM  # rays start here, inside the head


def build_mesh(n_theta: int = 150, n_phi: int = 300, r_max: float = 0.17, step: float = 1.0 * MM):
    """(points, faces, normals): the outermost surface along rays from CENTRE, a latitude-longitude grid of
    ``n_theta`` x ``n_phi`` rays plus the two poles."""
    theta = (np.arange(n_theta) + 0.5) / n_theta * math.pi
    phi = np.arange(n_phi) / n_phi * 2 * math.pi
    th, ph = np.meshgrid(theta, phi, indexing="ij")
    dirs = np.stack([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)], -1).reshape(-1, 3)
    dirs = np.concatenate([dirs, np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]])])
    radii = np.arange(0.0, r_max + step / 2, step)
    rad = np.empty(len(dirs))
    chunk = 4000
    for i in range(0, len(dirs), chunk):
        d = dirs[i:i + chunk]
        pts = CENTRE + d[:, None, :] * radii[None, :, None]
        sdf = head_sdf(pts.reshape(-1, 3)).reshape(len(d), len(radii))
        inside = sdf <= 0
        # the outermost sample still inside the head
        last = len(radii) - 1 - np.argmax(inside[:, ::-1], axis=1)
        lo, hi = radii[last], radii[np.minimum(last + 1, len(radii) - 1)]
        for _ in range(14):
            mid = (lo + hi) / 2
            s = head_sdf(CENTRE + d * mid[:, None])
            lo, hi = np.where(s <= 0, mid, lo), np.where(s <= 0, hi, mid)
        rad[i:i + chunk] = (lo + hi) / 2
    points = CENTRE + dirs * rad[:, None]
    faces = []
    idx = np.arange(n_theta * n_phi).reshape(n_theta, n_phi)
    nxt = np.roll(idx, -1, axis=1)
    a, b, c, e = idx[:-1], nxt[:-1], idx[1:], nxt[1:]
    # counter-clockwise seen from outside (theta down from +z, phi counter-clockwise from above)
    faces.append(np.stack([a, c, b], -1).reshape(-1, 3))
    faces.append(np.stack([b, c, e], -1).reshape(-1, 3))
    north, south = n_theta * n_phi, n_theta * n_phi + 1
    top, bot = idx[0], idx[-1]
    faces.append(np.stack([np.full(n_phi, north), top, np.roll(top, -1)], -1))
    faces.append(np.stack([np.full(n_phi, south), np.roll(bot, -1), bot], -1))
    faces = np.concatenate(faces).astype(np.int32)
    # smooth normals from the SDF's gradient
    h = 0.2 * MM
    grad = np.stack([head_sdf(points + np.eye(3)[k] * h) - head_sdf(points - np.eye(3)[k] * h) for k in range(3)], -1)
    normals = grad / np.maximum(np.linalg.norm(grad, axis=-1, keepdims=True), 1e-12)
    return points, faces, normals


# -- the ears -------------------------------------------------------------------------------------------------------
EAR_LEN, EAR_BREADTH = 62.0 * MM, 35.0 * MM  # ANSUR II ear length and breadth


def ear_mesh(side: float, front, out_deg: float = 22.0, tilt_deg: float = 12.0, n_r: int = 18, n_t: int = 72):
    """(points, faces, normals) of one auricle (side +1 left, -1 right), head frame: a closed shell, an egg-shaped
    outline 62 x 35 mm in a plane turned ``out_deg`` out from the side of the head about its front edge and leaning
    ``tilt_deg`` back; on its outer face a rounded rim (the helix, 3.5 mm proud) around the top and back, fading at the
    lobule, and a shallow bowl (the concha) inside it; its inner face a low dome. ``front``: the outline's front-most
    point (where the ear meets its root)."""
    a, b = EAR_LEN / 2, EAR_BREADTH / 2
    out, tilt = math.radians(out_deg), math.radians(tilt_deg)
    v0 = np.array([math.cos(out), side * math.sin(out), 0.0])  # back along the ear, away from the head
    u0 = np.array([0.0, 0.0, 1.0])
    u = math.cos(tilt) * u0 + math.sin(tilt) * v0  # up the ear, leaning back
    v = -math.sin(tilt) * u0 + math.cos(tilt) * v0
    w = np.cross(v, u) * side  # outward
    w = w if w[1] * side > 0 else -w
    centre = np.asarray(front, float) + v * b
    r = np.linspace(0.0, 1.0, n_r + 1)
    t = np.arange(n_t) / n_t * 2 * math.pi  # 0: top, pi/2: back, pi: bottom (lobule)
    R, Th = np.meshgrid(r, t, indexing="ij")
    egg = 1.0 + 0.12 * np.cos(Th)  # broader at the top than at the lobule
    uu, vv = a * R * np.cos(Th), b * R * np.sin(Th) * egg
    rim_w = np.where(np.sin(Th) < -0.2, 0.0, 1.0)  # no rim on the front edge (where it joins the root)
    lobe = np.clip(-np.cos(Th) - 0.6, 0.0, 0.4) / 0.4  # 1 at the lobule
    r0 = 0.78
    bump = np.sqrt(np.clip(1.0 - ((R - r0) / (1.0 - r0)) ** 2, 0.0, 1.0))
    rim = 3.5 * MM * np.where(R >= r0, bump, 0.0) * rim_w * (1.0 - lobe)
    bowl = 1.2 * MM + 1.6 * MM * (R / r0) ** 2  # the concha, lower than the rim
    edge = np.sqrt(np.clip(1.0 - R ** 6, 0.0, 1.0))
    hf = np.maximum(rim, np.where(R < r0, bowl, bowl[-1:] * 0 + 2.8 * MM) * edge) + 2.0 * MM * lobe * edge
    hb = -(2.2 * MM + 1.5 * MM * lobe) * edge
    pts_f = centre + uu[..., None] * u + vv[..., None] * v + hf[..., None] * w
    pts_b = centre + uu[..., None] * u + vv[..., None] * v + hb[..., None] * w
    nf = pts_f.reshape(-1, 3)
    nb = pts_b.reshape(-1, 3)
    points = np.concatenate([nf, nb])
    idx = np.arange((n_r + 1) * n_t).reshape(n_r + 1, n_t)
    nxt = np.roll(idx, -1, axis=1)
    a0, b0, c0, d0 = idx[:-1], nxt[:-1], idx[1:], nxt[1:]
    front_faces = np.concatenate([np.stack([a0, c0, b0], -1).reshape(-1, 3), np.stack([b0, c0, d0], -1).reshape(-1, 3)])
    back_faces = front_faces[:, ::-1] + len(nf)
    faces = np.concatenate([front_faces, back_faces]).astype(np.int32)
    # orient: front faces must point along w (outward); flip all if not
    tri = points[faces[: len(front_faces)]]
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    if float((nrm @ w).sum()) < 0:
        faces = faces[:, ::-1].copy()
    # vertex normals: area-weighted face normals
    tri = points[faces]
    fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    vn = np.zeros_like(points)
    for k in range(3):
        np.add.at(vn, faces[:, k], fn)
    vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)
    return points, faces, vn
