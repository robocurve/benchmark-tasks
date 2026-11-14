"""An adult care-training head at real size, as one smooth signed-distance shape (pure numpy, no assets).

Head frame (meters): origin at the middle of the skull; x = out of the face (front), y = the patient's left,
z = toward the crown. The patient's right eye is at y = -EYE_Y. Sizes are typical adult values: head breadth
15.0 cm, head length (brow to back of the skull) 19.5 cm, chin to crown 23 cm, eyes 6.4 cm apart, eye openings
3.0 x 1.0 cm.

The same function gives (1) the visible mesh (rays from the skull's middle to the surface), (2) where a falling drop
lands and (3) whether the bottle tip touches the skin, so what the cameras show and what the scorer tests agree.
The lower lid of the right eye is a separate moving part (task.py); `lid_sdf` gives its shape for a given pull-down.
"""
from __future__ import annotations

import math

import numpy as np

# -- dimensions (m) ------------------------------------------------------------------------------------------------
SKULL = (0.0975, 0.075, 0.115)  # semi-axes: half head length, half breadth, half chin-to-crown
SKULL_C = (-0.004, 0.0, 0.010)
EYE_Y = 0.032  # eye centers 6.4 cm apart
EYE_Z = 0.012  # eye level, a little above the skull's middle
EYE_X = 0.080  # front of the eye surface (cornea) in the eye opening
EYE_HALF_W, EYE_HALF_H = 0.015, 0.005  # the eye opening: 3.0 x 1.0 cm
EYEBALL_R = 0.012
LID_W = 0.028  # lower lid margin width
LID_R = 0.0035  # lower lid margin thickness (radius of its rounded edge)
POCKET_MIN = 0.003  # lid drawn down at least 3 mm: the lower-lid pocket is open


def _ell(p, c, r):
    """Signed distance (approximate, exact on the surface) to an ellipsoid."""
    q = (p - np.asarray(c)) / np.asarray(r)
    k0 = np.linalg.norm(q, axis=-1)
    k1 = np.linalg.norm(q / np.asarray(r), axis=-1)
    return np.where(k1 > 1e-12, k0 * (k0 - 1.0) / np.maximum(k1, 1e-12), -min(r))


def _smin(a, b, k):
    h = np.clip(0.5 + 0.5 * (b - a) / k, 0.0, 1.0)
    return b * (1 - h) + a * h - k * h * (1 - h)


def _smax(a, b, k):
    return -_smin(-a, -b, k)


def _capsule(p, a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    pa, ba = p - a, b - a
    h = np.clip((pa @ ba) / (ba @ ba), 0.0, 1.0)
    return np.linalg.norm(pa - h[..., None] * ba, axis=-1) - r


def eye_center(side: int = -1) -> np.ndarray:
    """Center of an eye opening on the eye's surface; side -1 = the patient's right eye, +1 = left."""
    return np.array([EYE_X, side * EYE_Y, EYE_Z])


def head_sdf(p) -> np.ndarray:
    """Signed distance (m) from points p (..., 3) in the head frame to the skin; negative inside."""
    p = np.asarray(p, float)
    d = _ell(p, SKULL_C, SKULL)
    d = _smin(d, _ell(p, (0.040, 0.0, -0.062), (0.050, 0.052, 0.050)), 0.02)  # jaw and chin
    for s in (-1, 1):
        d = _smin(d, _ell(p, (0.062, s * 0.040, -0.022), (0.028, 0.026, 0.026)), 0.012)  # cheeks
    d = _smin(d, _capsule(p, (0.078, -0.045, 0.030), (0.078, 0.045, 0.030), 0.010), 0.012)  # brow ridge
    nose = _ell(p, (0.092, 0.0, -0.012), (0.014, 0.011, 0.028))
    d = _smin(d, nose, 0.006)
    for s in (-1, 1):  # eye sockets, then the eyeball sitting in each
        sock = _ell(p, (EYE_X + 0.012, s * EYE_Y, EYE_Z), (0.017, 0.021, 0.013))
        d = _smax(d, -sock, 0.004)
        ball = np.linalg.norm(p - np.array([EYE_X - EYEBALL_R, s * EYE_Y, EYE_Z]), axis=-1) - EYEBALL_R
        d = np.minimum(d, ball)
    return d


def lid_sdf(p, pulled: float) -> np.ndarray:
    """The right lower lid margin (a rounded edge 2.8 cm wide) after being drawn `pulled` m toward the chin, in the
    head frame. It follows the face's slope (straight down the face, -z, sliding over the cheek)."""
    p = np.asarray(p, float)
    z = EYE_Z - EYE_HALF_H - LID_R - pulled
    x = EYE_X - 0.001
    return _capsule(p, (x, -EYE_Y - LID_W / 2, z), (x, -EYE_Y + LID_W / 2, z), LID_R)


def skin_sdf(p, pulled: float = 0.0) -> np.ndarray:
    """The whole face as the tip sees it: the head and the moving lower lid."""
    return np.minimum(head_sdf(p), lid_sdf(p, pulled))


def zone(p, pulled: float) -> str:
    """Where a drop resting at head-frame point p is, for the right eye with its lower lid drawn `pulled` m down:
    'pocket' (the exposed inner lower lid, open only when drawn at least 3 mm), 'eye' (the eye opening), or 'face'."""
    c = eye_center(-1)
    dy, dz = p[1] - c[1], p[2] - c[2]
    if (dy / EYE_HALF_W) ** 2 + (dz / EYE_HALF_H) ** 2 <= 1.0:
        return "eye"
    if abs(dy) <= LID_W / 2 - 0.001 and -EYE_HALF_H - pulled <= dz < 0:  # the strip uncovered by the lid
        return "pocket" if pulled >= POCKET_MIN else "eye"
    return "face"


def ray_hit(o, d, sdf=head_sdf, t_max: float = 0.6, eps: float = 2e-4):
    """First point where the ray o + t d (head frame, unit d) meets the surface, or None. Sphere tracing."""
    o, d = np.asarray(o, float), np.asarray(d, float)
    t = 0.0
    for _ in range(400):
        p = o + t * d
        s = float(sdf(p))
        if s < eps:
            return p
        t += max(s * 0.9, eps)
        if t > t_max:
            return None
    return None


def mesh(n_u: int = 96, n_v: int = 64):
    """The skin as a quad mesh: a ray from SKULL_C in each direction of a (n_u x n_v) longitude-latitude grid to the
    surface (the head is star-shaped from there). Returns (points, counts, indices)."""
    c = np.asarray(SKULL_C, float)
    th = np.repeat(np.pi * np.arange(n_v + 1) / n_v, n_u)  # from the crown (+z) down
    ph = np.tile(2 * np.pi * np.arange(n_u) / n_u, n_v + 1)
    d = np.stack([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)], axis=-1)
    lo, hi = np.zeros(len(d)), np.full(len(d), 0.2)
    for _ in range(30):  # bisection on the sign along every ray at once
        mid = (lo + hi) / 2
        inside = head_sdf(c + mid[:, None] * d) < 0
        lo, hi = np.where(inside, mid, lo), np.where(inside, hi, mid)
    pts = c + lo[:, None] * d
    counts, idx = [], []
    for i in range(n_v):
        for j in range(n_u):
            a, b = i * n_u + j, i * n_u + (j + 1) % n_u
            counts.append(4)
            idx += [a, a + n_u, b + n_u, b]
    return np.array(pts), counts, idx


def lid_mesh(n: int = 12):
    """The lower lid margin (at pull 0) as a closed tube mesh in the head frame."""
    z = EYE_Z - EYE_HALF_H - LID_R
    x = EYE_X - 0.001
    pts, counts, idx = [], [], []
    ys = np.linspace(-EYE_Y - LID_W / 2, -EYE_Y + LID_W / 2, 8)
    for y in ys:
        for k in range(n):
            a = 2 * math.pi * k / n
            pts.append((x + LID_R * math.cos(a), y, z + LID_R * math.sin(a)))
    for i in range(len(ys) - 1):
        for k in range(n):
            a, b = i * n + k, i * n + (k + 1) % n
            counts.append(4)
            idx += [a, b, b + n, a + n]
    for end, flip in ((0, True), (len(ys) - 1, False)):
        ring = [end * n + k for k in range(n)]
        counts.append(n)
        idx += ring[::-1] if flip else ring
    return np.array(pts), counts, idx
