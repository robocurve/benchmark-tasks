"""The lower-torso display form lying on its back, wearing jeans with five belt loops (one fixed kinematic part).

Collision: flat slabs tangent to the trousers' surface round the waist (where the belt runs) and coarser rings over
the hips; each belt loop is a bridge on two posts. Visual: a smooth lofted mesh of the jeans from the waist to the
leg cut, the form's flat waist top, and the loops.
"""

from __future__ import annotations

import math

import numpy as np

import geom as G
from partkit import part

DENIM = (0.20, 0.30, 0.50)
LOOP_DENIM = (0.17, 0.26, 0.45)
FORM_WHITE = (0.90, 0.89, 0.86)
ORIGIN = np.array([G.X_W, G.YC, G.ZC])

# sections of the jeans (x from the waist edge, half width, half depth, centre z above the table)
SECTIONS = [(0.000, G.A1, G.B1, G.LIFT + G.B1),
            (0.060, G.A1 + 0.006, G.B1 + 0.003, G.LIFT + G.B1 - 0.002),
            (0.180, 0.180, 0.105, 0.105),
            (0.300, 0.172, 0.100, 0.100),
            (G.FORM_LEN, 0.165, 0.095, 0.095)]


def _section(x):
    xs = [s[0] for s in SECTIONS]
    i = max(0, min(len(xs) - 2, int(np.searchsorted(xs, x) - 1)))
    x0, x1 = xs[i], xs[i + 1]
    f = (x - x0) / (x1 - x0)
    f = f * f * (3 - 2 * f)
    return tuple(SECTIONS[i][k] + f * (SECTIONS[i + 1][k] - SECTIONS[i][k]) for k in (1, 2, 3))


def _quat_x(a):
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def _slab_ring(x0, x1, n, thick=0.012, section=None):
    """Boxes tangent to the elliptic section (a, b, zc) over x0..x1, in the part frame."""
    a, b, zc = section if section is not None else _section((x0 + x1) / 2 - G.X_W)
    out = []
    ths = np.linspace(0, 2 * math.pi, n, endpoint=False)
    for th in ths:
        th2 = th + 2 * math.pi / n
        p1 = np.array([a * math.sin(th), b * math.cos(th)])
        p2 = np.array([a * math.sin(th2), b * math.cos(th2)])
        mid = (p1 + p2) / 2
        d = p2 - p1
        L = float(np.linalg.norm(d)) + 0.002
        nrm = np.array([-d[1], d[0]]) / np.linalg.norm(d)        # outward (theta runs toward +y at the top)
        c = mid - nrm * thick / 2
        ang = math.atan2(d[1], d[0])                               # slab's long axis in the y-z plane
        centre = (float((x0 + x1) / 2 - G.X_W), float(c[0]), float(c[1] + G.TOP + zc - G.ZC))
        out.append((centre, (float(x1 - x0), L, thick), DENIM, _quat_x(ang)))
    return out


def _loops():
    out = []
    for name, th in G.LOOPS.items():
        _, n = G.ellipse_pt(th)
        t = np.array([n[1], -n[0]])                     # along the belt (sign irrelevant)
        ang = math.atan2(t[1], t[0])
        q = _quat_x(ang)
        base, _ = G.ellipse_pt(th, 0.0)
        rb = G.LOOP_GAP + G.LOOP_T / 2
        p = base + n * rb
        xb = G.X_B - G.X_W
        span = G.LOOP_IN_X + 2 * G.LOOP_T
        out.append(((xb, float(p[0]), float(p[1] - G.ZC)), (span, G.LOOP_W, G.LOOP_T), LOOP_DENIM, q))
        for s in (1, -1):
            pp = base + n * (G.LOOP_GAP + G.LOOP_T) / 2
            out.append(((xb + s * (G.LOOP_IN_X / 2 + G.LOOP_T / 2), float(pp[0]), float(pp[1] - G.ZC)),
                        (G.LOOP_T, G.LOOP_W, G.LOOP_GAP + G.LOOP_T), LOOP_DENIM, q))
    return out


def _loft_mesh(nx=40, nt=72):
    pts, tris = [], []
    xs = np.linspace(0.0, G.FORM_LEN, nx)
    for x in xs:
        a, b, zc = _section(x)
        for k in range(nt):
            th = 2 * math.pi * k / nt
            pts.append((x, a * math.sin(th), b * math.cos(th) + G.TOP + zc - G.ZC))
    for i in range(nx - 1):
        for k in range(nt):
            a0, a1 = i * nt + k, i * nt + (k + 1) % nt
            b0, b1 = a0 + nt, a1 + nt
            tris += [(a0, b0, a1), (a1, b0, b1)]
    jeans = (np.array(pts) - [0, 0, 0], tris)
    # the form's flat waist top (just inside the waistband) and the leg cut
    caps = []
    for x, inset in ((0.004, 0.0015), (G.FORM_LEN - 0.002, 0.0)):
        a, b, zc = _section(x)
        c0 = len(caps)
        cp = [(x, 0.0, G.TOP + zc - G.ZC)]
        for k in range(nt):
            th = 2 * math.pi * k / nt
            cp.append((x, (a - inset) * math.sin(th), (b - inset) * math.cos(th) + G.TOP + zc - G.ZC))
        caps.append((np.array(cp), [(0, 1 + k, 1 + (k + 1) % nt) for k in range(nt)]))
    return jeans, caps


def form_part():
    boxes = []
    boxes += _slab_ring(G.X_W + 0.001, G.X_W + 0.060, 64, section=(G.A1, G.B1, G.LIFT + G.B1))
    for x0, x1 in ((0.060, 0.120), (0.120, 0.200), (0.200, 0.300)):
        boxes += _slab_ring(G.X_W + x0, G.X_W + x1, 28, thick=0.03)
    boxes += _loops()
    (jp, jt), caps = _loft_mesh()
    meshes = [(jp, jt, DENIM)] + [(cp, ct, FORM_WHITE) for cp, ct in caps]
    return part("form", pos=tuple(ORIGIN), boxes=tuple(boxes), mass=5.0, color=DENIM, friction=0.5, fixed=True,
                meshes=tuple(meshes))
