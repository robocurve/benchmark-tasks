"""Procedural triangle meshes for the visible parts: lathe solids, swept tubes and rounded boxes. NumPy only.

Every function returns ``(points, faces)``: an N x 3 float array (m) and an M x 3 int array of counter-clockwise
triangles (outward normals). The task's bodies render these with smooth normals; their colliders are separate,
simpler convex shapes.
"""

from __future__ import annotations

import math

import numpy as np


def lathe(profile, segments: int = 64, cap_start: bool = False, cap_end: bool = False):
    """Revolve a profile about the z axis. ``profile`` is a list of (r, z) points traced so that the surface's outside
    is on the right of the direction of travel when r points right and z up (outer walls go up, inner walls go down).
    ``cap_start``/``cap_end`` close the first/last ring with a flat disc."""
    prof = np.asarray(profile, float)
    n = len(prof)
    th = np.linspace(0.0, 2 * math.pi, segments, endpoint=False)
    c, s = np.cos(th), np.sin(th)
    pts = np.zeros((n * segments, 3))
    for i, (r, z) in enumerate(prof):
        pts[i * segments:(i + 1) * segments] = np.column_stack([r * c, r * s, np.full(segments, z)])
    faces = []
    for i in range(n - 1):
        a, b = i * segments, (i + 1) * segments
        for j in range(segments):
            k = (j + 1) % segments
            faces.append((a + j, a + k, b + k))
            faces.append((a + j, b + k, b + j))
    pts = list(pts)
    for ring, flip in ((0, True) if cap_start else (None, None), (n - 1, False) if cap_end else (None, None)):
        if ring is None:
            continue
        centre = len(pts)
        pts.append(np.array([0.0, 0.0, prof[ring][1]]))
        base = ring * segments
        for j in range(segments):
            k = (j + 1) % segments
            faces.append((centre, base + k, base + j) if flip else (centre, base + j, base + k))
    return np.asarray(pts), np.asarray(faces, int)


def _frames(path: np.ndarray, up_hint=(0.0, 0.0, 1.0)):
    """Rotation-minimizing frames (tangent, normal, binormal) along a polyline."""
    t = np.gradient(path, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-12)
    n0 = np.cross(t[0], up_hint)
    if np.linalg.norm(n0) < 1e-6:
        n0 = np.cross(t[0], (1.0, 0.0, 0.0))
    n0 /= np.linalg.norm(n0)
    normals = [n0]
    for i in range(1, len(path)):
        v = normals[-1] - np.dot(normals[-1], t[i]) * t[i]
        normals.append(v / max(np.linalg.norm(v), 1e-12))
    normals = np.asarray(normals)
    return t, normals, np.cross(t, normals)


def tube(path, radius, segments: int = 16, cap: bool = True, up_hint=(0.0, 0.0, 1.0)):
    """A tube of ``radius`` (a number or one per path point) swept along the polyline ``path`` (K x 3)."""
    path = np.asarray(path, float)
    radius = np.broadcast_to(np.asarray(radius, float), (len(path),))
    _, nrm, bnm = _frames(path, up_hint)
    th = np.linspace(0.0, 2 * math.pi, segments, endpoint=False)
    ring = np.cos(th)[None, :, None] * nrm[:, None, :] + np.sin(th)[None, :, None] * bnm[:, None, :]
    pts = (path[:, None, :] + radius[:, None, None] * ring).reshape(-1, 3)
    faces = []
    for i in range(len(path) - 1):
        a, b = i * segments, (i + 1) * segments
        for j in range(segments):
            k = (j + 1) % segments
            faces.append((a + j, a + k, b + k))
            faces.append((a + j, b + k, b + j))
    pts = list(pts)
    if cap:
        for ring_i, flip in ((0, True), (len(path) - 1, False)):
            centre = len(pts)
            pts.append(path[ring_i])
            base = ring_i * segments
            for j in range(segments):
                k = (j + 1) % segments
                faces.append((centre, base + k, base + j) if flip else (centre, base + j, base + k))
    return np.asarray(pts), np.asarray(faces, int)


def superellipse_sweep(path, half_w, half_d, power: float = 4.0, segments: int = 24, up_hint=(0.0, 1.0, 0.0)):
    """A rounded-rectangle section (half widths ``half_w`` along the frame normal and ``half_d`` along the binormal,
    each a number or one per point) swept along ``path``: a grip or a lever. ``power`` 2 is an ellipse; higher is
    squarer."""
    path = np.asarray(path, float)
    hw = np.broadcast_to(np.asarray(half_w, float), (len(path),))
    hd = np.broadcast_to(np.asarray(half_d, float), (len(path),))
    _, nrm, bnm = _frames(path, up_hint)
    th = np.linspace(0.0, 2 * math.pi, segments, endpoint=False)
    c, s = np.cos(th), np.sin(th)
    cx = np.sign(c) * np.abs(c) ** (2.0 / power)
    sy = np.sign(s) * np.abs(s) ** (2.0 / power)
    pts = (path[:, None, :] + (hw[:, None, None] * cx[None, :, None]) * nrm[:, None, :]
           + (hd[:, None, None] * sy[None, :, None]) * bnm[:, None, :]).reshape(-1, 3)
    faces = []
    for i in range(len(path) - 1):
        a, b = i * segments, (i + 1) * segments
        for j in range(segments):
            k = (j + 1) % segments
            faces.append((a + j, a + k, b + k))
            faces.append((a + j, b + k, b + j))
    pts = list(pts)
    for ring_i, flip in ((0, True), (len(path) - 1, False)):
        centre = len(pts)
        pts.append(path[ring_i])
        base = ring_i * segments
        for j in range(segments):
            k = (j + 1) % segments
            faces.append((centre, base + k, base + j) if flip else (centre, base + j, base + k))
    return np.asarray(pts), np.asarray(faces, int)


def transform(mesh, rot=None, offset=(0.0, 0.0, 0.0)):
    pts, faces = mesh
    pts = np.asarray(pts, float)
    if rot is not None:
        pts = pts @ np.asarray(rot, float).T
    return pts + np.asarray(offset, float), faces


def merge(*meshes):
    pts, faces, n = [], [], 0
    for p, f in meshes:
        pts.append(np.asarray(p, float))
        faces.append(np.asarray(f, int) + n)
        n += len(p)
    return np.vstack(pts), np.vstack(faces)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


Z_TO_X = rot_y(math.pi / 2)  # turns a lathe's z axis onto x
