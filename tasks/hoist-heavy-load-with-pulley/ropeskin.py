"""Smooth continuous braided-rope visual skin over the simulated rope links. NumPy only.

Draws a continuous Catmull-Rom tube through the link endpoints plus two counter-helical tracer strands so rope travel
over the pulley and through the grippers is unmistakably visible in every camera.
"""

from __future__ import annotations

import math

import numpy as np

import meshes as m


def centre_line(positions, rotations, lengths, start=None):
    """Smooth centreline points (M x 3) and unit side vectors (M x 3) along the rope."""
    pos = np.asarray(positions, float)
    rot = np.asarray(rotations, float)
    half = np.asarray(lengths, float)[:, None] / 2.0 * rot[:, :, 0]
    ends = np.vstack([pos[0] - half[0], (pos[1:] - half[1:] + pos[:-1] + half[:-1]) / 2.0, pos[-1] + half[-1]])
    if start is not None:
        ends[0] = start
    sides = np.vstack([rot[0][:, 2], (rot[1:, :, 2] + rot[:-1, :, 2]) / 2.0, rot[-1][:, 2]])

    pts, sds = [], []
    n = len(ends)
    for i in range(n - 1):
        p0, p1, p2, p3 = ends[max(i - 1, 0)], ends[i], ends[i + 1], ends[min(i + 2, n - 1)]
        for t in (0.0, 1.0 / 3.0, 2.0 / 3.0):
            pts.append(
                0.5
                * (
                    2.0 * p1
                    + (-p0 + p2) * t
                    + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t * t
                    + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t**3
                )
            )
            sds.append((1.0 - t) * sides[i] + t * sides[i + 1])
    pts.append(ends[-1])
    sds.append(sides[-1])
    pts, sds = np.asarray(pts), np.asarray(sds)
    tan = np.gradient(pts, axis=0)
    tan /= np.maximum(np.linalg.norm(tan, axis=1, keepdims=True), 1e-12)
    sds -= np.sum(sds * tan, axis=1, keepdims=True) * tan
    sds /= np.maximum(np.linalg.norm(sds, axis=1, keepdims=True), 1e-12)
    return pts, sds, tan


def skin(points, side, tangent, radius, pitch=0.055, strand_r=0.0016):
    """Return ``(core_mesh, strand_mesh)`` for a braided rope of ``radius`` along ``points``."""
    core = m.tube(points, radius * 0.96, 16, cap=True)
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    binorm = np.cross(tangent, side)
    ang = 2.0 * math.pi * s / pitch
    offset_r = radius - strand_r * 0.45
    h1 = points + offset_r * (np.cos(ang)[:, None] * side + np.sin(ang)[:, None] * binorm)
    h2 = points + offset_r * (np.cos(ang + math.pi)[:, None] * side + np.sin(ang + math.pi)[:, None] * binorm)
    s1 = m.tube(h1, strand_r, 6, cap=True)
    s2 = m.tube(h2, strand_r, 6, cap=True)
    return core, m.merge(s1, s2)
