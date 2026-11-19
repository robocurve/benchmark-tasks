"""The hose's visible skin: one smooth tube through the simulated links, redrawn every step. NumPy only.

The hose is simulated as short rigid links; drawing each link would show its joints. Instead the links' end points
are joined into one centre line, smoothed (Catmull-Rom, a few points per link) and swept into a tube, with a thin
coloured stripe along one side that turns with the links, so the hose's twist shows as it does on a real hose.
"""

from __future__ import annotations

import numpy as np

import meshes as m


def centre_line(positions, rotations, lengths, start=None, end=None):
    """Points along the hose (world) from link centres ``positions`` (K x 3), rotations (K x 3 x 3, each link's x
    along the hose) and lengths; ``start``/``end`` pin the two ends (the tap's outlet and the nozzle's coupling).
    Returns (points, side) where ``side`` is each point's stripe direction (unit, perpendicular to the line)."""
    pos = np.asarray(positions, float)
    rot = np.asarray(rotations, float)
    half = np.asarray(lengths, float)[:, None] / 2 * rot[:, :, 0]
    ends = np.vstack([pos[0] - half[0], (pos[1:] - half[1:] + pos[:-1] + half[:-1]) / 2, pos[-1] + half[-1]])
    if start is not None:
        ends[0] = start
    if end is not None:
        ends[-1] = end
    sides = np.vstack([rot[0][:, 2], (rot[1:, :, 2] + rot[:-1, :, 2]) / 2, rot[-1][:, 2]])
    # subdivide: three points per link
    pts, sds = [], []
    n = len(ends)
    for i in range(n - 1):
        p0, p1, p2, p3 = ends[max(i - 1, 0)], ends[i], ends[i + 1], ends[min(i + 2, n - 1)]
        for t in (0.0, 1 / 3, 2 / 3):
            pts.append(0.5 * (2 * p1 + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
            sds.append((1 - t) * sides[i] + t * sides[i + 1])
    pts.append(ends[-1])
    sds.append(sides[-1])
    pts, sds = np.array(pts), np.array(sds)
    tangent = np.gradient(pts, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
    sds -= np.sum(sds * tangent, axis=1, keepdims=True) * tangent
    sds /= np.maximum(np.linalg.norm(sds, axis=1, keepdims=True), 1e-12)
    return pts, sds


def skin(points, side, radius, stripe_r=0.0009):
    """(tube mesh, stripe mesh) for a hose of ``radius`` along ``points``."""
    tube = m.tube(points, radius, 18, cap=True)
    stripe = m.tube(points + side * (radius - stripe_r * 0.6), stripe_r, 6, cap=True)
    return tube, stripe
