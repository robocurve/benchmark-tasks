"""Geometry of the scene: the airer, the two face cloths draped on it, and the baby T-shirt. NumPy only.

Frames: world x forward from the arms, y left, z up (m); the station's table top is at z = TOP. Distances in the
docs are given "above the table" (z - TOP).

The airer is a three-rail wooden tabletop drying rack: three 12 mm beech dowels, 80 mm apart, through two pine end
frames. Its rails run left-right (along y). Rails B (middle) and C (back) each carry a damp face
cloth folded over it; rail A, the front rail nearest the arms, is free.

The garment is a short-sleeved cotton-jersey baby T-shirt, size 6-12 months (EU 74-80): its front and back are two
triangle layers sewn at the shoulders, along the sleeves' undersides and down the sides. The neck, both sleeve cuffs
and the hem are open, so it is a garment, not a flat cut-out.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field

import numpy as np

TOP = 0.7475  # the station's table top (m)

# -- the airer ---------------------------------------------------------------------------------------------------------
RAIL_R = 0.006  # 12 mm dowels
RAIL_TOP = 0.200  # top of every rail above the table
RAIL_Z = TOP + RAIL_TOP - RAIL_R  # rail axis height (world)
RAIL_X = {"A": 0.32, "B": 0.40, "C": 0.48}  # rail axes, front to back (80 mm pitch)
RAIL_LEN = 0.80  # between the outer faces of the end frames
FRAME_Y = 0.391  # end-frame mid-planes at y = +-FRAME_Y (18 mm boards, inner faces 382 mm from the centre)
FRAME_T = 0.018
FREE_RAIL = "A"  # the front rail is free
LOADED = {"B": "cloth_b", "C": "cloth_c"}  # damp face cloths already drying

# -- the face cloths ---------------------------------------------------------------------------------------------------
CLOTH_SIDE = 0.25  # 25 x 25 cm cotton terry baby washcloth
CLOTH_GSM = 0.40  # kg/m^2 (400 g/m^2 terry, as bought); damp (washed, spun) it is about 1.5x that
CLOTH_MASS = CLOTH_SIDE**2 * CLOTH_GSM * 1.5  # 37.5 g damp
CLOTH_T = 0.003  # terry is about 3 mm thick
CLOTH_EDGE = 0.0125  # mesh spacing

# -- the T-shirt (6-12 months) ---------------------------------------------------------------------------------------
TEE_W = 0.22  # body width, laid flat (chest half-girth)
TEE_L = 0.28  # length, shoulder (high point) to hem
TEE_SLEEVE = 0.055  # each sleeve sticks out this far past the body
TEE_ARMHOLE = 0.090  # armhole depth below the shoulder line (the sleeve's height at the body)
TEE_NECK_W = 0.10
TEE_NECK_D = 0.030
TEE_EDGE = 0.005  # finer than the 12 mm rails, so no rail slips between vertices
TEE_GAP = 0.0016  # the two layers start this far apart (their sewn seams join them)
TEE_MASS = 0.030  # damp jersey baby tee (dry: about 20 g)
TEE_T = 0.0008  # single jersey is about 0.6-0.8 mm thick
# The T-shirt lies flat on the table, front up, hem toward the arms, centred in y, its hem at this x.
TEE_HEM_X = 0.185
TEE_REST = 0.0012  # its back layer's height above the table at the start (contact rest offset)


@dataclass
class Mesh:
    rest: np.ndarray  # V x 3: the unstressed shape
    faces: np.ndarray  # F x 3
    start: np.ndarray  # V x 3: the starting shape (world), same topology
    face_kind: np.ndarray  # F: 0 = main colour, 1 = trim
    marks: dict = field(default_factory=dict)

    def check(self) -> dict:
        """Topology and size checks: manifold, connected, no degenerate triangles."""
        p, f = self.rest, self.faces
        if f.min() < 0 or f.max() >= len(p):
            raise ValueError("face index out of range")
        area = 0.5 * np.linalg.norm(np.cross(p[f[:, 1]] - p[f[:, 0]], p[f[:, 2]] - p[f[:, 0]]), axis=1)
        if area.min() < 1e-9:
            raise ValueError("degenerate triangle")
        edges = Counter(tuple(sorted(e)) for t in f for e in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])))
        if max(edges.values()) > 2:
            raise ValueError("non-manifold edge")
        used = np.zeros(len(p), bool)
        used[f.ravel()] = True
        if not used.all():
            raise ValueError("unused vertices")
        return {"vertices": len(p), "triangles": len(f), "area_m2": float(area.sum()),
                "boundary_edges": sum(1 for n in edges.values() if n == 1)}


def _grid_patch(inside, xs, ys):
    """Triangles over the grid cells whose centre is ``inside(x, y)``: (planar points, faces, cell->vertex map)."""
    ids, pts, faces = {}, [], []

    def vid(i, j):
        if (i, j) not in ids:
            ids[(i, j)] = len(pts)
            pts.append((xs[i], ys[j]))
        return ids[(i, j)]

    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            if inside((xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2):
                a, b, c, d = vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)
                faces += [(a, b, c), (a, c, d)] if (i + j) % 2 == 0 else [(a, b, d), (b, c, d)]
    return np.array(pts, float), np.array(faces, int)


def tshirt() -> Mesh:
    """The baby T-shirt, flat: planar coordinates (u across, v from hem up to the shoulder), two layers."""
    half_w, s = TEE_W / 2, TEE_EDGE
    reach = half_w + TEE_SLEEVE
    us = np.round(np.arange(-reach, reach + s / 2, s), 6)
    vs = np.round(np.arange(0.0, TEE_L + s / 2, s), 6)

    def inside(u, v):
        body = abs(u) < half_w
        sleeve = abs(u) < reach and v > TEE_L - TEE_ARMHOLE
        neck = abs(u) < TEE_NECK_W / 2 and v > TEE_L - TEE_NECK_D
        return (body or sleeve) and not neck

    uv, front = _grid_patch(inside, us, vs)
    n = len(uv)
    edges = Counter(tuple(sorted(e)) for t in front for e in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])))
    directed = {(int(t[k]), int(t[(k + 1) % 3])) for t in front for k in range(3)}

    def opening(a, b):
        u, v = (uv[a] + uv[b]) / 2
        hem = v < 1e-6
        cuff = abs(u) > reach - 1e-6
        neck = abs(u) < TEE_NECK_W / 2 + 1e-6 and v > TEE_L - TEE_NECK_D - 1e-6  # shoulder seams lie outside it
        return hem or cuff or neck

    seams = [(a, b) for (a, b), k in edges.items() if k == 1 and not opening(a, b)]
    # front layer on top (+z), back layer below; the back's triangles are reversed so both face outward
    pts = np.vstack([np.c_[uv, np.full(n, TEE_GAP)], np.c_[uv, np.zeros(n)]])
    faces = [tuple(t) for t in front] + [(a + n, c + n, b + n) for a, b, c in front]
    for a, b in seams:  # a narrow sewn strip joins each seam edge of the front to the back
        if (a, b) not in directed:
            a, b = b, a
        faces += [(b, a, a + n), (b, a + n, b + n)]
    faces = np.array(faces, int)
    # trim (rib knit at the neck, the cuffs and the hem) in a contrasting colour
    c = np.r_[uv, uv][faces].mean(axis=1)
    rib = ((np.abs(c[:, 0]) < TEE_NECK_W / 2 + 0.012) & (c[:, 1] > TEE_L - TEE_NECK_D - 0.012)) | (
        np.abs(c[:, 0]) > reach - 0.012) | (c[:, 1] < 0.012)
    kind = rib.astype(int)
    rest = pts.copy()
    # start: lying flat on the table, front up, hem toward the arms: v -> +x, u -> +y
    start = np.c_[TEE_HEM_X + pts[:, 1], pts[:, 0], TOP + TEE_REST + pts[:, 2]]
    rest_world = np.c_[TEE_HEM_X + rest[:, 1], rest[:, 0], TOP + TEE_REST + rest[:, 2]]
    marks = {"n_layer": n, "uv": uv, "reach": reach,
             "hem_corners": [int(np.argmin(np.hypot(uv[:, 0] - sx * half_w, uv[:, 1]))) for sx in (1, -1)]}
    return Mesh(rest_world, faces, start, kind, marks)


def draped(rest_xy: np.ndarray, rail_x: float, radius: float) -> np.ndarray:
    """A flat sheet's points (x along the fold direction, y along the rail, centred on the fold line) folded over a
    rail at ``rail_x``: the middle wraps the rail's top at ``radius`` from its axis, both halves hang straight down."""
    s, y = rest_xy[:, 0], rest_xy[:, 1]
    quarter = math.pi * radius / 2
    out = np.zeros((len(s), 3))
    over = np.abs(s) <= quarter
    th = s[over] / radius
    out[over] = np.c_[rail_x + radius * np.sin(th), y[over], RAIL_Z + radius * np.cos(th)]
    side = np.sign(s[~over])
    out[~over] = np.c_[rail_x + side * radius, y[~over], RAIL_Z - (np.abs(s[~over]) - quarter)]
    return out


def washcloth(rail: str) -> Mesh:
    """A 25 cm square terry washcloth folded in half over ``rail``: flat rest shape, draped start shape. A row of
    loop-stitched hem along two edges is the trim."""
    n = int(round(CLOTH_SIDE / CLOTH_EDGE)) + 1
    xs = np.linspace(-CLOTH_SIDE / 2, CLOTH_SIDE / 2, n)
    pts, faces = _grid_patch(lambda x, y: True, xs, xs)
    # a slight offset from the fold line keeps the two halves from hanging exactly level (as a real one hangs)
    offset = {"B": 0.008, "C": -0.006}[rail]
    flat = pts + [offset, 0.0]
    start = draped(flat, RAIL_X[rail], RAIL_R + CLOTH_T / 2 + 0.0015)
    rest = np.c_[flat, np.zeros(len(flat))] + [RAIL_X[rail], 0.0, RAIL_Z + 0.05]
    c = flat[faces].mean(axis=1) - [offset, 0.0]
    kind = ((np.abs(c[:, 0]) > CLOTH_SIDE / 2 - 0.012) | (np.abs(c[:, 1]) > CLOTH_SIDE / 2 - 0.012)).astype(int)
    corners = [int(np.argmin(np.hypot(pts[:, 0] - sx * CLOTH_SIDE / 2, pts[:, 1] - sy * CLOTH_SIDE / 2)))
               for sx in (1, -1) for sy in (1, -1)]
    return Mesh(rest, faces, start, kind, {"corners": corners, "flat": flat})


if __name__ == "__main__":
    t = tshirt()
    print("tee", t.check(), "start z range", (t.start[:, 2] - TOP).min().round(4), (t.start[:, 2] - TOP).max().round(4),
          "x", t.start[:, 0].min().round(3), t.start[:, 0].max().round(3), "y", t.start[:, 1].min().round(3),
          t.start[:, 1].max().round(3))
    for r in LOADED:
        w = washcloth(r)
        print("cloth", r, w.check(), "start x", w.start[:, 0].min().round(4), w.start[:, 0].max().round(4),
              "z above table", (w.start[:, 2] - TOP).min().round(4), (w.start[:, 2] - TOP).max().round(4))
    print("cloth mass g", round(CLOTH_MASS * 1000, 1))
