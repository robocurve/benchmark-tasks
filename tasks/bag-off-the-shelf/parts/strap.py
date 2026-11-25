"""The stiff tote's back strap: two soft legs, each sewn to the outside of the back wall just below the rim and running
down and out to one end of a rigid arch handle, which lies flat round the rake's tines. Frame: the bag frame (origin at the
centre of the tote's floor underside, x along its depth, y along its width, z up).

The legs are about 22 cm long. That is as short as they can be with the arch clear of the rake's handle, and it leaves no
slack with the tote lifted 12 cm: the strap drags the rake as soon as the tote is pulled. The +y leg runs low, under a hand
reaching in for a tine from +y. The -y leg runs high, above the rake's handle even when the rake is lifted 6 cm, so lifting the
rake does not lift the arch.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import trimesh

SEWN = (0.006, 0.19)  # the legs' upper ends: this far out from the back wall, this high above the tote's floor
FOOT = (0.085, 0.04)  # where the +y leg stops hanging: this far from the tote's centre along x, this high above the shelf
ARCH_PAST_RAKE = 0.024  # the arch's ends, this far beyond the rake's handle along x
OVER_HANDLE = (0.013, 0.07)  # the -y leg's last point above the handle: this far beyond it along x, this high
THICKNESS = 0.005  # as drawn; the simulated cloth is a surface


@dataclass(frozen=True)
class ToteSpec:
    """The tote's dimensions and the strap's mesh."""

    width: float = 0.20  # along y
    depth: float = 0.10  # along x
    height: float = 0.20
    cell: float = 0.0125  # the cloth's grid spacing
    strap_y: float = 0.0625  # each leg's centre line, from the tote's centre plane
    strap_width: float = 0.025


def _grid(n_u: int, n_v: int) -> np.ndarray:
    faces = []
    for i in range(n_u):
        for j in range(n_v):
            a, b = i * (n_v + 1) + j, i * (n_v + 1) + j + 1
            c, d = (i + 1) * (n_v + 1) + j, (i + 1) * (n_v + 1) + j + 1
            faces += [[a, c, b], [b, c, d]]
    return np.array(faces, dtype=np.int64)


def strap_legs(spec: ToteSpec, rake_x: float):
    """The legs' mesh, the indices of their sewn vertices and of their arch-end vertices, and the arch's two ends."""
    S, c = spec.strap_y, spec.cell

    def seg(a, b):  # points from a to b (excluding b), about one cell apart
        n = max(1, int(np.ceil(np.linalg.norm(np.subtract(b, a)) / c)))
        return [np.add(a, np.subtract(b, a) * i / n) for i in range(n)]

    x_arch = rake_x + ARCH_PAST_RAKE
    sewn = (spec.depth / 2 + SEWN[0], SEWN[1])
    arch_end = (x_arch, 0.0)
    over = (rake_x + OVER_HANDLE[0], OVER_HANDLE[1])
    lines = {-1.0: seg(sewn, over) + seg(over, arch_end) + [np.array(arch_end)],
             1.0: seg(sewn, FOOT) + seg(FOOT, arch_end) + [np.array(arch_end)]}
    n_w = round(spec.strap_width / c)
    ts = np.linspace(-spec.strap_width / 2, spec.strap_width / 2, n_w + 1)
    legs, rim, ends, offset = [], [], [], 0
    for sy in (-1.0, 1.0):
        pts = [np.array([x, sy * S, z]) for x, z in lines[sy]]
        verts = np.array([p + t * np.array([0.0, 1.0, 0.0]) for p in pts for t in ts])
        m = trimesh.Trimesh(verts, _grid(len(pts) - 1, n_w), process=False)
        rim += list(range(offset, offset + n_w + 1))
        ends += list(range(offset + len(m.vertices) - n_w - 1, offset + len(m.vertices)))
        offset += len(m.vertices)
        legs.append(m)
    arch_ends = (np.array([x_arch, -S, 0.0]), np.array([x_arch, S, 0.0]))
    return trimesh.util.concatenate(legs), np.array(rim), np.array(ends), arch_ends


def thicken(mesh: trimesh.Trimesh, thickness: float = THICKNESS) -> trimesh.Trimesh:
    """The cloth drawn as a shell round its simulated surface, closed along its open edges."""
    v, f = np.asarray(mesh.vertices), np.asarray(mesh.faces)
    n = np.zeros_like(v)
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    nv = len(v)
    verts = np.vstack([v + thickness / 2 * n, v - thickness / 2 * n])
    edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    uniq, count = np.unique(edges, axis=0, return_counts=True)
    open_edges = {tuple(e) for e in uniq[count == 1]}
    sides = []
    for a, b in np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]):
        if (min(a, b), max(a, b)) in open_edges:
            sides += [[a, a + nv, b + nv], [a, b + nv, b]]
    faces = np.vstack([f, f[:, ::-1] + nv, np.asarray(sides, dtype=np.int64).reshape(-1, 3)])
    return trimesh.Trimesh(verts, faces, process=False)
