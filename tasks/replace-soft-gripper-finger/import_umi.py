"""Turn the UMI gripper's published print files into the mesh files in umi_parts/.

The files are the CE3S1_0.6_finger, _gripper_covers and _gripper_internals 3MF plates, linked from the UMI hardware guide. This reads their triangles, in millimetres
as they lie on the plate, splits each finger holder's upper fork plate from the rest, finds the screw holes, and saves metres, triangles and per-corner normals.

Usage: python import_umi.py DIR_WITH_3MF_FILES OUT_DIR
"""

from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import trimesh

NS = "{http://schemas.microsoft.com/3dmanufacturing/core/2015/02}"
UPPER_FORK_Y = 13.1  # mm: the holder's upper fork plate lies above this
WEB_FRONT_Z = 6.75  # mm: and in front of the web that joins the two fork plates


def read(path: str, want: str) -> trimesh.Trimesh:
    root = ET.fromstring(zipfile.ZipFile(path).read("3D/3dmodel.model"))
    for obj in root.iter(NS + "object"):
        if want in (obj.get("name") or ""):
            mesh = obj.find(NS + "mesh")
            v = [[float(a.get(k)) for k in "xyz"] for a in mesh.find(NS + "vertices")]
            f = [[int(a.get(k)) for k in ("v1", "v2", "v3")] for a in mesh.find(NS + "triangles")]
            return trimesh.Trimesh(np.array(v), np.array(f), process=True)
    raise KeyError(want)


def holes(m: trimesh.Trimesh, radius=(1.6, 2.2)) -> np.ndarray:
    """Round holes along y: (x, z, top of the ring's y) in mm, one row per hole."""
    edges = m.face_adjacency_edges[m.face_adjacency_angles > np.radians(25)]
    rows: dict[tuple, float] = {}
    for comp in trimesh.graph.connected_components(edges, min_len=6):
        p = m.vertices[comp]
        if np.ptp(p[:, 1]) > 0.05:
            continue
        c = p[:, [0, 2]].mean(0)
        r = np.linalg.norm(p[:, [0, 2]] - c, axis=1)
        if radius[0] <= r.mean() <= radius[1] and r.std() < 0.1:
            key = tuple(np.round(c, 1))
            rows[key] = max(rows.get(key, -1e9), float(p[0, 1]))
    return np.array([(k[0], k[1], v) for k, v in sorted(rows.items())])


def corner_normals(v: np.ndarray, f: np.ndarray, crease_deg: float = 35.0) -> np.ndarray:
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    fn /= np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-12)
    around: dict[int, list[int]] = {}
    for i, tri in enumerate(f):
        for c in tri:
            around.setdefault(int(c), []).append(i)
    cos_crease = np.cos(np.radians(crease_deg))
    out = np.zeros((len(f), 3, 3), np.float32)
    for i, tri in enumerate(f):
        for k, c in enumerate(tri):
            faces = around[int(c)]
            near = fn[faces] @ fn[i] > cos_crease
            n = fn[faces][near].sum(axis=0)
            out[i, k] = n / max(np.linalg.norm(n), 1e-12)
    return out


def _clip(tri: np.ndarray, sign: float) -> np.ndarray:
    """The part of each triangle where sign * (z - WEB_FRONT_Z) <= 0, in triangles with the same winding."""
    out = []
    for t in tri:
        d = sign * (t[:, 2] - WEB_FRONT_Z)
        poly = []
        for i in range(3):
            a, b, da, db = t[i], t[(i + 1) % 3], d[i], d[(i + 1) % 3]
            if da <= 0:
                poly.append(a)
            if (da < 0 < db) or (db < 0 < da):
                poly.append(a + (b - a) * da / (da - db))
        out += [(poly[0], poly[k], poly[k + 1]) for k in range(1, len(poly) - 1)]
    return np.array(out)


def split_fork(m: trimesh.Trimesh) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Cut the holder along the upper fork plate's back edge, so neither part keeps a triangle that crosses it, and close both cut faces. Returns the rest and the plate."""
    tri = m.triangles
    upper = tri[:, :, 1].min(axis=1) > UPPER_FORK_Y - 1e-6
    fork = _clip(tri[upper], 1.0)
    rest_upper = _clip(tri[upper], -1.0)
    assert abs(trimesh.triangles.area(fork).sum() + trimesh.triangles.area(rest_upper).sum() - trimesh.triangles.area(tri[upper]).sum()) < 1e-6
    on_cut = fork.reshape(-1, 3)
    on_cut = on_cut[np.abs(on_cut[:, 2] - WEB_FRONT_Z) < 1e-6]
    x0, x1 = on_cut[:, 0].min(), on_cut[:, 0].max()
    y0, y1 = UPPER_FORK_Y, tri[upper][:, :, 1].max()
    a, b, c, d = ([x, y, WEB_FRONT_Z] for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)))
    faces_back = np.array([(a, c, b), (a, d, c)])  # looking along +z, the way the plate's back edge faces
    faces_front = faces_back[:, ::-1]

    def mesh(triangles: np.ndarray) -> trimesh.Trimesh:
        return trimesh.Trimesh(triangles.reshape(-1, 3), np.arange(len(triangles) * 3).reshape(-1, 3), process=True)

    return mesh(np.concatenate([tri[~upper], rest_upper, faces_front])), mesh(np.concatenate([fork, faces_back]))


def save(out: str, name: str, m: trimesh.Trimesh, faces=None, **extra) -> None:
    f = m.faces if faces is None else m.faces[faces]
    used, inverse = np.unique(f, return_inverse=True)
    f2 = inverse.reshape(-1, 3)
    v = m.vertices[used]
    np.savez_compressed(os.path.join(out, f"{name}.npz"), v=(v / 1000.0).astype(np.float32), f=f2.astype(np.int32), n=(corner_normals(v, f2) * 127).astype(np.int8), **extra)
    print(name, len(f2), "tris", np.round(np.ptp(v, axis=0), 1), {k: np.shape(x) for k, x in extra.items()})


def main() -> None:
    src, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    finger = read(f"{src}/finger.3mf", "soft_gripper")
    hull = finger.convex_hull
    save(out, "finger", finger, hull_v=(hull.vertices / 1000.0).astype(np.float32), hull_f=hull.faces.astype(np.int32))
    for name, want in (("holder_left", "finger_holder_left"), ("holder_right", "finger_holder_right")):
        m = read(f"{src}/gripper_internals.3mf", want)
        rest, fork = split_fork(m)
        save(out, name, rest, holes=(holes(m) / 1000.0).astype(np.float32))
        save(out, name + "_fork", fork)
    cover = read(f"{src}/gripper_covers.3mf", "top_cover")
    plate = read(f"{src}/gripper_covers.3mf", "bottom_plate")
    save(out, "top_cover", cover, holes=(holes(cover) / 1000.0).astype(np.float32))
    save(out, "bottom_plate", plate)


if __name__ == "__main__":
    main()
