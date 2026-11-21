"""The toddler manikin's head (about 2 to 3 years old) with its mouth open, as one triangle mesh (skin outside, mucosa
inside), and the signed distance of its surface. NumPy only (meshed once with VTK's flying-edges contour and cached
in ``child_head_mesh.npz`` next to this file). The nose comes from ``manikin_face.py`` (our adult care-manikin face),
scaled to 0.62.

Head frame (mm in the comments, m in the arrays): origin on the midline at the nasal root, x straight back from the
face, y to the head's left side as seen in the task (world +y), z up; the face looks toward -x.

Sizes (toddler, 2 to 3 years; rounded from published child anthropometry): head length 168 mm, head breadth 134 mm,
face height (nasal root to chin) 82 mm, mouth 46 mm wide when opened wide, lips opened 26 mm. Inside: the primary teeth
(incisors, canines, first and second molars) on upper and lower gum ridges, a 2 to 7 mm space between the side teeth
and the inner cheek (the buccal vestibule, the "cheek pocket"), the tongue, and the throat behind it. The teeth, gums,
tongue and the two inner cheek walls are separate colliders in the task file; this mesh is the skin, the lips and the
lining, with a recess in each side wall for the sprung inner cheek.
"""

from __future__ import annotations

import hashlib
import pathlib

import numpy as np

import manikin_face as mf

MM = 0.001
HERE = pathlib.Path(__file__).resolve().parent
M = np.array([-3.0, 0.0, -50.0]) * MM  # the middle of the lips' opening
NOSE_S = 0.62


def _box(p, c, h):
    q = np.abs(p - np.asarray(c, float)) - np.asarray(h, float)
    return np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(q.max(axis=-1), 0.0)


def _e(p, c_mm, r_mm):
    return mf._ellipsoid(p, np.array(c_mm) * MM, np.array(r_mm) * MM)


def _s(p, c_mm, r_mm):
    return mf._sphere(p, np.array(c_mm) * MM, r_mm * MM)


def outer_sdf(p: np.ndarray) -> np.ndarray:
    """The closed head: a large cranium, a small face with full cheeks, a short nose, lips, a small chin, a neck."""
    sm, smx = mf._smin, mf._smax
    d = _e(p, (82.0, 0.0, 26.0), (84.0, 67.0, 76.0))  # cranium: length 168, breadth 134
    d = sm(d, _e(p, (50.0, 0.0, -30.0), (56.0, 52.0, 46.0)), 14.0 * MM)  # face and cheeks
    d = sm(d, _e(p, (42.0, 0.0, -56.0), (48.0, 47.0, 27.0)), 12.0 * MM)  # jaw
    d = sm(d, _e(p, (4.0, 0.0, 9.0), (7.0, 28.0, 6.0)), 10.0 * MM)  # brow
    for k in (1.0, -1.0):
        d = sm(d, _s(p, (16.0, k * 27.0, -45.0), 19.0), 14.0 * MM)  # full cheeks
        d = sm(d, _s(p, (16.0, k * 30.0, -16.0), 11.0), 12.0 * MM)  # cheekbones
    d = sm(d, _e(p, (-1.0, 0.0, -37.0), (5.0, 18.0, 5.0)), 7.0 * MM)  # upper lip
    d = sm(d, _e(p, (-1.0, 0.0, -63.0), (5.5, 17.0, 5.0)), 7.0 * MM)  # lower lip
    d = sm(d, _e(p, (5.0, 0.0, -75.0), (9.0, 15.0, 9.0)), 12.0 * MM)  # chin
    for k in (1.0, -1.0):  # eye sockets
        d = smx(d, -_e(p, (5.0, k * 23.0, -3.0), (9.0, 13.5, 9.0)), 5.0 * MM)
    nose = NOSE_S * mf.nose_sdf(p / NOSE_S)
    d = sm(d, nose, 2.5 * MM)
    neck = mf._capsule(p, np.array([62.0, 0.0, -60.0]) * MM, np.array([70.0, 0.0, -260.0]) * MM, 37.0 * MM)
    return sm(d, neck, 14.0 * MM)


# -- the mouth (mm, head frame) ---------------------------------------------------------------------------------------
OPEN_C, OPEN_R = (-3.0, 0.0, -50.0), (11.0, 23.0, 13.0)  # the lips' opening: 46 x 26 mm
CAVITY_C, CAVITY_R = (20.0, 0.0, -50.0), (30.0, 24.0, 21.0)
PHARYNX_C, PHARYNX_R = (50.0, 0.0, -56.0), (14.0, 15.0, 28.0)
# a recess in each side wall for the sprung inner cheek (its face at |y| = 21.5 mm at rest, 3 mm thick, 4 mm travel)
RECESS_C, RECESS_H = (14.75, 24.75, -50.0), (12.25, 4.25, 12.0)


def cavity_sdf(p: np.ndarray) -> np.ndarray:
    """The air space of the open mouth (negative inside): the lips' opening, the mouth, the throat and the cheeks'
    recesses."""
    sm = mf._smin
    d = sm(_e(p, OPEN_C, OPEN_R), _e(p, CAVITY_C, CAVITY_R), 5.0 * MM)
    d = sm(d, _e(p, PHARYNX_C, PHARYNX_R), 6.0 * MM)
    for k in (1.0, -1.0):
        c = np.array([RECESS_C[0], k * RECESS_C[1], RECESS_C[2]]) * MM
        d = np.minimum(d, _box(p, c, np.array(RECESS_H) * MM))
    return d


def head_sdf(p: np.ndarray) -> np.ndarray:
    p = np.asarray(p, float)
    d = mf._smax(outer_sdf(p), -cavity_sdf(p), 1.5 * MM)
    return np.maximum(d, -(p @ BASE_N - BASE_D))


BASE_N = np.array([0.0, 0.0, 1.0])
BASE_D = -0.17


def set_base(n, d) -> None:
    global BASE_N, BASE_D
    BASE_N, BASE_D = np.asarray(n, float), float(d)


def build_mesh(step: float = 1.2 * MM, target_tris: int = 50000):
    """(points, faces, normals, inner): the surface meshed on a ``step`` grid (VTK flying edges), decimated to about
    ``target_tris`` triangles; ``inner`` marks faces of the mouth's lining (drawn as mucosa)."""
    import vtk
    from vtk.util import numpy_support as ns

    lo = np.array([-30.0, -75.0, BASE_D / MM - 3.0]) * MM
    hi = np.array([175.0, 75.0, 110.0]) * MM
    n = np.ceil((hi - lo) / step).astype(int) + 1
    xs, ys, zs = (lo[k] + np.arange(n[k]) * step for k in range(3))
    vals = np.empty((n[2], n[1], n[0]), np.float32)
    for k, z in enumerate(zs):
        X, Y = np.meshgrid(xs, ys, indexing="xy")
        pts = np.stack([X, Y, np.full_like(X, z)], -1).reshape(-1, 3)
        vals[k] = head_sdf(pts).reshape(n[1], n[0])
    img = vtk.vtkImageData()
    img.SetDimensions(int(n[0]), int(n[1]), int(n[2]))
    img.SetSpacing(step, step, step)
    img.SetOrigin(*lo)
    img.GetPointData().SetScalars(ns.numpy_to_vtk(vals.ravel(), deep=True))
    fe = vtk.vtkFlyingEdges3D()
    fe.SetInputData(img)
    fe.SetValue(0, 0.0)
    fe.ComputeNormalsOff()
    fe.Update()
    conn = vtk.vtkPolyDataConnectivityFilter()
    conn.SetInputConnection(fe.GetOutputPort())
    conn.SetExtractionModeToLargestRegion()
    conn.Update()
    tri = vtk.vtkTriangleFilter()
    tri.SetInputConnection(conn.GetOutputPort())
    tri.Update()
    total = tri.GetOutput().GetNumberOfPolys()
    dec = vtk.vtkQuadricDecimation()
    dec.SetInputConnection(tri.GetOutputPort())
    dec.SetTargetReduction(max(0.0, 1.0 - target_tris / max(total, 1)))
    dec.Update()
    poly = dec.GetOutput()
    points = ns.vtk_to_numpy(poly.GetPoints().GetData()).astype(np.float64)
    faces = ns.vtk_to_numpy(poly.GetPolys().GetData()).reshape(-1, 4)[:, 1:].astype(np.int32)
    c = points[faces].mean(axis=1)
    fn = np.cross(points[faces[:, 1]] - points[faces[:, 0]], points[faces[:, 2]] - points[faces[:, 0]])
    h = 0.3 * MM
    g = np.stack([head_sdf(c + np.eye(3)[k] * h) - head_sdf(c - np.eye(3)[k] * h) for k in range(3)], -1)
    flip = np.sum(fn * g, axis=1) < 0
    faces[flip] = faces[flip][:, ::-1]
    grad = np.stack([head_sdf(points + np.eye(3)[k] * h) - head_sdf(points - np.eye(3)[k] * h) for k in range(3)], -1)
    normals = grad / np.maximum(np.linalg.norm(grad, axis=-1, keepdims=True), 1e-12)
    inner = cavity_sdf(c) > -2.0 * MM
    inner &= np.abs(outer_sdf(c)) > 1.0 * MM
    inner &= c[:, 0] > 0.0
    return points, faces, normals, inner


def cached_mesh():
    src = (HERE / "child_head.py").read_bytes() + (HERE / "manikin_face.py").read_bytes()
    key = hashlib.sha256(src + np.asarray([*BASE_N, BASE_D]).tobytes()).hexdigest()[:16]
    cache = HERE / "child_head_mesh.npz"
    if cache.exists():
        d = np.load(cache)
        if str(d["key"]) == key:
            return d["points"].astype(np.float64), d["faces"], d["normals"].astype(np.float64), d["inner"]
    pts, faces, normals, inner = build_mesh()
    np.savez_compressed(cache, key=key, points=pts.astype(np.float32), faces=faces,
                        normals=normals.astype(np.float32), inner=inner)
    return pts, faces, normals, inner
