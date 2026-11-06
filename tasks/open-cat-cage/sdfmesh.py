"""Build smooth organic meshes from signed distance functions, unwrap them and bake a colour texture. Used only by make_animals.py, offline: nothing here runs at task load."""

from __future__ import annotations

import numpy as np
from scipy import ndimage
from skimage import measure
import fast_simplification
import xatlas


def smin(a: np.ndarray, b: np.ndarray, k: float) -> np.ndarray:
    """Smooth union of two distance fields; k is the blend width in metres."""
    if k <= 0:
        return np.minimum(a, b)
    h = np.clip(0.5 + 0.5 * (b - a) / k, 0.0, 1.0)
    return b * (1 - h) + a * h - k * h * (1 - h)


def cone(p: np.ndarray, a, b, ra: float, rb: float) -> np.ndarray:
    """A capsule from a to b whose radius runs from ra to rb."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1) - (ra + t * (rb - ra))


def chain(p: np.ndarray, pts, radii, k: float = 0.0) -> np.ndarray:
    """Cones through a list of points."""
    d = None
    for i in range(len(pts) - 1):
        c = cone(p, pts[i], pts[i + 1], radii[i], radii[i + 1])
        d = c if d is None else smin(d, c, k)
    return d


def ellipsoid(p: np.ndarray, c, r, yaw: float = 0.0, pitch: float = 0.0) -> np.ndarray:
    """An ellipsoid with radii r along its own axes, turned by yaw about z and then pitch about its y."""
    q = p - np.asarray(c, float)
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    x = q[..., 0] * cy + q[..., 1] * sy
    y = -q[..., 0] * sy + q[..., 1] * cy
    z = q[..., 2]
    x, z = x * cp - z * sp, x * sp + z * cp
    r = np.asarray(r, float)
    n = np.stack([x / r[0], y / r[1], z / r[2]], axis=-1)
    k = np.linalg.norm(n, axis=-1)
    k2 = np.linalg.norm(n / r, axis=-1)
    return k * (k - 1.0) / np.maximum(k2, 1e-9)


def flat_cone(p: np.ndarray, base, tip, width: float, thick: float, normal) -> np.ndarray:
    """A flattened cone (an ear, a wing tip): it tapers from `width` x `thick` at base to a point at tip. `normal` is the flat side's direction."""
    base, tip = np.asarray(base, float), np.asarray(tip, float)
    axis = tip - base
    h = np.linalg.norm(axis)
    axis = axis / h
    n = np.asarray(normal, float)
    n = n - (n @ axis) * axis
    n /= np.linalg.norm(n)
    w = np.cross(axis, n)
    q = p - base
    u, a, b = q @ axis, q @ w, q @ n
    t = np.clip(u / h, 0.0, 1.0)
    taper = np.maximum(1.0 - t, 0.04)
    e = np.sqrt((a / (width * taper)) ** 2 + (b / (thick * taper)) ** 2) - 1.0
    return np.maximum(e * thick * 0.8, np.maximum(-u, u - h))


def smooth_noise(p: np.ndarray, seed: int, scale: float, octaves: int = 3) -> np.ndarray:
    """Band-limited noise of about unit range at a feature size of `scale` metres: sums of random plane waves."""
    rng = np.random.default_rng(seed)
    out = np.zeros(p.shape[:-1])
    amp = 1.0
    for o in range(octaves):
        for _ in range(6):
            d = rng.normal(size=3)
            d /= np.linalg.norm(d)
            out = out + amp * np.sin(2 * np.pi * (p @ d) / (scale / 2**o) + rng.uniform(0, 2 * np.pi))
        amp *= 0.55
    return out / 4.0


def polygonise(field, bounds, voxel: float, target_faces: int):
    """Marching cubes over `field(points)` within bounds ((x0, y0, z0), (x1, y1, z1)), decimated to about target_faces."""
    lo, hi = np.asarray(bounds[0], float), np.asarray(bounds[1], float)
    n = np.ceil((hi - lo) / voxel).astype(int) + 1
    axes = [lo[i] + voxel * np.arange(n[i]) for i in range(3)]
    grid = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1)
    vol = np.empty(grid.shape[:3], dtype=np.float32)
    for i in range(n[0]):  # by slices: the grids are large
        vol[i] = field(grid[i])
    v, f, _, _ = measure.marching_cubes(vol, 0.0, spacing=(voxel, voxel, voxel))
    v = v + lo
    f = f[:, ::-1]  # outward, counter-clockwise
    r = 1.0 - min(1.0, target_faces / len(f))
    if r > 0:
        v, f = fast_simplification.simplify(v.astype(np.float32), f.astype(np.int32), target_reduction=r)
    return np.asarray(v, float), np.asarray(f, int)


def smooth_normals(v: np.ndarray, f: np.ndarray) -> np.ndarray:
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    vn = np.zeros_like(v)
    for k in range(3):
        np.add.at(vn, f[:, k], fn)
    return vn / np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)


def bake(v: np.ndarray, f: np.ndarray, colour, size: int = 1024):
    """Unwrap the mesh and bake `colour(points, normals) -> rgb in 0..1` into a texture. Returns (corner uvs [m, 3, 2], image [size, size, 3] uint8, corner normals [m, 3, 3])."""
    vmap, idx, uvs = xatlas.parametrize(v.astype(np.float32), f.astype(np.uint32))
    idx = idx.reshape(-1, 3)
    vn = smooth_normals(v, f)
    vv, nn = v[vmap], vn[vmap]
    xy = uvs * (size - 1)
    pos = np.zeros((size, size, 3))
    nor = np.zeros((size, size, 3))
    mask = np.zeros((size, size), bool)
    for t in idx:
        a, b, c = xy[t[0]], xy[t[1]], xy[t[2]]
        x0, x1 = int(np.floor(min(a[0], b[0], c[0]))), int(np.ceil(max(a[0], b[0], c[0])))
        y0, y1 = int(np.floor(min(a[1], b[1], c[1]))), int(np.ceil(max(a[1], b[1], c[1])))
        X, Y = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
        d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(d) < 1e-9:
            continue
        w0 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / d
        w1 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / d
        w2 = 1 - w0 - w1
        m = (w0 >= -0.02) & (w1 >= -0.02) & (w2 >= -0.02)
        ys, xs = Y[m], X[m]
        ok = (xs >= 0) & (xs < size) & (ys >= 0) & (ys < size)
        ys, xs = ys[ok], xs[ok]
        i = np.nonzero(m)
        w = np.stack([w0[i][ok], w1[i][ok], w2[i][ok]], axis=1)
        pos[ys, xs] = w @ vv[t]
        nor[ys, xs] = w @ nn[t]
        mask[ys, xs] = True
    rgb = np.zeros((size, size, 3))
    rows = np.nonzero(mask)
    nor_m = nor[rows]
    nor_m /= np.maximum(np.linalg.norm(nor_m, axis=1, keepdims=True), 1e-12)
    rgb[rows] = colour(pos[rows], nor_m)
    near = ndimage.distance_transform_edt(~mask, return_distances=False, return_indices=True)  # fill the gaps between islands, so that mip maps do not bleed
    rgb = rgb[near[0], near[1]]
    img = np.uint8(np.clip(rgb, 0, 1) * 255 + 0.5)[::-1]  # image rows run down, v runs up
    corner_uv = uvs[idx]
    corner_n = nn[idx]
    return vv, idx, corner_uv, img, corner_n
