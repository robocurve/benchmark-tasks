"""Small mesh kit for this task's visual parts: lathe, chamfered plates with holes, swept tubes. NumPy only.

Every mesh carries per-vertex normals and texture coordinates in metres times `UV_SCALE`, so a texture keeps one scale on all
parts. `usd_mesh` writes one as a plain-text USD prim.
"""

from __future__ import annotations

import math

import numpy as np

UV_SCALE = 1.0 / 0.05  # one texture tile per 5 cm


class Mesh:
    def __init__(self) -> None:
        self.v: list[np.ndarray] = []
        self.n: list[np.ndarray] = []
        self.t: list[np.ndarray] = []
        self.f: list[np.ndarray] = []
        self.count = 0

    def add(self, v, n, t, f) -> "Mesh":
        v = np.asarray(v, float).reshape(-1, 3)
        self.v.append(v)
        self.n.append(np.asarray(n, float).reshape(-1, 3))
        self.t.append(np.asarray(t, float).reshape(-1, 2))
        self.f.append(np.asarray(f, int).reshape(-1, 3) + self.count)
        self.count += len(v)
        return self

    def __iadd__(self, other: "Mesh") -> "Mesh":
        for v, n, t, f, base in zip(other.v, other.n, other.t, other.f, _offsets(other)):
            self.add(v, n, t, f - base)
        return self

    def moved(self, offset=(0.0, 0.0, 0.0), yaw: float = 0.0, rot: np.ndarray | None = None) -> "Mesh":
        """A copy turned (by `rot`, or `yaw` about z) and then shifted."""
        c, s = math.cos(yaw), math.sin(yaw)
        r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) if rot is None else rot
        out = Mesh()
        for v, n, t, f, base in zip(self.v, self.n, self.t, self.f, _offsets(self)):
            out.add(v @ r.T + np.asarray(offset, float), n @ r.T, t, f - base)
        return out

    def arrays(self):
        return np.vstack(self.v), np.vstack(self.n), np.vstack(self.t), np.vstack(self.f)


def _offsets(m: Mesh) -> list[int]:
    out, c = [], 0
    for v in m.v:
        out.append(c)
        c += len(v)
    return out


def lathe(profile, seg: int = 48) -> Mesh:
    """Revolve (radius, z) about z. Walk the profile with the solid on the left: bottom centre, outwards, up, back to the top
    centre. Each stretch of the profile is its own band, so every corner in the profile stays sharp."""
    m = Mesh()
    th = np.linspace(0.0, 2 * np.pi, seg + 1)
    co, si = np.cos(th), np.sin(th)
    idx = np.array([[i, i + 1, seg + 1 + i + 1, i, seg + 1 + i + 1, seg + 1 + i] for i in range(seg)]).reshape(-1, 3)
    for (r0, z0), (r1, z1) in zip(profile[:-1], profile[1:]):
        dr, dz = r1 - r0, z1 - z0
        length = math.hypot(dr, dz)
        if length < 1e-9:
            continue
        nr, nz = dz / length, -dr / length
        v0 = np.column_stack([r0 * co, r0 * si, np.full(seg + 1, z0)])
        v1 = np.column_stack([r1 * co, r1 * si, np.full(seg + 1, z1)])
        normal = np.column_stack([nr * co, nr * si, np.full(seg + 1, nz)])
        if abs(dz) < 0.3 * length:
            t0, t1 = v0[:, :2] * UV_SCALE, v1[:, :2] * UV_SCALE
        else:
            r = 0.5 * (r0 + r1)
            t0 = np.column_stack([th * r * UV_SCALE, np.full(seg + 1, z0 * UV_SCALE)])
            t1 = np.column_stack([th * r * UV_SCALE, np.full(seg + 1, z1 * UV_SCALE)])
        m.add(np.vstack([v0, v1]), np.vstack([normal, normal]), np.vstack([t0, t1]), idx)
    return m


def knurled_wall(r: float, z0: float, z1: float, teeth: int = 44, depth: float = 0.0005) -> Mesh:
    """A straight-knurled cylinder wall, flat shaded so the ridges catch the light."""
    m = Mesh()
    n = 2 * teeth
    th = np.linspace(0.0, 2 * np.pi, n + 1)
    rad = np.where(np.arange(n + 1) % 2 == 0, r, r - depth)
    for i in range(n):
        a = np.array([rad[i] * math.cos(th[i]), rad[i] * math.sin(th[i])])
        b = np.array([rad[i + 1] * math.cos(th[i + 1]), rad[i + 1] * math.sin(th[i + 1])])
        d = b - a
        nor = np.array([d[1], -d[0], 0.0]) / np.linalg.norm(d)
        v = [(a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1), (a[0], a[1], z1)]
        t = [(th[i] * r * UV_SCALE, z0 * UV_SCALE), (th[i + 1] * r * UV_SCALE, z0 * UV_SCALE), (th[i + 1] * r * UV_SCALE, z1 * UV_SCALE), (th[i] * r * UV_SCALE, z1 * UV_SCALE)]
        m.add(v, [nor] * 4, t, [[0, 1, 2], [0, 2, 3]])
    return m


def chain_outline(elements, keep_y=(), power: float = 16.0, step: float = 0.002) -> np.ndarray:
    """The outline, counter-clockwise, of a part symmetric about x = 0, as a union of `("bar", y0, y1, half_width)` and
    `("disc", y, radius)`. The union is a p-norm of the half-widths, which leaves small fillets where the shapes meet."""
    lo = min(e[1] if e[0] == "bar" else e[1] - e[2] for e in elements)
    hi = max(e[2] if e[0] == "bar" else e[1] + e[2] for e in elements)

    def half(y: np.ndarray) -> np.ndarray:
        total = np.zeros_like(y)
        for e in elements:
            if e[0] == "bar":
                h = np.where((y >= e[1] - 1e-9) & (y <= e[2] + 1e-9), e[3], 0.0)
            else:
                h = np.sqrt(np.maximum(e[2] ** 2 - (y - e[1]) ** 2, 0.0))
            total += h**power
        return total ** (1.0 / power)

    ys = list(np.arange(lo, hi, step)) + [hi] + list(keep_y)
    for e in elements:
        if e[0] == "disc":
            ys += list(e[1] + e[2] * np.sin(np.radians(np.arange(-90.0, 90.1, 5.0))))
    ys = np.unique(np.round(np.clip(ys, lo, hi), 6))
    hs = half(ys)
    keep = np.ones(len(ys), bool)
    locked = {round(float(k), 6) for k in keep_y}
    last = 0
    for i in range(1, len(ys) - 1):  # drop points that a straight line through their neighbours already passes
        if round(float(ys[i]), 6) in locked:
            last = i
            continue
        t = (ys[i] - ys[last]) / (ys[i + 1] - ys[last])
        if abs(hs[last] + t * (hs[i + 1] - hs[last]) - hs[i]) < 2e-5:
            keep[i] = False
        else:
            last = i
    ys, hs = ys[keep], hs[keep]
    right = [(h, y) for h, y in zip(hs, ys) if h > 1e-5]
    left = [(-h, y) for h, y in zip(hs[::-1], ys[::-1]) if h > 1e-5]
    if hs[0] <= 1e-5:
        right.insert(0, (0.0, ys[0]))
    if hs[-1] <= 1e-5:
        right.append((0.0, ys[-1]))
    return np.array(right + left)


def rounded_rect(cx: float, cy: float, sx: float, sy: float, r: float, seg: int = 6) -> np.ndarray:
    pts = []
    for (qx, qy), a0 in zip([(1, 1), (-1, 1), (-1, -1), (1, -1)], [0.0, 90.0, 180.0, 270.0]):
        c = (cx + qx * (sx / 2 - r), cy + qy * (sy / 2 - r))
        for a in np.radians(np.linspace(a0, a0 + 90.0, seg + 1)):
            pts.append((c[0] + r * math.cos(a), c[1] + r * math.sin(a)))
    pts = np.array(pts)
    i = np.argmin(pts[:, 1] - 1e-6 * pts[:, 0])
    return np.roll(pts, -i, axis=0)


def _outward(c: np.ndarray) -> np.ndarray:
    d = np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0)
    n = np.column_stack([d[:, 1], -d[:, 0]])
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def _ear_clip(p: np.ndarray) -> list[tuple[int, int, int]]:
    """Triangulate a simple counter-clockwise polygon, which may hold the zero-width slits that join holes to the outline."""
    idx = list(range(len(p)))
    tris: list[tuple[int, int, int]] = []
    k = 0
    tries = 0
    while len(idx) > 3 and tries < 2 * len(idx) + 8:
        n = len(idx)
        a, b, c = idx[(k - 1) % n], idx[k % n], idx[(k + 1) % n]
        pa, pb, pc = p[a], p[b], p[c]
        area = (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pb[1] - pa[1]) * (pc[0] - pa[0])
        ok = area > 1e-12
        if ok:
            q = p[idx]
            same = (np.abs(q - pa).sum(1) < 1e-9) | (np.abs(q - pb).sum(1) < 1e-9) | (np.abs(q - pc).sum(1) < 1e-9)
            d0 = (pb[0] - pa[0]) * (q[:, 1] - pa[1]) - (pb[1] - pa[1]) * (q[:, 0] - pa[0])
            d1 = (pc[0] - pb[0]) * (q[:, 1] - pb[1]) - (pc[1] - pb[1]) * (q[:, 0] - pb[0])
            d2 = (pa[0] - pc[0]) * (q[:, 1] - pc[1]) - (pa[1] - pc[1]) * (q[:, 0] - pc[0])
            ok = not np.any((d0 >= -1e-12) & (d1 >= -1e-12) & (d2 >= -1e-12) & ~same)
        if ok:
            tris.append((a, b, c))
            idx.pop(k % n)
            k = max(k - 1, 0)
            tries = 0
        elif abs(area) <= 1e-12:  # a straight run or a spike left by a slit
            idx.pop(k % n)
            tries = 0
        else:
            k += 1
            tries += 1
    if len(idx) == 3:
        tris.append(tuple(idx))
    return tris


def _with_holes(outer: np.ndarray, rings: list[np.ndarray]) -> np.ndarray:
    poly = [tuple(q) for q in outer]
    for ring in rings:
        hole = ring[::-1]  # clockwise
        s = int(np.argmax(hole[:, 0]))
        hole = np.roll(hole, -s, axis=0)
        m = hole[0]
        arr = np.array(poly)
        right = arr[:, 0] > m[0]
        d = np.hypot(*(arr - m).T)
        d[~right] = 1e9
        j = int(np.argmin(d))
        poly = poly[: j + 1] + [tuple(q) for q in hole] + [tuple(hole[0]), poly[j]] + poly[j + 1 :]
    return np.array(poly)


def _circle(x: float, y: float, r: float, seg: int) -> np.ndarray:
    a = np.linspace(0.0, 2 * np.pi, seg, endpoint=False)
    return np.column_stack([x + r * np.cos(a), y + r * np.sin(a)])


def plate(outer: np.ndarray, holes, z0: float, z1: float, chamfer: float = 0.0008, hole_chamfer: float = 0.0008, hole_seg: int = 28) -> tuple[Mesh, Mesh]:
    """A flat plate with chamfered edges. Returns (body, bores): the plate, and the bores' walls for a darker material."""
    body, bores = Mesh(), Mesh()
    outer = np.asarray(outer, float)
    inner = outer - _outward(outer) * chamfer
    rings = [_circle(x, y, r + hole_chamfer, hole_seg) for x, y, r in holes]
    poly = _with_holes(inner, rings)
    tris = np.array(_ear_clip(poly))
    uv = poly * UV_SCALE
    up = np.tile((0.0, 0.0, 1.0), (len(poly), 1))
    body.add(np.column_stack([poly, np.full(len(poly), z1)]), up, uv, tris)
    body.add(np.column_stack([poly, np.full(len(poly), z0)]), -up, uv + 0.37, tris[:, ::-1])
    n = len(outer)
    nor = _outward(outer)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(outer, axis=0), axis=1))] * UV_SCALE

    def band(pa, za, pb, zb, normal):
        va = np.column_stack([pa, np.full(n, za)])
        vb = np.column_stack([pb, np.full(n, zb)])
        ta = np.column_stack([s, np.full(n, za * UV_SCALE)])
        tb = np.column_stack([s, np.full(n, zb * UV_SCALE)])
        f = [[i, (i + 1) % n, n + (i + 1) % n, i, n + (i + 1) % n, n + i] for i in range(n)]
        body.add(np.vstack([va, vb]), np.vstack([normal, normal]), np.vstack([ta, tb]), f)

    k = math.sqrt(0.5)
    band(inner, z0, outer, z0 + chamfer, np.column_stack([k * nor, np.full(n, -k)]))
    band(outer, z0 + chamfer, outer, z1 - chamfer, np.column_stack([nor, np.zeros(n)]))
    band(outer, z1 - chamfer, inner, z1, np.column_stack([k * nor, np.full(n, k)]))
    for x, y, r in holes:
        wall = lathe([(r + hole_chamfer, z1), (r, z1 - hole_chamfer), (r, z0 + hole_chamfer), (r + hole_chamfer, z0)], hole_seg)
        bores += wall.moved((x, y, 0.0))
    return body, bores


def block(cx: float, cy: float, sx: float, sy: float, z0: float, z1: float, r: float = 0.002, chamfer: float = 0.0007) -> Mesh:
    return plate(rounded_rect(cx, cy, sx, sy, r), [], z0, z1, chamfer)[0]


def decal(x0: float, x1: float, y0: float, y1: float, z: float) -> Mesh:
    """A flat painted rectangle, facing up."""
    v = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
    t = [(q[0] * UV_SCALE, q[1] * UV_SCALE) for q in v]
    return Mesh().add(v, [(0.0, 0.0, 1.0)] * 4, t, [[0, 1, 2], [0, 2, 3]])


def tube(path, radius: float, seg: int = 10) -> Mesh:
    """A round tube swept along `path` (points), its cross-section carried without twist."""
    p = np.asarray(path, float)
    d = np.gradient(p, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    a = np.array([1.0, 0.0, 0.0]) if abs(d[0][0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(d[0], a)
    u /= np.linalg.norm(u)
    vs, ns, ts = [], [], []
    th = np.linspace(0.0, 2 * np.pi, seg + 1)
    length = 0.0
    for i, q in enumerate(p):
        if i:
            u = u - u.dot(d[i]) * d[i]
            u /= np.linalg.norm(u)
            length += float(np.linalg.norm(q - p[i - 1]))
        w = np.cross(d[i], u)
        for t in th:
            nor = math.cos(t) * u + math.sin(t) * w
            vs.append(q + radius * nor)
            ns.append(nor)
            ts.append((t * radius * UV_SCALE, length * UV_SCALE))
    r = seg + 1
    f = [[i * r + j, i * r + j + 1, (i + 1) * r + j + 1, i * r + j, (i + 1) * r + j + 1, (i + 1) * r + j] for i in range(len(p) - 1) for j in range(seg)]
    return Mesh().add(vs, ns, ts, f)


def coil(radius: float, wire: float, z0: float, z1: float, turns: float, start: float = 0.0) -> Mesh:
    """A helical spring about z."""
    a = np.linspace(start, start + turns * 2 * np.pi, int(turns * 28) + 1)
    z = np.linspace(z0, z1, len(a))
    return tube(np.column_stack([radius * np.cos(a), radius * np.sin(a), z]), wire, 8)


def socket_screw(x: float, y: float, z: float, r: float = 0.0032) -> tuple[Mesh, Mesh]:
    """A socket-head cap screw standing on z: (head, socket)."""
    h = 1.15 * r
    head = lathe([(r * 0.92, z), (r, z + 0.0003), (r, z + h - 0.0003), (r * 0.92, z + h), (0.55 * r, z + h)], 14)
    socket = lathe([(0.55 * r, z + h + 0.00005), (0.0, z + h + 0.00005)], 6)
    return head.moved((x, y, 0.0)), socket.moved((x, y, 0.0))


def usd_mesh(name: str, mesh: Mesh, material_path: str, indent: str = "        ") -> str:
    """One mesh as a USD prim, with its normals and texture coordinates, bound to a material."""
    v, n, t, f = mesh.arrays()
    points = ", ".join(f"({x:.5f}, {y:.5f}, {z:.5f})" for x, y, z in v)
    normals = ", ".join(f"({x:.4f}, {y:.4f}, {z:.4f})" for x, y, z in n)
    st = ", ".join(f"({x:.4f}, {y:.4f})" for x, y in t)
    indices = ", ".join(str(int(i)) for i in f.reshape(-1))
    counts = ", ".join("3" for _ in range(len(f)))
    return f"""
{indent}def Mesh "{name}" (
{indent}    prepend apiSchemas = ["MaterialBindingAPI"]
{indent})
{indent}{{
{indent}    rel material:binding = <{material_path}>
{indent}    uniform bool doubleSided = 1
{indent}    int[] faceVertexCounts = [{counts}]
{indent}    int[] faceVertexIndices = [{indices}]
{indent}    point3f[] points = [{points}]
{indent}    normal3f[] normals = [{normals}] (
{indent}        interpolation = "vertex"
{indent}    )
{indent}    texCoord2f[] primvars:st = [{st}] (
{indent}        interpolation = "vertex"
{indent}    )
{indent}    uniform token subdivisionScheme = "none"
{indent}}}"""


def read_stl(path: str) -> np.ndarray:
    """A binary STL's triangles, as an array of shape (n, 3, 3), in the file's units."""
    with open(path, "rb") as f:
        data = f.read()
    n = int.from_bytes(data[80:84], "little")
    rec = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n, offset=84)
    return rec["v"].reshape(-1, 3, 3).astype(float)


def creased_mesh(tri: np.ndarray, crease_deg: float = 32.0, uv_scale: float = 1.0 / 0.032) -> Mesh:
    """A triangle soup as a mesh with smooth normals across edges flatter than `crease_deg`, and textures laid flat on flat faces and round the axis on walls."""
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-15)
    pts = tri.reshape(-1, 3)
    _, ids = np.unique(np.round(pts * 1e6).astype(np.int64), axis=0, return_inverse=True)
    ids = ids.reshape(-1)
    adjacent: dict[int, list[int]] = {}
    for corner, vid in enumerate(ids):
        adjacent.setdefault(int(vid), []).append(corner // 3)
    limit = math.cos(math.radians(crease_deg))
    normals = np.zeros_like(pts)
    for corner, vid in enumerate(ids):
        face = corner // 3
        near = np.array(adjacent[int(vid)])
        keep = near[n[near] @ n[face] > limit]
        v = n[keep].sum(axis=0)
        normals[corner] = v / max(np.linalg.norm(v), 1e-15)
    flat = np.repeat(np.abs(n[:, 2]) > 0.7, 3)
    uv = np.where(flat[:, None], pts[:, :2], np.column_stack([np.arctan2(pts[:, 1], pts[:, 0]) * np.hypot(pts[:, 0], pts[:, 1]), pts[:, 2]])) * uv_scale
    return Mesh().add(pts, normals, uv, np.arange(len(pts)).reshape(-1, 3))
