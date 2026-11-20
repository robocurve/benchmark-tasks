"""yamgeom: the robot's collision hulls from the competition environment, as surface points for contact checks.

This folder ships no robot geometry. The hulls come from the environment at run time: yam_arena.analysis.link_hulls()
reads every <collision> hull that yam_arena.YAM_URDF lists, in its link's frame (the robot USD the simulator loads was
converted from the same URDF; check() confirms the two agree). PhysX cooks a hull with more than 64 vertices down to
64 or fewer, so the simulated collider can differ slightly from these hulls.

    yg.points(link): surface points (n x 3, m, the link's own frame) of the link's hulls, in rows at most SPACING
        (0.5 mm) apart on each face and at most SPACING apart along each row, less the points inside another hull of
        the same link (never the nearest to anything outside the link). Measured on links 3-5, the gripper and both
        fingertips: every point of the hulls' outer surface lies within about 0.5 mm of one (at most 0.51 mm, near
        the seams between overlapping hulls; 0.30 mm at the 99th percentile).
    yg.tcp_points(tip): the same for "tip_left" / "tip_right" with the gripper closed, in the tool point's frame (the
        gripper's frame moved by yam_arena.constants.TCP_OFFSET).
    yg.cloud(name): (points, centroid, radius about it) of a link or of "<tip>_tcp", for culling.
    yg.near(name, place, gap, cutoff): the placed points of cloud(name) that can lie within cutoff of an object (whole
        1 cm cells of points left out only when they cannot), so a contact check scans the few cells near it.
    yg.box(points, lo, hi): (center, size) of the axis-aligned box around the points within lo..hi.
    yg.proud_points, yg.visual_vertices, yg.turn_to: test aids (where the hulls bridge pockets of the visual mesh).

Points are built on first use, one link at a time (a few seconds each), and cached under the system temp folder (a
folder per user), keyed by a hash of the hulls' vertices and planes and of this module's code; a cached array that is
not a finite (n, 3) array of points on the hulls is built again. The cache is never part of the task.
"""

from __future__ import annotations

import functools
import hashlib
import os
import tempfile

import numpy as np

SPACING = 0.0005  # m: sample step on each hull face
INSIDE = 2e-5  # m: a point this far inside another hull of the same link is interior
VERTEX_TOL = 1e-6  # m: check(): the USD's and the URDF's hull vertices agree to this
_VERSION = "yamgeom-2"


@functools.lru_cache(maxsize=None)
def hulls(link: str) -> tuple:
    """((vertices, triangles, planes), ...) of `link`'s collision hulls in its frame, from yam_arena.analysis
    (triangles: the convex hull of each hull's vertices, which link_hulls does not keep). Reads only the URDF and its
    hull files (no pxr), so a demo can plan before the simulator app starts; warm() runs check() once the app is up."""
    from scipy.spatial import ConvexHull
    from yam_arena.analysis import link_hulls

    found = link_hulls().get(link)
    if not found:
        raise ValueError(f"no collision hulls for {link!r} in the environment's robot model")
    out = []
    for h in found:
        v = np.asarray(h.vertices, float)
        out.append((v, v[ConvexHull(v).simplices], np.asarray(h.planes, float)))
    return tuple(out)


@functools.lru_cache(maxsize=1)
def check() -> dict:
    """The robot USD the simulator loads (yam_arena.YAM_USD) has, under each link, as many convexHull collision meshes
    as link_hulls() has hulls, with the same vertices (in the link's frame, to 1 micrometre). Returns {link: count};
    raises RuntimeError on any difference."""
    import yam_arena
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from yam_arena.analysis import link_hulls

    stage = Usd.Stage.Open(str(yam_arena.require_usd()), Usd.Stage.LoadAll)
    cache = UsdGeom.XformCache()
    usd: dict = {}
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or not prim.IsA(UsdGeom.Mesh):
            continue
        if UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() != "convexHull":
            raise RuntimeError(f"{prim.GetPath()}: collider is not a convex hull")
        link = prim.GetParent().GetParent()  # .../<link>/<link>_NNN/<link>_NNN
        rel = cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(link).GetInverse()
        pts = np.array([rel.Transform(Gf.Vec3d(*p)) for p in UsdGeom.Mesh(prim).GetPointsAttr().Get()], float)
        usd.setdefault(link.GetName(), []).append(pts)
    ref = link_hulls()
    if set(usd) != set(ref):
        raise RuntimeError(f"links with colliders differ: robot USD {sorted(usd)}, URDF {sorted(ref)}")
    for link in sorted(ref):
        a, b = [np.asarray(h.vertices, float) for h in ref[link]], usd[link]
        if len(a) != len(b):
            raise RuntimeError(f"{link}: {len(b)} hull colliders in the robot USD, {len(a)} in the URDF")
        free = list(range(len(b)))
        for p in a:
            near = sorted(free, key=lambda j: float(np.linalg.norm(b[j].mean(axis=0) - p.mean(axis=0))))
            m = next((j for j in near if _same(p, b[j])), None)
            if m is None:
                raise RuntimeError(f"{link}: a URDF collision hull has no match in the robot USD")
            free.remove(m)
    return {link: len(usd[link]) for link in sorted(usd)}


def _same(p: np.ndarray, q: np.ndarray) -> bool:
    """Two vertex sets are one hull's: every point of each within VERTEX_TOL of a point of the other."""
    d = np.linalg.norm(p[:, None, :] - q[None, :, :], axis=2)
    return bool(d.min(axis=1).max() <= VERTEX_TOL and d.min(axis=0).max() <= VERTEX_TOL)


def _rows(tri: np.ndarray) -> np.ndarray:
    """Points on each triangle (f x 3 x 3) in rows parallel to its longest edge: rows at most SPACING apart from that
    edge to the opposite corner, points at most SPACING apart along each row, both ends included. Any point of the
    face then lies within SPACING / sqrt(2) of one (the count follows the area, so slivers stay cheap)."""
    e = np.linalg.norm(tri - np.roll(tri, -1, axis=1), axis=2)  # edge k runs from corner k to corner k + 1
    k = e.argmax(axis=1)
    idx = np.arange(len(tri))
    a, b, c = tri[idx, k], tri[idx, (k + 1) % 3], tri[idx, (k + 2) % 3]
    L = e[idx, k]
    h = np.linalg.norm(np.cross(b - a, c - a), axis=1) / np.maximum(L, 1e-12)  # the corner's height over that edge
    m = np.ceil(h / SPACING).astype(int)  # rows 0..m
    face = np.repeat(idx, m + 1)
    f = np.concatenate([np.arange(r + 1) / max(r, 1) for r in m])  # 0 on the edge, 1 at the corner
    n = np.ceil(L[face] * (1 - f) / SPACING).astype(int)  # steps along each row
    row = np.repeat(np.arange(len(f)), n + 1)
    s = np.concatenate([np.arange(q + 1) / max(q, 1) for q in n])
    fi, fr = face[row], f[row][:, None]
    p0 = a[fi] + fr * (c[fi] - a[fi])
    p1 = b[fi] + fr * (c[fi] - b[fi])
    return p0 + s[:, None] * (p1 - p0)


def _build(link: str) -> np.ndarray:
    hs = hulls(link)
    kept = []
    for i, (_, tri, _) in enumerate(hs):
        p = _rows(tri)
        for j, (v, _, planes) in enumerate(hs):
            if j == i or not len(p):
                continue
            near = np.all((p >= v.min(axis=0) - INSIDE) & (p <= v.max(axis=0) + INSIDE), axis=1)
            if near.any():
                inside = (p[near] @ planes[:, :3].T - planes[:, 3]).max(axis=1) <= -INSIDE
                drop = np.zeros(len(p), bool)
                drop[np.nonzero(near)[0][inside]] = True
                p = p[~drop]
        kept.append(p)
    return np.unique(np.round(np.vstack(kept), 7), axis=0)


def _key(link: str) -> str:
    h = hashlib.sha256(f"{_VERSION} {SPACING} {INSIDE} {link}".encode())
    with open(__file__, "rb") as f:
        h.update(f.read())  # this module's code: an edited copy never loads points another copy built
    for v, _, planes in hulls(link):
        h.update(np.round(v, 9).tobytes())
        h.update(np.round(planes, 9).tobytes())
    return h.hexdigest()[:24]


def _valid(link: str, p) -> bool:
    """A cached array is a finite (n, 3) float array of at least 1,000 points, a spread sample of which lies on the
    link's hulls (on a face of one hull, to 1 micrometre)."""
    if not (isinstance(p, np.ndarray) and p.ndim == 2 and p.shape[1] == 3 and len(p) >= 1000
            and np.issubdtype(p.dtype, np.floating) and np.isfinite(p).all()):
        return False
    q = p[:: max(1, len(p) // 200)]
    on = np.zeros(len(q), bool)
    for _, _, planes in hulls(link):
        s = (q @ planes[:, :3].T - planes[:, 3]).max(axis=1)
        on |= np.abs(s) <= 1e-6
    return bool(on.all())


@functools.lru_cache(maxsize=None)
def points(link: str) -> np.ndarray:
    """Surface points (m, the link's frame) of the link's collision hulls (see the module doc)."""
    user = os.getuid() if hasattr(os, "getuid") else "user"
    path = os.path.join(tempfile.gettempdir(), f"yamgeom_cache_{user}", f"{link}_{_key(link)}.npy")
    try:
        out = np.load(path)
        if not _valid(link, out):
            raise ValueError(path)
    except (OSError, ValueError):
        out = _build(link)
        tmp = f"{path}.{os.getpid()}.tmp.npy"
        try:
            os.makedirs(os.path.dirname(path), mode=0o700, exist_ok=True)
            np.save(tmp, out)
            os.replace(tmp, path)
        except OSError:
            if os.path.exists(tmp):
                os.remove(tmp)  # no cache: built again next time
    out.setflags(write=False)
    return out


@functools.lru_cache(maxsize=None)
def tcp_points(tip: str) -> np.ndarray:
    """A fingertip's collision-hull points with the gripper closed, in the tool point's frame."""
    import xml.etree.ElementTree as ET

    import yam_arena
    from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET
    from yam_arena.urdf import rpy_matrix

    joints = [j for j in ET.parse(yam_arena.YAM_URDF).getroot().iter("joint") if j.find("child").get("link") == tip]
    if len(joints) != 1 or joints[0].find("parent").get("link") != "gripper":
        raise ValueError(f"{tip} does not hang from the gripper by one joint")
    o = joints[0].find("origin")
    R = rpy_matrix(np.array([float(v) for v in o.get("rpy", "0 0 0").split()]))
    t = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
    axis = np.array([float(v) for v in joints[0].find("axis").get("xyz").split()])
    out = points(tip) @ R.T + t + R @ axis * FINGER_CLOSED - np.asarray(TCP_OFFSET, float)
    out.setflags(write=False)
    return out


@functools.lru_cache(maxsize=None)
def cloud(name: str) -> tuple:
    """(points, centroid, radius about it) of a link's hull points, or of "<tip>_tcp" (tcp_points)."""
    p = tcp_points(name[:-4]) if name.endswith("_tcp") else points(name)
    c = p.mean(axis=0)
    return p, c, float(np.linalg.norm(p - c, axis=1).max())


CELL = 0.01  # m: near()'s cells


@functools.lru_cache(maxsize=None)
def _cells(name: str) -> tuple:
    """cloud(name)'s points grouped by CELL cube: (points in cell order, cell starts, cell ends, cell centres, radii)."""
    p = cloud(name)[0]
    k = np.floor(p / CELL).astype(np.int64)
    order = np.lexsort((k[:, 2], k[:, 1], k[:, 0]))
    p, k = p[order], k[order]
    starts = np.r_[0, np.nonzero(np.any(np.diff(k, axis=0) != 0, axis=1))[0] + 1]
    ends = np.r_[starts[1:], len(p)]
    centres = np.add.reduceat(p, starts, axis=0) / (ends - starts)[:, None]
    radii = np.maximum.reduceat(np.linalg.norm(p - np.repeat(centres, ends - starts, axis=0), axis=1), starts)
    return p, starts, ends, centres, radii


def warm(names) -> None:
    """Check the robot USD against the URDF's hulls (check(), which imports pxr: call it only once the simulator app is
    up) and build near()'s cells for `names` now, not inside a trial's step."""
    check()
    for name in names:
        _cells(name)


def near(name: str, place, gap, cutoff: float) -> np.ndarray:
    """The points of cloud(name) moved by `place` (points -> points, a rigid motion) that can lie within `cutoff` of
    an object, for a `gap` (placed points -> m) that grows no faster than the distance (a distance to a convex shape,
    or the least of several): a cell is left out only when gap(its centre) minus its radius is over `cutoff`, so the
    least gap over the returned points equals the least over all of them whenever that is `cutoff` or less. Empty if
    no cell can be that close."""
    p, starts, ends, centres, radii = _cells(name)
    keep = np.nonzero(gap(place(centres)) - radii <= cutoff)[0]
    if not len(keep):
        return np.zeros((0, 3))
    return place(np.concatenate([p[starts[i]:ends[i]] for i in keep]))


def box(pts, lo=(None, None, None), hi=(None, None, None)) -> tuple:
    """(center, size) of the axis-aligned box around the points within lo..hi on each axis (None: unbounded)."""
    p = np.asarray(pts, float)
    keep = np.ones(len(p), bool)
    for k in range(3):
        if lo[k] is not None:
            keep &= p[:, k] >= lo[k]
        if hi[k] is not None:
            keep &= p[:, k] <= hi[k]
    if not keep.any():
        raise ValueError("no points in the box's range")
    a, b = p[keep].min(axis=0), p[keep].max(axis=0)
    return tuple(float(x) for x in (a + b) / 2), tuple(float(x) for x in b - a)


# -- test aids: where the collision hulls stand proud of the visual mesh (a touch only the hulls register) -------------
@functools.lru_cache(maxsize=None)
def visual_vertices(link: str) -> np.ndarray:
    """The link's visual mesh vertices (m, the link's frame), from the environment's URDF (for tests)."""
    import xml.etree.ElementTree as ET

    import trimesh
    import yam_arena
    from yam_arena.urdf import rpy_matrix

    el = [e for e in ET.parse(yam_arena.YAM_URDF).getroot().iter("link") if e.get("name") == link]
    out = []
    for vis in el[0].findall("visual") if el else []:
        o = vis.find("origin")
        R = rpy_matrix(np.array([float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]))
        t = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
        mesh = vis.find("geometry/mesh")
        m = trimesh.load(os.path.join(os.path.dirname(str(yam_arena.YAM_URDF)), mesh.get("filename")), force="mesh")
        out.append(np.asarray(m.vertices, float) @ R.T + t)
    if not out:
        raise ValueError(f"no visual mesh for {link!r}")
    return np.vstack(out)


def proud_points(link: str, width: float = 0.003, count: int = 12) -> list:
    """[(point, normal, depth), ...] for tests: up to `count` points on the link's collision hulls (link frame, at
    least 5 mm apart, deepest first) where the hulls bridge a pocket of the visual mesh, each with its hull face's
    outward normal and the pocket's depth under it (m: the least within `width` of the point). An object resting on
    the hulls there touches the simulated link while staying clear of its visual mesh under that face."""
    import trimesh
    from scipy.spatial import cKDTree

    p = points(link)
    pick = p[:: max(1, len(p) // 40000)]
    vis = trimesh.Trimesh(visual_vertices(link), _visual_faces(link))
    dense, _ = trimesh.sample.sample_surface_even(vis, 400000, seed=0)
    depth = cKDTree(np.vstack([dense, vis.vertices])).query(pick)[0]
    tree = cKDTree(pick)
    scored = sorted(((float(depth[tree.query_ball_point(pick[i], width)].min()), i) for i in np.argsort(-depth)[:2000]),
                    reverse=True)
    out = []
    for d, i in scored:
        q = pick[i]
        if len(out) >= count:
            break
        if any(np.linalg.norm(q - o[0]) < 0.005 for o in out):
            continue
        for _, _, planes in hulls(link):  # the face it lies on: its hull's plane through it
            s = q @ planes[:, :3].T - planes[:, 3]
            if s.max() <= 1e-6:
                n = planes[int(np.argmax(s)), :3]
                out.append((q, n / np.linalg.norm(n), d))
                break
    return out


@functools.lru_cache(maxsize=None)
def _visual_faces(link: str) -> np.ndarray:
    import xml.etree.ElementTree as ET

    import trimesh
    import yam_arena

    el = [e for e in ET.parse(yam_arena.YAM_URDF).getroot().iter("link") if e.get("name") == link][0]
    faces, base = [], 0
    for vis in el.findall("visual"):
        m = trimesh.load(os.path.join(os.path.dirname(str(yam_arena.YAM_URDF)), vis.find("geometry/mesh").get("filename")),
                         force="mesh")
        faces.append(np.asarray(m.faces) + base)
        base += len(m.vertices)
    return np.vstack(faces)


def turn_to(n, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """The rotation matrix that turns direction n onto `up` (for tests)."""
    a, b = np.asarray(n, float) / np.linalg.norm(n), np.asarray(up, float) / np.linalg.norm(up)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-12:
        if c > 0:
            return np.eye(3)
        w = np.cross(a, [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0])
        w /= np.linalg.norm(w)
        return 2.0 * np.outer(w, w) - np.eye(3)  # a half turn about w
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1.0 + c)
