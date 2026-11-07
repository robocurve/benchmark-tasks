"""plate: a flat sheet with cut-outs as one rigid body of box colliders, flat parts of any convex outline, and a
friction override for a task-file box.

yam_arena's task files spawn solid primitives. Two boxes here keep their spawned cuboid (so the bounds and the centre
the scorer reads are the object's outside) but lose its collider, which is replaced when the box spawns:

- :class:`Plate`: a ``Box`` whose collider is a set of box pieces, all shapes of the same rigid body. :func:`pieces`
  builds them for a sheet with convex holes: a ring of thin boxes along every edge of each hole, so the hole's walls
  are exact, and boxes merged from a grid over the rest of the sheet.
- :func:`collider`: any other box (a hinged lid, a fixed frame) gets box pieces as its collider the same way. A piece
  can also be pitched (turned about the box's y axis), for a lid that stands open.
- :class:`Prism`: a ``Box`` whose collider is one convex prism, its outline (a convex polygon in the box's own x, y)
  extruded through the box's height. A triangle, a disc as a many-sided polygon.

:func:`surface` gives a task-file box another friction and the way it combines with what touches it.

Everything that needs Isaac Sim is imported when the box spawns (``bonds.on_spawn``). Pure Python and NumPy at import
time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from bonds import on_spawn
from yam_arena.task_file import Box

__all__ = ["Plate", "Prism", "collider", "pieces", "plate", "polygon_area", "offset_polygon", "prism", "regular_polygon",
           "surface"]

Piece = tuple  # (centre, size, yaw) or (centre, size, yaw, pitch), in the box's frame, angles in radians


@dataclass(frozen=True)
class Plate(Box):
    """A box whose collider is ``parts``: (centre, size, yaw) boxes in its own frame, each inside the box."""

    parts: tuple = ()

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a plate must have a mass")
        if not self.parts:
            raise ValueError(f"{self.name}: a plate needs collider pieces")


@dataclass(frozen=True)
class Prism(Box):
    """A box whose collider is the convex polygon ``outline`` ((x, y) in the box's frame, counter-clockwise, inside
    the box) extruded through the box's full height."""

    outline: tuple = ()

    def _check(self) -> None:
        super()._check()
        if len(self.outline) < 3:
            raise ValueError(f"{self.name}: the outline needs at least three points")
        hx, hy = self.size[0] / 2, self.size[1] / 2
        if any(abs(x) > hx + 1e-9 or abs(y) > hy + 1e-9 for x, y in self.outline):
            raise ValueError(f"{self.name}: the outline must stay inside the box")
        if polygon_area(self.outline) <= 0:
            raise ValueError(f"{self.name}: the outline must be counter-clockwise")


# -- polygons --------------------------------------------------------------------------------------------------------


def polygon_area(pts) -> float:
    """Signed area (positive counter-clockwise)."""
    p = np.asarray(pts, float)
    x, y = p[:, 0], p[:, 1]
    return float(0.5 * (np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y)))


def regular_polygon(n: int, apothem: float, centre=(0.0, 0.0), turn: float = 0.0) -> list[tuple[float, float]]:
    """A regular ``n``-gon, counter-clockwise, whose sides are ``apothem`` from ``centre``."""
    r = apothem / math.cos(math.pi / n)
    return [(centre[0] + r * math.cos(turn + 2 * math.pi * (k + 0.5) / n),
             centre[1] + r * math.sin(turn + 2 * math.pi * (k + 0.5) / n)) for k in range(n)]


def _edges(pts):
    p = np.asarray(pts, float)
    a, b = p, np.roll(p, -1, axis=0)
    d = b - a
    length = np.linalg.norm(d, axis=1)
    t = d / length[:, None]
    n = np.stack([t[:, 1], -t[:, 0]], axis=1)  # outward for a counter-clockwise polygon
    return a, b, t, n, length


def offset_polygon(pts, d: float) -> list[tuple[float, float]]:
    """A convex counter-clockwise polygon with every side moved ``d`` outward."""
    a, _, t, n, _ = _edges(pts)
    out = []
    k = len(a)
    for i in range(k):  # the new corner i is where the moved sides i-1 and i meet
        j = (i - 1) % k
        p1, d1 = a[j] + d * n[j], t[j]
        p2, d2 = a[i] + d * n[i], t[i]
        m = np.array([d1, -d2]).T
        s = np.linalg.solve(m, p2 - p1)
        out.append(tuple(p1 + s[0] * d1))
    return out


def outside(pts, p) -> float:
    """How far ``p`` is outside the convex polygon, as the largest distance past one of its sides (negative inside).
    It never exceeds the true distance, and changes by at most the distance ``p`` moves."""
    a, _, _, n, _ = _edges(pts)
    return float(np.max(np.einsum("ij,ij->i", np.asarray(p, float)[None, :] - a, n)))


def pieces(size, holes, ring: float = 0.006, cell: float = 0.0025) -> list[Piece]:
    """Box pieces for a sheet of ``size`` (x, y, thickness) centred on its own origin, with ``holes`` (convex
    counter-clockwise polygons in the sheet's x, y) cut through it.

    Each side of each hole gets a box ``ring`` wide just outside it, running ``ring`` past both corners (past a corner
    of a convex hole, a side's line stays outside the hole). The rest of the sheet is a grid of ``cell`` squares, kept
    where a square lies clear of every hole by half a ring (so it never reaches into a hole), merged into rows and the
    rows into rectangles. Every piece is the sheet's full thickness, inside its outline."""
    sx, sy, sz = (float(v) for v in size)
    out: list[Piece] = []
    for hole in holes:
        a, b, t, n, length = _edges(hole)
        for i in range(len(a)):
            mid = (a[i] + b[i]) / 2 + n[i] * ring / 2
            yaw = math.atan2(t[i][1], t[i][0])
            out.append(((float(mid[0]), float(mid[1]), 0.0), (float(length[i] + 2 * ring), ring, sz), yaw))
    nx, ny = int(round(sx / cell)), int(round(sy / cell))
    cx, cy = sx / nx, sy / ny
    clear = ring / 2 + math.hypot(cx, cy) / 2
    keep = np.ones((nx, ny), bool)
    for i in range(nx):
        for j in range(ny):
            p = (-sx / 2 + (i + 0.5) * cx, -sy / 2 + (j + 0.5) * cy)
            keep[i, j] = all(outside(h, p) >= clear for h in holes)
    runs: dict[tuple[int, int], list[int]] = {}  # (j0, j1) along y -> the rows i that have that run
    for i in range(nx):
        j = 0
        while j < ny:
            if not keep[i, j]:
                j += 1
                continue
            j0 = j
            while j < ny and keep[i, j]:
                j += 1
            runs.setdefault((j0, j), []).append(i)
    for (j0, j1), rows in sorted(runs.items()):
        start = prev = rows[0]
        for i in rows[1:] + [None]:
            if i is not None and i == prev + 1:
                prev = i
                continue
            x0, x1 = -sx / 2 + start * cx, -sx / 2 + (prev + 1) * cx
            y0, y1 = -sy / 2 + j0 * cy, -sy / 2 + j1 * cy
            out.append((((x0 + x1) / 2, (y0 + y1) / 2, 0.0), (x1 - x0, y1 - y0, sz), 0.0))
            if i is not None:
                start = prev = i
    return out


def piece_corners(piece: Piece) -> np.ndarray:
    """The four corners (x, y) of a piece's footprint."""
    (cx, cy, _), (lx, ly, _), yaw = piece
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(cx + c * u - s * v, cy + s * u + c * v)
                     for u, v in ((-lx / 2, -ly / 2), (lx / 2, -ly / 2), (lx / 2, ly / 2), (-lx / 2, ly / 2))])


# -- spawning --------------------------------------------------------------------------------------------------------

_PLATES: dict[str, tuple] = {}  # each box's collider pieces
_PRISMS: dict[str, Prism] = {}
_SURFACES: dict[str, tuple[float, str]] = {}


def surface(name: str, friction: float, combine: str = "average") -> None:
    """Give the task-file box ``name`` this static and dynamic friction, combined with the other body's by
    ``combine`` ("average", "min", "multiply" or "max": PhysX uses the later of the two bodies' modes in that
    list)."""
    if not friction >= 0:
        raise ValueError("friction must not be negative")
    if combine not in ("average", "min", "multiply", "max"):
        raise ValueError(f"unknown combine mode {combine!r}")
    _SURFACES[name] = (float(friction), combine)


def _material(stage, path, friction: float, combine: str):
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction)
    api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr(combine)
    return material


def _bind(prim, material) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")


def _colliders(prim):
    from pxr import Usd, UsdPhysics

    return [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]


def _strip(prim) -> None:
    """The solid cuboid stays for the bounds (and is hidden by looks.py), without its collider."""
    for p in _colliders(prim):
        for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
            if api in p.GetAppliedSchemas():
                p.RemoveAppliedSchema(api)


def _collide(prim) -> None:
    """A collider only: invisible (the box's look draws it), still colliding."""
    from pxr import PhysxSchema, UsdGeom, UsdPhysics

    UsdGeom.Imageable(prim).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(prim)
    contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    contact.CreateContactOffsetAttr(0.003)
    contact.CreateRestOffsetAttr(0.0)


def _build_plate(prim, parts) -> None:
    from pxr import Gf, UsdGeom

    stage = prim.GetStage()
    _strip(prim)
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("pieces"))
    for k, piece in enumerate(parts):
        (x, y, z), (lx, ly, lz), yaw = piece[:3]
        pitch = piece[3] if len(piece) > 3 else 0.0
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"pieces/p{k:03d}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        cube.AddRotateZOp().Set(math.degrees(yaw))
        if pitch:
            cube.AddRotateYOp().Set(math.degrees(pitch))
        cube.AddScaleOp().Set(Gf.Vec3f(lx, ly, lz))
        _collide(cube.GetPrim())


def _build_prism(prim, spec: Prism) -> None:
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

    stage = prim.GetStage()
    _strip(prim)
    h = spec.size[2] / 2
    pts = [Gf.Vec3f(x, y, -h) for x, y in spec.outline] + [Gf.Vec3f(x, y, h) for x, y in spec.outline]
    k = len(spec.outline)
    faces = [list(range(k - 1, -1, -1)), list(range(k, 2 * k))]
    faces += [[i, (i + 1) % k, k + (i + 1) % k, k + i] for i in range(k)]
    mesh = UsdGeom.Mesh.Define(stage, prim.GetPath().AppendChild("hull"))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in faces]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    _collide(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")


@on_spawn
def _spawned(name: str, prim) -> None:
    if name in _PLATES:
        _build_plate(prim, _PLATES[name])
    if name in _PRISMS:
        _build_prism(prim, _PRISMS[name])
    if name in _SURFACES:
        friction, combine = _SURFACES[name]
        material = _material(prim.GetStage(), prim.GetPath().AppendChild("surface_material"), friction, combine)
        for p in _colliders(prim):
            _bind(p, material)


def plate(spec: Plate) -> Plate:
    """Register ``spec`` so its pieces are built when it spawns, and return it (for ``OBJECTS``)."""
    _PLATES[spec.name] = tuple(spec.parts)
    return spec


def collider(name: str, parts) -> None:
    """Give the task-file box ``name`` the collider ``parts``: (centre, size, yaw[, pitch]) boxes in its own frame,
    in place of its cuboid (which keeps its bounds)."""
    if not parts:
        raise ValueError(f"{name}: no collider pieces")
    _PLATES[name] = tuple(parts)


def prism(spec: Prism) -> Prism:
    """Register ``spec`` so its prism collider is built when it spawns, and return it (for ``OBJECTS``)."""
    _PRISMS[spec.name] = spec
    return spec
