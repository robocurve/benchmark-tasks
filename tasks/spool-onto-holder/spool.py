"""spool: a filament spool as one rigid body of boxes around its bore, and a horizontal rod with a stop on a fixed post.

yam_arena's task files spawn solid primitives, and its cylinders stand upright. :class:`Spool` is a ``Box`` with a
mass whose spawned cuboid keeps its size (so the bounds and the centre the scorer reads are the spool's outside) but
loses its collider. In its place the body gets a ring of box colliders around the bore, in ``sides`` segments: the
core (hub and wound filament) from the bore out to the filament's surface across the full width, and the two flanges
from there out to the rim, ``flange`` thick at each end. The bore is a regular polygon whose flats are ``bore`` from
the axis, open right through, and between the flanges a gap runs round the filament, where fingers can reach under a
flange. The spool's own axis is its box's z: it spawns lying flat.

:func:`rod` gives a fixed task-file box (the holder's post) extra colliders: cylinders along the world's x, which a
task-file ``Cylinder`` can't be (the rod and its stop collar), and a friction of its own for them.

Everything that needs Isaac Sim is imported when the box spawns (``bonds.on_spawn``). Pure Python at import time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from bonds import on_spawn
from yam_arena.task_file import Box

__all__ = ["Spool", "Rod", "spool", "rod"]

GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)


@dataclass(frozen=True)
class Spool(Box):
    """A spool lying flat: ``size`` is (diameter, diameter, width). The flats of its bore are ``bore`` from the axis,
    the wound filament's surface ``core`` from it, and each flange is ``flange`` thick."""

    bore: float = 0.026
    core: float = 0.075
    flange: float = 0.006
    sides: int = 24

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a spool must have a mass")
        d, d2, w = self.size
        if abs(d - d2) > 1e-9:
            raise ValueError(f"{self.name}: a spool's size is (diameter, diameter, width)")
        if not (0 < self.bore < self.core < d / 2 and 0 < 2 * self.flange < w and self.sides >= 8):
            raise ValueError(f"{self.name}: bore < core < rim, two flanges inside the width, 8 sides or more")

    @property
    def radius(self) -> float:
        return self.size[0] / 2

    @property
    def width(self) -> float:
        return self.size[2]

    def parts(self) -> list[tuple[str, tuple[float, float, float], float, tuple[float, float, float]]]:
        """Each collider as (name, centre, turn about the axis in radians, size) in the spool's own frame (centre of
        its outside, axis along z). A segment's size is (radial, tangential, axial): its tangential width covers the
        polygon's side at the segment's outer radius, so neighbours overlap toward the bore and leave no gaps."""
        w, t = self.width, self.flange
        half_turn = math.pi / self.sides
        rings = [("core", self.bore, self.core, 0.0, w)]
        for name, z in (("flange_top", w / 2 - t / 2), ("flange_bottom", -w / 2 + t / 2)):
            rings.append((name, self.core, self.radius, z, t))
        parts = []
        for name, inner, outer, z, length in rings:
            mid, tangential = (inner + outer) / 2, 2 * outer * math.tan(half_turn)
            for k in range(self.sides):
                turn = 2 * half_turn * k
                parts.append((f"{name}_{k:02d}", (mid * math.cos(turn), mid * math.sin(turn), z), turn,
                              (outer - inner, tangential, length)))
        return parts


@dataclass(frozen=True)
class Rod:
    """A cylinder along the world's x, fixed to the box it is registered on: its axis at (``y``, ``z``) runs from
    ``x0`` to ``x1``, with ``radius``. With ``taper`` > 0, its first ``taper`` metres from ``x0`` narrow to
    ``tip_radius`` at ``x0`` (a chamfered nose, a convex collider)."""

    name: str
    x0: float
    x1: float
    y: float
    z: float
    radius: float
    taper: float = 0.0
    tip_radius: float = 0.0


_SPOOLS: dict[str, Spool] = {}
_RODS: dict[str, list[tuple[tuple[float, float, float], Rod]]] = {}
_FRICTION: dict[str, tuple[float, str]] = {}


def _material(stage, path, friction, combine: str):
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    static, dynamic = friction if isinstance(friction, tuple) else (friction, friction)
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr(combine)
    return material


def _bind(prim, material) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")


def _collider(prim, material, offset: float) -> None:
    from pxr import PhysxSchema, UsdPhysics

    UsdPhysics.CollisionAPI.Apply(prim)
    contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    contact.CreateContactOffsetAttr(offset)
    contact.CreateRestOffsetAttr(0.0)
    _bind(prim, material)


def _build_spool(prim, spec: Spool) -> None:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = prim.GetStage()
    for p in Usd.PrimRange(prim):  # the solid cuboid stays for the bounds, without its collider
        if p.HasAPI(UsdPhysics.CollisionAPI):
            for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
                if api in p.GetAppliedSchemas():
                    p.RemoveAppliedSchema(api)
    material = _material(stage, prim.GetPath().AppendChild("spool_material"), GRIPPY, "average")
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("ring"))
    for name, centre, turn, size in spec.parts():
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"ring/{name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddRotateZOp().Set(math.degrees(turn))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
        _collider(cube.GetPrim(), material, 0.003)


def _build_rods(prim, name: str) -> None:
    from pxr import Gf, UsdGeom

    stage = prim.GetStage()
    friction, combine = _FRICTION.get(name, (GRIPPY, "average"))
    material = _material(stage, prim.GetPath().AppendChild("rod_material"), friction, combine)
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("rods"))
    for owner, r in _RODS[name]:
        start = r.x0 + r.taper
        cyl = UsdGeom.Cylinder.Define(stage, prim.GetPath().AppendPath(f"rods/{r.name}"))
        cyl.CreateAxisAttr("X")
        cyl.CreateRadiusAttr(r.radius)
        cyl.CreateHeightAttr(r.x1 - start)
        local = ((start + r.x1) / 2 - owner[0], r.y - owner[1], r.z - owner[2])
        cyl.AddTranslateOp().Set(Gf.Vec3d(*local))
        _collider(cyl.GetPrim(), material, 0.003)
        if r.taper > 0:
            _nose(stage, prim.GetPath().AppendPath(f"rods/{r.name}_nose"), r, owner, material)


def _nose(stage, path, r: Rod, owner, material) -> None:
    """The rod's chamfered nose: a truncated cone from ``tip_radius`` at ``x0`` to ``radius`` at ``x0 + taper``, as
    a convex mesh collider."""
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

    n = 32
    points = []
    for x, rad in ((r.x0, r.tip_radius), (r.x0 + r.taper, r.radius)):
        for k in range(n):
            a = 2 * math.pi * k / n
            points.append(Gf.Vec3f(x - owner[0], r.y - owner[1] + rad * math.cos(a), r.z - owner[2] + rad * math.sin(a)))
    counts, indices = [n, n], list(range(n))[::-1] + list(range(n, 2 * n))
    for k in range(n):
        j = (k + 1) % n
        counts.append(4)
        indices += [k, j, n + j, n + k]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    _collider(mesh.GetPrim(), material, 0.003)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")


@on_spawn
def _spawned(name: str, prim) -> None:
    if name in _SPOOLS:
        _build_spool(prim, _SPOOLS[name])
    if name in _RODS:
        _build_rods(prim, name)


def spool(spec: Spool) -> Spool:
    """Register ``spec`` so its ring of colliders is built when it spawns, and return it (for ``OBJECTS``)."""
    _SPOOLS[spec.name] = spec
    return spec


def rod(on: Box, part: Rod, friction: float | None = None, combine: str = "min") -> Rod:
    """Add ``part`` to the fixed box ``on`` (unturned) as a collider, with ``friction`` (static and dynamic) combined
    with the other body's by ``combine``, or the task objects' friction when None. Returns ``part``."""
    if on.free or abs(on.yaw) > 1e-12:
        raise ValueError(f"{on.name}: rods go on a fixed, unturned box")
    if not part.x1 > part.x0 + part.taper or not part.radius > 0:
        raise ValueError(f"{part.name}: x1 > x0 + taper and a positive radius")
    if part.taper < 0 or (part.taper > 0 and not 0 < part.tip_radius < part.radius):
        raise ValueError(f"{part.name}: a taper narrows to a smaller, positive tip radius")
    _RODS.setdefault(on.name, []).append((tuple(on.pos), part))
    if friction is not None:
        _FRICTION[on.name] = (float(friction), combine)
    return part
