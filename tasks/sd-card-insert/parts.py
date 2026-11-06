"""parts: one rigid body made of several box colliders, for a card with a cut corner and a reader with a slot.

yam_arena's task files spawn solid primitives. :class:`Compound` is a ``Box`` whose spawned cuboid keeps its size (so
the bounds and the centre the scorer reads are the object's outside) but loses its collider. In its place the body
gets the box colliders listed in ``parts``, each given by its centre and size in the box's own frame and turned
``yaw`` radians about the box's z axis. They are shapes of the same body, so the object moves as one piece, and a
fixed ``Compound`` (no mass) is one static collider of several shapes. Each part takes the friction every yam_arena
task object has (1.2 static, 1.0 dynamic) and contacts that start ``contact`` metres out and rest at zero
separation.

Adapted from the envelope module of the board-into-antistatic-bag task (any set of parts, turned parts). Everything
that needs Isaac Sim is imported when the box spawns (``bonds.on_spawn``). Pure Python at import time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from bonds import on_spawn
from yam_arena.task_file import Box

__all__ = ["Compound", "Part"]

GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)


@dataclass(frozen=True)
class Part:
    """One box collider: ``centre`` and ``size`` in its body's frame (m), tilted ``pitch`` rad about that frame's y,
    then turned ``yaw`` rad about its z."""

    name: str
    centre: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0
    pitch: float = 0.0

    def to_body(self, p):
        """A point of the part's own frame (centred, unturned) in its body's frame."""
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        x, y, z = cp * p[0] + sp * p[2], p[1], -sp * p[0] + cp * p[2]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.centre[0] + c * x - s * y, self.centre[1] + s * x + c * y, self.centre[2] + z)

    def to_part(self, q):
        """A point of the body's frame in the part's own frame."""
        d = [q[i] - self.centre[i] for i in range(3)]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y, z = c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        return (cp * x - sp * z, y, sp * x + cp * z)

    def contains(self, q) -> bool:
        local = self.to_part(q)
        return all(abs(local[i]) < self.size[i] / 2 for i in range(3))

    def corners(self):
        """The part's eight corners in its body's frame."""
        hx, hy, hz = (v / 2 for v in self.size)
        return [self.to_body((dx, dy, dz)) for dx in (-hx, hx) for dy in (-hy, hy) for dz in (-hz, hz)]


@dataclass(frozen=True)
class Compound(Box):
    """A box whose collider is its ``parts``, all inside its ``size``."""

    parts: tuple[Part, ...] = ()
    contact: float = 0.005

    def _check(self) -> None:
        super()._check()
        if not self.parts:
            raise ValueError(f"{self.name}: a compound needs parts")
        if len({p.name for p in self.parts}) != len(self.parts):
            raise ValueError(f"{self.name}: part names must be unique")
        half = [v / 2 + 1e-9 for v in self.size]
        for p in self.parts:
            if any(v <= 0 for v in p.size):
                raise ValueError(f"{self.name}.{p.name}: size must be positive")
            for corner in p.corners():
                if any(abs(corner[i]) > half[i] for i in range(3)):
                    raise ValueError(f"{self.name}.{p.name} sticks out of the box")
        if not self.contact > 0:
            raise ValueError(f"{self.name}: contact must be positive")


_COMPOUNDS: dict[str, Compound] = {}


def _material(stage, path):
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(GRIPPY[0])
    api.CreateDynamicFrictionAttr(GRIPPY[1])
    api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("average")
    return material


def _build(prim, spec: Compound) -> None:
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    for p in [q for q in Usd.PrimRange(prim) if q.HasAPI(UsdPhysics.CollisionAPI)]:
        for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
            if api in p.GetAppliedSchemas():
                p.RemoveAppliedSchema(api)
    material = _material(stage, prim.GetPath().AppendChild("parts_material"))
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("parts"))
    for part in spec.parts:
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"parts/{part.name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*part.centre))
        if part.yaw:
            cube.AddRotateZOp().Set(math.degrees(part.yaw))
        if part.pitch:
            cube.AddRotateYOp().Set(math.degrees(part.pitch))
        cube.AddScaleOp().Set(Gf.Vec3f(*part.size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(float(spec.contact))
        contact.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
            material, UsdShade.Tokens.strongerThanDescendants, "physics")


@on_spawn
def _spawned(name: str, prim) -> None:
    if name in _COMPOUNDS:
        _build(prim, _COMPOUNDS[name])


def compound(spec: Compound) -> Compound:
    """Register ``spec`` so its parts are built when it spawns, and return it (for ``OBJECTS``)."""
    _COMPOUNDS[spec.name] = spec
    return spec
