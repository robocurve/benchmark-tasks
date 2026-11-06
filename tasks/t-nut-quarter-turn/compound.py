"""compound: a task-file box that collides as several smaller boxes of one rigid body.

yam_arena's task files spawn solid primitives. :class:`Compound` is a ``Box`` with a mass whose spawned cuboid keeps
its size (so the bounds and the centre the scorer reads are the part's outside) but loses its collider. In its place,
the body gets the box colliders listed in ``parts``, in the box's own frame (centre of the box, before ``yaw``). They
are shapes of one rigid body, so the part moves as one piece: a T-nut with its stud, say. PhysX puts the centre of
mass where the parts' volume is.

The parts take the friction every yam_arena task object has (1.2 static, 1.0 dynamic, averaged with the other
body's), and contacts start ``contact`` metres out (yam_arena's 5 mm is too much for a part a few millimetres thick).

Everything that needs Isaac Sim is imported when the box spawns (``bonds.on_spawn``). Pure Python at import time.
"""

from __future__ import annotations

from dataclasses import dataclass

from bonds import on_spawn
from yam_arena.task_file import Box

__all__ = ["Compound", "compound"]

GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)
Part = tuple[str, tuple[float, float, float], tuple[float, float, float]]  # name, centre, size


@dataclass(frozen=True)
class Compound(Box):
    """A free box that collides as ``parts``: (name, centre, size) boxes in its own frame, each inside ``size``."""

    parts: tuple[Part, ...] = ()
    contact: float = 0.002

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a compound must have a mass")
        if not self.parts:
            raise ValueError(f"{self.name}: a compound needs at least one part")
        if not self.contact > 0:
            raise ValueError(f"{self.name}.contact must be positive")
        for name, centre, size in self.parts:
            if not name.isidentifier():
                raise ValueError(f"{self.name}: part name {name!r} is not an identifier")
            for c, s, outer in zip(centre, size, self.size):
                if not (s > 0 and abs(c) + s / 2 <= outer / 2 + 1e-9):
                    raise ValueError(f"{self.name}.{name} must lie inside the box")


_COMPOUNDS: dict[str, Compound] = {}


def _build(prim, spec: Compound) -> None:
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    for p in [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]:  # bounds stay, collider goes
        for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
            if api in p.GetAppliedSchemas():
                p.RemoveAppliedSchema(api)
    material = UsdShade.Material.Define(stage, prim.GetPath().AppendChild("compound_material"))
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(GRIPPY[0])
    api.CreateDynamicFrictionAttr(GRIPPY[1])
    api.CreateRestitutionAttr(0.0)
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("parts"))
    for name, centre, size in spec.parts:
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"parts/{name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(spec.contact)
        contact.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material, UsdShade.Tokens.strongerThanDescendants,
                                                               "physics")


@on_spawn
def _spawned(name: str, prim) -> None:
    if name in _COMPOUNDS:
        _build(prim, _COMPOUNDS[name])


def compound(spec: Compound) -> Compound:
    """Register ``spec`` so its parts are built when it spawns, and return it (for ``OBJECTS``)."""
    _COMPOUNDS[spec.name] = spec
    return spec
