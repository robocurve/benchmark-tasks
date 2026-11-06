"""parts: a task-file box whose collider is a set of smaller boxes, one rigid body with a shape no cuboid has.

yam_arena's task files spawn solid primitives. :class:`Compound` is a ``Box`` whose spawned cuboid keeps its size (so
the bounds and the centre the scorer reads are the part's outside) but loses its collider. In its place, the body
gets the box colliders listed in ``parts``, each a (name, centre, size) in the box's own frame. They are shapes of the
same rigid body (or of the same static collider, for a fixed box), so the part moves as one piece: a chip with a
notch and two rows of pins, say. Every part must fit inside the box.

The parts take the friction every yam_arena task object has (1.2 static, 1.0 dynamic, averaged with the other
body's), a 2 mm contact offset and no rest offset, so parts with a few millimetres of play keep it.

Everything that needs Isaac Sim is imported when the box spawns (``bonds.on_spawn``). Pure Python at import time.
Adapted from the envelope module of the board-into-antistatic-bag task.
"""

from __future__ import annotations

from dataclasses import dataclass

from bonds import on_spawn
from yam_arena.task_file import Box

__all__ = ["Compound", "compound"]

Part = tuple[str, tuple[float, float, float], tuple[float, float, float]]


@dataclass(frozen=True)
class Compound(Box):
    """A box whose collider is ``parts``: (name, centre, size) boxes in its own frame, all inside ``size``."""

    parts: tuple[Part, ...] = ()

    def _check(self) -> None:
        super()._check()
        if not self.parts:
            raise ValueError(f"{self.name}: a compound needs at least one part")
        for name, centre, size in self.parts:
            for c, s, outer in zip(centre, size, self.size):
                if s <= 0 or abs(c) + s / 2 > outer / 2 + 1e-9:
                    raise ValueError(f"{self.name}.{name} must be a box inside {self.size}")


_COMPOUNDS: dict[str, Compound] = {}
GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)
CONTACT_OFFSET = 0.002  # m
REST_OFFSET = 0.0


def _material(stage, path, friction):
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    static, dynamic = friction
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("average")
    return material


def _build(prim, spec: Compound) -> None:
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    for p in [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]:
        # the solid cuboid stays for the bounds (and its look), without its collider
        for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
            if api in p.GetAppliedSchemas():
                p.RemoveAppliedSchema(api)
    material = _material(stage, prim.GetPath().AppendChild("parts_material"), GRIPPY)
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("parts"))
    for name, centre, size in spec.parts:
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"parts/{name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(CONTACT_OFFSET)
        contact.CreateRestOffsetAttr(REST_OFFSET)
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
