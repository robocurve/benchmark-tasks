"""colliders: a task-file box that collides as several smaller boxes, each with its own friction.

yam_arena's task files spawn solid primitives. :func:`shape` names a box and the boxes it is made of, as
:class:`Part`\\ s in the box's own frame. When the box spawns, its cuboid keeps its size (so the bounds and the centre
the scorer reads are the box's outside) but loses its collider, and the body gets one box collider per part. They
are shapes of the same rigid body, so the box still moves as one piece: a print standing on a smaller foot (a gap
under its edges), a scraper with a thin blade on its handle.

Each part takes the friction every yam_arena task object has (1.2 static, 1.0 dynamic, averaged with the other
body's), or its own, with the way it combines with what touches it (``"min"``: the lower of the two wins).

Everything that needs Isaac Sim is imported when the box spawns (``bonds.on_spawn``). Pure Python at import time.
"""

from __future__ import annotations

from dataclasses import dataclass

from bonds import on_spawn

__all__ = ["Part", "shape", "shapes"]

GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)
CONTACT_OFFSET = 0.002  # m: contacts start this far out (the parts are thin), and rest at zero separation


@dataclass(frozen=True)
class Part:
    """One box collider: ``centre`` and ``size`` in the box's own frame (metres), ``friction`` (static, dynamic) and
    how it combines with the other body's ("average", "min", "multiply" or "max")."""

    name: str
    centre: tuple[float, float, float]
    size: tuple[float, float, float]
    friction: tuple[float, float] = GRIPPY
    combine: str = "average"

    def __post_init__(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"part name {self.name!r} must be an identifier")
        if not all(s > 0 for s in self.size):
            raise ValueError(f"{self.name}: sizes must be positive")
        if self.combine not in ("average", "min", "multiply", "max"):
            raise ValueError(f"{self.name}: unknown combine mode {self.combine!r}")


_SHAPES: dict[str, tuple[Part, ...]] = {}


def shape(name: str, parts) -> tuple[Part, ...]:
    """Make the task-file box ``name`` collide as ``parts`` instead of its cuboid. Returns the parts."""
    parts = tuple(parts)
    if not parts:
        raise ValueError(f"{name}: give at least one part")
    _SHAPES[name] = parts
    return parts


def shapes() -> dict[str, tuple[Part, ...]]:
    """Every registered box's parts."""
    return dict(_SHAPES)


def _material(stage, path, friction, combine: str):
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(float(friction[0]))
    api.CreateDynamicFrictionAttr(float(friction[1]))
    api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr(combine)
    return material


def _build(prim, parts) -> None:
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    for p in [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]:
        for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
            if api in p.GetAppliedSchemas():
                p.RemoveAppliedSchema(api)
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("parts"))
    for part in parts:
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"parts/{part.name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*part.centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*part.size))
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(CONTACT_OFFSET)
        contact.CreateRestOffsetAttr(0.0)
        material = _material(stage, prim.GetPath().AppendPath(f"parts/{part.name}_material"), part.friction,
                             part.combine)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
            material, UsdShade.Tokens.strongerThanDescendants, "physics")


@on_spawn
def _spawned(name: str, prim) -> None:
    if name in _SHAPES:
        _build(prim, _SHAPES[name])
