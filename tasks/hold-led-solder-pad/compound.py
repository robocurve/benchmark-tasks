"""compound: a task-file box that collides as several smaller boxes, all one rigid body.

yam_arena's task files spawn solid cuboids. :func:`compound` registers a box (a plain ``Box`` or a ``bonds.Sliding``)
whose spawned cuboid keeps its size, so its bounds and the centre the scorer reads stay the cuboid's, but loses its
collider. In its place the body gets the box colliders listed as parts, each given by its centre and size in the box's
own frame (before ``yaw``). They are shapes of the same rigid body, so it moves as one piece, and its mass is spread
over the parts (PhysX works out the centre of mass and the inertia from them). A soldering iron is a thick handle, a
thin barrel and a thinner tip. An LED is its body, and the legs that pass through the board collide with nothing.

The parts take the friction every yam_arena task object has (1.2 static, 1.0 dynamic). Everything that needs Isaac Sim
is imported when the box spawns (``bonds.on_spawn``). Pure Python at import time. Import it before ``looks``, so the
parts are hidden with the cuboid.
"""

from __future__ import annotations

from bonds import on_spawn
from yam_arena.task_file import Box

__all__ = ["compound", "parts_of"]

Part = tuple[str, tuple[float, float, float], tuple[float, float, float]]  # name, centre, size (box frame, m)
_PARTS: dict[str, list[Part]] = {}
GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)


def compound(spec: Box, parts: list[Part]) -> Box:
    """Register ``spec`` to collide as ``parts`` (each inside the box), and return it (for ``OBJECTS``)."""
    if spec.mass is None:
        raise ValueError(f"{spec.name}: a compound box must have a mass")
    for name, centre, size in parts:
        for i in range(3):
            if abs(centre[i]) + size[i] / 2 > spec.size[i] / 2 + 1e-9:
                raise ValueError(f"{spec.name}.{name} sticks out of the box")
    _PARTS[spec.name] = list(parts)
    return spec


def parts_of(name: str) -> list[Part]:
    """The parts registered for the box ``name``."""
    return list(_PARTS[name])


def _build(prim, parts: list[Part]) -> None:
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    for p in Usd.PrimRange(prim):  # the solid cuboid stays for the bounds, without its collider
        if p.HasAPI(UsdPhysics.CollisionAPI):
            for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
                if api in p.GetAppliedSchemas():
                    p.RemoveAppliedSchema(api)
    material = UsdShade.Material.Define(stage, prim.GetPath().AppendChild("parts_material"))
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(GRIPPY[0])
    api.CreateDynamicFrictionAttr(GRIPPY[1])
    api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("average")
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("parts"))
    for name, centre, size in parts:
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"parts/{name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(0.004)
        contact.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material, UsdShade.Tokens.strongerThanDescendants,
                                                               "physics")


@on_spawn
def _spawned(name: str, prim) -> None:
    if name in _PARTS:
        _build(prim, _PARTS[name])
