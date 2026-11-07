"""shell: a task-file box whose collider is a set of smaller boxes, all one rigid body.

yam_arena's task files spawn solid primitives. :func:`shell` gives the box named ``name`` its own colliders: the
spawned cuboid keeps its size (so the bounds and the centre the scorer reads are the box's) but loses its collider,
and the body gets one thin box collider per part, in the box's own frame. The parts are shapes of the same rigid
body, so they move as one piece. A lever with a clamp tooth under it, a wire whose bare end is thinner than its
insulation.

The parts take the friction every yam_arena task object has (1.2 static, 1.0 dynamic), contacts start 5 mm out and
rest at zero, as for the other task objects. They are invisible: ``looks.py`` draws the box.

Adapted from the envelope module of the board-into-antistatic-bag task. Everything that needs Isaac Sim is imported
when the box spawns (``bonds.on_spawn``). Pure Python at import time.
"""

from __future__ import annotations

from bonds import on_spawn

__all__ = ["Part", "shell"]

Part = tuple[str, tuple[float, float, float], tuple[float, float, float]]  # (name, centre, size), box frame, metres
_SHELLS: dict[str, tuple[Part, ...]] = {}
GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)


def shell(name: str, parts) -> None:
    """Give the box ``name`` these colliders instead of its own: (name, centre, size) in the box's frame."""
    parts = tuple((str(n), tuple(map(float, c)), tuple(map(float, s))) for n, c, s in parts)
    if not parts:
        raise ValueError(f"{name}: a shell needs at least one part")
    for part, _, size in parts:
        if len(size) != 3 or min(size) <= 0:
            raise ValueError(f"{name}.{part}: sizes must be positive")
    _SHELLS[name] = parts


def parts(name: str) -> tuple[Part, ...]:
    return _SHELLS[name]


def _build(prim, parts) -> None:
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    for p in [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]:
        for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
            if api in p.GetAppliedSchemas():
                p.RemoveAppliedSchema(api)
    material = UsdShade.Material.Define(stage, prim.GetPath().AppendChild("shell_material"))
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
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()  # looks.py draws the box
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(0.005)
        contact.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
            material, UsdShade.Tokens.strongerThanDescendants, "physics")


@on_spawn
def _spawned(name: str, prim) -> None:
    if name in _SHELLS:
        _build(prim, _SHELLS[name])
