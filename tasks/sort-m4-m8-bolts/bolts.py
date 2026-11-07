"""bolts: a hex bolt as one rigid body of two box colliders, its head and its shank.

yam_arena's task files spawn solid primitives. :class:`Bolt` is a ``Box`` with a mass whose spawned cuboid keeps its
size (so the bounds and the centre the scorer reads are the whole bolt's: head plus shank) but loses its collider. In
its place the body gets two box colliders of the same rigid body:

- the head, ``k`` long along the bolt's axis, ``s`` (the hex's width across flats) square across it;
- the shank, ``l`` long and ``d`` (the thread's diameter) square across it, on the head's axis.

The bolt's axis is the box's own x axis, the head at its -x end. Lying on the table, the head rests on a face and the
shank's far end touches down, so the bolt tilts a few degrees, as a real one does, and it cannot roll. The colliders
touch from 2 mm out (the task objects' 5 mm is wider than an M4 shank), and the body's spin is damped (PhysX has no
rolling friction) and solved with more iterations, so the light parts settle still. The friction is the one every
yam_arena task object has (1.2 static, 1.0 dynamic).

Everything that needs Isaac Sim is imported when the box spawns (``bonds.on_spawn``). Pure Python at import time.
"""

from __future__ import annotations

from dataclasses import dataclass

from bonds import on_spawn
from yam_arena.task_file import Box

__all__ = ["Bolt", "bolt"]

GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)
CONTACT_OFFSET = 0.002  # m
ANGULAR_DAMPING = 2.0  # 1/s
SOLVER_ITERATIONS = (16, 4)  # position, velocity


@dataclass(frozen=True)
class Bolt(Box):
    """A hex bolt lying along its box's x axis, head toward -x. ``size`` is its box: ``k + l`` long, the head's
    width across corners wide (y), and ``s`` high (z)."""

    d: float = 0.004  # m: the thread's diameter, the shank's square
    k: float = 0.0028  # m: the head's height (along the axis)
    s: float = 0.007  # m: the head's width across flats
    l: float = 0.020  # m: the shank's length  # noqa: E741

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a bolt must have a mass")
        if not (0 < self.d < self.s and self.k > 0 and self.l > 0):
            raise ValueError(f"{self.name}: the shank must be thinner than the head")
        L, w, h = self.size
        if abs(L - self.k - self.l) > 1e-6 or w < self.s or abs(h - self.s) > 1e-6:
            raise ValueError(f"{self.name}: size must be (k + l, at least s, s)")

    def parts(self) -> list[tuple[str, tuple[float, float, float], tuple[float, float, float]]]:
        """Each collider as (name, centre, size) in the bolt's frame (the centre of its box, the axis along x)."""
        L = self.k + self.l
        return [
            ("head", (-L / 2 + self.k / 2, 0.0, 0.0), (self.k, self.s, self.s)),
            ("shank", (L / 2 - self.l / 2, 0.0, 0.0), (self.l, self.d, self.d)),
        ]

    def grip_point(self, along: float = 0.0) -> tuple[float, float]:
        """The (x, y) on the table of the point ``along`` metres from the box's centre toward the tip, as placed."""
        import math

        return (self.pos[0] + along * math.cos(self.yaw), self.pos[1] + along * math.sin(self.yaw))


_BOLTS: dict[str, Bolt] = {}


def _material(stage, path):
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(GRIPPY[0])
    api.CreateDynamicFrictionAttr(GRIPPY[1])
    api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("average")
    return material


def _build(prim, spec: Bolt) -> None:
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    for p in [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]:
        for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
            if api in p.GetAppliedSchemas():
                p.RemoveAppliedSchema(api)
    material = _material(stage, prim.GetPath().AppendChild("bolt_material"))
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("colliders"))
    for name, centre, size in spec.parts():
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"colliders/{name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(CONTACT_OFFSET)
        contact.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material, UsdShade.Tokens.strongerThanDescendants,
                                                               "physics")
    body = next((p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)), prim)
    physx = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
    physx.CreateAngularDampingAttr(ANGULAR_DAMPING)
    physx.CreateSolverPositionIterationCountAttr(SOLVER_ITERATIONS[0])
    physx.CreateSolverVelocityIterationCountAttr(SOLVER_ITERATIONS[1])


@on_spawn
def _spawned(name: str, prim) -> None:
    if name in _BOLTS:
        _build(prim, _BOLTS[name])


def bolt(spec: Bolt) -> Bolt:
    """Register ``spec`` so its colliders are built when it spawns, and return it (for ``OBJECTS``)."""
    _BOLTS[spec.name] = spec
    return spec
