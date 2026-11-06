"""envelope: a hollow bag as one rigid body of thin boxes, and a friction override for a task-file box.

yam_arena's task files spawn solid primitives. :class:`Envelope` is a ``Box`` with a mass whose spawned cuboid keeps
its size (so the bounds and the centre the scorer reads are the bag's outside) but loses its collider. In its place,
the body gets thin box colliders: a bottom wall, a top wall with a thumb notch at the mouth, two side seams and a
closed end. They are shapes of the same rigid body, so the bag moves as one piece with a cavity open at its mouth.
The mouth is the box's own -y face (the world's at ``yaw`` 0), the closed end its +y face.

The walls take the friction every yam_arena task object has (1.2 static, 1.0 dynamic, averaged with the other
body's). :func:`surface` gives a task-file box another friction and the way it combines with what touches it: a slick
mat, say, whose minimum wins over the average, so everything slides on it at the mat's own friction.

Everything that needs Isaac Sim is imported when the box spawns (``bonds.on_spawn``). Pure Python at import time.
"""

from __future__ import annotations

from dataclasses import dataclass

from bonds import on_spawn
from yam_arena.task_file import Box

__all__ = ["Envelope", "envelope", "surface"]


@dataclass(frozen=True)
class Envelope(Box):
    """A flat bag: ``size`` is its outside (width along x, length along y, thickness along z, before ``yaw``). Its
    top and bottom walls are ``wall`` thick, its side seams ``seam`` wide and its closed end ``end`` deep, so the
    cavity is ``size[0] - 2 seam`` wide, ``size[1] - end`` deep from the mouth and ``size[2] - 2 wall`` high. The top
    wall has a notch ``notch[0]`` wide and ``notch[1]`` deep in the middle of the mouth."""

    wall: float = 0.002
    seam: float = 0.002
    end: float = 0.004
    notch: tuple[float, float] = (0.03, 0.015)

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: an envelope must have a mass")
        w, length, h = self.size
        if not (0 < 2 * self.wall < h and 0 < 2 * self.seam < w and 0 < self.end < length):
            raise ValueError(f"{self.name}: the walls leave no cavity")
        if not (0 <= self.notch[0] < w - 2 * self.seam and 0 <= self.notch[1] < length - self.end):
            raise ValueError(f"{self.name}: the notch must fit in the top wall")

    @property
    def cavity(self) -> tuple[float, float, float]:
        """The cavity's width, depth (from the mouth) and height (m)."""
        w, length, h = self.size
        return (w - 2 * self.seam, length - self.end, h - 2 * self.wall)

    def parts(self) -> list[tuple[str, tuple[float, float, float], tuple[float, float, float]]]:
        """Each wall as (name, centre, size) in the bag's own frame (centre of its outside, mouth toward -y)."""
        w, length, h = self.size
        cw, cd, ch = self.cavity
        nw, nd = self.notch
        top_z, mouth = h / 2 - self.wall / 2, -length / 2
        side = (w - nw) / 2
        parts = [
            ("bottom", (0.0, 0.0, -top_z), (w, length, self.wall)),
            ("top", (0.0, (nd + 0.0) / 2, top_z), (w, length - nd, self.wall)),
            ("seam_left", (cw / 2 + self.seam / 2, 0.0, 0.0), (self.seam, length, ch)),
            ("seam_right", (-cw / 2 - self.seam / 2, 0.0, 0.0), (self.seam, length, ch)),
            ("end", (0.0, length / 2 - self.end / 2, 0.0), (cw, self.end, ch)),
        ]
        if nd > 0 and nw > 0:
            for name, s in (("lip_left", 1), ("lip_right", -1)):
                parts.append((name, (s * (nw / 2 + side / 2), mouth + nd / 2, top_z), (side, nd, self.wall)))
        return parts


_ENVELOPES: dict[str, Envelope] = {}
_SURFACES: dict[str, tuple[float, str]] = {}
GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)


def surface(name: str, friction: float, combine: str = "min") -> None:
    """Give the task-file box ``name`` this static and dynamic friction, combined with the other body's by
    ``combine`` ("average", "min", "multiply" or "max": PhysX uses the later of the two bodies' modes in that
    list)."""
    if not friction >= 0:
        raise ValueError("friction must not be negative")
    if combine not in ("average", "min", "multiply", "max"):
        raise ValueError(f"unknown combine mode {combine!r}")
    _SURFACES[name] = (float(friction), combine)


def _material(stage, path, friction, combine: str):
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    static, dynamic = friction if isinstance(friction, tuple) else (friction, friction)
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateFrictionCombineModeAttr(combine)
    return material


def _bind(prim, material) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")


def _colliders(prim):
    from pxr import Usd, UsdPhysics

    return [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]


def _build(prim, spec: Envelope) -> None:
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    stage = prim.GetStage()
    for p in _colliders(prim):  # the solid cuboid stays for the bounds (and its look), without its collider
        for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
            if api in p.GetAppliedSchemas():
                p.RemoveAppliedSchema(api)
    material = _material(stage, prim.GetPath().AppendChild("envelope_material"), GRIPPY, "average")
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("walls"))
    for name, centre, size in spec.parts():
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"walls/{name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(0.005)
        contact.CreateRestOffsetAttr(0.0)
        _bind(cube.GetPrim(), material)


@on_spawn
def _spawned(name: str, prim) -> None:
    if name in _ENVELOPES:
        _build(prim, _ENVELOPES[name])
    if name in _SURFACES:
        friction, combine = _SURFACES[name]
        material = _material(prim.GetStage(), prim.GetPath().AppendChild("surface_material"), friction, combine)
        for p in _colliders(prim):
            _bind(p, material)


def envelope(spec: Envelope) -> Envelope:
    """Register ``spec`` so its walls are built when it spawns, and return it (for ``OBJECTS``)."""
    _ENVELOPES[spec.name] = spec
    return spec
