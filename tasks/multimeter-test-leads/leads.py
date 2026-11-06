"""leads: a multimeter test lead as one rigid body: a plug body, a 4 mm pin and a rigid stub of cable.

yam_arena's task files spawn solid primitives, and a task-file cylinder always stands upright. :class:`Lead` is a
``Box`` with a mass: the spawned cuboid is the plug's body (``size``, its length along the box's own x), and when it
spawns the body gets more box colliders, shapes of the same rigid body: the pin out of its front (-x) end, with a
narrower tip so it finds a jack, and a stub of cable out of its back (+x) end. They are invisible (``looks.py`` puts
the drawn lead on the body) and keep their colliders. The pin's metal slides on what it touches (friction 0.3, the
lower of the two wins), the body and the stub grip like every task object.

Everything that needs Isaac Sim is imported when the box spawns (``bonds.on_spawn``). Pure Python at import time.
"""

from __future__ import annotations

from dataclasses import dataclass

from bonds import on_spawn
from yam_arena.task_file import Box

__all__ = ["Lead", "lead"]


@dataclass(frozen=True)
class Lead(Box):
    """A test lead lying along its own x, pin toward -x. ``size`` is the plug body's (length, width, height). The pin
    is ``pin_width`` square and ``pin_length`` long from the body's front face, its last ``tip_length`` narrowed to
    ``tip_width``. The stub is ``stub_width`` square and ``stub_length`` long from the body's back face. ``com`` is
    the centre of mass along x from the body's centre."""

    pin_width: float = 0.004
    pin_length: float = 0.018
    tip_width: float = 0.003
    tip_length: float = 0.002
    stub_width: float = 0.005
    stub_length: float = 0.10
    com: float = 0.010

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a lead must have a mass")
        _, w, h = self.size
        if not (0 < self.tip_width < self.pin_width < min(w, h) and 0 < self.tip_length < self.pin_length):
            raise ValueError(f"{self.name}: the pin must be narrower than the body, its tip narrower than the pin")
        if not 0 < self.stub_width <= min(w, h):
            raise ValueError(f"{self.name}: the stub must not be wider than the body")

    @property
    def tip(self) -> float:
        """The pin's tip along x from the body's centre (m, negative)."""
        return -self.size[0] / 2 - self.pin_length

    @property
    def end(self) -> float:
        """The stub's far end along x from the body's centre (m)."""
        return self.size[0] / 2 + self.stub_length

    @property
    def centroid(self) -> tuple[float, float, float]:
        """The centre of the lead's bounds in its own frame: what the scorer's ``pos`` is."""
        return ((self.tip + self.end) / 2, 0.0, 0.0)

    def parts(self) -> list[tuple]:
        """The added colliders as (name, centre, size[, (rotation about x, y, z) in degrees]) in the body's frame."""
        front, back = -self.size[0] / 2, self.size[0] / 2
        shank = self.pin_length - self.tip_length
        return [
            ("pin", (front - shank / 2, 0.0, 0.0), (shank, self.pin_width, self.pin_width)),
            ("pin_tip", (self.tip + self.tip_length / 2, 0.0, 0.0), (self.tip_length, self.tip_width, self.tip_width)),
            ("stub", (back + self.stub_length / 2, 0.0, 0.0), (self.stub_length, self.stub_width, self.stub_width)),
        ]


_LEADS: dict[str, Lead] = {}
PIN_FRICTION = 0.3  # brass on the jack's contacts
GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)


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


def _build(prim, spec: Lead) -> None:
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    metal = _material(stage, prim.GetPath().AppendChild("pin_material"), PIN_FRICTION, "min")
    rubber = _material(stage, prim.GetPath().AppendChild("stub_material"), GRIPPY, "average")
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("parts"))
    for name, centre, size, *turn in spec.parts():
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"parts/{name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if turn:
            cube.AddRotateXYZOp().Set(Gf.Vec3f(*turn[0]))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdGeom.Imageable(cube).MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(0.002)
        contact.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
            metal if name.startswith("pin") else rubber, UsdShade.Tokens.strongerThanDescendants, "physics")
    # The plug, not the stub, carries most of the mass: the centre of mass sits in the body, so the lead lies flat on
    # it. The inertia is a 16 cm rod's about its ends' axes. About its own axis a 1 cm rod's would be tiny and let the
    # plug twitch as the pads close on it, so it is ten times that, for the rest of the cable, which resists twisting.
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(float(spec.com), 0.0, 0.0))
    length = spec.end - spec.tip
    across = float(spec.mass) * length**2 / 12
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(float(spec.mass) * 0.005**2 / 2 * 10, across, across))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


@on_spawn
def _spawned(name: str, prim) -> None:
    if name in _LEADS:
        _build(prim, _LEADS[name])


def lead(spec: Lead) -> Lead:
    """Register ``spec`` so its pin and stub are built when it spawns, and return it (for ``OBJECTS``)."""
    _LEADS[spec.name] = spec
    return spec
