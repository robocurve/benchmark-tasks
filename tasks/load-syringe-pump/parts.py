"""Loose parts built from several boxes (a processor with notches in its edge) for a yam_arena task file.

yam_arena's task files list single primitives. A loose part here is a ``Part``: one rigid body whose colliders are
several boxes, written to a USD file when the scene is built and spawned in place of a primitive. It subclasses
``Box`` with a mass, so yam_arena treats it as a free object and reports its state like any other.

Importing this module chains a patch of ``yam_arena.scene.task_object`` onto mechanism.py's import hook, so a task file
that uses both still runs with the standard ``yam_arena`` embodiment. Nothing here imports Isaac Sim at import time.
"""

from __future__ import annotations

import math
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import mechanism as mech
from yam_arena.task_file import Box

__all__ = ["Piece", "Part", "install"]

Color = tuple[float, float, float]


@dataclass(frozen=True)
class Piece:
    """One box collider of a part, in the part's frame (m): centre and edge lengths, turned by ``rot`` and ``yaw``."""

    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: Color | None = None  # the part's colour if None
    yaw: float = 0.0
    rot: tuple[float, float, float] = (0.0, 0.0, 0.0)  # turns about x, then y, then z (rad), before ``yaw``


@dataclass(frozen=True)
class Part(Box):
    """A loose rigid part: ``pieces`` in its own frame, whose origin is at ``pos`` turned by ``yaw``. ``size`` is its
    bounding box (for yam_arena's checks) and ``mass`` its total mass (kg)."""

    pieces: tuple[Piece, ...] = ()
    contact_offset: float = 0.001

    def _check(self) -> None:
        super()._check()
        if self.mass is None or not self.pieces:
            raise ValueError(f"{self.name}: a Part needs pieces and a mass (it is loose)")


def _write_usd(spec: Part) -> str:
    """The part as a USD rigid body: one cube collider per piece, coloured. Written under the temp folder."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    from yam_arena.station import linear_rgb

    folder = Path(tempfile.gettempdir()) / "yam_parts"
    folder.mkdir(exist_ok=True)
    path = folder / f"{spec.name}_{abs(hash((spec.pieces, spec.color))) % 10**10}.usda"
    path.unlink(missing_ok=True)  # CreateNew refuses an existing file
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/Part")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(spec.mass)
    looks: dict[Color, UsdShade.Material] = {}

    def look(color: Color):
        if color not in looks:
            mat = UsdShade.Material.Define(stage, f"/Part/Looks/color_{len(looks)}")
            shader = UsdShade.Shader.Define(stage, f"/Part/Looks/color_{len(looks)}/Surface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            looks[color] = mat
        return looks[color]

    for i, piece in enumerate(spec.pieces):
        cube = UsdGeom.Cube.Define(stage, f"/Part/piece_{i:02d}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*piece.center))
        if any(piece.rot):
            cube.AddRotateXYZOp().Set(Gf.Vec3f(*(math.degrees(a) for a in piece.rot)))
        if piece.yaw:
            cube.AddRotateZOp().Set(math.degrees(piece.yaw))
        cube.AddScaleOp().Set(Gf.Vec3f(*piece.size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(look(piece.color or spec.color))
    stage.GetRootLayer().Save()
    return str(path)


def _arena_object(spec: Part):
    """The part as an Isaac Lab-Arena rigid object, with the objects' grippy friction and a tight contact offset."""
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg

    from yam_arena import scene

    addon = {
        "physics_material": scene._grippy(),
        "mass_props": {"": [sim_utils.MassCfg(mass=spec.mass)]},
        "mass_props_create_if_missing": True,
        "collision_props": PhysxCollisionPropertiesCfg(contact_offset=spec.contact_offset, rest_offset=0.0),
        "rigid_props": PhysxRigidBodyPropertiesCfg(solver_position_iteration_count=32, solver_velocity_iteration_count=4),
    }
    return Object(
        name=spec.name,
        usd_path=_write_usd(spec),
        object_type=ObjectType.RIGID,
        initial_pose=Pose(position_xyz=tuple(spec.pos),
                          rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
        spawn_cfg_addon=addon,
    )


def _patch_scene(scene) -> None:
    if getattr(scene, "_parts_patched", False):
        return
    task_object = scene.task_object

    def patched_task_object(spec):
        return _arena_object(spec) if isinstance(spec, Part) else task_object(spec)

    scene.task_object = patched_task_object
    scene._parts_patched = True


def install() -> None:
    """Patch yam_arena.scene after mechanism.py's own patch, now if it is imported, else when it is."""
    before = mech._PATCHES["yam_arena.scene"]
    if not getattr(before, "_parts_chain", False):

        def chained(module, before=before):
            before(module)
            _patch_scene(module)

        chained._parts_chain = True
        mech._PATCHES["yam_arena.scene"] = chained
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])


install()
