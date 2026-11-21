"""hhgear: a worm gearmotor (a size-040 worm gear reducer with a flange-mounted IEC 56 motor) for a yam_arena task file.

:class:`Gearmotor` is one rigid body. Its spawned primitive is the reducer housing (a cuboid collider, drawn as cast
aluminium); the other solid parts (the output bosses, the motor's B14 flange, the motor body and its fan cowl, the
terminal box) are extra colliders on the same body, and the small details (housing ribs, the hollow output bore,
the motor's cooling ribs, the nameplate, the breather and drain plugs, the cable gland) are visual only. The body's
centre of mass is set to the real unit's (the reducer and the motor weigh about the same, so it lies between them),
not the housing's centre.

Importing this module hooks ``yam_arena.scene`` so a Gearmotor in ``OBJECTS`` is spawned as above (the same approach
as hhperson and hhcarry). Its state is yam_arena's own rigid-object state (the housing's pose). yam_arena is not
modified.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

from yam_arena import task_file
from yam_arena.task_file import Box


@dataclass(frozen=True)
class Gearmotor(Box):
    """``pos`` is the housing's centre; ``size`` the housing. ``parts``: (name, kind, dims, offset, axis, sRGB colour,
    collides, metallic) with kind "box" (dims = full size) or "cyl" (dims = (radius, length), along ``axis`` x/y/z).
    ``com``: the centre of mass, from the housing's centre."""

    parts: tuple = ()
    com: tuple = (0.0, 0.0, 0.0)
    friction: tuple = (0.5, 0.45)


_AXIS_Q = {"z": (1.0, 0.0, 0.0, 0.0), "x": (math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0),
           "y": (math.cos(math.pi / 4), -math.sin(math.pi / 4), 0.0, 0.0)}  # wxyz turning a cylinder's z onto the axis


def _gear_object(spec: Gearmotor, scene):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

    parts = tuple(spec.parts)

    @clone
    def _spawn(prim_path, cfg, *args, **kwargs):
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        phys = f"{prim_path}/Looks/gear_physics"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1])
        pm.func(phys, pm)
        for name, kind, dims, off, axis, color, collides, metallic in parts:
            path = f"{prim_path}/parts/{name}"
            if kind == "box":
                g = UsdGeom.Cube.Define(stage, path)
                g.CreateSizeAttr(1.0)
                g.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in off]))
                g.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in dims]))
            else:
                g = UsdGeom.Cylinder.Define(stage, path)
                g.CreateRadiusAttr(float(dims[0]))
                g.CreateHeightAttr(float(dims[1]))
                g.CreateAxisAttr("Z")
                g.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in off]))
                g.AddOrientOp().Set(Gf.Quatf(*_AXIS_Q[axis]))
            p = g.GetPrim()
            if collides:
                UsdPhysics.CollisionAPI.Apply(p)
                cp = PhysxSchema.PhysxCollisionAPI.Apply(p)
                cp.CreateContactOffsetAttr(0.004)
                cp.CreateRestOffsetAttr(0.0)
                sim_utils.bind_physics_material(path, phys, stage=stage)
            look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(color), roughness=0.45,
                                               metallic=float(metallic))
            look.func(f"{prim_path}/Looks/{name}", look)
            sim_utils.bind_visual_material(path, f"{prim_path}/Looks/{name}", stage=stage)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*[float(v) for v in spec.com]))
        return prim

    cfg = sim_utils.CuboidCfg(
        size=tuple(spec.size), func=_spawn,
        rigid_props=PhysxRigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=scene._contact_props(),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0],
                                                        dynamic_friction=spec.friction[1]),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.5,
                                                    metallic=0.3))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)))


def collider_boxes(spec: Gearmotor) -> list:
    """World-frame (centre, size) boxes bounding the housing and every colliding part (the unit is not yawed)."""
    import numpy as np

    c = np.asarray(spec.pos, float)
    out = [(c, tuple(spec.size))]
    for _n, kind, dims, off, axis, _c, collides, _m in spec.parts:
        if not collides:
            continue
        if kind == "box":
            size = tuple(dims)
        else:
            r, h = dims
            size = tuple(h if a == axis else 2 * r for a in "xyz")
        out.append((c + np.asarray(off, float), size))
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_hhgear", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, Gearmotor):
            return _gear_object(spec, scene)
        return orig(spec)

    scene.task_object, scene._hhgear = task_object, True


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


def install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
