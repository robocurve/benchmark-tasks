"""hhtool: a hand tool lying on the bench (a chisel, a mallet) as ONE rigid body made of boxes and lying cylinders.

The root is a box (the tool's handle, or a mallet's head) spawned the way yam_arena spawns its own cuboid; the other
parts are extra colliders and looks fixed to it, each (name, kind, dims, offset xyz in the root's frame, colour, quat).
``pos`` is the root box's centre. Importing this module hooks ``yam_arena.scene`` so a :class:`Tool` in ``OBJECTS``
is spawned this way; its state is yam_arena's own rigid-object state. yam_arena is not modified.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys
from dataclasses import dataclass

from yam_arena import task_file
from yam_arena.task_file import Box


@dataclass(frozen=True)
class Tool(Box):
    """``size`` is the root box; ``parts`` are (name, "box"|"cyl", dims, xyz, colour[, quat_wxyz]); a cylinder's dims are
    (radius, length) along its own z unless its quat turns it."""

    parts: tuple = ()
    friction: tuple = (0.9, 0.8)
    metallic_parts: tuple = ()


def _tool_object(spec: Tool, scene):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

    import hhcarry

    parts = tuple(spec.parts)

    @clone
    def _spawn(prim_path, cfg, *args, **kwargs):
        prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        phys = f"{prim_path}/Looks/tool_physics"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1])
        pm.func(phys, pm)
        for p in parts:
            name, kind, dims, xyz, color = p[:5]
            quat = p[5] if len(p) > 5 else (1.0, 0.0, 0.0, 0.0)
            metal = name in spec.metallic_parts
            look = hhcarry._look(f"{prim_path}/Looks/{name}", color, rough=0.3 if metal else 0.6, metallic=0.6 if metal else 0.0)
            hhcarry._geom(stage, f"{prim_path}/parts/{name}", kind, dims, xyz, quat, phys=phys, look=look)
        return prim

    cfg = sim_utils.CuboidCfg(
        size=tuple(spec.size), func=_spawn,
        rigid_props=PhysxRigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=scene._contact_props(),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1]),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.6))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)))


def _patch_scene(scene) -> None:
    if getattr(scene, "_hhtool", False):
        return
    orig_task_object = scene.task_object

    def task_object(spec):
        if isinstance(spec, Tool):
            return _tool_object(spec, scene)
        return orig_task_object(spec)

    scene.task_object, scene._hhtool = task_object, True


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
