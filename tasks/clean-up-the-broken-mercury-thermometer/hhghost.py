"""hhghost: drawn-only boxes for a yam_arena task file (own implementation).

``GhostBox`` is a box with no collider, simulated as a kinematic body so a task can move it. Here: the dressing on his
forearm and the water in the basin, drawn only (never moved). Importing this module hooks
``yam_arena.scene`` (as hhperson does) so task files can list GhostBox objects; it changes nothing in yam_arena.
Offline (no simulator) nothing here runs. Isaac modules are imported lazily.
"""
from __future__ import annotations

import importlib.abc
import importlib.util
import sys
from dataclasses import dataclass

from yam_arena.task_file import Box


@dataclass(frozen=True)
class GhostBox(Box):
    """A box that is only drawn: kinematic, no collider, moved by the task. ``mass`` stays None (not a loose object)."""


def _t(x):
    return getattr(x, "torch", x)


def move(env, name: str, xyz) -> None:
    """Put the ghost ``name`` at ``xyz`` (world frame, unrotated)."""
    import torch

    base = env.unwrapped
    origin = _t(base.scene.env_origins)[0]
    t = torch.tensor([[float(xyz[0]), float(xyz[1]), float(xyz[2]), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                     device=origin.device)
    t[:, :3] += origin
    base.scene[name].write_root_pose_to_sim_index(root_pose=t)


def _ghost_object(spec: GhostBox, scene):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

    cfg = sim_utils.CuboidCfg(
        size=tuple(spec.size), rigid_props=PhysxRigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.2))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)))


def _patch_scene(scene) -> None:
    if getattr(scene, "_hhghost", False):
        return
    task_object = scene.task_object

    def task_object_hook(spec):
        if isinstance(spec, GhostBox):
            return _ghost_object(spec, scene)
        return task_object(spec)

    scene.task_object, scene._hhghost = task_object_hook, True


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
        exec_module = spec.loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
