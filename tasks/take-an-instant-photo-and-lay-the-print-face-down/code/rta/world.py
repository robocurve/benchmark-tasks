"""What a task file's events() may do to the running simulation. Import after app start."""

from __future__ import annotations

import math

import numpy as np


class World:
    """What ``events()`` may do: ``set_pose`` teleports a free object (and zeroes its velocity); ``push`` holds an
    external force on one (the embodiment does this with a task's ``forces()``); ``show`` makes a
    named part of a model visible or invisible (a display's screens, a status light); ``pose(name)`` /
    ``objects`` read where things are as of the last control step (so a prop can be re-picked where physics left
    it). Before the first step ``objects`` is None and ``pose`` raises."""

    def __init__(self, env, torch) -> None:
        self._base = env.unwrapped
        self._torch = torch
        self._origin = self._base.scene.env_origins[0]
        self.objects = None  # the latest Snapshot, set by the embodiment each step

    def pose(self, name: str) -> np.ndarray:
        """World position (bounding-box centre) of ``name`` after the last control step."""
        if self.objects is None:
            raise RuntimeError("object poses are available from the first control step on")
        return np.asarray(self.objects[name].pos, dtype=float)

    def set_pose(self, name: str, pos, yaw: float = 0.0, quat=None) -> None:
        """Teleport the free object ``name`` (its root) to ``pos``, turned ``yaw`` about the vertical or, with
        ``quat`` (x, y, z, w), in any orientation; its velocity is zeroed."""
        torch = self._torch
        if name not in self._base.scene.rigid_objects:
            raise KeyError(f"{name!r} is not a free object; only free objects can be moved")
        obj = self._base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(np.asarray(pos, dtype=np.float64), dtype=pose.dtype, device=pose.device) + self._origin
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) if quat is None else tuple(float(v) for v in quat)
        pose[:, 3:] = torch.tensor(q, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))

    def push(self, name: str, force) -> None:
        """Hold an external force (N, world frame, at the centre of mass) on the free object ``name`` from now until
        the next ``push`` on it or the next reset: Isaac Lab applies it on every physics substep. On an articulated
        model (a task's ``JOINTS``) the force acts on its root link."""
        torch = self._torch
        scene = self._base.scene
        if name in scene.rigid_objects:
            obj, body_ids = scene[name], None
        elif name in scene.articulations and not name.endswith("_robot"):
            obj, body_ids = scene[name], [0]  # the root link
        else:
            raise KeyError(f"{name!r} is not a free object of the task; forces act on free objects only")
        f = torch.tensor(np.asarray(force, dtype=np.float64), dtype=torch.float32, device=self._origin.device)
        f = f.view(1, 1, 3).repeat(obj.num_instances, 1, 1)
        obj.permanent_wrench_composer.set_forces_and_torques_index(forces=f, torques=torch.zeros_like(f), body_ids=body_ids, is_global=True)

    def show(self, name: str, part: str, visible: bool = True) -> None:
        """Show or hide ``part`` (a child prim of the model spawned as ``name``): which screen a display shows,
        whether a light is lit. Rendering only; colliders are unaffected."""
        from pxr import UsdGeom

        prim = self._part(name, part)
        image = UsdGeom.Imageable(prim)
        image.MakeVisible() if visible else image.MakeInvisible()

    def _part(self, name: str, part: str):
        cache = self.__dict__.setdefault("_parts", {})
        if (name, part) not in cache:
            from isaaclab.sim.utils.stage import get_current_stage

            if name not in self._base.scene.keys():
                raise KeyError(f"{name!r} is not an object of this scene")
            root = self._base.scene[name].cfg.prim_path.replace("env_[^/]+", "env_0").replace("env_.*", "env_0")  # env 0 of the env pattern
            prim = get_current_stage().GetPrimAtPath(f"{root}/{part}")
            if not prim.IsValid():
                raise KeyError(f"{name!r} has no part {part!r} at {root}/{part}")
            cache[(name, part)] = prim
        return cache[(name, part)]
