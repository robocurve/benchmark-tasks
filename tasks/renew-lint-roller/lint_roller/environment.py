"""Standard RGB/joint observations; native task contacts; scoring-only measurements."""

from __future__ import annotations

import atexit
import hashlib
import os
import tempfile
from dataclasses import replace
from pathlib import Path

import numpy as np
from inspect_robots.embodiment import PRIVILEGED_SUCCESS
from inspect_robots.types import Observation, StepResult
from inspect_robots_yam.config import DEFAULT_CAMERAS, observation_space
from yam_arena import ik
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.inspect_embodiment import (
    ARMS,
    JOINT_GUIDE,
    ROOM_DOCS,
    YamArenaEmbodiment,
)
from yam_arena.runtime import tcp_world
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from . import builder, config, materials, scoring

ROOT = Path(__file__).resolve().parents[1]
_APP = None


def array(value):
    if hasattr(value, "torch"):
        value = value.torch
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy().astype(float)
    if hasattr(value, "numpy"):
        return value.numpy().astype(float)
    return np.asarray(value, dtype=float)


def source_fingerprint():
    h = hashlib.sha256()
    for p in sorted(ROOT.rglob("*")):
        if (
            p.is_file()
            and (
                p.suffix == ".py"
                or p.name == "pyproject.toml"
                or p.parent.name == "motions"
            )
            and not any(
                x in str(p)
                for x in ("__pycache__", ".egg-info", "/runs/", "/validation/")
            )
        ):
            h.update(
                p.relative_to(ROOT).as_posix().encode()
                + b"\0"
                + hashlib.sha256(p.read_bytes()).digest()
            )
    return h.hexdigest()


class Environment(YamArenaEmbodiment):
    def __init__(
        self,
        *,
        physics_device="cpu",
        render_images=True,
        object_jitter=0,
        control_hz=20,
        **kwargs,
    ):
        if float(object_jitter) != 0 or float(control_hz) != 20:
            raise ValueError("Fixed reset and 20 Hz required")
        super().__init__(scene="station", object_jitter=0, control_hz=20, **kwargs)
        camera = self.rig.top_cam
        eye = np.array([.50, -.60, 1.49])
        direction = np.array([.34, .035, .82]) - eye
        direction /= np.linalg.norm(direction)
        right = np.cross(direction, [0., 0., 1.])
        right /= np.linalg.norm(right)
        up = np.cross(right, direction)
        rotation = np.column_stack((right, up, -direction))
        self.rig = replace(self.rig, top_cam=replace(
            camera, pos=tuple(eye), rot_xyzw=tuple(ik.matrix_to_quat_xyzw(rotation))
        ))
        self.render_images = bool(render_images)
        self.physics_device = physics_device
        self.info = replace(
            self.info,
            name=config.KEY,
            supported_target_kinds=frozenset(),
            capabilities=self.info.capabilities | {PRIVILEGED_SUCCESS},
            docs="\n\n".join((JOINT_GUIDE, ROOM_DOCS["station"], config.DESCRIPTION)),
            environment_id=config.KEY + "-" + source_fingerprint()[:16],
            environment_revision=source_fingerprint()[:12],
        )
        if not self.render_images:
            self.info = replace(
                self.info,
                observation_space=observation_space(
                    self.cam_height, self.cam_width, ()
                ),
            )
        self.task = self
        self.metrics = {}
        self.scorer = scoring.Scorer()
        self._active = False
        self._temp = None
        self._audit_error = None
        self.material = materials.Tape()
        self._material_callback = None

    def _ensure_sim(self, goal=None):
        global _APP
        if self._env is not None:
            return
        if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in (
            "YES",
            "Y",
            "TRUE",
            "1",
        ):
            raise RuntimeError("Activate the accepted-license simulator environment")
        if _APP is None:
            from isaaclab.app import AppLauncher

            # Forward a documented SimulationApp option in this process only.
            AppLauncher._SIM_APP_CFG_TYPES = {
                **AppLauncher._SIM_APP_CFG_TYPES,
                "enable_crashreporter": [bool],
            }
            _APP = AppLauncher(
                headless=self.headless,
                device=self.device,
                enable_cameras=self.render_images,
                multi_gpu=False,
                width=self.cam_width,
                height=self.cam_height,
                anti_aliasing=0,
                enable_crashreporter=False,
            ).app
            if (
                Path("/proc/self/maps").exists()
                and "libcarb.crashreporter" in Path("/proc/self/maps").read_text()
            ):
                _APP.close()
                raise RuntimeError("Crash reporter plugin unexpectedly loaded")
            atexit.register(_APP.close)
        import torch
        from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from isaaclab_arena.environments.isaaclab_arena_environment import (
            IsaacLabArenaEnvironment,
        )
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_physx.physics.physx_manager import IsaacEvents
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback, task_room

        self._temp = tempfile.TemporaryDirectory(prefix=config.KEY + "-")
        self.spec = builder.create_assets(self._temp.name)
        kind = {
            "rigid": ObjectType.RIGID,
            "articulation": ObjectType.ARTICULATION,
            "base": ObjectType.BASE,
        }
        objects = []
        for s in self.spec["assets"]:
            cls = {
                "rigid": RigidObjectCfg,
                "articulation": ArticulationCfg,
                "base": AssetBaseCfg,
            }[s["type"]]
            objects.append(
                Object(
                    name=s["name"],
                    object_type=kind[s["type"]],
                    usd_path=str(s["file"]),
                    initial_pose=Pose(
                        position_xyz=tuple(map(float, s.get("position", (0, 0, 0)))),
                        rotation_xyzw=tuple(
                            map(float, s.get("quaternion", (0, 0, 0, 1)))
                        ),
                    ),
                    asset_cfg_addon={"init_state": cls.InitialStateCfg()},
                )
            )
        all_objects = [*task_room("station", self.rig.top_cam), *objects]
        for o in all_objects:
            if isinstance(o, Object):
                cls = {
                    ObjectType.RIGID: RigidObjectCfg,
                    ObjectType.ARTICULATION: ArticulationCfg,
                }.get(o.object_type, AssetBaseCfg)
                o.asset_cfg_addon = {
                    **o.asset_cfg_addon,
                    "init_state": cls.InitialStateCfg(),
                }

        def configure(cfg):
            from isaaclab_physx.physics.physx_manager_cfg import PhysxCfg
            from yam_arena.constants import FINGER_OPEN

            cfg = env_cfg_callback(20)(cfg)
            cfg.sim.dt = 1.0 / config.PHYSICS_HZ
            cfg.decimation = config.PHYSICS_HZ // 20
            for name in ARMS:
                initial = getattr(cfg.scene, name).init_state
                initial.joint_pos = {
                    **dict(zip(ARM_JOINTS, READY)),
                    **dict.fromkeys(FINGER_JOINTS, FINGER_OPEN),
                }
                initial.joint_vel = {".*": 0.0}
            cfg.sim.use_newton_actuators = False
            if self.physics_device == "cpu":
                cfg.sim.use_fabric = False
            cfg.sim.physics = PhysxCfg(
                enable_ccd=True,
                gpu_max_rigid_contact_count=2**20,
                gpu_max_rigid_patch_count=2**18,
                gpu_found_lost_pairs_capacity=2**20,
                gpu_found_lost_aggregate_pairs_capacity=2**20,
                gpu_total_aggregate_pairs_capacity=2**20,
            )
            return cfg

        robot = BimanualYamEmbodiment(
            enable_cameras=self.render_images,
            rig=self.rig,
            contact_sensors=False,
            robot_contact_offset=0.001,
            cam_width=self.cam_width,
            cam_height=self.cam_height,
        )
        arena = IsaacLabArenaEnvironment(
            name=self.info.environment_id,
            scene=Scene(all_objects),
            embodiment=robot,
            env_cfg_callback=configure,
        )
        self._env = ArenaEnvBuilder(
            arena,
            ArenaEnvBuilderCfg(
                num_envs=1, solve_relations=False, device=self.physics_device
            ),
        ).make_registered()
        self._base = self._env.unwrapped
        self._torch = torch
        self._ready = torch.tensor(
            [[*READY, 1.0, *READY, 1.0]], dtype=torch.float32, device=self._base.device
        )
        self._ids = {
            n: (
                self._base.scene[n].find_joints(ARM_JOINTS, preserve_order=True)[0],
                self._base.scene[n].find_joints(FINGER_JOINTS, preserve_order=True)[0],
            )
            for n in ARMS
        }
        import warp as wp

        self._wp = wp
        self._index = wp.array([0], dtype=wp.int32, device=self._base.device)
        self._roller = self._base.scene["roller"]
        self._tab = self._base.scene["sheet_edge"]
        self._cloth = self._base.scene["cloth"]
        manager = self._base.sim.physics_manager
        simulation_view = manager.get_physics_sim_view()
        hub_path = "/World/envs/env_0/roller/Hub"
        self._hub = simulation_view.create_rigid_body_view(hub_path)
        self._handle = simulation_view.create_rigid_body_view("/World/envs/env_*/roller/Handle")
        self._material_callback = manager.register_callback(
            self._material_step, IsaacEvents.PRE_PHYSICS_STEP, order=50,
            name=config.KEY + "_material", wrap_weak_ref=False,
        )

    def _vector(self, value):
        return self._wp.array(np.asarray(value, dtype=np.float32).reshape(1, 3),
                              device=self._base.device)

    def _apply(self, view, force=None, position=None, torque=None):
        view.apply_forces_and_torques_at_position(
            force_data=None if force is None else self._vector(force),
            torque_data=None if torque is None else self._vector(torque),
            position_data=None if position is None else self._vector(position),
            indices=self._index,
            is_global=True,
        )

    def _material_step(self, dt):
        if not self._active:
            return
        try:
            view = self._roller.root_view
            hub = array(self._hub.get_transforms())[0]
            hub_velocity = array(self._hub.get_velocities())[0]
            tab = array(self._tab.root_view.get_transforms())[0]
            cloth = array(self._cloth.root_view.get_transforms())[0]
            spin = float(array(view.get_dof_velocities())[0, 0])
            force = self.material.advance(float(dt), hub, tab, cloth, hub_velocity[3:])
            drag = -materials.BEARING_DRAG * spin * self.material.rotation[:, 1]
            self._apply(self._handle, torque=-drag)
            self._apply(self._tab.root_view, force, tab[:3])
            hub_force = np.zeros(3)
            hub_torque = drag.copy()
            if self.material.progress < config.SHEET_LENGTH:
                hub_force -= force
                hub_torque += np.cross(self.material.front - hub[:3], -force)
            if self.material.load:
                cloth_velocity = array(self._cloth.root_view.get_velocities())[0]
                offset = -config.RADIUS * self.material.cloth_rotation[:, 2]
                contact = hub[:3] + offset
                roller_speed = hub_velocity[:3] + np.cross(hub_velocity[3:], offset)
                cloth_speed = cloth_velocity[:3] + np.cross(cloth_velocity[3:], contact - cloth[:3])
                nap_force = self.material.nap_force(roller_speed - cloth_speed)
                hub_force += nap_force
                hub_torque += np.cross(offset, nap_force)
                self._apply(self._cloth.root_view, -nap_force, contact)
            self._apply(self._hub, hub_force, hub[:3], torque=hub_torque)
        except Exception as error:
            self._audit_error = error

    def _render_material(self):
        self.material.render(self._base.sim.stage)

    def _refresh_cameras(self, frames=1):
        for _ in range(frames):
            self._base.sim.render()
        for name in DEFAULT_CAMERAS:
            self._base.scene[name].update(0., force_recompute=True)

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._active = False
        self._audit_error = None
        self._env.reset(seed=seed)
        self.material.reset()
        self._instruction = scene.instruction
        for _ in range(self.spec.get("settle_actions", 24)):
            self._env.step(self._ready)
        if self.render_images:
            self._render_material()
            self._refresh_cameras(frames=4)
        self._t = 0.0
        self.scorer.reset()
        if self._audit_error:
            raise RuntimeError(
                "Material integration failed during reset"
            ) from self._audit_error
        self.metrics = self.scorer.evaluate(self.state())
        self._active = True
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        return obs

    def state(self):
        result = {"time_s": self._t, "objects": {}}
        for s in self.spec["assets"]:
            if s["type"] == "base":
                continue
            o = self._base.scene[s["name"]]
            if s["type"] == "articulation":
                d = {
                    "pose": array(o.root_view.get_root_transforms())[0].tolist(),
                    "velocity": array(o.root_view.get_root_velocities())[0].tolist(),
                    "joints": {
                        n: float(q)
                        for n, q in zip(
                            o.joint_names, array(o.root_view.get_dof_positions())[0]
                        )
                    },
                    "links": {
                        n: p.tolist()
                        for n, p in zip(
                            o.body_names, array(o.root_view.get_link_transforms())[0]
                        )
                    },
                }
            else:
                d = {
                    "pose": array(o.root_view.get_transforms())[0].tolist(),
                    "velocity": array(o.root_view.get_velocities())[0].tolist(),
                }
            result["objects"][s["name"]] = d
        result["material"] = self.material.snapshot()
        result["hands"] = {}
        for name in ARMS:
            arm = self._base.scene[name]
            poses = array(arm.data.body_link_pose_w)[0]
            _, fingers = self._ids[name]
            result["hands"][name] = {
                "points": [
                    tcp_world(arm).tolist(),
                    tcp_world(arm, past_tcp=0.013).tolist(),
                    *[
                        pose[:3].tolist()
                        for part, pose in zip(arm.body_names, poses)
                        if part in ("gripper", "tip_left", "tip_right")
                    ],
                ],
                "opening_m": float(-array(arm.data.joint_pos)[0, fingers].sum()),
            }
        return result

    def _task_step(self):
        if self._audit_error:
            raise RuntimeError("Physics audit failed") from self._audit_error
        self.metrics = self.scorer.evaluate(self.state())
        if self.render_images:
            self._render_material()
            self._refresh_cameras()
        return StepResult(
            observation=self._observation(),
            terminated=self.metrics["success"],
            termination_reason="success" if self.metrics["success"] else None,
            info=self.metrics.copy(),
        )

    def diagnostics(self):
        return self.state()

    def seam_feedback(self):
        """The local reference measures the existing edge through native pose."""
        return array(self._tab.root_view.get_transforms())[0, :3].copy()

    def _observation(self):
        if self.render_images:
            return super()._observation()
        from yam_arena.constants import FINGER_OPEN

        values = []
        for n in ARMS:
            q = array(self._base.scene[n].data.joint_pos)[0]
            ids, fingers = self._ids[n]
            values.extend(
                [*q[ids], float(np.clip(q[fingers].mean() / FINGER_OPEN, 0, 1))]
            )
        return Observation(
            images={},
            state={"joint_pos": np.asarray(values)},
            instruction=self._instruction,
            state_time=self._t,
        )

    def close(self):
        self._active = False
        if self._material_callback:
            self._material_callback.deregister()
            self._material_callback = None
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp:
            self._temp.cleanup()


def embodiment(**kwargs):
    return Environment(**kwargs)
