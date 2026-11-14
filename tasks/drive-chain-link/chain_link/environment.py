"""Native free chain, connecting link and keeper; only normal robot controls act."""

from __future__ import annotations
import atexit
from dataclasses import replace
import hashlib
import importlib.util
import json
import os
import tempfile
import numpy as np
from inspect_robots.embodiment import PRIVILEGED_SUCCESS
from inspect_robots.types import StepResult, Observation
from inspect_robots_yam.config import observation_space
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.inspect_embodiment import (
    ARMS,
    JOINT_GUIDE,
    ROOM_DOCS,
    YamArenaEmbodiment,
)
from . import TASK_ROOT
from .config import (
    CONTROL_HZ,
    ROOT_POSITION,
    ROOT_QUATERNION,
    CONNECTOR_PICKUP_POSITION,
    KEEPER_POSITION,
)
from .geometry import local_solids, world_points, robot_clearance
from .scoring import Scorer

_APP = None


def numpy(value):
    if hasattr(value, "torch"):
        value = value.torch
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy().astype(float)
    if hasattr(value, "numpy"):
        return value.numpy().astype(float)
    return np.asarray(value, dtype=float)


def source_fingerprint():
    d = hashlib.sha256()
    for p in sorted(TASK_ROOT.rglob("*")):
        if (
            not p.is_file()
            or any(
                s in p.parts for s in ("__pycache__", ".pytest_cache", ".ruff_cache")
            )
            or ".egg-info" in str(p)
        ):
            continue
        if (
            p.suffix == ".py"
            or p.name == "pyproject.toml"
            or (p.parent.name == "motions" and p.suffix == ".json")
        ):
            d.update(p.relative_to(TASK_ROOT).as_posix().encode() + b"\0")
            d.update(hashlib.sha256(p.read_bytes()).digest())
    return d.hexdigest()


def close_app():
    if _APP is not None:
        _APP.close()


class ChainLinkEnvironment(YamArenaEmbodiment):
    def __init__(
        self,
        *,
        object_jitter=0,
        control_hz=CONTROL_HZ,
        physics_device=None,
        render_images=True,
        **kwargs,
    ):
        if float(object_jitter) != 0 or float(control_hz) != CONTROL_HZ:
            raise ValueError("Fixed layout at 20 Hz required")
        super().__init__(
            scene="station", object_jitter=0, control_hz=CONTROL_HZ, **kwargs
        )
        self.info = replace(
            self.info,
            name="chain_link",
            supported_target_kinds=frozenset(),
            capabilities=self.info.capabilities | {PRIVILEGED_SUCCESS},
            docs="\n\n".join(
                (
                    JOINT_GUIDE,
                    ROOM_DOCS["station"],
                    "A dark grey open training chain with bare end links is located by a dark grey fixed toothed assembly former. "
                    "A separate yellow two-pin connecting link lies on the table; its service cradle is empty. "
                    "A blue forked retaining plate lies on the left. All hardware is unpowered.",
                )
            ),
            environment_id="chain-link-" + source_fingerprint()[:16],
            environment_revision=source_fingerprint()[:12],
        )
        self.render_images = bool(render_images)
        if not self.render_images:
            self.info = replace(
                self.info,
                observation_space=observation_space(
                    self.cam_height, self.cam_width, ()
                ),
                environment_id=self.info.environment_id + "-physics-only",
            )
        self.physics_device = self.device if physics_device is None else physics_device
        self.task = self
        self.scorer = Scorer()
        self.metrics = {}
        self._callback = None
        self._temp = None
        self._audit_active = False
        self._audit_error = None

    def _ensure_sim(self, goal=None):
        global _APP
        if self._env is not None:
            return
        if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {
            "YES",
            "Y",
            "TRUE",
            "1",
        }:
            raise RuntimeError("Operator license acceptance is required")
        if _APP is None:
            from isaaclab.app import AppLauncher

            _APP = AppLauncher(
                headless=self.headless,
                device=self.device,
                enable_cameras=self.render_images,
                multi_gpu=False,
                width=self.cam_width,
                height=self.cam_height,
                anti_aliasing=0,
            ).app
            atexit.register(close_app)
        import torch
        from isaaclab.assets import AssetBaseCfg, RigidObjectCfg, ArticulationCfg
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

        spec = importlib.util.spec_from_file_location(
            "chain_asset_builder", TASK_ROOT / "build_scene.py"
        )
        builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(builder)
        self._temp = tempfile.TemporaryDirectory(prefix="chain-link-assets-")
        assets = builder.create_assets(self._temp.name)
        self._local = local_solids(
            json.loads(
                (assets["fixtures"].parent / "geometry-inventory.json").read_text()
            )
        )
        robot = BimanualYamEmbodiment(
            enable_cameras=self.render_images,
            rig=self.rig,
            contact_sensors=False,
            robot_contact_offset=0.001,
            cam_width=self.cam_width,
            cam_height=self.cam_height,
        )
        objects = [
            Object(
                name="chain",
                object_type=ObjectType.ARTICULATION,
                usd_path=str(assets["chain"]),
                initial_pose=Pose(
                    position_xyz=tuple(map(float, ROOT_POSITION)),
                    rotation_xyzw=tuple(map(float, ROOT_QUATERNION)),
                ),
            ),
            Object(
                name="connector",
                object_type=ObjectType.RIGID,
                usd_path=str(assets["connector"]),
                initial_pose=Pose(
                    position_xyz=tuple(map(float, CONNECTOR_PICKUP_POSITION))
                ),
            ),
            Object(
                name="keeper",
                object_type=ObjectType.RIGID,
                usd_path=str(assets["keeper"]),
                initial_pose=Pose(position_xyz=tuple(map(float, KEEPER_POSITION))),
            ),
            Object(
                name="fixtures",
                object_type=ObjectType.BASE,
                usd_path=str(assets["fixtures"]),
                initial_pose=Pose(),
            ),
        ]
        all_objects = [*task_room("station", self.rig.top_cam), *objects]
        # Arena mutates init_state when applying a pose. Supply a separate
        # supported configuration object to each asset, avoiding shared defaults.
        for item in all_objects:
            if not isinstance(item, Object):
                continue
            cls = {
                ObjectType.RIGID: RigidObjectCfg,
                ObjectType.ARTICULATION: ArticulationCfg,
            }.get(item.object_type, AssetBaseCfg)
            item.asset_cfg_addon = {
                **item.asset_cfg_addon,
                "init_state": cls.InitialStateCfg(),
            }

        def configure(cfg):
            from isaaclab_physx.physics.physx_manager_cfg import PhysxCfg

            cfg = env_cfg_callback(CONTROL_HZ)(cfg)
            # Empty passive actuator groups use the standard PhysX path.
            # The optional Newton actuator wrapper must not drive free hinges.
            cfg.sim.use_newton_actuators = False
            if self.physics_device == "cpu":
                cfg.sim.use_fabric = False
            # Capacity settings only: avoid reserving multi-environment buffers
            # for this single small scene. Contact dynamics are unchanged.
            cfg.sim.physics = PhysxCfg(
                gpu_max_rigid_contact_count=2**19,
                gpu_max_rigid_patch_count=2**17,
                gpu_found_lost_pairs_capacity=2**18,
                gpu_found_lost_aggregate_pairs_capacity=2**18,
                gpu_total_aggregate_pairs_capacity=2**18,
            )
            return cfg

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
        self._base, self._torch = self._env.unwrapped, torch
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
        self._callback = self._base.sim.physics_manager.register_callback(
            self._observe_physics,
            IsaacEvents.POST_PHYSICS_STEP,
            order=100,
            name="chain_link_validity",
            wrap_weak_ref=False,
        )

    def _observe_physics(self, dt):
        if not self._audit_active:
            return
        try:
            chain = self._base.scene["chain"]
            transforms = numpy(chain.root_view.get_link_transforms())[0]
            poses = {"chain:" + n: p for n, p in zip(chain.body_names, transforms)}
            poses.update(
                {
                    n + ":Body": numpy(self._base.scene[n].root_view.get_transforms())[
                        0
                    ]
                    for n in ("connector", "keeper")
                }
            )
            self.scorer.observe_physics(world_points(self._local, poses))
        except Exception as exc:
            self._audit_error = exc

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._audit_active = False
        self._env.reset(seed=seed)
        chain = self._base.scene["chain"]
        q = chain.data.joint_pos
        q = q.torch if hasattr(q, "torch") else q
        zeros = self._torch.zeros(q.shape, dtype=q.dtype, device=q.device)
        chain.write_joint_state_to_sim_index(position=zeros, velocity=zeros)
        self._instruction = scene.instruction
        for _ in range(10):
            self._env.step(self._ready)
        if self.render_images:
            for _ in range(4):
                self._base.sim.render()
        self.scorer.reset()
        self._audit_error = None
        self._t = 0.0
        self._audit_active = True
        self.metrics = self.scorer.evaluate(self.state())
        if self.metrics["score"] != 0 or self.metrics["violations"]:
            raise RuntimeError(f"Invalid reset: {self.metrics}")
        state = self.state()
        for name, expected in [
            ("connector", CONNECTOR_PICKUP_POSITION),
            ("keeper", KEEPER_POSITION),
        ]:
            if np.linalg.norm(np.array(state[name]["pose"][:3]) - expected) > 0.012:
                raise RuntimeError(
                    f"{name} reset pose {state[name]['pose'][:3]} differs from {expected.tolist()}"
                )
        observation = self._observation()
        from pxr import UsdGeom

        cache = UsdGeom.XformCache()
        self.reset_diagnostics = {"cameras": {}, "state": state}
        for prim in self._base.sim.stage.Traverse():
            if prim.GetTypeName() == "Camera":
                m = cache.GetLocalToWorldTransform(prim)
                self.reset_diagnostics["cameras"][str(prim.GetPath())] = {
                    "position": list(m.ExtractTranslation()),
                    "rotation_matrix": [list(row) for row in m.ExtractRotationMatrix()],
                }
        self.reset_image_stats = {
            n: {
                "std": float(np.std(image)),
                "min": int(image.min()),
                "max": int(image.max()),
            }
            for n, image in observation.images.items()
        }
        if self.render_images and any(
            np.std(image) < 1 for image in observation.images.values()
        ):
            raise RuntimeError(
                f"Invalid uniform camera frames: {self.reset_image_stats}"
            )
        return observation

    def state(self):
        from yam_arena.runtime import tcp_world

        chain = self._base.scene["chain"]
        poses = numpy(chain.data.body_link_pose_w)[0]
        velocity = numpy(chain.data.body_com_vel_w)[0]
        result = {
            "time_s": self._t,
            "bodies": {
                n: {"pose": p.tolist(), "velocity": v.tolist()}
                for n, p, v in zip(chain.body_names, poses, velocity)
            },
            "mechanism": {
                n: float(q)
                for n, q in zip(chain.joint_names, numpy(chain.data.joint_pos)[0])
            },
            "hands": {},
        }
        for n in ("connector", "keeper"):
            obj = self._base.scene[n]
            result[n] = {
                "pose": numpy(obj.data.root_link_pose_w)[0].tolist(),
                "velocity": numpy(obj.data.root_com_vel_w)[0].tolist(),
            }
        for n in ARMS:
            robot = self._base.scene[n]
            q = numpy(robot.data.joint_pos)[0]
            result["hands"][n] = {
                "tcp": tcp_world(robot).tolist(),
                "joint_positions": q.tolist(),
                "opening_m": float(-q[self._ids[n][1]].sum()),
                "poses": numpy(robot.data.body_link_pose_w)[0].tolist(),
                "names": robot.body_names,
            }
        points = world_points(
            self._local,
            {
                **{"chain:" + n: v["pose"] for n, v in result["bodies"].items()},
                **{n + ":Body": result[n]["pose"] for n in ("connector", "keeper")},
            },
        )
        result["bounds"] = {
            n: [p.min(0).tolist(), p.max(0).tolist()] for n, p in points.items()
        }
        result["robot_clearance_m"] = robot_clearance(points, result["hands"])
        return result

    def _task_step(self):
        if self._audit_error is not None:
            raise RuntimeError("Native physics audit failed") from self._audit_error
        if "invalid_state" in self.scorer.violations:
            raise RuntimeError("Nonfinite native physics state; trial is invalid")
        if self.scorer.physics_samples != round(self._t * 240):
            raise RuntimeError("Incomplete per-physics-step audit")
        self.metrics = self.scorer.evaluate(self.state())
        return StepResult(
            observation=self._observation(),
            terminated=self.metrics["success"],
            termination_reason="success" if self.metrics["success"] else None,
            info=self.metrics.copy(),
        )

    def diagnostics(self):
        return self.state()

    def _observation(self):
        if self.render_images:
            return super()._observation()
        per_arm = []
        for n in ARMS:
            q = numpy(self._base.scene[n].data.joint_pos)[0]
            arm_ids, finger_ids = self._ids[n]
            from yam_arena.constants import FINGER_OPEN

            grip = float(np.clip(q[finger_ids].mean() / FINGER_OPEN, 0, 1))
            per_arm.append(np.r_[q[arm_ids], grip])
        return Observation(
            images={},
            state={"joint_pos": np.r_[*per_arm]},
            instruction=self._instruction,
            state_time=self._t,
        )

    def close(self):
        self._audit_active = False
        if self._callback is not None:
            self._callback.deregister()
            self._callback = None
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp is not None:
            self._temp.cleanup()


def chain_link_embodiment(**kwargs):
    return ChainLinkEnvironment(**kwargs)
