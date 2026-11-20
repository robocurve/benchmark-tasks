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
from inspect_robots_yam.config import observation_space
from yam_arena import ik
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.inspect_embodiment import (
    ARMS,
    JOINT_GUIDE,
    ROOM_DOCS,
    YamArenaEmbodiment,
)
from yam_arena.runtime import tcp_world

from . import builder, config, scoring

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
        position = np.array([.66, -.24, 1.37])
        sight = np.array([.363, .027, .830]) - position
        sight /= np.linalg.norm(sight)
        screen_right = np.cross(sight, [0.0, 0.0, 1.0])
        screen_right /= np.linalg.norm(screen_right)
        camera = replace(
            camera, pos=tuple(position),
            rot_xyzw=tuple(ik.matrix_to_quat_xyzw(
                np.column_stack((screen_right, np.cross(screen_right, sight), -sight))
            )),
        )
        self.rig = replace(self.rig, top_cam=camera)
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
        room = task_room("station", self.rig.top_cam)
        # The overhead is mounted off the active worktop. The stock rear
        # mast would obscure this closer forward-facing camera pose.
        fixtures = [item for item in room if item.name in ("floor", "table", "dome_light", "lamp")]
        all_objects = [*fixtures, *objects]
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
        self._guard = self._base.scene["guard"]
        self._skate = self._base.scene["skate"]

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._env.reset(seed=seed)
        self._instruction = scene.instruction
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        self._t = 0.0
        self.scorer.reset()
        self.metrics = self.scorer.evaluate(self.state())
        if self.render_images:
            for _ in range(4):
                self._base.sim.render()
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        return obs

    def state(self):
        from yam_arena.urdf import quat_xyzw_to_matrix

        pose = array(self._skate.data.root_link_pose_w)[0]
        guard_view = self._guard.root_view
        links = dict(zip(self._guard.body_names,
                         array(guard_view.get_link_transforms())[0], strict=True))
        toe, heel = links["Toe"], links["Heel"]
        extension = max(0., -float(array(guard_view.get_dof_positions())[0, 0]))
        boot_rotation = quat_xyzw_to_matrix(pose[3:])
        toe_rotation = quat_xyzw_to_matrix(toe[3:])
        heel_rotation = quat_xyzw_to_matrix(heel[3:])
        # Measure physical cup tips and the channel bed in the runner frame.
        toe_stop = toe[:3] + toe_rotation @ [0., config.GUARD_TOE_INNER, config.GUARD_SEAT_Z]
        heel_stop = heel[:3] + heel_rotation @ [
            0., config.HEEL_AT_REST - config.HEEL_BODY_Y, config.GUARD_SEAT_Z,
        ]
        local = [boot_rotation.T @ (point - pose[:3]) for point in (toe_stop, heel_stop)]
        local.sort(key=lambda value: value[1])
        heel_gap = float(config.RUNNER_HEEL - local[0][1])
        toe_gap = float(local[1][1] - config.RUNNER_TOE)
        aligned = (
            all(abs(point[0]) <= .0068 and -.006 <= point[2] <= .006 for point in local)
            and abs(float(toe_rotation[:, 1] @ boot_rotation[:, 1])) >= .985
            and float(toe_rotation[:, 2] @ boot_rotation[:, 2]) >= .985
        )
        engaged = (
            aligned and .013 <= extension <= .045
            and -.004 <= toe_gap <= .009 and -.004 <= heel_gap <= .009
        )
        # Use the actual profile stations. A full-height rectangle around the
        # whole boot adds empty corners, especially above the rounded toe box.
        boot_points = config.BOOT_CORNERS @ boot_rotation.T + pose[:3]
        footprint = boot_points
        if engaged:
            footprint = np.vstack((boot_points, config.TOE_CORNERS @ toe_rotation.T + toe[:3],
                                   config.HEEL_CORNERS @ heel_rotation.T + heel[:3]))
        inside_horizontal = bool(np.all(
            np.abs(footprint[:, :2] - config.BAG[:2]) <= np.array(config.BAG_SIZE[:2]) / 2 - .002
        ))
        runner = config.RUNNER_CORNERS @ boot_rotation.T + pose[:3]
        packed = (
            inside_horizontal
            and float(boot_points[:, 2].min()) <= config.TABLE + config.BAG_FLOOR + .026
            and float(runner[:, 2].max()) <= config.TABLE + config.BAG_SIZE[2]
        )
        hands = {n: tcp_world(self._base.scene[n]) for n in ARMS}
        handles = [(pose[:3] + boot_rotation @ point, .087) for point in (
            config.BOOT_TOE_GRIP, config.BOOT_HEEL_GRIP,
        )] + [
            (toe[:3] + toe_rotation @ config.GUARD_TOE_GRIP, .077),
            (heel[:3] + heel_rotation @ [0., 0., .016], .077),
        ]
        hand_clear = all(
            np.linalg.norm(hand - handle) >= clearance
            for hand in hands.values() for handle, clearance in handles
        )
        return {
            "boot_pose": pose.tolist(), "guard_toe_pose": toe.tolist(),
            "guard_heel_pose": heel.tolist(), "extension_m": extension,
            "guard_engaged": bool(engaged), "boot_in_bag": bool(packed),
            "boot_speed_m_s": float(np.linalg.norm(array(self._skate.data.root_link_vel_w)[0, :3])),
            "toe_gap_m": toe_gap, "heel_gap_m": heel_gap,
            "guard_z_m": float(np.mean([point[2] for point in local])),
            "hands_clear": bool(hand_clear),
            "hands": {key: value.tolist() for key, value in hands.items()},
        }

    def _task_step(self):
        state = self.state()
        self.metrics = self.scorer.evaluate(state)
        return StepResult(
            observation=self._observation(), terminated=self.metrics["success"],
            termination_reason="success" if self.metrics["success"] else None,
            info=self.metrics.copy(),
        )

    def diagnostics(self):
        return self.state()

    def _observation(self):
        if self.render_images:
            return super()._observation()
        from yam_arena.constants import FINGER_OPEN

        values = []
        for n in ARMS:
            q = array(self._base.scene[n].data.joint_pos)[0]
            ids, fingers = self._ids[n]
            values.extend([*q[ids], float(np.clip(q[fingers].mean() / FINGER_OPEN, 0, 1))])
        return Observation(
            images={}, state={"joint_pos": np.asarray(values)}, instruction=self._instruction,
            state_time=self._t,
        )

    def close(self):
        self._active = False
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp is not None:
            self._temp.cleanup()


def embodiment(**kwargs):
    return Environment(**kwargs)
