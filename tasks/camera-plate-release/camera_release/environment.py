"""Native PhysX scene and the ordinary Inspect Robots/YAM policy interface.

Only robot joint/gripper targets are actuated during an episode. A read-only
post-physics callback records apparatus motion/contact for the validity scorer.
"""

from __future__ import annotations

import atexit
from dataclasses import replace
import hashlib
import importlib.util
import os
from pathlib import Path
import tempfile

import numpy as np
from inspect_robots.embodiment import PRIVILEGED_SUCCESS
from inspect_robots.types import StepResult
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.inspect_embodiment import ARMS, JOINT_GUIDE, ROOM_DOCS, YamArenaEmbodiment

from . import TASK_ROOT
from .config import (
    CAM_PIVOT, CONTACT_THRESHOLD_N, CONTROL_HZ, INSTRUCTION, MOUNT_POSITION,
    PAD_POSITION, PAYLOAD_POSITION, PHYSICS_HZ,
)
from .geometry import aabb_gap, bounds, payload_corners, rotation_xyzw
from .scoring import Sample, Scorer

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
    digest = hashlib.sha256()
    for path in sorted(TASK_ROOT.rglob("*")):
        if not path.is_file() or any(x in path.parts for x in ("__pycache__", ".pytest_cache", ".ruff_cache")):
            continue
        if ".egg-info" in str(path) or path.suffix == ".pyc":
            continue
        if path.suffix != ".py" and "assets" not in path.relative_to(TASK_ROOT).parts and path.name != "pyproject.toml":
            continue
        digest.update(path.relative_to(TASK_ROOT).as_posix().encode()+b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def close_app():
    if _APP is not None:
        _APP.close()


class CameraReleaseEnvironment(YamArenaEmbodiment):
    """One fixed passive camera mount; images and proprioception are policy inputs."""

    def __init__(self, *, object_jitter=0, control_hz=CONTROL_HZ, **kwargs):
        if float(object_jitter) != 0 or float(control_hz) != CONTROL_HZ:
            raise ValueError("The submitted task uses object_jitter=0 and control_hz=20")
        super().__init__(scene="station", object_jitter=0, control_hz=CONTROL_HZ, **kwargs)
        self.info = replace(
            self.info, name="camera_release", supported_target_kinds=frozenset(),
            capabilities=self.info.capabilities | {PRIVILEGED_SUCCESS},
            docs="\n\n".join((JOINT_GUIDE, ROOM_DOCS["station"],
                "A camera fastened to a plate rests in a cam-clamped bench mount. A green receiving pad is beside it. "
                "The fixture, camera and pad are unpowered. The camera stays attached to its plate.")),
            environment_id="camera-release-"+source_fingerprint()[:16],
            environment_revision=source_fingerprint()[:12],
        )
        self.task = self  # route the inherited robot step to this task's scorer
        self.scorer = Scorer()
        self.metrics = {}
        self._audit_active = False
        self._audit_error = None
        self._callback = None
        self._temp = None

    def _ensure_sim(self, goal=None):
        global _APP
        if self._env is not None:
            return
        if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {"YES", "Y", "TRUE", "1"}:
            raise RuntimeError("Set OMNI_KIT_ACCEPT_EULA only after the operator accepts NVIDIA's license")
        if _APP is None:
            from isaaclab.app import AppLauncher

            _APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True, multi_gpu=False).app
            atexit.register(close_app)
        import torch
        from isaaclab.sensors import ContactSensorCfg
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_physx.physics.physx_manager import IsaacEvents
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback, task_room

        spec = importlib.util.spec_from_file_location("camera_release_asset_builder", TASK_ROOT/"build_scene.py")
        builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(builder)
        self._temp = tempfile.TemporaryDirectory(prefix="camera-release-assets-")
        mount, payload, pad = builder.create_assets(self._temp.name)
        robot = BimanualYamEmbodiment(enable_cameras=True, rig=self.rig, contact_sensors=False,
                                     robot_contact_offset=0.001)
        apparatus = [
            Object(name="camera_mount", object_type=ObjectType.ARTICULATION, usd_path=str(mount),
                   initial_pose=Pose(position_xyz=tuple(map(float, MOUNT_POSITION)))),
            Object(name="camera_payload", object_type=ObjectType.RIGID, usd_path=str(payload),
                   initial_pose=Pose(position_xyz=tuple(map(float, PAYLOAD_POSITION)))),
            Object(name="camera_pad", object_type=ObjectType.BASE, usd_path=str(pad),
                   initial_pose=Pose(position_xyz=tuple(map(float, PAD_POSITION)))),
        ]
        base_config = env_cfg_callback(CONTROL_HZ)

        def configure(cfg):
            cfg = base_config(cfg)
            # Separate finger partners avoid cancellation of opposing grasp forces.
            partners = ["{ENV_REGEX_NS}/"+side+robot.link_paths[link]
                        for side in ("LeftArm", "RightArm") for link in ("tip_left", "tip_right", "gripper")]
            cfg.scene.payload_contacts = ContactSensorCfg(
                prim_path="{ENV_REGEX_NS}/camera_payload", filter_prim_paths_expr=partners,
                history_length=0, update_period=0, max_contact_data_count_per_prim=128,
            )
            return cfg

        arena = IsaacLabArenaEnvironment(
            name=self.info.environment_id, scene=Scene([*task_room("station", self.rig.top_cam), *apparatus]),
            embodiment=robot, env_cfg_callback=configure,
        )
        self._env = ArenaEnvBuilder(arena, ArenaEnvBuilderCfg(
            num_envs=1, solve_relations=False, device=self.device,
        )).make_registered()
        self._base, self._torch = self._env.unwrapped, torch
        self._ready = torch.tensor([[*READY, 1.0, *READY, 1.0]], dtype=torch.float32, device=self._base.device)
        self._ids = {name: (
            self._base.scene[name].find_joints(ARM_JOINTS, preserve_order=True)[0],
            self._base.scene[name].find_joints(FINGER_JOINTS, preserve_order=True)[0],
        ) for name in ARMS}
        self._callback = self._base.sim.physics_manager.register_callback(
            self._observe_physics, IsaacEvents.POST_PHYSICS_STEP, order=100,
            name="camera_release_validity", wrap_weak_ref=False,
        )

    def _contact_present(self, dt=1/PHYSICS_HZ):
        view = self._base.scene["payload_contacts"].contact_view
        net = numpy(view.get_net_contact_forces(dt=dt)).reshape(-1, 3)
        partners = numpy(view.get_contact_force_matrix(dt=dt)).reshape(-1, 3)
        return bool(max(np.linalg.norm(net, axis=1).max(initial=0),
                        np.linalg.norm(partners, axis=1).max(initial=0)) > CONTACT_THRESHOLD_N)

    def _observe_physics(self, dt):
        if not self._audit_active:
            return
        try:
            pose = numpy(self._base.scene["camera_payload"].root_view.get_transforms())[0]
            self.scorer.observe_physics(pose[:3], pose[3:], self._contact_present(float(dt)))
        except Exception as exc:
            self._audit_error = exc
            self.scorer.violations.add("invalid_state")

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._audit_active = False
        self._env.reset(seed=seed)
        mount = self._base.scene["camera_mount"]
        index = mount.joint_names.index("cam_angle")
        current = mount.data.joint_pos
        current = current.torch if hasattr(current, "torch") else current
        values = self._torch.zeros((1, 1), dtype=current.dtype, device=current.device)
        mount.write_joint_state_to_sim_index(position=values, velocity=values, joint_ids=[index])
        self._instruction = scene.instruction
        for _ in range(10):
            self._env.step(self._ready)
        for _ in range(4):
            self._base.sim.render()
        self.scorer.reset()
        self._audit_error = None
        self._t = 0.0
        self._audit_active = True
        self.metrics = self.scorer.evaluate(self.sample())
        if self.metrics["score"] != 0 or self.metrics["violations"]:
            raise RuntimeError(f"Invalid reset: {self.metrics}")
        position = self.sample().position
        if np.linalg.norm(position-PAYLOAD_POSITION) > 0.005:
            raise RuntimeError(f"Payload did not settle in its declared starting layout: {position}")
        return self._observation()

    def sample(self):
        from yam_arena.analysis import link_hulls

        payload = self._base.scene["camera_payload"]
        pose = numpy(payload.data.root_link_pose_w)[0]
        velocity = numpy(payload.data.root_com_vel_w)[0]
        mount = self._base.scene["camera_mount"]
        body = numpy(mount.data.body_link_pose_w)[0]
        stand = body[mount.body_names.index("Stand")]
        cam = body[mount.body_names.index("Cam")]
        angle = float(numpy(mount.data.joint_pos)[0, mount.joint_names.index("cam_angle")])
        anchor = stand[:3]+rotation_xyzw(stand[3:]) @ CAM_PIVOT
        payload_bounds = bounds(payload_corners(pose[:3], pose[3:]))
        distances = []
        for name in ARMS:
            robot = self._base.scene[name]
            robot_poses = numpy(robot.data.body_link_pose_w)[0]
            for link, link_pose in zip(robot.body_names, robot_poses):
                rot = rotation_xyzw(link_pose[3:])
                for hull in link_hulls().get(link, []):
                    world = hull.vertices @ rot.T+link_pose[:3]
                    distances.append(aabb_gap(payload_bounds, bounds(world)))
        if not distances:
            raise RuntimeError("No robot collision geometry available for the release measurement")
        return Sample(
            time=self._t, position=pose[:3], quaternion=pose[3:], velocity=velocity,
            cam_angle=angle, stand_position=stand[:3], stand_quaternion=stand[3:],
            cam_anchor_error=float(np.linalg.norm(cam[:3]-anchor)),
            support_contact=self._contact_present(), robot_clearance=min(distances),
        )

    def _task_step(self):
        if self._audit_error is not None:
            raise RuntimeError("Native per-physics-step audit failed") from self._audit_error
        expected = round(self._t*PHYSICS_HZ)
        if self.scorer.physics_samples != expected:
            raise RuntimeError(f"Incomplete physics audit: {self.scorer.physics_samples} of {expected} samples")
        self.metrics = self.scorer.evaluate(self.sample())
        return StepResult(observation=self._observation(), terminated=self.metrics["success"],
                          termination_reason="success" if self.metrics["success"] else None,
                          info=self.metrics.copy())

    def diagnostics(self):
        """Measured native robot and mechanism poses, retained only in the private run trace."""
        from yam_arena.runtime import tcp_world

        result = {}
        for name in ARMS:
            robot = self._base.scene[name]
            poses = numpy(robot.data.body_link_pose_w)[0]
            result[name] = {
                "tcp_m": tcp_world(robot).tolist(),
                "joint_positions": numpy(robot.data.joint_pos)[0].tolist(),
                "hand_link_poses_xyzw": {link: poses[robot.body_names.index(link)].tolist()
                                         for link in ("gripper", "tip_left", "tip_right")},
            }
        mount = self._base.scene["camera_mount"]
        pose = numpy(mount.data.body_link_pose_w)[0, mount.body_names.index("Cam")]
        result["cam_lever_centre_m"] = (pose[:3]+rotation_xyzw(pose[3:])@np.array([-0.02, -0.018, 0])).tolist()
        return result

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


def camera_release_embodiment(**kwargs):
    return CameraReleaseEnvironment(**kwargs)
