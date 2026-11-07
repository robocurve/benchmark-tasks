# SPDX-License-Identifier: MIT
"""Physical captive-slider runtime for the pinned YAM Arena adapter.

Scene construction hooks expire before stepping. The carrier floats freely and
its prismatic joint is passive. State writes occur only during episode reset.
Privileged object measurements never enter ordinary policy observations.
"""
from __future__ import annotations

import atexit
import hashlib
import json
import math
import os
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np
from yam_arena.inspect_embodiment import RESET_RENDERS, SETTLE_S, YamArenaEmbodiment
from . import task_spec as spec
from .scoring import DockingScorer, config_dict


def _numpy(value):
    value = value.torch if hasattr(value, "torch") else value
    return value.detach().cpu().numpy().copy()


def source_manifest():
    root = Path(__file__).parent
    files = {str(f.relative_to(root)).replace("\\", "/"): hashlib.sha256(f.read_bytes()).hexdigest()
             for f in sorted([*root.glob("*.py"), *root.glob("assets/*.usda")])}
    digest = hashlib.sha256()
    for name, checksum in files.items():
        digest.update(name.encode() + b"\0" + checksum.encode() + b"\n")
    return dict(schema="captive-crossbar-docking-source-v1", revision=digest.hexdigest(), files=files)


def _reset_carrier(env, env_ids, *, height_offset=0.):
    """Restore free root and slider only as part of an episode reset."""
    carrier = env.scene["carrier"]
    origins = env.scene.env_origins[env_ids]
    half_yaw = spec.CARRIER_INITIAL_YAW / 2
    pose = origins.new_tensor([*spec.CARRIER_INITIAL_POS, 0., 0., math.sin(half_yaw), math.cos(half_yaw)])
    pose = pose.repeat(len(env_ids), 1)
    pose[:, :3] += origins
    pose[:, 2] += height_offset
    carrier.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    carrier.write_root_velocity_to_sim_index(root_velocity=origins.new_zeros((len(env_ids), 6)), env_ids=env_ids)
    carrier.write_joint_position_to_sim_index(
        position=origins.new_full((len(env_ids), 1), spec.CARRIER_INITIAL_Q), env_ids=env_ids)
    carrier.write_joint_velocity_to_sim_index(velocity=origins.new_zeros((len(env_ids), 1)), env_ids=env_ids)


class CaptiveCrossbarDockingEmbodiment(YamArenaEmbodiment):
    def __init__(self, *, output_dir=None, demo_observations=False, terminate_on_success=True, **kwargs):
        self.demo_observations = bool(demo_observations)
        self.terminate_on_success = bool(terminate_on_success)
        kwargs.setdefault("task", str(Path(spec.__file__).resolve()))
        kwargs.setdefault("object_jitter", 0.)
        # Preserve the established delivered-image pipeline and native cameras.
        kwargs.setdefault("cam_width", 320)
        kwargs.setdefault("cam_height", 176)
        kwargs.setdefault("control_hz", 20.)
        if float(kwargs["control_hz"]) != 20.:
            raise ValueError("This task requires the frozen 20 Hz control interface")
        if (int(kwargs["cam_width"]), int(kwargs["cam_height"])) != (320, 176):
            raise ValueError("This task's frozen policy observations are 320 x 176")
        if float(kwargs["object_jitter"]) != 0.:
            raise ValueError("This fixed-layout task requires object_jitter=0")
        super().__init__(**kwargs)
        self._manifest = source_manifest()
        self.info = replace(self.info, environment_revision=self._manifest["revision"])
        self.output_dir = Path(output_dir).resolve() if output_dir else None
        self._trial = -1
        self._records, self._pending = [], []
        self._scorer = DockingScorer()
        self._metrics = {"score": 0., "success": False}
        self._physical_settings = {}
        self._reset_diagnostics = {}

    def _ensure_sim(self, goal):
        if self._env is not None and goal == self._goal:
            return
        # Kit must start before importing Isaac/PhysX/USD scene types.
        import yam_arena.inspect_embodiment as upstream
        if upstream._APP is None:
            from isaaclab.app import AppLauncher
            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            options = dict(headless=self.headless, device=self.device, enable_cameras=True)
            if not self.headless:
                options["visualizer"] = ["kit"]
                os.environ["DISPLAY"] = upstream.window_display(os.environ)
                if os.environ["DISPLAY"] == upstream.DESKTOP_DISPLAY:
                    options["window_width"], options["window_height"] = upstream.DESKTOP_SIZE
            upstream._APP = AppLauncher(**options).app
            atexit.register(upstream._close_app_at_exit)

        import isaaclab.sim as sim_utils
        import yam_arena.scene as scene_module
        from isaaclab.assets import ArticulationCfg
        from isaaclab.managers import EventTermCfg
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg
        original_object = scene_module.task_object
        original_callback = scene_module.env_cfg_callback

        def task_object(obj):
            if obj.name not in spec.COMPOUND_ASSETS:
                return original_object(obj)
            spawn = sim_utils.UsdFileCfg(usd_path=spec.COMPOUND_ASSETS[obj.name])
            if obj.name in spec.FIXED_ASSETS:
                spawn.rigid_props = PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)
            return Object(name=obj.name, prim_path="{ENV_REGEX_NS}/" + obj.name,
                          object_type=ObjectType.RIGID, spawner_cfg=spawn,
                          initial_pose=Pose(position_xyz=tuple(obj.pos),
                          rotation_xyzw=(0., 0., math.sin(obj.yaw / 2), math.cos(obj.yaw / 2))))

        def callback(control_hz=None):
            base_callback = original_callback(control_hz)
            def configure(cfg):
                cfg = base_callback(cfg)
                cfg.sim.physics.solver_type = 0
                cfg.sim.physics.min_position_iteration_count = spec.POSITION_ITERATIONS
                cfg.sim.physics.min_velocity_iteration_count = spec.VELOCITY_ITERATIONS
                cfg.scene.carrier = ArticulationCfg(
                    prim_path="{ENV_REGEX_NS}/carrier",
                    spawn=sim_utils.UsdFileCfg(usd_path=spec.CARRIER_ASSET),
                    init_state=ArticulationCfg.InitialStateCfg(
                        pos=spec.CARRIER_INITIAL_POS,
                        rot=(0., 0., math.sin(spec.CARRIER_INITIAL_YAW / 2), math.cos(spec.CARRIER_INITIAL_YAW / 2)),
                        joint_pos={"crossbar_slide": spec.CARRIER_INITIAL_Q},
                        joint_vel={"crossbar_slide": 0.}),
                    actuators={})
                cfg.events.reset_captive_carrier = EventTermCfg(func=_reset_carrier, mode="reset")
                return cfg
            return configure

        with patch.object(scene_module, "task_object", task_object), patch.object(scene_module, "env_cfg_callback", callback):
            super()._ensure_sim(goal)
        stage = self._base.sim.stage
        physics = stage.GetPrimAtPath(self._base.cfg.sim.physics_prim_path)
        self._physical_settings = dict(
            solver=physics.GetAttribute("physxScene:solverType").Get(),
            min_position_iterations=physics.GetAttribute("physxScene:minPositionIterationCount").Get(),
            min_velocity_iterations=physics.GetAttribute("physxScene:minVelocityIterationCount").Get())
        expected = (spec.PHYSICS_SOLVER, spec.POSITION_ITERATIONS, spec.VELOCITY_ITERATIONS)
        if tuple(self._physical_settings.values()) != expected:
            raise RuntimeError(f"Unexpected physics solver readback: {self._physical_settings}")
        self._carrier = self._base.scene["carrier"]
        if self._carrier.is_fixed_base or self._carrier.num_joints != 1:
            raise RuntimeError("Carrier must float freely with exactly one passive prismatic joint")
        self._carrier_id, self._crossbar_id = self._carrier.find_bodies(["carrier", "crossbar"], preserve_order=True)[0]
        self._joint_id = self._carrier.find_joints("crossbar_slide")[0][0]
        # Linear drive units are SI; unlike angular joints no degree conversion applies.
        for name, expected in (("joint_stiffness", 0.),
                               ("joint_damping", spec.CARRIER_DAMPING),
                               ("joint_effort_limits", spec.CARRIER_MAX_FORCE),
                               ("joint_friction_coeff", spec.CARRIER_STATIC_FRICTION)):
            value = float(_numpy(getattr(self._carrier.data, name))[0, self._joint_id])
            if not math.isclose(value, expected, rel_tol=1e-4, abs_tol=1e-8):
                raise RuntimeError(f"Unexpected slider {name}={value}; expected {expected}")
            self._physical_settings[name] = value
        limits = _numpy(self._carrier.data.joint_pos_limits)[0, self._joint_id]
        if not np.allclose(limits, spec.SLIDE_LIMITS, atol=1e-6, rtol=0.):
            raise RuntimeError(f"Unexpected slider limits: {limits}")
        self._physical_settings["joint_position_limits"] = limits.tolist()

    def _poses(self):
        links = _numpy(self._carrier.data.body_link_pose_w)[0]
        poses = {"carrier": links[self._carrier_id].tolist(), "crossbar": links[self._crossbar_id].tolist()}
        poses.update({name: _numpy(self._base.scene[name].data.root_link_pose_w)[0].tolist()
                      for name in spec.REFERENCE_NAMES})
        return poses

    def _tools(self):
        data = self._carrier.data
        return dict(**{name + "_pose": pose for name, pose in self._poses().items()},
                    joint_position=float(_numpy(data.joint_pos)[0, self._joint_id]),
                    joint_velocity=float(_numpy(data.joint_vel)[0, self._joint_id]),
                    crossbar_q=float(_numpy(data.joint_pos)[0, self._joint_id]))

    def _observation(self):
        observation = super()._observation()
        if self.demo_observations:
            return replace(observation, extra={**observation.extra, "demo_tools": self._tools()})
        return observation

    def _measure(self, observation):
        from inspect_robots_yam.packing import STATE_KEY, split
        from yam_arena import ik, station
        joints = np.asarray(observation.state[STATE_KEY], dtype=float)
        left, right = split(joints)
        tcp = {}
        for arm, q in (("left", left), ("right", right)):
            pos, rot = ik.tcp_pose(q[:6], station.BASES[arm])
            tcp[arm] = [*pos.tolist(), *ik.matrix_to_quat_xyzw(rot).tolist()]
        data = self._carrier.data
        velocities = _numpy(data.body_com_vel_w)[0]
        record = dict(t=float(self._t), step=len(self._records), poses=self._poses(),
                      velocities={"carrier": velocities[self._carrier_id].tolist(),
                                  "crossbar": velocities[self._crossbar_id].tolist()},
                      joint_position=float(_numpy(data.joint_pos)[0, self._joint_id]),
                      joint_velocity=float(_numpy(data.joint_vel)[0, self._joint_id]),
                      tcp=tcp, grippers=dict(left=float(left[6]), right=float(right[6])), joints=joints.tolist())
        def finite(value):
            if isinstance(value, dict):
                return all(finite(v) for v in value.values())
            return bool(np.isfinite(value).all())
        if not finite(record):
            raise RuntimeError("Nonfinite measured state")
        self._records.append(record)
        self._pending.append(record)
        self._metrics = self._scorer.update(record)
        return record

    def _reset_snapshot(self):
        data = self._carrier.data
        return dict(pose=_numpy(data.root_link_pose_w)[0].tolist(),
                    velocity=_numpy(data.root_com_vel_w)[0].tolist(),
                    joint_position=float(_numpy(data.joint_pos)[0, self._joint_id]),
                    joint_velocity=float(_numpy(data.joint_vel)[0, self._joint_id]),
                    arms={name: dict(position=_numpy(self._base.scene[name].data.joint_pos)[0].tolist(),
                                     velocity=_numpy(self._base.scene[name].data.joint_vel)[0].tolist())
                          for name in ("left_robot", "right_robot")})

    def _verify_reset(self):
        data = self._carrier.data
        origin = _numpy(self._base.scene.env_origins)[0]
        pose = _numpy(data.root_link_pose_w)[0]
        velocity = _numpy(data.root_com_vel_w)[0]
        q = float(_numpy(data.joint_pos)[0, self._joint_id])
        qd = float(_numpy(data.joint_vel)[0, self._joint_id])
        target_quat = np.array([0., 0., math.sin(spec.CARRIER_INITIAL_YAW / 2), math.cos(spec.CARRIER_INITIAL_YAW / 2)])
        orientation_error = 2 * math.acos(min(1., abs(float(np.dot(pose[3:], target_quat)))))
        if not (np.linalg.norm(pose[:3] - origin - spec.CARRIER_INITIAL_POS) <= .002
                and orientation_error <= math.radians(1.) and np.linalg.norm(velocity[:3]) <= .005
                and np.linalg.norm(velocity[3:]) <= math.radians(2.)
                and abs(q - spec.CARRIER_INITIAL_Q) <= .001 and abs(qd) <= .005):
            raise RuntimeError(f"Carrier reset did not settle: pose={pose}, velocity={velocity}, q={q}, qd={qd}")

    def reset(self, scene, *, seed=None):
        repeated = self._trial >= 0
        if repeated:
            self._export()
        self._trial += 1
        if self.output_dir:
            while (self.output_dir / f"trial-{self._trial:04d}").exists():
                self._trial += 1
        self._records, self._pending = [], []
        self._scorer = DockingScorer()
        self._metrics = {"score": 0., "success": False}
        observation = super().reset(scene, seed=seed)
        self._reset_diagnostics = {"inherited_settle": self._reset_snapshot()}
        if repeated:
            # Articulation.reset clears wrench/actuator buffers, not persistent
            # PhysX contacts. Give the previous dock/table contacts real free
            # substeps before the final placement, with both arms reset open.
            # Every state write here is confined to the episode reset method.
            from yam_arena.actions import reset_arms_to_default
            base = self._base
            env_ids = self._torch.arange(base.num_envs, dtype=self._torch.int32, device=base.device)
            base.action_manager.reset(env_ids)
            for name in ("left_robot", "right_robot"):
                base.scene[name].reset(env_ids)
            reset_arms_to_default(base, env_ids)
            self._carrier.reset(env_ids)
            _reset_carrier(base, env_ids, height_offset=.20)
            base.scene.write_data_to_sim()
            base.sim.forward()
            # At rest, .20 m of staging clearance exceeds the .05 m gravity
            # fall during these .10 s. No grip, table or dock contact is kept.
            for _ in range(2):
                self._env.step(self._ready)
            self._reset_diagnostics["contact_purge"] = self._reset_snapshot()
            self._carrier.reset(env_ids)
            _reset_carrier(base, env_ids)
            base.scene.write_data_to_sim()
            base.sim.forward()
            self._reset_diagnostics["fresh_placement"] = self._reset_snapshot()
            for _ in range(max(1, round(SETTLE_S / base.step_dt))):
                self._env.step(self._ready)
            for _ in range(RESET_RENDERS):
                base.sim.render()
            observation = self._observation()
        self._reset_diagnostics["final_settle"] = self._reset_snapshot()
        self._write_json("reset_audit.json", self._reset_diagnostics)
        self._verify_reset()
        self._measure(observation)
        self._export(label="setup")
        return observation

    def metrics(self, *, force=True):
        return dict(self._metrics)

    def _task_step(self):
        from inspect_robots.types import StepResult
        observation = self._observation()
        self._measure(observation)
        self._flush()
        self._write_json("metrics.json", self._metrics)
        done = bool(self._metrics["success"]) and self.terminate_on_success
        return StepResult(observation=observation, terminated=done,
                          termination_reason="success" if done else None, info=dict(self._metrics))

    def _trial_path(self, name):
        folder = self.output_dir / f"trial-{self._trial:04d}"
        folder.mkdir(parents=True, exist_ok=True)
        return folder / name

    def _flush(self):
        if self.output_dir and self._pending:
            with self._trial_path("measurements.jsonl").open("a", encoding="utf-8") as stream:
                for record in self._pending:
                    stream.write(json.dumps(record, allow_nan=False) + "\n")
        self._pending.clear()

    def _write_json(self, name, value):
        if self.output_dir:
            self._trial_path(name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    def _export(self, label="result"):
        self._flush()
        self._write_json("metrics.json", self._metrics)
        if not self.output_dir:
            return
        self._write_json("source_manifest.json", self._manifest)
        self._write_json("run_config.json", dict(
            schema=spec.TRACE_SCHEMA, demo_observations=self.demo_observations,
            terminate_on_success=self.terminate_on_success, control_hz=self.info.control_hz,
            environment_revision=self.info.environment_revision, object_jitter=self.object_jitter,
            policy_image_size=[self.cam_width, self.cam_height],
            thresholds=config_dict(), physics=self._physical_settings,
            reset_diagnostics=self._reset_diagnostics))
        if self._base is not None:
            from PIL import Image
            for name in ("top_cam", "left_cam", "right_cam"):
                rgb = _numpy(self._base.scene[name].data.output["rgb"])[0, ..., :3]
                Image.fromarray(rgb.astype(np.uint8)).save(self._trial_path(label + "_" + name + ".png"))
            # Exact delivered policy frames, distinct from native diagnostics.
            observation = self._observation()
            for name, rgb in observation.images.items():
                Image.fromarray(rgb).save(self._trial_path(label + "_policy_" + name + ".png"))

    def close(self):
        if self._trial >= 0:
            self._export()
        super().close()


def make_embodiment(**kwargs):
    return CaptiveCrossbarDockingEmbodiment(**kwargs)
