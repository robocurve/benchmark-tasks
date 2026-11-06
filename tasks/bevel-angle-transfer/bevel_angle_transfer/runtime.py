# SPDX-License-Identifier: MIT
"""Physical bevel-gauge and marker runtime for the pinned YAM Arena adapter."""

from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np
from yam_arena.constants import TCP_OFFSET
from yam_arena.inspect_embodiment import RESET_RENDERS, SETTLE_S, YamArenaEmbodiment

from . import task_spec as spec
from .contact import config_dict, rotation_xyzw, source_manifest
from .scoring import TransferTracker


# Each filtered reporter observes exactly one pair of rigid bodies.
_PAIR_PATHS = {
    "marker_board": ("marker", "board"),
    "marker_blade": ("marker", "gauge/blade"),
    "stock_board": ("gauge/stock", "board"),
    "stock_reference": ("gauge/stock", "reference_stock"),
    "blade_reference": ("gauge/blade", "reference_blade"),
}
_TOOL_POSES = ("marker", "board", "reference_stock", "reference_blade")
_FINGER_NAMES = ("tip_left", "tip_right")
_MIN_POSITION_ITERATIONS = 64
_MIN_VELOCITY_ITERATIONS = 16


def _numpy(value):
    value = value.torch if hasattr(value, "torch") else value
    return value.detach().cpu().numpy().copy()


def _reset_gauge(env, env_ids):
    """Initialize free pose and hinge state once per episode, before settling."""
    gauge = env.scene["gauge"]
    origins = env.scene.env_origins[env_ids]
    half_yaw = spec.GAUGE_INITIAL_YAW / 2
    pose = origins.new_tensor([*spec.GAUGE_INITIAL_POS, 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)])
    pose = pose.repeat(len(env_ids), 1)
    pose[:, :3] += origins
    gauge.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    gauge.write_root_velocity_to_sim_index(root_velocity=origins.new_zeros((len(env_ids), 6)), env_ids=env_ids)
    gauge.write_joint_position_to_sim_index(
        position=origins.new_full((len(env_ids), 1), spec.GAUGE_INITIAL_ANGLE), env_ids=env_ids,
    )
    gauge.write_joint_velocity_to_sim_index(velocity=origins.new_zeros((len(env_ids), 1)), env_ids=env_ids)


class BevelAngleTransferEmbodiment(YamArenaEmbodiment):
    def __init__(self, *, output_dir=None, demo_observations=False, terminate_on_success=True, **kwargs):
        self.demo_observations = bool(demo_observations)
        self.terminate_on_success = bool(terminate_on_success)
        kwargs.setdefault("task", str(Path(spec.__file__).resolve()))
        kwargs.setdefault("object_jitter", 0.0)
        super().__init__(**kwargs)
        self._source_manifest = source_manifest()
        self.info = replace(self.info, environment_revision=self._source_manifest["revision"])
        self.output_dir = Path(output_dir).resolve() if output_dir else None
        self._trial = -1
        self._drawing = False
        self._mesh = None
        self._ink_dirty = False

    def _ensure_sim(self, goal):
        if self._env is not None and goal == self._goal:
            return
        # Kit must start before imports of USD and scene types.
        import atexit
        import os
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
        from isaaclab.sensors import ContactSensorCfg
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg
        from yam_arena.robot import asset_link_paths

        original_object = scene_module.task_object
        original_callback = scene_module.env_cfg_callback
        link_paths = asset_link_paths()
        pair_paths = dict(_PAIR_PATHS)
        for name in _FINGER_NAMES:
            pair_paths[f"blade_{name}"] = ("gauge/blade", "RightArm" + link_paths[name])

        def task_object(obj):
            rotation = (0.0, 0.0, math.sin(obj.yaw / 2), math.cos(obj.yaw / 2))
            pose = Pose(position_xyz=tuple(obj.pos), rotation_xyzw=rotation)
            if obj.name in spec.VISUAL_ONLY:
                return Object(
                    name=obj.name, prim_path="{ENV_REGEX_NS}/" + obj.name,
                    object_type=ObjectType.BASE, initial_pose=pose,
                    spawner_cfg=sim_utils.CuboidCfg(
                        size=tuple(obj.size), collision_props=None,
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=scene_module.linear_rgb(obj.color), roughness=.4),
                    ),
                )
            if obj.name in spec.COMPOUND_ASSETS:
                asset = Object(
                    name=obj.name, prim_path="{ENV_REGEX_NS}/" + obj.name,
                    object_type=ObjectType.RIGID, initial_pose=pose,
                    spawner_cfg=sim_utils.UsdFileCfg(
                        usd_path=str(spec.COMPOUND_ASSETS[obj.name]),
                        activate_contact_sensors=True,
                    ),
                )
            else:
                asset = original_object(obj)
            if obj.name in ("board", "reference_stock", "reference_blade"):
                spawn = asset.object_cfg.spawn
                spawn.rigid_props = PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)
                spawn.activate_contact_sensors = True
                if obj.name != "board":
                    spawn.collision_props = PhysxCollisionPropertiesCfg(contact_offset=.001, rest_offset=0.0)
                    asset = Object(
                        name=obj.name, prim_path="{ENV_REGEX_NS}/" + obj.name,
                        object_type=ObjectType.RIGID, initial_pose=pose, spawner_cfg=spawn,
                    )
            return asset

        def callback(control_hz=None):
            base_callback = original_callback(control_hz)

            def configure(cfg):
                cfg = base_callback(cfg)
                cfg.sim.physics.solver_type = 0
                cfg.sim.physics.min_position_iteration_count = _MIN_POSITION_ITERATIONS
                cfg.sim.physics.min_velocity_iteration_count = _MIN_VELOCITY_ITERATIONS
                cfg.scene.gauge = ArticulationCfg(
                    prim_path="{ENV_REGEX_NS}/gauge",
                    spawn=sim_utils.UsdFileCfg(
                        usd_path=spec.GAUGE_ASSET,
                        activate_contact_sensors=True,
                    ),
                    init_state=ArticulationCfg.InitialStateCfg(
                        pos=spec.GAUGE_INITIAL_POS,
                        rot=(0.0, 0.0, math.sin(spec.GAUGE_INITIAL_YAW / 2), math.cos(spec.GAUGE_INITIAL_YAW / 2)),
                        joint_pos={"hinge": spec.GAUGE_INITIAL_ANGLE},
                        joint_vel={"hinge": 0.0},
                    ),
                    actuators={},
                )
                cfg.events.reset_bevel_gauge = EventTermCfg(func=_reset_gauge, mode="reset")
                for name, (source, target) in pair_paths.items():
                    setattr(cfg.scene, name + "_contact", ContactSensorCfg(
                        prim_path="{ENV_REGEX_NS}/" + source,
                        filter_prim_paths_expr=["{ENV_REGEX_NS}/" + target],
                        update_period=0.0, history_length=0, track_contact_points=True,
                        max_contact_data_count_per_prim=32,
                    ))
                return cfg

            return configure

        with patch.object(scene_module, "task_object", task_object), patch.object(scene_module, "env_cfg_callback", callback):
            super()._ensure_sim(goal)
        physics = self._base.sim.stage.GetPrimAtPath(self._base.cfg.sim.physics_prim_path)
        self._physics_solver = physics.GetAttribute("physxScene:solverType").Get()
        if self._physics_solver != "PGS":
            raise RuntimeError(f"Expected PhysX PGS solver, got {self._physics_solver!r}")
        self._physics_iterations = {
            "min_position_iteration_count": physics.GetAttribute("physxScene:minPositionIterationCount").Get(),
            "min_velocity_iteration_count": physics.GetAttribute("physxScene:minVelocityIterationCount").Get(),
        }
        expected_iterations = (_MIN_POSITION_ITERATIONS, _MIN_VELOCITY_ITERATIONS)
        if tuple(self._physics_iterations.values()) != expected_iterations:
            raise RuntimeError(f"Unexpected PhysX solver iteration counts: {self._physics_iterations}")
        scene = self._base.scene
        self._sensors = {name: scene[name + "_contact"] for name in pair_paths}
        for name, sensor in self._sensors.items():
            count = getattr(sensor, "num_sensors", None)
            if count is None:
                count = sensor.num_bodies
            if count != 1:
                raise RuntimeError(f"{name} contact sensor must bind exactly one rigid body")
        self._gauge = scene["gauge"]
        if self._gauge.is_fixed_base or self._gauge.num_joints != 1:
            raise RuntimeError("The bevel gauge must have one hinge on a floating articulation")
        self._stock_id, self._blade_id = self._gauge.find_bodies(["stock", "blade"], preserve_order=True)[0]
        self._hinge_id = self._gauge.find_joints("hinge")[0][0]
        static_friction = float(_numpy(self._gauge.data.joint_friction_coeff)[0, self._hinge_id])
        if not math.isclose(static_friction, spec.GAUGE_STATIC_FRICTION, rel_tol=1e-4):
            raise RuntimeError(
                f"PhysX parsed hinge static friction {static_friction}, expected {spec.GAUGE_STATIC_FRICTION} N m"
            )
        # USD angular gains are per degree; PhysX exposes the solver gain per radian.
        for name, expected in (
            ("joint_stiffness", 0.0),
            ("joint_damping", math.degrees(spec.GAUGE_DAMPER_PER_DEGREE)),
            ("joint_effort_limits", spec.GAUGE_DAMPER_MAX_TORQUE),
        ):
            value = float(_numpy(getattr(self._gauge.data, name))[0, self._hinge_id])
            if not math.isclose(value, expected, rel_tol=1e-4, abs_tol=1e-8):
                raise RuntimeError(f"PhysX parsed hinge {name}={value}, expected {expected}")
        self._right = scene["right_robot"]
        self._right_ids = self._right.find_bodies(["gripper", *_FINGER_NAMES], preserve_order=True)[0]

        self._create_ink_mesh()
        # Sample contact and pose buffers from the same physics substep.
        self._scene_update = scene.update

        def update(dt, *args, **kwargs):
            self._scene_update(dt, *args, **kwargs)
            if self._drawing:
                self._sample_contact(dt)

        scene.update = update

    def _root_pose(self, name):
        return _numpy(self._base.scene[name].data.root_link_pose_w)[0]

    def _poses(self):
        poses = {name + "_pose": self._root_pose(name).tolist() for name in _TOOL_POSES}
        links = _numpy(self._gauge.data.body_link_pose_w)[0]
        right_gripper, *right_fingers = _numpy(self._right.data.body_link_pose_w)[0, self._right_ids]
        return dict(
            poses, stock_pose=links[self._stock_id].tolist(), blade_pose=links[self._blade_id].tolist(),
            gauge_angle=float(_numpy(self._gauge.data.joint_pos)[0, self._hinge_id]),
            gauge_angle_velocity=float(_numpy(self._gauge.data.joint_vel)[0, self._hinge_id]),
            right_gripper_pose=right_gripper.tolist(),
            right_finger_poses=[finger.tolist() for finger in right_fingers],
            right_tcp_position=(right_gripper[:3] + rotation_xyzw(right_gripper[3:]) @ TCP_OFFSET).tolist(),
        )

    def _pair(self, name):
        data = self._sensors[name].data
        force = getattr(data, "normal_force_matrix_w", None)
        if force is None:
            force = data.force_matrix_w
        if force is None or data.contact_pos_w is None:
            raise RuntimeError(f"{name} filtered physical force or contact point unavailable")
        force = _numpy(force)
        if force.shape != (1, 1, 1, 3):
            raise RuntimeError(f"Unexpected {name} contact shape: {force.shape}")
        return force[0, 0, 0].tolist(), _numpy(data.contact_pos_w)[0, 0, 0].tolist()

    def _sample_contact(self, dt):
        self._physics_tick += 1
        force, point = self._pair("marker_board")
        ref_stock_force, ref_stock_point = self._pair("stock_reference")
        ref_blade_force, ref_blade_point = self._pair("blade_reference")
        raw = dict(
            self._poses(), tick=self._physics_tick, dt=float(dt),
            pair_force=force, pair_point=point,
            guide_pair_force=self._pair("marker_blade")[0],
            stock_board_force=self._pair("stock_board")[0],
            reference_stock_force=ref_stock_force, reference_stock_point=ref_stock_point,
            reference_blade_force=ref_blade_force, reference_blade_point=ref_blade_point,
            right_finger_blade_forces=[self._pair(f"blade_{name}")[0] for name in _FINGER_NAMES],
        )
        record = self._tracker.sample(raw)
        self._last_sample = record
        if self.output_dir:
            self._pending.append(record)
        if record["ink"]:
            point_board = np.asarray(record["point_board"][:2])
            previous = None if record["ink_segment_start"] else self._last_ink_point
            self._add_ink(point_board, previous)
            self._last_ink_point = point_board

    def _create_ink_mesh(self):
        from pxr import UsdGeom

        path = self._base.scene.env_prim_paths[0] + "/board/contact_ink"
        self._mesh = UsdGeom.Mesh.Define(self._base.sim.stage, path)
        self._mesh.CreateSubdivisionSchemeAttr("none")
        self._mesh.CreateDoubleSidedAttr(True)
        self._mesh.CreateDisplayColorAttr([(0.005, 0.005, 0.005)])
        self._commit_ink()

    def _add_ink(self, end, start):
        radius = spec.INK_WIDTH / 2

        def polygon(points):
            index = len(self._points)
            self._points.extend((float(x), float(y), spec.BOARD_SIZE[2] / 2 + .00015) for x, y in points)
            self._faces.extend(range(index, index + len(points)))
            self._face_counts.append(len(points))

        if start is not None and np.linalg.norm(end - start) >= 1e-8:
            direction = end - start
            normal = np.array([-direction[1], direction[0]]) / np.linalg.norm(direction) * radius
            polygon(np.array([start - normal, end - normal, end + normal, start + normal]))
        angles = np.arange(24) * 2 * math.pi / 24
        polygon(end + radius * np.column_stack((np.cos(angles), np.sin(angles))))
        self._ink_dirty = True

    def _commit_ink(self):
        if self._mesh is not None:
            self._mesh.GetPointsAttr().Set(self._points)
            self._mesh.GetFaceVertexCountsAttr().Set(self._face_counts)
            self._mesh.GetFaceVertexIndicesAttr().Set(self._faces)

    def _observation(self):
        if self._ink_dirty:
            self._commit_ink()
            self._base.sim.render()
            from inspect_robots_yam.config import DEFAULT_CAMERAS

            for name in DEFAULT_CAMERAS:
                camera = self._base.scene[name]
                if hasattr(camera, "_invalidate_buffers"):
                    camera._invalidate_buffers()
                    camera.update(dt=0.0, force_recompute=True)
                else:
                    camera.update(dt=self._base.physics_dt, force_recompute=True)
            self._ink_dirty = False
        observation = super()._observation()
        if not self.demo_observations:
            return observation
        tools = self._poses()
        tools["gauge_velocity"] = tools["gauge_angle_velocity"]
        tools["marker_board_force"], _ = self._pair("marker_board")
        tools["nib_contact"] = bool(self._last_sample and self._last_sample["ink"])
        for key in ("capture_valid", "released", "transfer_valid", "guide_used"):
            tools[key] = bool(self._last_sample and self._last_sample.get(key, False))
        tools["reference_stock_force"] = self._pair("stock_reference")[0]
        tools["reference_blade_force"] = self._pair("blade_reference")[0]
        tools["right_finger_blade_forces"] = [self._pair(f"blade_{name}")[0] for name in _FINGER_NAMES]
        tools["stock_board_force"] = self._pair("stock_board")[0]
        return replace(observation, extra={**observation.extra, "demo_tools": tools})

    def reset(self, scene, *, seed=None):
        repeated = self._trial >= 0
        if repeated:
            self._export()
        self._drawing = False
        self._trial += 1
        if self.output_dir:
            while (self.output_dir / f"trial-{self._trial:04d}").exists():
                self._trial += 1
        self._tracker = TransferTracker(spec)
        self._physics_tick = 0
        self._pending = []
        self._last_sample = None
        self._last_ink_point = None
        self._points, self._faces, self._face_counts = [], [], []
        self._commit_ink()
        observation = super().reset(scene, seed=seed)
        if repeated:
            # The previous trial can end with the gauge pinched. First let the
            # base reset open and settle the arms, then place the free gauge.
            base = self._base
            env_ids = self._torch.arange(base.num_envs, dtype=self._torch.int32, device=base.device)
            self._gauge.reset(env_ids)
            _reset_gauge(base, env_ids)
            base.scene.write_data_to_sim()
            base.sim.forward()
            for _ in range(max(1, round(SETTLE_S / base.step_dt))):
                self._env.step(self._ready)
            for _ in range(RESET_RENDERS):
                base.sim.render()
            observation = self._observation()
        self._verify_gauge_reset()
        self._drawing = True
        self._export()
        return observation

    def _verify_gauge_reset(self):
        data = self._gauge.data
        origin = _numpy(self._base.scene.env_origins)[0]
        position = _numpy(data.root_link_pose_w)[0, :3] - origin
        root_velocity = _numpy(data.root_com_vel_w)[0]
        angle = float(_numpy(data.joint_pos)[0, self._hinge_id])
        velocity = float(_numpy(data.joint_vel)[0, self._hinge_id])
        if not (
            np.linalg.norm(position - spec.GAUGE_INITIAL_POS) <= .001
            and np.linalg.norm(root_velocity[:3]) <= .002
            and np.linalg.norm(root_velocity[3:]) <= math.radians(1)
            and abs(angle - spec.GAUGE_INITIAL_ANGLE) <= math.radians(.25)
            and abs(velocity) <= math.radians(1)
        ):
            raise RuntimeError(
                f"Gauge failed to settle at reset: position={position}, "
                f"root_velocity={root_velocity}, hinge={angle}, hinge_velocity={velocity}"
            )

    def metrics(self, *, force=True):
        return self._tracker.metrics(force=force)

    def _task_step(self):
        from inspect_robots.types import StepResult

        metrics = self.metrics(force=True)
        terminated = metrics["success"] and self.terminate_on_success
        self._flush()
        self._write_json("metrics.json", metrics)
        return StepResult(
            observation=self._observation(), terminated=terminated,
            termination_reason="success" if terminated else None, info=metrics,
        )

    def _trial_path(self, name):
        folder = self.output_dir / f"trial-{self._trial:04d}"
        folder.mkdir(parents=True, exist_ok=True)
        return folder / name

    def _flush(self):
        if self.output_dir:
            with self._trial_path("contacts.jsonl").open("a", encoding="utf-8") as stream:
                for record in self._pending:
                    stream.write(json.dumps(_json_clean(record), allow_nan=False) + "\n")
        self._pending.clear()

    def _write_json(self, filename, data):
        if self.output_dir:
            self._trial_path(filename).write_text(
                json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8",
            )

    def _export(self):
        if not self.output_dir:
            return
        self._flush()
        self._write_json("metrics.json", self.metrics())
        self._write_json("run_config.json", {
            "demo_observations": self.demo_observations,
            "terminate_on_success": self.terminate_on_success,
            "control_hz": self.info.control_hz,
            "physics_solver": self._physics_solver,
            "physics_iterations": self._physics_iterations,
            "environment_revision": self.info.environment_revision,
        })
        if self._base is not None:
            from PIL import Image

            label = "setup" if self._physics_tick == 0 else "result"
            for name in ("top_cam", "left_cam", "right_cam"):
                rgb = _numpy(self._base.scene[name].data.output["rgb"])[0, ..., :3]
                Image.fromarray(rgb.astype(np.uint8)).save(self._trial_path(f"{label}_{name}.png"))
        self._write_json("source_manifest.json", self._source_manifest)
        self._write_json("ink.json", dict(
            schema="bevel-angle-transfer-contact-ink-v1",
            strokes=self._tracker.strokes, guided_strokes=self._tracker.guided_strokes,
            target_start=self._tracker.target_start, target_end=self._tracker.target_end,
            contact_config=config_dict(spec), environment_revision=self.info.environment_revision,
            source="filtered physical marker-board contacts",
        ))

    def close(self):
        self._drawing = False
        if self._trial >= 0:
            self._export()
        if self._base is not None and hasattr(self, "_scene_update"):
            self._base.scene.update = self._scene_update
            del self._scene_update
        self._mesh = None
        super().close()


def _json_clean(value):
    if isinstance(value, dict):
        return {key: _json_clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_clean(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def make_embodiment(**kwargs):
    return BevelAngleTransferEmbodiment(**kwargs)
