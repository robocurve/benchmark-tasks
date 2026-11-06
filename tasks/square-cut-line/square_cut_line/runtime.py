# SPDX-License-Identifier: MIT
"""Physical marker runtime for the pinned yam-arena adapter.

The adapter's private construction seams are wrapped only during first build;
all task rigid bodies and contact reporters therefore precede physics startup.
Ink is a visual mesh derived from actual filtered contacts, never a controller
target or a gripper-attached proxy. No Isaac imports occur before AppLauncher.
"""
from __future__ import annotations

import json
import math
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np

from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import task_spec as spec
from .scoring import score_strokes
from .contact import rotation_xyzw, contact_gate, lift_gate, advance_lift_hold, guide_metrics, config_dict, source_manifest






def _numpy(value):
    value = value.torch if hasattr(value, "torch") else value
    return value.detach().cpu().numpy().copy()




class SquareCutLineEmbodiment(YamArenaEmbodiment):
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
        self._physics_tick = 0
        self._strokes = []
        self._guided_strokes = []
        self._guide_stroke = None
        self._samples = []
        self._pending = []
        self._stroke = None
        self._points, self._faces = [], []
        self._face_counts = []
        self._last_ink_tick = None
        self._lift_hold_ticks = 0
        self._score_cache = None
        self._score_tick = -1
        self._score_urgent = False
        self._nib_count = self._guide_count = 0

    def _ensure_sim(self, goal):
        if self._env is not None and goal == self._goal:
            return
        # Start Kit before importing scene, sensors, PhysX or pxr modules.
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
        from isaaclab.sensors import ContactSensorCfg
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

        original_object = scene_module.task_object
        original_callback = scene_module.env_cfg_callback

        def task_object(obj):
            if obj.name in getattr(spec, "VISUAL_ONLY", ()):
                return Object(
                    name=obj.name, prim_path="{ENV_REGEX_NS}/"+obj.name,
                    object_type=ObjectType.BASE,
                    spawner_cfg=sim_utils.CuboidCfg(
                        size=tuple(obj.size), collision_props=None,
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=scene_module.linear_rgb(obj.color), roughness=.4),
                    ),
                    initial_pose=Pose(position_xyz=tuple(obj.pos),
                                     rotation_xyzw=(0, 0, math.sin(obj.yaw/2), math.cos(obj.yaw/2))),
                )
            if obj.name in spec.COMPOUND_ASSETS:
                return Object(
                    name=obj.name, prim_path="{ENV_REGEX_NS}/"+obj.name,
                    object_type=ObjectType.RIGID,
                    spawner_cfg=sim_utils.UsdFileCfg(
                        usd_path=str(spec.COMPOUND_ASSETS[obj.name]),
                        activate_contact_sensors=True,
                    ),
                    initial_pose=Pose(position_xyz=tuple(obj.pos),
                                     rotation_xyzw=(0, 0, math.sin(obj.yaw/2), math.cos(obj.yaw/2))),
                )
            built = original_object(obj)
            if obj.name == "board":
                built.object_cfg.spawn.rigid_props = PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)
                built.object_cfg.spawn.activate_contact_sensors = True
            return built

        def callback(control_hz=None):
            base_callback = original_callback(control_hz)

            def configure(cfg):
                cfg = base_callback(cfg)
                for target in ("board", "square"):
                    setattr(cfg.scene, "marker_"+target+"_contact", ContactSensorCfg(
                        prim_path="{ENV_REGEX_NS}/marker",
                        filter_prim_paths_expr=["{ENV_REGEX_NS}/"+target],
                        update_period=0.0, history_length=0, track_pose=True,
                        track_contact_points=True, max_contact_data_count_per_prim=32,
                    ))
                return cfg
            return configure

        with ExitStack() as stack:
            stack.enter_context(patch.object(scene_module, "task_object", task_object))
            stack.enter_context(patch.object(scene_module, "env_cfg_callback", callback))
            super()._ensure_sim(goal)
        for name in ("marker_board_contact", "marker_square_contact"):
            sensor = self._base.scene[name]
            count = getattr(sensor, "num_sensors", None)
            if count is None:
                count = sensor.num_bodies
            if count != 1:
                raise RuntimeError(f"{name} must bind exactly one marker rigid body")
        self._create_ink_mesh()
        # Scene updates occur after physics and before the next substep. This
        # narrow, instance-local seam pairs each contact with its current pose.
        self._scene_update = self._base.scene.update

        def update(dt, *args, **kwargs):
            self._scene_update(dt, *args, **kwargs)
            if self._drawing:
                self._sample_contact(dt)
        self._base.scene.update = update

    def _create_ink_mesh(self):
        from pxr import UsdGeom
        path = self._base.scene.env_prim_paths[0] + "/board/contact_ink"
        self._mesh = UsdGeom.Mesh.Define(self._base.sim.stage, path)
        self._mesh.CreateSubdivisionSchemeAttr("none")
        self._mesh.CreateDoubleSidedAttr(True)
        self._mesh.CreateDisplayColorAttr([(0.005, 0.005, 0.005)])
        self._commit_ink()

    def _root_pose(self, name):
        return _numpy(self._base.scene[name].data.root_link_pose_w)[0]

    def _pair(self, target):
        sensor = self._base.scene["marker_"+target+"_contact"]
        data = sensor.data
        force = getattr(data, "normal_force_matrix_w", None)
        if force is None:
            force = data.force_matrix_w
        if force is None:
            raise RuntimeError("Filtered marker contact force is unavailable")
        force = _numpy(force)
        if force.shape != (1, 1, 1, 3):
            raise RuntimeError(f"Unexpected filtered contact shape: {force.shape}")
        point = data.contact_pos_w
        if point is None:
            raise RuntimeError("Contact point tracking unavailable; refusing virtual ink")
        return force[0, 0, 0], _numpy(point)[0, 0, 0]


    def _sample_contact(self, dt):
        self._physics_tick += 1
        force, point = self._pair("board")
        guide_force, _ = self._pair("square")
        marker, board, square = (self._root_pose(n) for n in ("marker", "board", "square"))
        accepted, details = contact_gate(
            force, point, marker, board, nib_local=spec.MARKER_NIB_LOCAL,
            nib_radius=spec.MARKER_NIB_RADIUS, board_size=spec.BOARD_SIZE)
        self._lift_hold_ticks = advance_lift_hold(self._lift_hold_ticks, accepted, marker, board, spec)
        guide = guide_metrics(point, square, board, guide_force, spec) if np.isfinite(point).all() else {"guide_used": False}
        record = dict(tick=self._physics_tick, dt=float(dt), ink=accepted,
                      marker_pose=marker.tolist(), board_pose=board.tolist(), square_pose=square.tolist(),
                      pair_force=force.tolist(), pair_point=point.tolist(),
                      guide_pair_force=guide_force.tolist(), **details, **guide)
        self._samples.append(record)
        self._pending.append(record)
        if not accepted:
            if self._stroke is not None:
                self._score_urgent = True
            self._stroke = None
            self._guide_stroke = None
            return
        # Scorer frame: board-relative, centred on the target midpoint.
        board_r = rotation_xyzw(board[3:])
        target_mid = (np.asarray(spec.TARGET_START)+np.asarray(spec.TARGET_END))/2
        target_mid_w = np.array([*target_mid, spec.BOARD_TOP])
        target_mid_b = board_r.T @ (target_mid_w-board[:3])
        point_b = np.asarray(details["point_board"])
        point_score = (point_b[:2]-target_mid_b[:2]).tolist()
        if self._stroke is None:
            self._stroke = []
            self._strokes.append(self._stroke)
        previous = self._stroke[-1] if self._stroke else None
        # Do not invent ink across a physically implausible sample jump.
        if previous is not None and np.linalg.norm(np.asarray(point_score)-previous) > .01:
            self._stroke = []
            self._strokes.append(self._stroke)
            previous = None
            self._guide_stroke = None
        self._stroke.append(point_score)
        self._last_ink_tick = self._physics_tick
        self._nib_count += 1
        self._guide_count += int(guide.get("guide_used", False))
        if guide.get("guide_used", False):
            if self._guide_stroke is None:
                self._guide_stroke = []
                self._guided_strokes.append(self._guide_stroke)
            self._guide_stroke.append(point_score)
        else:
            self._guide_stroke = None
        record["point_score"] = point_score
        self._add_ink(point_b[:2], None if previous is None else np.asarray(previous)+target_mid_b[:2])

    def _add_ink(self, end, start):
        end = np.asarray(end)
        radius = float(spec.INK_WIDTH)/2

        def polygon(points):
            index = len(self._points)
            self._points.extend((float(x), float(y), spec.BOARD_SIZE[2]/2+.00015) for x, y in points)
            self._faces.extend(range(index, index+len(points)))
            self._face_counts.append(len(points))

        if start is not None and np.linalg.norm(end-start) >= 1e-8:
            direction = end-start
            normal = np.array([-direction[1], direction[0]]) / np.linalg.norm(direction)*radius
            polygon(np.array([start-normal, end-normal, end+normal, start+normal]))
        # Round end caps match the scorer's 2 mm diameter capsule footprint.
        angles = np.arange(24)*2*math.pi/24
        polygon(end+radius*np.column_stack((np.cos(angles), np.sin(angles))))
        self._ink_dirty = True

    def _commit_ink(self):
        if self._mesh is None:
            return
        self._mesh.GetPointsAttr().Set(self._points)
        self._mesh.GetFaceVertexCountsAttr().Set(self._face_counts)
        self._mesh.GetFaceVertexIndicesAttr().Set(self._faces)
        self._ink_dirty = True

    def _observation(self):
        if self._ink_dirty and self._base is not None:
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
        if self.demo_observations:
            force, _ = self._pair("board")
            tools = {name+"_pose": self._root_pose(name).tolist() for name in ("marker", "square", "board")}
            tools["marker_board_force"] = force.tolist()
            tools["nib_contact"] = bool(self._samples and self._samples[-1]["ink"])
            return replace(observation, extra={**observation.extra, "demo_tools": tools})
        return observation

    def reset(self, scene, *, seed=None):
        if self._trial >= 0:
            self._export()
        self._drawing = False
        self._trial += 1
        if self.output_dir:
            while (self.output_dir/f"trial-{self._trial:04d}").exists():
                self._trial += 1
        self._physics_tick = 0
        self._strokes, self._samples, self._pending = [], [], []
        self._guided_strokes = []
        self._guide_stroke = None
        self._stroke = None
        self._points, self._faces = [], []
        self._face_counts = []
        self._last_ink_tick = None
        self._lift_hold_ticks = 0
        self._score_cache = None
        self._score_tick = -1
        self._score_urgent = False
        self._nib_count = self._guide_count = 0
        self._commit_ink()
        observation = super().reset(scene, seed=seed)
        self._drawing = True
        self._export()
        return observation

    def metrics(self, *, force=False):
        # Geometry scoring is bounded to 2 Hz during a stroke, plus contact loss
        # and final export. Physics contact sampling remains at 240 Hz.
        due = self._physics_tick-self._score_tick >= round(.5/self._base.physics_dt) if self._base is not None else True
        if force or self._score_cache is None or self._score_urgent or due:
            target_start, target_end = self._target_local()
            self._score_cache = score_strokes(self._strokes, target_start, target_end)
            self._score_cache["unique_guided_ink_length"] = score_strokes(self._guided_strokes, target_start, target_end)["unique_ink_length"]
            self._score_tick = self._physics_tick
            self._score_urgent = False
        metrics = dict(self._score_cache)
        guided_length = metrics["unique_guided_ink_length"]
        ink_length = metrics["unique_ink_length"]
        guided_fraction = min(1.0, guided_length/ink_length) if ink_length else 0.0
        metrics.update(physics_samples=len(self._samples), nib_contact_samples=self._nib_count,
                       geometry_evaluated_tick=self._score_tick,
                       guide_used_samples=self._guide_count, guide_used_fraction=guided_fraction,
                       unique_guided_ink_length=guided_length,
                       geometry_score=metrics["score"])
        if getattr(spec, "REQUIRE_GUIDE_USE", True):
            if guided_fraction < .5:
                metrics["score"] = 0.0
            elif guided_fraction < .9:
                metrics["score"] = min(.5, metrics["score"])
        quiet_s = ((self._physics_tick-self._last_ink_tick)*self._base.physics_dt
                   if self._last_ink_tick is not None and self._base is not None else 0.0)
        lifted, lift = lift_gate(
            self._root_pose("marker"), self._root_pose("board"), quiet_s,
            nib_local=spec.MARKER_NIB_LOCAL, nib_radius=spec.MARKER_NIB_RADIUS,
            board_size=spec.BOARD_SIZE, clear_s=self._lift_hold_ticks*self._base.physics_dt) if self._base is not None else (False, {})
        metrics.update(lift, marker_lift_complete=lifted)
        if not lifted:
            metrics["score"] = min(.5, metrics["score"])
        metrics["success"] = metrics["score"] >= 1.0
        return metrics

    def _target_local(self):
        mid = (np.asarray(spec.TARGET_START)+np.asarray(spec.TARGET_END))/2
        rotation = rotation_xyzw(self._root_pose("board")[3:]) if self._base is not None else np.eye(3)
        return tuple((rotation.T @ np.array([*(np.asarray(point)-mid), 0.0]))[:2].tolist()
                     for point in (spec.TARGET_START, spec.TARGET_END))

    def _task_step(self):
        from inspect_robots.types import StepResult
        metrics = self.metrics()
        self._flush()
        self._write_metrics(metrics)
        return StepResult(observation=self._observation(),
                          terminated=metrics["success"] and self.terminate_on_success,
                          termination_reason="success" if metrics["success"] and self.terminate_on_success else None,
                          info=metrics)

    def _trial_path(self, name):
        folder = self.output_dir / f"trial-{self._trial:04d}"
        folder.mkdir(parents=True, exist_ok=True)
        return folder/name

    def _flush(self):
        if self.output_dir:
            with self._trial_path("contacts.jsonl").open("a", encoding="utf-8") as stream:
                for record in self._pending:
                    # Noncontacts can carry PhysX NaN points; standard JSON uses null.
                    clean = _json_clean(record)
                    stream.write(json.dumps(clean, allow_nan=False)+"\n")
        self._pending.clear()

    def _write_metrics(self, metrics):
        if self.output_dir:
            self._trial_path("metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False)+"\n", encoding="utf-8")

    def _export(self):
        self._flush()
        self._write_metrics(self.metrics(force=True))
        if self.output_dir:
            self._trial_path("run_config.json").write_text(json.dumps({
                "demo_observations": self.demo_observations,
                "terminate_on_success": self.terminate_on_success,
                "control_hz": self.info.control_hz,
                "environment_revision": self.info.environment_revision,
            }, indent=2)+"\n", encoding="utf-8")
            if self._base is not None:
                from PIL import Image
                label = "setup" if self._physics_tick == 0 else "result"
                for camera_name in ("top_cam", "left_cam", "right_cam"):
                    rgb = _numpy(self._base.scene[camera_name].data.output["rgb"])[0, ..., :3]
                    Image.fromarray(rgb.astype(np.uint8)).save(self._trial_path(label+"_"+camera_name+".png"))
            target_start, target_end = self._target_local()
            manifest_text = json.dumps(self._source_manifest, indent=2)+"\n"
            self._trial_path("source_manifest.json").write_text(manifest_text, encoding="utf-8")
            manifest_path = self.output_dir/"source_manifest.json"
            if manifest_path.exists() and manifest_path.read_text(encoding="utf-8") != manifest_text:
                manifest_path = self.output_dir/("source_manifest-"+self._source_manifest["revision"]+".json")
            manifest_path.write_text(manifest_text, encoding="utf-8")
            self._trial_path("ink.json").write_text(json.dumps(dict(
                schema="square-cut-line-contact-ink-v1", strokes=self._strokes,
                guided_strokes=self._guided_strokes,
                target_start=target_start, target_end=target_end,
                board_center=spec.BOARD_CENTER, board_size=spec.BOARD_SIZE,
                marker_nib_local=spec.MARKER_NIB_LOCAL, marker_nib_radius=spec.MARKER_NIB_RADIUS,
                contact_config=config_dict(spec),
                environment_revision=self._source_manifest["revision"],
                source="filtered physical marker-board contacts",
            ), indent=2, allow_nan=False)+"\n", encoding="utf-8")

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
    return SquareCutLineEmbodiment(**kwargs)
