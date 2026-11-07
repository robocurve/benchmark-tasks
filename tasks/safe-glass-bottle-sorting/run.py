#!/usr/bin/env python3
"""Instrument the stock YAM runner in memory; never changes installed packages.

Run inside the supplied Isaac Lab-Arena environment. All remaining arguments are
passed to `inspect-robots`. See README.md for the reproducible commands.
"""

import argparse
import atexit
import json
import uuid
from dataclasses import replace
from pathlib import Path

import numpy as np


def install(trace_dir=None):
    import yam_arena.inspect_embodiment as ie
    from yam_arena import scripted
    from yam_arena.task_file import load

    # Inspect Robots can construct policy plans before starting the simulator.
    scripted.ArmPlan = load(Path(__file__).with_name("task.py")).module.RackArmPlan

    original_ensure = ie.YamArenaEmbodiment._ensure_sim
    original_reset = ie.YamArenaEmbodiment.reset
    original_task_step = ie.YamArenaEmbodiment._task_step
    installed = False

    def ensure(self, goal):
        nonlocal installed
        if ie._APP is None:
            import os

            from isaaclab.app import AppLauncher

            opts = {
                "headless": self.headless,
                "device": self.device,
                "enable_cameras": True,
            }
            if not self.headless:
                os.environ["DISPLAY"] = ie.window_display(os.environ)
                opts.update(visualizer=["kit"], window_width=1920, window_height=1080)
            ie._APP = AppLauncher(**opts).app
            atexit.register(ie._close_app_at_exit)
        if not installed:
            install_scene()
            installed = True
        # The rack requires full quaternion targets rather than stock top-down yaw.
        from yam_arena import scripted

        scripted.ArmPlan = self.task.module.RackArmPlan
        original_ensure(self, goal)

    def reset(self, scene, *, seed=None):
        module = self.task.module
        module.MONITOR = module.SafetyMonitor()
        obs = original_reset(self, scene, seed=seed)
        base = self._base
        if not getattr(base.scene, "_bottle_monitor_installed", False):
            original_update = base.scene.update

            def update(dt):
                original_update(dt)
                monitor = self.task.module.MONITOR
                if not monitor.armed:
                    return
                from yam_arena.scene import object_states

                states = object_states(self._env, module.NAMES)
                forces = {}
                for name in module.NAMES:
                    data = base.scene[f"{name}_contacts"].data.net_normal_forces_w
                    data = data.torch if hasattr(data, "torch") else data
                    forces[name] = float(data[0].norm(dim=-1).max().item())
                monitor.update(states, forces)

            base.scene.update = update
            base.scene._bottle_monitor_installed = True
        module.MONITOR.armed = True
        self._bottle_episode = uuid.uuid4().hex
        self._bottle_trace = None
        if trace_dir is not None:
            trace_dir.mkdir(parents=True, exist_ok=True)
            self._bottle_trace = trace_dir / "bottle-safety.jsonl"
        return obs

    def task_step(self):
        from yam_arena.constants import FINGER_OPEN
        from yam_arena.runtime import tcp_world
        from yam_arena.scene import object_states

        module = self.task.module
        states = object_states(self._env, module.NAMES)
        released = True
        tcp_positions = {}
        for arm in ie.ARMS:
            robot = self._base.scene[arm]
            opening = (
                robot.data.joint_pos.torch[0, self._ids[arm][1]].mean().item()
                / FINGER_OPEN
            )
            tcp = tcp_world(robot)
            tcp_positions[arm] = tcp.tolist()
            released &= opening >= 0.75 and all(
                np.linalg.norm(tcp - states[n].pos) >= 0.085 for n in module.NAMES
            )
        module.MONITOR.observe_control(states, self._t, bool(released))
        result = original_task_step(self)
        info = dict(result.info)
        info.update(
            safety_violations=sorted(module.MONITOR.reasons),
            peak_force_n=dict(module.MONITOR.peak_force_n),
            max_tilt_deg=dict(module.MONITOR.max_tilt_deg),
            grippers_clear=bool(released),
            stable_window=module.MONITOR.stable,
            tcp_positions=tcp_positions,
            sorted_count=sum(module.sorted_bottle(n, states[n]) for n in module.NAMES),
        )
        if self._bottle_trace is not None:
            record = {
                "episode": self._bottle_episode,
                "time_s": self._t,
                "task_revision": self.task.sha256,
                **info,
            }
            record["bottles"] = {
                name: {
                    "pos": states[name].pos.tolist(),
                    "quat_xyzw": states[name].quat_xyzw.tolist(),
                    "speed_m_s": float(np.linalg.norm(states[name].lin_vel)),
                    "spin_rad_s": float(np.linalg.norm(states[name].ang_vel)),
                }
                for name in module.NAMES
            }
            with self._bottle_trace.open("a") as stream:
                stream.write(json.dumps(record) + "\n")
        # Fail immediately; later repairs cannot conceal an unsafe event.
        unsafe = bool(module.MONITOR.reasons)
        return replace(
            result,
            info=info,
            terminated=unsafe or result.terminated,
            termination_reason="unsafe" if unsafe else result.termination_reason,
        )

    ie.YamArenaEmbodiment._ensure_sim = ensure
    ie.YamArenaEmbodiment.reset = reset
    ie.YamArenaEmbodiment._task_step = task_step


def install_scene():
    from isaaclab import sim
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import (
        PhysxCollisionPropertiesCfg,
        PhysxRigidBodyPropertiesCfg,
    )
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade
    from yam_arena import scene

    original_object = scene.task_object
    original_callback = scene.env_cfg_callback
    names = (
        "blue_swap",
        "red_left",
        "red_right",
        "red_swap",
        "blue_left",
        "blue_right",
    )

    @clone
    def spawn_bottle(prim_path, cfg, translation=None, orientation=None, **kwargs):
        # A single rigid body with three joined convex cylinder colliders and
        # matching visual geometry: body, simplified shoulder, and narrow neck.
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        root = UsdGeom.Xform.Define(stage, prim_path)
        root.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0, 0, 0))))
        if orientation:
            root.AddOrientOp().Set(Gf.Quatf(orientation[0], Gf.Vec3f(*orientation[1:])))
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(root.GetPrim())
        mass.CreateMassAttr(0.08)
        PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim()).CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(root.GetPrim()).CreateThresholdAttr(0.0)
        material = UsdShade.Material.Define(stage, prim_path + "/Glass")
        shader = UsdShade.Shader.Define(stage, prim_path + "/Glass/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*cfg.visual_material.diffuse_color)
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.18)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr(0.65)
        physics.CreateDynamicFrictionAttr(0.5)
        physics.CreateRestitutionAttr(0.0)
        for name, radius, height, z in [
            ("body", 0.018, 0.070, -0.015),
            ("shoulder", 0.015, 0.012, 0.026),
            ("neck", 0.009, 0.030, 0.035),
        ]:
            shape = UsdGeom.Cylinder.Define(stage, prim_path + "/" + name)
            shape.CreateRadiusAttr(radius)
            shape.CreateHeightAttr(height)
            shape.CreateAxisAttr("Z")
            shape.AddTranslateOp().Set(Gf.Vec3d(0, 0, z))
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            collision = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
            collision.CreateContactOffsetAttr(0.001)
            collision.CreateRestOffsetAttr(0.0)
            UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material)
            UsdShade.MaterialBindingAPI(shape.GetPrim()).Bind(
                material, materialPurpose="physics"
            )
        return root.GetPrim()

    def task_object(spec):
        if spec.name not in names:
            return original_object(spec)
        cfg = sim.CylinderCfg(
            radius=spec.radius,
            height=spec.height,
            func=spawn_bottle,
            rigid_props=PhysxRigidBodyPropertiesCfg(),
            mass_props=sim.MassPropertiesCfg(mass=spec.mass),
            collision_props=PhysxCollisionPropertiesCfg(),
            activate_contact_sensors=True,
            visual_material=sim.PreviewSurfaceCfg(
                diffuse_color=scene.linear_rgb(spec.color)
            ),
        )
        return Object(
            name=spec.name,
            object_type=ObjectType.RIGID,
            spawner_cfg=cfg,
            initial_pose=Pose(position_xyz=spec.pos, rotation_xyzw=(0, 0, 0, 1)),
        )

    def callback(control_hz=None):
        original = original_callback(control_hz)

        def configure(cfg):
            cfg = original(cfg)
            cfg.sim.default_visualizer_cfg.eye = (-0.18, -0.01, 1.08)
            cfg.sim.default_visualizer_cfg.lookat = (0.32, 0, 0.95)
            for name in names:
                setattr(
                    cfg.scene,
                    f"{name}_contacts",
                    ContactSensorCfg(
                        prim_path="{ENV_REGEX_NS}/" + name,
                        update_period=0.0,
                        history_length=1,
                    ),
                )
            return cfg

        return configure

    scene.task_object = task_object
    scene.env_cfg_callback = callback


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--log-dir", type=Path)
    recording_args, _ = parser.parse_known_args()
    install(recording_args.log_dir)
    from inspect_robots.cli import main

    main()
