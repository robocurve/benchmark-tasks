"""Isaac Lab integration: tilted spawn, materials, physical pull and safety trace."""

import argparse
import atexit
import json
from dataclasses import replace
from pathlib import Path

import numpy as np


def install(trace_dir=None):
    import yam_arena.inspect_embodiment as ie
    from yam_arena import scripted
    from yam_arena.task_file import load

    scripted.ArmPlan = load(Path(__file__).with_name("task.py")).module.PanelArmPlan
    original_ensure = ie.YamArenaEmbodiment._ensure_sim
    original_reset = ie.YamArenaEmbodiment.reset
    original_step = ie.YamArenaEmbodiment._task_step
    installed = False

    def ensure(self, goal):
        nonlocal installed
        if ie._APP is None:
            from isaaclab.app import AppLauncher

            options = {
                "headless": self.headless,
                "device": self.device,
                "enable_cameras": True,
            }
            if not self.headless:
                import os

                os.environ["DISPLAY"] = ie.window_display(os.environ)
                options.update(
                    visualizer=["kit"], window_width=1920, window_height=1080
                )
            ie._APP = AppLauncher(**options).app
            atexit.register(ie._close_app_at_exit)
        if not installed:
            install_scene(self.task.module)
            installed = True
        scripted.ArmPlan = self.task.module.PanelArmPlan
        original_ensure(self, goal)

    def reset(self, scene, *, seed=None):
        m = self.task.module
        m.MONITOR = m.Monitor()
        obs = original_reset(self, scene, seed=seed)
        base = self._base
        if not getattr(base.scene, "_panel_monitor_installed", False):
            original_update = base.scene.update

            def update(dt):
                original_update(dt)
                from yam_arena.scene import object_states

                module = self.task.module
                monitor = module.MONITOR
                if not monitor.armed:
                    return
                states = object_states(self._env, module.NAMES)
                monitor.physics(states, dt)

            base.scene.update = update
            original_write = base.scene.write_data_to_sim

            def write():
                original_write()
                import warp as wp
                from yam_arena.scene import object_states

                module = self.task.module
                monitor = module.MONITOR
                force_n = getattr(self, "_panel_probe_force_n", None)
                if force_n is None:
                    force_n = (
                        -module.PULL_N
                        if monitor.pulling and not monitor.reasons
                        else 0.0
                    )
                states = object_states(self._env, module.NAMES)
                panel = states["panel"]
                force_np = np.array((force_n, 0.0, 0.0), dtype=np.float32)
                torque_np = np.cross(
                    module.panel_foot(panel) - panel.pos, force_np
                ).astype(np.float32)
                force = self._torch.tensor(force_np, device=base.device)
                torque = self._torch.tensor(torque_np, device=base.device)
                obj = base.scene["panel"]
                # Use the PhysX world-wrench API immediately before each physics
                # step, with the equivalent torque about the centre of mass.
                obj.root_view.apply_forces_and_torques_at_position(
                    force_data=wp.from_torch(force),
                    torque_data=wp.from_torch(torque),
                    position_data=None,
                    indices=obj._ALL_INDICES,
                    is_global=True,
                )

            base.scene.write_data_to_sim = write
            base.scene._panel_monitor_installed = True
        self._panel_probe_force_n = None
        base.scene["panel"].permanent_wrench_composer.reset()
        m.MONITOR.armed = True
        self._panel_trace = None
        if trace_dir is not None:
            trace_dir.mkdir(parents=True, exist_ok=True)
            self._panel_trace = trace_dir / "panel-trace.jsonl"
        return obs

    def task_step(self):
        from yam_arena.constants import FINGER_OPEN
        from yam_arena.runtime import tcp_world
        from yam_arena.scene import object_states

        m = self.task.module
        states = object_states(self._env, m.NAMES)
        # Clearance is measured against complete world bounding boxes, not centres.
        clear = True
        tcp_positions = {}
        for arm in ie.ARMS:
            robot = self._base.scene[arm]
            opening = (
                robot.data.joint_pos.torch[0, self._ids[arm][1]].mean().item()
                / FINGER_OPEN
            )
            tcp = np.asarray(tcp_world(robot))
            tcp_positions[arm] = tcp.tolist()
            clear &= opening >= 0.8
            for state in states.values():
                delta = np.maximum(np.maximum(state.low - tcp, tcp - state.high), 0)
                clear &= np.linalg.norm(delta) >= 0.08
        m.MONITOR.control(states, bool(clear))
        result = original_step(self)
        info = dict(result.info)
        info.update(
            unsafe_reasons=sorted(m.MONITOR.reasons),
            hands_clear=bool(clear),
            tcp_positions=tcp_positions,
            stopper_seated=m.seated(states["stopper"]),
            proof_start_s=m.MONITOR.proof_start,
            proof_pulling=m.MONITOR.pulling,
            proof_complete=m.MONITOR.complete,
            max_panel_tilt_deg=m.MONITOR.max_tilt,
            max_foot_drift_m=m.MONITOR.max_foot_drift,
        )
        if self._panel_trace:
            record = {"time_s": self._t, "task_sha256": self.task.sha256, **info}
            record["objects"] = {
                n: {
                    "pos": s.pos.tolist(),
                    "quat_xyzw": s.quat_xyzw.tolist(),
                    "lin_vel": s.lin_vel.tolist(),
                }
                for n, s in states.items()
            }
            with self._panel_trace.open("a") as f:
                f.write(json.dumps(record) + "\n")
        unsafe = bool(m.MONITOR.reasons)
        return replace(
            result,
            info=info,
            terminated=unsafe or result.terminated,
            termination_reason="unsafe" if unsafe else result.termination_reason,
        )

    ie.YamArenaEmbodiment._ensure_sim = ensure
    ie.YamArenaEmbodiment.reset = reset
    ie.YamArenaEmbodiment._task_step = task_step


def install_scene(module):
    from isaaclab import sim
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
    from yam_arena import scene

    original = scene.task_object

    def task_object(spec):
        obj = original(spec)
        if spec.free:
            obj.spawner_cfg.collision_props = PhysxCollisionPropertiesCfg(
                contact_offset=0.001, rest_offset=0.0
            )
            obj.spawner_cfg.rigid_props.sleep_threshold = 0.0
        if spec.name == "panel":
            obj.initial_pose = Pose(
                position_xyz=tuple(spec.pos), rotation_xyzw=module.PANEL_QUAT
            )
        if spec.name in {
            "panel",
            "base_plate",
            "back_wall",
            "foot_guide_left",
            "foot_guide_right",
        }:
            # Minimum combination keeps the station table's rubber-like default
            # from replacing this deliberately sliding-prone contact model.
            mu = 0.22 if spec.name in {"panel", "base_plate"} else 0.0
            obj.spawner_cfg.physics_material = sim.RigidBodyMaterialCfg(
                static_friction=mu,
                dynamic_friction=mu * 0.8,
                friction_combine_mode="min",
            )
        return Object(
            name=spec.name,
            object_type=obj.object_type,
            spawner_cfg=obj.spawner_cfg,
            initial_pose=Pose(
                position_xyz=tuple(map(float, spec.pos)),
                rotation_xyzw=tuple(
                    map(
                        float,
                        module.PANEL_QUAT if spec.name == "panel" else (0, 0, 0, 1),
                    )
                ),
            ),
        )

    scene.task_object = task_object


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--log-dir", type=Path)
    args, _ = parser.parse_known_args()
    install(args.log_dir)
    from inspect_robots.cli import main

    main()
