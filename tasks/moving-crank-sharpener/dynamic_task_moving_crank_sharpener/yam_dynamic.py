"""Dynamic support fixtures and feedback demonstrations for the YAM Arena adapter.

Only explicitly named supports receive bounded actuator forces and torques.
All bodies move through physics. Importing this module does not start Isaac.
"""

import math
import weakref
from dataclasses import replace

import numpy as np

from yam_arena import ik, task_file
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.scripted import YamScriptedPolicy
from dynamic_task_moving_crank_sharpener.planning import solve_target

_ACTIVE = weakref.WeakValueDictionary()


class DynamicYamEmbodiment(YamArenaEmbodiment):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.info = replace(self.info, name="dyna_yam")
        if self.task is None:
            raise ValueError("dyna_yam requires -E task=<file>")
        self.motions = dict(getattr(self.task.module, "MOTIONS", {}))
        self.conveyors = dict(getattr(self.task.module, "CONVEYORS", {}))
        self.fixture_joints = dict(getattr(self.task.module, "OBJECT_JOINTS", {}))
        self._conveyor_controls = []
        specs = {o.name: o for o in self.task.objects}
        self._support_masses = {name: specs[name].mass for name in self.motions if name in specs}
        for name, trajectory in self.motions.items():
            if name not in specs or not specs[name].free:
                raise ValueError(f"motion support {name} must be a declared rigid object")
            if not hasattr(trajectory, "at"):
                raise ValueError(f"motion support {name} has no trajectory.at(time)")
            initial = trajectory.at(0.0)
            if np.linalg.norm(np.asarray(specs[name].pos)-np.asarray(initial.position)) > 1e-6:
                raise ValueError(f"motion support {name} initial pose does not match its trajectory")
        for name, velocity in self.conveyors.items():
            if name not in specs or not specs[name].free or name in self.motions:
                raise ValueError("a conveyor must be a declared rigid body without a pose driver")
            if len(velocity) != 3 or not np.isfinite(velocity).all():
                raise ValueError("conveyor surface velocity must contain three finite numbers")
        for child, joint in self.fixture_joints.items():
            if child not in specs or joint["parent"] not in specs or joint["parent"] == child:
                raise ValueError("fixture joint requires distinct declared parent and child bodies")
            if not specs[child].free or not specs[joint["parent"]].free:
                raise ValueError("fixture joints require declared rigid bodies")
        self._motion_time = 0.0
        self._motion_enabled = False
        self._driver_base = None
        _ACTIVE[self.info.environment_id] = self

    def _ensure_sim(self, goal):
        # Isaac modules must be imported only after SimulationApp starts.
        import atexit
        import os
        import yam_arena.inspect_embodiment as adapter
        if adapter._APP is None:
            from isaaclab.app import AppLauncher
            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            options = {"headless": self.headless, "device": self.device, "enable_cameras": True}
            if not self.headless:
                os.environ["DISPLAY"] = adapter.window_display(os.environ)
                options["visualizer"] = ["kit"]
            adapter._APP = AppLauncher(**options).app
            atexit.register(adapter._close_app_at_exit)
        from yam_arena import scene

        original = scene.task_object

        def support_object(spec):
            obj = original(spec)
            solver = getattr(self.task.module, "RIGID_SOLVER_ITERATIONS", {}).get(spec.name)
            if solver:
                position, velocity = solver
                if not (isinstance(position, int) and isinstance(velocity, int)
                        and 1 <= position <= 255 and 0 <= velocity <= 255):
                    raise ValueError("invalid per-body solver iteration counts")
                spawn_solver = obj.spawner_cfg.func

                def spawn_precise(prim_path, cfg, *args, **kwargs):
                    from pxr import PhysxSchema
                    prim = spawn_solver(prim_path, cfg, *args, **kwargs)
                    api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                    api.CreateSolverPositionIterationCountAttr(position)
                    api.CreateSolverVelocityIterationCountAttr(velocity)
                    return prim

                obj.spawner_cfg.func = spawn_precise
            if self.conveyors and spec.free:
                # A settled part must respond when the belt starts at trial t=0.
                obj.spawner_cfg.rigid_props.sleep_threshold = 0.0
            parts = getattr(self.task.module, "BODY_PARTS", {}).get(spec.name)
            if parts:
                spawn_shape = obj.spawner_cfg.func

                def spawn_compound(prim_path, cfg, *args, **kwargs):
                    from dynamic_task_moving_crank_sharpener.geometry import author_compound
                    prim = spawn_shape(prim_path, cfg, *args, **kwargs)
                    author_compound(prim, parts, cfg)
                    return prim

                obj.spawner_cfg.func = spawn_compound
            if spec.name in self.conveyors:
                obj.spawner_cfg.rigid_props.kinematic_enabled = True
                obj.spawner_cfg.rigid_props.disable_gravity = True
                obj.spawner_cfg.rigid_props.enable_ccd = False
                spawn_belt = obj.spawner_cfg.func
                velocity = self.conveyors[spec.name]

                def spawn_conveyor(prim_path, cfg, *args, **kwargs):
                    from pxr import Gf, PhysxSchema
                    prim = spawn_belt(prim_path, cfg, *args, **kwargs)
                    surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
                    surface.CreateSurfaceVelocityLocalSpaceAttr(False)
                    surface.CreateSurfaceVelocityAttr(Gf.Vec3f(*velocity))
                    surface.CreateSurfaceAngularVelocityAttr(Gf.Vec3f(0, 0, 0))
                    enabled = surface.CreateSurfaceVelocityEnabledAttr(False)
                    self._conveyor_controls.append(enabled)
                    return prim

                obj.spawner_cfg.func = spawn_conveyor
            if spec.name in self.motions:
                obj.spawner_cfg.rigid_props.kinematic_enabled = False
                obj.spawner_cfg.rigid_props.disable_gravity = True
                obj.spawner_cfg.rigid_props.enable_ccd = False
                joint_spec = getattr(self.task.module, "SUPPORT_JOINTS", {}).get(spec.name)
                if joint_spec:
                    spawn_body = obj.spawner_cfg.func

                    def spawn_guided(prim_path, cfg, *args, **kwargs):
                        from pxr import Gf, Sdf, UsdPhysics
                        prim = spawn_body(prim_path, cfg, *args, **kwargs)
                        path = str(prim.GetPath())
                        joint = UsdPhysics.PrismaticJoint.Define(prim.GetStage(), path+"_guide")
                        joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
                        joint.CreateAxisAttr(joint_spec.get("axis", "Y"))
                        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*spec.pos))
                        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
                        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1))
                        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
                        joint.CreateLowerLimitAttr().Set(float(joint_spec["lower"]))
                        joint.CreateUpperLimitAttr().Set(float(joint_spec["upper"]))
                        return prim

                    obj.spawner_cfg.func = spawn_guided
            if spec.name in self.fixture_joints:
                spawn_child = obj.spawner_cfg.func
                joint_description = self.fixture_joints[spec.name]
                object_specs = {o.name: o for o in self.task.objects}

                def spawn_jointed(prim_path, cfg, *args, **kwargs):
                    from dynamic_task_moving_crank_sharpener.fixtures import attach_fixture_joint
                    prim = spawn_child(prim_path, cfg, *args, **kwargs)
                    attach_fixture_joint(prim, joint_description, object_specs)
                    return prim

                obj.spawner_cfg.func = spawn_jointed
            # Object constructs object_cfg eagerly, and its config class copies
            # the spawner. Commit the amended spawner to the config actually used.
            obj.object_cfg.spawn = obj.spawner_cfg
            return obj

        scene.task_object = support_object
        try:
            super()._ensure_sim(goal)
        finally:
            scene.task_object = original
        if self._driver_base is self._base:
            return
        self._verify_physical_scene()
        if getattr(self._base, "_physics_handles_decimation", False):
            raise RuntimeError("dynamic supports require the PhysX per-step action loop")
        self._driver_base = self._base
        apply_robot_actions = self._base.action_manager.apply_action

        def advance_supports():
            if self._motion_enabled:
                self._motion_time += self._base.physics_dt
                self._drive_supports(self._motion_time)
            apply_robot_actions()

        self._base.action_manager.apply_action = advance_supports

    def _verify_physical_scene(self):
        """Check the spawned USD, not just the Python declarations."""
        import json
        import os
        from pathlib import Path
        import omni.usd
        from pxr import PhysxSchema, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        solver_settings = getattr(self.task.module, "RIGID_SOLVER_ITERATIONS", {})
        needed = (set(self.motions) | set(self.conveyors) | set(self.fixture_joints)
                  | set(getattr(self.task.module, "BODY_PARTS", {})) | set(solver_settings))
        audit = {"verified": True, "objects": {}}
        for name in needed:
            matches = [p for p in stage.Traverse() if p.GetName() == name and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if len(matches) != 1:
                raise RuntimeError(f"expected one spawned rigid body named {name}, found {len(matches)}")
            body = matches[0]
            path = str(body.GetPath())
            entry = {"path": path}
            parts = getattr(self.task.module, "BODY_PARTS", {}).get(name)
            if parts:
                actual = [p for p in body.GetChild("geometry").GetChildren() if p.GetName().startswith("solid_")]
                if len(actual) != len(parts) or stage.GetPrimAtPath(path+"/geometry/mesh").IsValid():
                    raise RuntimeError(f"compound geometry for {name} was not spawned as declared")
                entry["compound_solids"] = len(actual)
                entry["placeholder_mesh_removed"] = True
            guide = getattr(self.task.module, "SUPPORT_JOINTS", {}).get(name)
            if guide:
                joint = UsdPhysics.PrismaticJoint.Get(stage, path+"_guide")
                if not joint or joint.GetBody1Rel().GetTargets() != [body.GetPath()]:
                    raise RuntimeError(f"world prismatic guide missing for {name}")
                entry["world_prismatic_guide"] = True
            if name in self.motions:
                if not PhysxSchema.PhysxRigidBodyAPI(body).GetDisableGravityAttr().Get():
                    raise RuntimeError(f"actuated support {name} still has gravity enabled")
                entry["gravity_disabled"] = True
            if name in self.conveyors:
                if not UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get():
                    raise RuntimeError(f"conveyor {name} is not kinematic")
                if not body.HasAPI(PhysxSchema.PhysxSurfaceVelocityAPI):
                    raise RuntimeError(f"conveyor surface velocity schema missing for {name}")
                entry["kinematic_contact_conveyor"] = True
            if name in self.fixture_joints:
                joint = UsdPhysics.Joint.Get(stage, path+"_joint")
                expected_parent = path.rsplit("/", 1)[0]+"/"+self.fixture_joints[name]["parent"]
                if (not joint or joint.GetBody1Rel().GetTargets() != [body.GetPath()]
                        or [str(p) for p in joint.GetBody0Rel().GetTargets()] != [expected_parent]):
                    raise RuntimeError(f"fixture joint missing or incorrectly connected for {name}")
                entry["fixture_joint"] = self.fixture_joints[name]["kind"]
            audit["objects"][name] = entry
            if name in solver_settings:
                api = PhysxSchema.PhysxRigidBodyAPI(body)
                actual = [api.GetSolverPositionIterationCountAttr().Get(), api.GetSolverVelocityIterationCountAttr().Get()]
                if actual != list(solver_settings[name]):
                    raise RuntimeError(f"contact solver settings not applied for {name}")
                entry["solver_iterations"] = actual
        target = os.environ.get("DYNA_SCENE_AUDIT")
        if target:
            Path(target).write_text(json.dumps(audit, indent=2)+"\n")

    def _drive_supports(self, seconds):
        import torch
        import warp as wp

        for name, trajectory in self.motions.items():
            state = trajectory.at(seconds)
            view = self._base.scene[name].root_view
            sample = view.get_transforms()
            torch_backend = isinstance(sample, torch.Tensor)
            pose = sample[0] if torch_backend else wp.to_torch(sample)[0]
            raw_velocity = view.get_velocities()
            velocity = raw_velocity[0] if torch_backend else wp.to_torch(raw_velocity)[0]
            desired_pos = torch.tensor(state.position, dtype=pose.dtype, device=pose.device)
            desired_vel = torch.tensor(state.velocity, dtype=pose.dtype, device=pose.device)
            mass = self._support_masses[name]
            force = mass*(800*(desired_pos-pose[:3])+50*(desired_vel-velocity[:3]))
            force *= torch.clamp(20.0/torch.linalg.vector_norm(force).clamp_min(1e-8), max=1.0)
            target_v = torch.tensor([0.0, 0.0, math.sin(state.yaw/2)], dtype=pose.dtype, device=pose.device)
            target_w = math.cos(state.yaw/2)
            current_v, current_w = pose[3:6], pose[6]
            error_v = -target_w*current_v + current_w*target_v - torch.linalg.cross(target_v, current_v)
            error_w = target_w*current_w + torch.dot(target_v, current_v)
            error_v *= torch.where(error_w < 0, -1.0, 1.0)
            desired_spin = torch.tensor(state.angular_velocity, dtype=pose.dtype, device=pose.device)
            torque = 10*error_v + 0.3*(desired_spin-velocity[3:])
            torque *= torch.clamp(0.5/torch.linalg.vector_norm(torque).clamp_min(1e-8), max=1.0)
            if torch_backend:
                forces, torques = force.reshape(1, 3), torque.reshape(1, 3)
                indices = torch.tensor([0], dtype=torch.int32, device=pose.device)
            else:
                forces = wp.from_torch(force.reshape(1, 3).contiguous())
                torques = wp.from_torch(torque.reshape(1, 3).contiguous())
                indices = wp.array([0], dtype=wp.int32, device=self._base.device)
            view.apply_forces_and_torques_at_position(force_data=forces, torque_data=torques,
                                                      position_data=None, indices=indices, is_global=True)

    def reset(self, scene, *, seed=None):
        self._motion_enabled = False
        for control in self._conveyor_controls:
            control.Set(False)
        observation = super().reset(scene, seed=seed)
        self._motion_time = 0.0
        self._motion_enabled = True
        for control in self._conveyor_controls:
            control.Set(True)
        module = self.task.module
        if callable(getattr(module, "on_reset", None)):
            module.on_reset()
        self._capture_task_state()
        return observation

    def _capture_task_state(self):
        from yam_arena.runtime import tcp_world, link_poses
        from yam_arena.scene import object_states

        self.task.module.LIVE_OBJECTS = object_states(self._env, self.task.names)
        world = self._base.arena_world
        origin = self._base.scene.env_origins[0]
        self.task.module.LIVE_CONTEXT = {
            "time_s": self._motion_time,
            "tcp": {side: tcp_world(self._base.scene[side+"_robot"]) for side in ("left", "right")},
            "tcp_rotation": {side: link_poses(self._base.scene[side+"_robot"])["gripper"][:3, :3]
                             for side in ("left", "right")},
            "body_origins": {name: (world.get_pose_w(name)[0, :3]-origin).detach().cpu().numpy()
                             for name in self.task.names},
            "gripper": {side: float(self._base.scene[side+"_robot"].data.joint_pos.torch[
                0, self._ids[side+"_robot"][1]].mean().cpu().item()/FINGER_OPEN) for side in ("left", "right")},
        }

    def _task_step(self):
        self._capture_task_state()
        observe = getattr(self.task.module, "observe", None)
        if callable(observe):
            observe(self.task.module.LIVE_OBJECTS, self.task.module.LIVE_CONTEXT, self._base.step_dt)
        material_update = getattr(self.task.module, "update_materials", None)
        if callable(material_update):
            material_update()
        from inspect_robots.types import StepResult
        task = self.task
        states = task.module.LIVE_OBJECTS
        info = {}
        sustained = False
        if task.success is not None:
            self._held = self._held+1 if task.success(states) else 0
            sustained = self._held >= max(1, round(task.success_hold_s*self.info.control_hz))
            info["success"] = sustained
        if task.distance is not None:
            info["distance"] = float(task.distance(states))
        if task.score is not None:
            now = min(max(float(task.score(states)), 0.0), 1.0)
            # An instantaneous placement must not bypass the documented hold
            # duration by ending the trial while info['score'] is already 1.
            if now == 1.0 and task.success is not None and not sustained:
                now = float(getattr(task.module, "MAX_PARTIAL_SCORE", 0.0))
                if not 0 <= now < 1:
                    raise ValueError("MAX_PARTIAL_SCORE must be in [0,1)")
            self._best_score = max(self._best_score, now)
            info["score"] = self._best_score if task.score_over == "best" else now
        result = StepResult(observation=self._observation(), terminated=sustained,
                            termination_reason="success" if sustained else None, info=info)
        # Privileged diagnostic evidence is written locally, never added to the
        # policy observation. It makes scoring and failed grasps auditable.
        import json
        import os
        trace_path = os.environ.get("DYNA_STATE_TRACE")
        if trace_path:
            if not hasattr(self, "_trace_file"):
                import atexit
                self._trace_file = open(trace_path, "a", buffering=65536)
                atexit.register(self._trace_file.close)
            states = {name: {key: getattr(state, key).tolist() for key in
                            ("pos", "quat_xyzw", "lin_vel", "ang_vel", "low", "high")}
                      for name, state in self.task.module.LIVE_OBJECTS.items()}
            context = self.task.module.LIVE_CONTEXT
            record = {"time_s": self._motion_time, "objects": states,
                      "tcp": {k: v.tolist() for k, v in context["tcp"].items()},
                      "body_origins": {k: v.tolist() for k, v in context["body_origins"].items()},
                      "gripper": context["gripper"], "score_info": result.info}
            self._trace_file.write(json.dumps(record, separators=(",", ":"))+"\n")
            if round(self._motion_time*self.info.control_hz) % 20 == 0:
                self._trace_file.flush()
        return result


class DynamicScriptedPolicy(YamScriptedPolicy):
    """The official waypoint policy with state feedback during moving-object acquisition."""

    def act(self, observation):
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import pack

        t = float(observation.state_time) + self._dt
        active = _ACTIVE.get(self._environment_id)
        if active is None:
            raise RuntimeError("dyna_scripted must run with a live dyna_yam embodiment")
        module = active.task.module
        tracked = getattr(module, "TRACK_OBJECTS", {})
        anchors = getattr(module, "TRACK_ANCHORS", {})
        tracked_phases = getattr(module, "TRACK_PHASES", {})
        initial = {o.name: np.array(o.pos) for o in active.task.objects if hasattr(o, "pos")}
        targets = []
        for arm in ("left", "right"):
            if arm not in self._plans:
                targets.append(np.r_[READY, 1.0])
                continue
            plan = self._plans[arm]
            pos, rotation, finger, label = plan.target(t)
            name = tracked.get(arm)
            if name and label in tracked_phases.get(arm, {"to the object", "descend", "close"}):
                pos = pos + module.LIVE_OBJECTS[name].pos - np.asarray(anchors.get(arm, initial[name]))
            orientation = getattr(module, "demo_rotation", None)
            if callable(orientation):
                rotation = orientation(arm, label, t, rotation)
            target_hook = getattr(module, "demo_target", None)
            if callable(target_hook):
                pos, rotation, finger = target_hook(arm, label, t, pos, rotation, finger)
            q, _, _ = solve_target(pos, rotation, plan.q, plan.base_pos)
            plan.q = q
            targets.append(np.r_[q, np.clip(finger/FINGER_OPEN, 0, 1)])
        meta = {"request_stop": True} if t > self._end+self.hold_s else {}
        return ActionChunk(actions=[Action(data=pack(*targets), meta=meta)], control_hz=1/self._dt)


def make_embodiment(**kwargs):
    return DynamicYamEmbodiment(**kwargs)


def make_scripted_policy(**kwargs):
    return DynamicScriptedPolicy(**kwargs)


class IdlePolicy(YamScriptedPolicy):
    """Hold the reset pose to test that environmental motion alone earns no credit."""

    def reset(self, scene):
        pass

    def act(self, observation):
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import pack
        return ActionChunk(actions=[Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))],
                           control_hz=1/self._dt)


def make_idle_policy(**kwargs):
    return IdlePolicy(**kwargs)

