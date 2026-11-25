"""Dynamic support fixtures and feedback demonstrations for the YAM Arena adapter.

Only explicitly named supports receive bounded actuator forces and torques.
An optional, audited cutting rule applies passive resistance at a blade point.
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
        self.contact_pairs = dict(getattr(self.task.module, "CONTACT_PAIRS", {}))
        self.contact_friction_pairs=set(getattr(self.task.module,'CONTACT_FRICTION_PAIRS',()))
        if not self.contact_friction_pairs.issubset(self.contact_pairs):
            raise ValueError('friction reporting requires a declared filtered contact pair')
        self._friction_sums={name:np.zeros(3) for name in self.contact_friction_pairs}
        from dynamic_task_moving_crank_sharpener.passive_loads import PlaningResistance
        self.passive_loads=dict(getattr(self.task.module,'PASSIVE_CUTTING_LOADS',{}))
        self._passive_sums={name:np.zeros(4) for name in self.passive_loads}
        self._passive_work={name:0. for name in self.passive_loads}
        self._passive_samples=0
        from dynamic_task_moving_crank_sharpener.geometry import contact_patch_radii
        self.contact_patch_radii={name:contact_patch_radii(value) for name,value in
                                  getattr(self.task.module,'CONTACT_PATCH_RADII',{}).items()}
        self.contact_offsets={name:float(value) for name,value in
                              getattr(self.task.module,'CONTACT_OFFSETS_M',{}).items()}
        self._contact_views = {}
        self._contact_sums = {}
        self._contact_samples = 0
        self._conveyor_controls = []
        specs = {o.name: o for o in self.task.objects}
        for name,load in self.passive_loads.items():
            if (not isinstance(load,PlaningResistance) or name not in specs or not specs[name].free
                    or name in self.motions or name in self.conveyors or name in self.fixture_joints
                    or load.contact_pair not in self.contact_pairs
                    or self.contact_pairs[load.contact_pair][0]!=name):
                raise ValueError('passive cutting load requires a free, unactuated contact-reporting body')
            target=self.contact_pairs[load.contact_pair][1]
            if target not in specs or specs[target].free:
                raise ValueError('world-space planing resistance requires fixed stock')
        if any(name not in specs for name in self.contact_patch_radii):
            raise ValueError('contact patch settings require a declared task body')
        if any(name not in specs or not math.isfinite(offset) or offset<=0.
               for name,offset in self.contact_offsets.items()):
            raise ValueError('contact offsets require declared task bodies and positive finite distances')
        for pair in self.contact_pairs.values():
            if len(pair)!=2 or any(name not in specs for name in pair):
                raise ValueError("contact pairs require two declared bodies")
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
        for name in getattr(self.task.module, "SUPPORT_JOINTS", {}):
            if name not in specs or not specs[name].free or name in self.fixture_joints:
                raise ValueError("world guides require rigid bodies without a fixture parent")
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

        antialiasing=getattr(self.task.module,"RENDER_ANTIALIASING",None)
        if antialiasing is not None:
            if antialiasing not in {"Off","FXAA","TAA","DLSS","DLAA"}:
                raise ValueError("unsupported task rendering antialiasing mode")
            import omni.replicator.core as rep
            rep.settings.set_render_rtx_realtime(antialiasing=antialiasing)

        original = scene.task_object

        def support_object(spec):
            obj = original(spec)
            if spec.name in {pair[0] for pair in self.contact_pairs.values()}:
                spawn_contact = obj.spawner_cfg.func

                def spawn_reporting(prim_path, cfg, *args, **kwargs):
                    from pxr import PhysxSchema
                    prim = spawn_contact(prim_path, cfg, *args, **kwargs)
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
                    return prim

                obj.spawner_cfg.func = spawn_reporting
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
            visual_parts = getattr(self.task.module, 'VISUAL_PARTS', {}).get(spec.name)
            if visual_parts:
                spawn_visual_base = obj.spawner_cfg.func

                def spawn_visual_parts(prim_path, cfg, *args, **kwargs):
                    from dynamic_task_moving_crank_sharpener.geometry import author_compound
                    prim = spawn_visual_base(prim_path, cfg, *args, **kwargs)
                    author_compound(prim, visual_parts, cfg, visual_only=True)
                    return prim

                obj.spawner_cfg.func = spawn_visual_parts
            patch_radii=self.contact_patch_radii.get(spec.name)
            contact_offset=self.contact_offsets.get(spec.name)
            if patch_radii is not None or contact_offset is not None:
                spawn_surface=obj.spawner_cfg.func

                def spawn_contact_surface(prim_path,cfg,*args,**kwargs):
                    from pxr import PhysxSchema,Usd,UsdPhysics
                    prim=spawn_surface(prim_path,cfg,*args,**kwargs)
                    for collider in Usd.PrimRange(prim):
                        if collider.HasAPI(UsdPhysics.CollisionAPI):
                            api=PhysxSchema.PhysxCollisionAPI.Apply(collider)
                            if patch_radii is not None:
                                api.CreateTorsionalPatchRadiusAttr(patch_radii[0])
                                api.CreateMinTorsionalPatchRadiusAttr(patch_radii[1])
                            if contact_offset is not None:
                                api.CreateContactOffsetAttr(contact_offset)
                                api.CreateRestOffsetAttr(0.)
                    return prim

                obj.spawner_cfg.func=spawn_contact_surface
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
                    from dynamic_task_moving_crank_sharpener.fixtures import attach_world_guide
                    prim = spawn_body(prim_path, cfg, *args, **kwargs)
                    attach_world_guide(prim, joint_spec, spec.pos, spec.yaw)
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
        self._machine_visuals = None
        machine_specs = getattr(self.task.module, "MACHINE_VISUALS", {})
        if machine_specs:
            import omni.usd
            from dynamic_task_moving_crank_sharpener.mechanisms import MechanismVisuals
            self._machine_visuals = MechanismVisuals(
                omni.usd.get_context().get_stage(), machine_specs,
                self._verified_body_paths, {o.name:o for o in self.task.objects}, self.conveyors)
        if self.contact_pairs:
            import json
            from pathlib import Path
            import omni.usd
            from pxr import Usd,UsdPhysics
            from isaaclab_physx.physics import PhysxManager
            sim_view=PhysxManager.get_physics_sim_view()
            usd_stage=omni.usd.get_context().get_stage()
            specs={o.name:o for o in self.task.objects};view_audit={};self._contact_views={}
            for name,pair in self.contact_pairs.items():
                sensor=self._verified_body_paths[pair[0]];target=self._verified_body_paths[pair[1]]
                # A static object's Xform is not a physics-registered actor.
                # Its concrete collision shapes must be selected individually.
                filters=([target] if specs[pair[1]].free else
                         [str(p.GetPath()) for p in Usd.PrimRange(usd_stage.GetPrimAtPath(target))
                          if p.HasAPI(UsdPhysics.CollisionAPI)])
                if not filters:raise RuntimeError('contact target has no registered collision shapes')
                view=sim_view.create_rigid_contact_view([sensor],filter_patterns=[filters],max_contact_data_count=128)
                if view.sensor_count!=1 or view.filter_count!=len(filters):
                    raise RuntimeError('native contact view did not resolve all declared sensor/filter objects')
                self._contact_views[name]=view
                view_audit[name]={'configured_sensor_path':sensor,'configured_filter_paths':filters,
                                  'native_sensor_count':int(view.sensor_count),'native_filter_count':int(view.filter_count),
                                  'reports_filtered_friction':name in self.contact_friction_pairs}
            audit_path=os.environ.get('DYNA_SCENE_AUDIT')
            if audit_path:
                audit_file=Path(audit_path);audit=json.loads(audit_file.read_text());audit['contact_views']=view_audit
                audit_file.write_text(json.dumps(audit,indent=2)+'\n')
            self._contact_sums={name:np.zeros(3) for name in self.contact_pairs}
        if getattr(self._base, "_physics_handles_decimation", False):
            raise RuntimeError("dynamic supports require the PhysX per-step action loop")
        self._driver_base = self._base
        apply_robot_actions = self._base.action_manager.apply_action

        def advance_supports():
            if self._motion_enabled:
                self._motion_time += self._base.physics_dt
                self._drive_supports(self._motion_time)
                instantaneous_contacts={}
                for name,view in self._contact_views.items():
                    raw=view.get_contact_force_matrix(dt=self._base.physics_dt)
                    values=raw.detach().cpu().numpy() if hasattr(raw,"detach") else raw.numpy()
                    if values.shape!=(1,view.filter_count,3) or not np.isfinite(values).all():
                        raise RuntimeError("invalid native filtered contact-force data")
                    instantaneous_contacts[name]=values[0].sum(axis=0)
                    self._contact_sums[name]+=instantaneous_contacts[name]
                    if name in self.contact_friction_pairs:
                        from dynamic_task_moving_crank_sharpener.contact_forces import filtered_friction_sum
                        buffers=view.get_friction_data(dt=self._base.physics_dt)
                        def numpy(value):
                            return value.detach().cpu().numpy() if hasattr(value,'detach') else value.numpy()
                        self._friction_sums[name]+=filtered_friction_sum(
                            numpy(buffers[0]),numpy(buffers[2]),numpy(buffers[3]),view.filter_count)
                self._apply_passive_loads(instantaneous_contacts)
                self._contact_samples+=1
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
        contact_bodies={body for pair in self.contact_pairs.values() for body in pair}
        needed = (set(self.motions) | set(self.conveyors) | set(self.fixture_joints) | set(self.contact_patch_radii) | set(self.passive_loads)
                  | set(self.contact_offsets) | set(getattr(self.task.module, 'VISUAL_PARTS', {}))
                  | set(getattr(self.task.module, "BODY_PARTS", {})) | set(solver_settings) | contact_bodies)
        self._verified_body_paths={}
        specs={o.name:o for o in self.task.objects}
        audit = {"verified": True, "objects": {}}
        if self.contact_patch_radii:
            scenes=[p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
            if len(scenes)!=1:
                raise RuntimeError('finite torsional contact patches require one verified physics scene')
            solver=PhysxSchema.PhysxSceneAPI(scenes[0]).GetSolverTypeAttr().Get()
            if solver!='TGS':
                raise RuntimeError('finite torsional contact patches require the native TGS solver')
            audit['contact_patch_solver']={'type':solver,'path':str(scenes[0].GetPath())}
        for name in needed:
            matches = [p for p in stage.Traverse() if p.GetName() == name
                       and (not specs[name].free or p.HasAPI(UsdPhysics.RigidBodyAPI))]
            if len(matches) != 1:
                raise RuntimeError(f"expected one spawned task body named {name}, found {len(matches)}")
            body = matches[0]
            path = str(body.GetPath())
            self._verified_body_paths[name]=path
            entry = {"path": path}
            visual_parts = getattr(self.task.module, 'VISUAL_PARTS', {}).get(name)
            if visual_parts:
                visuals = [p for p in body.GetChild('geometry').GetChildren()
                           if p.GetName().startswith('visual_') and not p.GetName().endswith('_material')]
                if len(visuals) != len(visual_parts) or any(
                        p.HasAPI(UsdPhysics.CollisionAPI) or p.HasAPI(UsdPhysics.RigidBodyAPI) for p in visuals):
                    raise RuntimeError('surface visuals must match declarations and have no physics schemas')
                entry['nonphysical_visual_parts'] = len(visuals)
            if name in self.passive_loads:
                from dataclasses import asdict
                entry['passive_cutting_load']={**asdict(self.passive_loads[name]),'discrete_stopping_impulse_cap':True}
            if name in self.contact_patch_radii:
                from pxr import Usd
                expected=self.contact_patch_radii[name]
                colliders=[p for p in Usd.PrimRange(body) if p.HasAPI(UsdPhysics.CollisionAPI)]
                actual=[]
                for collider in colliders:
                    api=PhysxSchema.PhysxCollisionAPI(collider)
                    values=(api.GetTorsionalPatchRadiusAttr().Get(),api.GetMinTorsionalPatchRadiusAttr().Get())
                    if any(v is None or abs(v-e)>1e-7 for v,e in zip(values,expected)):
                        raise RuntimeError('native contact patch radii differ from their declared values')
                    actual.append({'path':str(collider.GetPath()),'radius_m':values[0],'minimum_m':values[1]})
                if not actual:raise RuntimeError('contact patch body has no colliders')
                entry['torsional_contact_patches']=actual
            if name in self.contact_offsets:
                from pxr import Usd
                expected=self.contact_offsets[name]
                colliders=[p for p in Usd.PrimRange(body) if p.HasAPI(UsdPhysics.CollisionAPI)]
                if not colliders:
                    raise RuntimeError('contact offset body has no colliders')
                for collider in colliders:
                    api=PhysxSchema.PhysxCollisionAPI(collider)
                    offset=api.GetContactOffsetAttr().Get()
                    if offset is None or not math.isclose(offset,expected,rel_tol=1e-6) or api.GetRestOffsetAttr().Get()!=0.:
                        raise RuntimeError('native collision offsets differ from their declared values')
                entry['contact_offset_m']=expected
                entry['zero_rest_offset']=True
            parts = getattr(self.task.module, "BODY_PARTS", {}).get(name)
            if parts:
                actual = [p for p in body.GetChild("geometry").GetChildren()
                          if p.GetName().startswith("solid_") and p.HasAPI(UsdPhysics.CollisionAPI)]
                if len(actual) != len(parts) or stage.GetPrimAtPath(path+"/geometry/mesh").IsValid():
                    raise RuntimeError(f"compound geometry for {name} was not spawned as declared")
                entry["compound_solids"] = len(actual)
                entry["placeholder_mesh_removed"] = True
            guide = getattr(self.task.module, "SUPPORT_JOINTS", {}).get(name)
            if guide:
                from dynamic_task_moving_crank_sharpener.fixtures import world_guide_settings
                kind, axis, lower, upper = world_guide_settings(guide)
                cls = {'fixed': UsdPhysics.FixedJoint, 'prismatic': UsdPhysics.PrismaticJoint,
                       'revolute': UsdPhysics.RevoluteJoint}[kind]
                joint = cls.Get(stage, path+"_guide")
                if (not joint or joint.GetBody1Rel().GetTargets() != [body.GetPath()]
                        or joint.GetBody0Rel().GetTargets()):
                    raise RuntimeError(f"world {kind} guide missing or incorrect for {name}")
                if axis is not None and (joint.GetAxisAttr().Get() != axis
                                         or abs(joint.GetLowerLimitAttr().Get()-lower)>1e-6
                                         or abs(joint.GetUpperLimitAttr().Get()-upper)>1e-6):
                    raise RuntimeError(f"world {kind} guide limits are incorrect for {name}")
                entry["world_"+kind+"_guide"] = True
                entry['world_guide_axis'] = axis
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
            if name in {pair[0] for pair in self.contact_pairs.values()}:
                if not body.HasAPI(PhysxSchema.PhysxContactReportAPI):
                    raise RuntimeError(f"contact reporter not enabled for {name}")
                entry["contact_reporter"]=True
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

        if self._machine_visuals is not None:
            self._machine_visuals.update(seconds)
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

    def _apply_passive_loads(self,contacts):
        if not self.passive_loads:return
        import torch
        import warp as wp
        for name,load in self.passive_loads.items():
            view=self._base.scene[name].root_view
            sample=view.get_transforms();torch_backend=isinstance(sample,torch.Tensor)
            pose=sample[0] if torch_backend else wp.to_torch(sample)[0]
            velocity=view.get_velocities();com=view.get_coms()
            velocity=velocity[0] if torch_backend else wp.to_torch(velocity)[0]
            com=com[0,:3] if torch_backend else wp.to_torch(com)[0,:3]
            raw_mass=view.get_inv_masses();raw_inertia=view.get_inv_inertias()
            inverse_mass=(raw_mass if torch_backend else wp.to_torch(raw_mass)).detach().cpu().numpy().reshape(-1)[0]
            inverse_inertia=(raw_inertia if torch_backend else wp.to_torch(raw_inertia)).detach().cpu().numpy()[0].reshape((3,3),order='F')
            force,point,speed=load.evaluate(pose.detach().cpu().numpy(),velocity.detach().cpu().numpy(),
                                          com.detach().cpu().numpy(),contacts[load.contact_pair][2],
                                          dt=self._base.physics_dt,inverse_mass=float(inverse_mass),
                                          inverse_inertia_local=inverse_inertia)
            force_tensor=torch.as_tensor(force,dtype=pose.dtype,device=pose.device).reshape(1,3)
            point_tensor=torch.as_tensor(point,dtype=pose.dtype,device=pose.device).reshape(1,3)
            indices=torch.tensor([0],dtype=torch.int32,device=pose.device)
            if not torch_backend:
                force_tensor=wp.from_torch(force_tensor.contiguous());point_tensor=wp.from_torch(point_tensor.contiguous())
                indices=wp.from_torch(indices)
            view.apply_forces_and_torques_at_position(force_data=force_tensor,torque_data=None,
                                                      position_data=point_tensor,indices=indices,is_global=True)
            power=float(np.dot(force,speed))
            self._passive_sums[name][:3]+=force;self._passive_sums[name][3]+=power
            self._passive_work[name]+=-power*self._base.physics_dt
        self._passive_samples+=1

    def reset(self, scene, *, seed=None):
        self._motion_enabled = False
        self._contact_samples=0
        self._passive_samples=0
        for value in self._passive_sums.values():value[:]=0.
        for name in self._passive_work:self._passive_work[name]=0.
        for force in self._contact_sums.values():force[:]=0.
        for force in self._friction_sums.values():force[:]=0.
        for control in self._conveyor_controls:
            control.Set(False)
        reset_visuals = getattr(self.task.module, 'reset_visuals', None)
        if callable(reset_visuals):
            reset_visuals()
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
        contact_forces={name:force/max(1,self._contact_samples) for name,force in self._contact_sums.items()}
        friction_forces={name:force/max(1,self._contact_samples) for name,force in self._friction_sums.items()}
        passive={name:{'force_world_N':(value[:3]/max(1,self._passive_samples)).copy(),
                       'power_W':float(value[3]/max(1,self._passive_samples)),
                       'dissipated_work_J':self._passive_work[name]} for name,value in self._passive_sums.items()}
        self._passive_samples=0
        for value in self._passive_sums.values():value[:]=0.
        self._contact_samples=0
        for force in self._contact_sums.values():force[:]=0.
        for force in self._friction_sums.values():force[:]=0.
        self.task.module.LIVE_CONTEXT = {
            "contact_forces": contact_forces,
            "contact_friction_forces":friction_forces,
            "passive_loads":passive,
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
            if context.get('tcp_rotation'):
                record['tcp_rotation']={name:value.tolist() for name,value in context['tcp_rotation'].items()}
            if context.get("contact_forces"):
                record["contact_forces"]={name:value.tolist() for name,value in context["contact_forces"].items()}
            if context.get('contact_friction_forces'):
                record['contact_friction_forces']={name:value.tolist() for name,value in context['contact_friction_forces'].items()}
            if context.get('passive_loads'):
                record['passive_loads']={name:{**value,'force_world_N':value['force_world_N'].tolist()}
                                         for name,value in context['passive_loads'].items()}
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
            joint_hook=getattr(module,'demo_joint_target',None)
            if callable(joint_hook):
                q=np.asarray(joint_hook(arm,label,t,q.copy()),dtype=float)
                if (q.shape!=(6,) or not np.isfinite(q).all()
                        or np.any(q<ik.LOWER) or np.any(q>ik.UPPER)):
                    raise ValueError('scripted joint targets must respect the native arm limits')
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

