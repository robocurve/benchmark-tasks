"""Task-local physics extension. No installed adapter files are modified."""

import atexit
import json
from pathlib import Path

import numpy as np

from yam_arena import inspect_embodiment as adapter
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, PAD_TIP_PAST_TCP

ACTIVE = None


def spawn_wood(prim_path, cfg, *args, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from pxr import UsdGeom
    from geometry import wood_mesh, write_mesh
    prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
    UsdGeom.Imageable(prim.GetStage().GetPrimAtPath(prim_path+'/geometry/mesh')).MakeInvisible()
    write_mesh(UsdGeom.Mesh.Define(prim.GetStage(),prim_path+'/WoodSurface'),wood_mesh(cfg.size))
    return prim


def spawn_target(prim_path, cfg, *args, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cylinder
    from pxr import UsdGeom
    from geometry import target_mesh, write_mesh
    prim = spawn_cylinder.__wrapped__(prim_path, cfg, *args, **kwargs)
    UsdGeom.Imageable(prim.GetStage().GetPrimAtPath(prim_path+'/geometry/mesh')).MakeInvisible()
    write_mesh(UsdGeom.Mesh.Define(prim.GetStage(),prim_path+'/Marking'),target_mesh(cfg.radius))
    return prim


def array(value):
    if hasattr(value, "torch"):
        value = value.torch
    return value.detach().cpu().numpy()


def spawn_drill(prim_path, cfg, *args, **kwargs):
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema

    prim = spawn_from_usd.__wrapped__(prim_path, cfg, *args, **kwargs)
    stage = prim.GetStage()
    bit = UsdGeom.Cylinder.Define(stage, prim_path + "/BluntBit")
    bit.CreateAxisAttr("X")
    bit.CreateRadiusAttr(0.003)
    bit.CreateHeightAttr(0.035)
    bit.AddTranslateOp().Set(Gf.Vec3d(-0.1095, -0.0695, 0.0))
    bit.CreateDisplayColorAttr([Gf.Vec3f(0.48, 0.50, 0.55)])
    UsdPhysics.CollisionAPI.Apply(bit.GetPrim())
    collision = PhysxSchema.PhysxCollisionAPI.Apply(bit.GetPrim())
    collision.CreateContactOffsetAttr(0.001)
    collision.CreateRestOffsetAttr(0.0)
    # Only this pair uses the compliant cutting material instead of a rigid
    # contact. All fingers, the chuck and the housing retain ordinary collisions.
    UsdPhysics.FilteredPairsAPI.Apply(bit.GetPrim()).CreateFilteredPairsRel().SetTargets(
        [Sdf.Path(prim_path.rsplit('/',1)[0]+'/wood')])
    stripe = UsdGeom.Mesh.Define(stage,prim_path+'/SpindleStripe')
    points, indices = [], []
    for i,x in enumerate(np.linspace(-.1268,-.0922,100)):
        theta=(x+.1268)/.0346*8*np.pi
        for offset in (-.14,.14):
            points.append((x,-.0695+.00304*np.cos(theta+offset),.00304*np.sin(theta+offset)))
        if i:
            j=2*i
            indices.extend([j-2,j-1,j+1,j])
    stripe.CreatePointsAttr(points)
    stripe.CreateFaceVertexCountsAttr([4]*99)
    stripe.CreateFaceVertexIndicesAttr(indices)
    stripe.CreateDisplayColorAttr([(0.06,0.06,0.06)])
    stripe.CreateDoubleSidedAttr(True)
    # Pivot around the bit's own axis, keeping the physical tool body unaltered.
    stripe.AddTranslateOp().Set(Gf.Vec3d(0,-.0695,0))
    stripe.AddRotateXOp()
    stripe.AddTranslateOp(opSuffix='unpivot').Set(Gf.Vec3d(0,.0695,0))
    return prim


def spawn_trigger(prim_path, cfg, *args, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from pxr import Gf, Sdf, UsdPhysics, PhysxSchema
    from task import TRIGGER_LOCAL, TRIGGER_TRAVEL

    prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
    PhysxSchema.PhysxContactReportAPI.Apply(prim)
    namespace = prim_path.rsplit("/", 1)[0]
    joint = UsdPhysics.PrismaticJoint.Define(prim.GetStage(), namespace + "/TriggerJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(namespace + "/drill")])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(prim_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*TRIGGER_LOCAL))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(TRIGGER_TRAVEL)
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(150.0)
    drive.CreateDampingAttr(2.0)
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateMaxForceAttr(5.0)
    return prim


class WoodDrillEmbodiment(YamArenaEmbodiment):
    def __init__(self, **kwargs):
        global ACTIVE
        super().__init__(**kwargs)
        ACTIVE = self
        self.latest = None
        self._diagnostics = None
        self._steps = 0
        self._states_time = self._robot_time = None
        self._material = None
        self._material_enabled = False
        self._visual_depth = -1.
        self._material_samples = []
        self._wrench_active = False

    def _ensure_sim(self, goal):
        if self._env is not None:
            return
        if adapter._APP is None:
            from isaaclab.app import AppLauncher
            options = dict(headless=self.headless, device=self.device, enable_cameras=True)
            if not self.headless:
                options["visualizer"] = ["kit"]
            adapter._APP = AppLauncher(**options).app
            atexit.register(adapter._close_app_at_exit)

        from yam_arena import scene, embodiment
        from isaaclab.sensors import ContactSensorCfg
        from isaaclab_physx.sensors.contact_sensor import ContactSensor
        from isaaclab.sim.utils import clone
        from isaaclab_arena.utils.configclass import make_configclass
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

        original_object, original_embodiment = scene.task_object, embodiment.BimanualYamEmbodiment

        class SubstepContacts(ContactSensor):
            def update(self, dt, force_recompute=False):
                super().update(dt, force_recompute=True)

        def make_object(spec):
            obj = original_object(spec)
            if spec.name == "drill":
                obj.spawn_cfg_addon["func"] = clone(spawn_drill)
                obj.object_cfg = obj._init_object_cfg()
            elif spec.name == "trigger":
                obj.spawner_cfg.func = clone(spawn_trigger)
                obj.object_cfg = obj._init_object_cfg()
            elif spec.name == "wood":
                # Kinematic fixture: the workpiece stays secured, while its contacts
                # can be filtered independently from the tool's grasp contacts.
                obj.object_type = ObjectType.RIGID
                obj.spawner_cfg.rigid_props = PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)
                obj.spawner_cfg.func = clone(spawn_wood)
                obj.object_cfg = obj._init_object_cfg()
            elif spec.name == "target":
                obj.spawner_cfg.collision_props.collision_enabled = False
                obj.spawner_cfg.func = clone(spawn_target)
                obj.object_cfg = obj._init_object_cfg()
            return obj

        def make_embodiment(*args, **kwargs):
            kwargs["contact_sensors"] = True
            emb = original_embodiment(*args, **kwargs)
            # A filtered sensor provides proof that the left arm touches the trigger,
            # independently of the trigger displacement and of the scripted policy.
            trigger_sensor = ContactSensorCfg(
                class_type=SubstepContacts,
                prim_path="{ENV_REGEX_NS}/trigger",
                filter_prim_paths_expr=[
                    "{ENV_REGEX_NS}/LeftArm" + emb.link_paths[name]
                    for name in ("tip_left", "tip_right")
                ],
                history_length=12,
            )
            emb.scene_config = make_configclass(
                "WoodDrillScene", [
                    ("trigger_contact", ContactSensorCfg, trigger_sensor),
                    ("bit_wood_contact", ContactSensorCfg, ContactSensorCfg(
                        class_type=SubstepContacts,
                        prim_path="{ENV_REGEX_NS}/drill",
                        filter_prim_paths_expr=["{ENV_REGEX_NS}/wood"], history_length=12,
                    )),
                    ("drill_grip_contact", ContactSensorCfg, ContactSensorCfg(
                        class_type=SubstepContacts,
                        prim_path="{ENV_REGEX_NS}/drill",
                        filter_prim_paths_expr=[
                            "{ENV_REGEX_NS}/RightArm" + emb.link_paths[name]
                            for name in ("tip_left", "tip_right")
                        ], history_length=12,
                    )),
                ],
                bases=(type(emb.scene_config),),
            )()
            return emb

        scene.task_object, embodiment.BimanualYamEmbodiment = make_object, make_embodiment
        try:
            super()._ensure_sim(goal)
        finally:
            scene.task_object, embodiment.BimanualYamEmbodiment = original_object, original_embodiment
        # InteractiveScene writes once immediately before every 240 Hz step.
        # Apply material wrenches here, through the same public buffers used by
        # physical assets. Neither the policy nor the scorer writes object poses.
        original_write = self._base.scene.write_data_to_sim
        def write_with_material():
            if self._material_enabled:
                self._material_substep(self._base.physics_dt)
            original_write()
        self._base.scene.write_data_to_sim = write_with_material

    def _material_substep(self, dt):
        from yam_arena.urdf import quat_xyzw_to_matrix
        from yam_arena.runtime import tcp_world
        module, base = self.task.module, self._base
        drill, trigger = base.scene['drill'], base.scene['trigger']
        pose = array(drill.data.root_link_pose_w)[0]
        rot = quat_xyzw_to_matrix(pose[3:])
        tip = pose[:3]+rot@module.BIT_TIP_LOCAL
        axis = rot@module.BIT_AXIS_LOCAL
        offset = rot.T@(array(trigger.data.root_link_pose_w)[0,:3]-pose[:3])-module.TRIGGER_LOCAL
        attached = np.linalg.norm(offset[1:])<.003 and -.002<=offset[0]<=.007
        qualified = False
        near_wood = np.all(np.abs(tip[:2]-module.TARGET_XY)<np.array(module.WOOD_SIZE[:2])/2)
        if near_wood and tip[2]<module.SURFACE_Z+.001 and attached and offset[0]>=module.TRIGGER_THRESHOLD:
            contact = array(base.scene['drill_grip_contact'].data.normal_force_matrix_w)[0]
            forces = np.linalg.norm(contact,axis=-1).sum(axis=0)
            right = base.scene['right_robot']
            gap = float(np.sum(np.abs(array(right.data.joint_pos)[0,self._ids['right_robot'][1]])))
            held = module.holding_handle(tcp_world(right),pose[:3]+rot@np.array([.040,.005,0.]),gap,forces)
            left_force = float(np.linalg.norm(array(base.scene['trigger_contact'].data.normal_force_matrix_w)[0],axis=-1).sum())
            qualified = held and left_force>.05 and -axis[2]>=np.cos(np.radians(module.MAX_TILT_DEG))
        force, torque = self._material.step(tip,axis,float(offset[0]) if attached else 0.,dt,qualified=qualified)
        active = bool(np.any(force) or np.any(torque))
        if active or self._wrench_active:
            com = array(drill.data.root_com_pose_w)[0,:3]
            torque += np.cross(tip-com,force)
            self._force_tensor[0,0] = self._torch.as_tensor(force,dtype=self._torch.float32,device=base.device)
            self._torque_tensor[0,0] = self._torch.as_tensor(torque,dtype=self._torch.float32,device=base.device)
            drill.permanent_wrench_composer.set_forces_and_torques_index(
                forces=self._force_tensor,torques=self._torque_tensor,is_global=True)
            if not active:
                drill.permanent_wrench_composer.reset()
        self._wrench_active = active
        self._material_samples.append(self._material.last['feed_force_n'])

    def _update_cut_visuals(self):
        from pxr import UsdGeom
        from geometry import wood_mesh, target_mesh, write_mesh
        from isaaclab.sim import get_current_stage
        stage = get_current_stage()
        root = '/World/envs/env_0'
        material, module = self._material,self.task.module
        stripe = stage.GetPrimAtPath(root+'/drill/SpindleStripe')
        stripe.GetAttribute('xformOp:rotateX').Set(float(np.degrees(material.phase)))
        if abs(material.depth-self._visual_depth) >= .0001:
            centre = np.array([*module.TARGET_XY,module.TOP+module.WOOD_SIZE[2]/2])
            entry = None if material.entry is None else material.entry-centre
            write_mesh(UsdGeom.Mesh(stage.GetPrimAtPath(root+'/wood/WoodSurface')),
                       wood_mesh(module.WOOD_SIZE,entry,material.bore_axis,material.depth))
            hole = None if material.entry is None else material.entry[:2]-module.TARGET_XY
            if hole is not None and np.linalg.norm(hole)>.006:
                hole = None
            write_mesh(UsdGeom.Mesh(stage.GetPrimAtPath(root+'/target/Marking')),
                       target_mesh(module.TARGET_RADIUS,hole))
            self._visual_depth = material.depth

    def robot_state(self):
        if self._robot_time == self._t:
            return self._robot_cache
        from yam_arena.runtime import tcp_world
        base = self._base
        right, left = base.scene["right_robot"], base.scene["left_robot"]
        tip_sensor = base.scene["trigger_contact"]
        def control_period_force(sensor):
            history = array(sensor.data.normal_force_matrix_w_history)[0]
            return float(np.linalg.norm(history,axis=-1).sum(axis=(1,2)).mean())
        trigger_force = control_period_force(tip_sensor)
        wood_force = control_period_force(base.scene["bit_wood_contact"])
        if self._material_samples:
            wood_force += float(np.mean(self._material_samples[-12:]))
        grip_history = array(base.scene["drill_grip_contact"].data.normal_force_matrix_w_history)[0]
        finger_forces = np.linalg.norm(grip_history, axis=-1).sum(axis=1).mean(axis=0)
        right_ids = right.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        gap = float(np.sum(np.abs(array(right.data.joint_pos)[0, right_ids])))
        rtcp, ltcp = tcp_world(right), tcp_world(left)
        drill = self.object_states()["drill"]
        from task import MODEL_CENTRE
        from yam_arena.urdf import quat_xyzw_to_matrix
        rot = quat_xyzw_to_matrix(drill.quat_xyzw)
        handle = np.asarray(drill.pos) + rot @ (np.array([0.040, 0.005, 0.0]) - MODEL_CENTRE)
        held = self.task.module.holding_handle(rtcp, handle, gap, finger_forces)
        self._robot_cache = {
            "right_holding": held, "left_trigger_contact": trigger_force > 0.05,
            "bit_wood_contact": wood_force > 0.05, "wood_contact_force_n": wood_force,
            "trigger_contact_force_n": trigger_force, "right_gripper_gap_m": gap,
            "right_pad_drill_forces_n": finger_forces.tolist(),
            "right_tcp_m": rtcp.tolist(), "left_tcp_m": ltcp.tolist(),
            **({} if self._material is None else self._material.last),
        }
        self._robot_time = self._t
        return self._robot_cache

    def object_states(self):
        from yam_arena.scene import object_states
        if self._states_time != self._t:
            self._states_cache = object_states(self._env, self.task.names)
            self._states_time = self._t
        return self._states_cache

    def reset(self, scene, *, seed=None):
        from cutting import CuttingModel
        module=self.task.module
        self._material_enabled = False
        self._material = CuttingModel(module.TARGET_XY.copy(),np.array(module.WOOD_SIZE),module.SURFACE_Z)
        self._material_samples = []
        self._wrench_active = False
        self._visual_depth = -1.
        if self._env is not None:
            self._update_cut_visuals()
        self._steps = 0
        self._states_time = self._robot_time = None
        self.task.module._robot = {}
        try:
            result = super().reset(scene, seed=seed)
        except Exception:
            import traceback
            traceback.print_exc()
            raise
        self.latest = None
        self._states_time = self._robot_time = None
        self._force_tensor = self._torch.zeros((1,1,3),device=self._base.device)
        self._torque_tensor = self._torch.zeros_like(self._force_tensor)
        self._material_enabled = True
        self._update_cut_visuals()
        if self._diagnostics:
            self._save_camera_images('setup')
        return result

    def _save_camera_images(self, prefix):
        from PIL import Image
        output=Path(self._diagnostics).parent
        for name in ('top_cam','left_cam','right_cam'):
            rgb=array(self._base.scene[name].data.output['rgb'])[0,...,:3]
            Image.fromarray(rgb).save(output/(prefix+'-'+name+'.png'))

    def _task_step(self):
        self._update_cut_visuals()
        module = self.task.module
        module._robot = self.robot_state()
        self.latest = module.measure(self.object_states(), module._robot)
        self._steps += 1
        if self._steps % 20 == 0:
            m = self.latest
            print(f"[wood drill] t={self._t:.1f} held={m['right_holding']} "
                  f"tilt={m['tilt_deg']:.1f} xy={m['target_error_m']*1000:.1f}mm "
                  f"z={m['surface_error_m']*1000:.1f}mm trigger={m['trigger_travel_m']*1000:.1f}mm "
                  f"feed={m.get('feed_force_n',0):.1f}N depth={m.get('cut_depth_m',0)*1000:.2f}mm "
                  f"qualified={m.get('qualified_depth_m',0)*1000:.2f}mm rpm={m.get('spindle_rpm',0):.0f} success={m['success_now']}", flush=True)
        if self._diagnostics:
            with Path(self._diagnostics).open("a") as stream:
                stream.write(json.dumps({"time_s": self._t, **self.latest}) + "\n")
        result = super()._task_step()
        result.info.update(self.latest)
        if result.terminated and self._diagnostics:
            self._save_camera_images('completed-hole')
        return result
