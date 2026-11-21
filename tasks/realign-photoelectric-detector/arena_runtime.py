"""The procedural photoelectric scene inside an actual Isaac Lab-Arena environment.

Policy observations are produced separately from evaluator measurements. Reset
rebuilds the environment, including every clamp and optical proof state and camera product.
"""
from pathlib import Path
from dataclasses import asdict
import json
import hashlib
import math
import time
import numpy as np
import spec
import clamp
from scipy.spatial.transform import Rotation
from scorer import Evidence, Scorer

APP = None
CAMERAS = {'overhead_cam': 'top_cam', 'left_wrist_cam': 'left_cam', 'right_wrist_cam': 'right_cam'}


def array(value):
    value = value.torch if hasattr(value, 'torch') else value
    return value.detach().cpu().numpy() if hasattr(value, 'detach') else np.asarray(value)


class PhotoWorld:
    def __init__(self, output, seed=0):
        global APP
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.env = None
        self.steps = 0
        self.scorer = Scorer()
        self.history = []
        self.started = time.monotonic()
        self.trace = (self.output/'trace.jsonl').open('w')
        if APP is None:
            from isaaclab.app import AppLauncher
            APP = AppLauncher(headless=True, enable_cameras=True, device='cpu').app
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
        import omni.usd
        import torch
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from yam_arena import station, constants, ik
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback
        import scene
        import demo
        self.torch, self.constants = torch, constants
        omni.usd.get_context().new_stage()
        asset_stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(asset_stage, '/World').GetPrim()
        asset_stage.SetDefaultPrim(root)
        objects = scene.build(asset_stage)
        # Application/renderer warm-up must not consume any task dynamics.
        # Restore every authored rigid pose and zero velocity before step zero.
        authored = {}
        for prim in asset_stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                q = matrix.ExtractRotationQuat()
                authored[str(prim.GetPath())] = [*matrix.ExtractTranslation(), *q.GetImaginary(), q.GetReal()]
                PhysxSchema.PhysxRigidBodyAPI(prim).CreateDisableGravityAttr(True)
        asset_stage.RemovePrim('/World/physics')
        light = UsdLux.DomeLight.Define(asset_stage, '/World/light')
        light.CreateIntensityAttr(800.)
        sun = UsdLux.DistantLight.Define(asset_stage, '/World/sun')
        sun.CreateIntensityAttr(1600.)
        sun.AddRotateXYZOp().Set(Gf.Vec3f(25, -30, 20))
        usd = self.output/'workbench.usda'
        asset_stage.Export(str(usd))
        self.prefix = '/World/envs/env_0/workbench'
        self.objects = objects
        assembly = Object(name='workbench', prim_path='{ENV_REGEX_NS}/workbench',
                          usd_path=str(usd), object_type=ObjectType.BASE,
                          initial_pose=Pose(position_xyz=(0., 0., 0.), rotation_xyzw=(0., 0., 0., 1.)))
        import task_station
        rig = task_station.rig()
        embodiment = BimanualYamEmbodiment(enable_cameras=True, rig=rig,
                        gravity_compensation=True, cam_height=360, cam_width=640,
                        robot_contact_offset=.0002, contact_sensors=False)
        embodiment.camera_config.set_use_tiled_camera(False)
        initial, _ = demo.plan(ik, constants, station)
        self.initial_command = initial[0][1]
        for side in ['left', 'right']:
            arm = getattr(embodiment.scene_config, side+'_robot')
            arm.init_state.joint_pos = dict(zip(constants.ALL_JOINTS, self.initial_command[side]))
            arm.init_state.joint_vel = {'.*': 0.}
            arm.actuators['gripper'].joint_effort_limit = spec.FINGER_FORCE_LIMIT
            arm.spawn.articulation_props.solver_position_iteration_count = 32
            arm.spawn.articulation_props.solver_velocity_iteration_count = 0
            original_spawn = arm.spawn.func

            def spawn_arm(prim_path, cfg, *args, _spawn=original_spawn, **kwargs):
                from pxr import UsdPhysics, UsdShade, PhysxSchema
                prim = _spawn(prim_path, cfg, *args, **kwargs)
                stage = prim.GetStage()
                matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                # Keep the articulation container at identity. With a translated
                # non-rigid container, this Lab/PhysX pair adds its translation
                # again when it synchronizes the fixed root back to USD.
                prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0,0,0))
                for child in Usd.PrimRange(prim):
                    if child.GetName()=='root_joint' and child.IsA(UsdPhysics.FixedJoint):
                        joint = UsdPhysics.FixedJoint(child)
                        # The asset's non-rigid body0 parent makes Lab's anchor
                        # update add the base offset repeatedly. Anchor to the
                        # world explicitly before physics is initialized.
                        joint.GetBody0Rel().SetTargets([])
                        joint.GetLocalPos0Attr().Set(Gf.Vec3f(*matrix.ExtractTranslation()))
                        joint.GetLocalRot0Attr().Set(Gf.Quatf(matrix.ExtractRotationQuat()))
                        base = stage.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0])
                        UsdGeom.Xformable(base).MakeMatrixXform().Set(matrix)
                    if child.HasAPI(UsdPhysics.CollisionAPI):
                        contact = PhysxSchema.PhysxCollisionAPI.Apply(child)
                        contact.CreateContactOffsetAttr(.0002)
                        contact.CreateRestOffsetAttr(0.)
                        pad = any(x in str(child.GetPath()).split('/') for x in ('tip_left','tip_right'))
                        if pad:
                            contact.CreateTorsionalPatchRadiusAttr(spec.PAD_TORSIONAL_RADIUS)
                            contact.CreateMinTorsionalPatchRadiusAttr(spec.PAD_TORSIONAL_RADIUS)
                        UsdShade.MaterialBindingAPI.Apply(child).Bind(
                            UsdShade.Material(stage.GetPrimAtPath(self.path('/World/pad_contact' if pad else '/World/contact'))), materialPurpose='physics')
                # Imported links are nested. Make every rigid link's transform
                # independent while preserving its world pose and joint frames;
                # this avoids parent transforms being applied twice by RTX.
                bodies = [(child, UsdGeom.Xformable(child).ComputeLocalToWorldTransform(0))
                          for child in Usd.PrimRange(prim) if child.HasAPI(UsdPhysics.RigidBodyAPI)]
                for child, pose in bodies:
                    xf = UsdGeom.Xformable(child)
                    xf.MakeMatrixXform().Set(pose)
                    xf.SetResetXformStack(True)
                fingers={child.GetName():child for child in Usd.PrimRange(prim)
                         if child.GetName() in ('joint7','joint8') and child.IsA(UsdPhysics.PrismaticJoint)}
                if len(fingers)!=2:raise RuntimeError('Missing parallel-gripper joints')
                for finger in fingers.values():
                    UsdPhysics.PrismaticJoint(finger).GetLowerLimitAttr().Set(-.0375)
                mimic=PhysxSchema.PhysxMimicJointAPI.Apply(fingers['joint8'],'rotX')
                mimic.CreateReferenceJointRel().SetTargets([fingers['joint7'].GetPath()])
                mimic.CreateReferenceJointAxisAttr('rotX');mimic.CreateGearingAttr(-1.)
                mimic.CreateOffsetAttr(0.);mimic.CreateNaturalFrequencyAttr(0.)
                return prim

            arm.spawn.func = spawn_arm
        standard = env_cfg_callback(spec.CONTROL_HZ)

        def configure(cfg):
            cfg = standard(cfg)
            cfg.seed = seed
            cfg.sim.dt = 1/spec.PHYSICS_HZ
            cfg.decimation = spec.PHYSICS_HZ//spec.CONTROL_HZ
            cfg.sim.render_interval = cfg.decimation
            cfg.sim.use_fabric = False
            cfg.sim.enable_scene_query_support = True
            cfg.sim.physics.enable_scene_query_support = True
            cfg.sim.physics.enable_ccd = True
            # Finite elastic microslip prevents the position drift observed
            # with damping-only standalone brakes; native force caps remain.
            cfg.sim.physics.solver_type = 1
            cfg.sim.physics.min_velocity_iteration_count = 0
            cfg.sim.physics.max_velocity_iteration_count = 0
            # Contact dimensions are millimetres; the default 25 mm anchor
            # correlation radius spans the entire clamp and finger pads.
            cfg.sim.physics.friction_correlation_distance = .0005
            cfg.sim.physics.friction_offset_threshold = .0001
            cfg.episode_length_s = 130.
            return cfg

        description = IsaacLabArenaEnvironment(name='photoelectric', scene=Scene([assembly]),
                            embodiment=embodiment, env_cfg_callback=configure)
        self.env = ArenaEnvBuilder(description, ArenaEnvBuilderCfg(num_envs=1,
                              solve_relations=False, device='cpu', disable_fabric=True)).make_registered()
        self.base = self.env.unwrapped
        self.stage = self.base.sim.stage
        # Lab disables reporting globally unless a ContactSensor is created.
        # These evaluator-only native views need the same reporting switch.
        self.base.sim.set_setting('/physics/disableContactProcessing',False)
        self.env.reset(seed=seed)
        import omni.physics.tensors as tensors
        from isaaclab.sim.utils.stage import get_current_stage_id
        # This pinned Lab backend leaves Fabric's USD notice listener disabled
        # after initialization. Variable passive joint resistance must reach
        # native PhysX, rather than only changing the authored USD values.
        import usdrt
        from isaaclab.cloner.fabric_notices import get_bindings
        notices = get_bindings()
        fabric_id = usdrt.Usd.Stage.Attach(get_current_stage_id()).GetFabricId().id
        if notices is None or not notices.validate_with(fabric_id):
            raise RuntimeError('Cannot enable native physics property updates')
        notices_before = notices.is_enabled(fabric_id)
        notices.set_enable(fabric_id, True)
        if not notices.is_enabled(fabric_id):
            raise RuntimeError('Physics property update listener remains disabled')
        (self.output/'physics-update-self-test.json').write_text(json.dumps({
            'usd_notices_before':notices_before,'usd_notices_enabled':True},indent=2)+'\n')
        self.reset_sim_view = tensors.create_simulation_view('numpy', stage_id=get_current_stage_id())
        self.reset_sim_view.set_subspace_roots('/')
        self.prop_views = {}
        reset_state = {}
        for original, pose in authored.items():
            path = self.path(original)
            view = self.reset_sim_view.create_rigid_body_view(path)
            ids = np.array([0], dtype=np.uint32)
            view.set_transforms(np.asarray([pose], dtype=np.float32), ids)
            view.set_velocities(np.zeros((1,6), dtype=np.float32), ids)
            view.set_disable_gravities(np.zeros((1,1), dtype=np.uint8), ids)
            PhysxSchema.PhysxRigidBodyAPI(self.stage.GetPrimAtPath(path)).GetDisableGravityAttr().Set(False)
            self.prop_views[original] = view
            reset_state[original] = {'pose':view.get_transforms().tolist(), 'velocity':view.get_velocities().tolist()}
        (self.output/'reset-state.json').write_text(json.dumps(reset_state, indent=2)+'\n')
        # Lab's current backend initializes fixed-root transforms separately
        # from joint state; the pinned YAM reset event resets only joints.
        for side in ['left','right']:
            arm = self.base.scene[side+'_robot']
            arm.write_root_pose_to_sim_index(root_pose=arm.data.default_root_pose.torch.clone())
            arm.write_root_velocity_to_sim_index(root_velocity=arm.data.default_root_vel.torch.clone())
        self.base.scene.write_data_to_sim()
        self.base.sim.forward()
        robot_reset = {}
        for side in ['left','right']:
            arm = self.base.scene[side+'_robot']
            robot_reset[side] = {'root_pose':array(arm.data.root_pose_w).tolist(),
                                 'root_velocity':array(arm.data.root_vel_w).tolist(),
                                 'joint_position':array(arm.data.joint_pos).tolist(),
                                 'joint_velocity':array(arm.data.joint_vel).tolist(),
                                 'joint_names':list(arm.joint_names)}
        (self.output/'reset-robot-state.json').write_text(json.dumps(robot_reset,indent=2)+'\n')
        gripper_contract={}
        for side in ['left','right']:
            arm=self.base.scene[side+'_robot']
            ids,_=arm.find_joints(constants.FINGER_JOINTS,preserve_order=True)
            force=arm.root_view.get_dof_max_forces().numpy()[0,ids]
            limits=arm.root_view.get_dof_limits().numpy()[0,ids,:]
            if not np.allclose(force,40.) or not np.allclose(limits,[[-.0375,0.],[-.0375,0.]],atol=1e-7):
                raise RuntimeError('Native gripper force or aperture contract differs from the specification')
            gripper_contract[side]={'force_caps_n':force.tolist(),'position_limits_m':limits.tolist()}
        (self.output/'reset-gripper-physics.json').write_text(json.dumps(gripper_contract,indent=2)+'\n')
        self.pads = []
        self.robot_colliders = []
        self.hands = []
        from pxr import UsdPhysics, UsdShade, PhysxSchema
        for prim in Usd.PrimRange.Stage(self.stage, Usd.TraverseInstanceProxies()):
            if any(str(prim.GetPath()).startswith('/World/envs/env_0/'+s) for s in ['LeftArm','RightArm']):
                if prim.HasAPI(UsdPhysics.CollisionAPI):self.robot_colliders.append(str(prim.GetPath()))
                if prim.GetName() in ['tip_left','tip_right'] and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    self.pads.append(str(prim.GetPath()))
                if prim.GetName() in ['gripper','tip_left','tip_right'] and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    self.hands.append(str(prim.GetPath()))
        if len(self.pads) != 4:
            raise RuntimeError('Expected four physical finger pads')
        if len(self.hands)!=6:raise RuntimeError('Expected two gripper bodies and four fingers')
        if not self.robot_colliders:raise RuntimeError('Robot collision geometry missing')
        self.fixed_positions={name:np.asarray(self.matrix(self.path('/World/'+name)).ExtractTranslation())
                              for name in ['frame','reflector']}
        for _ in range(4):
            self.base.sim.render()
        for name in CAMERAS.values():
            # update(0, force_recompute=True) refreshes only outdated buffers
            # in this backend. Discard env.reset's pre-restoration RGB cache
            # so the first delivered frame shows the published robot pose.
            self.base.scene[name].reset()
            self.base.scene[name].update(0., force_recompute=True)
        self.stage.Export(str(self.output/'reset.usda'))
        self.peak_clamp_torque=spec.RESET_TORQUE
        self.peak_tool_torque=0.
        self.previous_nut=0.;self.previous_slip=0.
        self.friction_contacts={name:clamp.FrictionContact(spec.CONTACT_ANGULAR_STIFFNESS,position,True)
                                for name,position in [('yaw',spec.INITIAL_YAW),('pitch',0.),('nut',0.)]}
        self.friction_contacts['height']=clamp.FrictionContact(spec.CONTACT_LINEAR_STIFFNESS,0.)
        self.socket_key_active=True
        self.update_clamp()
        self.physics_callback_count=0
        # This backend steps IPhysxSimulation directly, bypassing the legacy
        # IPhysx on-step event used by register_callback. Wrap this world's
        # actual native step so passive material updates cannot silently stop.
        self.original_physics_step=self.base.sim.backend_manager.step
        def step_with_material():
            self.physics_callback_count+=1
            if self.physics_callback_count%(spec.PHYSICS_HZ//spec.CLAMP_UPDATE_HZ)==0:
                self.update_clamp()
            return self.original_physics_step()
        self.base.sim.backend_manager.step=step_with_material
        from omni.physx import get_physx_scene_query_interface
        origin=np.asarray(spec.CARD_START)+[0,0,.08]
        probe=get_physx_scene_query_interface().raycast_closest(
            Gf.Vec3f(*origin),Gf.Vec3f(0,0,-1),.10)
        if not probe['hit'] or str(probe['rigidBody'])!=self.path('/World/card'):
            raise RuntimeError('Native ray query failed the known-card self-test')
        (self.output/'query-self-test.json').write_text(json.dumps({'known_card_hit':True,'distance_m':probe['distance']})+'\n')
        self.save_frames('00000')
        (self.output/'reset-cameras.json').write_text(json.dumps(
            {name:{'shape':list(im.shape),'sha256':hashlib.sha256(im.tobytes()).hexdigest()}
             for name,im in self.images().items()},indent=2)+'\n')

    def path(self, original):
        return original.replace('/World', self.prefix, 1)

    def matrix(self, path):
        from pxr import UsdGeom
        return UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)

    def bounds(self, path):
        from geometry_bounds import world_bounds
        return world_bounds(self.stage.GetPrimAtPath(path))

    def images(self, native=False):
        from PIL import Image
        frames = {name:array(self.base.scene[key].data.output['rgb'])[0,:,:,:3].astype(np.uint8).copy()
                  for name,key in CAMERAS.items()}
        if not native:
            frames = {name:np.asarray(Image.fromarray(im).resize((320,176), Image.Resampling.BILINEAR)) for name,im in frames.items()}
        return frames

    def joints(self):
        result = []
        for side in ['left','right']:
            arm = self.base.scene[side+'_robot']
            ids, _ = arm.find_joints(self.constants.ARM_JOINTS, preserve_order=True)
            fingers, _ = arm.find_joints(self.constants.FINGER_JOINTS, preserve_order=True)
            q = array(arm.data.joint_pos)[0]
            result.extend([*q[ids], float(np.clip(q[fingers].mean()/self.constants.FINGER_OPEN,0,1))])
        return np.asarray(result, dtype=np.float32)

    def save_frames(self, label):
        from PIL import Image
        for name,im in self.images(native=True).items():
            folder = self.output/name
            folder.mkdir(exist_ok=True)
            Image.fromarray(im).save(folder/(label+'.png'))
            if label == '00000' and name == 'overhead_cam':
                Image.fromarray(im).save(self.output/'setup.png')

    def update_clamp(self):
        from pxr import Gf, UsdPhysics, PhysxSchema
        body_rotation=Rotation.from_quat(self.prop_views['/World/clamp_body'].get_transforms()[0,3:]).as_matrix()
        bolt_rotation=Rotation.from_quat(self.prop_views['/World/bolt'].get_transforms()[0,3:]).as_matrix()
        rotation=Rotation.from_euler('y',90,degrees=True).as_matrix()@body_rotation.T@bolt_rotation
        angle=math.degrees(math.atan2(rotation[1,0],rotation[0,0]))
        force,torque,cap=clamp.load(-angle)
        self.peak_clamp_torque=max(self.peak_clamp_torque,torque)
        bolt=UsdPhysics.DriveAPI(self.stage.GetPrimAtPath(self.path('/World/joints/bolt')),'angular')
        bolt.GetMaxForceAttr().Set(torque)
        bolt.GetTargetPositionAttr().Set(math.degrees(self.friction_contacts['nut'].update(math.radians(angle),torque)))
        mount=UsdPhysics.DriveAPI(self.stage.GetPrimAtPath(self.path('/World/joints/mount')),'angular')
        mount.GetMaxForceAttr().Set(cap)
        post=self.stage.GetPrimAtPath(self.path('/World/joints/post_clamp'))
        UsdPhysics.DriveAPI(post,'rotZ').GetMaxForceAttr().Set(spec.FRICTION*force*.006)
        UsdPhysics.DriveAPI(post,'transZ').GetMaxForceAttr().Set(spec.FRICTION*force)
        body_pose=self.prop_views['/World/clamp_body'].get_transforms()[0]
        yaw_joint=math.atan2(body_rotation[1,0],body_rotation[0,0])
        UsdPhysics.DriveAPI(post,'rotZ').GetTargetPositionAttr().Set(math.degrees(self.friction_contacts['yaw'].update(yaw_joint,spec.FRICTION*force*.006)))
        UsdPhysics.DriveAPI(post,'transZ').GetTargetPositionAttr().Set(self.friction_contacts['height'].update(float(body_pose[2])-spec.CLAMP_POST[2],spec.FRICTION*force))
        mount_rotation=Rotation.from_quat(self.prop_views['/World/mount'].get_transforms()[0,3:]).as_matrix()
        relative_mount=body_rotation.T@mount_rotation
        pitch=math.atan2(relative_mount[0,2],relative_mount[0,0])
        mount.GetTargetPositionAttr().Set(math.degrees(self.friction_contacts['pitch'].update(pitch,cap)))
        socket_rotation=Rotation.from_quat(self.prop_views['/World/socket'].get_transforms()[0,3:]).as_matrix()
        handle_rotation=Rotation.from_quat(self.prop_views['/World/driver'].get_transforms()[0,3:]).as_matrix()
        bolt_pose=self.prop_views['/World/bolt'].get_transforms()[0]
        socket_pose=self.prop_views['/World/socket'].get_transforms()[0]
        key_offset=bolt_rotation.T@(socket_pose[:3]-bolt_pose[:3])
        key=UsdPhysics.Joint(self.stage.GetPrimAtPath(self.path('/World/joints/socket_key')))
        if self.socket_key_active and key_offset[2]>.00075:
            key.GetJointEnabledAttr().Set(False)
            self.socket_key_active=False
        elif not self.socket_key_active:
            relative_key=bolt_rotation.T@socket_rotation
            key_angle=math.atan2(relative_key[1,0],relative_key[0,0])
            index=round(key_angle/(math.pi/3))*(math.pi/3)
            if (-.002<=key_offset[2]<=-.001 and np.linalg.norm(key_offset[:2])<=.00005
                    and relative_key[2,2]>.99999 and abs(key_angle-index)<=math.radians(.5)):
                key.GetLocalRot1Attr().Set(Gf.Quatf(math.cos(index/2),0.,0.,-math.sin(index/2)))
                key.GetJointEnabledAttr().Set(True)
                self.socket_key_active=True
        clutch_relative=socket_rotation.T@handle_rotation
        clutch_angle=math.atan2(clutch_relative[1,0],clutch_relative[0,0])
        if self.socket_key_active:
            self.peak_tool_torque=max(self.peak_tool_torque,-clutch_angle*spec.DRIVER_SPRING_NM_RAD)
        dial_angle=clutch_angle*spec.DRIVER_SPRING_NM_RAD*math.radians(spec.DIAL_DEG_PER_NM)
        self.stage.GetPrimAtPath(self.path('/World/socket/needle')).GetAttribute('xformOp:orient').Set(
            Gf.Quatf(math.cos(dial_angle/2),0.,0.,math.sin(dial_angle/2)))
        yaw=math.atan2(mount_rotation[1,0],mount_rotation[0,0])
        omega=float(self.prop_views['/World/mount'].get_velocities()[0,5])
        cable=float(np.clip(-spec.CABLE_STIFFNESS_NM_RAD*(yaw-spec.INITIAL_YAW)-spec.CABLE_DAMPING_NMS_RAD*omega,-.03,.03))
        load=PhysxSchema.PhysxForceAPI(self.stage.GetPrimAtPath(self.path('/World/mount')))
        load.GetTorqueAttr().Set(Gf.Vec3f(0.,0.,cable))
        # This Lab backend steps IPhysxSimulation directly. Commit buffered
        # USD drive/force edits before its next native step.
        from omni.physx import get_physx_simulation_interface
        get_physx_simulation_interface().flush_changes()
        self.clamp_data={'bolt_angle_deg':angle,'clamp_force_estimate_n':force,
                         'thread_resistance_nm':torque,'peak_thread_resistance_nm':self.peak_clamp_torque,
                         'friction_torque_cap_nm':cap,'cable_torque_nm':cable,
                         'friction_rest_si':{k:v.rest for k,v in self.friction_contacts.items()}}

    def measure(self):
        from pxr import Gf, UsdShade
        import optics
        matrix=self.matrix(self.path('/World/mount'))
        rot=np.asarray(matrix.ExtractRotationMatrix()).T
        origin=np.asarray(matrix.Transform(Gf.Vec3d(*spec.LENS_LOCAL)))
        yaw=math.degrees(math.atan2(rot[1,0],rot[0,0]))
        clo,chi=self.bounds(self.path('/World/card'))
        dlo,dhi=self.bounds(self.path('/World/driver'))
        slo,shi=self.bounds(self.path('/World/socket'))
        from omni.physx import get_physx_scene_query_interface
        query=get_physx_scene_query_interface()
        def occlusion(origin,direction,distance):
            hit=query.raycast_closest(Gf.Vec3f(*origin),Gf.Vec3f(*direction),float(distance-.003))
            if not hit['hit']:return None
            body=str(hit.get('rigidBody',hit.get('collision','unknown')))
            if '/card' in body:return 'card'
            if '/driver' in body:return 'driver'
            if 'Arm/' in body:return 'robot'
            return 'fixture'
        optical=optics.response(origin,rot,spec.REFLECTOR,[],occlusion=occlusion)
        # The physical output indicator is visible to every camera; no score lamp.
        led=UsdShade.Shader(self.stage.GetPrimAtPath(self.path('/World/led_material/surface')))
        color=Gf.Vec3f(1.,.7,.01) if optical['on'] else Gf.Vec3f(.025,.02,0.)
        led.GetInput('diffuseColor').Set(color);led.GetInput('emissiveColor').Set(color)
        mlo,mhi=self.bounds(self.path('/World/mount'))
        distance=lambda lo,hi,a,b:float(np.linalg.norm(np.maximum(0,np.maximum(lo-b,a-hi))))
        # Query actual robot shapes against each mount component's oriented
        # 5 mm margin. A single AABB around an angled gripper fills empty space
        # between its palm and fingers and can falsely reject the test pose.
        import carb
        near_robot=[]
        def overlap(hit):
            body=str(hit.rigid_body)
            if 'Arm/' in body:near_robot.append(body)
            return True
        orientation=matrix.ExtractRotationQuat()
        q=carb.Float4(*map(float,orientation.GetImaginary()),float(orientation.GetReal()))
        for local,size in [((-.006,0,0),(.030,.0235,.002)),
                           ((.009,0,.02425),(.002,.019,.0465)),
                           ((.012,.006,.006),(.030,.004,.024)),
                           ((0,.010,0),(.024,.005,.024)),
                           (spec.SENSOR_CENTER_LOCAL,spec.SENSOR_SIZE)]:
            center=matrix.Transform(Gf.Vec3d(*local))
            half=np.asarray(size)/2+.005
            query.overlap_box(carb.Float3(*map(float,half)),carb.Float3(*map(float,center)),q,overlap,False)
        clamp_matrix=self.matrix(self.path('/World/clamp_body'))
        clamp_q=clamp_matrix.ExtractRotationQuat()
        query.overlap_box(carb.Float3(.0115+.005,.02115+.005,.0115+.005),
                          carb.Float3(*map(float,clamp_matrix.Transform(Gf.Vec3d(0,.00385,0)))),
                          carb.Float4(*map(float,clamp_q.GetImaginary()),float(clamp_q.GetReal())),overlap,False)
        hands_clear=not near_robot
        near_card=[]
        def overlap_card(hit):
            body=str(hit.rigid_body)
            if 'Arm/' in body:near_card.append(body)
            return True
        card_matrix=self.matrix(self.path('/World/card'))
        card_q=card_matrix.ExtractRotationQuat()
        query.overlap_box(carb.Float3(*map(float,np.array(spec.CARD_SIZE)/2+.005)),
                          carb.Float3(*map(float,card_matrix.ExtractTranslation())),
                          carb.Float4(*map(float,card_q.GetImaginary()),float(card_q.GetReal())),
                          overlap_card,False)
        blo,bhi=self.bounds(self.path('/World/clamp_body'))
        tool_clear=min(distance(lo,hi,a,b) for lo,hi in [(mlo,mhi),(blo,bhi)]
                       for a,b in [(dlo,dhi),(slo,shi)])>=.005
        all_bounds={name:self.bounds(self.path(path)) for name,path in self.objects.items()}
        inside=all(np.all(lo>=spec.WORKSPACE[0]) and np.all(hi<=spec.WORKSPACE[1]) for lo,hi in all_bounds.values())
        position=np.asarray(matrix.ExtractTranslation())
        post_matrix=self.matrix(self.path('/World/clamp_body'))
        anchor=np.asarray(post_matrix.Transform(Gf.Vec3d(*spec.PLATE_OFFSET)))
        captive=np.asarray(matrix.Transform(Gf.Vec3d(0,.010,0)))
        intact=np.linalg.norm(captive-anchor)<.002
        parked=bool(np.all(clo[:2]>=np.array(spec.CARD_START[:2])-.0375) and np.all(chi[:2]<=np.array(spec.CARD_START[:2])+.0375) and abs(clo[2]-(spec.TABLE_Z+.001))<.003)
        # Visible indexes measure elastic deflection of the calibrated tool.
        # No bolt angle endpoint grants the fastening proof.
        socket_matrix=self.matrix(self.path('/World/socket'))
        handle_matrix=self.matrix(self.path('/World/driver'))
        bolt_matrix=self.matrix(self.path('/World/bolt'))
        sr=np.asarray(socket_matrix.ExtractRotationMatrix()).T
        hr=np.asarray(handle_matrix.ExtractRotationMatrix()).T
        br=np.asarray(bolt_matrix.ExtractRotationMatrix()).T
        relative=sr.T@hr
        slip=math.degrees(math.atan2(relative[1,0],relative[0,0]))
        slip_step=-clamp.angle_delta(slip,self.previous_slip)
        self.previous_slip=slip
        nut=self.clamp_data['bolt_angle_deg']
        nut_motion=clamp.angle_delta(nut,self.previous_nut)
        self.previous_nut=nut
        offset=br.T@(np.asarray(socket_matrix.ExtractTranslation())-np.asarray(bolt_matrix.ExtractTranslation()))
        engaged=bool(self.socket_key_active and np.linalg.norm(offset[:2])<.001 and -.006<offset[2]<.00075
                     and np.dot(sr[:,2],br[:,2])>math.cos(math.radians(3)))
        torque=-math.radians(slip)*spec.DRIVER_SPRING_NM_RAD
        closed=nut<=-1.  # Exclude reset numerical noise; torque proof establishes retention.
        frame=self.matrix(self.path('/World/frame'))
        up=np.asarray(frame.ExtractRotationMatrix())[2,:]
        tilt=math.degrees(math.acos(float(np.clip(up[2]/np.linalg.norm(up),-1,1))))
        shortcut=any(np.linalg.norm(np.asarray(self.matrix(self.path('/World/'+name)).ExtractTranslation())-pos)>.002
                     for name,pos in self.fixed_positions.items())
        evidence=Evidence(optical['on'],optical['geometric_hits']>=7 and optical['blocked_by']==['card'],closed,hands_clear,parked,tool_clear,yaw,tilt,bool(inside),bool(intact),bool(shortcut),not near_card,torque,self.peak_tool_torque,float(slip),float(nut_motion),float(nut),engaged)
        diagnostics={'sensor_xyz':position.tolist(),'sensor_yaw_deg':yaw,'sensor_rotation':rot.tolist(),'card_xyz':list(self.matrix(self.path('/World/card')).ExtractTranslation()),'driver_xyz':list(self.matrix(self.path('/World/driver')).ExtractTranslation()),'driver_matrix':np.asarray(self.matrix(self.path('/World/driver'))).tolist(),'optical':optical,'clamp':self.clamp_data,'clutch_relative_deg':slip,'socket_offset_local_m':offset.tolist(),'robot_inside_5mm_margin':sorted(set(near_robot)),'pad_centers_usd':[list(self.matrix(p).ExtractTranslation()) for p in self.pads]}
        # Independent physics diagnostic: input contact moment about the tool
        # axis. This remains evaluator-only and never reaches either policy.
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        headers, contacts = get_physx_simulation_interface().get_contact_report()
        moment = np.zeros(3)
        origin = np.asarray(handle_matrix.Transform(Gf.Vec3d(0.,0.,spec.DRIVER_HEAD_Z)))
        count = 0
        unexpected_tool_contacts=[]
        for header in headers:
            paths = [str(PhysicsSchemaTools.intToSdfPath(v)) for v in [header.collider0,header.collider1]]
            if header.num_contact_data:
                for i,p in enumerate(paths):
                    other=paths[1-i]
                    if p.startswith(self.path('/World/bolt')+'/') and not any(other.startswith(self.path('/World/'+n)+'/') for n in ['socket','clamp_body']):
                        # Holding the nut itself could imitate a tightening
                        # witness without producing any fastener preload.
                        unexpected_tool_contacts.append(other)
                    if p.startswith(self.path('/World/socket')+'/') and not any(other.startswith(self.path('/World/'+n)+'/') for n in ['bolt','driver']):
                        unexpected_tool_contacts.append(other)
                    if p.startswith(self.path('/World/driver')+'/') and 'Arm/' not in other and not other.startswith(self.path('/World/socket')+'/'):
                        unexpected_tool_contacts.append(other)
            match = [p.startswith(self.path('/World/driver')+'/') for p in paths]
            if sum(match) != 1:
                continue
            sign = 1. if match[0] else -1.
            for index in range(header.contact_data_offset,header.contact_data_offset+header.num_contact_data):
                contact = contacts[index]
                moment += np.cross(np.asarray(contact.position)-origin,sign*np.asarray(contact.impulse))*spec.PHYSICS_HZ
                count += 1
        diagnostics['driver_contact_moment_nm'] = moment.tolist()
        diagnostics['driver_axial_contact_moment_nm'] = float(moment@sr[:,2])
        diagnostics['driver_contact_points'] = count
        diagnostics['unexpected_tool_contacts'] = sorted(set(unexpected_tool_contacts))
        evidence.torque_path_clear=not unexpected_tool_contacts
        diagnostics['object_bounds']={name:[lo.tolist(),hi.tolist()] for name,(lo,hi) in all_bounds.items()}
        return evidence,diagnostics

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (14,) or not np.isfinite(action).all():
            raise ValueError('Expected 14 finite joint/gripper targets')
        self.update_clamp()
        previous_physics_steps=self.physics_callback_count
        self.env.step(self.torch.tensor(action[None,:], device=self.base.device))
        if self.physics_callback_count-previous_physics_steps != spec.PHYSICS_HZ//spec.CONTROL_HZ:
            raise RuntimeError('Passive clamp material did not update at the native physics rate')
        measured=self.joints()
        if not np.isfinite(measured).all() or np.max(np.abs(measured))>7.:
            raise RuntimeError('Native robot dynamics diverged; this is invalid evaluation evidence')
        self.update_clamp()
        evidence, diagnostics = self.measure()
        score = self.scorer.update(evidence)
        row = {'step':self.steps,'score':score,'dwell':self.scorer.dwell,'invalid':self.scorer.invalid,
               'evidence':asdict(evidence),'joint_targets':action.tolist(),'measured_joints':self.joints().tolist(),
               'physics_steps':self.physics_callback_count,**diagnostics}
        self.trace.write(json.dumps(row)+'\n')
        self.trace.flush()
        self.steps += 1
        # Render the updated physical LED without advancing physics.
        self.base.sim.render()
        from isaaclab.sensors.sensor_base import SensorBase
        cameras=[self.base.scene[key] for key in CAMERAS.values()]
        for camera in cameras:
            camera._invalidate_buffers()
        SensorBase.update_batch(cameras,0.)
        self.save_frames(f'{self.steps:05d}')
        self.last = row
        if self.steps % 50 == 0:
            print('PHOTO_STEP',self.steps,'score',score,'invalid',self.scorer.invalid,flush=True)
        return row

    def close(self):
        result = {'steps':self.steps,'score':self.scorer.score,'invalid':self.scorer.invalid,
                  'dwell':self.scorer.dwell,'wall_seconds':time.monotonic()-self.started,
                  'control_hz':spec.CONTROL_HZ,'physics_hz':spec.PHYSICS_HZ}
        (self.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        self.trace.close()
        if self.env is not None:
            self.base.sim.backend_manager.step=self.original_physics_step
            self.stage.Export(str(self.output/'terminal.usda'))
            self.env.close()
            self.env = None
