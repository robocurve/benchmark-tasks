"""The full-size hand-truck scene inside an Isaac Lab-Arena environment.

Policy observations are produced separately from evaluator measurements. Reset
rebuilds the environment, including every body, wheel joint, scorer counter and camera product.
"""
from pathlib import Path
from dataclasses import asdict
import json
import hashlib
import math
import time
import numpy as np
import spec
from scipy.spatial.transform import Rotation
from scorer import Evidence, Scorer
from wheel_support import support_gap

APP = None
CAMERAS = {'overhead_cam': 'top_cam', 'left_wrist_cam': 'left_cam', 'right_wrist_cam': 'right_cam'}


def array(value):
    value = value.torch if hasattr(value, 'torch') else value
    return value.detach().cpu().numpy() if hasattr(value, 'detach') else np.asarray(value)


class TrolleyWorld:
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
        from yam_arena import constants, ik
        import task_station as station
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
        rig = station.rig()
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
        self.fixed_positions={name:np.asarray(self.matrix(self.path('/World/'+name)).ExtractTranslation()) for name in ['sill','floor']}
        for _ in range(4):self.base.sim.render()
        for name in CAMERAS.values():
            # Discard the RGB cache captured before the final pose restoration.
            self.base.scene[name].reset()
            self.base.scene[name].update(0.,force_recompute=True)
        self.stage.Export(str(self.output/'reset.usda'))
        self.previous_poses=None
        self.save_frames('00000')
        (self.output/'reset-cameras.json').write_text(json.dumps({name:{'shape':list(im.shape),'sha256':hashlib.sha256(im.tobytes()).hexdigest()} for name,im in self.images().items()},indent=2)+'\n')

    def path(self, original):
        return original.replace('/World', self.prefix, 1)

    def matrix(self, path):
        from pxr import UsdGeom
        return UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)

    def bounds(self, path):
        from pxr import Usd,UsdGeom,UsdPhysics,Gf
        if path.startswith(self.prefix):
            # Transform actual collider vertices. Rotating a cylinder's local
            # bounding box spuriously places its corners below the floor.
            points=[]
            for prim in Usd.PrimRange(self.stage.GetPrimAtPath(path)):
                if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
                if prim.IsA(UsdGeom.Mesh):local=UsdGeom.Mesh(prim).GetPointsAttr().Get()
                elif prim.IsA(UsdGeom.Cube):
                    h=UsdGeom.Cube(prim).GetSizeAttr().Get()/2
                    local=[(x,y,z) for x in [-h,h] for y in [-h,h] for z in [-h,h]]
                else:continue
                matrix=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                points.extend([list(matrix.Transform(Gf.Vec3d(*v))) for v in local])
            a=np.asarray(points)
            return a.min(0),a.max(0)
        bound=UsdGeom.BBoxCache(0,['default','render','proxy','guide']).ComputeWorldBound(self.stage.GetPrimAtPath(path)).ComputeAlignedRange()
        return np.array(bound.GetMin()),np.array(bound.GetMax())

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

    def measure(self):
        from pxr import Gf
        poses={name:self.prop_views['/World/'+name].get_transforms()[0].copy() for name in self.objects}
        position=poses['cart'][:3];rotation=Rotation.from_quat(poses['cart'][3:]).as_matrix()
        roll,pitch,yaw=Rotation.from_matrix(rotation).as_euler('xyz',degrees=True)
        case_rotation=rotation.T@Rotation.from_quat(poses['case'][3:]).as_matrix()
        case_position=rotation.T@(poses['case'][:3]-position)
        size=np.array(spec.CASE_SIZE)
        lower=np.array([[x,y,-size[2]/2] for x in [-size[0]/2,size[0]/2] for y in [-size[1]/2,size[1]/2]])@case_rotation.T+case_position
        plate_top=-spec.AXLE[2]+spec.PLATE_SIZE[2]
        case_on=bool(np.all(lower[:,0]>=-.003) and np.all(lower[:,0]<=spec.PLATE_SIZE[0]+.003)
                     and np.all(np.abs(lower[:,1])<=spec.PLATE_SIZE[1]/2+.003)
                     and np.all(np.abs(lower[:,2]-plate_top)<=.015))
        case_aligned=bool(np.linalg.norm(case_position-(np.array(spec.CASE_START)-np.array(spec.AXLE)))<=.020
                          and Rotation.from_matrix(case_rotation).magnitude()<=math.radians(5))
        bounds={name:self.bounds(self.path('/World/'+name)) for name in self.objects}
        low,high=map(np.asarray,spec.WORKSPACE)
        inside=all(np.all(a>=low) and np.all(b<=high) for a,b in bounds.values())
        intact=all(np.linalg.norm(np.asarray(self.matrix(self.path('/World/'+name)).ExtractTranslation())-p)<.002 for name,p in self.fixed_positions.items())
        for side,sign in [('left',1),('right',-1)]:
            centre=rotation.T@(poses['wheel_'+side][:3]-position)
            intact &= np.linalg.norm(centre-[0,sign*spec.WHEEL_Y,0])<=.002
        clear=True
        for hand in self.hands:
            a,b=self.bounds(hand)
            for c,d in bounds.values():
                if np.all(b+.005>=c) and np.all(d+.005>=a):clear=False
        velocities={name:self.prop_views['/World/'+name].get_velocities()[0].tolist() for name in self.objects}
        stable=all(np.linalg.norm(v[:3])<=.010 and np.linalg.norm(v[3:])<=math.radians(2) for v in velocities.values())
        bottom=np.array([[x,y,-spec.AXLE[2]] for x in [0,spec.PLATE_SIZE[0]] for y in [-spec.PLATE_SIZE[1]/2,spec.PLATE_SIZE[1]/2]])@rotation.T+position
        e=Evidence(float(position[0]),max(float(bounds[n][1][0]) for n in ['wheel_left','wheel_right']),
                   max(float(b[0]) for a,b in bounds.values()),float(pitch),float(roll),float(yaw),float(bottom[:,2].min()),
                   case_on,case_aligned,clear,bool(stable),bool(inside),bool(intact),support_gap(poses))
        return e,{'poses':{k:v.tolist() for k,v in poses.items()},'velocities':velocities,
                  'case_local_position':case_position.tolist(),'case_bottom_local':lower.tolist(),
                  'object_bounds':{n:[a.tolist(),b.tolist()] for n,(a,b) in bounds.items()}}

    def step(self,action):
        action=np.asarray(action,dtype=np.float32)
        if action.shape!=(14,) or not np.isfinite(action).all():raise ValueError('Expected 14 finite joint targets')
        self.env.step(self.torch.tensor(action[None,:],device=self.base.device))
        measured=self.joints()
        if not np.isfinite(measured).all() or np.max(np.abs(measured))>7.:raise RuntimeError('Native robot dynamics diverged')
        evidence,diagnostics=self.measure();score=self.scorer.update(evidence)
        row={'step':self.steps,'score':score,'dwell':self.scorer.dwell,'invalid':self.scorer.invalid,
             'evidence':asdict(evidence),'joint_targets':action.tolist(),'measured_joints':measured.tolist(),**diagnostics}
        row['applied_joint_effort']={s:array(self.base.scene[s+'_robot'].data.applied_torque)[0].tolist() for s in ['left','right']}
        self.trace.write(json.dumps(row)+'\n');self.trace.flush();self.steps+=1
        self.save_frames(f'{self.steps:05d}');self.last=row
        if self.steps%50==0:print('TROLLEY_STEP',self.steps,'score',score,'invalid',self.scorer.invalid,flush=True)
        return row

    def close(self):
        result={'steps':self.steps,'score':self.scorer.score,'invalid':self.scorer.invalid,'dwell':self.scorer.dwell,
                'wall_seconds':time.monotonic()-self.started,'control_hz':spec.CONTROL_HZ,'physics_hz':spec.PHYSICS_HZ}
        (self.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');self.trace.close()
        if self.env is not None:
            self.stage.Export(str(self.output/'terminal.usda'));self.env.close();self.env=None
