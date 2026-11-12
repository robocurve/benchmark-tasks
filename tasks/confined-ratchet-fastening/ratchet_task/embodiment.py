"""Native YAM cameras and actions with authored open-bore USD assets."""
from pathlib import Path
from dataclasses import replace
import hashlib
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from ratchet_task.config import *
ROOT=Path(__file__).resolve().parents[1]
FEEDBACK={}
def array(v):
    v=v.torch if hasattr(v,'torch') else v
    return v.detach().cpu().numpy() if hasattr(v,'detach') else np.asarray(v)
class RatchetEmbodiment(YamArenaEmbodiment):
    def __init__(self,walls=False,mechanisms=False,physics_hz=PHYSICS_HZ,solver_type=1,solver_velocity=SOLVER_VELOCITY,**kwargs):
        kwargs.setdefault('object_jitter',0.)
        super().__init__(**kwargs)
        if self.object_jitter:raise ValueError('Joint-consistent layout requires object_jitter=0')
        self.walls=bool(walls);self.mechanisms=bool(mechanisms);self.physics_hz=int(physics_hz);self.solver_type=int(solver_type);self.solver_velocity=int(solver_velocity)
        self.hardware=None;self._hardware_active=False;self._raw_physics_step=None
        if self.walls:
            from yam_arena.task_file import Box
            self.task=replace(self.task,objects=self.task.objects+(Box('walls',pos=(float(AXIS[0]),float(AXIS[1]),TABLE),size=(.09,.12,.162)),))
        h=hashlib.sha256()
        for p in sorted(ROOT.rglob('*')):
            if p.is_file() and p.suffix in ('.py','.usda','.toml','.png') and not (len(p.relative_to(ROOT).parts)>2 and p.relative_to(ROOT).parts[:2]==('assets','vendor') and p.relative_to(ROOT).parts[2].startswith('handTool_')):h.update(p.relative_to(ROOT).as_posix().encode()+b'\0'+p.read_bytes())
        import json
        h.update(json.dumps(dict(walls=self.walls,mechanisms=self.mechanisms,physics_hz=self.physics_hz,solver_type=self.solver_type,solver_velocity=self.solver_velocity,control_hz=self.info.control_hz,camera=[self.cam_width,self.cam_height]),sort_keys=True).encode())
        self.info=replace(self.info,name='yam_confined_ratchet',environment_revision=h.hexdigest())
    def _ensure_sim(self,goal):
        if self._env is not None:return
        from yam_arena import inspect_embodiment as base
        if base._APP is None:
            import atexit
            from isaaclab.app import AppLauncher
            base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene,embodiment as robots
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        import isaaclab.sim as sim
        old_object,old_callback,old_robot=scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment
        def spawn(spec):
            if spec.name not in self.task.names:return old_object(spec)
            props=dict(collision_props=sim.CollisionPropertiesCfg(contact_offset=.0001,rest_offset=0.),physics_material=sim.RigidBodyMaterialCfg(static_friction=.55,dynamic_friction=.4))
            if spec.name=='handle':props['physics_material']=sim.RigidBodyMaterialCfg(static_friction=.9,dynamic_friction=.7)
            if spec.free:props['rigid_props']=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=SOLVER_POSITION,solver_velocity_iteration_count=self.solver_velocity,max_depenetration_velocity=.2,linear_damping=.02,angular_damping=.02)
            return Object(name=spec.name,usd_path=str(ROOT/'assets'/f'{spec.name}.usda'),object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,initial_pose=Pose(position_xyz=tuple(map(float,spec.pos))),spawn_cfg_addon=props)
        def callback(hz=None):
            original=old_callback(hz)
            def configure(cfg):
                cfg=original(cfg);cfg.sim.dt=1/self.physics_hz;cfg.decimation=round(self.physics_hz/self.info.control_hz);cfg.sim.render_interval=cfg.decimation;cfg.sim.physics.enable_ccd=True;cfg.sim.physics.solver_type=self.solver_type;cfg.episode_length_s=MAX_STEPS/CONTROL_HZ+10.
                return cfg
            return configure
        owner=self
        class NativeResolutionRobot(old_robot):
            def __init__(robot,**options):
                options.update(cam_width=owner.cam_width,cam_height=owner.cam_height,robot_contact_offset=.0002)
                super().__init__(**options)
        scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=spawn,callback,NativeResolutionRobot
        try:super()._ensure_sim(goal)
        finally:scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=old_object,old_callback,old_robot
        # Only the scored seating pair uses a 10 um candidate-contact offset.
        # Rest offset remains zero: the physical seat is the authored surface.
        from pxr import PhysxSchema
        for path in ['/World/envs/env_0/bolt/HeadContact']+[
                f'/World/envs/env_0/fixture/Bracket/Sector{i:02}' for i in range(BRACKET_SECTORS)]:
            prim=self._base.sim.stage.GetPrimAtPath(path)
            if not prim:raise RuntimeError('Missing seating collider '+path)
            props=PhysxSchema.PhysxCollisionAPI.Apply(prim)
            props.CreateContactOffsetAttr(SEAT_CONTACT_OFFSET);props.CreateRestOffsetAttr(0.)
        import carb
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
        from pxr import UsdPhysics,Sdf
        self.drive_joint=UsdPhysics.FixedJoint.Define(self._base.sim.stage,'/World/envs/env_0/RatchetDriveJoint')
        self.drive_joint.CreateBody0Rel().SetTargets([Sdf.Path('/World/envs/env_0/pawl')])
        self.drive_joint.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/drive')])
        self.drive_joint.CreateCollisionEnabledAttr(False)
        self.drive_joint.CreateExcludeFromArticulationAttr(True)
        self.pawl_joint=UsdPhysics.RevoluteJoint.Define(self._base.sim.stage,'/World/envs/env_0/PawlPivot')
        self.pawl_joint.CreateBody0Rel().SetTargets([Sdf.Path('/World/envs/env_0/handle')]);self.pawl_joint.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/pawl')]);self.pawl_joint.CreateAxisAttr('Z');self.pawl_joint.CreateCollisionEnabledAttr(False);self.pawl_joint.CreateExcludeFromArticulationAttr(True)
        pair=UsdPhysics.Joint.Define(self._base.sim.stage,'/World/envs/env_0/ToolInternalFilter')
        pair.CreateBody0Rel().SetTargets([Sdf.Path('/World/envs/env_0/pawl')]);pair.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/drive')]);pair.CreateCollisionEnabledAttr(False);pair.CreateExcludeFromArticulationAttr(True)
        self._reset_supports(True)
        if self.mechanisms:
            self._raw_physics_step=self._base.sim.step
            def hardware_step(*args,**kwargs):
                if self._hardware_active and self.hardware is not None:self.hardware.step()
                result=self._raw_physics_step(*args,**kwargs)
                if self._hardware_active and self.hardware is not None:self.hardware.after_step()
                return result
            self._base.sim.step=hardware_step
    def _reset_supports(self,enabled):
        # Hold only the started bolt during reset, until its native thread exists.
        # The tool rests in its physical cradle; both arms use the standard READY.
        from pxr import UsdPhysics,Sdf,Gf
        for name,position in [('bolt',BOLT_START)]:
            path='/World/envs/env_0/ResetSupport_'+name
            if not enabled:self._base.sim.stage.RemovePrim(path);continue
            j=UsdPhysics.FixedJoint.Define(self._base.sim.stage,path)
            j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/'+name)])
            j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,position)))
            j.CreateExcludeFromArticulationAttr(True)
    def _read(self):
        from yam_arena import ik,station
        from yam_arena.urdf import quat_xyzw_to_matrix
        poses={};rotations={}
        for n in self.task.names:
            v=array(self._base.arena_world.get_pose_w(n))[0];poses[n]=v[:3];rotations[n]=quat_xyzw_to_matrix(v[3:])
        hands={}
        for side,key in [('left','left_robot'),('right','right_robot')]:
            arm,fingers=self._ids[key];q=array(self._base.scene[key].data.joint_pos)[0];p,r=ik.tcp_pose(q[arm],station.BASES[side]);hands[side]=dict(tcp=p,rotation=r,joints=q[arm],fingers=q[fingers])
        FEEDBACK.update(poses=poses,rotations=rotations,hands=hands)
    def reset(self,scene,*,seed=None):
        self._hardware_active=False
        if self.hardware is not None:self.hardware.contact_subscription=None
        if self._env is not None:
            from pxr import UsdPhysics,Gf
            drive=UsdPhysics.DriveAPI.Get(self.drive_joint.GetPrim(),'angular')
            if drive:drive.GetMaxForceAttr().Set(0.)
            self.drive_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.))
            for name in ['ThreadPitch','ThreadRotation','ThreadTranslation','ThreadGuide','ThreadAngleReference','SeatStop','ThreadCarrier','PawlSpring','PawlGate']:
                self._base.sim.stage.RemovePrim('/World/envs/env_0/'+name)
        if self._env is not None:
            for p in self._base.sim.stage.Traverse():
                if str(p.GetPath()).startswith('/World/envs/env_0/entry_stop') and p.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(True)
        if self._env is not None:self._reset_supports(True)
        obs=super().reset(scene,seed=seed)
        from ratchet_task.scoring import History
        self.history=History();self.task.module.HISTORY=self.history
        if self.mechanisms:
            from ratchet_task.physics import Hardware
            self.hardware=Hardware(self._base.sim.stage,1/self.physics_hz)
            b=self.hardware.pose('bolt')
            self.hardware.capture(b[0],b[2],b[3])
            self._hardware_active=True
        self._reset_supports(False)
        FEEDBACK.clear();self._read()
        if self.mechanisms:
            self.hardware.step();FEEDBACK['hardware']=self.hardware.telemetry
        return obs
    def _task_step(self):
        self._read()
        if self.mechanisms and self.hardware is not None and self.hardware.telemetry:
            from ratchet_task.scoring import State
            f=self.hardware.telemetry;f.update(self.hardware.contact_summary());f['interface_torque']=f['socket_torque_nm'] if f['socket_engaged'] else 0.;FEEDBACK['hardware']=f
            fixture=float(np.linalg.norm(FEEDBACK['poses']['fixture']-[AXIS[0],AXIS[1],TABLE]))
            valid=bool(f['valid']) and abs(f.get('pitch_error_m',0.))<=.0002
            state=State(captured=f['captured'],valid=valid,turns=f['turns'],gap=f.get('seat_gap_m',f['gap']),interface_torque=f['interface_torque'],socket_engaged=f['socket_engaged'],ratchet_cycles=f['ratchet_cycles'],fixture_displacement=fixture,
                        seat_contact=f['seat_contact'],seat_contact_fraction=f['seat_contact_fraction'],seat_force_n=f['seat_force_mean_n'],
                        seat_gap_min_m=f.get('seat_gap_min_m',1.),seat_gap_max_m=f.get('seat_gap_max_m',1.))
            self.history.update(state,1/self.info.control_hz);FEEDBACK.update(score=self.history.score,tightened=self.history.tightened,proof=self.history.proof)
        return super()._task_step()
def make_embodiment(**kwargs):
    kwargs.setdefault('walls',True);kwargs.setdefault('mechanisms',True)
    return RatchetEmbodiment(**kwargs)
