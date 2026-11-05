"""Physical world-anchored revolute wheels with code-independent soft detents."""
from pathlib import Path
import os
import atexit
import math
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

FEEDBACK={}

class RotaryEmbodiment(YamArenaEmbodiment):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        from dataclasses import replace
        import hashlib
        root=Path(__file__).resolve().parent.parent;h=hashlib.sha256()
        for p in sorted(root.rglob('*')):
            if p.is_file() and p.suffix in ('.py','.usdc','.png','.toml') and '__pycache__' not in p.parts:h.update(str(p.relative_to(root)).encode()+b'\0'+p.read_bytes())
        self.info=replace(self.info,environment_revision=h.hexdigest())

    def _ensure_sim(self,goal):
        from yam_arena import inspect_embodiment as base
        if base._APP is None:
            from isaaclab.app import AppLauncher
            os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
            base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        import isaaclab.sim as sim
        original=scene.task_object
        def spawn(spec):
            if spec.name in ('box','lid') or spec.name.startswith('wheel_'):
                asset=spec.name+'.usdc' if spec.name in ('box','lid') else 'wheel.usdc'
                addon={'physics_material':scene._grippy(),'collision_props':sim.CollisionPropertiesCfg(contact_offset=.0003,rest_offset=0.)}
                if spec.free:addon['rigid_props']=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=32,solver_velocity_iteration_count=4)
                return Object(name=spec.name,usd_path=str(Path(__file__).resolve().parent.parent/'assets'/asset),object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2))),spawn_cfg_addon=addon)
            return original(spec)
        scene.task_object=spawn
        try:super()._ensure_sim(goal)
        finally:scene.task_object=original
        from pxr import UsdPhysics,Gf,Sdf
        stage=self._base.sim.stage
        for i,pos in enumerate(self.task.module.CENTERS):
            joint=UsdPhysics.RevoluteJoint.Define(stage,f'/World/envs/env_0/WheelJoint{i}')
            joint.CreateBody1Rel().SetTargets([Sdf.Path(f'/World/envs/env_0/wheel_{i}')])
            joint.CreateAxisAttr('Z');joint.CreateLocalPos0Attr(Gf.Vec3f(*pos));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
            joint.CreateLocalRot0Attr(Gf.Quatf(1,0,0,0));joint.CreateLocalRot1Attr(Gf.Quatf(1,0,0,0));joint.CreateExcludeFromArticulationAttr(True)

        joint=UsdPhysics.RevoluteJoint.Define(stage,'/World/envs/env_0/LidJoint')
        joint.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/lid')]);joint.CreateAxisAttr('Y')
        joint.CreateLocalPos0Attr(Gf.Vec3f(*self.task.module.HINGE));joint.CreateLocalPos1Attr(Gf.Vec3f(-.07,0,0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1,0,0,0));joint.CreateLocalRot1Attr(Gf.Quatf(1,0,0,0));joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateLowerLimitAttr(0.);joint.CreateUpperLimitAttr(0.)
        self._lid_joint=joint;self._unlocked=False

    def reset(self,scene,*,seed=None):
        from rotary_box.scoring import History
        self.task.module.HISTORY=History(self.task.module.CODE,self.task.module.CENTERS)
        if hasattr(self,'_lid_joint'):self._lid_joint.GetLowerLimitAttr().Set(0.)
        self._unlocked=False
        super().reset(scene,seed=seed)
        for _ in range(48):self._env.step(self._ready)
        for name in ('top_cam','left_cam','right_cam'):self._base.scene[name].reset()
        self._read()
        return self._observation()

    def _read(self):
        from yam_arena.urdf import quat_xyzw_to_matrix
        angles=[];positions=[];tilts=[];speeds=[]
        for i in range(len(self.task.module.CODE)):
            name=f'wheel_{i}';pose=self._base.arena_world.get_pose_w(name)[0].detach().cpu().numpy();r=quat_xyzw_to_matrix(pose[3:])
            angles.append(float(math.atan2(r[1,0],r[0,0])));positions.append(pose[:3].tolist());tilts.append(float(math.acos(np.clip(r[2,2],-1,1))))
            speeds.append(float(self._base.arena_world.get_root_angular_velocity_w(name)[0,2].item()))
        FEEDBACK.update(angles=angles,positions=positions,tilts=tilts,speeds=speeds)
        return angles,positions,tilts,speeds

    def _task_step(self):
        from yam_arena import ik,station
        from yam_arena.constants import FINGER_OPEN
        angles,positions,tilts,speeds=self._read()
        clear=[True]*len(angles)
        for side,key in (('left','left_robot'),('right','right_robot')):
            arm,fingers=self._ids[key];q=self._base.scene[key].data.joint_pos.torch[0];tcp,_=ik.tcp_pose(q[arm].cpu().numpy(),station.BASES[side])
            for i,pos in enumerate(positions):
                clear[i] &= np.linalg.norm(tcp-pos)>.08
        h=self.task.module.HISTORY;h.update(angles,positions,tilts,speeds,clear)
        for i,(a,speed) in enumerate(zip(angles,speeds)):
            # A periodic potential with all ten digit positions equally stable.
            # This force never depends on the supplied combination code.
            torque=-.0005*math.sin(10*a)-.0005*speed
            t=self._torch.tensor([[[0.,0.,torque]]],dtype=self._torch.float32,device=self._base.device)
            self._base.scene[f'wheel_{i}'].permanent_wrench_composer.set_forces_and_torques(torques=t,is_global=True)
        from rotary_box.scoring import error,TOLERANCE
        from yam_arena.urdf import quat_xyzw_to_matrix
        all_correct=all(abs(error(a,d))<=TOLERANCE and not bad for a,d,bad in zip(angles,h.code,h.invalid))
        if all_correct and not self._unlocked:
            self._unlocked=True;self._lid_joint.GetLowerLimitAttr().Set(-110.)
            print('LATCH_UNLOCKED',round(self._t,2),flush=True)
        pose=self._base.arena_world.get_pose_w('lid')[0].detach().cpu().numpy();rot=quat_xyzw_to_matrix(pose[3:])
        opening=math.atan2(rot[2,0],rot[0,0]);hinge=pose[:3]+rot@np.array([-.07,0,0])
        h.update_lid(opening,float(np.linalg.norm(hinge-self.task.module.HINGE)),float(math.acos(np.clip(rot[1,1],-1,1))),self._unlocked)
        # Light hinge damping; opening torque must come from robot contact.
        speed=float(self._base.arena_world.get_root_angular_velocity_w('lid')[0,1].item())
        t=self._torch.tensor([[[0.,-.004*speed,0.]]],dtype=self._torch.float32,device=self._base.device)
        self._base.scene['lid'].permanent_wrench_composer.set_forces_and_torques(torques=t,is_global=True)
        FEEDBACK.update(opening=opening,unlocked=self._unlocked)
        result=super()._task_step()
        if round(self._t*self.info.control_hz)%20==0:print('ROTARY_STATE',round(self._t,2),'angles_deg',np.round(np.degrees(angles),3).tolist(),'centers',np.round(positions,4).tolist(),'speed',np.round(speeds,4).tolist(),'clear',clear,'retained',h.verified,'invalid',h.invalid,'opening_deg',round(math.degrees(opening),2),'unlocked',self._unlocked,'bypassed',h.bypassed,'score',result.info.get('score'),flush=True)
        return result

def make_embodiment(**kwargs):return RotaryEmbodiment(**kwargs)
