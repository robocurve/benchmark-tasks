"""External-torque gauge and reset checks. Diagnostic fixtures, never a demo."""
import argparse,json,time
from pathlib import Path
import numpy as np

def main():
 p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 from inspect_robots import Scene
 from chain_task.embodiment import ChainEmbodiment,FEEDBACK
 from chain_task.geometry import GATE_OFFSET
 start=time.monotonic();e=ChainEmbodiment(task=a.source,device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360,object_jitter=0.);scene=Scene(id='chain-mechanism-fixture',instruction='Diagnostic gate torque gauge and fresh reset only.');checks=[];rows=[];layouts=[]
 def advance(n=1,torque=None):
  for _ in range(n):
   if torque is not None:
    obj=e._base.scene['gate2'];z=e._torch.zeros((1,1,3),dtype=e._torch.float32,device=e._base.device);v=z.clone();v[0,0,2]=torque;obj.permanent_wrench_composer.set_forces_and_torques(forces=z,torques=v,is_global=True)
   e._env.step(e._ready);e._t+=e._base.step_dt;e._read()
 def valid():e._integrity();return not e._hardware_invalid
 try:
  e.reset(scene,seed=17)
  from pxr import UsdPhysics,Gf
  e._base.cfg.sim.render_interval=10**9;advance(40);original={n:p.copy() for n,p in FEEDBACK['poses'].items()};checks.append(dict(name='quiet_live_audit_240_hz',passed=valid() and not e.audit.invalid and e.audit.steps>=481))
  for name,mutate in [('disabled_joint',lambda:e.joints[0].GetJointEnabledAttr().Set(False)),('altered_joint_frame',lambda:e.joints[0].GetLocalPos0Attr().Set(Gf.Vec3f(.026,-.0195,0))),('altered_spring',lambda:UsdPhysics.DriveAPI(e.joints[0].GetPrim(),'angular').GetStiffnessAttr().Set(1.)),('kinematic_frame',lambda:UsdPhysics.RigidBodyAPI(e._base.sim.stage.GetPrimAtPath('/World/envs/env_0/link0')).GetKinematicEnabledAttr().Set(True))]:
   mutate();checks.append(dict(name=name+'_invalidates',passed=not valid()));e.reset(scene,seed=17);checks.append(dict(name=name+'_reset_restores',passed=valid() and not e.audit.invalid));e._base.cfg.sim.render_interval=10**9
  e.reset(scene,seed=17);e._base.cfg.sim.render_interval=10**9;advance(40)
  difference=max(float(np.linalg.norm(FEEDBACK['poses'][n]-original[n])) for n in original)
  checks.append(dict(name='same_seed_settled_reset_reproduces',passed=difference<.0001,maximum_position_difference_m=difference))
  for torque in [0.,.002,.005,.010,.020,0.]:
   # Move an uncluttered diagnostic fixture above the table; this is explicitly
   # NOT an evaluation initial state and earns no benchmark credit.
   if torque==0. and not rows:
    e.reset(scene,seed=17);e._base.cfg.sim.render_interval=10**9
    for name,position in [('link2',np.array([.30,-.20,.95])),('gate2',np.array([.30,-.20,.95])+GATE_OFFSET)]:
     obj=e._base.scene[name];pose=np.r_[position,[0,0,0,1]];obj.write_root_pose_to_sim_index(root_pose=e._torch.tensor(pose,dtype=e._torch.float32,device=e._base.device).unsqueeze(0));obj.write_root_velocity_to_sim_index(root_velocity=e._torch.zeros_like(obj.data.default_root_vel.torch))
    UsdPhysics.RigidBodyAPI(e._base.sim.stage.GetPrimAtPath('/World/envs/env_0/link2')).GetKinematicEnabledAttr().Set(True)
   advance(100,torque);row=dict(external_torque_Nm=torque,angle_deg=float(np.degrees(FEEDBACK['angles']['2'])),equivalent_tip_force_N=torque/.039);rows.append(row);print('CHAIN_GAUGE',json.dumps(row),flush=True)
  UsdPhysics.RigidBodyAPI(e._base.sim.stage.GetPrimAtPath('/World/envs/env_0/link2')).GetKinematicEnabledAttr().Set(False);e.reset(scene,seed=17);checks.append(dict(name='post_gauge_normal_reset',passed=valid() and not e.audit.invalid))
  result=dict(diagnostic_only=True,fixture='Main frame temporarily clamped above table; external torque on gate2 only. No assembly or credited scores.',checks=checks,torque_gauge=rows,wall_seconds=time.monotonic()-start);(a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
 finally:e.close()
if __name__=='__main__':main()
