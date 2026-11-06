"""Freely suspended initial-state fixture. NOT an assembly demonstration."""
import argparse,json,time
from pathlib import Path
import numpy as np

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--source',required=True);ap.add_argument('--state-log',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 from inspect_robots import Scene
 from chain_task.embodiment import ChainEmbodiment,FEEDBACK,array
 from yam_arena.ik import matrix_to_quat_xyzw
 e=ChainEmbodiment(task=a.source,device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360,object_jitter=0.);scene=Scene(id='free-link-settling-fixture',instruction='Diagnostic freely suspended link; no assembly credit.');start=time.monotonic();state=json.loads([x for x in a.state_log.read_text().splitlines() if x.startswith('CHAIN_STATE ')][-1].split(' ',2)[2]);rows=[]
 try:
  e.reset(scene,seed=17)
  from pxr import Sdf
  originals={}
  for kind in ['link','gate']:
   prim=e._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+kind+'1');originals[kind]={n:prim.GetAttribute('physxRigidBody:'+n).Get() for n in ['angularDamping','linearDamping','solverPositionIterationCount','solverVelocityIterationCount','maxDepenetrationVelocity']}
  print('ACTUAL_AUTHORED_PROPERTIES',json.dumps(originals),flush=True)
  for damping in [None,5.,15.]:
   e.reset(scene,seed=17);e._audit_enabled=False;e._base.cfg.sim.render_interval=10**9
   for kind in ['link','gate']:
    name=kind+'1';prim=e._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+name)
    if damping is not None:prim.CreateAttribute('physxRigidBody:angularDamping',Sdf.ValueTypeNames.Float).Set(damping)
    pose=np.r_[state['positions'][name],matrix_to_quat_xyzw(np.array(state['rotations'][name]))];obj=e._base.scene[name];obj.write_root_pose_to_sim_index(root_pose=e._torch.tensor(pose,dtype=e._torch.float32,device=e._base.device).unsqueeze(0));obj.write_root_velocity_to_sim_index(root_velocity=e._torch.zeros_like(obj.data.default_root_vel.torch))
   samples=[];timer=0.;maximum=0.
   for i in range(600):
    e._env.step(e._ready);e._t+=e._base.step_dt;e._read();speeds={}
    for name in ['link1','gate1']:
     v=array(e._base.arena_world.get_root_linear_velocity_w(name))[0];w=array(e._base.arena_world.get_root_angular_velocity_w(name))[0];speeds[name]=dict(linear_m_s=float(np.linalg.norm(v)),angular_rad_s=float(np.linalg.norm(w)))
    quiet=all(v['linear_m_s']<=.01 and v['angular_rad_s']<=.2 for v in speeds.values());timer=timer+e._base.step_dt if quiet else 0.;maximum=max(maximum,timer)
    if i%20==19:samples.append(dict(seconds=(i+1)*e._base.step_dt,speeds=speeds,quiet_seconds=timer,position=FEEDBACK['poses']['link1'].tolist()))
   row=dict(angular_damping_per_s=damping if damping is not None else 'authored',maximum_continuous_quiet_seconds=maximum,final_quiet_seconds=timer,samples=samples);rows.append(row);print('SETTLING_FIXTURE',json.dumps({k:v for k,v in row.items() if k!='samples'}),flush=True)
  for kind in ['link','gate']:
   prim=e._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+kind+'1')
   for name,value in originals[kind].items():
    if value is not None:prim.GetAttribute('physxRigidBody:'+name).Set(value)
  e.reset(scene,seed=17);(a.output/'result.json').write_text(json.dumps(dict(diagnostic_only=True,not_an_assembly_demo=True,initial_state='Previously released real link poses, initialized at zero velocity; no added constraints or kinematic bodies. Scorer/audit disabled only for this explicitly interlinked initialization fixture.',authored_properties=originals,variants=rows,wall_seconds=time.monotonic()-start),indent=2)+'\n')
 finally:e.close()
if __name__=='__main__':main()
