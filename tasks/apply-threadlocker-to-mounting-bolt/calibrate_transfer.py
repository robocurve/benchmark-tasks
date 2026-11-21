"""Development-only native contact test, not a robot feasibility demonstration.

A virtual test fixture holds and rotates the bolt and guides the applicator with
bounded forces. Controlled initial pose changes are explicitly part of this
material test. No result from this script is a benchmark submission score.
"""
import argparse,fcntl,hashlib,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import spec,arena_runtime
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
lock=open('/tmp/robocurve-shared-gpu.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
a.output.mkdir(parents=True,exist_ok=False)
(a.output/'source-manifest.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in Path(__file__).parent.glob('*.py')},indent=2)+'\n')
w=None;report={'calibration_only':True,'robot_feasibility':False,'native_steps':0,'cases':[]}
try:
 w=arena_runtime.ThreadlockerWorld(a.output/'scene')
 ids=np.array([0],np.uint32)
 start={'bolt':[.40,0,.62,0,0,0,1],'tool':[.330,0,.627,0,0,0,1],'wax':[.330+spec.WAX_OFFSET,0,.627,0,0,0,1]}
 for name,pose in start.items():
  v=w.prop_views['/World/'+name];v.set_transforms(np.asarray([pose],np.float32),ids);v.set_velocities(np.zeros((1,6),np.float32),ids)
 wrapped=w.base.sim.backend_manager.step;counter=0;max_force=0.
 def fixture_step():
  global counter,max_force
  t=counter/spec.PHYSICS_HZ;counter+=1
  x=.330 if t<2 else .330+(.3448-.330)*min((t-2)/1,1)
  if t>9:x=.3448-(.3448-.330)*min(t-9,1)
  angle=6*math.pi*np.clip((t-3)/6,0,1)
  z=.621+.002*min(max(int((t-3)/2),0),2)
  for name,target,rotation in [('bolt',np.array([.40,0,.62]),Rotation.from_euler('z',angle)),('tool',np.array([x,0,z]),Rotation.identity())]:
   v=w.prop_views['/World/'+name];pose=v.get_transforms()[0];vel=v.get_velocities()[0]
   current=Rotation.from_quat(pose[3:])
   # Critically damped fixture drives scaled by actual native mass/inertia.
   # A fixed angular damping coefficient destabilizes the small axial inertia.
   force=spec.MASSES[name]*(100**2*(target-pose[:3])-2*100*vel[:3])+[0,0,spec.MASSES[name]*9.81]
   inertia=np.asarray(v.get_inertias()[0]).reshape(3,3)
   world_inertia=current.as_matrix()@inertia@current.as_matrix().T
   torque=world_inertia@(150**2*(rotation*current.inv()).as_rotvec()-2*150*vel[3:])
   force=np.clip(force,-10,10);torque=np.clip(torque,-2,2);max_force=max(max_force,float(np.linalg.norm(force)))
   v.apply_forces_and_torques_at_position(np.asarray([force],np.float32),np.asarray([torque],np.float32),None,ids,True)
  return wrapped()
 w.base.sim.backend_manager.step=fixture_step
 command=w.initial_command
 target=np.concatenate([np.r_[command[s][:6],command[s][6]/w.constants.FINGER_OPEN] for s in ['left','right']])
 for i in range(200):
  row=w.step(target)
  if i in [39,59,119,179,199]:report['cases'].append({'step':i+1,'material':row['coating'],'bolt':row['poses']['bolt'],'tool':row['poses']['tool']})
 np.savez_compressed(a.output/'film-state.npz',film=w.coating.film,area=w.coating.area,theta=w.coating.theta,height=w.coating.height)
 report.update(native_steps=counter,max_fixture_force_n=max_force,terminal_material=w.coating.measurements())
 assert report['cases'][0]['material']['used_volume_m3']==0.,'Coating appeared during noncontact control'
 assert report['terminal_material']['used_volume_m3']>1e-11,'Native swipe did not deposit material'
 assert report['terminal_material']['native_contacts']>0,'No genuine native wax/bolt contacts'
 assert report['terminal_material']['head_clean'],'Material unexpectedly reached head'
 report['passed_contact_smoke']=True
except BaseException as e:
 report.update(error_type=type(e).__name__,error=str(e),passed_contact_smoke=False)
 raise
finally:
 (a.output/'material-check.json').write_text(json.dumps(report,indent=2)+'\n')
 if w is not None:w.close()
 if arena_runtime.APP is not None:arena_runtime.APP.close(exit_code=0 if report.get('passed_contact_smoke') else 1)
