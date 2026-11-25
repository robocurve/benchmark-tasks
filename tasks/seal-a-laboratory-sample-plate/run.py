"""Native YAM execution and camera evidence for the task's disclosed material model."""
import argparse,hashlib,json,os,time,shutil
from pathlib import Path
import numpy as np
from common import rotation,Drawing
from model import Model
HERE=Path(__file__).resolve().parent

def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)

def main():
 p=argparse.ArgumentParser();p.add_argument('--mode',choices=['demo','dream','hold'],default='demo');p.add_argument('--steps',type=int);p.add_argument('--out',type=Path,required=True);p.add_argument('--cpu-preview',action='store_true');p.add_argument('--capture-every',type=int,default=1,help='Diagnostic camera stride; submission evidence requires 1.');a=p.parse_args()
 if not 1<=a.capture_every<=20:p.error('Capture stride must be 1 to 20')
 plan=json.loads((HERE/'demo-plan.json').read_text()) if (HERE/'demo-plan.json').exists() else None
 if a.mode=='demo' and plan is None:p.error('No demo plan has been authored yet; use hold only for native reset inspection.')
 limit=json.loads((HERE/'task.json').read_text())['action_limit'];steps=a.steps or (len(plan['frames']) if a.mode=='demo' else limit)
 if not 1<=steps<=limit or (a.mode=='demo' and steps>len(plan['frames'])):p.error('Invalid action count')
 if a.cpu_preview and a.mode=='dream':p.error('Dream requires native robot cameras')
 a.out.mkdir(parents=True,exist_ok=False);source=a.out/'source';source.mkdir();hashes={}
 for f in HERE.iterdir():
  if f.is_file():shutil.copyfile(f,source/f.name);hashes[f.name]=hashlib.sha256(f.read_bytes()).hexdigest()
 report=dict(status='initializing',mode=a.mode,steps=0,requested_steps=steps,source_sha256=hashes,control_hz=20,score=None,cpu_preview=a.cpu_preview,submission_evidence=not a.cpu_preview and a.capture_every==1,capture_every=a.capture_every)
 start=time.monotonic()
 def save():
  report['wall_seconds']=time.monotonic()-start;(a.out/'run.json').write_text(json.dumps(report,indent=2,default=serial)+'\n')
 save();app=None;writers={};policy=None;requests=[];sessions={};trace=None;m=Model()
 try:
  if a.cpu_preview:
   raise RuntimeError('Use the offline reach audit; native contact dynamics are required for this task.')
  else:
   from isaaclab.app import AppLauncher
   app=AppLauncher(headless=True,enable_cameras=True,limit_cpu_threads=8).app
   import torch,omni.usd,imageio.v2 as imageio
   from PIL import Image
   from yam_arena import inspect_embodiment as wrapper,scene
   from review_views import configure,images
   if hasattr(m,'configure_station'):m.configure_station()
   from yam_arena.constants import TCP_OFFSET,FINGER_JOINTS
   scene.PHYSICS_DT=1./480
   original=scene.make_env
   def with_cameras(embodiment,**kwargs):configure(embodiment);return original(embodiment,**kwargs)
   scene.make_env=with_cameras;wrapper._APP=app
   emb=wrapper.YamArenaEmbodiment(task=str(HERE/'task.py'),object_jitter=0.,control_hz=20,cam_width=320,cam_height=176)
   m.subscribe_events()
   emb._ensure_sim(None);base=emb._base;emb._env.reset();instruction=json.loads((HERE/'task.json').read_text())['instruction'];emb._instruction=instruction
   m.bind_native(base)
   drawing=Drawing(omni.usd.get_context().get_stage());m.draw(drawing)
   def refresh():
    base.sim.render()
    for name in ['top_cam','left_cam','right_cam','judging_front','judging_rear','judging_side']:base.scene[name].update(0.,force_recompute=True)
   for _ in range(6):refresh()
   def array(value):return getattr(value,'torch',value).detach().cpu().numpy()
   def hands():
    out={}
    for side in ['left','right']:
     robot=base.scene[side+'_robot'];idx=list(robot.body_names).index('gripper');pose=array(robot.data.body_link_pose_w)[0,idx];R=rotation(pose[3:]);ids=robot.find_joints(list(FINGER_JOINTS),preserve_order=True)[0];opening=float(np.abs(array(robot.data.joint_pos)[0,ids]).sum()/.094)
     out[side]=dict(tcp=pose[:3]+R@np.asarray(TCP_OFFSET),rotation=R,opening=opening,joint_positions=array(robot.data.joint_pos)[0,emb._ids[side+'_robot'][0]],finger_joint_positions=array(robot.data.joint_pos)[0,ids])
    return out
   m.time=-1.
   for _ in range(20):
    emb._env.step(emb._ready);m.update(hands())
   m.time=0.;m.release=0.;m.draw(drawing);refresh()
   for name in ['front','rear','side','policy']:writers[name]=imageio.get_writer(a.out/(name+'.mp4'),fps=20/a.capture_every,codec='libx264',pixelformat='yuv420p',ffmpeg_params=['-movflags','+faststart'],macro_block_size=1)
   current=emb._ready.detach().cpu().numpy()[0].copy()
  from native_contact_audit import record
  record(base,a.out)
  trace=(a.out/'trace.jsonl').open('w')
  def capture(step):
   if not a.cpu_preview and (step%a.capture_every==0 or step==steps):
    obs=emb._observation();views=images(base)
    for label in ['front','rear','side']:
     rgb=views['judging_'+label];writers[label].append_data(rgb)
     if step in [0,steps]:Image.fromarray(rgb).save(a.out/(label+('-start.png' if step==0 else '-final.png')))
     if step and step%100==0:Image.fromarray(rgb).save(a.out/(label+'-progress.png'))
    writers['policy'].append_data(np.asarray(obs.images['top_cam']))
   row=dict(step=step,action14=current,measurements=m.measure(),hands=hands(),state=m.state())
   trace.write(json.dumps(row,default=serial,separators=(',',':'))+'\n')
  capture(0)
  if m.integrity_failed() or m.score()!=0. or not m.reset_valid():raise RuntimeError("Reset changed task integrity or already earned credit; fix the native reset before a trial.")
  if a.mode=='dream':
   from inspect_robots import Scene
   from dream_policy import make_policy
   policy=make_policy(requests,sessions);policy.bind(emb.info);policy.reset(Scene(id=HERE.name,instruction=instruction))
  from joint_tracking import JointTracking
  from yam_arena.constants import SHOULDER,WRIST,GRIPPER,FINGER_OPEN
  from demo_control import DemoControl
  controller=DemoControl() if a.mode=="demo" else None
  trackers=[JointTracking(side=s) for s in ['left','right']];limit_vector=np.tile([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/20],2)
  report.update(status='running',initial_measurements=m.measure());save()
  for k in range(1,steps+1):
   if a.mode=='demo':
    wanted=np.asarray(plan['frames'][k-1]['action14'])
    if not a.cpu_preview:
     observed=[]
     for robot_name in ['left_robot','right_robot']:
      arm_ids,finger_ids=emb._ids[robot_name];q=array(base.scene[robot_name].data.joint_pos)[0]
      observed.extend(q[arm_ids]);observed.append(float(np.clip(q[finger_ids].mean()/FINGER_OPEN,0,1)))
     obs=np.asarray(observed)
     wanted=controller.action(k,wanted,m,obs,hands())
     for tracker in trackers:
      wanted=tracker.action(wanted,obs,enabled=True)
   elif policy is not None:
    chunk=policy.act(emb._observation());assert len(chunk.actions)==1;wanted=np.asarray(chunk.actions[0].data,float)
   else:wanted=current.copy()
   if wanted.shape!=(14,) or not np.isfinite(wanted).all():raise ValueError('Invalid ordinary robot action')
   if not a.cpu_preview:wanted=np.clip(wanted,emb._low,emb._high)
   current=current+np.clip(wanted-current,-limit_vector,limit_vector)
   if not a.cpu_preview:
    # Recording stride must not change the simulator update schedule.
    base.render_enabled=True  # Keep normal USD/PhysX synchronization even in diagnostic recordings.
    emb._env.step(torch.as_tensor(current[None,:],device=base.device,dtype=torch.float32));emb._t=k/20
   m.update(hands())
   if not a.cpu_preview and (k%a.capture_every==0 or k==steps):m.draw(drawing);refresh()
   capture(k);report.update(steps=k,measurements=m.measure(),score=m.score())
   if k%20==0:save();trace.flush()
   if (a.out/'stop-requested').exists():raise RuntimeError('Diagnostic stopped at a control-step boundary by operator request.')
   if a.mode=='demo' and m.failed:raise RuntimeError('Native task integrity failed; development demo stopped early.')
  report.update(status='complete',events=m.events,score=m.score(),requires_visual_review=True)
 except BaseException as error:
  import traceback
  report.update(status='failed',error_type=type(error).__name__,error_message=str(error),traceback=traceback.format_exc());raise
 finally:
  if trace is not None:trace.close()
  for writer in writers.values():writer.close()
  if policy is not None:
   from dream_policy import public_transcript
   (a.out/'dream-transcript.json').write_text(json.dumps(public_transcript(policy),indent=2)+'\n')
  report.update(dream_requests=len(requests),dream_sessions=sessions,events=m.events);save()
  if app is not None:app.close()
if __name__=='__main__':main()
