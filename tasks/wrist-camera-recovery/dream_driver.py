"""Official DreamPolicy adapter; actual native observations, no scripted commands."""
import json,time
from pathlib import Path
import numpy as np
class DreamDriver:
 def __init__(self,base,instruction,out):
  import dream
  from inspect_robots import Scene
  from inspect_robots_dream.policy import DreamPolicy
  from inspect_robots_dream.session import DreamSession
  from yam_arena.inspect_embodiment import YamArenaEmbodiment
  from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS
  key=next(x.split('=',1)[1].strip().strip('\"\x27') for x in (Path.home()/'.dreamlabs/robocurve_credentials').read_text().splitlines() if x.startswith('DREAM_API_KEY='))
  self.base=base;self.instruction=instruction;self.out=out;self.actions=[];self.states=[];self.step=0
  self.ids={n:(base.scene[n].find_joints(ARM_JOINTS,preserve_order=True)[0],base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0]) for n in ['left_robot','right_robot']}
  interface=YamArenaEmbodiment(scene='station',control_hz=20,cam_width=320,cam_height=176,object_jitter=0)
  self.low,self.high=interface._low,interface._high
  self.request_audit=[];self.session_id=None;self.last_reply=None
  def make_client():
   client=dream.DreamClient(api_key=key)
   original=client.get_action
   def checked_action(*args,**kwargs):
    sent=time.monotonic()
    gap=None if self.last_reply is None else sent-self.last_reply
    try:reply=original(*args,**kwargs)
    except dream.DreamSessionExpired as error:
     (self.out/'SESSION_INVALID.json').write_text(json.dumps({'reason':str(error),'step':self.step,'idle_gap_s':gap},indent=2))
     raise RuntimeError('Session expired; trial aborted, no automatic session restart accepted') from error
    sid=client.session_id
    if self.session_id is not None and sid!=self.session_id:
     (self.out/'SESSION_INVALID.json').write_text(json.dumps({'reason':'Session ID changed','step':self.step},indent=2))
     raise RuntimeError('Session changed; trial invalid')
    self.session_id=sid;self.last_reply=time.monotonic()
    self.request_audit.append({'step':self.step,'session_id':sid,'idle_gap_s':gap,'wall_rtt_s':self.last_reply-sent})
    (self.out/'session_continuity.json').write_text(json.dumps({'sessions':len({x['session_id'] for x in self.request_audit}),'requests':self.request_audit},indent=2))
    for name in ('q_l_chunk','q_r_chunk'):
     chunk=np.asarray(reply[name],dtype=float)
     if chunk.ndim!=2 or chunk.shape[1]!=7 or not np.isfinite(chunk).all():raise RuntimeError('Invalid action chunk')
     grip=chunk[:,6]
     if grip.min() < -0.1 or grip.max() > 1.1:raise RuntimeError('Gripper output exceeds allowed 0.1 overshoot margin')
     if (grip<0).any() or (grip>1).any():
      with (self.out/'gripper_clipping.jsonl').open('a') as f:f.write(json.dumps({'step':self.step,'arm':name,'raw_min':float(grip.min()),'raw_max':float(grip.max()),'raw_gripper':grip.tolist()})+'\n')
      chunk[:,6]=np.clip(grip,0,1);reply[name]=chunk.tolist()
    return reply
   client.get_action=checked_action
   return client
  self.policy=DreamPolicy(session=DreamSession(make_client=make_client),rows_per_request=8,record=False,timeout_s=90)
  self.policy.bind(interface.info);self.policy.reset(Scene(id='robocurve-local-trial',instruction=instruction,init_seed=42))
  self.save()
 def action(self):
  import cv2
  from inspect_robots import Observation
  from yam_arena.constants import FINGER_OPEN
  images={};state=[]
  for n in ['top_cam','left_cam','right_cam']:
   rgb=self.base.scene[n].data.output['rgb'];rgb=rgb.torch if hasattr(rgb,'torch') else rgb
   images[n]=np.ascontiguousarray(cv2.resize(rgb[0,:,:,:3].cpu().numpy(),(320,176)),dtype=np.uint8)
  for n in ['left_robot','right_robot']:
   q=self.base.scene[n].data.joint_pos.torch[0];a,f=self.ids[n]
   state.extend(q[a].cpu().numpy().tolist());state.append(float(np.clip(q[f].mean().item()/FINGER_OPEN,0,1)))
  o=Observation(images=images,state={'joint_pos':np.array(state)},instruction=self.instruction,image_times={n:self.step/20 for n in images},state_time=self.step/20)
  row=np.asarray(self.policy.act(o).actions[0].data)
  if row.shape!=(14,) or not np.isfinite(row).all():raise ValueError('Nonfinite/wrong shape Dream action')
  action=np.clip(row,self.low,self.high);self.actions.append(action.copy());self.states.append(state);self.step+=1
  if self.step%8==0:self.save()
  return action
 def save(self):
  (self.out/'dream_transcript.json').write_text(json.dumps(self.policy.transcript(),indent=2))
  np.savez_compressed(self.out/'policy_actions.npz',actions=np.array(self.actions),measured_states=np.array(self.states))
 def close(self):
  self.save();self.policy.close()
