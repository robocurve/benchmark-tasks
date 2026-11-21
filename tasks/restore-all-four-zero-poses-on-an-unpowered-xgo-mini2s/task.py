"""Complete true-size XGO-mini2S, torque disabled, on the station's level development table.
Restore the maker calibration posture: shoulders square to body, thighs vertical,
lower legs square to thighs, four feet down, hands clear. No software zero capture.
"""
import pathlib,sys,json,math,os
import numpy as np
from collections import deque
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts,lots,tilt,observation
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN,grip_rotation
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=.1
P=pathlib.Path(__file__).resolve().parent
NATIVE=json.loads((P/'assets/native.json').read_text());LEGS=('fl','fr','br','bl')
AXES=tuple(f'{leg}_{j}_joint' for leg in LEGS for j in ('hip','thigh','calf'))
VERTS={r['name']:np.concatenate([np.asarray(m['vertices']) for m in r['meshes']])-np.asarray(r['origin']) for r in NATIVE['links']}
ROWS={r['name']:r for r in NATIVE['links']}
# Eight bounding-box corners per link: no point of a rigid link moves farther than its farthest corner.
CORNERS={n:np.array([[x,y,z] for x in (v[:,0].min(),v[:,0].max()) for y in (v[:,1].min(),v[:,1].max()) for z in (v[:,2].min(),v[:,2].max())]) for n,v in VERTS.items()}
TOE={}
for leg in LEGS:
 row=ROWS[leg+'_calf_joint'];v=VERTS[row['name']]+row['origin'];section=v[np.abs(v[:,0]-(v[:,0].max()-.010))<.004]
 TOE[leg]=(section.min(0)+section.max(0))/2
# Passive geared-servo dry resistance: an elasto-plastic rule, never a zero-pose attractor.
BREAKAWAY=.12;STIFFNESS=4.;DAMPING=.08

def shapes(row):
 out=[]
 for mesh in row['meshes']:
  v=np.asarray(mesh['vertices']);lo=v.min(0);hi=v.max(0)
  out.append(parts.Shape('mesh',tuple(hi-lo),tuple((hi+lo)/2),tuple(mesh['color']),mesh=mesh))
 return tuple(out)
base=NATIVE['links'][0];joints=[]
for row in NATIVE['links'][1:]:
 joints.append(parts.Hinge(row['name'],shapes(row),tuple(row['origin']),tuple(row['axis']),*row['limits'],
  mass=row['mass'],spring=STIFFNESS,damping=DAMPING,max_effort=BREAKAWAY,parent=row['parent']))
# OEM export lacks 166.186 g of the complete specification; allocate that difference to base.
XGO=parts.Part('xgo',pos=(.300,0.,TOP+.140),mass=base['mass']+.915-NATIVE['mass_native'],
 shapes=shapes(base),joints=joints,friction=(.6,.5),contact_offset=.0005,self_collision=False)
OBJECTS=[XGO]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'xgo':XGO.pos}),
 lots.Lot('B',place={'xgo':(.220,0.,TOP+.140)},yaw={'xgo':math.pi}))
# Starts are fixed local angles, same visible deformation mirrored with whole robot in B.
# The source's q=0 is a simulation default, not the illustrated calibration posture.
# Mid-shaft cross-sections put the long calf bars14.4–15.5deg above horizontal.
GOAL={n:(math.radians(15) if '_calf_' in n else 0.) for n in AXES}
START={f'{leg}_{j}_joint':math.radians(v) for leg in LEGS for j,v in [('hip',0.),('thigh',0.),('calf',36.)]}


def poses(s):return {'base':(s.pos,parts._quat_to_matrix(s.quat_xyzw)),**{n:(p,parts._quat_to_matrix(q)) for n,(p,q) in s.links.items()}}
def stamp(s):return tuple(np.r_[s.pos,s.quat_xyzw,*[s.joints[n] for n in AXES],*[np.r_[*s.links[n]] for n in sorted(s.links)]])
def signed_angle(a,b,axis):
 axis=np.asarray(axis);axis=axis/np.linalg.norm(axis)
 a=np.asarray(a)-axis*np.dot(axis,a);b=np.asarray(b)-axis*np.dot(axis,b)
 if min(np.linalg.norm(a),np.linalg.norm(b))<1e-6:raise ValueError('unresolved reference axis')
 return math.atan2(float(axis@np.cross(a,b)),float(a@b))
def observed_angles(s):
 # H,T,C are natural joint-shaft centres; F is the original round toe centre.
 # Local angles are reconstructed from adjoining link geometry, not world silhouettes.
 pp=poses(s);rb=pp['base'][1];out={}
 for leg in LEGS:
  hn,tn,cn=[leg+'_'+j+'_joint' for j in ('hip','thigh','calf')]
  h,t,c=[pp[n][0] for n in (hn,tn,cn)]
  h0,t0,c0=[np.array(ROWS[n]['origin']) for n in (hn,tn,cn)]
  ah,at,ac=[np.array(ROWS[n]['axis']) for n in (hn,tn,cn)]
  qh=signed_angle(t0-h0,rb.T@(t-h),ah);rh=rb@parts._rot(ah,qh)
  qt=signed_angle(c0-t0,rh.T@(c-t),at);rt=rh@parts._rot(at,qt)
  f=c+pp[cn][1]@(TOE[leg]-c0)
  qc=signed_angle(TOE[leg]-c0,rt.T@(f-c),ac)
  out.update({hn:qh,tn:qt,cn:qc})
 return out
def measure(s):
 try:
  pp=poses(s)
  if not set(AXES).issubset(s.joints) or not np.isfinite(np.r_[s.pos,s.quat_xyzw,list(s.joints.values())]).all():return None
  feet=[]
  for leg in LEGS:
   name=leg+'_calf_joint';row=next(r for r in NATIVE['links'] if r['name']==name)
   v=VERTS[name];v=v[v[:,0]>v[:,0].max()-.020];p,r=pp[name]
   feet.append(float((v@r.T+p)[:,2].min()-TOP))
  angles=observed_angles(s)
  if not np.isfinite(list(angles.values())).all() or any(abs(angles[n]-s.joints[n])>math.radians(1) for n in AXES):return None
  aligned=[all(abs(angles[f'{leg}_{j}_joint']-GOAL[f'{leg}_{j}_joint'])<=math.radians(10)+1e-9 for j in ('hip','thigh','calf')) and math.acos(float(np.clip(-(pp[leg+'_calf_joint'][0]-pp[leg+'_thigh_joint'][0])[2]/np.linalg.norm(pp[leg+'_calf_joint'][0]-pp[leg+'_thigh_joint'][0]),-1,1)))<=math.radians(10) for leg in LEGS]
  body_z=pp['base'][1][:,2]
  belly=float(min(c[2]-abs(r[2])@(size/2) for n,c,r,size in XGO.world_boxes(pp) if n=='base')-TOP)
  cloud=np.concatenate([VERTS[n]@r.T+p for n,(p,r) in pp.items()])
  # The XGO's own arm and head must stay off the table: it may not prop the body.
  arm=float(min((VERTS[n]@r.T+p)[:,2].min() for n,(p,r) in pp.items() if n.startswith('arm_'))-TOP)
  return dict(observed_deg={n:math.degrees(v) for n,v in angles.items()},bounds_low=cloud.min(0).tolist(),bounds_high=cloud.max(0).tolist(),feet=feet,aligned=aligned,count=sum(aligned),upright=float(body_z[2])>=math.cos(math.radians(10)),belly=belly,arm=arm)
 except (KeyError,ValueError,AttributeError,TypeError):return None

def released(clouds,s):
 if not clouds or any(p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all() for p in clouds):return False
 try:
  # Conservative native-link boxes include all surfaces, not only endpoints or hull vertices.
  for n,c,r,size in XGO.world_boxes(poses(s)):
   if any(np.linalg.norm(np.maximum(np.abs((p-c)@r)-size/2,0.),axis=1).min()<.030 for p in clouds):return False
  return True
 except (KeyError,ValueError,AttributeError,TypeError):return False

_known=False;_invalid=False;_clear=False;_held=0.;_previous=None;_prev_eligible=False;_time=0.;_certificate=();_rest={};_peak=0.;_anchor=None;_aligned=[False]*4;_window=deque();_supported=False
@parts.on_reset
def reset(sim):
 global _known,_invalid,_clear,_held,_previous,_prev_eligible,_time,_certificate,_rest,_peak,_anchor,_aligned,_window,_supported
 _known=False;_invalid=False;_clear=False;_held=0.;_previous=None;_prev_eligible=False;_time=0.;_certificate=();_peak=0.;_anchor=None;_aligned=[False]*4;_window=deque();_supported=False;observation.reset()
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)
 # The YAM grippers keep the stock arm's own collision settings.
 import torch
 asset=sim.env.unwrapped.scene['xgo'];q=asset.data.joint_pos.clone()*0;v=q*0
 for n,val in START.items():q[0,asset.joint_names.index(n)]=val
 asset.write_joint_state_to_sim(q,v)
 root=asset.data.root_state_w.clone();root[:,:7]=torch.tensor([*XGO.pos,0.,0.,0.,1.],device=root.device,dtype=root.dtype)
 asset.write_root_pose_to_sim(root[:,:7]);asset.write_root_velocity_to_sim(root[:,7:]*0)
 _rest={n:float(q[0,i]) for i,n in enumerate(asset.joint_names)}
 for n,val in _rest.items():sim.set_target('xgo',n,val)
 for arm in ('left_robot','right_robot'):
  robot=sim.env.unwrapped.scene[arm]
  ids=robot.find_joints(['joint7','joint8'],preserve_order=True)[0]
  robot.write_joint_effort_limit_to_sim_index(limits=40.,joint_ids=ids)
 print('[xgo reset] bodies='+str(asset.body_names)+' root_pose='+str(root[0,:7].tolist()),flush=True)

@parts.on_step
def step(sim):
 global _known,_invalid,_clear,_held,_previous,_prev_eligible,_time,_certificate,_rest,_peak,_anchor,_aligned,_window,_supported
 s=sim.states()['xgo'];_capture_visible(sim,s);m=measure(s);dt=float(sim.t)-_time
 # Elastic deflection is capped at breakaway/k; permanent slip follows any forced motion.
 # The last passive configuration, not calibration zero, is always the reference.
 for n,q in s.joints.items():
  rest=_rest.get(n,q);rest=float(q-np.clip(q-rest,-BREAKAWAY/STIFFNESS,BREAKAWAY/STIFFNESS));_rest[n]=rest;sim.set_target('xgo',n,rest)
 try:
  clouds,force=observation.hand_state(sim);support=observation.arm_support_force(sim);_supported=all(observation.foot_supports(sim));_known=True;_clear=released(clouds,s) and support<=.05;_peak=max(_peak,force)
 except (KeyError,ValueError,AttributeError,IndexError,RuntimeError) as exc:
  _known=False;_clear=False;_supported=False
  if float(sim.t)<.11:print('[xgo observation unavailable] '+str(exc),flush=True)
 if m is None or not 0<dt<=.10001:_invalid=True
 if m is not None:
  # Exact native mesh bounds exclude void corners in conservative link boxes.
  lo=m['bounds_low'];hi=m['bounds_high']
  if lo[0]<-.190 or hi[0]>.910 or lo[1]<-.650 or hi[1]>.650 or lo[2]<TOP-.01:_invalid=True
 # Grade a fresh trailing one-second window, not every past released angle.
 eligible=bool(_known and _supported and not _invalid and _clear and m and m['upright'] and m['belly']>=.015 and m['arm']>=.010 and all(-.002<=f<=.003 for f in m['feet']))
 if eligible:
  _window.append((float(sim.t),s,m['aligned'].copy(),poses(s)))
  while len(_window)>1 and _window[1][0]<=float(sim.t)-1.+1e-9:_window.popleft()
  _held=float(sim.t)-_window[0][0]
  _aligned=[all(row[2][i] for row in _window) for i in range(4)]
  current=poses(s)
  # Still: no point of any link moves more than 2 mm from where it was anywhere in the window.
  still=all(np.linalg.norm((CORNERS[n]@r.T+p)-(CORNERS[n]@oldposes[n][1].T+oldposes[n][0]),axis=1).max()<=.002 for _,old,_,oldposes in _window for n,(p,r) in current.items())
  if not still:
   _window.clear();_window.append((float(sim.t),s,m['aligned'].copy(),poses(s)));_held=0.;_aligned=m['aligned'].copy()
 else:
  _window.clear();_held=0.;_aligned=[False]*4
 _prev_eligible=eligible
 _previous=s;_time=float(sim.t);_certificate=stamp(s)
 if os.environ.get('TRACE_OUT'):
  from yam_arena.constants import TCP_OFFSET
  env=sim.env.unwrapped;origin=parts._np(env.scene.env_origins[0]);hands={}
  for arm in ('left_robot','right_robot'):
   robot=env.scene[arm];gp=parts._np(robot.data.body_link_pose_w)[0,robot.body_names.index('gripper')]
   qq=parts._np(robot.data.joint_pos)[0]
   hands[arm]=dict(tcp=(gp[:3]-origin+parts._quat_to_matrix(gp[3:])@np.array(TCP_OFFSET)).tolist(),quat=gp[3:].tolist(),fingers=[float(qq[robot.joint_names.index(n)]) for n in ('joint7','joint8')])
  with pathlib.Path(os.environ['TRACE_OUT']).with_name('xgo.jsonl').open('a') as f:
   f.write(json.dumps(dict(t=_time,dt=dt,q={n:math.degrees(q) for n,q in s.joints.items()},pos=s.pos.tolist(),known=_known,supported=_supported,invalid=_invalid,clear=_clear,held=_held,peak=_peak,metrics=m,hands=hands,score=score({'xgo':s})))+'\n')

def score(objects):
 try:
  s=objects['xgo'];m=measure(s)
  if not _known or not _supported or _invalid or not _clear or _held<1.-1e-9 or _certificate!=stamp(s) or not m:return 0.
  if not m['upright'] or m['belly']<.015 or m['arm']<.010 or not all(-.002<=f<=.003 for f in m['feet']):return 0.
  return sum(_aligned)/4
 except (KeyError,ValueError,AttributeError,TypeError):return 0.
def success(objects):return score(objects)==1.

# A single 5mm-quantized natural-feature observation precedes each reach.
# Subsequent motion is open loop; no joint values, goal flags or friction are read.
_demo_refs=[];_body_refs=[]
def _capture_visible(sim,s):
 for ref in _demo_refs:
  if ref['done'] or float(sim.t)<ref['at']:continue
  n=ref['leg']+'_calf_joint';p,r=poses(s)[n]
  pick=np.round((p+r@(TOE[ref['leg']]-np.array(ROWS[n]['origin'])))/.005)*.005
  for w,offset in ref['waypoints']:w.pos=tuple(pick+offset)
  ref['done']=True
  print('[coarse visible toe]',ref['leg'],pick.tolist(),flush=True)
@lots.after_reset
def _visible_reset(sim):
 s=sim.states()['xgo'];p,r=poses(s)['base'];pick=np.round((p+r@np.array((-.075,0,-.010)))/.005)*.005
 for w,offset in _body_refs:w.pos=tuple(pick+offset)
 print('[coarse visible body]',pick.tolist(),flush=True)
def demo():
 global _demo_refs,_body_refs
 _demo_refs=[];_body_refs=[]
 lot=lots.current();p=np.asarray(lot.pos('xgo'));yaw=lot.yaw.get('xgo',XGO.yaw);rot=parts._rot((0,0,1),yaw)
 b=p+rot@np.array((-.075,0,-.010));move=rot@np.array((.040,0,-.010));high=np.array((0,0,.100))
 # The fingers open slowly and stay open for 1 s, then the hand leaves straight up, as it came,
 # before moving aside: a quick or sideways exit jolts the chassis and can lift a toe.
 park={'left':(.25,.30,TOP+.18),'right':(.25,-.30,TOP+.18)}
 plan=[]
 for dur,offset,f,label in ((2,high,FINGER_OPEN,'approach chassis'),(3,np.zeros(3),FINGER_OPEN,'lower to chassis'),(2,np.zeros(3),-.020,'grasp original chassis'),(4,move,-.020,'guide body forward and down'),(2,move,-.020,'let feet settle'),(2.5,move,FINGER_OPEN,'open slowly'),(1,move,FINGER_OPEN,'hold open'),(3,move+high,FINGER_OPEN,'lift straight up off chassis')):
  w=Waypoint(dur,tuple(b+offset),f,label,tilt._TiltedYaw(0,tilt.rotation(0,35,-math.pi/2)));plan.append(w);_body_refs.append((w,offset))
 plan+=[Waypoint(2,park['left'],FINGER_OPEN,'clear body'),Waypoint(4,park['left'],FINGER_OPEN,'released observation')]
 return {'left':plan,'right':[Waypoint(sum(w.duration for w in plan),park['right'],FINGER_OPEN,'wait clear')]}
