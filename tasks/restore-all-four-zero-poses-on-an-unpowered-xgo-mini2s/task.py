"""Complete true-size XGO-lite robot dog (500 g, the XGO-lite V2 sold as the ElecFreaks XGO Robot Kit V2), servos
without torque, on the station's level development table. The rear legs are in the maker's calibration posture and
the front lower legs are turned toe-up out of it. Set every leg to the posture (shoulder square to the body, thigh
vertical, lower leg square to the thigh and lying on the table) without disturbing the rear legs; then hands clear.
No software zero capture.
"""
import pathlib,sys,json,math,os,dataclasses
import numpy as np
from collections import deque
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts,lots,tilt,observation
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN,grip_rotation
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=.1
P=pathlib.Path(__file__).resolve().parent
NATIVE=json.loads((P/'assets/native.json').read_text());LEGS=('FL','FR','RR','RL')
AXES=tuple(f'{leg}_{j}_joint' for leg in LEGS for j in ('hip','thigh','shank'))
ARM=('mainarm_joint','forearm_joint')
ROWS={r['name']:r for r in NATIVE['links']}
VERTS={r['name']:np.concatenate([np.asarray(m['vertices']) for m in r['meshes']])-np.asarray(r['origin']) for r in NATIVE['links']}
# Eight bounding-box corners per link: no point of a rigid link moves farther than its farthest corner.
CORNERS={n:np.array([[x,y,z] for x in (v[:,0].min(),v[:,0].max()) for y in (v[:,1].min(),v[:,1].max()) for z in (v[:,2].min(),v[:,2].max())]) for n,v in VERTS.items()}
# The maker's foot point on each lower leg (lite3.xml sites FL/FR/RR/RL): the centre of the round toe pad.
PAD={leg:np.asarray(ROWS[leg+'_shank_joint']['origin'])+np.array([.068,-.02 if leg in ('FL','RL') else .02,0.]) for leg in LEGS}
# Where the demo pinches each pad: the centre of the pad's mesh near that foot point (about 1 mm outboard of it).
PAD_GRIP={}
for _leg in LEGS:
 _v=VERTS[_leg+'_shank_joint']+np.asarray(ROWS[_leg+'_shank_joint']['origin']);_v=_v[np.linalg.norm(_v-PAD[_leg],axis=1)<.0095]
 PAD_GRIP[_leg]=(_v.min(0)+_v.max(0))/2
# Passive geared servo with torque off: elasto-plastic hold, never a pull toward the calibration posture.
BREAKAWAY=.06;STIFFNESS=2.;DAMPING=.04
BAND=7.  # degrees: each leg angle, and the thigh from vertical
MASS=.500  # XGO-lite V2 complete (ElecFreaks product parameters)

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
# The maker model's links weigh 345 g; the rest of the 500 g unit (battery, boards, screws) is added to the body.
XGO=parts.Part('xgo',pos=(.310,-.060,TOP+.080),mass=base['mass']+MASS-NATIVE['mass_native'],
 shapes=shapes(base),joints=joints,friction=(.6,.5),contact_offset=.0005,self_collision=False)
# The illustrated posture in the maker model's joint angles: shoulders and thighs at 0 (shoulder square, thigh
# vertical); each lower leg turned -12.16 deg so its toe pad and its heel under the knee are level
# (checks/flat_axis.py reproduces this from the meshes).
QF=-.2123
GOAL={n:(QF if '_shank_' in n else 0.) for n in AXES}
# Start: the rear legs and both front shoulders and thighs in the posture; each front lower leg turned toe-up from
# it, by a different angle (as after the front knee servos have slipped teeth or been refitted).
UP={'FL':22.,'FR':26.,'RR':0.,'RL':0.};FRONT=('FL','FR');REAR=('RR','RL')
START={**{f'{leg}_{j}_joint':(QF+math.radians(UP[leg]) if j=='shank' else 0.) for leg in LEGS for j in ('hip','thigh','shank')},**{n:0. for n in ARM}}

def poses(s):return {'base':(s.pos,parts._quat_to_matrix(s.quat_xyzw)),**{n:(p,parts._quat_to_matrix(q)) for n,(p,q) in s.links.items()}}
def lowest(pp):return min(float((VERTS[n]@r.T+p)[:,2].min()) for n,(p,r) in pp.items())
# Set down with its lowest point 0.5 mm above the table.
_s0=XGO.state(joints=START,pos=(.310,-.060,TOP+.080));XGO=dataclasses.replace(XGO,pos=(.310,-.060,round(TOP+.080-(lowest(poses(_s0))-TOP)+.0005,5)),yaw=math.radians(84))
OBJECTS=[XGO]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'xgo':XGO.pos},yaw={'xgo':math.radians(84)}),
 lots.Lot('B',place={'xgo':(.330,-.045,XGO.pos[2])},yaw={'xgo':math.radians(102)}))

def stamp(s):return tuple(np.r_[s.pos,s.quat_xyzw,*[s.joints[n] for n in AXES],*[np.r_[*s.links[n]] for n in sorted(s.links)]])
def signed_angle(a,b,axis):
 axis=np.asarray(axis);axis=axis/np.linalg.norm(axis)
 a=np.asarray(a)-axis*np.dot(axis,a);b=np.asarray(b)-axis*np.dot(axis,b)
 if min(np.linalg.norm(a),np.linalg.norm(b))<1e-6:raise ValueError('unresolved reference axis')
 return math.atan2(float(axis@np.cross(a,b)),float(a@b))
def observed_angles(s):
 # H, T, C are the shoulder, thigh and knee shaft centres; F is the centre of the round toe pad.
 # Local angles are rebuilt from adjoining link geometry, not from world silhouettes.
 pp=poses(s);rb=pp['base'][1];out={}
 for leg in LEGS:
  hn,tn,cn=[leg+'_'+j+'_joint' for j in ('hip','thigh','shank')]
  h,t,c=[pp[n][0] for n in (hn,tn,cn)]
  h0,t0,c0=[np.array(ROWS[n]['origin']) for n in (hn,tn,cn)]
  ah,at,ac=[np.array(ROWS[n]['axis']) for n in (hn,tn,cn)]
  qh=signed_angle(t0-h0,rb.T@(t-h),ah);rh=rb@parts._rot(ah,qh)
  qt=signed_angle(c0-t0,rh.T@(c-t),at);rt=rh@parts._rot(at,qt)
  f=c+pp[cn][1]@(PAD[leg]-c0)
  qc=signed_angle(PAD[leg]-c0,rt.T@(f-c),ac)
  out.update({hn:qh,tn:qt,cn:qc})
 return out
def measure(s):
 try:
  pp=poses(s)
  if not set(AXES).issubset(s.joints) or not np.isfinite(np.r_[s.pos,s.quat_xyzw,list(s.joints.values())]).all():return None
  angles=observed_angles(s)
  if not np.isfinite(list(angles.values())).all() or any(abs(angles[n]-s.joints[n])>math.radians(1) for n in AXES):return None
  def vertical(leg):
   d=pp[leg+'_shank_joint'][0]-pp[leg+'_thigh_joint'][0];return math.acos(float(np.clip(-d[2]/np.linalg.norm(d),-1,1)))
  # A leg is set when its three angles are each within 7 degrees of the posture and its thigh within 7 of vertical.
  aligned=[all(abs(angles[f'{leg}_{j}_joint']-GOAL[f'{leg}_{j}_joint'])<=math.radians(BAND)+1e-9 for j in ('hip','thigh','shank')) and vertical(leg)<=math.radians(BAND) for leg in LEGS]
  body_z=pp['base'][1][:,2]
  belly=float((VERTS['base']@pp['base'][1].T+pp['base'][0])[:,2].min()-TOP)
  cloud=np.concatenate([VERTS[n]@r.T+p for n,(p,r) in pp.items()])
  # The XGO's own folded arm must stay off the table: it may not prop the body.
  arm=float(min((VERTS[n]@r.T+p)[:,2].min() for n,(p,r) in pp.items() if n in ARM)-TOP)
  return dict(observed_deg={n:math.degrees(v) for n,v in angles.items()},bounds_low=cloud.min(0).tolist(),bounds_high=cloud.max(0).tolist(),aligned=aligned,count=sum(aligned),upright=float(body_z[2])>=math.cos(math.radians(10)),belly=belly,arm=arm)
 except (KeyError,ValueError,AttributeError,TypeError):return None

def released(clouds,s):
 if not clouds or any(p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all() for p in clouds):return False
 try:
  # Conservative native-link boxes include all surfaces, not only endpoints or hull vertices.
  for n,c,r,size in XGO.world_boxes(poses(s)):
   if any(np.linalg.norm(np.maximum(np.abs((p-c)@r)-size/2,0.),axis=1).min()<.030 for p in clouds):return False
  return True
 except (KeyError,ValueError,AttributeError,TypeError):return False

_known=False;_invalid=False;_clear=False;_held=0.;_previous=None;_time=0.;_certificate=();_rest={};_peak=0.;_aligned=[False]*4;_window=deque()
@parts.on_reset
def reset(sim):
 global _known,_invalid,_clear,_held,_previous,_time,_certificate,_rest,_peak,_aligned,_window
 _known=False;_invalid=False;_clear=False;_held=0.;_previous=None;_time=0.;_certificate=();_peak=0.;_aligned=[False]*4;_window=deque();observation.reset()
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)
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
 global _known,_invalid,_clear,_held,_previous,_time,_certificate,_rest,_peak,_aligned,_window
 s=sim.states()['xgo'];m=measure(s);dt=float(sim.t)-_time
 # Elastic deflection is capped at breakaway/k; permanent slip follows any forced motion.
 # The last passive configuration, not the calibration posture, is always the reference.
 for n,q in s.joints.items():
  rest=_rest.get(n,q);rest=float(q-np.clip(q-rest,-BREAKAWAY/STIFFNESS,BREAKAWAY/STIFFNESS));_rest[n]=rest;sim.set_target('xgo',n,rest)
 try:
  clouds,force=observation.hand_state(sim);support=observation.arm_support_force(sim);_known=True;_clear=released(clouds,s) and support<=.05;_peak=max(_peak,force)
 except (KeyError,ValueError,AttributeError,IndexError,RuntimeError) as exc:
  _known=False;_clear=False
  if float(sim.t)<.11:print('[xgo observation unavailable] '+str(exc),flush=True)
 if m is None or not 0<dt<=.10001:_invalid=True
 if m is not None:
  lo=m['bounds_low'];hi=m['bounds_high']
  if lo[0]<-.190 or hi[0]>.910 or lo[1]<-.650 or hi[1]>.650 or lo[2]<TOP-.01:_invalid=True
 # Grade a fresh trailing one-second window, not every past released angle.
 eligible=bool(_known and not _invalid and _clear and m and m['upright'] and m['belly']>=.015 and m['arm']>=.010)
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
 _previous=s;_time=float(sim.t);_certificate=stamp(s)
 _steady(sim)
 if os.environ.get('TRACE_OUT'):
  from yam_arena.constants import TCP_OFFSET
  env=sim.env.unwrapped;origin=parts._np(env.scene.env_origins[0]);hands={}
  for arm in ('left_robot','right_robot'):
   robot=env.scene[arm];gp=parts._np(robot.data.body_link_pose_w)[0,robot.body_names.index('gripper')]
   qq=parts._np(robot.data.joint_pos)[0]
   hands[arm]=dict(tcp=(gp[:3]-origin+parts._quat_to_matrix(gp[3:])@np.array(TCP_OFFSET)).tolist(),quat=gp[3:].tolist(),fingers=[float(qq[robot.joint_names.index(n)]) for n in ('joint7','joint8')])
  with pathlib.Path(os.environ['TRACE_OUT']).with_name('xgo.jsonl').open('a') as f:
   f.write(json.dumps(dict(t=_time,dt=dt,q={n:math.degrees(q) for n,q in s.joints.items()},pos=s.pos.tolist(),quat=s.quat_xyzw.tolist(),known=_known,invalid=_invalid,clear=_clear,held=_held,peak=_peak,metrics=m,hands=hands,score=score({'xgo':s})))+'\n')

def score(objects):
 try:
  s=objects['xgo'];m=measure(s)
  if not _known or _invalid or not _clear or _held<1.-1e-9 or _certificate!=stamp(s) or not m:return 0.
  if not m['upright'] or m['belly']<.015 or m['arm']<.010:return 0.
  # Both rear legs start set and must still be set; each front leg set earns half.
  if not all(_aligned[LEGS.index(l)] for l in REAR):return 0.
  return sum(_aligned[LEGS.index(l)] for l in FRONT)/2
 except (KeyError,ValueError,AttributeError,TypeError):return 0.
def success(objects):return score(objects)==1.

# ---- scripted demo: open loop from the declared layout; it reads nothing of the robot dog ----
# One hand holds the chassis and lifts the robot about 30 mm, so its body cannot move and no foot is on the table;
# the other hand pinches each front toe pad and moves it along its arc about the knee to the posture. Then the robot
# is set down and both hands leave.
GRASP=np.array((.010,0.,.003))  # chassis grip point, body frame: over the centre of mass, below the folded arm
HOLD_TILT=30.  # the holding hand leans back so its fingers clear the folded arm
LIFT=.050;SAG={'A':(.0184,),'B':(.0184,)}  # commanded lift (m) and the holding arm's sag under the robot, per lot
# Where the hanging robot settles in the holding hand, per lot, as measured in a development run of this same motion
# (world x, y, z of the body origin in m; roll, pitch, yaw in degrees, world z-y-x): the hand sags about 18 mm under
# the robot's weight and draws the body toward it.
HELD={'A':(.3032,-.0677,.8560,2.20,4.22,83.92),'B':(.3283,-.0563,.8576,.87,2.78,101.54)}
TILT=22.;PINCH=-.0035;OPEN_PAD=-.0140;GRIP=-.030;OVER=math.radians(13.)
_DOWN=grip_rotation(0.);_APPROACH=_DOWN.T@np.array((0,0,-1.))  # TCP-frame direction from wrist to fingertips
def _yaw(R):return tilt._TiltedYaw(0.,R)
def _robot(p,yaw,q,R=None):
 R=parts._rot((0,0,1),yaw) if R is None else R;pp=XGO.link_poses(np.asarray(p,float),R,q);return pp,R
def _rpy(roll,pitch,yaw):
 return parts._rot((0,0,1),math.radians(yaw))@parts._rot((0,1,0),math.radians(pitch))@parts._rot((1,0,0),math.radians(roll))
def _settled(p,yaw,q):
 pp,R=_robot(p,yaw,q);return np.asarray(p,float)-[0,0,lowest(pp)-TOP]
def _pad_plan(pp,leg,q0):
 """TCP waypoints that pinch ``leg``'s toe pad and turn its lower leg about the knee to the posture."""
 n=leg+'_shank_joint';o,Rs=pp[n];c0=o+Rs@(PAD_GRIP[leg]-np.asarray(ROWS[n]['origin']));axis=Rs@np.asarray(ROWS[n]['axis'])
 Rb=pp['base'][1];yaw=math.atan2(Rb[1,0],Rb[0,0])
 # Fingers close across the pad's flat faces; the hand leans out ahead of the robot, clear of the head, and the turn
 # leans it further.
 R0=tilt.rotation(_grip_yaw(yaw),TILT,yaw+math.pi);delta=(QF-OVER)-q0
 # The pad's centre follows its arc about the knee; the hand turns half as far (the pad may turn in the fingers).
 arc=[(o+parts._rot(axis,f*delta)@(c0-o),parts._rot(axis,f*delta/2)@R0) for f in np.linspace(0,1,6)]
 back=lambda pos,R,d:pos-d*(R@_APPROACH)
 w=[Waypoint(2.0,tuple(back(c0,R0,.050)+[0,0,.040]),OPEN_PAD,f'over {leg} pad',_yaw(R0)),
    Waypoint(1.0,tuple(back(c0,R0,.040)),OPEN_PAD,f'line up on {leg} pad',_yaw(R0)),
    Waypoint(1.5,tuple(back(c0,R0,.040)),OPEN_PAD,f'line up and steady on {leg} pad',_yaw(R0)),
    Waypoint(1.5,tuple(c0),OPEN_PAD,f'around {leg} pad',_yaw(R0)),
    Waypoint(1.2,tuple(c0),PINCH,f'pinch {leg} pad',_yaw(R0))]
 w+=[Waypoint(.6,tuple(pos),PINCH,f'turn {leg} lower leg',_yaw(R)) for pos,R in arc[1:]]
 pe,Re=arc[-1]
 w+=[Waypoint(.5,tuple(pe),PINCH,f'hold {leg}',_yaw(Re)),Waypoint(1.0,tuple(pe),FINGER_OPEN,f'open {leg}',_yaw(Re)),
     Waypoint(1.0,tuple(pe+[0,0,.030]),FINGER_OPEN,f'back off {leg}',_yaw(Re)),
     Waypoint(1.0,tuple(pe+[0,0,.070]),FINGER_OPEN,f'rise from {leg}',_yaw(Re))]
 return w
def _grip_yaw(yaw):
 """The same two-finger grip turned by a half turn or not, whichever needs less wrist turn."""
 return (yaw+math.pi/2)%math.pi-math.pi/2
def _wait(w,t,label='wait'):return Waypoint(t,w.pos,w.finger,label,w.yaw)
def demo():
 lot=lots.current();p=np.asarray(lot.pos('xgo'),float);yaw=lot.yaw_of('xgo');sag=SAG.get(lot.name,(.0184,))
 HOLD=tilt.rotation(_grip_yaw(yaw),HOLD_TILT,yaw)  # fingers close across the body; fingertips lean toward the head
 q=dict(START);p=_settled(p,yaw,q);pp,R=_robot(p,yaw,q)
 park={'left':(.20,.30,TOP+.20),'right':(.20,-.30,TOP+.20)};plans={'left':[],'right':[]};clock={'left':0.,'right':0.}
 def add(arm,wps):
  plans[arm]+=wps;clock[arm]+=sum(w.duration for w in wps)
 def sync(arm,t,label='wait'):
  if t>clock[arm]+1e-9:
   last=plans[arm][-1] if plans[arm] else Waypoint(0.,park[arm],FINGER_OPEN,'start')
   add(arm,[_wait(last,t-clock[arm],label)])
 # The right hand holds the chassis; the left hand turns both front lower legs (the robot faces the left arm).
 for phase,(holder,worker) in enumerate((('right','left'),)):
  # holder: grip the chassis sides, lift; the robot hangs with its legs clear of the table
  g=p+R@GRASP;dz=LIFT
  add(holder,[Waypoint(2.5,tuple(g+[0,0,.060]),FINGER_OPEN,'over chassis',_yaw(HOLD)),
              Waypoint(1.5,tuple(g+[0,0,.060]),FINGER_OPEN,'line up and steady over chassis',_yaw(HOLD)),
              Waypoint(2.5,tuple(g),FINGER_OPEN,'around chassis',_yaw(HOLD)),
              Waypoint(1.5,tuple(g),GRIP,'grip chassis',_yaw(HOLD)),
              Waypoint(2.5,tuple(g+[0,0,dz]),GRIP,'lift robot',_yaw(HOLD))])
  held=p+[0,0,dz-sag[phase]];Rh=None
  if lot.name in HELD:
   x,y,z,roll,pitch,hy=HELD[lot.name];held=np.array((x,y,z));Rh=_rpy(roll,pitch,hy)
  hp,_=_robot(held,yaw,q,Rh)
  side=list(FRONT)
  side.sort(key=lambda leg:abs(hp[leg+'_shank_joint'][0][0]-park[worker][0]))
  sync(worker,clock[holder]+.5,'wait for lift')
  for leg in side:
   add(worker,_pad_plan(hp,leg,q[leg+'_shank_joint']));q[leg+'_shank_joint']=QF-OVER
  add(worker,[Waypoint(2.0,park[worker],FINGER_OPEN,'clear robot',_yaw(_DOWN))])
  sync(holder,clock[worker]-2.0,'hold while legs are turned')
  # set it down: lower until the turned legs are about to touch, then open slowly and leave straight up
  q2={k:(QF if k.endswith('_shank_joint') and v==QF-OVER else v) for k,v in q.items()}
  hyaw=yaw if Rh is None else math.atan2(Rh[1,0],Rh[0,0])
  down=_settled(held,hyaw,q2);gz=g+[0,0,(down[2]-p[2])+sag[phase]-.001]
  add(holder,[Waypoint(2.5,tuple(gz),GRIP,'set robot down',_yaw(HOLD)),
              Waypoint(2.5,tuple(gz),FINGER_OPEN,'open slowly',_yaw(HOLD)),
              Waypoint(1.0,tuple(gz),FINGER_OPEN,'hold open',_yaw(HOLD)),
              Waypoint(2.5,tuple(gz+[0,0,.060]),FINGER_OPEN,'lift straight up',_yaw(HOLD)),
              Waypoint(2.0,park[holder],FINGER_OPEN,'clear robot',_yaw(_DOWN))])
  q=q2;p=down;yaw=hyaw;pp,R=_robot(p,yaw,q)
  sync(worker,clock[holder],'wait for set-down')
 end=max(clock.values())+3.
 for arm in plans:sync(arm,end,'released observation')
 _TRACK.clear();_TRACK.update({arm:dict(plan=__import__('yam_arena.demo',fromlist=['ArmPlan']).ArmPlan(list(w),(0,0,0)),wps=w,base=[np.asarray(x.pos,float) for x in w],corr=np.zeros(3)) for arm,w in plans.items()})
 return plans

# Scripted arms only: each arm corrects its own steady tracking error from its joint readings (TCP from forward
# kinematics) while it waits or lines up in free space, and holds that correction fixed through every contact. It reads nothing
# of the robot dog. Dream-1 runs never call demo(), so this does nothing for them or for scoring.
_TRACK={};FREE=('line up','wait','released')
def _steady(sim):
 if not _TRACK:return
 try:
  from yam_arena.constants import TCP_OFFSET
  env=sim.env.unwrapped;origin=parts._np(env.scene.env_origins[0]);t=float(sim.t)-.05
  for arm,tr in _TRACK.items():
   robot=env.scene[arm+'_robot'];gp=parts._np(robot.data.body_link_pose_w)[0,robot.body_names.index('gripper')]
   tcp=gp[:3]-origin+parts._quat_to_matrix(gp[3:])@np.array(TCP_OFFSET)
   want,_,_,label=tr['plan'].target(max(t,0.))
   if not label.startswith(FREE):continue
   desired=want+tr['corr']  # the uncorrected plan point
   tr['corr']=np.clip(tr['corr']+.1*(tcp-desired),-.03,.03)
   for w,b in zip(tr['wps'],tr['base']):w.pos=tuple(b-tr['corr'])
 except (KeyError,ValueError,AttributeError,IndexError,RuntimeError):return
