"""Ordinary loose-roll acquisition/lift checkpoint; no sealing/peel credit.

`RollPickupProbe(env, contact_reader)` reads live scene/joints only. The reader
is called as reader('right') and must return coeval {time_s, body_names,
target_names, normal_force_matrix_w}. The finite matrix has shape(B,F,3),
body names uniquely include tip_left/tip_right, targets are exactly roll plus
all tape_i cells. Values are measured WORLD normal forces, not commands or
estimated motor efforts. Root's native sensor integration supplies this API.
"""
import math,struct,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from yam_arena import YAM_URDF,ik
from yam_arena.constants import FINGER_OPEN,ARM_JOINTS
from yam_arena.urdf import forward_kinematics,rpy_matrix,quat_xyzw_to_matrix,rotation_angle_deg
from config import TOP,ROLL_START,CELLS,ACTIVE_INNER_RADIUS,CELL_LENGTH,PRODUCTS,CENTER,CUTTER_START
from model import CORE_RADIUS,OUTER_RADIUS,TAPE_WIDTH,TAPE_THICKNESS
from carton_scene import layout

BASES={'left':np.array((0.,.305,.75)),'right':np.array((0.,-.305,.75))}
OPEN=.012/abs(FINGER_OPEN)
HOLD=.0065/abs(FINGER_OPEN)
# Curved core needs slightly inboard centering for the actual wide finger pads.
RIM_RADIUS=.043
FAMILY={'roll',*(f'tape_{i}' for i in range(CELLS))}


def array(value):
 value=value.torch if hasattr(value,'torch') else value
 return np.asarray(value.detach().cpu().numpy())


def opposed_contacts(sample,time):
 bodies=list(sample['body_names']);targets=list(sample['target_names']);forces=np.asarray(sample['normal_force_matrix_w'],float)
 if len(set(bodies))!=len(bodies) or len(set(targets))!=len(targets) or set(targets)!=FAMILY:raise ValueError('incomplete/ambiguous roll contact binding')
 if forces.shape!=(len(bodies),len(targets),3) or not np.isfinite(forces).all() or not math.isfinite(sample['time_s']) or abs(sample['time_s']-time)>1e-6:raise ValueError('missing/nonfinite/stale native normal forces')
 a,b=(forces[bodies.index(n)].sum(0) for n in ('tip_left','tip_right'));na,nb=np.linalg.norm(a),np.linalg.norm(b)
 return bool(na>=.1 and nb>=.1 and a@b<=-.5*na*nb),[float(na),float(nb)]


def grip_target(roll):
 p,R=roll;x=R@np.array((0.,-1.,0.));z=-R[:,2];hand=np.column_stack((x,np.cross(z,x),z))
 contact=p+R@np.array((0.,-RIM_RADIUS,TAPE_WIDTH/2-.004))
 return contact-hand@np.array((0.,0.,.004)),hand


def box_gap(tri,p,R,size):
 v=(tri-p)@R;flat=v.reshape(-1,3);half=np.asarray(size)/2
 broad=np.maximum(flat.min(0)-half,-half-flat.max(0)).max()
 if broad>.003:return float(broad)
 edges=np.concatenate((v[:,1]-v[:,0],v[:,2]-v[:,1],v[:,0]-v[:,2]))
 normals=np.cross(v[:,1]-v[:,0],v[:,2]-v[:,0]);axes=np.concatenate((np.eye(3),normals,np.cross(edges,[1,0,0]),np.cross(edges,[0,1,0]),np.cross(edges,[0,0,1])))
 lengths=np.linalg.norm(axes,axis=1);axes=np.unique(np.round(axes[lengths>1e-10]/lengths[lengths>1e-10,None],8),axis=0)
 projection=flat@axes.T;radius=np.abs(axes)@half
 return float(np.maximum(projection.min(0)-radius,-radius-projection.max(0)).max())


def mesh_gap(a,b):
 av,bv=a.reshape(-1,3),b.reshape(-1,3)
 gap=float(np.maximum(av.min(0)-bv.max(0),bv.min(0)-av.max(0)).max())
 if gap>.003:return gap
 tri=np.concatenate((a,b));axes=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);n=np.linalg.norm(axes,axis=1);axes=np.unique(np.round(axes[n>1e-10]/n[n>1e-10,None],8),axis=0)
 for start in range(0,len(axes),128):
  pa,pb=av@axes[start:start+128].T,bv@axes[start:start+128].T
  gap=max(gap,float(np.maximum(pa.min(0)-pb.max(0),pb.min(0)-pa.max(0)).max()))
 return gap


class Geometry:
 """All315 native STL pieces, collision xyz/RPY/scale preserved; no cross-task import."""
 def __init__(self):
  self.raw=[];self.defs={p['name']:p for p in layout()[0]}
  for link in ET.parse(YAM_URDF).getroot().findall('link'):
   for c in link.findall('collision'):
    mesh=c.find('geometry/mesh');origin=c.find('origin')
    if mesh is None:raise ValueError('unsupported robot collider')
    data=(Path(YAM_URDF).parent/mesh.get('filename')).read_bytes();count=struct.unpack_from('<I',data,80)[0]
    tri=np.ndarray(count,dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]),buffer=data,offset=84)['v'].astype(float)
    p=np.fromstring(origin.get('xyz','0 0 0'),sep=' ');R=rpy_matrix(np.fromstring(origin.get('rpy','0 0 0'),sep=' '));scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
    self.raw.append((link.get('name'),(tri*scale)@R.T+p))
  if len(self.raw)!=315:raise ValueError('ordinary315 robot collision topology changed')
 def pieces(self,side,q,f):
  fk=forward_kinematics(YAM_URDF,{**dict(zip(ARM_JOINTS,map(float,q))), 'joint7':float(f[0]),'joint8':float(f[1])})
  return [(n,t@fk[n][:3,:3].T+fk[n][:3,3]+BASES[side]) for n,t in self.raw]
 def obstacles(self,poses):
  obs=[]
  for n,d in self.defs.items():p,R=poses[n];obs.append((n,p,R,d['size']))
  for n,_,size,_,_ in PRODUCTS:p,R=poses[n];obs.append((n,p,R,size))
  p,R=poses['roll'];ro=ACTIVE_INNER_RADIUS-TAPE_THICKNESS/2;rm=(CORE_RADIUS+ro)/2
  for k in range(96):
   a=2*math.pi*k/96;c,s=math.cos(a),math.sin(a);Q=R@np.array([[c,-s,0],[s,c,0],[0,0,1.]])
   obs.append(('roll',p+R@np.array((rm*c,rm*s,0)),Q,(ro-CORE_RADIUS,2*rm*math.tan(math.pi/96),TAPE_WIDTH)))
  for i in range(CELLS):p,R=poses[f'tape_{i}'];obs.append((f'tape_{i}',p,R,(CELL_LENGTH,TAPE_THICKNESS,TAPE_WIDTH)))
  p,R=poses['cutter']
  for n,size,off in [('handle',(.10,.025,.025),(0,0,0)),('guard',(.02,.03,.02),(.052,0,.002)),('blade',(.014,.0005,.008),(.062,0,-.016))]:obs.append(('cutter_'+n,p+R@off,R,size))
  obs.append(('cutter_pad',np.array((.36,-.22,TOP+.001)),np.eye(3),(.16,.07,.002)))
  return obs
 def check(self,qs,fs,poses,contact=False):
  if not all(np.isfinite(v).all() for v in [*qs,*fs,*[a for p in poses.values() for a in p]]):raise ValueError('nonfinite roll guard state')
  hands=[self.pieces(s,q,f) for s,q,f in zip(('left','right'),qs,fs)];obs=self.obstacles(poses);hard=(10.,None);target=10.
  centers=np.array([p for _,p,_,_ in obs]);extents=np.array([np.abs(R)@(np.array(size)/2) for _,_,R,size in obs]);omin,omax=centers-extents,centers+extents
  for side,pieces in enumerate(hands):
   pmin=np.array([t.min((0,1)) for _,t in pieces]);pmax=np.array([t.max((0,1)) for _,t in pieces])
   gaps=np.maximum(pmin[:,None]-omax[None,:],omin[None,:]-pmax[:,None]).max(2)
   for i,j in np.argwhere(gaps<=.003):
    n,p,R,size=obs[j];gaps[i,j]=box_gap(pieces[i][1],p,R,size)
   allowed=np.array([[contact and side==1 and name in ('tip_left','tip_right') and n in FAMILY for n,_,_,_ in obs] for name,_ in pieces])
   if allowed.any():target=min(target,float(gaps[allowed].min()))
   values=np.where(allowed,np.inf,gaps);i,j=np.unravel_index(values.argmin(),values.shape)
   if values[i,j]<hard[0]:hard=(float(values[i,j]),('left' if side==0 else 'right',pieces[i][0],obs[j][0]))
   table=pmin[:,2]-TOP;i=int(table.argmin())
   if table[i]<hard[0]:hard=(float(table[i]),('left' if side==0 else 'right',pieces[i][0],'table'))
  # Every collision piece remains included; unresolved overlapping AABBs get
  # face-plane refinement. A positive plane certifies disjoint surfaces.
  aa=[(t.min((0,1)),t.max((0,1))) for _,t in hands[0]];bb=[(t.min((0,1)),t.max((0,1))) for _,t in hands[1]]
  amin,amax=np.array(aa).transpose(1,0,2);bmin,bmax=np.array(bb).transpose(1,0,2)
  gaps=np.maximum(amin[:,None]-bmax[None,:],bmin[None,:]-amax[:,None]).max(2)
  for i,j in np.argwhere(gaps<.0002):gaps[i,j]=mesh_gap(hands[0][i][1],hands[1][j][1])
  mutual=float(gaps.min())
  return dict(safe=bool(hard[0]>=.0002 and mutual>=.0002 and target>=-.001),hard_gap_m=hard[0],hard_pair=hard[1],target_gap_m=target,other_arm_gap_m=mutual)


class RollPickupProbe:
 """Acquire/lift60mm, then HOLD. Requires real opposing roll-family contacts.

 No film-end pinch is attempted: its acquisition needs a separately checked
 physical peel. This checkpoint cannot contribute seal/full-task credit.
 """
 def __init__(self,env,contact_reader):
  self.env=env;self.contact_reader=contact_reader;self.geometry=Geometry();self.previous=array(env._ready).reshape(-1).copy();self.stage='approach_roll';self.started=None;self.failure=None;self.done=False;self.stable=0;self.origin=None;self.grasp=None;self.last_roll=None;self.last_t=None;self.events=[]
  self.start=ik.tcp_pose(self.previous[7:13],env.rig.bases['right'])
 def pose(self,n):
  v=array(self.env._base.arena_world.get_pose_w(n))[0]
  if v.shape!=(7,) or not np.isfinite(v).all() or np.linalg.norm(v[3:])<1e-12:raise ValueError('missing finite native pose')
  return v[:3],quat_xyzw_to_matrix(v[3:])
 def measured(self):
  out=[]
  for side in ('left','right'):
   ids,fids=self.env._ids[side+'_robot'];v=array(self.env._base.scene[side+'_robot'].data.joint_pos)[0]
   q,f=v[ids],v[fids];out.append((q,f,*ik.tcp_pose(q,self.env.rig.bases[side])))
  return out
 def change(self,stage,time):
  self.events.append({'time':time,'from':self.stage,'to':stage});self.stage=stage;self.started=time;self.stable=0;self.start=ik.tcp_pose(self.previous[7:13],self.env.rig.bases['right']);self.start_g=float(self.previous[13])
 def command(self,time):
  if self.failure or self.done:return self.previous.copy(),dict(phase=self.stage,failure=self.failure,complete=self.done)
  try:
   names=[*self.geometry.defs,*[p[0] for p in PRODUCTS],'roll','cutter',*[f'tape_{i}' for i in range(CELLS)]];poses={n:self.pose(n) for n in names};m=self.measured();roll=poses['roll']
   contacts,forces=opposed_contacts(self.contact_reader('right'),time)
   if self.origin is None:self.origin=(roll[0].copy(),roll[1].copy());self.start_g=float(self.previous[13]);self.started=time
   dt=.05 if self.last_t is None else time-self.last_t
   if not np.isfinite(time) or not 0<dt<=.1:raise ValueError('missing/nonfinite controller clock')
   speed=0. if self.last_roll is None else float(np.linalg.norm(roll[0]-self.last_roll)/dt);self.last_roll=roll[0].copy();self.last_t=time
   p,R=grip_target(self.origin);age=time-self.started;g=OPEN;duration=2.
   if self.grasp is None and (np.linalg.norm(roll[0]-self.origin[0])>.005 or rotation_angle_deg(roll[1],self.origin[1])>5.):raise ValueError('loose roll moved before verified grasp')
   if self.grasp is not None:
    off,rel=self.grasp;drift=np.linalg.norm(roll[1].T@(m[1][2]-roll[0])-off);angle=rotation_angle_deg(roll[1].T@m[1][3],rel)
    if drift>.005 or angle>5.:raise ValueError('roll moved within original measured grasp')
   if self.stage=='approach_roll':
    p=p-R[:,2]*.04;duration=max(4.,1.5*np.linalg.norm(p-self.start[0])/.04,1.5*rotation_angle_deg(self.start[1],R)/15.)
   elif self.stage=='enter_roll':duration=6.
   elif self.stage=='close_roll':p,R=self.start;g=self.start_g+(HOLD-self.start_g)*ik.smoothstep(age/2.)
   elif self.stage in ('lift_roll','verified_roll_hold'):
    p=self.start[0]+[0,0,.06 if self.stage=='verified_roll_hold' else .06*ik.smoothstep(age/4.5)];R=self.start[1];g=HOLD;duration=4.5
    if not contacts:raise ValueError('opposing native roll contacts lost during lift')
   else:raise ValueError('unknown roll checkpoint phase')
   if self.stage in ('approach_roll','enter_roll'):
    blend=ik.smoothstep(age/duration);p=self.start[0]+blend*(p-self.start[0]);R=ik.slerp_matrix(self.start[1],R,blend);g=self.start_g+blend*(OPEN-self.start_g)
   if age>duration+6.:raise ValueError('roll checkpoint phase deadline')
   q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),self.previous[7:13],base_pos=self.env.rig.bases['right'],iters=180,tol=1e-6)
   if pe>.0005 or re>.005:raise ValueError('roll target IK unavailable')
   out=self.previous.copy();out[7:13]=q;out[13]=g;slew=np.array([.025]*6+[.04]+[.025]*6+[.04]);out=np.clip(np.clip(out,self.previous-slew,self.previous+slew),self.env._low,self.env._high)
   allowed=self.stage!='approach_roll';actual=self.geometry.check([a[0] for a in m],[a[1] for a in m],poses,allowed);issued=self.geometry.check([out[:6],out[7:13]],[[FINGER_OPEN*out[6]]*2,[FINGER_OPEN*out[13]]*2],poses,allowed)
   if not actual['safe'] or not issued['safe']:raise ValueError('full315 roll clearance rejected')
   settled=np.linalg.norm(m[1][2]-p)<.003 and rotation_angle_deg(m[1][3],R)<2.
   if self.stage in ('approach_roll','enter_roll'):ready=age>=duration and settled
   elif self.stage=='close_roll':ready=age>=2. and contacts and np.all(np.abs(m[1][1])>.004) and np.all(np.abs(m[1][1])<.010)
   else:ready=age>=duration and roll[0][2]-self.origin[0][2]>=.05 and speed<.01 and settled and contacts
   self.stable=self.stable+1 if ready else 0;self.previous=out.copy()
   if self.stable>=5:
    if self.stage=='approach_roll':self.change('enter_roll',time)
    elif self.stage=='enter_roll':self.change('close_roll',time)
    elif self.stage=='close_roll':self.grasp=(roll[1].T@(m[1][2]-roll[0]),roll[1].T@m[1][3]);self.change('lift_roll',time)
    else:self.done=True
   return out,dict(phase=self.stage,failure=None,complete=self.done,roll_lift_m=float(roll[0][2]-self.origin[0][2]),opposed_native_contacts=contacts,normal_magnitudes_N=forces,clearance_actual=actual,clearance_issued=issued,scope='Unscored loose-roll acquisition only; no peel/seal claim')
  except (ValueError,KeyError,TypeError,IndexError) as error:
   self.failure=str(error);return self.previous.copy(),dict(phase=self.stage,failure=self.failure,complete=False)
