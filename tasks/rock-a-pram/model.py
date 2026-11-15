"""Planar mass/inertia model of a full-size pram on a level floor.

The wheeled body has x/y/yaw dynamics and rolling resistance. Measured gripper
motion acts through bounded compliant handle grasps. Vertical suspension,
wheel lift, occupant motion and infant response are outside this model.
"""
import numpy as np
from common import Grips
HANDLE=np.array([.32,0.,1.02]);MASS=15.;INERTIA=2.1
class Model:
 def __init__(self):
  self.q=np.zeros(3);self.v=np.zeros(3);self.grips=Grips();self.events=self.grips.events
  self.local=np.array([[HANDLE[0],y,HANDLE[2]] for y in np.linspace(-.235,.235,95)])
  self.cycles=0;self.phase=0;self.max_speed=0.;self.max_accel=0.;self.max_drift=0.;self.max_yaw=0.;self.rest=0.;self.time=0.;self.failed=False;self.base=None
 def bind_native(self,base):
  self.base=base;self.sync_native()
 def sync_native(self):
  if self.base is None:return
  import torch
  from body_contact import SHELLS
  quat=[0.,0.,np.sin(self.q[2]/2),np.cos(self.q[2]/2)]
  for shape in SHELLS:
   pose=torch.tensor([list(self.world(shape.pos))+quat],device=self.base.device,dtype=torch.float32)
   pose[:,:3]+=self.base.scene.env_origins
   self.base.scene[shape.name].write_root_pose_to_sim_index(root_pose=pose)
 @staticmethod
 def configure_station():
  from yam_arena import station
  station.TABLE_SIZE=(.14,.90,.06);station.TABLE_CENTER=(0.,0.,station.TABLE_TOP-.03)
 def R(self):
  c,s=np.cos(self.q[2]),np.sin(self.q[2]);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
 def world(self,p):return (np.asarray(p)-[.70,0,0])@self.R().T+[.70+self.q[0],self.q[1],0.]
 def handle(self):return self.world(self.local)
 def update(self,hands,dt=.05):
  self.time+=dt;self.grips.update(self.handle(),hands);targets=self.grips.targets(hands);last=self.v.copy()
  for _ in range(10):
   pts=self.handle();force=np.zeros(3)
   for i,target in targets.items():
    lever=pts[i,:2]-[.70+self.q[0],self.q[1]];velocity=self.v[:2]+self.v[2]*np.array([-lever[1],lever[0]])
    f=650*(target[:2]-pts[i,:2])-45*velocity
    f*=min(1.,25./max(np.linalg.norm(f),1e-9));force[:2]+=f;force[2]+=lever[0]*f[1]-lever[1]*f[0]
   # Smooth Coulomb rolling resistance; lateral tyre resistance is larger.
   local_v=self.R()[:2,:2].T@self.v[:2];res=np.array([1.3*np.tanh(local_v[0]/.01),18*np.tanh(local_v[1]/.01)])
   force[:2]-=self.R()[:2,:2]@res;force[2]-=2.8*np.tanh(self.v[2]/.015)
   self.v+=force/np.array([MASS,MASS,INERTIA])*(dt/10);self.q+=self.v*(dt/10)
  speed=float(np.linalg.norm(self.v[:2]));acc=float(np.linalg.norm(self.v[:2]-last[:2])/dt)
  self.max_speed=max(self.max_speed,speed);self.max_accel=max(self.max_accel,acc);self.max_drift=max(self.max_drift,abs(float(self.q[1])));self.max_yaw=max(self.max_yaw,abs(float(self.q[2])))
  if self.phase==0 and self.q[0]>=.165:self.phase=1
  if self.phase==1 and self.q[0]<=.035:self.phase=0;self.cycles+=1;self.events.append(dict(event='complete_rock',cycle=self.cycles,time=self.time))
  self.rest=self.rest+dt if speed<.01 and abs(self.v[2])<.02 and not self.grips.held else 0.
  if abs(self.q[0])>.35 or abs(self.q[1])>.12 or abs(self.q[2])>.35:self.failed=True
 def measure(self):return dict(completed_rocks=self.cycles,fore_aft_m=float(self.q[0]),side_drift_m=float(self.q[1]),max_side_drift_m=self.max_drift,max_yaw_degrees=float(np.degrees(self.max_yaw)),max_speed_m_s=self.max_speed,max_acceleration_m_s2=self.max_accel,released_rest_seconds=self.rest,failed=self.failed)
 def score(self):
  m=self.measure()
  if m['failed'] or m['max_side_drift_m']>.04 or m['max_yaw_degrees']>10 or m['max_speed_m_s']>.30 or m['max_acceleration_m_s2']>1.5:return 0.
  if self.cycles>=10 and self.rest>=3. and abs(self.q[0])<=.05:return 1.
  return .5 if self.cycles>=5 else 0.
 def state(self):return dict(q=self.q,v=self.v,held=list(self.grips.held))
 def draw(self,d):
  R=self.R()
  def tube(n,pts,r,c,sides=10):d.tube(n,self.world(pts),r,c,sides)
  def box(n,p,s,c):d.box(n,self.world(p),s,c,R)
  for x,label in [(.35,'rear'),(1.00,'front')]:
   for y,side in [(-.255,'right'),(.255,'left')]:
    tube(label+'_'+side+'_wheel',[[x,y-.025,.108],[x,y+.025,.108]],.12,(.055,.055,.06),24)
    spoke=np.array([[x-.10*np.sin(self.q[0]/.12),y-.027,.108-.10*np.cos(self.q[0]/.12)],[x+.10*np.sin(self.q[0]/.12),y-.027,.108+.10*np.cos(self.q[0]/.12)]])
    tube(label+'_'+side+'_spoke',spoke,.006,(.65,.65,.67))
  for y,side in [(-.22,'right'),(.22,'left')]:
   tube(side+'_rear_frame',[[.35,y,.12],[.85,y,.65]],.013,(.32,.34,.37))
   tube(side+'_front_frame',[[1.00,y,.12],[.42,y,.65]],.013,(.32,.34,.37))
   tube(side+'_handle_frame',[[.35,y,.30],[.50,y,.70],[.32,y,1.02]],.013,(.32,.34,.37))
  tube('handle',[[.32,-.235,1.02],[.32,.235,1.02]],.014,(.12,.12,.14))
  box('carrycot_base',[.72,0,.57],[.76,.40,.07],(.10,.30,.42))
  for y in [-.205,.205]:box('carrycot_side_'+str(y).replace('.','_').replace('-','n'),[.72,y,.69],[.76,.035,.22],(.10,.30,.42))
  for x in [.325,1.115]:box('carrycot_end_'+str(x).replace('.','_'),[x,0,.69],[.035,.40,.22],(.10,.30,.42))
  box('mattress',[.72,0,.62],[.70,.36,.05],(.90,.89,.83))
  box('weighted_doll_body',[.72,0,.69],[.27,.15,.09],(.82,.63,.23))
  tube('weighted_doll_head',[[.92,-.035,.70],[.92,.035,.70]],.065,(.70,.48,.33),20)
  for y in [-.055,.055]:tube('doll_leg_'+str(y).replace('.','_').replace('-','n'),[[.58,y,.69],[.45,y,.67]],.025,(.82,.63,.23))
  box('doll_restraint',[.72,0,.747],[.035,.21,.006],(.18,.18,.20))
  # Fixed, unbranded floor marks make excursion and drift visible.
  d.box('near_floor_mark',[.35,-.35,-.010],[.006,.12,.002],(.82,.55,.10));d.box('far_floor_mark',[.55,-.35,-.010],[.006,.12,.002],(.82,.55,.10))
