"""Reduced compliant tube, loaded rolling abrasion and gravity-driven emptying.

The tube's two axes deform under measured distal pads. Rolling is no-slip at
the board and the contacting pad; skin loss requires relative loaded sliding
inside the tube. These are disclosed material laws, not native silicone FEM.
No demo stage, action target, progress variable or scorer drives the material.
"""
import numpy as np
from common import TABLE,grip_candidates

CENTER=np.array([.34,0.,TABLE+.019])
LENGTH=.120
RADIUS=.0175
CLOVE_Z=.0105
WALL=.0015
BOARD=TABLE+.003

def rot(axis,angle):
 axis=np.asarray(axis,float);axis/=np.linalg.norm(axis);x,y,z=axis
 K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
 return np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*(K@K)

def ellipsoid(center,axes,R,nu=32,nv=16):
 a=np.arange(nu)*2*np.pi/nu;b=np.linspace(-np.pi/2,np.pi/2,nv)
 p=np.array([[np.cos(v)*np.cos(u),np.cos(v)*np.sin(u),np.sin(v)] for v in b for u in a])*axes
 faces=[(j*nu+i,j*nu+(i+1)%nu,(j+1)*nu+(i+1)%nu,(j+1)*nu+i) for j in range(nv-1) for i in range(nu)]
 return p@R.T+center,faces

class Model:
 def __init__(self):
  self.center=CENTER.copy();self.center[2]=BOARD+RADIUS
  self.R=np.eye(3);self.v=np.zeros(3);self.radius_z=RADIUS
  self.held=None;self.grip_local=None;self.grip_R=None
  self.roll=0.;self.s=0.;self.svel=0.;self.inside=True
  self.clove=self.center.copy();self.clove[2]=BOARD+CLOVE_Z
  self.clove_v=np.zeros(3);self.clove_R=np.eye(3)
  self.skin=np.ones((16,8));self.skin_work=np.zeros_like(self.skin)
  self.skin_angle=np.linspace(0,2*np.pi,16,endpoint=False)
  self.skin_y=np.linspace(-.011,.011,8)
  self.previous={};self.events=[];self.time=0.;self.release=0.
  self.crushed=False;self.failed=False;self.travel=0.;self.max_compression=0.
  self.grip_released=False;self.flakes=[];self.pads_contact=0;self.held_end=1;self.grip_squeeze=RADIUS

 def axis(self):return self.R[:,1]

 def grab(self,hands):
  if self.held:
   if hands[self.held]['opening']>.65:
    self.events.append(dict(event='release_tube',time=self.time));self.held=None
   else:
    h=hands[self.held];old=self.center.copy()
    self.center=h['tcp']+h['rotation']@self.grip_local
    self.R=h['rotation']@self.grip_R;self.v=(self.center-old)/.05
    return
  for side,h in hands.items():
   prev=self.previous.get(side,h)
   if prev['opening']-h['opening']<=.002 or h['opening']>.43:continue
   # Grasp only the open end wall, with opposing fingers straddling the tube.
   for sign in [-1,1]:
    end=self.center+self.axis()*sign*(LENGTH/2-.014)
    ids,local=grip_candidates([end],h)
    if not len(ids) or abs(h['rotation'][:,0]@self.axis())>.4:continue
    if np.linalg.norm((end-h['tcp'])@h['rotation']-np.array([0,0,-.005]))>.025:continue
    self.held=side;self.held_end=sign;self.grip_local=(self.center-h['tcp'])@h['rotation'];self.grip_R=h['rotation'].T@self.R
    self.events.append(dict(event='grasp_tube_end',side=side,time=self.time));return

 def update(self,hands,dt=.05):
  self.time+=dt;self.grab(hands);old_center=self.center.copy();self.pads_contact=0
  axis=self.axis();floor=BOARD
  requested=min(RADIUS,hands[self.held]['opening']*.094/2+WALL) if self.held else RADIUS
  self.grip_squeeze+=(requested-self.grip_squeeze)*.7
  if not self.held:
   self.v[2]-=9.81*dt;self.center+=self.v*dt;self.v*=.90
   # Horizontal tube rests on board. Large rotations outside a grasp are not
   # needed for the fixed reset, but a released tilted tube can fall freely.
   support=RADIUS*np.sqrt(max(0.,1-axis[2]**2))+LENGTH/2*abs(axis[2])
   if self.center[2]<floor+support:self.center[2]=floor+support;self.v[2]=0.
   pads=[]
   if abs(axis[2])<.15 and self.center[2]<floor+RADIUS+.005:
    for side,h in hands.items():
     prev=self.previous.get(side,h)
     for sign in [-1,1]:
      local=np.array([sign*h['opening']*.094/2,0,.01284])
      pad=h['tcp']+h['rotation']@local
      prior=prev['tcp']+prev['rotation']@np.array([sign*prev['opening']*.094/2,0,.01284])
      delta=pad-self.center
      if abs(delta@axis)<LENGTH/2-.020 and abs(delta[0])<RADIUS+.008 and floor+.008<pad[2]<floor+2*RADIUS+.002:
       pads.append((pad,prior));self.pads_contact+=1
   compression=0.
   if pads:
    top=min(p[0][2] for p in pads);compression=np.clip(floor+2*RADIUS-top,0.,.027)
    self.radius_z=RADIUS-compression/2;self.center[2]=floor+self.radius_z
    dx=float(np.mean([p[0][0]-p[1][0] for p in pads]))*.5
    # Actual finite pad movement, not intended motion, turns the compliant tube.
    dx=float(np.clip(dx,-.012,.012));self.center[0]+=dx;self.roll+=dx/max(self.radius_z,.007)
    if self.inside and compression>(2*RADIUS-2*WALL-2*CLOVE_Z):
     relative=abs(dx)*(RADIUS/.009-1.)
     load=np.clip((compression-(2*RADIUS-2*WALL-2*CLOVE_Z))/.002,0.,1.5)
     # Skin sectors meeting either compressed wall accumulate tangential work.
     phase=self.skin_angle+self.roll*RADIUS/.009
     loaded=(abs(np.cos(phase))>.45)[:,None]
     self.skin_work+=loaded*relative*load
     newly=(self.skin_work>=.050)&(self.skin>.5)
     if newly.any():
      self.events.append(dict(event='skin_detached',patches=int(newly.sum()),time=self.time))
      for i,j in zip(*np.where(newly)):self.flakes.append(dict(index=(int(i),int(j)),local=(self.clove-self.center)@self.R))
     self.skin[newly]=0.;self.travel+=abs(dx)
    if self.inside and 2*self.radius_z-2*WALL<.017:self.crushed=True
   else:self.radius_z+=(RADIUS-self.radius_z)*.5
   self.max_compression=max(self.max_compression,float(compression))
  else:self.radius_z+=(RADIUS-self.radius_z)*.5
  axis=self.axis()
  if self.inside:
   # The clove slides out only when gravity overcomes the disclosed dry friction.
   drive=-9.81*axis[2];friction=.28*9.81*np.sqrt(max(0.,1-axis[2]**2))
   acceleration=np.sign(drive)*max(0.,abs(drive)-friction)
   self.svel=(self.svel+acceleration*dt)*.96;self.s+=self.svel*dt
   self.clove=self.center+axis*self.s-self.R[:,2]*(max(0.,self.radius_z-WALL-CLOVE_Z))
   self.clove_R=self.R@rot([0,1,0],self.roll*RADIUS/.009)
   if abs(self.s)>LENGTH/2+.016:
    self.inside=False;self.clove_v=self.v+axis*self.svel
    self.events.append(dict(event='clove_exited',time=self.time))
  else:
   self.clove_v[2]-=9.81*dt;self.clove+=self.clove_v*dt
   if self.clove[2]<floor+CLOVE_Z:
    if self.clove_v[2]<-2.5:self.crushed=True
    self.clove[2]=floor+CLOVE_Z;self.clove_v[2]=0.;self.clove_v[:2]*=.7
   self.clove_R=np.eye(3)
  # Detached papery skin adheres to the silicone and remains in the tube,
  # as in the real rolling peeler. Rinsing the tube is an operator reset.
  self.failed |= not np.isfinite(self.clove).all() or not (.05<self.center[0]<.9 and abs(self.center[1])<.5)
  clear=not self.held and all(np.linalg.norm(h['tcp']-self.clove)>.05 and np.linalg.norm(h['tcp']-self.center)>.07 for h in hands.values())
  self.release=self.release+dt if clear and not self.inside else 0.
  self.previous={s:{k:(v.copy() if isinstance(v,np.ndarray) else v) for k,v in h.items()} for s,h in hands.items()}

 def measure(self):
  offset=self.clove-self.center;along=float(offset@self.axis());radial=np.linalg.norm(offset-along*self.axis())
  separate=abs(along)>LENGTH/2+.021 or radial>RADIUS+.021
  return dict(clove_clear_of_tube=bool(separate),peeled_fraction=float(1-self.skin.mean()),out_of_tube=not self.inside,intact=not self.crushed,
              released_seconds=self.release,loaded_roll_travel_m=self.travel,max_compression_mm=self.max_compression*1000,
              clove_on_board=bool(.10<self.clove[0]<.65 and abs(self.clove[1])<.25 and self.clove[2]<BOARD+.015),failed=bool(self.failed))
 def score(self):
  m=self.measure()
  if self.failed or self.crushed:return 0.
  amount=m['peeled_fraction']
  if not m['out_of_tube'] or not m['clove_clear_of_tube']:return .25 if amount>=.50 else 0.
  if not m['clove_on_board']:return 0.
  if amount>=.95:return 1. if self.release>=3 else .75
  return .5 if amount>=.75 else .25 if amount>=.50 else 0.
 def state(self):return dict(tube_center=self.center,tube_rotation=self.R,tube_radius_z=self.radius_z,clove_center=self.clove,skin=self.skin,skin_work=self.skin_work,inside=self.inside,held=self.held,roll=self.roll)
 def draw(self,d):
  angle=np.linspace(0,2*np.pi,49);r=RADIUS**2/max(self.radius_z,.006)
  # Open tube wall with finite thickness; visible squash is the simulated state.
  pts=[];levels=np.array([-LENGTH/2,-.035,0,.035,LENGTH/2])
  for y in levels:
   factor=np.clip((self.held_end*y-.023)/.022,0,1)
   rx=r*(1-factor)+self.grip_squeeze*factor
   rz=self.radius_z*(1-factor)+(RADIUS*RADIUS/max(self.grip_squeeze,.009))*factor
   for inner in [0,1]:
    pts.extend([[(rx-inner*WALL)*np.cos(t),y,(rz-inner*WALL)*np.sin(t)] for t in angle[:-1]])
  faces=[]
  for section in range(len(levels)-1):
   for i in range(48):
    j=(i+1)%48;a=section*96;b=(section+1)*96
    faces.extend([(a+i,a+j,b+j,b+i),(a+48+i,b+48+i,b+48+j,a+48+j)])
  for a in [0,(len(levels)-1)*96]:
   for i in range(48):
    j=(i+1)%48;faces.append((a+i,a+48+i,a+48+j,a+j))
  d.mesh('silicone_peeler',np.asarray(pts)@self.R.T+self.center,faces,(.22,.61,.24))
  cp,cf=ellipsoid(self.clove,[.009,.016,CLOVE_Z],self.clove_R)
  # Make a clove with a tapered tip rather than a coloured sphere.
  local=(cp-self.clove)@self.clove_R;local[:,0]*=1-.28*(local[:,1]/.016);cp=local@self.clove_R.T+self.clove
  d.mesh('garlic_flesh',cp,cf,(.94,.88,.67))
  for i in range(16):
   for j in range(8):
    if not self.skin[i,j]:continue
    az=np.linspace(self.skin_angle[i],self.skin_angle[i]+2*np.pi/16,3);yy=np.linspace(-.014+j*.0035,-.014+(j+1)*.0035,3)
    p=[]
    for y in yy:
     cross=np.sqrt(max(.03,1-(y/.017)**2))
     for a in az:p.append([(.0095*np.cos(a)*cross)*(1-.28*y/.016),y,(CLOVE_Z+.0005)*np.sin(a)*cross])
    f=[(a*3+b,(a+1)*3+b,(a+1)*3+b+1,a*3+b+1) for a in range(2) for b in range(2)]
    d.mesh('skin_%02d_%02d'%(i,j),np.asarray(p)@self.clove_R.T+self.clove,f,(.69,.48,.33))
  # A detached patch is removed from the clove's surface mesh, not hidden whole.
  for i in range(16):
   for j in range(8):
    if self.skin[i,j]:continue
    d.mesh('skin_%02d_%02d'%(i,j),np.empty((0,3)),[],(.69,.48,.33))
  for k,f in enumerate(self.flakes):
   i,j=f['index'];a=self.skin_angle[i]
   c=self.center+self.R@np.array([(r-WALL)*np.cos(a),self.skin_y[j],(self.radius_z-WALL)*np.sin(a)])
   d.mesh('flake_%03d'%k,c+np.array([[-.002,-.002,0],[.002,-.002,.0005],[.001,.002,0]]),[(0,1,2)],(.69,.48,.33))
