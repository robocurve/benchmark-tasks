"""Flexible polar cloth with sliding annular friction and a screw clamp.

Only cloth intersecting the real hoop annulus is retained. Interior vertices
remain free under gravity. Turning the grasped screw changes the friction;
pulling fabric through a loose ring changes which material lies inside it.
The reduced friction and stretch laws are approximate, not calibrated textile
or thread mechanics. There is no progress-triggered flattening or pinning.
"""
import numpy as np
from common import TABLE,Soft,grip_candidates
from scipy.sparse import coo_matrix,diags
from scipy.sparse.linalg import factorized

CENTER=np.array([.32,0.,TABLE+.045])
RADIUS=.065
NA=32
NR=12
DR=.010
SCREW=CENTER+np.array([.079,0.,.004])

class Model:
 def __init__(self):
  self.theta=np.arange(NA)*2*np.pi/NA;self.radial=np.c_[np.cos(self.theta),np.sin(self.theta),np.zeros(NA)]
  points=np.vstack([CENTER]+[CENTER+self.radial*r*DR for r in range(1,NR+1)])
  self.faces=[];edges=[];ks=[]
  for j in range(NA):self.faces.append((0,1+j,1+(j+1)%NA));edges.append((0,1+j));ks.append(1.)
  for r in range(1,NR+1):
   for j in range(NA):
    a=1+(r-1)*NA+j;b=1+(r-1)*NA+(j+1)%NA
    edges.append((a,b));ks.append(1.)
    if r<NR:
     c=a+NA;e=1+r*NA+(j+1)%NA
     edges.extend([(a,c),(a,e),(b,c)]);ks.extend([1.,.7,.7]);self.faces.extend([(a,c,b),(b,c,e)])
    if r<NR-1:edges.append((a,a+2*NA));ks.append(.035)
  self.soft=Soft(points,edges,ks,radius=.0005,floor=TABLE)
  # Unstretched cotton starts with extra material, expressed as real geometric
  # slack rather than a different rest length switched during the trial.
  self.soft.rest*=1.06
  self.soft.inverse_mass=np.ones(len(points));self.soft.inverse_mass[0]=4/NA
  rad=np.linalg.norm(points[:,:2]-CENTER[:2],axis=1)
  self.soft.p[:,2]-=.008*np.maximum(0,1-(rad/RADIUS)**2)
  self.soft.p[:,2]+=.004*np.sin(np.arctan2(points[:,1],points[:,0]-.32)*5)*np.minimum(rad/RADIUS,1)
  self.anchor_s=np.full(NA,RADIUS/DR)
  self.tightness=0.;self.turns=0.;self.knob_grip=None;self.knob_reference=None
  self.previous={};self.events=[];self.time=0.;self.release=0.;self.failed=False;self.strain_time=0.
  self.captured=np.ones(NA,bool);self.held_nodes={};self.max_strain=1.
  self.initial_slack=None
  e=self.soft.edges;n=len(points);k=self.soft.k*220.
  a,b=e[:,0],e[:,1]
  self.spring_k=k
  self.spring_L=coo_matrix((np.r_[k,k,-k,-k],(np.r_[a,b,a,b],np.r_[a,b,b,a])),shape=(n,n)).tocsc()
  self.masses=np.full(n,.012/n);self.masses[0]*=4


 def clamp(self,p):
  # Interpolate actual material edges at each contact sector. During slip the
  # material coordinate changes; the interior is never pinned to the hoop.
  js=np.arange(NA);s=self.anchor_s;lo=np.floor(s).astype(int);u=s-lo
  valid=(lo>=1)&(lo<NR);safe=np.clip(lo,1,NR-1)
  a=1+(safe-1)*NA+js;b=a+NA;q=(1-u[:,None])*p[a]+u[:,None]*p[b]
  target=CENTER+self.radial*RADIUS
  error=np.sum((q-target)*self.radial,axis=1)
  yield_m=.0008+.003*self.tightness
  ds=-np.sign(error)*np.maximum(0.,abs(error)-yield_m)/DR
  self.anchor_s=np.clip(s+np.clip(ds,-.025,.025)*(1-self.tightness),.8,NR-.1)
  lo=np.floor(self.anchor_s).astype(int);u=self.anchor_s-lo
  self.captured=(lo>=1)&(lo<NR);safe=np.clip(lo,1,NR-1)
  a=1+(safe-1)*NA+js;b=a+NA;q=(1-u[:,None])*p[a]+u[:,None]*p[b]
  correction=(target-q)*.70*self.captured[:,None];den=(1-u)**2+u*u
  np.add.at(p,a,correction*((1-u)/den)[:,None]);np.add.at(p,b,correction*(u/den)[:,None])

 def screw(self,hands):
  if self.knob_grip and hands[self.knob_grip]['opening']>.38:
   self.events.append(dict(event='release_screw',time=self.time));self.knob_grip=None
  if not self.knob_grip:
   for side,h in hands.items():
    prev=self.previous.get(side,h)
    if prev['opening']-h['opening']<.001 or h['opening']>.25:continue
    ids,local=grip_candidates([SCREW],h)
    if len(ids) and abs(h['rotation'][:,2]@np.array([0.,1.,0.]))>.85:
     self.knob_grip=side;self.knob_reference=h['rotation'].copy();self.events.append(dict(event='grasp_screw',time=self.time));break
  if self.knob_grip:
   h=hands[self.knob_grip]
   if np.linalg.norm(h['tcp']-SCREW)>.035:self.knob_grip=None;return
   a=self.knob_reference[:,0];b=h['rotation'][:,0]
   change=np.arctan2(np.dot(np.cross(a,b),[0.,1.,0.]),np.dot(a,b))
   self.turns=max(0.,self.turns+change/(2*np.pi));self.knob_reference=h['rotation'].copy()
   self.tightness=float(np.clip(self.turns/.25,0,1))

 def cloth_step(self,pins,dt):
  # Projective dynamics: stiff in-plane springs with lumped cloth mass.
  # The sparse global solve distributes a tug through the fabric instead of
  # stretching the few edges next to a grip as a short Jacobi solve can do.
  p=self.soft.p;v=self.soft.v;n=len(p);h=dt/4;edges=self.soft.edges
  starts={j:p[j].copy() for j in pins}
  for sub in range(4):
   old=p.copy();v[:,2]-=9.81*h;v*=np.exp(-3*h);prediction=p+v*h
   inertia=self.masses/(h*h);A=self.spring_L+diags(inertia)
   rhs_base=inertia[:,None]*prediction
   js=np.arange(NA);lo=np.clip(np.floor(self.anchor_s).astype(int),1,NR-1);u=self.anchor_s-lo
   aa=1+(lo-1)*NA+js;bb=aa+NA;ca=1-u;cb=u;strength=4000.
   A+=coo_matrix((strength*np.r_[ca*ca,cb*cb,ca*cb,ca*cb],(np.r_[aa,bb,aa,bb],np.r_[aa,bb,bb,aa])),shape=(n,n)).tocsc()
   targets=CENTER+self.radial*RADIUS
   np.add.at(rhs_base,aa,strength*ca[:,None]*targets);np.add.at(rhs_base,bb,strength*cb[:,None]*targets)
   for j,goal in pins.items():
    A[j,j]+=1e6;rhs_base[j]+=1e6*(starts[j]+(sub+1)/4*(goal-starts[j]))
   solve=factorized(A.tocsc())
   for iteration in range(8):
    delta=p[edges[:,1]]-p[edges[:,0]];length=np.linalg.norm(delta,axis=1)
    projected=delta*(self.soft.rest/np.maximum(length,1e-9))[:,None]
    rhs=rhs_base.copy();contribution=self.spring_k[:,None]*projected
    np.add.at(rhs,edges[:,0],-contribution);np.add.at(rhs,edges[:,1],contribution)
    p[:]=np.column_stack([solve(rhs[:,k]) for k in range(3)])
    p[:,2]=np.maximum(p[:,2],TABLE+.0005)
   q=ca[:,None]*p[aa]+cb[:,None]*p[bb]
   force=strength*np.sum((q-targets)*self.radial,axis=1)
   friction=.025+1.5*self.tightness
   slip=np.sign(force)*np.maximum(0.,abs(force)-friction)/strength
   self.anchor_s=np.clip(self.anchor_s-np.clip(slip/DR,-.12,.12)*(1-self.tightness),1.0,NR-.1)
   v[:]=(p-old)/h
   floor=p[:,2]<=TABLE+.00051;v[floor,:2]*=.65;v[floor,2]=np.maximum(0,v[floor,2])
  self.captured=(self.anchor_s>1.05)&(self.anchor_s<NR-.11)
  if not np.isfinite(p).all():self.soft.failed=True

 def update(self,hands,dt=.05):
  self.time+=dt;self.screw(hands)
  filtered={s:h for s,h in hands.items() if s!=self.knob_grip and np.linalg.norm(h['tcp']-SCREW)>.025}
  for side in list(self.soft.grips.held):
   if side not in filtered:self.soft.grips.held.pop(side)
  before=set(self.soft.grips.held);self.soft.grips.update(self.soft.p,filtered)
  for side in set(self.soft.grips.held)-before:
   i=self.soft.grips.held[side]['index'];h=hands[side]
   ids=np.flatnonzero(np.linalg.norm(self.soft.p-self.soft.p[i],axis=1)<.012)
   self.held_nodes[side]={int(k):(self.soft.p[k]-h['tcp'])@h['rotation'] for k in ids}
  for side in list(self.held_nodes):
   if side not in self.soft.grips.held:del self.held_nodes[side]
  pins={}
  for s,nodes in self.held_nodes.items():
   for j,local in nodes.items():pins[j]=hands[s]['tcp']+hands[s]['rotation']@local
  self.cloth_step(pins,dt)
  length=np.linalg.norm(self.soft.p[self.soft.edges[:,1]]-self.soft.p[self.soft.edges[:,0]],axis=1)
  strain=float(np.max(length/self.soft.rest));self.max_strain=max(self.max_strain,strain)
  self.strain_time=self.strain_time+dt if strain>1.35 else 0.
  self.failed |= self.soft.failed or self.strain_time>.15 or np.min(self.anchor_s)<1.5
  clear=not self.soft.grips.held and not self.knob_grip and all(np.min(np.linalg.norm(self.soft.p-h['tcp'],axis=1))>.035 for h in hands.values())
  self.release=self.release+dt if clear else 0.
  self.previous={s:{k:(v.copy() if isinstance(v,np.ndarray) else v) for k,v in h.items()} for s,h in hands.items()}

 def measure(self):
  p=self.soft.p;rad=np.linalg.norm(p[:,:2]-CENTER[:2],axis=1);inner=rad<.048
  height=p[inner,2]-CENTER[2]
  return dict(clamped_fraction=float(self.captured.mean()),screw_quarter_turns=self.turns*4,
              centre_sag_mm=float(CENTER[2]-p[0,2])*1000,
              central_height_range_mm=float(np.ptp(height))*1000,
              max_central_deflection_mm=float(np.max(abs(height)))*1000,
              released_seconds=self.release,max_strain=self.max_strain,failed=bool(self.failed))
 def score(self):
  m=self.measure()
  if self.failed:return 0.
  if m['clamped_fraction']<.95:return .25 if m['clamped_fraction']>=.5 else 0.
  if self.tightness<.9:return .25 if m['max_central_deflection_mm']<=5. else 0.
  if m['max_central_deflection_mm']<=5. and m['central_height_range_mm']<=5.:
   return 1. if self.release>=3 else .75
  return .5
 def state(self):return dict(points=self.soft.p,material_at_rim=self.anchor_s,tightness=self.tightness,turns=self.turns,held=list(self.soft.grips.held),knob_held=self.knob_grip)
 def draw(self,d):
  # Ordinary hoop supported at its rim, leaving a 37 mm gap under the fabric.
  for n,a in enumerate([np.pi/2,7*np.pi/6,11*np.pi/6]):
   pos=CENTER+np.array([np.cos(a)*RADIUS,np.sin(a)*RADIUS,-.0265])
   d.box('support_%d'%n,pos,[.018,.018,.037],(.45,.29,.15))
  a=np.linspace(0,2*np.pi,97)
  for name,r,z,color in [('inner',RADIUS-.002,CENTER[2]-.004,(.72,.49,.26)),('outer',RADIUS+.0025,CENTER[2]-.001,(.82,.62,.35))]:
   points=[]
   for zz in [-.004,.004]:
    for rr in [r-.0018,r+.0018]:points.extend(np.c_[CENTER[0]+rr*np.cos(a[:-1]),CENTER[1]+rr*np.sin(a[:-1]),np.full(96,z+zz)])
   faces=[]
   for i in range(96):
    j=(i+1)%96;faces.extend([(i,j,96+j,96+i),(192+i,288+i,288+j,192+j),(i,192+i,192+j,j),(96+i,96+j,288+j,288+i)])
   d.mesh(name+'_hoop',points,faces,color)
  d.mesh('cotton_fabric',self.soft.p,self.faces,(.82,.80,.65))
  # Sparse woven grid follows the actual material vertices and makes slack visible.
  for j in range(0,NA,4):
   p=np.vstack([self.soft.p[0],self.soft.p[1+j::NA]])+np.array([0,0,.0006]);d.tube('weave_radial_%d'%j,p,.00025,(.33,.43,.40),4)
  for r in [3,5,7,9,11]:
   p=self.soft.p[1+(r-1)*NA:1+r*NA]+np.array([0,0,.0006]);d.tube('weave_ring_%d'%r,np.vstack([p,p[0]]),.00025,(.33,.43,.40),4)
  d.box('brass_lug_left',SCREW+[0,-.012,-.002],[.008,.011,.015],(.62,.45,.15))
  d.box('brass_lug_right',SCREW+[0,.012,-.002],[.008,.011,.015],(.62,.45,.15))
  d.tube('screw_shaft',[SCREW+[0,-.02,0],SCREW+[0,.02,0]],.0015,(.55,.56,.58),12)
  theta=self.turns*2*np.pi
  direction=np.array([np.cos(theta),0,-np.sin(theta)])
  d.tube('thumb_screw', [SCREW-direction*.007,SCREW+direction*.007],.0025,(.68,.49,.17),10)
