"""Flexible patch, pressure-created adhesive bonds and connected leak paths.

The clean, lightly abraded puncture and an unbacked pre-glued patch are prepared
by the operator. Adhesion is a reduced contact/dwell model, not glue chemistry.
Leakage is an explicit interface-connectivity proxy, not a fluid simulation.
"""
import numpy as np
from common import Soft,Grips,sheet,TABLE

TARGET=np.array([.32,0.,TABLE+.020])
PATCH=np.array([.25,-.115,TABLE+.001])
HOLD=np.array([.425,.08*np.sqrt(3),TABLE+.015])
N=9

class Model:
 def __init__(self):
  p,e,k,self.faces=sheet(PATCH,(.024,.024),N,N)
  self.soft=Soft(p,e,k,.0004);self.bonds={};self.dwell=np.zeros(N*N)
  self.held_nodes={};self.tube_grip=Grips();self.offset=np.zeros(3);self.previous_opening={}
  self.events=[];self.time=0.;self.release=0.;self.failed=False
  self.cover=False;self.sealed=False;self.retained=0.;self.max_strain=1.
 def surface(self,x):
  return TARGET[2]+self.offset[2]-(x-TARGET[0]-self.offset[0])**2/(2*.014)
 def tube_point(self):return HOLD+self.offset
 def update(self,hands,dt=.05):
  self.time+=dt
  previous_offset=self.offset.copy()
  self.tube_grip.update(np.array([self.tube_point()]),hands)
  goals=self.tube_grip.targets(hands)
  if goals:
   v=list(goals.values())[0]-HOLD
   self.offset[:2]+=np.clip(v[:2]-self.offset[:2],-.003,.003)
   if abs(v[2])>.025:self.failed=True
  # Adhesive bonds are attached to rubber material, so they move with the tube.
  shift=self.offset-previous_offset
  for j in self.bonds:self.bonds[j]+=shift
  # Capture while the fingers close around the patch; a stationary partly
  # closed pressing hand must not acquire a fresh grasp.
  closing={s:self.previous_opening.get(s,1.)-h['opening']>.001 for s,h in hands.items()}
  others={s:(h if (closing[s] and h['opening']<.30) or s in self.soft.grips.held else dict(h,opening=max(.31,h['opening']))) for s,h in hands.items() if s not in self.tube_grip.held}
  self.previous_opening={s:h['opening'] for s,h in hands.items()}
  before=set(self.soft.grips.held);self.soft.grips.update(self.soft.p,others)
  for s in set(self.soft.grips.held)-before:
   i=self.soft.grips.held[s]['index'];h=hands[s]
   ids=np.flatnonzero(np.linalg.norm(self.soft.p-self.soft.p[i],axis=1)<.0041)
   self.held_nodes[s]={int(j):(self.soft.p[j]-h['tcp'])@h['rotation'] for j in ids}
  for s in list(self.held_nodes):
   if s not in self.soft.grips.held:del self.held_nodes[s]
  p=self.soft.p;cx=TARGET[0]+self.offset[0];cy=TARGET[1]+self.offset[1]
  over=(abs(p[:,0]-cx)<.014)&(abs(p[:,1]-cy)<.10)
  close=over & (abs(p[:,2]-self.surface(p[:,0]))<.0025)
  pressed=np.zeros(len(p),bool)
  # Two actual distal finger pads, including their measured 12.84 mm TCP offset.
  for s,h in hands.items():
   if s in self.tube_grip.held:continue
   for sign in [-1,1]:
    pad=h['tcp']+h['rotation']@np.array([sign*h['opening']*.094/2,0,.01284])
    contact=(abs(p[:,0]-pad[0])<.007)&(abs(p[:,1]-pad[1])<.012)
    contact&=(pad[2]<=self.surface(p[:,0])+.002)&(pad[2]>=self.surface(p[:,0])-.006)
    pressed |= contact & close
  self.dwell=np.where(pressed,self.dwell+dt,np.maximum(0,self.dwell-dt*.2))
  for j in np.flatnonzero((self.dwell>=.20)&close):
   self.bonds[int(j)]=np.array([p[j,0],p[j,1],self.surface(p[j,0])])
  pins=dict(self.bonds)
  for s,local in self.held_nodes.items():
   for j,v in local.items():
    goal=hands[s]['tcp']+hands[s]['rotation']@v
    if j in self.bonds and np.linalg.norm(goal-self.bonds[j])>.003:
     self.bonds.pop(j);pins.pop(j,None);self.dwell[j]=0.
    pins[j]=goal
  def collide(p):
   hit=(abs(p[:,0]-cx)<.014)&(abs(p[:,1]-cy)<.10)
   p[hit,2]=np.maximum(p[hit,2],self.surface(p[hit,0]))
   card=(abs(p[:,0]-PATCH[0])<.0175)&(abs(p[:,1]-PATCH[1])<.0175)
   p[card,2]=np.maximum(p[card,2],TABLE+.0014)
  self.soft.step(others,dt,collide=collide,pins=pins,substeps=8,iterations=24)
  length=np.linalg.norm(self.soft.p[self.soft.edges[:,1]]-self.soft.p[self.soft.edges[:,0]],axis=1)
  self.max_strain=max(self.max_strain,float(np.max(length/self.soft.rest)))
  self.failed=self.failed or self.soft.failed
  self.cover,self.sealed=self.leak_check()
  self.release=self.release+dt if not self.soft.grips.held and not self.tube_grip.held else 0.
  self.retained=self.retained+dt if self.sealed and not self.soft.grips.held else 0.
 def leak_check(self):
  # Locate the real puncture in the patch's material grid, then flood-fill the
  # unbonded interface. An open route to ANY edge leaks; center alignment alone
  # is insufficient. The 3 mm grid is a stated resolution limit.
  p=self.soft.p;target=TARGET+self.offset
  distances=np.linalg.norm(p[:,:2]-target[:2],axis=1);j=int(np.argmin(distances))
  covered=bool(distances[j]<.0025 and abs(p[j,2]-self.surface(p[j,0]))<.003)
  if not covered:return False,False
  blocked=set(self.bonds);blocked.discard(j)
  seen={j};todo=[j]
  while todo:
   q=todo.pop();i,k=divmod(q,N)
   if i in (0,N-1) or k in (0,N-1):return True,False
   for r in [q-N,q+N,q-1,q+1]:
    if r not in blocked and r not in seen:seen.add(r);todo.append(r)
  return True,True
 def measure(self):return dict(puncture_covered=self.cover,sealed_interface=self.sealed,bonded_fraction=len(self.bonds)/(N*N),released_seconds=self.release,seal_retained_seconds=self.retained,failed=self.failed)
 def score(self):
  if self.failed:return 0.
  if self.sealed:return 1. if self.release>=3. and self.retained>=3. else .75
  return .25 if self.cover and len(self.bonds)>=12 else 0.
 def state(self):return dict(points=self.soft.p,bonded=sorted(self.bonds),tube_offset=self.offset,held={s:g['index'] for s,g in self.soft.grips.held.items()},tube_held=list(self.tube_grip.held))
 def draw(self,d):
  # Lightly rounded deflated tube, arranged as an oval with the puncture forward.
  angle=np.linspace(0,2*np.pi,100)
  center=np.c_[.53-.21*np.cos(angle),.16*np.sin(angle),np.full(len(angle),TABLE+.009)]+self.offset
  d.tube('inner_tube',center,.010,(.065,.07,.075),12)
  t=TARGET+self.offset
  rubber=np.array([[x,y,self.surface(np.array([x]))[0]] for x in np.linspace(t[0]-.014,t[0]+.014,13) for y in np.linspace(t[1]-.0425,t[1]+.0425,15)])
  faces=[(i*15+j,(i+1)*15+j,(i+1)*15+j+1,i*15+j+1) for i in range(12) for j in range(14)]
  d.mesh('prepared_rubber',rubber,faces,(.10,.105,.11))
  d.tube('puncture',[t+[0,0,-.0002],t+[0,0,.0001]],.0006,(0.,0.,0.),12)
  # Material points represent the adhesive underside; show the 0.4 mm
  # upper face above the rubber/card instead of coplanar depth fighting.
  d.mesh('pre_glued_patch',self.soft.p+np.array([0.,0.,.0004]),self.faces,(.95,.57,.10))
