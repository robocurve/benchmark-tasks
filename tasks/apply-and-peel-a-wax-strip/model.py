"""Coarse flexible wax-strip handling on a rigid silicone leg form.

A strip is a 5x17 mass-point sheet. Pressed contact creates breakable surface
bonds; pulling stretches and releases those bonds in a peel front. Hair is not
simulated and this task makes no hair-removal or skin-safety claim. It measures
application, smoothing and peeling of a real cold-wax strip on a practice form.
"""
import numpy as np
from common import Soft,sheet,TABLE
CENTER=np.array([.36,0.,.81]);RADIUS=.060
class Model:
 def __init__(self):
  p,e,k,self.faces=sheet((.20,0.,TABLE+.002),(.045,.160),5,17);self.soft=Soft(p,e,k,.0006)
  self.bonds={};self.pressed=np.zeros(len(p),bool);self.removed=np.zeros(len(p),bool);self.peak_bonds=0;self.applied_flat=0.
  self.peel_reference=None;self.peel_start=None;self.peel_end=None;self.peel_high=False;self.events=[];self.failed=False;self.time=0.;self.release=0.
 def surface(self,x):return CENTER[2]+np.sqrt(np.maximum(0.,RADIUS**2-(x-CENTER[0])**2))+.0006
 def collide(self,p):
  d=p[:,[0,2]]-CENTER[[0,2]];r=np.linalg.norm(d,axis=1);hit=(r<RADIUS+.0006)&(abs(p[:,1])<.16)
  p[hit,0]=CENTER[0]+d[hit,0]/np.maximum(r[hit],1e-9)*(RADIUS+.0006);p[hit,2]=CENTER[2]+d[hit,1]/np.maximum(r[hit],1e-9)*(RADIUS+.0006)
 def update(self,hands,dt=.05):
  self.time+=dt;p=self.soft.p;self.soft.grips.update(p,hands)
  # Cold wax is tacky on first contact; smoothing is measured separately.
  touching=(abs(p[:,0]-CENTER[0])<.035)&(abs(p[:,1])<.15)&(abs(p[:,2]-self.surface(p[:,0]))<.0025)
  for idx in np.flatnonzero(touching):
   if int(idx) not in self.bonds and not self.removed[idx]:self.bonds[int(idx)]=np.array([p[idx,0],p[idx,1],self.surface(p[idx,0])])
  # Only closed finger-pad contact near the leg crown can press the strip.
  for side,h in hands.items():
   if side in self.soft.grips.held:continue
   for sign in [-1,1]:
    pad=h['tcp']+h['rotation']@np.array([sign*h['opening']*.094/2,0,.01284])
    contact=(np.linalg.norm(p[:,:2]-pad[:2],axis=1)<.016)&(abs(p[:,0]-CENTER[0])<.035)&(p[:,2]>=pad[2]-.004)&(p[:,2]<self.surface(p[:,0])+.025)&(pad[2]<=self.surface(p[:,0])+.004)&(pad[2]>=self.surface(p[:,0])-.008)
    for idx in np.flatnonzero(contact):
     self.pressed[idx]=True;self.bonds[int(idx)]=np.array([p[idx,0],p[idx,1],self.surface(p[idx,0])])
  self.peak_bonds=max(self.peak_bonds,len(self.bonds))
  # Peel bonds fail from local tensile extension, not from a demo phase.
  edge=self.soft.edges;length=np.linalg.norm(p[edge[:,1]]-p[edge[:,0]],axis=1)
  strained=(length>self.soft.rest*1.12)&(self.soft.k>.5)
  detach=set()
  held=self.soft.grips.targets(hands)
  for i,goal in held.items():
   if i in self.bonds and np.linalg.norm(goal-self.bonds[i])>.006 and goal[2]>=self.bonds[i][2]-.002:detach.add(i)
  for i,j in edge[strained]:
   if i in self.bonds and j not in self.bonds:detach.add(int(i))
   if j in self.bonds and i not in self.bonds:detach.add(int(j))
  for i in detach:
   self.bonds.pop(i,None);self.removed[i]=True
  if detach:self.events.append(dict(event='peel',time=self.time,cells=len(detach)))
  def release_bonds(points,pins):
   length=np.linalg.norm(points[edge[:,1]]-points[edge[:,0]],axis=1)
   strained=(length>self.soft.rest*1.12)&(self.soft.k>.5)
   broken=set()
   for i,j in edge[strained]:
    if i in self.bonds and j not in self.bonds:broken.add(int(i))
    if j in self.bonds and i not in self.bonds:broken.add(int(j))
   for i in broken:
    self.bonds.pop(i,None);self.removed[i]=True
    if i not in held:pins.pop(i,None)
   if broken and self.pressed.mean()>=.7:
     self.events.append(dict(event='peel',time=self.time,cells=len(broken)))
  if self.pressed.mean()>=.7 and held:
   index=min(held)
   if self.peel_reference is None:self.peel_reference=(index,held[index].copy())
   i,origin=self.peel_reference
   if i in held and np.linalg.norm(held[i]-origin)>.020 and self.peel_start is None:self.peel_start=self.time
  self.soft.step(hands,dt,collide=self.collide,pins=self.bonds,substeps=8,iterations=12,callback=release_bonds)
  edge_held=any(g['index']%17>=15 for g in self.soft.grips.held.values())
  flat=(abs(self.soft.p[:,2]-self.surface(self.soft.p[:,0]))<.003)&self.pressed
  bonded=np.zeros(len(flat),bool);bonded[list(self.bonds)]=True
  if edge_held:self.applied_flat=max(self.applied_flat,float(np.mean(flat&bonded)))
  if self.peak_bonds>0 and not self.bonds and self.peel_start is not None and self.peel_end is None:self.peel_end=self.time
  if self.peel_start is not None and self.bonds:
   for s,g in self.soft.grips.held.items():
    target=hands[s]['tcp'];nearest=min(self.bonds.values(),key=lambda v:np.linalg.norm(target-v))
    delta=target-nearest
    if delta[2]>.07 and delta[2]>abs(delta[1])*np.tan(np.radians(55.)):self.peel_high=True
  # Entry into the leg form is a contact failure, not successful smoothing.
  self.failed=self.failed or self.soft.failed
  self.release=self.release+dt if not self.soft.grips.held else 0.
 def measure(self):
  clear=bool(np.min(np.linalg.norm(self.soft.p[:,[0,2]]-CENTER[[0,2]],axis=1))>RADIUS+.012 or np.all(abs(self.soft.p[:,1])>.17))
  return dict(pressed_fraction=float(self.pressed.mean()),applied_flat_fraction=self.applied_flat,adhered_cells=len(self.bonds),peeled_cells=int(self.removed.sum()),peel_seconds=None if self.peel_start is None or self.peel_end is None else self.peel_end-self.peel_start,strip_clear_of_leg=clear,released_seconds=self.release,high_angle_pull=self.peel_high,failed=self.failed)
 def score(self):
  m=self.measure()
  if m['failed'] or m['high_angle_pull'] or m['applied_flat_fraction']<.70:return 0.
  if m['adhered_cells']==0 and m['strip_clear_of_leg'] and m['released_seconds']>=2. and m['peel_seconds'] is not None:return 1. if m['peel_seconds']<=2. else .5
  return .25
 def state(self):return dict(points=self.soft.p,pressed=self.pressed,bonded=sorted(self.bonds),held={s:g['index'] for s,g in self.soft.grips.held.items()})
 def draw(self,d):
  d.mesh('wax_strip',self.soft.p,self.faces,(.92,.84,.36))
  # The dry grip margin is a coloured edge of the same flexible sheet.
  ids=[i*17+16 for i in range(5)];d.tube('dry_grip_edge',self.soft.p[ids],.0008,(.95,.95,.88),6)
