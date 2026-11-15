"""Flexible bracelet with a small push-in pin-lock jewellery clasp.

The strap is a freely bending inextensible chain with wrist contact. The clasp
uses geometric pin/socket clearance and a spring-catch state after 4.5 mm of
actual insertion. The catch is simplified, not a detailed metal spring mesh.
"""
import numpy as np
from common import Soft,Grips,TABLE,grip_candidates
WRIST=np.array([.34,0.,.80]);RADIUS=.027
class Model:
 def __init__(self):
  p=np.c_[np.full(43,.34),np.linspace(.105,-.105,43),np.full(43,TABLE+.0035)]
  edges=[(i,i+1) for i in range(len(p)-1)]+[(i,i+2) for i in range(len(p)-2)];k=[1.]*(len(p)-1)+[.02]*(len(p)-2)
  self.soft=Soft(p,edges,k,.001);self.grips=Grips();self.events=self.grips.events;self.clasp=np.array([p[0],p[-1]])
  self.axis=np.tile([0.,1.,0.],(2,1));self.held_R={};self.latched=False;self.failed=False;self.entered=0.;self.time=0.;self.release=0.
 def collide(self,p):
  p[[0,-1],2]=np.maximum(p[[0,-1],2],TABLE+.0035)
  d=p[:,1:]-WRIST[1:];r=np.linalg.norm(d,axis=1);hit=(r<RADIUS+.001)&(abs(p[:,0]-.34)<.11)
  p[hit,1:]=WRIST[1:]+d[hit]/np.maximum(r[hit,None],1e-9)*(RADIUS+.001)
 def update(self,hands,dt=.05):
  self.time+=dt;self.clasp=np.array([self.soft.p[0],self.soft.p[-1]])
  old=set(self.grips.held);grasp_points=self.clasp+self.axis*np.array([.004,-.004])[:,None];self.grips.update(grasp_points,hands)
  for s in set(self.grips.held)-old:
   g=self.grips.held[s];self.held_R[s]=hands[s]['rotation'].T@self.axis[g['index']]
  goals=self.grips.targets(hands);pins={}
  for side,g in self.grips.held.items():self.axis[g['index']]=hands[side]['rotation']@self.held_R[side]
  goals={i:point-self.axis[i]*([.004,-.004][i]) for i,point in goals.items()}
  for i,target in goals.items():pins[0 if i==0 else len(self.soft.p)-1]=target
  female=goals.get(0,self.clasp[0]);male=goals.get(1,self.clasp[1]);axis=self.axis[0];tip=male+self.axis[1]*.011;entry=female-axis*.005
  relative=tip-entry;depth=float(relative@axis);radial=float(np.linalg.norm(relative-depth*axis));angle=float(np.arccos(np.clip(self.axis[0]@self.axis[1],-1,1)))
  self.entered=depth if radial<=.0010 and angle<=np.radians(12) else 0.
  if not self.latched and self.entered>=.0045 and self.entered<=.008:
   self.latched=True;self.soft.edges=np.vstack([self.soft.edges,[0,len(self.soft.p)-1]]);self.soft.rest=np.r_[self.soft.rest,np.linalg.norm(female-male)];self.soft.k=np.r_[self.soft.k,1.];self.events.append(dict(event='pin_lock_caught',time=self.time,depth_m=depth))
  if self.latched and 0 in goals and 1 in goals and np.linalg.norm(female-male)>.035:self.failed=True
  # The actual rigid housings cannot pass through each other on a missed entry.
  if not self.latched and depth>0 and self.entered==0 and 0 in goals and 1 in goals and np.linalg.norm(female-male)<.012:
   direction=(male-female)/max(np.linalg.norm(male-female),1e-9);pins[len(self.soft.p)-1]=female+direction*.012
  # Model grasps act at the clasp bodies, not at arbitrary hidden strap points.
  self.soft.step({},dt,collide=self.collide,pins=pins,substeps=8,iterations=16)
  self.clasp=np.array([self.soft.p[0],self.soft.p[-1]])
  if self.latched:
   d=self.clasp[0]-self.clasp[1];self.axis[:]=d/max(np.linalg.norm(d),1e-9)
  self.failed=self.failed or self.soft.failed
  self.release=self.release+dt if not self.grips.held else 0.
 def measure(self):
  # Winding around the wrist in the YZ cross-section, with the clasp as closure.
  p=self.soft.p[:,1:]-WRIST[1:];angles=np.unwrap(np.arctan2(p[:,1],p[:,0]));closed=np.r_[angles,np.arctan2(p[0,1],p[0,0])];turn=float(abs(np.sum((np.diff(closed)+np.pi)%(2*np.pi)-np.pi))/(2*np.pi))
  return dict(clasp_closed=self.latched,inserted_mm=max(0.,self.entered)*1000,around_wrist=bool(turn>.8),axial_offset_mm=float(np.max(abs(self.soft.p[:,0]-.34))*1000),released_seconds=self.release,failed=self.failed)
 def score(self):
  m=self.measure()
  if m['failed'] or not m['around_wrist'] or m['axial_offset_mm']>50:return 0.
  if self.latched:return 1. if self.release>=3. else .5
  return .25 if self.entered>=.001 else 0.
 def state(self):return dict(points=self.soft.p,clasp=self.clasp,axes=self.axis,latched=self.latched,held={s:g['index'] for s,g in self.grips.held.items()})
 def draw(self,d):
  p=self.soft.p;pts=np.stack([p+[-.005,0,0],p+[.005,0,0]],axis=1).reshape(-1,3);faces=[(2*i,2*i+1,2*i+3,2*i+2) for i in range(len(p)-1)];d.mesh('leather_band',pts,faces,(.16,.28,.40))
  f,m=self.clasp;a,b=self.axis
  d.tube('female_housing',[f-a*.005,f+a*.005],.0035,(.72,.72,.75),16)
  d.tube('socket_dark_opening',[f-a*.0051,f-a*.0052],.0015,(.04,.04,.05),16)
  d.tube('male_housing',[m-b*.005,m+b*.005],.0035,(.72,.72,.75),16)
  d.tube('male_pin',[m+b*.005,m+b*.011],.0011,(.80,.80,.83),12)
