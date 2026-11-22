"""Read native tape state. Scoring cannot move or attach the measuring tape."""
import numpy as np
from native_model import NativeModel
from geometry import *
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.grasp_state={};self.peak_gap=0.;self.hold=0.;self.best=0.
 def configure_station(self):
  from pad_covers import install
  install()
 def bind_native(self,base):
  super().bind_native(base)
  from finite_pinch import FinitePinch
  self.pinch=FinitePinch(self,CELLS,THICK/2,.001,.005,(-.036,.004),force_limit=1.2,torque_limit=.012)
 def update(self,hands,dt=.05):
  self.read();self.time+=dt;self.pinch.update(hands)
  self.peak_gap=max(self.peak_gap,max(np.linalg.norm(self.joint_gap(j['name'])) for j in JOINTS))
  self.failed |= self.integrity_failed()
  q=self.measure();self.hold=self.hold+dt if q['readable_loop'] else 0.
 def integrity_failed(self):return super().integrity_failed() or bool(self.broken) or self.peak_gap>.003
 def reset_valid(self):return not self.failed and self.measure()['angular_coverage_deg']<180
 def measure(self):
  p=np.array([self.p(n) for n in CELLS]);r=p-CENTER
  theta=np.unwrap(np.arctan2(r[:,1]/RY,r[:,0]/RX))
  # Find the scale point adjacent to the actual zero end. Only one continuous
  # traversal of the torso is eligible; a doubled-back strip cannot count.
  zero=self.p(CELLS[0],[0,-DX/2,0]);best=None
  for i in range(35,N-1):
   a=self.p(CELLS[i],[0,-DX/2,0]);b=self.p(CELLS[i],[0,DX/2,0]);v=b-a
   u=float(np.clip(np.dot(zero-a,v)/np.dot(v,v),0,1));pt=a+u*v;dist=float(np.linalg.norm(pt-zero))
   if best is None or dist<best[0]:best=(dist,i,u,pt)
  dist,i,u,pt=best;active=p[:i+1];rr=active-CENTER
  sweep=abs(float(theta[i]-theta[0]));coverage=min(360.,sweep*180/np.pi)
  monotone=bool(np.sum(abs(np.diff(theta[:i+1])))-sweep<.20)
  height=float(np.max(abs(active[:,2]-CHEST)))
  # Signed shortest distance to the actual 64-sided chest cross-section.
  # This matches a ruler/gap gauge; normalized ellipse radius overestimates
  # front/back gaps because the two radii differ.
  t=np.arange(64)*2*np.pi/64
  boundary=np.c_[RX*np.cos(t),RY*np.sin(t)]
  edges=np.roll(boundary,-1,axis=0)-boundary
  delta=rr[:,None,:2]-boundary[None,:,:]
  projection=np.clip(np.sum(delta*edges,axis=2)/np.sum(edges*edges,axis=1),0.,1.)
  distances=np.linalg.norm(delta-projection[:,:,None]*edges,axis=2).min(axis=1)
  inside=np.all(edges[None,:,0]*delta[:,:,1]-edges[None,:,1]*delta[:,:,0]>=0.,axis=1)
  signed=np.where(inside,-distances,distances)
  gap=float(np.max(signed));inward=float(np.min(signed))
  upright=[abs(float(self.orientation(n)[2,0])) for n in CELLS[:i+1]]
  flat=bool(min(upright)>.90)
  reading=(i+u)*DX*100
  perimeter=sum(np.linalg.norm(np.array([RX*np.cos(t+2*np.pi/64),RY*np.sin(t+2*np.pi/64)])-np.array([RX*np.cos(t),RY*np.sin(t)])) for t in np.arange(64)*2*np.pi/64)*100
  reading_error=abs(reading-perimeter)
  front=bool(zero[0]>CENTER[0]+RX*.75 and abs(zero[1])<.07)
  loop=bool(coverage>=345 and monotone and dist<.012 and height<=.015 and gap<=.010 and inward>=-.003 and flat and front and reading_error<=2.)
  return dict(angular_coverage_deg=coverage,zero_scale_gap_mm=dist*1000,circumference_reading_cm=reading,reading_error_cm=reading_error,monotone=monotone,maximum_level_error_mm=height*1000,maximum_gap_mm=gap*1000,minimum_surface_offset_mm=inward*1000,flat=flat,front_overlap=front,readable_loop=loop,hold_seconds=self.hold,failed=self.failed,maximum_joint_gap_mm=self.peak_gap*1000)
 def score(self):
  q=self.measure()
  if self.failed:return 0.
  if q['readable_loop'] and self.hold>=3.:return 1.
  if q['monotone'] and q['minimum_surface_offset_mm']>=-3 and q['angular_coverage_deg']>=300 and q['maximum_level_error_mm']<=30 and q['flat'] and q['maximum_gap_mm']<=30:return .5
  if q['monotone'] and q['minimum_surface_offset_mm']>=-3 and q['angular_coverage_deg']>=180 and q['maximum_level_error_mm']<=30 and q['flat'] and q['maximum_gap_mm']<=30:return .25
  return 0.
 def state(self):return dict(super().state(),grasp_state=self.grasp_state,peak_gap=self.peak_gap,hold=self.hold,best=self.best)
 def draw(self,d):
  # Printed ticks and digits follow the observed physical paper; visual only.
  for i,n in enumerate(CELLS):
   for cm in range(int(np.ceil(i*DX*100)),int(np.ceil((i+1)*DX*100))):
    local_y=cm/100-(i+.5)*DX;R=self.orientation(n)
    for face in [-1,1]:
     center=self.p(n,[0,local_y,face*(THICK/2+.00003)])
     d.box(f'tick_{cm}_{face+1}',center,(WIDTH*.35,.00045,.00004),(.12,.10,.08),R)
     if cm%5==0:
      from printed_scale import digits
      for k,(cx,cy,wx,wy) in enumerate(digits(str(cm))):
       pos=self.p(n,[face*cx,local_y+cy,face*(THICK/2+.00006)])
       d.box(f'number_{cm}_{face+1}_{k}',pos,(wx,wy,.00004),(.05,.05,.05),R)
  # Discrete shoulder/chest anatomy markers, not a circumferential tape guide.
  for side in [-1,1]:d.box('landmark_'+str(side+1),CENTER+[RX-.002,side*.055,0],(.004,.008,.008),(.27,.36,.40))
