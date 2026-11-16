"""Conserved quasi-static free-surface liquid in a moving spoon.

Local bowl shape is an elliptical paraboloid. The world free surface responds
to gravity and measured container acceleration. Overflow crosses the lowest
rim at a weir rate; released volume travels ballistically. This is a reduced
liquid model, not rigid beads and not an implementation of swallowing.
"""
import numpy as np

A=.024; B=.016; DEPTH=.0095

class Liquid:
 def __init__(self,volume_ml=5.):
  self.dx=.001
  x,y=np.meshgrid(np.arange(-A,A+self.dx/2,self.dx),np.arange(-B,B+self.dx/2,self.dx),indexing='ij')
  self.mask=(x/A)**2+(y/B)**2<=1
  self.xy=np.c_[x[self.mask],y[self.mask]]
  self.bottom=-DEPTH*(1-(self.xy[:,0]/A)**2-(self.xy[:,1]/B)**2)
  self.volume=volume_ml*1e-6;self.initial=self.volume;self.delivered=0.;self.spilled=0.
  self.parcels=[];self.pool=[];self.old=None;self.velocity=np.zeros(3);self.accel=np.zeros(3)
  lo=float(self.bottom.min());hi=.02
  for _ in range(38):
   level=(lo+hi)/2
   if np.maximum(0.,level-self.bottom).sum()*self.dx**2>self.volume:hi=level
   else:lo=level
  self.local_surface=np.c_[self.xy,np.maximum(self.bottom,(lo+hi)/2)];self.max_balance_error=0.
 def advance(self,center,R,dt,receiver):
  c=np.asarray(center);R=np.asarray(R)
  velocity=np.zeros(3) if self.old is None else (c-self.old)/dt
  self.accel=.8*self.accel+.2*np.clip((velocity-self.velocity)/dt,-30,30)
  self.old=c.copy();self.velocity=velocity
  gravity=np.array([0.,0.,-9.81])-self.accel
  g=R.T@gravity
  # Remaining liquid drains if the spoon is inverted; otherwise solve the
  # hydrostatic plane with exact volume on this finite area grid.
  slope=-g[:2]/min(-.3,g[2]);base=self.xy@slope
  lo=float(np.min(self.bottom-base));hi=float(np.max(self.bottom-base)+.06)
  for _ in range(38):
   level=(lo+hi)/2;v=np.maximum(0.,base+level-self.bottom).sum()*self.dx**2
   if v>self.volume:hi=level
   else:lo=level
  level=(lo+hi)/2;heights=np.maximum(self.bottom,base+level)
  angles=np.arange(128)*2*np.pi/128;rim=np.c_[A*np.cos(angles),B*np.sin(angles),np.zeros(128)]
  head=rim[:,:2]@slope+level
  j=int(np.argmax(head));depth=max(0.,float(head[j]))
  q=.009*(2/3)*np.sqrt(2*np.linalg.norm(gravity))*depth**1.5
  if g[2]>=0:q=max(q,self.volume/.15)
  emitted=min(self.volume,q*dt);self.volume-=emitted
  if emitted>0.:
   local=rim[j];p=c+R@local
   outward=R@np.r_[local[:2]/max(1e-9,np.linalg.norm(local[:2])),0.]
   v=velocity+outward*np.sqrt(2*9.81*depth)*.35
   self.parcels.append([p.copy(),v.copy(),emitted])
  self.local_surface=np.c_[self.xy,heights]
  active=[]
  for p,v,vol in self.parcels:
   before=p.copy();v+=np.array([0,0,-9.81])*dt;p+=v*dt
   state=receiver(before,p)
   if state=='delivered':self.delivered+=vol
   elif state=='spill':self.spilled+=vol;self.pool.append([p.copy(),vol])
   else:active.append([p,v,vol])
  self.parcels=active
  total=self.volume+self.delivered+self.spilled+sum(p[2] for p in self.parcels)
  self.max_balance_error=max(self.max_balance_error,abs(total-self.initial))
 def measure(self):return dict(in_spoon_ml=self.volume*1e6,delivered_ml=self.delivered*1e6,spilled_ml=self.spilled*1e6,airborne_ml=sum(p[2] for p in self.parcels)*1e6,balance_error_ml=self.max_balance_error*1e6)
