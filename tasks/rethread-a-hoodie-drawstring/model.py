"""Flexible hood casing and frictional sliding of a blunt bodkin and cord.

The enclosed path is a reduced guide constraint following the deforming casing,
not a rigid pipe. Misalignment blocks advance, fabric can gather/move, and the
trailing end can be lost. No trajectory label or score drives material motion.
"""
import numpy as np
from common import Soft,Grips,TABLE

RAD=.18
Z=TABLE+.0025
COUNT=41
LENGTH=np.pi*RAD

def arc(s):
 a=-np.pi/2+np.clip(s,0,LENGTH)/RAD
 p=np.array([.16+RAD*np.cos(a),RAD*np.sin(a),Z])
 t=np.array([-np.sin(a),np.cos(a),0.])
 return p+t*(s-np.clip(s,0,LENGTH)),t

class Model:
 def __init__(self):
  angles=np.linspace(-np.pi/2,np.pi/2,COUNT)
  p=np.array([[.16+r*np.cos(a),r*np.sin(a),Z] for r in np.linspace(RAD,RAD+.10,5) for a in angles])
  e=[];stiff=[];self.fabric_faces=[]
  for row in range(5):
   for col in range(COUNT):
    a=row*COUNT+col
    for dr,dc,k in [(0,1,1.),(1,0,1.),(1,1,.6),(1,-1,.6),(0,2,.015),(2,0,.015)]:
     if row+dr<5 and 0<=col+dc<COUNT:e.append((a,(row+dr)*COUNT+col+dc));stiff.append(k)
    if row<4 and col<COUNT-1:self.fabric_faces.append((a,a+1,a+COUNT+1,a+COUNT))
  self.casing=Soft(p,e,stiff,.0006)
  self.rest=p.copy();self.grips=Grips();self.cloth_grips=Grips()
  self.progress=.015;self.tool_axis=arc(self.progress)[1];self.tool_local={}
  # A 90 cm cord is fixed to the bodkin. Its free part remains a physical chain.
  self.cord_length=.90;self.step_length=.01
  path=np.array([arc(self.progress)[0],arc(0)[0],[.08,-.18,Z],[.08,-.28,Z],[.53,-.28,Z],[.53,-.40,Z],[.16,-.40,Z]])
  lengths=np.linalg.norm(np.diff(path,axis=0),axis=1);cum=np.r_[0,np.cumsum(lengths)]
  points=[]
  for s in np.arange(91)*.01:
   j=min(len(lengths)-1,max(0,np.searchsorted(cum,s,side='right')-1))
   points.append(path[j]+(path[j+1]-path[j])*(s-cum[j])/lengths[j])
  points=np.array(points)
  self.cord=Soft(points,[(i,i+1) for i in range(90)],None,.002)
  self.cord.rest[:]=.01
  self.events=[];self.time=0.;self.release=0.;self.failed=False
  self.alignment=0.;self.force=0.;self.max_gather=0.
 def path(self,s):
  p=self.casing.p[:COUNT];seg=np.linalg.norm(np.diff(p,axis=0),axis=1);cum=np.r_[0,np.cumsum(seg)]
  ss=float(np.clip(s,0,cum[-1]));i=min(len(seg)-1,max(0,int(np.searchsorted(cum,ss,side='right')-1)))
  t=(p[i+1]-p[i])/max(seg[i],1e-9);point=p[i]+t*(ss-cum[i])
  return point+t*(s-ss),t,float(cum[-1]),i
 def tool_point(self):return self.path(self.progress-.020)[0]
 def update(self,hands,dt=.05):
  self.time+=dt;old=set(self.grips.held)
  self.grips.update(np.array([self.tool_point()]),hands)
  for s in set(self.grips.held)-old:self.tool_local[s]=hands[s]['rotation'].T@self.tool_axis
  available={s:h for s,h in hands.items() if s not in self.grips.held}
  self.cloth_grips.update(self.casing.p,available)
  pins=self.cloth_grips.targets(available)
  goals=self.grips.targets(hands)
  point,t,total,i=self.path(self.progress-.020)
  self.force=0.
  if goals:
   side=next(iter(self.grips.held));self.tool_axis=hands[side]['rotation']@self.tool_local[side]
   self.alignment=float(np.degrees(np.arccos(np.clip(abs(self.tool_axis@t),0,1))))
   delta=goals[0]-point;self.force=float(np.clip(delta@t*100.,-5.,5.))
   # A 40 mm x 3 mm bodkin must fit the 16 mm fabric tunnel; wrong-way pushing
   # loads/moves the casing rather than travelling through its wall.
   blocked=self.alignment>25. or np.linalg.norm(delta-(delta@t)*t)>.025
   lo=max(0,i-1);hi=min(i+1,COUNT-1)
   local=np.linalg.norm(self.casing.p[hi]-self.casing.p[lo])
   gathered=max(0.,1.-local/((hi-lo)*LENGTH/(COUNT-1)));self.max_gather=max(self.max_gather,gathered)
   friction=.20+.55*gathered
   if not blocked:
    speed=np.sign(self.force)*max(0.,abs(self.force)-friction)/8.
    self.progress=float(np.clip(self.progress+np.clip(speed,-.08,.08)*dt,0.,self.cord_length-.015))
   # Equal drag on the locally contacted casing makes unsupported fabric move.
   drag=np.clip(self.force,-.5,.5)*dt*.025*t
   for j in range(max(0,i-1),min(COUNT,i+2)):
    if j not in pins:self.casing.p[j]+=drag
   if abs(self.force)>4.5 and blocked:self.failed=True
  # The whole fabric panel is free. Table friction and the other hand provide
  # the counterhold; there is no spring returning the casing to a drawn path.
  def hem_contact(points):points[:COUNT,2]=np.maximum(points[:COUNT,2],TABLE+.0025)
  self.casing.step({},dt,pins=pins,collide=hem_contact,substeps=4,iterations=8)
  route={}
  total=self.path(0)[2]
  for j in range(len(self.cord.p)):
   s=self.progress-j*self.step_length
   if 0<=s<=total:route[j]=self.path(s)[0]
  route[0]=self.path(self.progress)[0]
  self.cord.step({},dt,pins=route,substeps=8,iterations=20)
  self.failed=self.failed or self.casing.failed or self.cord.failed
  self.release=self.release+dt if not self.grips.held and not self.cloth_grips.held else 0.
 def measure(self):
  total=self.path(0)[2];out=max(0.,self.progress-total);tail=self.cord_length-self.progress
  return dict(bodkin_through=out>.040,leading_end_m=out,trailing_end_m=tail,
              casing_length_m=total,tool_alignment_deg=self.alignment,
              maximum_gather=self.max_gather,released_seconds=self.release,failed=self.failed)
 def score(self):
  m=self.measure()
  if self.failed or m['trailing_end_m']<=0.:return 0.
  if m['leading_end_m']>=.08 and m['trailing_end_m']>=.08:return 1. if self.release>=3. else .75
  return .5 if m['bodkin_through'] else 0.
 def state(self):return dict(casing=self.casing.p,cord=self.cord.p,progress=self.progress,tool_axis=self.tool_axis,tool_held=list(self.grips.held),cloth_held={s:g['index'] for s,g in self.cloth_grips.held.items()})
 def draw(self,d):
  d.mesh('hood_fabric',self.casing.p,self.fabric_faces,(.23,.37,.49))
  p=self.casing.p[:COUNT]
  # Visible seam follows the actual flexible guide, with a hollow opening.
  tangent=np.gradient(p,axis=0);tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-12)
  across=np.c_[-tangent[:,1],tangent[:,0],np.zeros(len(p))]
  phase=np.arange(10)*2*np.pi/10
  vertices=(p[:,None,:]+.008*across[:,None,:]*np.cos(phase)[None,:,None]+np.array([0,0,.0025])*np.sin(phase)[None,:,None]).reshape(-1,3)
  casing_faces=[(i*10+j,i*10+(j+1)%10,(i+1)*10+(j+1)%10,(i+1)*10+j) for i in range(len(p)-1) for j in range(10)]
  d.mesh('fabric_casing',vertices,casing_faces,(.32,.48,.61))
  d.tube('drawstring',self.cord.p,.002,(.95,.91,.73),8)
  c=self.tool_point();t=self.tool_axis
  d.tube('blunt_bodkin',[c-t*.020,c+t*.020],.0015,(.75,.77,.80),10)
  for side in [0,-1]:
   q=p[side];_,t,_,_=self.path(0 if side==0 else self.path(0)[2])
   d.tube('opening_'+('entry' if side==0 else 'exit'),[q-t*.0002,q+t*.0002],.002,(.04,.06,.07),12)
