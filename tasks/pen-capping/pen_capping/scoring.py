"""Physical insertion history and released, resting color-correct assemblies."""
import math
import numpy as np
from pen_capping.retention import SEATED_X, engagement
class Insertion:
 def __init__(self):self.approach=False;self.count=0;self.metrics={}
 def update(self,relative,alignment,cap_rotation=None):
  x,y,z=relative
  if x<-.115 and math.hypot(y,z)<.003 and alignment>math.cos(math.radians(10)):self.approach=True
  # Full orientation is required in the simulator; the scalar form supports
  # coaxial callers/tests and conservatively represents tilt in one plane.
  if cap_rotation is None:
   angle=math.acos(float(np.clip(alignment,-1,1)));c,s=math.cos(angle),math.sin(angle)
   cap_rotation=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
  self.metrics=engagement(relative,cap_rotation)
  if self.approach and self.metrics['eligible']:self.count+=1
  else:self.count=0
  return self.count>=4
class History:
 def __init__(self,colors):self.colors=tuple(colors);self.retained=[0]*len(colors);self.resting=[0]*len(colors);self.capped=[False]*len(colors);self.placed=[False]*len(colors)
 def update(self,pairs,positions,axes,speeds,clear,top,destinations):
  for i,color in enumerate(self.colors):
   good=pairs[i]==color
   if good and clear[i]:self.retained[i]+=1
   else:self.retained[i]=0
   if not good:self.capped[i]=False
   if self.retained[i]>=20:self.capped[i]=True
   x,y,z=positions[i];cx,cy,sx,sy=destinations[color]
   flat=abs(axes[i][2])<math.sin(math.radians(8))
   inside=True
   for k,(center,size) in enumerate(((cx,sx),(cy,sy))):
    a=float(axes[i][k]);radial=math.sqrt(max(0.,1-a*a))
    low=min(min(-.0924*a,.0728*a)-.00972*radial,SEATED_X*a-.0175*abs(a)-.0164*radial)
    high=max(max(-.0924*a,.0728*a)+.00972*radial,SEATED_X*a+.0175*abs(a)+.0164*radial)
    inside &= positions[i][k]+low>center-size/2+.001 and positions[i][k]+high<center+size/2-.001
   rest=good and clear[i] and inside and flat and top+.006<z<top+.025 and speeds[i]<.015
   self.resting[i]=self.resting[i]+1 if rest else 0
   self.placed[i]=self.capped[i] and self.resting[i]>=20
 def score(self):return .125*(sum(self.capped)+sum(self.placed))


def hands_clear(tcps,position,axis,cap_position=None):
    """Distance to the entire pen, so holding its tail cannot fake release."""
    position=np.asarray(position);axis=np.asarray(axis)
    for tcp in tcps:
        tcp=np.asarray(tcp);along=float(np.clip((tcp-position)@axis,-.100,.073))
        if np.linalg.norm(tcp-(position+along*axis))<=.060:return False
        if cap_position is not None and np.linalg.norm(tcp-np.asarray(cap_position))<=.060:return False
    return True


class RestMotion:
 """Measure resolved rigid-body motion over a control interval.

 A last-substep solver velocity can spike while a supported body's pose is
 stationary. Full orientation differences still reject spin about the pen axis.
 """
 def __init__(self):self.previous={}
 def speed(self,name,position,rotation,dt):
  position=np.asarray(position,dtype=float);rotation=np.asarray(rotation,dtype=float)
  previous=self.previous.get(name);self.previous[name]=(position.copy(),rotation.copy())
  if previous is None or dt<=0:return math.inf
  old_position,old_rotation=previous;delta=rotation@old_rotation.T
  skew=np.array([delta[2,1]-delta[1,2],delta[0,2]-delta[2,0],delta[1,0]-delta[0,1]])*.5
  angle=math.atan2(float(np.linalg.norm(skew)),float(np.clip((np.trace(delta)-1)*.5,-1,1)))
  return float((np.linalg.norm(position-old_position)+.06*angle)/dt)
