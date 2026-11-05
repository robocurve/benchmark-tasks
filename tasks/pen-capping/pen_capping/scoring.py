"""Physical insertion history and released, resting color-correct assemblies."""
import math
import numpy as np
SEATED_X=-.0821
class Insertion:
 def __init__(self):self.approach=False;self.count=0
 def update(self,relative,alignment):
  x,y,z=relative;aligned=alignment>math.cos(math.radians(5));centered=math.hypot(y,z)<.0015
  if x<-.115 and math.hypot(y,z)<.003 and alignment>math.cos(math.radians(10)):self.approach=True
  if self.approach and aligned and centered and abs(x-SEATED_X)<.0015:self.count+=1
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
