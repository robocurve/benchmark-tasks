"""Adhesive-face footprint on measured oriented carton faces, no path painting."""
from math import sqrt,isfinite
from model import TAPE_WIDTH,TAPE_THICKNESS
PATCHES=12

def dot(a,b):return sum(x*y for x,y in zip(a,b))
def sub(a,b):return tuple(x-y for x,y in zip(a,b))
def add(a,b):return tuple(x+y for x,y in zip(a,b))
def mul(a,s):return tuple(x*s for x in a)
def column(r,i):return tuple(v[i] for v in r)
def local(r,v):return tuple(dot(column(r,i),v) for i in range(3))
def width_contacts(film,target):
 """Return actual width-bin centres within 0.5 mm and facing a box surface.

 film/target contain native root poses and rotations; target size is authored
 collider size. Call only with actual filtered native contact force separately.
 """
 normal=column(film['rotation'],1);width=column(film['rotation'],2)
 center=add(film['pos'],mul(normal,TAPE_THICKNESS/2))
 hits=[]
 for k in range(PATCHES):
  p=add(center,mul(width,(-.5+(k+.5)/PATCHES)*TAPE_WIDTH));v=local(target['rotation'],sub(p,target['pos']));half=[s/2 for s in target['size']]
  choices=[]
  for axis in range(3):
   gap=abs(v[axis])-half[axis]
   if -.0002<=gap<=.0005 and all(abs(v[j])<=half[j]+.0001 for j in range(3) if j!=axis):
    outward=mul(column(target['rotation'],axis),1. if v[axis]>=0 else -1.)
    if -dot(normal,outward)>=.9:choices.append((axis,outward))
  if choices:hits.append((k,p,choices[0][1]))
 return hits
