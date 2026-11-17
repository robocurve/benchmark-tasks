"""Full-size empty passenger-seat buck, outboard pillar and locked belt routing.

Coordinates in metres. The 750 mm working belt runs from the lower seat-side
anchor to the shoulder guide. A further 550 mm runs behind it to a locked
retractor. No person or powered pretensioner is present.
"""
import numpy as np
LOCAL_CENTER=np.array([.32,0.,.7875])
WORLD_CENTER=np.array([.365,.080,.855])
v=np.array([.65,0.,.76]);v/=np.linalg.norm(v)
R=np.column_stack(([0.,-1.,0.],v,np.cross([0.,-1.,0.],v)))
BOXES=[
 ('seat_cushion',(.365,-.170,.660),(.450,.450,.100),(.20,.24,.28)),
 ('seat_back',(.625,-.170,.965),(.070,.450,.510),(.20,.24,.28)),
 ('pillar',(.640,.155,.975),(.100,.100,.650),(.38,.40,.42)),
 ('door_inner_panel',(.365,.240,.800),(.600,.030,.400),(.32,.34,.36)),
 ('seat_frame',(.365,-.170,.570),(.410,.410,.080),(.15,.16,.18)),
 ('floor_buck',(.365,-.080,.490),(.700,.700,.060),(.28,.29,.31)),
]
BOXES=[(n,(p[0],p[1],p[2]-(.20 if n=='floor_buck' else .12)),s,c) for n,p,s,c in BOXES]
def world(p):return (np.asarray(p)-LOCAL_CENTER)@R.T+WORLD_CENTER
def local(p):return (np.asarray(p)-WORLD_CENTER)@R+LOCAL_CENTER
def hands_local(hands):return {s:dict(h,tcp=local(h['tcp']),rotation=R.T@h['rotation']) for s,h in hands.items()}
def cloth_contact(p):
 w=world(p)
 for name,c,size,color in BOXES:
  c=np.asarray(c);half=np.asarray(size)/2+.0006;q=w-c
  inside=np.all(abs(q)<half,axis=1)
  ids=np.flatnonzero(inside)
  if len(ids):
   axis=np.argmin(half-abs(q[ids]),axis=1)
   w[ids,axis]=c[axis]+np.where(q[ids,axis]>=0,1.,-1.)*half[axis]
 p[:]=local(w)
class Drawing:
 def __init__(self,d):self.d=d
 def mesh(self,n,p,f,c):self.d.mesh(n,world(p),f,c)
 def tube(self,n,p,r,c,sides=8):self.d.tube(n,world(p),r,c,sides)
 def box(self,n,p,size,c,orientation=None):self.d.box(n,world(p),size,c,R if orientation is None else R@orientation)
