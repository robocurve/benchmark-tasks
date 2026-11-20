"""Approximate opaque finger occlusion using oriented hardware boxes.

The light travels vertically from the compact lamp roof. Precision fingers use
exact dimensions from pad_covers; the original palm is a conservative box.
"""
import numpy as np

def intersects_segment(a,b,centre,size):
 a=np.asarray(a)-centre;d=np.asarray(b)-np.asarray(centre)-a;lo=0.;hi=1.
 for i in range(3):
  h=size[i]/2
  if abs(d[i])<1e-12:
   if abs(a[i])>h:return False
  else:
   t=sorted([(-h-a[i])/d[i],(h-a[i])/d[i]]);lo=max(lo,t[0]);hi=min(hi,t[1])
   if lo>hi:return False
 return True

def shadowed(point,hands,roof_z):
 start=np.asarray(point)+[0,0,.0003];end=np.array([*start[:2],roof_z])
 if end[2]<=start[2]:return False
 for h in hands.values():
  R=np.asarray(h['rotation']);tcp=np.asarray(h['tcp']);a=R.T@(start-tcp);b=R.T@(end-tcp)
  q=np.asarray(h.get('finger_joint_positions',[-h['opening']*.04695]*2))
  boxes=[([0,0,-.084],(.070,.065,.040))]
  for opening,sign in [(q[0],1),(-q[1],-1)]:
   boxes.extend([([opening+sign*.002,0,-.003],(.002,.004,.012)),([opening+sign*.0015,0,-.0255],(.003,.004,.033))])
  if any(intersects_segment(a,b,np.asarray(c),s) for c,s in boxes):return True
 return False
