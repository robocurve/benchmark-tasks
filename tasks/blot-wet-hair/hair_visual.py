"""Fine fibres drawn from native lock poses, with no independent animation.

The collision model groups fibres into twelve flexible locks. These meshes only
make those same locks legible as hair; the contact envelopes remain unchanged.
"""
import numpy as np
from geometry import BODIES,STRANDS,SEGMENTS,HEAD

LENGTHS={b['name']:b['shapes'][0]['length'] for b in BODIES if b['name'].startswith('hair_')}
FIBRES=72;SIDES=4
angles=np.arange(FIBRES)*np.pi*(3-np.sqrt(5))
radii=.0023*np.sqrt((np.arange(FIBRES)+.5)/FIBRES)
offsets=np.c_[radii*np.cos(angles),radii*np.sin(angles)]
ring=np.arange(SIDES)*2*np.pi/SIDES

def draw(model,drawing):
 points=[];faces=[]
 for s in range(STRANDS):
  starts=[0]+[i for i in range(1,SEGMENTS) if f'hair_join_{s}_{i}' in model.broken]
  for first,last in zip(starts,starts[1:]+[SEGMENTS]):
   names=[f'hair_{s}_{i}' for i in range(first,last)]
   p=np.array([model.p(names[0],[0,0,-LENGTHS[names[0]]/2])]+[model.p(n,[0,0,LENGTHS[n]/2]) for n in names])
   tangent=np.gradient(p,axis=0);tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-12)
   ref=np.tile([1.,0.,0.],(len(p),1));ref[np.abs(tangent[:,0])>.9]=[0,1,0]
   u=np.cross(tangent,ref);u/=np.maximum(np.linalg.norm(u,axis=1)[:,None],1e-12);v=np.cross(tangent,u)
   for a,b in offsets:
    centre=p+a*u+b*v
    verts=(centre[:,None,:]+.00016*(u[:,None,:]*np.cos(ring)[None,:,None]+v[:,None,:]*np.sin(ring)[None,:,None])).reshape(-1,3)
    start=len(points);points.extend(verts)
    faces.extend((start+i*SIDES+j,start+i*SIDES+(j+1)%SIDES,start+(i+1)*SIDES+(j+1)%SIDES,start+(i+1)*SIDES+j) for i in range(len(p)-1) for j in range(SIDES))
 drawing.mesh('hair_fibres',points,faces,(.16,.071,.028))
 # A visual scalp layer sits on the head, above the independently moving roots.
 # It introduces no collision surface or additional hair constraint.
 if 'hair_scalp' not in drawing.meshes:
  n,m=32,14;verts=[]
  for j in range(m+1):
   polar=.03+1.55*j/m
   for i in range(n+1):
    azimuth=np.pi/2+np.pi*i/n
    verts.append(HEAD+np.array([.091*np.sin(polar)*np.cos(azimuth),.079*np.sin(polar)*np.sin(azimuth),.116*np.cos(polar)]))
  faces=[(j*(n+1)+i,j*(n+1)+i+1,(j+1)*(n+1)+i+1,(j+1)*(n+1)+i) for j in range(m) for i in range(n)]
  drawing.mesh('hair_scalp',verts,faces,(.16,.071,.028))
