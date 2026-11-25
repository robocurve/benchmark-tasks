"""Explicit construction helpers; metres, kg, newtons, newton metres and radians for drive gains."""
import numpy as np
from scipy.spatial.transform import Rotation
TABLE=.7475

def box(size,pos=(0,0,0),color=None,quat=None):
 d=dict(kind='box',size=list(size),pos=list(pos))
 if color is not None:d['color']=list(color)
 if quat is not None:d['quat']=list(quat)
 return d

def sphere(radii,pos=(0,0,0),color=None):
 # PhysX spheres cannot retain unequal radii. Bake an ellipsoid into a convex
 # mesh so the collision surface matches what the cameras show.
 if np.ptp(radii)>1e-9:
  n,m=16,8;points=[[0,0,radii[2]]]
  for j in range(1,m):
   a=np.pi*j/m
   for i in range(n):
    t=2*np.pi*i/n;points.append([radii[0]*np.sin(a)*np.cos(t),radii[1]*np.sin(a)*np.sin(t),radii[2]*np.cos(a)])
  points.append([0,0,-radii[2]]);faces=[(0,1+i,1+(i+1)%n) for i in range(n)]
  for j in range(m-2):
   for i in range(n):
    a=1+j*n+i;b=1+j*n+(i+1)%n;faces.append((a,a+n,b+n,b))
  faces += [(len(points)-1,1+(m-2)*n+(i+1)%n,1+(m-2)*n+i) for i in range(n)]
  d=dict(kind='mesh',points=points,faces=faces,pos=list(pos))
 else:d=dict(kind='sphere',radii=list(radii),pos=list(pos))
 if color is not None:d['color']=list(color)
 return d

def cylinder(radius,length,pos=(0,0,0),quat=None,color=None):
 d=dict(kind='cylinder',radius=radius,length=length,pos=list(pos))
 if quat is not None:d['quat']=list(quat)
 if color is not None:d['color']=list(color)
 return d

def body(name,pos,shapes,mass=.02,color=(.6,.6,.6),**kwargs):return dict(name=name,pos=list(pos),shapes=shapes,mass=mass,color=list(color),**kwargs)

def joint(name,a,b,p0,p1,**kwargs):return dict(name=name,a=a,b=b,p0=list(p0),p1=list(p1),**kwargs)

def fixed_axes():return {axis:dict(limits=(1,-1)) for axis in ['transX','transY','transZ','rotX','rotY','rotZ']}

def bend_axes(stiffness=.0001,damping=.00001,limit=150):return {axis:dict(limits=(-limit,limit),stiffness=stiffness,damping=damping) for axis in ['rotX','rotY','rotZ']}

def ring(radius,height,thickness,n=24,z=0.,color=None):
 return [box((2*radius*np.tan(np.pi/n),thickness,height),(radius*np.sin(t),radius*np.cos(t),z),color,Rotation.from_euler('z',-t).as_quat()) for t in np.arange(n)*2*np.pi/n]

def rod_between(a,b,radius):
 a,b=np.asarray(a),np.asarray(b);v=b-a;length=np.linalg.norm(v);R=Rotation.align_vectors([v/length],[[0,0,1]])[0]
 return cylinder(radius,length,(a+b)/2,R.as_quat())

def finalize(bodies,joints):
 """Make rest frames agree in world space; reject an inconsistent reset."""
 lookup={b['name']:b for b in bodies}
 assert len(lookup)==len(bodies)
 for b in bodies:
  b['pos']=list(map(float,b['pos']));b['quat']=list(map(float,b.get('quat',[0,0,0,1])))
 for j in joints:
  j['name']=j['name'].replace('-','n')
  Ra=Rotation.from_quat(lookup[j['a']]['quat']);Rb=Rotation.from_quat(lookup[j['b']]['quat'])
  if 'q1' not in j:j['q1']=(Rb.inv()*Ra*Rotation.from_quat(j.get('q0',[0,0,0,1]))).as_quat().tolist()
  assert j['a']!=j['b']
