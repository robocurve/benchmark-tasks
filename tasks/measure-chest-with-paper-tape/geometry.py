"""Adult torso form and a loose, graduated paper tape; SI units throughout."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
CENTER=np.array([.18,0.,TABLE+.145]);CHEST=CENTER[2]
RX=.120;RY=.160;N=72;DX=.015;WIDTH=.020;THICK=.00020
# Euler-Bernoulli ribbon estimate, E=1.8 GPa and nu=.3; not a stock-specific calibration.
E=1.8e9;G=E/(2*1.3)
BEND=E*WIDTH*THICK**3/(12*DX)
IN_PLANE=E*THICK*WIDTH**3/(12*DX)
TWIST=G*WIDTH*THICK**3/(3*DX)
# Reusable adult torso form: waist, ribcage, shoulders and neck. A standard
# dress-form stand supports it; no feature captures or guides the tape.
levels=[(-.140,.100,.125),(-.090,.105,.133),(-.040,.119,.158),(0.,RX,RY),(.045,.118,.160),(.085,.099,.185),(.110,.075,.165),(.140,.050,.060),(.200,.048,.055)]
points=[];faces=[];segments=64
for z,rx,ry in levels:
 for t in np.arange(segments)*2*np.pi/segments:points.append([rx*np.cos(t),ry*np.sin(t),z])
for j in range(len(levels)-1):
 for i in range(segments):
  k=j*segments+i;l=j*segments+(i+1)%segments;faces.append((k,l,l+segments,k+segments))
faces += [tuple(range(segments-1,-1,-1)),tuple(range((len(levels)-1)*segments,len(points)))]
# Separate convex bands preserve the torso's neck and waist rather than filling
# them with a single convex hull across the whole figure.
shapes=[]
for j in range(len(levels)-1):
 p=points[j*segments:(j+2)*segments]
 f=[tuple(range(segments-1,-1,-1)),tuple(range(segments,2*segments))]+[(i,(i+1)%segments,(i+1)%segments+segments,i+segments) for i in range(segments)]
 shapes.append(dict(kind='mesh',points=p,faces=f))
BODIES.append(body('torso',CENTER,shapes,mass=3.0,fixed=True,color=(.65,.72,.76),friction=.35))
BODIES.append(body('base',[.18,0,TABLE+.008],[cylinder(.105,.016)],mass=2.,fixed=True,color=(.22,.24,.27)))
# A loose paper tape rests on its lower long edge in a shallow arc behind
# the form. Its curved footprint supports the 20 mm high strip without a jig.
VERTICES=[np.array([.14,.50,TABLE+WIDTH/2])]
for i in range(N):
 sx=.32*np.clip((i-30)/30,-1,1)
 VERTICES.append(VERTICES[-1]+DX*np.array([sx,-np.sqrt(1-sx*sx),0.]))
VERTICES=np.array(VERTICES)
CELLS=[]
for i in range(N):
 name=f'tape_{i:03d}';CELLS.append(name);y=(VERTICES[i+1]-VERTICES[i])/DX;x=np.array([0.,0.,1.]);z=np.cross(x,y);R=np.column_stack([x,y,z])
 BODIES.append(body(name,(VERTICES[i]+VERTICES[i+1])/2,[box((WIDTH,DX-.00002,THICK))],quat=Rotation.from_matrix(R).as_quat(),mass=WIDTH*DX*.16,color=(.98,.95,.80),friction=.5,contact_offset=.00012,speculative_ccd=True,linear_damping=.15,angular_damping=.25))
 if i:
  axes={'rotX':dict(limits=(-150,150),stiffness=BEND,damping=.00002),'rotY':dict(limits=(-60,60),stiffness=TWIST,damping=.00002),'rotZ':dict(limits=(-5,5),stiffness=IN_PLANE,damping=.002)}
  JOINTS.append(joint(f'paper_{i}',CELLS[-2],name,[0,DX/2,0],[0,-DX/2,0],axes=axes,break_force=4.,break_torque=.06))
finalize(BODIES,JOINTS)
