"""Whole fish with compliant skin sectors and retained overlapping scale groups."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
C=np.array([.32,.015,TABLE+.027]);RX=.035;RY=.085;RZ=.018
BODIES.append(body('board',[.38,0,TABLE+.004],[box((.25,.30,.008))],mass=.8,fixed=True,color=(.73,.62,.45),friction=.6))
head=sphere((.026,.031,.017),(0,-.084,0),(.43,.48,.39));bodyshape=sphere((.034,.081,.016))
fish=[bodyshape,head,cylinder(.008,.020,(0,.084,0),Rotation.from_euler('x',90,degrees=True).as_quat())]
# Rounded head, visible eye on each side, mouth, gill covers and tail lobes.
for sign in [-1,1]:
 fish.append(sphere((.004,.004,.0015),(.014,-.091,sign*.013),(.035,.035,.025)))
 fish.append(sphere((.0015,.0015,.0006),(.014,-.092,sign*.0145),(.7,.75,.6)))
fin=dict(kind='mesh',points=[[-.008,.106,0],[-.030,.145,0],[0,.137,0],[.030,.145,0],[.008,.106,0],[-.008,.106,.001],[-.030,.145,.001],[0,.137,.001],[.030,.145,.001],[.008,.106,.001]],faces=[(0,1,2,3,4),(5,9,8,7,6),(0,5,6,1),(1,6,7,2),(2,7,8,3),(3,8,9,4),(4,9,5,0)],pos=[0,0,0],color=[.31,.36,.29])
fish.append(fin)
fish.append(sphere((.008,.014,.007),(0,.103,0)))
BODIES.append(body('fish',C,fish,mass=.235,color=(.48,.53,.42),friction=.35,contact_offset=.0001))
TAIL=C+[0,.103,0]
PATCHES=[];PATCH_POINTS=[];PATCH_ROTATIONS=[]
def surface(x,y):
 z=RZ*np.sqrt(max(.01,1-(x/RX)**2-(y/RY)**2));normal=np.array([x/RX**2,y/RY**2,z/RZ**2]);normal/=np.linalg.norm(normal)
 ex=np.array([1.,0,0])-normal*normal[0];ex/=np.linalg.norm(ex);ey=np.cross(normal,ex)
 return np.array([x,y,z]),np.column_stack([ex,ey,normal])
for x in [-.022,-.011,0,.011,.022]:
 for y in [-.060,-.036,-.012,.012,.036,.060]:
  if (x/RX)**2+(y/RY)**2>.85:continue
  i=len(PATCHES);q,R=surface(x,y);PATCHES.append(i);PATCH_POINTS.append(q);PATCH_ROTATIONS.append(R)
  skin=f'skin_{i}';scale=f'scales_{i}';quat=Rotation.from_matrix(R).as_quat()
  BODIES.append(body(skin,C+q,[box((.0105,.023,.0004))],quat=quat,mass=.0005,color=(.50,.54,.44),friction=.4,contact_offset=.00005,exclude=['fish']))
  axes=fixed_axes();axes['transZ']=dict(limits=(-.003,.001),stiffness=700.,damping=.25)
  # Soft tissue also yields in shear and bending; a rigidly oriented facet
  # generates artificial edge impulses on the curved flank.
  for axis in ['transX','transY']:axes[axis]=dict(limits=(-.002,.002),stiffness=1000.,damping=.5)
  for axis in ['rotX','rotY','rotZ']:axes[axis]=dict(limits=(-15.,15.),stiffness=.02,damping=.001)
  JOINTS.append(joint('skin_bond_'+str(i),'fish',skin,q,[0,0,0],q0=quat,axes=axes,break_force=5.,break_torque=.04))
  # A small group represents overlapping real scales; no skinned patch is removed.
  shapes=[]
  for sx in [-.0035,0,.0035]:
   for sy in [-.008,-.003,.002,.007]:shapes.append(sphere((.0025,.0036,.00012),(sx,sy,.00035),(.70+.03*(i%3),.74,.64)))
  BODIES.append(body(scale,C+q,shapes,quat=quat,mass=.00007,color=(.72,.75,.65),friction=.75,contact_offset=.00005,exclude=['fish']))
  JOINTS.append(joint('scale_retention_'+str(i),skin,scale,[0,0,0],[0,0,0],axes=fixed_axes(),break_force=10.))
TOOL=np.array([.18,-.17,TABLE+.035]);HEAD_OFFSET=np.array([0,.060,-.030]);TOOTH_Z=-.035
shapes=[box((.012,.160,.010),(0,-.042,0)),box((.012,.010,.030),(0,.038,-.015)),box((.024,.028,.006),(0,.060,-.030))]
for x in [-.008,0.,.008]:
 for y in [.052,.060,.068]:shapes.append(sphere((.0011,.0015,.001),(x,y,-.034)))
BODIES.append(body('scaler',TOOL,shapes,mass=.087,color=(.62,.66,.68),friction=.8,contact_offset=.0001))
finalize(BODIES,JOINTS)
