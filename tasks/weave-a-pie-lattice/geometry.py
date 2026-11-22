"""Three flexible pastry strips; loose ordinary pie pan and preparation board."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
PIE=np.array([.35,.025,TABLE+.024]);Z=PIE[2]+.018
N=24;LENGTH=.250;DX=LENGTH/N;WIDTH=.025;THICK=.002
LONG_X=[-.032,.032];CROSS_Y=[-.050]
# Real-size 180 mm filled tart, 190 mm rim. Pan, crust and level filling move together.
shapes=[cylinder(.090,.006,(0,0,-.018)),*ring(.092,.036,.004,n=32)]
shapes.append(cylinder(.087,.036,(0,0,0),color=(.50,.27,.10)))
shapes.extend(ring(.089,.005,.006,n=40,z=.016,color=(.89,.73,.44)))
BODIES.append(body('pie',PIE,shapes,mass=.55,color=(.55,.58,.60),friction=.8))
# Standard chopping board; all loose strips lie flat and separate on it.
BODIES.append(body('board',[.13,.025,TABLE+.021],[box((.18,.18,.042))],mass=.8,color=(.73,.53,.31),friction=.6))
ORIGINS=[PIE+[x,0,.019] for x in LONG_X]+[np.array([.13,.025+CROSS_Y[i],TABLE+.043]) for i in range(len(CROSS_Y))]
ROTATIONS=[Rotation.identity()]*2+[Rotation.from_euler('z',-90,degrees=True)]*len(CROSS_Y)
STRIPS=[];CELLS=[]
for s,(origin,rot) in enumerate(zip(ORIGINS,ROTATIONS)):
 names=[]
 for i in range(N):
  name=f'strip_{s}_{i}';names.append(name);CELLS.append(name)
  p=origin+rot.apply([0,(i-(N-1)/2)*DX,0])
  BODIES.append(body(name,p,[box((WIDTH,DX-.00002,THICK))],quat=rot.as_quat(),mass=.01375/N,color=(.90,.76,.47),friction=.6,contact_offset=.00015,speculative_ccd=True))
  if i:
   axes={'rotX':dict(limits=(-110,110),stiffness=.001,damping=.000003),'rotY':dict(limits=(-90,90),stiffness=.0005,damping=.000003),'rotZ':dict(limits=(-30,30),stiffness=.002,damping=.00002)}
   JOINTS.append(joint(f'pastry_{s}_{i}',names[-2],name,[0,DX/2,0],[0,-DX/2,0],axes=axes,break_force=2.,break_torque=.025))
 STRIPS.append(names)
finalize(BODIES,JOINTS)
