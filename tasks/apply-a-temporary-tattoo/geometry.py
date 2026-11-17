"""Flexible transfer paper, compressible wet sponge, and a curved PVC forearm."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
ARM=np.array([.34,.025,TABLE+.045]);ARM_RADIUS=.032
# A rigid practice form covered in smooth PVC vinyl, mounted on ordinary rests.
BODIES.append(body('forearm',ARM,[cylinder(ARM_RADIUS,.20,quat=Rotation.from_euler('y',90,degrees=True).as_quat())],mass=.25,fixed=True,color=(.69,.44,.31)))
for side in [-1,1]:BODIES.append(body('support_'+str(side).replace('-','n'),ARM+[side*.080,0,-.032],[box((.025,.07,.026))],mass=.05,fixed=True,color=(.25,.29,.33)))
# Printed target is a 40 x 35 mm rectangle near the top of the form.
forearm=next(b for b in BODIES if b['name']=='forearm')
for side in [-1,1]:
 a=side*.0175/ARM_RADIUS;y=(ARM_RADIUS+.0002)*np.sin(a);z=(ARM_RADIUS+.0002)*np.cos(a)
 line=rod_between([-.020,y,z],[.020,y,z],.00025);line.update(collision=False,color=(.12,.12,.12));forearm['shapes'].append(line)
 arc=np.linspace(-.0175/ARM_RADIUS,.0175/ARM_RADIUS,13)
 for a,b in zip(arc[:-1],arc[1:]):
  pts=[[side*.020,(ARM_RADIUS+.0002)*np.sin(t),(ARM_RADIUS+.0002)*np.cos(t)] for t in [a,b]]
  line=rod_between(*pts,.00025);line.update(collision=False,color=(.12,.12,.12));forearm['shapes'].append(line)

NX,NY=10,12;DX=DY=.005;PAPER=np.array([.34,.13,TABLE+.0303])
BODIES.append(body('paper_rest',[.34,.13,TABLE+.015],[box((.030,.040,.030))],mass=.02,fixed=True,color=(.55,.63,.69)))
PATTERN=['0110110','1111111','1111111','0111110','0011100','0001000'];INK=[];INK_BODY={};INK_ORIGINAL={}
# Twelve full-width paper bands retain the bending needed around the forearm.
# The printed coating is finer than the collision discretization.
for j in range(NY):
 name=f'paper_band_{j}';y=(j-(NY-1)/2)*DY;shapes=[box((NX*DX,DY*.98,.00030))]
 for i in range(NX):
  x=(i-(NX-1)/2)*DX;ink=1<=i<=7 and 3<=j<=8 and PATTERN[j-3][i-1]=='1'
  if ink:
   patch=f'paper_{i}_{j}';INK.append(patch);INK_BODY[patch]=(name,[x,0,0],len(shapes));INK_ORIGINAL[patch]=[x,y]
   shape=box((DX*.95,DY*.95,.00004),(x,0,-.00017),(.8,.03,.05));shape['collision']=False;shapes.append(shape)
 BODIES.append(body(name,PAPER+[0,y,0],shapes,mass=.000060,color=(.95,.93,.85),friction=.6,contact_offset=.00003,linear_damping=.5,angular_damping=2.))
 if j:
  axes=fixed_axes();axes['rotX']=dict(limits=(-120,120),stiffness=.00002,damping=.000000001)
  JOINTS.append(joint('sheet_'+str(j),f'paper_band_{j-1}',name,[0,DY/2,0],[0,-DY/2,0],axes=axes,break_force=3.,break_torque=.02))
for patch,(name,offset,index) in INK_BODY.items():
 JOINTS.append(joint('tack_'+patch,'forearm',name,[0,0,ARM_RADIUS],offset,axes=bend_axes(0.,0.),break_force=.20,enabled=False,collision=True))
SPONGE=np.array([.34,-.16,TABLE+.025])
BODIES.append(body('sponge',SPONGE,[box((.040,.040,.015))],mass=.012,color=(.85,.67,.16),friction=.9))
PADCOUNT=16
for i,(x,y) in enumerate([(x,y) for x in [-.015,-.005,.005,.015] for y in [-.015,-.005,.005,.015]]):
 name='sponge_pad_'+str(i);rel=np.array([x,y,-.0125])
 BODIES.append(body(name,SPONGE+rel,[box((.010,.010,.010))],mass=.00075,color=(.87,.71,.20),friction=.8,exclude=['sponge_pad_'+str(j) for j in range(PADCOUNT) if j!=i]))
 axes=fixed_axes();axes['transZ']=dict(limits=(-.001,.011),stiffness=100.,damping=.2)
 JOINTS.append(joint('compression_'+str(i),'sponge',name,rel,[0,0,0],axes=axes,break_force=30.))
BODIES.append(body('sponge_dish',[.34,-.16,TABLE+.003],[box((.070,.070,.006))]+[box((.070,.003,.015),(0,s*.034,.006)) for s in [-1,1]]+[box((.003,.070,.015),(s*.034,0,.006)) for s in [-1,1]],mass=.08,color=(.65,.73,.80)))
finalize(BODIES,JOINTS)
