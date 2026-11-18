"""Flexible trouser cuff with a loose turned hem and an open sprung safety pin."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
NX,NY=18,10;DX,DY=.010,.024;C=np.array([.34,.12,TABLE+.006]);HEM=[]
# Both faces of a real flattened tubular leg, with a previously turned 20 mm hem.
for layer in ['front','back']:
 for i in range(NX):
  for j in range(NY):
   name=f'{layer}_{i}_{j}';pos=C+[(i-(NX-1)/2)*DX,(j-4.5)*DY,0 if layer=='front' else -.003]
   BODIES.append(body(name,pos,[box((DX*.99,DY*.99,.001))],mass=.0005,color=(.13,.19,.26),friction=.8))
   for di,dj in [(-1,0),(0,-1)]:
    if i+di>=0 and j+dj>=0:
     a=np.array([-di*DX/2,-dj*DY/2,0]);JOINTS.append(joint(f'cloth_{layer}_{i}_{j}_{di}_{dj}',f'{layer}_{i+di}_{j+dj}',name,a,-a,axes=bend_axes(.000002,.000001),break_force=35.,break_torque=.12))
for j in range(NY):
 for i in [0,NX-1]:JOINTS.append(joint(f'side_seam_{i}_{j}',f'front_{i}_{j}',f'back_{i}_{j}',[0,0,-.0015],[0,0,.0015],axes=bend_axes(.000004,.000002),break_force=20.))
for i in range(NX):
 for j in range(2):
  name=f'hem_{i}_{j}';pos=C+[(i-(NX-1)/2)*DX,-.115+j*.010,.0015];HEM.append(name)
  BODIES.append(body(name,pos,[box((DX*.99,.0099,.001))],mass=.00025,color=(.16,.22,.30),friction=.8))
  if j==0:JOINTS.append(joint('fold_'+str(i),f'front_{i}_0',name,[0,-.012,.00075],[0,-.005,-.00075],axes=bend_axes(.000003,.000001),break_force=10.))
  else:JOINTS.append(joint('hem_across_'+str(i),f'hem_{i}_0',name,[0,.005,0],[0,-.005,0],axes=bend_axes(.000002,.000001),break_force=10.))
  if i:JOINTS.append(joint(f'hem_along_{i}_{j}',f'hem_{i-1}_{j}',name,[DX/2,0,0],[-DX/2,0,0],axes=bend_axes(.000002,.000001),break_force=10.))
# Only the central 100 mm is loose; original stitching supports the rest of the hem.
for i in range(NX):
 if not 4<=i<14:
  JOINTS.append(joint('existing_stitch_'+str(i),f'front_{i}_0',f'hem_{i}_1',[0,.003,.0015],[0,0,0],axes=bend_axes(.000002,.000001),break_force=12.))
# Commercial-size 75 mm steel safety pin. Spring hinge is open at rest.
PIN=np.array([.46,-.11,TABLE+.016]);Q=Rotation.from_euler('z',90,degrees=True)
# A small ordinary tool rest supports the shaft, leaving the coil overhanging
# by 25 mm so the fingers can surround it without colliding with the table.
BODIES.append(body('pin_rest',[.46,-.065,TABLE+.0075],[box((.020,.040,.015))],mass=.03,fixed=True,color=(.62,.43,.25)))
BODIES.append(body('pin_frame',PIN,[rod_between([0,0,0],[.0685,0,0],.00065),cylinder(.003,.0026,quat=Rotation.from_euler('x',90,degrees=True).as_quat()),box((.007,.005,.002),(.0685,0,.002))],mass=.0028,color=(.7,.72,.74),quat=list(Q.as_quat()),friction=.8,contact_offset=.00005))
openR=Q*Rotation.from_euler('y',-25,degrees=True)
BODIES.append(body('pin_arm',PIN,[rod_between([0,0,0],[.0665,0,0],.00065)],mass=.0011,color=(.7,.72,.74),quat=list(openR.as_quat()),contact_offset=.00005))
axes=fixed_axes();axes['rotY']=dict(limits=(-5,35),stiffness=.20,damping=.002)
JOINTS.append(joint('pin_spring','pin_frame','pin_arm',[0,0,0],[0,0,0],axes=axes,break_force=50.,break_torque=.35))
cone=dict(kind='mesh',points=[[-.001,0,.0006],[-.001,.0006,0],[-.001,0,-.0006],[-.001,-.0006,0],[.001,0,0]],faces=[(0,1,4),(1,2,4),(2,3,4),(3,0,4),(0,3,2,1)])
BODIES.append(body('needle_point',PIN+openR.apply([.0675,0,0]),[cone],mass=.0001,color=(.7,.72,.74),quat=list(openR.as_quat()),contact_offset=.00003))
axes=fixed_axes();axes['transX']=dict(limits=(-.001,.0001),stiffness=5000.,damping=.5)
JOINTS.append(joint('point_compression','pin_arm','needle_point',[.0675,0,0],[0,0,0],axes=axes,break_force=50.))
# The shield retains the point. A ball-style contact avoids redundantly
# locking all rotations in the already hinged spring-and-catch loop.
JOINTS.append(joint('pin_catch','pin_frame','pin_arm',[.0665,0,.001],[.0665,0,0],axes=bend_axes(0.,0.,179),enabled=False,break_force=15.,break_torque=.10))
for b in list(BODIES):
 if b['name'].startswith(('front_','back_','hem_')):
  axes=bend_axes(0.,.000001,179);axes['transY']=dict(limits=(-.0005,.0005),stiffness=500.,damping=.1);axes['transZ']=dict(limits=(-.0005,.0005),stiffness=500.,damping=.1);axes['transX']=dict(limits=(-.0665,.001),stiffness=0.,damping=.05,max_force=.5)
  JOINTS.append(joint('hole_'+b['name'],'pin_arm',b['name'],[.0685,0,0],[0,0,0],axes=axes,enabled=False,break_force=3.,break_torque=.02))
# An ordinary wooden sewing support sits inside the cuff, separating its faces.
# The turned front edge overhangs it by 46 mm so a fingertip can get underneath.
cloth=[b for b in BODIES if b['name'].startswith(('front_','back_','hem_'))]
old={b['name']:(np.array(b['pos']),Rotation.from_quat(b.get('quat',[0,0,0,1]))) for b in cloth}
def height(x):return .066-min(1.,max(0.,(abs(x-.34)-.065)/.020))*.063
for b in cloth:
 x,y,z=b['pos'];back=b['name'].startswith('back_');hem=b['name'].startswith('hem_')
 slope=(height(x+.0001)-height(x-.0001))/.0002 if not back else 0.
 rot=Rotation.from_euler('y',-np.arctan(slope));b['quat']=rot.as_quat().tolist()
 b['pos']=[x,.27-y,TABLE+(.003 if back else height(x)+(.0015 if hem else 0.))]
 b['mass']=.00004 if hem else .00008;b['shapes'][0]['size'][0]*=np.sqrt(1+slope*slope)
lookup={b['name']:b for b in BODIES}
for j in JOINTS:
 if j['a'] in old and j['b'] in old:
  a,ra=old[j['a']];b,rb=old[j['b']];point=(a+ra.apply(j['p0'])+b+rb.apply(j['p1']))/2
  back=j['a'].startswith('back_') and j['b'].startswith('back_')
  extra=.0015*sum(n.startswith('hem_') for n in [j['a'],j['b']])/2
  point=np.array([point[0],.27-point[1],TABLE+(.003 if back else height(point[0])+extra)])
  for side,name in enumerate([j['a'],j['b']]):
   spec=lookup[name];j['p'+str(side)]=Rotation.from_quat(spec['quat']).inv().apply(point-spec['pos']).tolist()
  for axis in ['rotX','rotY','rotZ']:
   if 'stiffness' in j['axes'][axis]:j['axes'][axis]['stiffness']=.0001 if j['name'].startswith(('fold_','hem_')) else .00002
BODIES.append(body('sewing_support',[.34,.124,TABLE+.034],[box((.140,.190,.060))],mass=.25,fixed=True,color=(.62,.43,.25)))
for b in BODIES:
 if b['name'].startswith(('front_','back_','hem_')) or b['name']=='sewing_support':b['pos'][1]-=.15
# Small passive seam compliance avoids impulse spikes in the closed cloth grid.
for j in JOINTS:
 if j['name'].startswith(('cloth_','side_seam_','fold_','hem_','existing_stitch_')):
  for axis in ['transX','transY','transZ']:j['axes'][axis]=dict(limits=(-.0005,.0005),stiffness=500.,damping=.10)
  for axis in ['rotX','rotY','rotZ']:
   if 'stiffness' in j['axes'][axis]:j['axes'][axis].update(max_force=.00005)
# The separate 2 mm contact tip is a spring force probe, not a loose grain.
# Share a small shaft-scale rotational inertia to regularize its stiff contact.
for b in BODIES:
 if b['name']=='needle_point':b['inertia']=[1e-8,1e-8,1e-8]
 if b['name']=='pin_frame':b['inertia']=[1e-7,2e-6,2e-6]
 if b['name']=='pin_arm':b['inertia']=[1e-8,5e-7,5e-7]
# Finite rubber-pad contact area supplies native spin friction at a grip.
for part in BODIES:
 if part['name']=='pin_frame':part.update(torsional_patch_radius=.003,min_torsional_patch_radius=.0015)
finalize(BODIES,JOINTS)
