"""Ordinary pin brush, steel pin-tail comb, and freely bending hair bundles."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
BRUSH=np.array([.30,.015,TABLE+.025])
# A 190 mm brush: 95 mm head and 95 mm handle. It rests on a 20 mm rubber mat that also supports the handle.
BODIES.append(body('mat',[.30,.075,TABLE+.010],[box((.12,.22,.020))],fixed=True,mass=.08,color=(.23,.31,.35),friction=1.))
shapes=[box((.052,.095,.010)),sphere((.026,.0475,.005)),box((.021,.095,.009),(0,.09,0))]
BODIES.append(body('brush',BRUSH,shapes,mass=.08,color=(.75,.24,.33),friction=.9,torsional_patch_radius=.006,min_torsional_patch_radius=.003))
for i in range(4):
 for j in range(11):
  pos=np.array([(i-1.5)*.016,(j-5)*.006,.013]);name=f'pin_{i}_{j}'
  BODIES.append(body(name,BRUSH+pos,[cylinder(.00085,.016),sphere((.0013,)*3,(0,0,.008))],mass=.00012,color=(.90,.88,.77),contact_offset=.0001))
  JOINTS.append(joint(name+'_root','brush',name,pos,[0,0,0],axes={**fixed_axes(),'rotX':dict(limits=(-65,65),stiffness=.05,damping=.001),'rotY':dict(limits=(-65,65),stiffness=.05,damping=.001)},break_force=18.,break_torque=.12))
# Ten loose locks represent 2 g of shed hair. Each lock is a 1.4 mm contact
# proxy for many finer hairs; it is neither rigid nor glued to the brush.
HAIRS=10;SEGMENTS=14;SNAGS_PER_LOCK=1
for strand in range(HAIRS):
 y=-.030+strand*.006;u=np.linspace(-1,1,SEGMENTS+1)
 # Each strand snakes between neighbouring pins, with two loose ends.
 points=np.c_[.037*u,y+.00155*np.sin(np.pi*(.037*u)/.016),.012+.001*np.cos(3*np.pi*u+strand*.6)]+BRUSH
 for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
  v=b-a;length=np.linalg.norm(v);R=Rotation.align_vectors([v/length],[[1,0,0]])[0]
  part=body(f'hair_{strand}_{i}',(a+b)/2,[dict(kind='capsule',radius=.0007,length=float(length),pos=[0,0,0],quat=Rotation.from_euler('y',90,degrees=True).as_quat().tolist())],mass=.0002/SEGMENTS,color=(.12+.018*(strand%3),.065,.025),quat=R.as_quat().tolist(),friction=.5,contact_offset=.0001,linear_damping=.15,angular_damping=.1,visible=False)
  I=part['mass']*length*length/12;part['inertia']=[max(I/15,1e-12),max(I,1e-12),max(I,1e-12)];BODIES.append(part)
  if i:
   previous=BODIES[-2];axes={axis:dict(limits=(-180,180),stiffness=2e-7,damping=1e-8) for axis in ('rotX','rotY','rotZ')}
   JOINTS.append(joint(f'hair_join_{strand}_{i}',previous['name'],part['name'],[previous['shapes'][0]['length']/2,0,0],[-length/2,0,0],axes=axes))
# Fine fibres caught around bristle tips resist a light shake. A small passive
# breakaway constraint approximates this unresolved snag at one actual pin
# contact per lock. It supplies resistance only: the comb must contact and
# lift the flexible hair, and released locks remain ordinary dynamic bodies.
# This is an explicit material approximation, not a proximity release rule.
for strand in range(HAIRS):
 candidates=[]
 for part in [b for b in BODIES if b['name'].startswith(f'hair_{strand}_')]:
  index=int(part['name'].rsplit('_',1)[1])
  if not 2<=index<=SEGMENTS-3:continue
  for pin in [b for b in BODIES if b['name'].startswith('pin_')]:
   distance=float(np.linalg.norm((np.asarray(part['pos'])-pin['pos'])[:2]))
   if distance<.002:candidates.append((distance,index,part,pin))
 selected=[]
 for _,index,part,pin in sorted(candidates,key=lambda c:c[:2]):
  if any(abs(index-old)<4 for old in selected):continue
  selected.append(index)
  axes={axis:dict(limits=(-180,180),stiffness=0.,damping=0.) for axis in ('rotX','rotY','rotZ')}
  JOINTS.append(joint(f'hair_snag_{strand}_{len(selected)}',pin['name'],part['name'],np.asarray(part['pos'])-pin['pos'],[0,0,0],axes=axes,break_force=.010))
  if len(selected)==SNAGS_PER_LOCK:break
 assert len(selected)==SNAGS_PER_LOCK,'Each lock needs its specified contact snag'
TOOL=np.array([.43,-.10,TABLE+.027])
# 200 mm comb: 100 mm plastic toothed section and 100 mm steel pin-tail.
comb=[box((.012,.090,.004),(0,-.020,0)),cylinder(.0009,.095,(0,.0725,0),Rotation.from_euler('x',90,degrees=True).as_quat(),(.67,.69,.72))]
# A real pin-tail tapers over its last centimetre so it can wedge under hair.
angles=np.arange(12)*2*np.pi/12
v=[[r*np.cos(a),y,r*np.sin(a)] for y,r in [(.120,.0009),(.130,.00015)] for a in angles]
f=[[i,(i+1)%12,12+(i+1)%12,12+i] for i in range(12)]+[list(range(11,-1,-1)),list(range(12,24))]
comb.append(dict(kind='mesh',points=v,faces=f,pos=[0,0,0],quat=[0,0,0,1],color=[.67,.69,.72]))
comb += [box((.017,.0012,.004),(.009,-.060+i*.003,0)) for i in range(26)]
BODIES.append(body('comb',TOOL,comb,contact_report=True,mass=.016,color=(.15,.44,.61),friction=.9,torsional_patch_radius=.005,min_torsional_patch_radius=.002))
BODIES.append(body('comb_rest',[.43,-.145,TABLE+.016],[box((.030,.020,.025))],fixed=True,mass=.03,color=(.58,.43,.27)))
BODIES.append(body('comb_tail_rest',[.43,-.045,TABLE+.013],[box((.020,.020,.026))],fixed=True,mass=.02,color=(.58,.43,.27)))
BODIES.append(body('waste_dish',[.43,.15,TABLE+.012],[box((.12,.11,.003)),box((.004,.11,.020),(-.06,0,.008)),box((.004,.11,.020),(.06,0,.008)),box((.12,.004,.020),(0,-.055,.008)),box((.12,.004,.020),(0,.055,.008))],fixed=False,mass=.12,color=(.77,.84,.75),friction=.7))
finalize(BODIES,JOINTS)
