"""Full-size curved practice nail, clear extension and compact C-shaped lamp."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
BASE=np.array([.36,0,TABLE+.050]);REST=np.array([.44,.085,TABLE+.050]);LAMP=np.array([.34,0,TABLE+.030])
def arch(y):return .014-np.sqrt(.014**2-np.asarray(y)**2)
def tip_width(x):return np.interp(x,[-.020,-.014,-.006,.008],[.005,.007,.006,.0048])
# Replaceable curved ABS practice nail sits in an anatomically shaped finger.
finger=[sphere((.032,.011,.010),(-.027,0,-.009)),sphere((.014,.010,.010),(-.050,0,-.009)),box((.025,.024,.035),(-.050,0,-.0275),(.61,.62,.63))]
# Curved replaceable ABS insert supports the thin gel layer; it is not a
# floating collection of contact patches.
for x in np.linspace(-.019,-.007,7):
 for y in np.linspace(-.005,.005,5):
  top=.0026-arch(y);tilt=Rotation.from_euler('x',-np.arcsin(y/.014))
  centre=np.array([x,y,top])-tilt.apply([0,0,.0015])
  finger.append(box((.002,.0025,.003),centre,(.88,.65,.61),tilt.as_quat()))
BODIES.append(body('practice_finger',BASE,finger,fixed=True,mass=.050,color=(.77,.60,.48),contact_offset=.00002))
BODIES.append(body('finger_clamp',BASE+[-.050,0,-.044],[box((.050,.055,.012))],fixed=True,mass=.15,color=(.32,.34,.36)))
PATCHES=[]
for i,x in enumerate(np.linspace(-.019,-.007,7)):
 for j,y in enumerate(np.linspace(-.005,.005,5)):
  name=f'gel_pad_{i}_{j}';z=.00272-arch(y);q=Rotation.from_euler('x',-np.arcsin(y/.014)).as_quat()
  p=BASE+[x,y,z];PATCHES.append(dict(name=name,x=float(x),y=float(y),z=float(z)))
  BODIES.append(body(name,p,[box((.00195,.00245,.00024),quat=q)],fixed=True,mass=.00002,color=(.87,.64,.60),friction=.7,contact_offset=.00002))
  # Fixed-thickness gel contact layer; no independent tiny moving tiles.
  # The native rigid contact supports the tip; wetting and cure are reduced laws.
# A rigid approximation of a thin curved full-cover tip, assembled from small
# convex sections. Concavity is preserved instead of filling the underside.
sh=[]
for x in np.linspace(-.019,.007,14):
 w=tip_width(x)
 for y in np.linspace(-w+.0007,w-.0007,9):
  q=Rotation.from_euler('x',-np.arcsin(y/.014)).as_quat()
  sh.append(box((.002,.0016,.0005),(x,y,.003-arch(y)+.00025),(.86,.91,.94),q))
for shape in sh:shape['opacity']=.35
BODIES.append(body('extension',REST,sh,mass=.00025,color=(.86,.91,.94),friction=1.,contact_offset=.00002,external_force=True))
BODIES.append(body('tip_rest',REST+[-.013,0,-.025],[box((.014,.010,.050))],fixed=True,mass=.015,color=(.61,.44,.27)))
# 66 x 58 x 47 mm compact open-front lamp; the robot reaches under its roof
# from +X with horizontal fingers. The switch is a passive spring button.
lamp=[box((.066,.058,.004),(0,0,.002)),box((.004,.058,.043),(-.031,0,.0255)),box((.066,.058,.005),(0,0,.0445))]
for y in [-.012,.012]:lamp.append(cylinder(.004,.001,(.005,y,.041),color=(.42,.35,.83)))
BODIES.append(body('lamp',LAMP,lamp,fixed=True,mass=.15,color=(.88,.88,.91)))
BUTTON=LAMP+[-.021,.017,.048]
BODIES.append(body('lamp_button',BUTTON,[cylinder(.005,.002)],mass=.001,color=(.29,.30,.35),contact_offset=.0001))
JOINTS.append(joint('button_spring','lamp','lamp_button',BUTTON-LAMP,[0,0,0],axes={**fixed_axes(),'transZ':dict(limits=(-.002,0),stiffness=600.,damping=.1)},break_force=15.))
for p in PATCHES:
 JOINTS.append(joint('bond_'+p['name'],'practice_finger','extension',[p['x'],p['y'],p['z']],[p['x'],p['y'],p['z']],enabled=False,collision=True,axes={a:dict(limits=(-.003,.003),stiffness=400.,damping=.05) for a in ['transX','transY','transZ']},break_force=.08))
BODIES.append(body('lamp_riser',LAMP+[0,0,-.015],[box((.066,.058,.030))],fixed=True,mass=.08,color=(.61,.44,.27)))
FRAME=Rotation.from_euler('z',180,degrees=True).as_matrix()
for b in BODIES:
 b['pos']=(BASE+FRAME@(np.asarray(b['pos'])-BASE)).tolist()
 b['quat']=(Rotation.from_matrix(FRAME)*Rotation.from_quat(b.get('quat',[0,0,0,1]))).as_quat().tolist()
REST=BASE+FRAME@(REST-BASE);BUTTON=BASE+FRAME@(BUTTON-BASE);LAMP=BASE+FRAME@(LAMP-BASE)
# An ordinary 20 mm spacer raises the open curing lamp clear of the robot wrist.
# It changes neither the nail nor the contact-loaded cure/proof requirements.
LAMP_SPACER=.020
for b in BODIES:
 if b['name'] in ('lamp','lamp_button'):b['pos'][2]+=LAMP_SPACER
 if b['name']=='lamp_riser':
  b['pos'][2]+=LAMP_SPACER/2
  b['shapes'][0]['size'][2]+=LAMP_SPACER
LAMP[2]+=LAMP_SPACER;BUTTON[2]+=LAMP_SPACER
RAISE=.065
for b in BODIES:b['pos'][2]+=RAISE
for p in [BASE,REST,BUTTON,LAMP]:p[2]+=RAISE
# Mount the practice finger on the lamp's base, rather than intersecting it.
# The 24 mm lift clears the 4 mm lamp floor; 20 mm towards the opening keeps
# the proximal clamp clear of the lamp's rear wall. All pieces remain full size.
FINGER_SHIFT=np.array([-.020,0,.024])
for b in BODIES:
 if b['name'] in ('practice_finger','finger_clamp') or b['name'].startswith('gel_pad_'):
  b['pos']=(np.asarray(b['pos'])+FINGER_SHIFT).tolist()
 if b['name']=='practice_finger':
  b['shapes'][2]=box((.025,.024,.006),(-.050,0,-.017),(.61,.62,.63))
 if b['name']=='finger_clamp':
  b['pos']=(BASE+FINGER_SHIFT+FRAME@np.array([-.050,0,-.017])).tolist()
  b['shapes']=[box((.025,.025,.006))]
BASE+=FINGER_SHIFT
BODIES.append(body('work_riser',[.35,-.04,TABLE+RAISE/2],[box((.24,.30,RAISE))],fixed=True,mass=.8,color=(.58,.43,.28)))
finalize(BODIES,JOINTS)
