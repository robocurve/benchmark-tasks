"""Ordinary 32 mm two-hook closure on passive elastic wings around a torso form."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
Z=TABLE+.155;X=.345;WIDTH=.032;GAP=.040
# Full-sized dress form cropped below the shoulders; its base is fixed to the table.
BODIES.append(body('torso',[.470,0,TABLE+.155],[sphere((.080,.130,.155)),sphere((.075,.11,.075),(0,0,.125)),cylinder(.040,.045,(0,0,.205))],mass=3.,fixed=True,color=(.74,.60,.46),friction=.6))
BODIES.append(body('stand',[.470,0,TABLE+.005],[box((.24,.30,.01))],mass=1.,fixed=True,color=(.25,.28,.30)))
# Non-working cups on the front of the torso establish the garment context.
for sign in [-1,1]:
 BODIES.append(body('cup_'+str(sign).replace('-','n'),[.546,sign*.065,TABLE+.17],[sphere((.06,.065,.068))],mass=.02,fixed=True,color=(.20,.28,.45)))
BANDS=[]
for side,sign in [('left',1),('right',-1)]:
 # Seven flexible elastic sectors, x-normal, y-longitudinal, z-width.
 points=np.array([[X+.125*(1-t)**2,sign*(.130-.075*t),Z] for t in np.linspace(0,1,8)])
 previous='torso';previous_center=np.array([.470,0,TABLE+.155])
 for i in range(7):
  center=(points[i]+points[i+1])/2;length=np.linalg.norm(points[i+1]-points[i]);name=f'{side}_wing_{i}';BANDS.append(name)
  direction=(points[i+1]-points[i])/length
  # geometry aligns with the local longitudinal direction without changing width
  R=Rotation.from_euler('z',np.arctan2(-direction[0]*sign,direction[1]*sign))
  BODIES.append(body(name,center,[box((.0006,length-.0001,WIDTH))],quat=R.as_quat(),mass=.0005,color=(.20,.28,.45),friction=.7,contact_offset=.0001))
  lookup={b['name']:b for b in BODIES};prev=lookup[previous];Rp=Rotation.from_quat(prev['quat']) if 'quat' in prev else Rotation.identity()
  axes=bend_axes(.005,.0001,limit=100);axes['rotX']=dict(limits=(-45,45),stiffness=.05,damping=.002);axes['rotY']=dict(limits=(-100,100),stiffness=.01,damping=.0002);axes['transY']=dict(limits=(-.016,.016),stiffness=300.,damping=.2)
  JOINTS.append(joint(name+'_seam',previous,name,Rp.inv().apply(points[i]-previous_center),R.inv().apply(points[i]-center),axes=axes,q0=(R.as_quat() if i==0 else [0,0,0,1]),break_force=18.,break_torque=.1))
  previous=name;previous_center=center
 name=side+'_tab';center=np.array([X,sign*GAP,Z]);shapes=[box((.0006,.030 if side=='left' else .032,WIDTH))]
 if side=='left':
  # Two ungated J hooks on the UNDERSIDE of the overlapping hook tape.
  # Tape stays outside the eye tape; hook tips face +X toward the torso.
  # Pockets open along +Y, away from wing tension.
  for z in [-.008,.008]:
   pts=np.array([[0,-.006,z],[0,-.012,z],[.004,-.012,z],[.004,-.009,z]])
   shapes += [rod_between(a,b,.0004) for a,b in zip(pts[:-1],pts[1:])]
 else:
  # Three recessed adjustment rows on the outer face of the underlapping tape.
  # Outer bar is 3 mm inside the fabric edge, never projecting past it.
  for y in [.013,.004,-.005]:
   for z in [-.008,.008]:
    pts=np.array([[0,y-.006,z-.003],[-.002,y,z-.003],[-.002,y,z+.003],[0,y-.006,z+.003]])
    shapes += [rod_between(a,b,.0004) for a,b in zip(pts[:-1],pts[1:])]
 for shape in shapes[1:]:shape['color']=[.68,.70,.73]
 BODIES.append(body(name,center,shapes,mass=.002,color=(.20,.28,.45),friction=.65,contact_offset=.00008))
 prev=next(b for b in BODIES if b['name']==previous);Rp=Rotation.from_quat(prev['quat']);anchor=np.array([X,sign*.055,Z])
 JOINTS.append(joint(name+'_seam',previous,name,Rp.inv().apply(anchor-previous_center),anchor-center,axes={**bend_axes(.005,.0001,limit=80),'rotX':dict(limits=(-45,45),stiffness=.05,damping=.002)},break_force=18.,break_torque=.1))
# Real shoulder straps support the rear wings; omitting them made a hanging
# pair of free ribbons rather than a bra already draped on a form.
for side,sign in [('left',1),('right',-1)]:
 wing=next(b for b in BODIES if b['name']==side+'_wing_2');end=np.array(wing['pos'])+[0,0,WIDTH/2]
 start=np.array([.46,sign*.080,TABLE+.340]);points=np.array([start+(end-start)*t+np.array([-.010*np.sin(np.pi*t),0,0]) for t in np.linspace(0,1,7)])
 previous='torso';previous_center=np.array([.470,0,TABLE+.155]);previous_R=Rotation.identity()
 for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
  centre=(a+b)/2;v=b-a;length=np.linalg.norm(v);Q=Rotation.align_vectors([v/length],[[0,0,1]])[0];name=f'{side}_shoulder_{i}'
  BODIES.append(body(name,centre,[box((.0008,.012,length-.0001))],quat=Q.as_quat(),mass=.0006,color=(.20,.28,.45),friction=.7,contact_offset=.0001))
  axes=bend_axes(.001,.00005,limit=160);axes['transZ']=dict(limits=(-.004,.008),stiffness=1000.,damping=.3)
  JOINTS.append(joint(name+'_seam',previous,name,previous_R.inv().apply(a-previous_center),Q.inv().apply(a-centre),axes=axes,break_force=18.,break_torque=.1))
  previous=name;previous_center=centre;previous_R=Q
 Qwing=Rotation.from_quat(wing['quat'])
 JOINTS.append(joint(side+'_shoulder_wing',previous,wing['name'],previous_R.inv().apply(end-previous_center),Qwing.inv().apply(end-np.array(wing['pos'])),axes=bend_axes(.001,.00005,limit=160),break_force=18.,break_torque=.1))
# The front/side garment is supported on the dress form, as when a garment
# is pinned at its side seams for a closure demonstration. Rear working wings
# and both closure tapes remain passive, elastic and unfastened.
for item in BODIES:
 if '_shoulder_' in item['name'] or item['name'] in [f'{side}_wing_{i}' for side in ['left','right'] for i in range(3)]:item['fixed']=True
JOINTS[:]=[j for j in JOINTS if '_shoulder_' not in j['name'] and j['name'] not in [f'{side}_wing_{i}_seam' for side in ['left','right'] for i in range(3)]]
X-=.075
for item in BODIES:item["pos"][0]-=.075
# Reduced-order elastic wings: native compliant end links carry load. The
# intermediate ribbon is drawn from those endpoints; distributed cloth contact
# is intentionally not simulated. It cannot latch or guide either hook.
BODIES[:]=[b for b in BODIES if b['name'] not in [f'{side}_wing_{i}' for side in ['left','right'] for i in range(3,7)]]
JOINTS=[];BAND_ANCHORS={};BAND_REST={}
lookup={b['name']:b for b in BODIES}
for side,sign in [('left',1),('right',-1)]:
 fixed=lookup[side+'_wing_2'];F=Rotation.from_quat(fixed['quat']);half=fixed['shapes'][0]['size'][1]/2
 anchor=np.array(fixed['pos'])+F.apply([0,-sign*half,0]);end=np.array(lookup[side+'_tab']['pos'])+[0,sign*.015,0]
 BAND_ANCHORS[side]=anchor;BAND_REST[side]=float(np.linalg.norm(end-anchor))
 axes={f'trans{a}':dict(limits=(-.080,.080),stiffness=k,damping=.25) for a,k in zip('XYZ',[20.,100.,20.])}
 axes.update(bend_axes(.003,.0001,limit=160))
 JOINTS.append(joint(side+'_elastic_wing',side+'_wing_2',side+'_tab',F.inv().apply(end-np.array(fixed['pos'])),[0,sign*.015,0],q0=F.inv().as_quat(),axes=axes,break_force=15.,break_torque=.08))
finalize(BODIES,JOINTS)
