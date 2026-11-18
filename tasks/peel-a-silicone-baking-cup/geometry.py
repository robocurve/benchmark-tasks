"""Jointed silicone walls around a cohesive muffin with peelable adhesion."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
C=np.array([.34,0,TABLE+.00475]);N=16
BODIES.append(body('tray',[.34,0,TABLE+.002],[box((.32,.28,.004))],mass=.2,fixed=True,color=(.65,.69,.72)))
BODIES.append(body('cup_bottom',C,[cylinder(.026,.0015)],mass=.004,color=(.18,.46,.74),friction=.9))
BODIES.append(body('muffin',C+[0,0,.030],[cylinder(.024,.042,(0,0,-.007))],mass=.025,color=(.63,.38,.14),friction=.65))
# Native passive wall panels allow the cup to turn inside out under load.
for i,t in enumerate(np.arange(N)*2*np.pi/N):
 radial=np.array([np.sin(t),np.cos(t),0]);q=Rotation.from_euler('z',-t).as_quat()
 for row in range(2):
  r=.026+row*.004;z=.010+row*.018;name=f'cup_{row}_{i:02d}'
  BODIES.append(body(name,C+radial*r+[0,0,z],[box((2*r*np.tan(np.pi/N),.0015,.020))],mass=.0007,color=(.18,.46,.74),quat=list(q),friction=.9,exclude=[f'cup_{rr}_{jj:02d}' for rr in range(2) for jj in [(i-1)%N,i,(i+1)%N] if (rr,jj)!=(row,i)]))
  if row==0:
   JOINTS.append(joint('base_'+str(i),'cup_bottom',name,radial*.026,[0,0,-.010],q0=list(q),axes=bend_axes(.00002,.000008,179),break_force=20.))
  else:
   JOINTS.append(joint('wall_'+str(i),f'cup_0_{i:02d}',name,[0,.002,.009],[0,-.002,-.009],axes=bend_axes(.00002,.000008,179),break_force=20.))
  # Soft circumferential seams maintain a cup, rather than independent flaps.
  if i:
   prev=f'cup_{row}_{i-1:02d}';pt=(C+np.array([np.sin(t-np.pi/N),np.cos(t-np.pi/N),0])*r+[0,0,z])
   pprev=C+np.array([np.sin(t-2*np.pi/N),np.cos(t-2*np.pi/N),0])*r+[0,0,z]
   Rprev=Rotation.from_euler('z',-(t-2*np.pi/N)).as_matrix();Ri=Rotation.from_quat(q).as_matrix()
   JOINTS.append(joint(f'ring_{row}_{i}',prev,name,Rprev.T@(pt-pprev),Ri.T@(pt-(C+radial*r+[0,0,z])),axes=bend_axes(.000004,.000003,179),break_force=20.))
  if row==0:
   local=radial*.024+[0,0,-.020]
   JOINTS.append(joint('adhesion_'+str(i),'muffin',name,local,[0,-.002,0],axes=bend_axes(.000001,.000001),break_force=.60,break_torque=.02,collision=True))
JOINTS.append(joint('bottom_adhesion','muffin','cup_bottom',[0,0,-.030],[0,0,0],axes=bend_axes(.000001,.000001),break_force=1.5,break_torque=.01,collision=True))

# Four damageable quarter-cap solids form one continuous ordinary muffin top.
# Convex baked meshes keep native contacts aligned with the visual surface.
from scipy.spatial import ConvexHull
for i in range(4):
 rel=np.array([0.,0.,.022]);name='crumb_'+str(i)
 points=np.array([[.036*np.sin(p)*np.cos(t),.036*np.sin(p)*np.sin(t),.014*np.cos(p)] for p in np.linspace(0,np.pi,9) for t in np.linspace(i*np.pi/2,(i+1)*np.pi/2,9)])
 points=np.unique(np.round(points,10),axis=0);hull=ConvexHull(points)
 faces=[]
 for face,equation in zip(hull.simplices,hull.equations):
  a,b,c=points[face]
  faces.append(face.tolist() if np.dot(np.cross(b-a,c-a),equation[:3])>0 else face[::-1].tolist())
 shape=dict(kind='mesh',points=points.tolist(),faces=faces,pos=[0.,0.,0.])
 BODIES.append(body(name,C+[0,0,.030]+rel,[shape],mass=.005,color=(.70,.43,.16),exclude=['crumb_'+str(j) for j in range(4) if j!=i],friction=.8))
 axes={**{a:dict(limits=(-.004,.004),stiffness=1000.,damping=.05,max_force=6.) for a in ['transX','transY','transZ']},**{a:dict(limits=(-20,20),stiffness=.01,damping=.0005,max_force=.12) for a in ['rotX','rotY','rotZ']}}
 JOINTS.append(joint('crumb_bond_'+str(i),'muffin',name,rel,[0,0,0],axes=axes,break_force=6.,break_torque=.12))
for row in range(2):
 a=next(b for b in BODIES if b['name']==f'cup_{row}_{N-1:02d}');b=next(b for b in BODIES if b['name']==f'cup_{row}_00')
 centre=(np.array(a['pos'])+b['pos'])/2
 JOINTS.append(joint(f'ring_{row}_0',a['name'],b['name'],Rotation.from_quat(a['quat']).inv().apply(centre-a['pos']),Rotation.from_quat(b['quat']).inv().apply(centre-b['pos']),axes=bend_axes(.000004,.000003,179),break_force=20.))

# Ordinary moulded pull tabs give fingers clearance from the fragile muffin.
for i in [0,8]:
 t=i*2*np.pi/N;radial=np.array([np.sin(t),np.cos(t),0]);q=Rotation.from_euler('z',-t).as_quat();name='cup_tab_'+str(i)
 BODIES.append(body(name,C+radial*.042+[0,0,.032],[box((.016,.024,.0015))],mass=.00035,color=(.18,.46,.74),quat=list(q),friction=1.))
 JOINTS.append(joint('tab_seam_'+str(i),f'cup_1_{i:02d}',name,[0,0,.004],[0,-.012,0],axes=bend_axes(.00002,.000008,179),break_force=20.))

# Flexible cup seams use passive angular hinges; closed loops preserve continuity.
# Adhesion supplies the resistance to peeling, without unstable tiny-body drives.
for j in JOINTS:
 for axis,cfg in j['axes'].items():
  if axis.startswith('rot') and 'stiffness' in cfg:
   if j['name'].startswith('tab_seam_'):cfg.update(stiffness=.0002,damping=.00001,max_force=.0004)
   elif j['name'].startswith(('ring_','wall_','base_')):
    # USD gains are per degree: about 0.00029 N m/rad of passive recovery.
    cfg.update(stiffness=.000005,damping=.00000002,max_force=.0005)
   elif not j['name'].startswith('crumb_bond_'):cfg.update(stiffness=0.,damping=0.)
for j in JOINTS:
 if j['name'].startswith(('ring_','wall_','base_')):
  for axis in ['transX','transY','transZ']:j['axes'][axis]=dict(limits=(-.003,.003),stiffness=50.,damping=.05)
for b in BODIES:b.update(linear_damping=.5,angular_damping=2.)
finalize(BODIES,JOINTS)
