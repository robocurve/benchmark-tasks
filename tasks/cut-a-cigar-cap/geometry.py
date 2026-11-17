"""Unlit cigar, compliant wrapper sectors, and an ordinary double guillotine."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
BODIES.append(body('board',[.34,0,TABLE+.010],[box((.32,.34,.020))],mass=1.,fixed=True,color=(.57,.36,.20)))
CIGAR=np.array([.34,.09,TABLE+.030]);CQ=Rotation.from_euler('x',90,degrees=True);RADIUS=.0092
BODIES.append(body('cigar',CIGAR,[cylinder(.0072,.134)],mass=.007,color=(.35,.19,.09),quat=list(CQ.as_quat()),friction=.6))
# Four yielding wrapper sectors preserve a meaningful crushing failure.
for i,t in enumerate(np.arange(4)*np.pi/2):
 radial=np.array([np.cos(t),np.sin(t),0]);name='wrapper_'+str(i);shape=[]
 for a in np.linspace(-.68,.68,7):
  q=t+a;shape.append(box((.0025,.0015,.134),(.0085*np.cos(q),.0085*np.sin(q),0),quat=Rotation.from_euler('z',q+np.pi/2).as_quat()))
 BODIES.append(body(name,CIGAR,shape,mass=.00035,color=(.42,.23,.12),quat=list(CQ.as_quat()),friction=.7))
 axes=fixed_axes();axes['transX']=dict(limits=(-.0016,.0002),stiffness=8000.,damping=.5)
 q=Rotation.from_euler('z',t).as_quat()
 JOINTS.append(joint('wrapper_spring_'+str(i),'cigar',name,[0,0,0],[0,0,0],q0=list(q),axes=axes,break_force=40.))
CAP=CIGAR+CQ.apply([0,0,.068])
BODIES.append(body('cap',CAP,[cylinder(RADIUS,.002)],mass=.0002,color=(.43,.24,.13),quat=list(CQ.as_quat()),friction=.6))
JOINTS.append(joint('cap_bond','cigar','cap',[0,0,.068],[0,0,0],axes=fixed_axes(),break_force=60.,break_torque=.4))
CUTTER=np.array([.34,-.04,TABLE+.0245])
# A 24 mm opening with a protective housing; handles extend to 82 mm when open.
BODIES.append(body('cutter',CUTTER,ring(.018,.008,.008,n=24),mass=.018,color=(.12,.15,.18),friction=.8))
for side in [-1,1]:
 tag='left' if side<0 else 'right';origin=np.array([side*.034,0,0]);shapes=[box((.014,.028,.009),(side*.001,0,0),(.18,.20,.22))]
 # Semicircular cutting edge, one half on each side of the aperture.
 start=np.pi/2 if side<0 else -np.pi/2
 for i in range(16):
  aa=start+i*np.pi/16;bb=aa+np.pi/16
  xy=np.array([[.012*np.cos(aa),.012*np.sin(aa)],[.024*np.cos(aa),.024*np.sin(aa)],[.024*np.cos(bb),.024*np.sin(bb)],[.012*np.cos(bb),.012*np.sin(bb)]])
  points=np.array([[x-origin[0],y,z] for z in [side*.00015-.0001,side*.00015+.0001] for x,y in xy]);faces=[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
  shapes.append(dict(kind='mesh',points=points.tolist(),faces=faces,color=(.65,.68,.70)))
 BODIES.append(body('blade_'+tag,CUTTER+origin,shapes,mass=.012,color=(.6,.63,.66),friction=.9,exclude=['cap','blade_'+('right' if side<0 else 'left')],contact_offset=.0001))
 axes=fixed_axes();axes['transX']=dict(limits=((0,.022) if side<0 else (-.022,0)),stiffness=350.,damping=.3)
 JOINTS.append(joint('slider_'+tag,'cutter','blade_'+tag,origin,[0,0,0],axes=axes,break_force=100.))
BODIES.append(body('waste_dish',[.48,-.05,TABLE+.003],[cylinder(.042,.006)]+ring(.042,.018,.004,z=.009),mass=.06,color=(.65,.73,.80)))
for b in BODIES:
 if b['name'] in ['cap','blade_left','blade_right']:b['external_force']=True
finalize(BODIES,JOINTS)
