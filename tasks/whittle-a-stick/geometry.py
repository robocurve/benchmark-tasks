"""Uncut basswood square stock, sharp carving knife and chip tray."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
Q0=Rotation.from_euler("z",90,degrees=True).as_matrix()
STICK=np.array([.34,.12,TABLE+.014]);KNIFE=np.array([.34,-.06,TABLE+.018]);HANDLE=np.array([-.075,0,0.]);EDGE=np.array([0,-.017,0.])
# A genuinely uncut blank is partitioned into a core and five shaving volumes.
# No slit, weakened line or pre-cut surface exists in the physical setup.
core=[box((.012,.135,.012),(0,.0225,0))]
pts=[[x,y,z] for y,top in [(-.045,.006),(-.09,-.004)] for x,z in [(-.006,-.006),(.006,-.006),(.006,top),(-.006,top)]]
faces=[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
core.append(dict(kind='mesh',points=pts,faces=faces))
BODIES.append(body('stick',STICK,core,mass=.008,quat=Rotation.from_matrix(Q0).as_quat(),color=(.82,.65,.42),friction=.65,external_force=True))
CHIPS=[]
for i in range(5):
 name='chip_'+str(i);CHIPS.append(name)
 top=.006-.002*i;bottom=top-.002
 pts=[[-.006,-.045,.006],[.006,-.045,.006],[-.006,-.09,top],[.006,-.09,top],[-.006,-.09,bottom],[.006,-.09,bottom]]
 fs=[(0,1,3,2),(2,3,5,4),(0,4,5,1),(0,2,4),(1,5,3)]
 BODIES.append(body(name,STICK,[dict(kind='mesh',points=pts,faces=fs)],mass=.000216,quat=Rotation.from_matrix(Q0).as_quat(),color=(.88,.73,.51),friction=.6,external_force=True))
 JOINTS.append(joint('grain_'+str(i),'stick',name,[0,0,0],[0,0,0],axes=fixed_axes(),break_force=25.,break_torque=.20))
# Short broad carving blade with ordinary wooden handle. Its sharp working edge
# is a separate, rigidly attached body so only edge/wood collision is replaced.
BODIES.append(body('knife',KNIFE,[box((.10,.018,.020),HANDLE,color=(.36,.22,.10))],quat=Rotation.from_matrix(Q0).as_quat(),mass=.065,color=(.6,.63,.65),friction=.8))
BODIES.append(body('edge',KNIFE+Q0@EDGE,[box((.05,.004,.0003)),box((.05,.03,.003),-EDGE)],quat=Rotation.from_matrix(Q0).as_quat(),mass=.003,color=(.72,.74,.75),exclude=['stick']+CHIPS,external_force=True,contact_offset=.0001))
JOINTS.append(joint('blade_fix','knife','edge',EDGE,[0,0,0],axes=fixed_axes(),break_force=150.,break_torque=3.))
TRAY=np.array([.39,0,TABLE+.004])
BODIES.append(body('tray',TRAY,[box((.32,.40,.008)),*[box((.004,.40,.015),(x,0,.01)) for x in [-.16,.16]],*[box((.32,.004,.015),(0,y,.01)) for y in [-.20,.20]]],mass=.35,fixed=True,color=(.35,.40,.43)))
finalize(BODIES,JOINTS)
