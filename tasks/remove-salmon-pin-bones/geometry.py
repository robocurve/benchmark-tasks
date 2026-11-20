"""Free fillet, compliant flesh sectors and four retained, breakable pin bones."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
C=np.array([.36,0,TABLE+.025]);N=4
AXIS=np.array([-2**-.5,0,2**-.5]);BQ=Rotation.align_vectors([AXIS],[[0,0,1]])[0].as_quat()
BODIES.append(body('board',[.36,0,TABLE+.006],[box((.28,.26,.012))],fixed=True,mass=.7,color=(.75,.63,.43)))
BODIES.append(body('skin',C+[0,0,-.011],[box((.09,.15,.002))],mass=.012,color=(.37,.43,.43),friction=.8))
FLESH=[];TIPS=[]
for row in range(4):
 y=(row-1.5)*.037
 for col in range(3):
  x=(col-1)*.0298;name=f'flesh_{row}_{col}';FLESH.append(name)
  # Close-packed yielding sectors approximate local compression and tearing.
  BODIES.append(body(name,C+[x,y,0],[box((.0294,.0365,.022))],mass=.014,color=(.94,.43+.02*(row%2),.29),friction=.55,contact_offset=.00008,visible=False))
  axes={a:dict(limits=(-.012,.012),stiffness=400.,damping=.4) for a in ['transX','transY','transZ']}
  axes.update({a:dict(limits=(-25,25),stiffness=.08,damping=.002) for a in ['rotX','rotY','rotZ']})
  JOINTS.append(joint('flesh_bond_'+name,'skin',name,[x,y,.011],[0,0,0],axes=axes,break_force=3.,break_torque=.04))
for i in range(N):
 y=(i-1.5)*.037;base=C+[.010,y,-.004];top=base+AXIS*.025;TIPS.append(top)
 exclusions=FLESH+['skin']
 BODIES.append(body(f'bone_base_{i}',base+AXIS*.011,[cylinder(.0006,.022)],quat=BQ,mass=.00004,color=(.95,.91,.78),friction=.7,exclude=exclusions,contact_offset=.00005))
 BODIES.append(body(f'bone_tip_{i}',top,[cylinder(.0006,.006)],quat=BQ,mass=.000015,color=(.97,.94,.83),friction=.8,exclude=exclusions,contact_offset=.00005))
 axes=fixed_axes();axes['transZ']=dict(limits=(0.,.045),stiffness=1500.,damping=.05,max_force=.35)
 JOINTS.append(joint(f'bone_socket_{i}',f'flesh_{i}_1',f'bone_base_{i}',base-(C+[0,y,0]),[0,0,-.011],q0=BQ,axes=axes))
 JOINTS.append(joint(f'bone_integrity_{i}',f'bone_base_{i}',f'bone_tip_{i}',[0,0,.011],[0,0,-.003],axes=bend_axes(.003,.00003,limit=30),break_force=3.,break_torque=.012))
DISH=np.array([.32,-.215,TABLE+.002])
BODIES.append(body('dish',DISH,[cylinder(.045,.004)]+ring(.045,.024,.004,z=.010),mass=.08,color=(.82,.86,.88),friction=.7))
finalize(BODIES,JOINTS)
