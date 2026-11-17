"""Flexible stalk and paired leaf strips with weaker stalk attachments."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
BODIES.append(body('board',[.34,0,TABLE+.010],[box((.32,.34,.020))],mass=1.,fixed=True,color=(.65,.42,.22)))
N=15;DS=.018;Z=TABLE+.025
for i in range(N):
 y=.126-i*DS;name=f'stalk_{i:02d}'
 BODIES.append(body(name,[.34,y,Z],[cylinder(.003,DS,quat=Rotation.from_euler('x',90,degrees=True).as_quat())],mass=.0007,color=(.31,.48,.17),friction=.9))
 if i:JOINTS.append(joint('stem_'+str(i),f'stalk_{i-1:02d}',name,[0,-DS/2,0],[0,DS/2,0],axes=bend_axes(.002,.0001,50),break_force=25.,break_torque=.20))
 # First 45 mm remains an ordinary bare stalk to hold.
 if i<3:continue
 width=.033*np.sin(np.pi*(i-2)/(N-1))+.009
 for side in [-1,1]:
  name=f'leaf_{side:+d}_{i:02d}'.replace('+','p').replace('-','n');x=.34+side*(width/2+.003)
  BODIES.append(body(name,[x,y,Z-.001],[box((width,DS*.98,.0012))],mass=.00035,color=(.11,.36,.09),friction=.9))
  JOINTS.append(joint(f'attach_{side}_{i}',f'stalk_{i:02d}',name,[side*.003,0,0],[-side*width/2,0,.001],axes=bend_axes(.000002,.000001),break_force=.9,break_torque=.004,collision=True))
  if i>3:
   prev=f'leaf_{side:+d}_{i-1:02d}'.replace('+','p').replace('-','n');wp=.033*np.sin(np.pi*(i-3)/(N-1))+.009
   # Adjacent leaf patches meet near their shared mid-width, permitting curl.
   JOINTS.append(joint(f'leaf_seam_{side}_{i}',prev,name,[side*(width-wp)/4,-DS/2,0],[side*(wp-width)/4,DS/2,0],axes=bend_axes(.000002,.000001),break_force=4.,break_torque=.02))

finalize(BODIES,JOINTS)
