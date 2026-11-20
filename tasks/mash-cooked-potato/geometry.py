"""Cooked pieces as breakable cohesive particle clusters, not tool-path scoring."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
BOWL=np.array([.34,0,TABLE+.003])
BODIES.append(body('bowl',BOWL,[cylinder(.099,.006)]+ring(.10,.075,.005,z=.037),mass=.30,color=(.62,.70,.76),friction=.6))
# Twelve 17.0 mm cooked cubes, 4.86 g per cluster and 58.32 g in total.
# The coarse particle gaps do not represent air within real potato.
PARTICLE_DIAMETER=.007*(.017/.0214)
SPACING=.0072*(.017/.0214)
GRID=np.array(list(np.ndindex(3,3,3)))-1
for k in range(12):
 center=BOWL+np.array([(k%4-1.5)*.020,(k//4-1)*.020,.014])
 for n,xyz in enumerate(GRID):
  pos=center+xyz*SPACING;name=f'food_{k:02d}_{n}'
  BODIES.append(body(name,pos,[sphere((PARTICLE_DIAMETER/2,)*3)],mass=.00018,color=(.91,.82,.56),friction=.5,contact_offset=.0001,linear_damping=20.,angular_damping=5.,max_linear_velocity=.5))
 for a in range(len(GRID)):
  for b in range(a+1,len(GRID)):
   if np.sum(abs(GRID[a]-GRID[b]))!=1:continue
   delta=(GRID[b]-GRID[a])*SPACING
   JOINTS.append(joint(f'cohesion_{k}_{a}_{b}',f'food_{k:02d}_{a}',f'food_{k:02d}_{b}',delta/2,-delta/2,axes={**{a:dict(limits=(-.004,.004),stiffness=40.,damping=.10) for a in ['transX','transY','transZ']},**bend_axes(.000001,.000001,179)},break_force=.04,break_torque=.001))
# An ordinary broad wire masher, standing on its head. Gaps let fragments through.
shapes=[box((.075,.004,.004),(0,y,0)) for y in np.linspace(-.027,.027,5)]
shapes += [box((.004,.060,.004),(x,0,0)) for x in [-.036,.036]]
shapes += [cylinder(.003,.10,(x,0,.05)) for x in [-.025,.025]]
shapes += [box((.056,.006,.006),(0,0,.10)),cylinder(.010,.090,(0,0,.135),color=(.18,.20,.23))]
BODIES.append(body('masher',[.34,-.17,TABLE+.003],shapes,mass=.14,color=(.68,.70,.72),friction=.9))

finalize(BODIES,JOINTS)
