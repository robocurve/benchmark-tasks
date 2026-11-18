"""Cooked pieces as breakable cohesive particle clusters, not tool-path scoring."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
BOWL=np.array([.34,0,TABLE+.003])
BODIES.append(body('bowl',BOWL,[cylinder(.099,.006)]+ring(.10,.075,.005,z=.037),mass=.30,color=(.62,.70,.76),friction=.6))
# 12 cooked pieces, each 21 mm across: 27 cohesive 7 mm particles, 58 g total.
GRID=np.array(list(np.ndindex(3,3,3)))-1
for k in range(12):
 center=BOWL+np.array([(k%4-1.5)*.024,(k//4-1)*.024,.018])
 for n,xyz in enumerate(GRID):
  pos=center+xyz*.0072;name=f'food_{k:02d}_{n}'
  BODIES.append(body(name,pos,[sphere((.0035,.0035,.0035))],mass=.00018,color=(.91,.82,.56),friction=.5,contact_offset=.0001,linear_damping=2.,angular_damping=5.,max_linear_velocity=2.))
 for a in range(len(GRID)):
  for b in range(a+1,len(GRID)):
   if np.sum(abs(GRID[a]-GRID[b]))!=1:continue
   delta=(GRID[b]-GRID[a])*.0072
   JOINTS.append(joint(f'cohesion_{k}_{a}_{b}',f'food_{k:02d}_{a}',f'food_{k:02d}_{b}',delta/2,-delta/2,axes={**{a:dict(limits=(-.004,.004),stiffness=40.,damping=.10) for a in ['transX','transY','transZ']},**bend_axes(.000001,.000001,179)},break_force=.04,break_torque=.001))
# An ordinary broad wire masher, standing on its head. Gaps let fragments through.
shapes=[box((.075,.004,.004),(0,y,0)) for y in np.linspace(-.027,.027,5)]
shapes += [box((.004,.060,.004),(x,0,0)) for x in [-.036,.036]]
shapes += [cylinder(.003,.10,(x,0,.05)) for x in [-.025,.025]]
shapes += [box((.056,.006,.006),(0,0,.10)),cylinder(.010,.090,(0,0,.135),color=(.18,.20,.23))]
BODIES.append(body('masher',[.34,-.17,TABLE+.003],shapes,mass=.14,color=(.68,.70,.72),friction=.9))

finalize(BODIES,JOINTS)
