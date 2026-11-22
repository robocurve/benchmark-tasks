"""Full-size local roof-edge assembly, gutter scoop, bucket and flexible dry leaves."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
GUTTER=np.array([.30,0,TABLE+.14]);RADIUS=.056;LENGTH=.60
# Half-round PVC rain gutter, open upwards, with a real fascia and overhang.
shapes=[]
for a in np.linspace(np.pi,2*np.pi,25)[:-1]+np.pi/48:
 x=RADIUS*np.cos(a);z=RADIUS*np.sin(a)
 shapes.append(box((np.pi*RADIUS/24+.001,LENGTH,.003),(x,0,z),quat=Rotation.from_euler('y',-a-np.pi/2).as_quat()))
shapes += [box((.112,.003,.056),(0,y,-.028)) for y in [-.30,.30]]
shapes += [box((.018,.62,.18),(.080,0,.015),color=(.55,.37,.22)),box((.20,.62,.014),(.14,0,.055),color=(.25,.26,.28))]
# Bracket crosses the access opening at the far edge of this leaf-filled bay.
shapes += [box((.118,.018,.008),(0,-.22,.002),color=(.27,.28,.29)),box((.035,.018,.13),(.065,-.22,-.055))]
BODIES.append(body('roof_edge',GUTTER,shapes,mass=6.,fixed=True,color=(.75,.77,.78),friction=.45))
# Bench-safe timber legs, securing a local 1:1 roof section at reachable height.
for y in [-.24,.24]:
 BODIES.append(body('support_'+str(y).replace('-','n').replace('.','p'),[.38,y,TABLE+.045],[box((.06,.06,.09))],mass=.4,fixed=True,color=(.55,.37,.22)))
BUCKET=np.array([.15,.24,TABLE+.075]);BUCKET_R=.085
BODIES.append(body('bucket',BUCKET,[cylinder(BUCKET_R,.004,(0,0,-.073)),*ring(BUCKET_R,.15,.003,n=32)],mass=.30,color=(.18,.37,.56),friction=.8))
# Plastic gutter scoop with a thin bevelled mouth and open top, not a grasp tool.
SCOOP=np.array([.18,.02,TABLE+.006]);HANDLE=np.array([0,-.12,.070])
scoop=[]
# Curved, tapered mouth follows the gutter floor; a flat shovel cannot reach it.
for x0,x1 in zip(np.linspace(-.045,.045,19)[:-1],np.linspace(-.045,.045,19)[1:]):
 z0=.052-np.sqrt(.052**2-x0*x0);z1=.052-np.sqrt(.052**2-x1*x1)
 points=[[x0,-.055,z0],[x1,-.055,z1],[x0,.055,z0],[x1,.055,z1],[x0,-.055,z0+.003],[x1,-.055,z1+.003],[x0,.055,z0+.0003],[x1,.055,z1+.0003]]
 faces=[(0,2,3,1),(4,5,7,6),(0,1,5,4),(1,3,7,5),(2,6,7,3),(0,4,6,2)]
 scoop.append(dict(kind='mesh',points=points,faces=faces))
scoop += [box((.003,.11,.025),(-.0465,0,.040)),box((.003,.11,.025),(.0465,0,.040))]
# The rear wall follows the bowl: flat low corners would strike the gutter.
for x in np.linspace(-.0425,.0425,18):
 low=.052-np.sqrt(.052**2-(abs(x)+.0025)**2)+.003
 scoop.append(box((.005,.003,.055-low),(x,-.0565,(.055+low)/2)))
scoop += [box((.002,.024,.045),(x,-.055,.038)) for x in [-.008,.008]]
# A moulded hollow handle keeps the scoop balanced on its bowl.
handle_R=Rotation.from_euler('x',-20,degrees=True)
for size,local in [((.0015,.14,.020),(-.01325,0,0)),((.0015,.14,.020),(.01325,0,0)),((.025,.14,.0015),(0,0,-.00925)),((.025,.14,.0015),(0,0,.00925))]:
 scoop.append(box(size,HANDLE+handle_R.apply(local),quat=handle_R.as_quat()))
BODIES.append(body('scoop',SCOOP,scoop,mass=.085,color=(.88,.42,.12),friction=.7))
LEAVES=[];SEEDS=[]
# Four connected panels retain a shallow dry-leaf curl, about 8 mm at the tips.
angles=[-25.,-8.,8.,25.]
vertices=[np.zeros(3)]
for angle in angles:
 tangent=Rotation.from_euler('x',angle,degrees=True).apply([0,.015,0])
 vertices.append(vertices[-1]+tangent)
vertices=np.array(vertices);vertices[:,1]-=(vertices[0,1]+vertices[-1,1])/2;vertices[:,2]-=vertices[:,2].min()
for i in range(3):
 x=GUTTER[0]+[-.015,0,.015][i%3];y=.250+(i//3)*.040
 z=GUTTER[2]-np.sqrt(RADIUS**2-(x-GUTTER[0])**2)+.003+(i%3)*.001
 angle=[-20,15,40][i%3];R=Rotation.from_euler('z',angle,degrees=True);origin=np.array([x,y,z]);names=[]
 for j in range(4):
  name=f'leaf_{i}_{j}';names.append(name);width=[.013,.026,.026,.013][j]
  local=(vertices[j]+vertices[j+1])/2;Q=R*Rotation.from_euler('x',angles[j],degrees=True)
  BODIES.append(body(name,origin+R.apply(local),[box((width,.015-.00002,.0004))],quat=Q.as_quat(),mass=.0003/4,color=(.41+.02*(i%3),.32,.14),friction=.45,contact_offset=.00015,speculative_ccd=True))
  if j:
   a=BODIES[-2];b=BODIES[-1];anchor=origin+R.apply(vertices[j])
   Ra=Rotation.from_quat(a['quat']);Rb=Rotation.from_quat(b['quat'])
   JOINTS.append(joint(f'leaf_flex_{i}_{j}',names[-2],name,Ra.inv().apply(anchor-a['pos']),Rb.inv().apply(anchor-b['pos']),axes=bend_axes(.0002,.000003,120)))
 LEAVES.append(names)
finalize(BODIES,JOINTS)
