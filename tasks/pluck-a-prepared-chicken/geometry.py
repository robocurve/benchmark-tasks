"""Loose prepared carcass, yielding skin and independently retained feathers."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
C=np.array([.36,0,TABLE+.070]);N=9;GROUPS=4
AXIS=np.array([-.5,0,np.sqrt(.75)]);BQ=Rotation.align_vectors([AXIS],[[0,0,1]])[0].as_quat()
tray=[box((.42,.38,.006))]+[box((.42,.006,.035),(0,y,.020)) for y in [-.187,.187]]+[box((.006,.38,.035),(x,0,.020)) for x in [-.207,.207]]
BODIES.append(body('tray',[.36,0,TABLE+.003],tray,fixed=True,mass=.8,color=(.61,.68,.70),friction=.7))
# Eviscerated small broiler: body, folded wings and drumsticks are a loose
# rigid support. Only the working breast skin is deformable in this model.
parts=[sphere((.075,.115,.060),(0,0,0)),sphere((.027,.072,.032),(-.074,0,-.009)),sphere((.027,.072,.032),(.074,0,-.009))]
for sign in [-1,1]:
 parts+=[sphere((.029,.065,.030),(sign*.052,.100,-.015)),sphere((.012,.035,.014),(sign*.060,.155,-.020))]
parts+=[sphere((.025,.030,.024),(0,-.122,-.005))]
BODIES.append(body('carcass',C,parts,mass=1.05,color=(.91,.73,.59),friction=.6,contact_offset=.0001))
def breast_z(x,y):
 return .060*np.sqrt(max(.01,1-(x/.075)**2-(y/.115)**2))
def skin_shape(x,y,z):
 # A thin curved patch follows the actual breast ellipsoid. Its native convex
 # collision hull and rendered mesh share these vertices.
 points=[];faces=[];n=5
 for layer in [0.,-.001]:
  for v in np.linspace(y-.01475,y+.01475,n):
   for u in np.linspace(x-.01025,x+.01025,n):
    points.append([u-x,v-y,breast_z(u,v)+.0005+layer-z])
 for layer in [0,1]:
  for j in range(n-1):
   for i in range(n-1):
    a=layer*n*n+j*n+i;f=(a,a+1,a+n+1,a+n)
    faces.append(f if not layer else tuple(reversed(f)))
 edges=list(range(n))+[j*n+n-1 for j in range(1,n)]+list(range(n*n-2,n*(n-1)-1,-1))+[j*n for j in range(n-2,0,-1)]
 for a,b in zip(edges,edges[1:]+edges[:1]):faces.append((a,b,b+n*n,a+n*n))
 return dict(kind='mesh',points=points,faces=faces,pos=[0,0,0])
SKIN=[];TIPS=[];ROOTS=[]
for row in range(GROUPS):
 y=(row-1.5)*.030
 for col in range(3):
  x=(col-1)*.021;z=breast_z(x,y);name=f'skin_{row}_{col}';SKIN.append(name)
  BODIES.append(body(name,C+[x,y,z],[skin_shape(x,y,z)],mass=.0015,color=(.96,.80,.65),friction=.6,contact_offset=.0001,exclude=['carcass']))
  axes={a:dict(limits=(-.006,.006),stiffness=250.,damping=.45) for a in ['transX','transY','transZ']}
  axes.update({a:dict(limits=(-20,20),stiffness=.04,damping=.002) for a in ['rotX','rotY','rotZ']})
  JOINTS.append(joint('skin_bond_'+name,'carcass',name,[x,y,z],[0,0,0],axes=axes,break_force=2.,break_torque=.025))
for i in range(N):
 group=i//3;off=i%3-1;y=(group-1.5)*.030+off*.005;root=C+[0,y,breast_z(0,y)+.001];ROOTS.append(root)
 exclusions=SKIN+['carcass'];tip=root+AXIS*.020;TIPS.append(tip)
 BODIES.append(body(f'quill_{i}',root+AXIS*.002,[cylinder(.0006,.016)],quat=BQ,mass=.00005,color=(.90,.85,.70),friction=.65,exclude=exclusions,contact_offset=.00005))
 # The upper shaft and feather vane bend relative to the embedded calamus.
 shapes=[cylinder(.0006,.020)]
 # Feather barbs are visual-only, attached to the bending shaft. Their soft
 # collapse is not resolved; finger contact and extraction use the quill.
 shapes += [dict(sphere((.00025,.0033,.010),(0,0,.004),(.92,.91,.84)),collision=False)]
 BODIES.append(body(f'feather_{i}',tip,shapes,quat=BQ,mass=.00012,color=(.94,.92,.84),friction=.65,exclude=exclusions,contact_offset=.00005))
 axes=fixed_axes();axes['transZ']=dict(limits=(0.,.020),stiffness=2000.,damping=.02,max_force=.35)
 JOINTS.append(joint(f'follicle_{i}',f'skin_{group}_1',f'quill_{i}',[0,off*.005,breast_z(0,y)-breast_z(0,(group-1.5)*.030)+.001]-AXIS*.006,[0,0,-.008],q0=BQ,axes=axes))
 JOINTS.append(joint(f'shaft_{i}',f'quill_{i}',f'feather_{i}',[0,0,.008],[0,0,-.010],axes=bend_axes(.002,.00003,limit=45),break_force=1.8,break_torque=.012))
finalize(BODIES,JOINTS)
