"""Original procedural CAD. Spring ring is a continuous articulated steel spiral.
The visible vise clamps one lower-coil segment; a contact-only sliding splitter
has a permanently enabled return spring. Neither fixture attaches to the key.
"""
from pathlib import Path
import math
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
import geometry as G
HERE=Path(__file__).resolve().parent
ROOT='/hardware'
def quat(r):
 x,y,z,w=Rotation.from_matrix(r).as_quat();return Gf.Quatf(float(w),float(x),float(y),float(z))
def body(st,path,p,r,mass,inertia,com=(0,0,0)):
 b=UsdGeom.Xform.Define(st,path);b.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)));b.AddOrientOp().Set(quat(r));UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
 m=UsdPhysics.MassAPI.Apply(b.GetPrim());m.CreateMassAttr(mass);m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float,com)));m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,inertia)));m.CreatePrincipalAxesAttr(Gf.Quatf(1,0,0,0));return b

def material(st,name,rgb,metal=0):
 m=UsdShade.Material.Define(st,f'{ROOT}/looks/{name}');sh=UsdShade.Shader.Define(st,f'{m.GetPath()}/shader');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb));sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.35 if metal else .7);sh.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return m

def finish(prim,mat,collision=True):
 UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
 if collision:
  UsdPhysics.CollisionAPI.Apply(prim);prim.AddAppliedSchema('PhysxCollisionAPI');prim.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(G.CONTACT_OFFSET);prim.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(G.REST_OFFSET)

def box(st,path,p,size,mat,rot=None,collision=True):
 b=UsdGeom.Cube.Define(st,path);b.CreateSizeAttr(1.);b.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)))
 if rot is not None:b.AddOrientOp().Set(quat(rot))
 b.AddScaleOp().Set(Gf.Vec3f(*map(float,size)));finish(b.GetPrim(),mat,collision)

def capsule(st,path,a,b,r,mat):
 a,b=np.asarray(a),np.asarray(b);d=b-a;n=np.linalg.norm(d);x=d/n;ref=np.array([0,0,1.]) if abs(x[2])<.95 else np.array([0,1.,0]);y=np.cross(ref,x);y/=np.linalg.norm(y)
 c=UsdGeom.Capsule.Define(st,path);c.CreateAxisAttr('X');c.CreateRadiusAttr(r);c.CreateHeightAttr(float(n));c.AddTranslateOp().Set(Gf.Vec3d(*map(float,(a+b)/2)));c.AddOrientOp().Set(quat(np.stack([x,y,np.cross(x,y)],1)));finish(c.GetPrim(),mat)

def prism(st,path,poly,thickness,mat):
 # Polygon lives in XZ; extrusion is along Y. Convex pieces only.
 p=np.array([[x,y,z] for y in (-thickness/2,thickness/2) for x,z in poly]);n=len(poly)
 faces=[list(range(n-1,-1,-1)),list(range(n,2*n))]+[[j,(j+1)%n,(j+1)%n+n,j+n] for j in range(n)]
 m=UsdGeom.Mesh.Define(st,path);m.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in p]);m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr(sum(faces,[]));m.CreateSubdivisionSchemeAttr('none');finish(m.GetPrim(),mat);UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')

def joint(st,name,b0,p0,r0,b1,p1,r1,p,frame,k,d,linear=False,limits=(-25,25)):
 j=(UsdPhysics.PrismaticJoint if linear else UsdPhysics.RevoluteJoint).Define(st,f'{ROOT}/joints/{name}');j.CreateBody0Rel().SetTargets([b0]);j.CreateBody1Rel().SetTargets([b1])
 for pp,rr,pa,ra in ((p0,r0,j.CreateLocalPos0Attr,j.CreateLocalRot0Attr),(p1,r1,j.CreateLocalPos1Attr,j.CreateLocalRot1Attr)):
  pa(Gf.Vec3f(*map(float,rr.T@(p-pp))));ra(quat(rr.T@frame))
 j.CreateAxisAttr('X');j.CreateLowerLimitAttr(limits[0]);j.CreateUpperLimitAttr(limits[1]);j.GetPrim().AddAppliedSchema('PhysxJointAPI');j.GetPrim().CreateAttribute('physxJoint:maxJointVelocity',Sdf.ValueTypeNames.Float).Set(5. if linear else 180.)
 dr=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear' if linear else 'angular');dr.CreateStiffnessAttr(k if linear else k*math.pi/180);dr.CreateDampingAttr(d if linear else d*math.pi/180);dr.CreateTargetPositionAttr(0.);dr.CreateMaxForceAttr(30.)
 UsdPhysics.FilteredPairsAPI.Apply(st.GetPrimAtPath(b1)).CreateFilteredPairsRel().AddTarget(b0)

def solid_mass_properties(st,path):
 # Exact primitive-volume mass and principal inertia for the handwheel.
 # Steel 7900 kg/m3, anodized aluminum 2700, polymer grips 1200.
 cache=UsdGeom.XformCache();parts=[];root=st.GetPrimAtPath(path)
 for prim in Usd.PrimRange(root):
  material_name=str(prim.GetRelationship('material:binding').GetTargets()[0]).rsplit('/',1)[-1] if prim.HasRelationship('material:binding') else ''
  density={'steel':7900.,'blue':2700.,'orange':1200.}.get(material_name)
  if density is None:continue
  mat=np.array(cache.GetLocalToWorldTransform(prim)*cache.GetLocalToWorldTransform(root).GetInverse());linear=mat[:3,:3].T;pos=mat[3,:3]
  scales=np.linalg.norm(linear,axis=0);rot=linear/scales
  if prim.IsA(UsdGeom.Cube):
   lengths=scales*UsdGeom.Cube(prim).GetSizeAttr().Get();mass=density*np.prod(lengths)
   inertia=np.diag(mass/12*(np.sum(lengths**2)-lengths**2))
  elif prim.IsA(UsdGeom.Capsule):
   shape=UsdGeom.Capsule(prim);radius=shape.GetRadiusAttr().Get();height=shape.GetHeightAttr().Get();axis='XYZ'.index(shape.GetAxisAttr().Get())
   mc=density*math.pi*radius**2*height;ms=density*4*math.pi*radius**3/3;mass=mc+ms
   axial=.5*mc*radius**2+.4*ms*radius**2
   transverse=mc*(3*radius**2+height**2)/12+ms*(.4*radius**2+3*height*radius/8+height**2/4)
   values=np.full(3,transverse);values[axis]=axial;inertia=np.diag(values)
  else:continue
  parts.append((mass,pos,rot@inertia@rot.T))
 mass=sum(m for m,_,_ in parts);com=sum(m*p for m,p,_ in parts)/mass
 inertia=sum(I+m*(np.dot(p-com,p-com)*np.eye(3)-np.outer(p-com,p-com))for m,p,I in parts)
 values,axes=np.linalg.eigh(inertia)
 if np.linalg.det(axes)<0:axes[:,0]*=-1
 api=UsdPhysics.MassAPI(root);api.CreateMassAttr(float(mass));api.CreateCenterOfMassAttr(Gf.Vec3f(*map(float,com)));api.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,values)));api.CreatePrincipalAxesAttr(quat(axes))
 return {'mass_kg':float(mass),'com_m':com.tolist(),'principal_inertia_kg_m2':values.tolist()}

def stage(name):
 st=Usd.Stage.CreateNew(str(HERE/'assets'/name));UsdGeom.SetStageUpAxis(st,'Z');UsdGeom.SetStageMetersPerUnit(st,1.);root=UsdGeom.Xform.Define(st,ROOT).GetPrim();st.SetDefaultPrim(root)
 mats={n:material(st,n,c,m) for n,c,m in [('steel',(.48,.52,.56),.7),('brass',(.72,.46,.12),.65),('blue',(.045,.20,.30),.15),('dark',(.055,.065,.075),.1),('orange',(.83,.27,.035),0),('red',(.9,.025,.015),0),('green',(.04,.75,.12),0),('white',(.9,.9,.9),0)]}
 return st,root,mats

def build_ring():
 st,root,m=stage('ring.usda');UsdPhysics.ArticulationRootAPI.Apply(root);root.AddAppliedSchema('PhysxArticulationAPI');root.CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(True)
 I=np.eye(3);O=np.zeros(3);pts=G.ring_points();c=G.CLAMP_INDEX;axis=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
 body(st,ROOT+'/base',O,I,2.,(.02,.02,.02));j=UsdPhysics.FixedJoint.Define(st,ROOT+'/joints/world_fixed');j.CreateBody1Rel().SetTargets([ROOT+'/base'])
 h=G.CENTER[2]-G.TABLE_Z
 box(st,ROOT+'/base/foot',G.W.T@np.array([.130,0,-h+.008]),(.30,.20,.016),m['blue'],rot=G.W.T)
 box(st,ROOT+'/base/splitter_foot',G.W.T@np.array([.235,.185,-h+.008]),(.070,.240,.016),m['blue'],rot=G.W.T)
 box(st,ROOT+'/base/upright',G.W.T@np.array([.250,0,-h/2]),(.025,.030,h-.016),m['blue'],rot=G.W.T)
 capsule(st,ROOT+'/base/bearing',(0,0,-.267),(0,0,-.233),.015,m['dark'])
 # Unpowered rotary vise and balanced hand crank. The key has no joint.
 body(st,ROOT+'/rotor',O,I,.25,(.0008,.0008,.0014),com=(0,0,-.150))
 joint(st,'turn',ROOT+'/base',O,I,ROOT+'/rotor',O,I,O,axis,0.,G.TURN_DAMPING,limits=(-900,900))
 capsule(st,ROOT+'/rotor/shaft',(0,0,-.265),(0,0,-.015),.006,m['steel'])
 for n in range(4):
  q=n*math.pi/2;rad=G.rz(q)
  box(st,ROOT+f'/rotor/spoke{n}',rad@np.array([G.CRANK_R/2,0,-.214]),(G.CRANK_R,.012,.008),m['blue'],rot=rad)
 # Visible neck from the spindle to the small lower-coil clamp, on its far side.
 p=pts[c]
 capsule(st,ROOT+'/rotor/clamp_stalk',(0,0,-.018),(p[0],p[1],p[2]-.003),.002,m['steel'])
 box(st,ROOT+'/rotor/jaw_lower',p+[0,0,-.0025],(.005,.005,.0037),m['blue'])
 box(st,ROOT+'/rotor/jaw_inner',p+[.0027,0,-.001],(.003,.004,.0017),m['blue'])
 for n in range(4):
  q=n*math.pi/2;kp=G.rz(q)@np.array([G.CRANK_R,0,G.CRANK_Z]);name='knob' if n==0 else f'knob{n}'
  body(st,ROOT+'/'+name,kp,I,.010,(2e-7,2e-7,1e-7))
  joint(st,f'knob_spin{n}',ROOT+'/rotor',O,I,ROOT+'/'+name,kp,I,kp,axis,0.,.000002,limits=(-18000,18000))
  capsule(st,ROOT+f'/rotor/knob_pin{n}',kp+[0,0,-.214-G.CRANK_Z],kp,.0025,m['steel'])
  capsule(st,ROOT+'/'+name+'/grip',(0,0,-.010),(0,0,.010),.006,m['orange'])
 for branch,indices in [('a',list(range(c-1,-1,-1))),('b',list(range(c,G.N)))]:
  prev=ROOT+'/rotor';pp=O;pr=I
  for jidx,i in enumerate(indices):
   a,b=(pts[i+1],pts[i]) if branch=='a' else (pts[i],pts[i+1]);x=(b-a)/np.linalg.norm(b-a);y=np.cross([0,0,1.],x);y/=np.linalg.norm(y);r=np.stack([x,y,np.cross(x,y)],1);name=f'{ROOT}/wire{i:02d}'
   length=float(np.linalg.norm(b-a));mass=7900*math.pi*G.WIRE_R**2*length
   body(st,name,a,r,mass,(max(mass*G.WIRE_R**2/2,1e-11),max(mass*length**2/12,1e-11),max(mass*length**2/12,1e-11)),com=(length/2,0,0))
   radial=np.array([a[0],a[1],0]);radial/=np.linalg.norm(radial);frame=np.stack([radial,np.cross([0,0,1.],radial),[0,0,1.]],1)
   joint(st,f'flex{i:02d}',prev,pp,pr,name,a,r,a,frame,G.SPRING_K,G.SPRING_DAMPING)
   curve=G.segment_curve(i)
   if branch=='a':curve=curve[::-1]
   curve=(curve-a)@r
   for ci in range(len(curve)-1):
    color='green' if i==0 and ci==len(curve)-2 else 'red' if i==G.N-1 and ci==len(curve)-2 else 'steel'
    capsule(st,f'{name}/steel{ci}',curve[ci],curve[ci+1],G.WIRE_R,m[color])
   prev,pp,pr=name,a,r
 # A supported tapered catch enters the outside cusp behind the exposed end.
 # There is no authored inter-turn clearance. Its 60 um leading edge widens
 # to a 1.4 mm heel before the axial slide lifts the upper turn. All motion is
 # contact driven; the lower shoe and permanently enabled springs are physical.
 aa=G.OPENER_ANGLE;rad=np.array([math.cos(aa),math.sin(aa),0.]);rot=G.rz(aa)
 zz=G.OPENER_Z
 p=rad*(G.R+G.OPENER_RETRACT)+[0,0,zz]
 lift=rot.T@G.W.T@np.array([0,0,G.OPENER_LIFT])
 body(st,ROOT+'/carriage',p,rot,.030,(1e-5,1e-5,1e-5),com=lift+np.array([G.OPENER_HANDLE-.02,0,-.010]))
 joint(st,'retract_slide',ROOT+'/base',O,I,ROOT+'/carriage',p,rot,p,rot,5.,1.,linear=True,limits=(-G.OPENER_RETRACT-.0005,0))
 box(st,ROOT+'/carriage/slide_block',lift+np.array([G.OPENER_HANDLE-.02,0,-.010]),(.025,.020,.010),m['blue'])
 body(st,ROOT+'/splitter',p,rot,.025,(1e-5,1e-5,1e-5),com=lift+np.array([G.OPENER_HANDLE/2,0,0]))
 joint(st,'splitter_slide',ROOT+'/carriage',p,rot,ROOT+'/splitter',p,rot,p,axis,35.,1.,linear=True,limits=(0,G.OPENER_STROKE))
 prism(st,ROOT+'/splitter/tooth',[(-.002,-.00003),(.003,-.0007),(.003,.0007),(-.002,.00003)],.0012,m['steel'])
 box(st,ROOT+'/splitter/tooth_neck',(.0035,0,0),(.001,.0012,.0014),m['steel'])
 # Fixed to the radial carriage, independent of the lifting blade.
 box(st,ROOT+'/carriage/backing_shoe',(.0005,0,-.00258),(.005,.0016,.001),m['steel'])
 box(st,ROOT+'/carriage/shoe_neck',(.004,0,-.004),(.002,.002,.004),m['steel'])
 box(st,ROOT+'/carriage/shoe_arm',((G.OPENER_HANDLE+.004)/2,0,-.006),(G.OPENER_HANDLE-.004,.004,.003),m['steel'])
 if G.OPENER_LIFT>0:capsule(st,ROOT+'/splitter/riser',(.004,0,0),lift+np.array([.004,0,0]),.001,m['steel'])
 box(st,ROOT+'/splitter/arm',lift+np.array([(G.OPENER_HANDLE+.004)/2,0,0]),(G.OPENER_HANDLE-.004,.006,.003),m['steel'])
 capsule(st,ROOT+'/splitter/grip_standoff',lift+np.array([G.OPENER_HANDLE,0,0]),lift+np.array([G.OPENER_HANDLE,0,G.OPENER_GRIP_Z]),.003,m['steel'])
 box(st,ROOT+'/splitter/grip',lift+np.array([G.OPENER_HANDLE,0,G.OPENER_GRIP_Z]),(.016,.016,.016),m['orange'])
 support=p+rot@(lift+np.array([G.OPENER_HANDLE,0,-.012]));world=G.W@support
 box(st,ROOT+'/base/splitter_post',G.W.T@np.array([.250,world[1],(-h+world[2])/2]),(.010,.014,h+world[2]),m['dark'],rot=G.W.T)
 box(st,ROOT+'/base/splitter_bridge',G.W.T@np.array([(.250+world[0]) / 2,world[1],world[2]]),(.250-world[0],.014,.010),m['blue'],rot=G.W.T)
 capsule(st,ROOT+'/base/withdrawal_rail',support-rad*(G.OPENER_RETRACT-.03)+np.array([0,0,-.002]),support+rad*.02+np.array([0,0,-.002]),.004,m['blue'])
 capsule(st,ROOT+'/carriage/slide_rod',lift+np.array([G.OPENER_HANDLE-.02,0,-.014]),lift+np.array([G.OPENER_HANDLE+.05,0,-.014]),.002,m['steel'])
 box(st,ROOT+'/base/splitter_guide',support+np.array([0,0,-.008]),(.025,.025,.012),m['blue'])
 # Fixed photographic scales: 1 mm ticks, 5 mm major ticks, outside contact
 # workspace. Front scale follows the ring's horizontal/vertical axes. Edge
 # scale follows its axial direction. Scale bars are noncolliding ink/plates.
 for name,origin,axis1,axis2,count in [
  ('front_h',np.array([-.022,-.024,0.]),np.array([.001,0,0]),np.array([0,1,0]),44),
  ('front_v',np.array([.024,-.022,0.]),np.array([0,.001,0]),np.array([1,0,0]),44),
  ('edge',np.array([.026,0,-.008]),np.array([0,0,.001]),np.array([1,0,0]),16)]:
  for tick in range(count+1):
   p=origin+tick*axis1
   size=np.abs(axis1)*.12+np.abs(axis2)*(.003 if tick%5==0 else .0015)+np.abs(np.cross(axis1,axis2))*.1
   box(st,ROOT+f'/base/scale_{name}_{tick:02d}',p,size,m['dark'],collision=False)
 # A visible 1.2 g trim slug balances the clamp and the open spiral's small
 # static first moment. Its position is determined from authored mass only.
 props=solid_mass_properties(st,ROOT+'/rotor');moment=props['mass_kg']*np.array(props['com_m'])
 cache=UsdGeom.XformCache()
 for i in range(G.N):
  prim=st.GetPrimAtPath(ROOT+f'/wire{i:02d}');api=UsdPhysics.MassAPI(prim);mat=np.array(cache.GetLocalToWorldTransform(prim));com=np.array(api.GetCenterOfMassAttr().Get())@mat[:3,:3]+mat[3,:3]
  moment+=api.GetMassAttr().Get()*com
 trim_mass=.0012;side=(trim_mass/7900.)**(1/3)
 box(st,ROOT+'/rotor/balance_slug',(-moment[0]/trim_mass,-moment[1]/trim_mass,-.207),(side,side,side),m['steel'])
 for name in ['rotor','knob','knob1','knob2','knob3']:solid_mass_properties(st,ROOT+'/'+name)
 st.GetRootLayer().Save()

def build_key():
 st,root,m=stage('key.usda');body(st,ROOT+'/key',np.zeros(3),np.eye(3),G.KEY_MASS,(1e-7,2e-6,2e-6),com=(.020,0,0))
 for i in range(32):
  a,b=2*math.pi*np.array([i,i+1])/32
  poly=[(G.HOLE_R*math.cos(a),G.HOLE_R*math.sin(a)),(G.BOW_R*math.cos(a),G.BOW_R*math.sin(a)),(G.BOW_R*math.cos(b),G.BOW_R*math.sin(b)),(G.HOLE_R*math.cos(b),G.HOLE_R*math.sin(b))]
  prism(st,f'{ROOT}/key/bow{i:02d}',poly,.002,m['brass'])
 # A normal flat blade and shoulder; the hole is physically empty.
 prism(st,ROOT+'/key/shoulder',[(.004,-.004),(.008,-.009),(.016,-.008),(.018,-.004),(.018,.004),(.016,.008),(.008,.009),(.004,.004)],.002,m['brass'])
 box(st,ROOT+'/key/blade',(.029,0,.0005),(.026,.002,.006),m['brass'])
 for i,dep in enumerate((.0015,.0005,.002,.001,.0018)):
  box(st,f'{ROOT}/key/tooth{i}',(.019+i*.0045,0,-.0025-dep/2),(.0024,.002,dep),m['brass'])
 st.GetRootLayer().Save()

def build_stand():
 st,root,m=stage('stand.usda');body(st,ROOT+'/stand',np.zeros(3),np.eye(3),1.,(.01,.01,.01))
 h=G.KEY_START[2]-G.TABLE_Z
 box(st,ROOT+'/stand/base',(-.015,0,-h+.006),(.075,.060,.012),m['blue'])
 box(st,ROOT+'/stand/column',(-.014,0,-h/2),(.014,.012,h-.012),m['dark'])
 box(st,ROOT+'/stand/rest',(-.015,0,-.004),(.014,.020,.006),m['blue'])
 st.GetRootLayer().Save()

if __name__=='__main__':
 (HERE/'assets').mkdir(exist_ok=True);build_ring();build_key();build_stand();print('Built spring spiral, hollow key and workholder.')
