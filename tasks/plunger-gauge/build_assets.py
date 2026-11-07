"""Self-contained brush and machined chip tray. Regenerate with usd-core."""
from pathlib import Path
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf
import math
import numpy as np

HERE=Path(__file__).resolve().parent/'assets'

def stage(name,rigid=False):
 s=Usd.Stage.CreateNew(str(HERE/(name+'.usdc')));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z')
 r=UsdGeom.Xform.Define(s,'/Asset');s.SetDefaultPrim(r.GetPrim())
 if rigid:UsdPhysics.RigidBodyAPI.Apply(r.GetPrim());UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(.08)
 return s

def mat(s,name,color,metal=0.,rough=.4):
 m=UsdShade.Material.Define(s,'/Asset/Materials/'+name);sh=UsdShade.Shader.Define(s,m.GetPath().AppendChild('Surface'));sh.CreateIdAttr('UsdPreviewSurface')
 sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));sh.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal);sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(rough);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return m

def box(s,name,pos,size,m,collision=True):
 c=UsdGeom.Cube.Define(s,'/Asset/'+name);c.CreateSizeAttr(1.);c.AddTranslateOp().Set(Gf.Vec3d(*pos));c.AddScaleOp().Set(Gf.Vec3f(*size));UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(m)
 if collision:UsdPhysics.CollisionAPI.Apply(c.GetPrim())
 return c

def cylinder(s,name,pos,radius,height,m,collision=False):
 c=UsdGeom.Cylinder.Define(s,'/Asset/'+name);c.CreateRadiusAttr(radius);c.CreateHeightAttr(height);c.AddTranslateOp().Set(Gf.Vec3d(*pos));UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(m)
 if collision:UsdPhysics.CollisionAPI.Apply(c.GetPrim())
 return c

def ring(s,name,center,inner,outer,height,m):
 """Visual bore mesh with compound convex collision sectors."""
 n=48;verts=[]
 for z in (-height/2,height/2):
  for r in (outer,inner):
   verts += [(center[0]+r*math.cos(2*math.pi*i/n),center[1]+r*math.sin(2*math.pi*i/n),center[2]+z) for i in range(n)]
 faces=[]
 for i in range(n):
  j=(i+1)%n
  faces += [(i,j,2*n+j,2*n+i),(n+j,n+i,3*n+i,3*n+j),(j,i,n+i,n+j),(2*n+i,2*n+j,3*n+j,3*n+i)]
 mesh=UsdGeom.Mesh.Define(s,'/Asset/'+name);mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in verts]);mesh.CreateFaceVertexCountsAttr([4]*len(faces));mesh.CreateFaceVertexIndicesAttr([v for f in faces for v in f]);mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
 UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(m)
 # Convex sector envelopes avoid PhysX triangle-contact material warnings.
 segments=32
 for k in range(segments):
  angle=2*math.pi*k/segments
  radius=(inner+outer)/2
  pos=(center[0]+radius*math.cos(angle),center[1]+radius*math.sin(angle),center[2])
  c=box(s,name+'_collider_'+str(k),pos,(outer-inner,2*outer*math.tan(math.pi/segments),height),m)
  c.AddRotateZOp().Set(math.degrees(angle))
  ops=c.GetOrderedXformOps();c.SetXformOpOrder([ops[0],ops[2],ops[1]])
  c.CreateVisibilityAttr('invisible')
 return mesh

def build():
 s=stage('punch',True);steel=mat(s,'steel',(.55,.60,.64),.85,.25);rubber=mat(s,'grip',(.10,.27,.32),0,.65)
 cylinder(s,'shaft',(0,0,-.020),.0035,.080,steel,True)
 box(s,'grip',(0,0,0),(.018,.024,.030),rubber)
 cylinder(s,'cap',(0,0,.019),.013,.006,steel,True)
 s.GetRootLayer().Save()
 s=stage('punch_stand');metal=mat(s,'powdercoat',(.16,.24,.29),.3,.45);steel=mat(s,'rim',(.50,.55,.59),.8,.3)
 box(s,'foot',(0,0,.005),(.07,.07,.010),metal)
 ring(s,'shaft_guide',(0,0,.0325),.0085,.022,.045,steel)
 # The shaft bottom rests on the solid pedestal at +10 mm, root at +70 mm.
 s.GetRootLayer().Save()
 s=stage('fixture');metal=mat(s,'anodized',(.20,.29,.35),.6,.32);steel=mat(s,'bushings',(.56,.60,.64),.85,.25);yellow=mat(s,'well_mark',(.95,.61,.08),.15,.5);black=mat(s,'fasteners',(.025,.03,.035),.5,.4)
 # Fixture origin at (.35,.09,tabletop), wells at local y=+/-50 mm.
 box(s,'base',(0,0,.005),(.12,.23,.010),metal)
 for i,y in enumerate((-.05,.05)):
  ring(s,'well_'+str(i),(0,y,.030),.0075,.022,.040,steel)
  ring(s,'mouth_'+str(i),(0,y,.0505),.0075,.024,.001,yellow)
 # Open-sided bridge body leaves the cylindrical wells visible from the camera.
 for sign in (-1,1):box(s,'side_'+str(sign).replace('-','n'),(sign*.044,0,.026),(.016,.22,.032),metal)
 for i,(x,y) in enumerate(((-.044,-.095),(.044,-.095),(-.044,.095),(.044,.095))):cylinder(s,'bolt_'+str(i),(x,y,.043),.003,.002,black)
 s.GetRootLayer().Save()
 for i,y in enumerate((.04,.14)):
  s=stage('plunger_'+str(i),True);steel=mat(s,'pin',(.45,.50,.55),.85,.25);cap=mat(s,'tip',(.8,.24,.08),.25,.4)
  cylinder(s,'pin',(0,0,0),.005,.012,steel,True)
  cylinder(s,'contact_face',(0,0,.0061),.0047,.0002,cap,False)
  UsdPhysics.MassAPI.Apply(s.GetDefaultPrim()).GetMassAttr().Set(.03)
  # Maximal-coordinate joint to world: the pin remains a normal RigidObject.
  joint=UsdPhysics.PrismaticJoint.Define(s,'/Asset/guide')
  joint.CreateBody1Rel().SetTargets(['/Asset']);joint.CreateAxisAttr('Z')
  joint.CreateLocalPos0Attr(Gf.Vec3f(.35,y,.7475+.030))
  joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));joint.CreateLowerLimitAttr(-.010);joint.CreateUpperLimitAttr(0.)
  drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(500.);drive.CreateDampingAttr(2.);drive.CreateMaxForceAttr(20.);drive.CreateTargetPositionAttr(0.)
  s.GetRootLayer().customLayerData={'joint':'World Z guide, -10..0 mm travel, 500 N/m spring, 2 N s/m damping. Authored absolute world anchor matches fixed task setup; reset jitter must be zero.'}
  s.GetRootLayer().Save()
if __name__=='__main__':build()
