"""Self-contained brush and machined chip tray. Regenerate with usd-core."""
from pathlib import Path
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf
import math

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

def build():
 s=stage('lifter',True);steel=mat(s,'steel',(.45,.52,.56),.85,.26);rubber=mat(s,'grip',(.08,.19,.23),0,.6);yellow=mat(s,'mark',(.95,.60,.07),.1,.45)
 box(s,'handle',(-.06,0,.014),(.15,.018,.012),steel)
 box(s,'grip_sleeve',(-.1,0,.016),(.035,.024,.018),rubber)
 box(s,'crossbar',(.018,0,.0075),(.016,.080,.015),steel)
 for i,y in enumerate((-.03,.03)):
  box(s,'tine_'+str(i),(.095,y,.0025),(.15,.012,.005),steel)
  box(s,'tip_mark_'+str(i),(.155,y,.0052),(.020,.008,.0004),yellow,False)
 s.GetRootLayer().Save()
 s=stage('pallet',True);steel=mat(s,'pallet_plate',(.35,.43,.49),.8,.3);aluminum=mat(s,'blank',(.66,.70,.73),.8,.25);black=mat(s,'bolts',(.035,.04,.045),.5,.35)
 UsdPhysics.MassAPI.Apply(s.GetDefaultPrim()).GetMassAttr().Set(.12)
 box(s,'plate',(0,0,0),(.12,.16,.006),steel)
 for i,(x,y) in enumerate(((-.045,-.07),(-.045,.07),(.045,-.07),(.045,.07))):
  box(s,'foot_'+str(i),(x,y,-.013),(.018,.014,.020),steel)
  cylinder(s,'bolt_'+str(i),(x,y,.0038),.003,.0016,black)
 cylinder(s,'workpiece',(0,0,.009),.020,.012,aluminum,True)
 # Face grooves and a center dimple make this read as a machined blank on a pallet.
 cylinder(s,'center_dimple',(0,0,.0151),.003,.0002,black)
 for i,x in enumerate((-.012,.012)):box(s,'face_groove_'+str(i),(x,0,.0152),(.001,.027,.0004),steel,False)
 s.GetRootLayer().Save()
 s=stage('rests');steel=mat(s,'anodized',(.16,.28,.33),.6,.35);green=mat(s,'seat',(.08,.42,.29),.2,.5);black=mat(s,'bolts',(.035,.04,.045),.5,.4)
 # Origin at the destination xy. No floor closes the tine withdrawal channels.
 for i,y in enumerate((-.07,.07)):
  box(s,'rail_'+str(i),(0,y,.0075),(.15,.018,.015),steel)
  box(s,'contact_mark_'+str(i),(0,y,.0151),(.12,.014,.0002),green,False)
  for j,x in enumerate((-.065,.065)):cylinder(s,f'bolt_{i}_{j}',(x,y,.0158),.0025,.0016,black)
 s.GetRootLayer().Save()
if __name__=='__main__':build()
