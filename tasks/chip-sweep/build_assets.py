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
 s=stage('rake',True);wood=mat(s,'maple',(.42,.23,.10),0,.55);rubber=mat(s,'grip',(.04,.12,.15),0,.65);steel=mat(s,'steel',(.5,.55,.58),.8,.28)
 box(s,'handle',(-.0175,0,.014),(.235,.017,.012),wood)
 box(s,'grip_sleeve',(-.10,0,.050),(.025,.020,.050),rubber)
 box(s,'neck',(.10,0,-.003),(.006,.006,.028),steel)
 box(s,'T_foot',(.10,0,-.0185),(.012,.020,.003),steel)
 box(s,'head',(.10,0,.009),(.025,.025,.008),steel)
 s.GetRootLayer().customLayerData={'fidelity':'Rigid T-slot rake. Twenty mm T-foot and six mm neck enter the undercut from an open channel end.'};s.GetRootLayer().Save()
 s=stage('chip_tray');metal=mat(s,'anodized',(.20,.28,.33),.65,.33);bright=mat(s,'edges',(.54,.60,.63),.8,.25);green=mat(s,'collection',(.08,.36,.28),.25,.4);black=mat(s,'screws',(.035,.04,.045),.5,.4)
 # Origin at world (.35,0,tabletop). One coplanar floor prevents a step at the pan mouth.
 box(s,'floor',(0,0,.006),(.35,.19,.012),metal)
 box(s,'collection_inlay',(-.105,0,.0121),(.12,.17,.0002),green,False)
 for side in (-1,1):
  box(s,'rail_'+str(side).replace('-','n'),(0,side*.0975,.018),(.35,.005,.036),bright)
 # Open-ended undercut grooves require the T-head to enter from the rear.
 for i,y in enumerate((-.055,0.,.055)):
  for j,sign in enumerate((-1,1)):
   box(s,f'ledge_{i}_{j}',(.075,y+sign*.0115,.029),(.20,.009,.010),metal)
 # Full-height strips outside the 32 mm lower chambers support the ledges.
 intervals=[(-.095,-.071),(-.039,-.016),(.016,.039),(.071,.095)]
 for i,(lo,hi) in enumerate(intervals):box(s,'rib_'+str(i),(.075,(lo+hi)/2,.023),(.20,hi-lo,.022),metal)
 box(s,'front_lip',(-.1775,0,.011),(.005,.20,.022),bright)
 for i,(x,y) in enumerate(((-.157,-.084),(-.157,.084),(.157,-.084),(.157,.084))):cylinder(s,'screw_'+str(i),(x,y,.0127),.003,.0012,black)
 # Visual reference marks are outside the sweep lane.
 for i in range(8):
  for sign in (-1,1):box(s,f'tick_{i}_{sign}'.replace('-','n'),(-.13+i*.035,sign*.089,.0123),(.0008,.007,.0004),bright,False)
 s.GetRootLayer().Save()
if __name__=='__main__':build()
