"""Original full-size rigid bench riser, SI units; no moving mechanism."""
from pathlib import Path
import json
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,Gf,Sdf
HERE=Path(__file__).resolve().parent
stage=Usd.Stage.CreateNew(str(HERE/'stand.usda'))
root=UsdGeom.Xform.Define(stage,'/RecoveryStand');stage.SetDefaultPrim(root.GetPrim())
UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
box=UsdGeom.Cube.Define(stage,'/RecoveryStand/Plinth');box.CreateSizeAttr(1.)
box.AddTranslateOp().Set(Gf.Vec3d(0.,0.,.03375));box.AddScaleOp().Set(Gf.Vec3f(.100,.140,.0675))
UsdPhysics.CollisionAPI.Apply(box.GetPrim())
material=UsdShade.Material.Define(stage,'/RecoveryStand/Material')
shader=UsdShade.Shader.Define(stage,'/RecoveryStand/Material/Surface');shader.CreateIdAttr('UsdPreviewSurface')
shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.52,.58,.57))
shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.8)
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
UsdShade.MaterialBindingAPI.Apply(box.GetPrim()).Bind(material);stage.GetRootLayer().Save()
(HERE/'stand.json').write_text(json.dumps({'units':'m','material':'rigid polypropylene','size_m':[.100,.140,.0675],'origin':'centre of bottom face','fixed_fixture':True},indent=2)+'\n')
