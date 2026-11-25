"""Open passive instrument rests for an inert catheter-care bench, full-size metres.

They support the authored reset. No clamp, joint, grasp attachment or actuation.
"""
from pathlib import Path
import json
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf


def build():
    here=Path(__file__).resolve().parent
    stage=Usd.Stage.CreateNew(str(here/'rests.usda'))
    root=UsdGeom.Xform.Define(stage,'/InstrumentRests')
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    material=UsdShade.Material.Define(stage,'/InstrumentRests/Polymer')
    shader=UsdShade.Shader.Define(stage,'/InstrumentRests/Polymer/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.68,.76,.75))
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    boxes=[]
    def box(name,center,size):
        obj=UsdGeom.Cube.Define(stage,'/InstrumentRests/'+name)
        obj.CreateSizeAttr(1.);obj.AddTranslateOp().Set(Gf.Vec3d(*center));obj.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(material)
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        obj.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
        obj.GetPrim().CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.0003)
        obj.GetPrim().CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
        boxes.append({'name':name,'center':center,'size':size})
    # The valve's 30x24x20mm tab rests on a smaller flat top; both
    # gripper sides remain exposed and the valve must still be held to connect.
    box('ValveFoot',(.336,-.124,.768),(.045,.035,.006))
    box('ValveStem',(.3449,-.124,.80575),(.010,.010,.0695))
    box('ValveTop',(.3449,-.124,.8435),(.012,.020,.006))
    # Rear corner ledges keep the rigid trainer's tab initially supported.
    # The branch crosses the original forward left ledge. More importantly,
    # the released valve snagged the original tall posts during extraction.
    # Put the tall posts outside the complete valve sweep; only short ledges
    # remain under the rear tab corners. Ends and top stay open.
    box('HubFoot',(.375,-.142,.768),(.160,.035,.006))
    for name,x,armx in [('Left',.300,.331),('Right',.450,.419)]:
        box('HubPost'+name,(x,-.142,.80825),(.006,.012,.0745))
        box('HubArm'+name,(armx,-.142,.8435),(.064,.006,.004))
    for name,x in [('Left',.362),('Right',.388)]:
        box('HubLedge'+name,(x,-.142,.8495),(.002,.006,.008))
    stage.GetRootLayer().Save()
    (here/'dimensions.json').write_text(json.dumps({'units':'metres','boxes':boxes,
        'valve_support_top_z':.8465,'hub_tab_support_top_z':.8535,
        'mechanism':'open support only; no joints or actuators'},indent=2)+'\n')


if __name__=='__main__':build()
