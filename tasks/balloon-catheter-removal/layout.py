"""Reproducible free-valve staging; dimensions and joints remain in bundled assets."""
import math
from pathlib import Path
import numpy as np

RECEIVER_DISPLACEMENT=np.array([.137,.450,0.])
RECEIVER_WORLD_POSITION=(.300,.300,.820)
VALVE_YAW_DEG=-35.
ANGLE=math.radians(VALVE_YAW_DEG)
VALVE_ROTATION=np.array([[math.cos(ANGLE),-math.sin(ANGLE),0.],
                         [math.sin(ANGLE),math.cos(ANGLE),0.],[0.,0.,1.]])
HUB_ANCHOR=np.array([.375,-.132,.851])
VALVE_WORLD_POSITION=tuple(HUB_ANCHOR-VALVE_ROTATION@np.array([.039,.007,0.]))
VALVE_QUATERNION=(0.,0.,math.sin(ANGLE/2),math.cos(ANGLE/2))
RECOVERY_DISH_DISPLACEMENT=np.array([-.070,-.050,.050])
RECOVERY_DISH_CENTER=(.240,-.311,.829)
RECOVERY_STAND_POSITION=(.240,-.311,.7615)
RECOVERY_TRAY_POSITION=(.240,-.341,.7615)
PLACEMENT_CARRY=(.280,-.330,.930)
PLACEMENT_APPROACH=(.250,-.330,.930)
PLACEMENT_LOWER_START=(.250,-.330,.910)
RECOVERY_SHAFT_Z_BOUNDS=(.868,.888)
RECOVERY_RESTS_POSITION=(.250,-.220,.829)
PLACEMENT_ROOT_TARGET=(.250,-.330,.8875)


def build_layouts():
    from pxr import Usd,UsdGeom,Gf
    here=Path(__file__).resolve().parent
    station=Usd.Stage.CreateNew(str(here/'station-layout.usda'))
    root=UsdGeom.Xform.Define(station,'/MedicalStation');station.SetDefaultPrim(root.GetPrim())
    root.GetPrim().GetReferences().AddReference('assets/balloon-catheter-bimanual-trainer-v3/medical_station.usda','/MedicalStation')
    # The old kidney dish is replaced by a separate ordinary receiving tray.
    station.GetPrimAtPath('/MedicalStation/RecoveryDish').SetActive(False)
    valve=UsdGeom.Xform(station.GetPrimAtPath('/MedicalStation/Valve'))
    valve.GetOrderedXformOps()[0].Set(Gf.Vec3d(*(np.array(VALVE_WORLD_POSITION)-[0.,.10,0.])))
    valve.AddRotateZOp().Set(VALVE_YAW_DEG)
    for name in ('SyringePad','SyringeCradle/S0','SyringeCradle/S1','Syringe/Barrel','Syringe/Plunger'):
        obj=UsdGeom.Xformable(station.GetPrimAtPath('/MedicalStation/'+name))
        translate=obj.GetOrderedXformOps()[0]
        translate.Set(Gf.Vec3d(*(np.array(translate.Get())+RECEIVER_DISPLACEMENT)))
    UsdGeom.SetStageMetersPerUnit(station,1.);UsdGeom.SetStageUpAxis(station,'Z');station.GetRootLayer().Save()
    rests=Usd.Stage.CreateNew(str(here/'rest-layout.usda'))
    root=UsdGeom.Xform.Define(rests,'/InstrumentRests');rests.SetDefaultPrim(root.GetPrim())
    root.GetPrim().GetReferences().AddReference('assets/catheter-staging-rests-v2/rests.usda','/InstrumentRests')
    for name,offset in [('ValveFoot',(0.,.015,-.083)),('ValveStem',(.0089,.015,-.04525)),('ValveTop',(.0089,.015,-.0075))]:
        box=UsdGeom.Xformable(rests.GetPrimAtPath('/InstrumentRests/'+name))
        box.GetOrderedXformOps()[0].Set(Gf.Vec3d(*(np.array(VALVE_WORLD_POSITION)+VALVE_ROTATION@offset)))
        box.AddRotateZOp().Set(VALVE_YAW_DEG)
        ops=box.GetOrderedXformOps();box.SetXformOpOrder([op for op in ops if op.GetOpType()!=UsdGeom.XformOp.TypeScale]+[op for op in ops if op.GetOpType()==UsdGeom.XformOp.TypeScale])
    UsdGeom.SetStageMetersPerUnit(rests,1.);UsdGeom.SetStageUpAxis(rests,'Z');rests.GetRootLayer().Save()
    return {'valve_world_position':VALVE_WORLD_POSITION,'valve_quaternion_xyzw':VALVE_QUATERNION,
            'branch_anchor_world_m':HUB_ANCHOR.tolist(),'valve_yaw_deg':VALVE_YAW_DEG,
            'receiver_world_position':RECEIVER_WORLD_POSITION,'receiver_tray_displacement_m':RECEIVER_DISPLACEMENT.tolist(),
            'device_dimensions_and_joints_unchanged':True,'staging_only_no_trial_actuation':True,
            'inactive_legacy_recovery_dish_displacement_m':RECOVERY_DISH_DISPLACEMENT.tolist(),
            'recovery_dish_center_m':RECOVERY_DISH_CENTER,'inactive_legacy_recovery_stand_position_m':RECOVERY_STAND_POSITION,
            'recovery_open_rest_pair_position_m':RECOVERY_RESTS_POSITION,
            'recovery_tray_position_m':RECOVERY_TRAY_POSITION,
            'storage_score_datum_unchanged_m':RECOVERY_DISH_CENTER}

if __name__=='__main__':
    import json
    print(json.dumps(build_layouts(),indent=2))
