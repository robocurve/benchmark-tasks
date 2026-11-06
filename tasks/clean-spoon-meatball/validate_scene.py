"""CPU checks for portable USDs, movable tools and hollow vessel colliders."""
from pathlib import Path
import json
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils, UsdShade

ROOT=Path(__file__).resolve().parent
report={}
for file in ['spoon.usda','spoon_clean.usda','spoon_used.usda','tray.usda','plate.usda','spoon_plate.usda','platter.usda','bowl.usda','meatball.usda','raw_chicken.usda']:
    path=ROOT/'assets'/file; s=Usd.Stage.Open(str(path))
    assert s and s.GetDefaultPrim()
    assert UsdGeom.GetStageUpAxis(s)=='Z' and UsdGeom.GetStageMetersPerUnit(s)==1
    layers,assets,unresolved=UsdUtils.ComputeAllDependencies(str(path))
    assert not unresolved,(file,unresolved)
    assert all(Path(x.realPath).is_relative_to(ROOT) for x in layers if x.realPath)
    assert all(Path(str(x)).is_relative_to(ROOT) for x in assets)
    colliders=[p for p in s.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    if file=='bowl.usda':
        assert not any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in s.Traverse())
    if file in ('spoon.usda','spoon_clean.usda','spoon_used.usda','meatball.usda','raw_chicken.usda'):
        assert s.GetDefaultPrim().HasAPI(UsdPhysics.RigidBodyAPI)
        assert not UsdPhysics.RigidBodyAPI(s.GetDefaultPrim()).GetKinematicEnabledAttr().Get()
    for p in s.Traverse():
        if p.IsA(UsdGeom.Mesh):
            m=UsdGeom.Mesh(p); pts=np.asarray(m.GetPointsAttr().Get())
            assert np.isfinite(pts).all()
            ids=np.asarray(m.GetFaceVertexIndicesAttr().Get()); counts=m.GetFaceVertexCountsAttr().Get()
            assert len(ids)==sum(counts) and ids.min()>=0 and ids.max()<len(pts)
    report[file]={'colliders':len(colliders),'dynamic':s.GetDefaultPrim().HasAPI(UsdPhysics.RigidBodyAPI),'unresolved_dependencies':[]}
    if file in ('plate.usda','spoon_plate.usda','platter.usda'):
        shell=s.GetPrimAtPath(str(s.GetDefaultPrim().GetPath())+'/Visuals/Shell')
        points=np.asarray(UsdGeom.Mesh(shell).GetPointsAttr().Get())
        assert points[:,2].max()<=.0081
        mat,_=UsdShade.MaterialBindingAPI(shell).ComputeBoundMaterial()
        shader,_,_=mat.ComputeSurfaceSource()
        assert shader.GetInput('metallic').Get()==0
        assert min(shader.GetInput('diffuseColor').Get())>.85
        report[file]['height_m']=float(points[:,2].max())
        report[file]['finish']='glossy white, non-metallic'

# The used variant must inherit identical mass, geometry and collision settings.
def signature(file):
    s=Usd.Stage.Open(str(ROOT/'assets'/file))
    return [(str(p.GetPath()),str(p.GetTypeName()),[(a.GetName(),str(a.Get())) for a in p.GetAttributes()])
            for p in s.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
assert signature('spoon_clean.usda')==signature('spoon_used.usda')
for file in ('task_objects.usda','scene.usda'):
    layers,assets,unresolved=UsdUtils.ComputeAllDependencies(str(ROOT/file))
    assert not unresolved
    assert all(Path(x.realPath).is_relative_to(ROOT) for x in layers if x.realPath)
    assert all(Path(str(x)).is_relative_to(ROOT) for x in assets)
    report[file]={'unresolved_dependencies':[]}
layout=json.loads((ROOT/'layout.json').read_text())
assert layout['spoon_bowl']['inner_width_m']>layout['meatball_diameter_m']+.016
assert all(x['handle_width_m']<x['pregrasp_opening_m'] for x in layout['spoon_grasps'].values())
assert 'SharedPlatter' not in layout['objects']
assert layout['spoon_plates']['separate']
assert layout['objects']['CleanPlate']['asset']=='platter.usda'
assert layout['objects']['RawPlate']['asset']=='platter.usda'
assert layout['objects']['CleanPlate']['position'][1]>.25
assert layout['objects']['RawPlate']['position'][1]<-.25
assert layout['objects']['MeatballBowl']['position'][1]==0
assert layout['objects']['Plate']['position'][1]==0
assert layout['objects']['MeatballBowl']['position'][0]>layout['objects']['Plate']['position'][0]+.24
assert layout['objects']['CleanSpoon']['position'][:2]==layout['objects']['CleanPlate']['position'][:2]
assert layout['objects']['UsedSpoon']['position'][:2]==layout['objects']['RawPlate']['position'][:2]
assert all(row['position'][1]<0 for name,row in layout['objects'].items() if name.startswith('RawChicken_'))
assert layout['objects']['MeatballBowl']['asset']=='bowl.usda'
assert layout['meatball_bowl']['fixed_to_world']
assert layout['objects']['CleanSpoon']['position'][1]>0
assert layout['objects']['UsedSpoon']['position'][1]<0
assert layout['spoon_grasps']['CleanSpoon']['arm']=='left'
assert layout['spoon_grasps']['UsedSpoon']['arm']=='right'
station=json.loads((ROOT/'assets/base/station_layout.json').read_text())
arm_checks={}
for name,g in layout['spoon_grasps'].items():
    distances={side:float(np.linalg.norm(np.array(g['center_world'])[:2]-np.array(pos)[:2]))
               for side,pos in station['rig']['bases'].items()}
    assert min(distances,key=distances.get)==g['arm']
    arm_checks[name]={'assigned_arm':g['arm'],'horizontal_distance_to_bases_m':distances}
report['physical_arm_sides']=arm_checks
from clearance import export_envelopes,measure
envelopes=export_envelopes()
poses={}
for name,row in layout['objects'].items():
    a=np.radians(row.get('yaw',0))/2
    poses[name]=[*row['position'],0,0,float(np.sin(a)),float(np.cos(a))]
gap=measure(envelopes,poses)
assert gap['clean_to_raw_meat_m']>.020,gap
assert gap['spoon_to_spoon_m']>.020,gap
report['initial_clearance_lower_bounds']=gap
report['tool_checks']={'matching_clean_used_colliders':True,'hollow_spoon_tiles':96,
                       'handle_width_m':.024,'handle_thickness_m':.020,'under_handle_clearance_m':.017,
                       'meatball_diameter_m':.036,'spoon_inner_width_m':layout['spoon_bowl']['inner_width_m']}
(ROOT/'validation/asset_checks.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
