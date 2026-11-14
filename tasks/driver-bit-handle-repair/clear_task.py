"""Larger round recessed screw-head prototype, still no repair score."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from horizontal_task import *
from dataclasses import replace
import math
# Same-colour matching tip; physical contact dimensions agree with bench probe.
OBJECTS=[replace(o,boxes=(o.boxes[0],((0,0,.064),(.007,.0016,.003),GREY),((0,0,.064),(.0016,.007,.003),GREY))) if o.name=='bit' else o for o in OBJECTS]
# Tangible backstop resists insertion along the horizontal tool axis.
OBJECTS.append(part('tool_backstop',(.18,-.026,TOP),[((0,0,.037),(.052,.008,.074),DARK)],.2,fixed_to_world=True))
_previous_spawn=partkit._spawn_part
def clear_spawn(prim_path,cfg,*args,**kwargs):
    result=_previous_spawn(prim_path,cfg,*args,**kwargs)
    if cfg.part_name!='drawer_context':return result
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema
    from isaaclab.sim.utils.stage import get_current_stage
    stage=get_current_stage()
    # Remove the original square raised blocks and their backing decals.
    for k in range(13,23):stage.RemovePrim(prim_path+f'/body/box_{k}')
    # Enlarged 12 mm head for camera legibility; M4 pitch is retained.
    radius=.006; slot=.0013
    for index,yy in enumerate((-.025,.095)):
        center=(-.012,yy-.035,.099)
        for sx in (-1,1):
            for sy in (-1,1):
                angle=math.asin(slot/radius)
                poly=[(slot,slot)]+[(radius*math.cos(t),radius*math.sin(t)) for t in [angle+(math.pi/2-2*angle)*j/12 for j in range(13)]]
                n=len(poly)
                pts=[Gf.Vec3f(center[0]+sx*x,center[1]+sy*y,center[2]+z) for z in (-.002,.002) for x,y in poly]
                faces=[list(range(n-1,-1,-1)),list(range(n,2*n))]+[[j,(j+1)%n,(j+1)%n+n,j+n] for j in range(n)]
                if sx*sy<0:faces=[list(reversed(f)) for f in faces]
                m=UsdGeom.Mesh.Define(stage,prim_path+f'/body/round_screw_{index}_{sx+1}_{sy+1}')
                m.CreatePointsAttr(pts);m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(.65,.67,.7)])
                UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
                PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim()).CreateContactOffsetAttr(.0001)
        c=UsdGeom.Cylinder.Define(stage,prim_path+f'/body/screw_floor_{index}')
        c.CreateRadiusAttr(radius);c.CreateHeightAttr(.001);c.AddTranslateOp().Set(Gf.Vec3d(center[0],center[1],center[2]-.0025));c.CreateDisplayColorAttr([Gf.Vec3f(.10,.11,.12)])
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return result
partkit._spawn_part=clear_spawn
