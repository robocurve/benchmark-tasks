"""Move handle and both fastening sites together for real driver clearance."""
from preload_grip_task import *
FASTENER_X=-.032
# 20 mm toward the open area, preserving drawer walls and tool geometry.
OBJECTS=[replace(o,boxes=tuple(((b[0][0]-.020,*b[0][1:]),*b[1:]) if 5<=i<=7 else b for i,b in enumerate(o.boxes))) if o.name=='drawer_context' else o for o in OBJECTS]
_old_clearance_spawn=partkit._spawn_part
def _clearance_spawn(prim_path,cfg,*args,**kwargs):
    result=_old_clearance_spawn(prim_path,cfg,*args,**kwargs)
    if cfg.part_name=='drawer_context':
        from pxr import UsdGeom,Gf
        from isaaclab.sim.utils.stage import get_current_stage
        stage=get_current_stage()
        for prim in stage.GetPrimAtPath(prim_path+'/body').GetChildren():
            if prim.GetName().startswith('round_screw_'):
                mesh=UsdGeom.Mesh(prim);mesh.GetPointsAttr().Set([Gf.Vec3f(v[0]-.020,v[1],v[2]) for v in mesh.GetPointsAttr().Get()])
            elif prim.GetName().startswith('screw_floor_'):
                for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                    if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:
                        v=op.Get();op.Set(Gf.Vec3d(v[0]-.020,v[1],v[2]))
    return result
partkit._spawn_part=_clearance_spawn
