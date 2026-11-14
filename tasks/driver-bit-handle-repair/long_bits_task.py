"""Horizontal bit rests for top-down grasp; no functional repair yet."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from clear_task import *
_previous_object=partkit._arena_object
_bits={'bit':-.13,'bit_hex':-.04,'bit_flat':-.22}
def bit_object(spec):
    obj=_previous_object(spec)
    if spec.name in _bits:
        from isaaclab_arena.utils.pose import Pose
        obj.initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=(0.,2**-.5,0.,2**-.5))
        obj._pose_event_cfg=obj._build_reset_event()
    return obj
partkit._arena_object=bit_object
OBJECTS=[replace(o,pos=(.045,_bits[o.name],TOP+.070)) if o.name in _bits else o for o in OBJECTS if o.name not in ('bit_rack','rack_hex','rack_flat')]
for name,y in _bits.items():
    boxes=[((0,0,.005),(.095,.032,.010),DARK)]
    for x in (-.035,.045):
        boxes.append(((x,0,.0334),(.010,.018,.0668),DARK))
        for sign in (-1,1):boxes.append(((x,sign*.008,.070),(.010,.005,.012),DARK))
    OBJECTS.append(part('rest_'+name,(.085,y,TOP),boxes,.2,fixed_to_world=True))

# Molded flat on underside keeps the horizontal housing from rolling in its rest.
OBJECTS=[replace(o,boxes=o.boxes+(((0,.021,.050),(.028,.002,.100),DARK),)) if o.name=='driver' else o for o in OBJECTS]

# Longer shanks create robot-jaw clearance at full socket depth.
OBJECTS=[replace(o,boxes=(((0,0,.045),(.00635,.00635,.090),GREY),)+tuple(((b[0][0],b[0][1],b[0][2]+.030),*b[1:]) for b in o.boxes[1:])) if o.name in _bits else o for o in OBJECTS]
_long_base_spawn=partkit._spawn_part
def long_spawn(prim_path,cfg,*args,**kwargs):
    result=_long_base_spawn(prim_path,cfg,*args,**kwargs)
    if cfg.part_name in _bits:
        from pxr import UsdGeom,Gf
        from isaaclab.sim.utils.stage import get_current_stage
        mesh=UsdGeom.Mesh.Get(get_current_stage(),prim_path+'/body/hex_shank')
        points=list(mesh.GetPointsAttr().Get())
        for i in range(12,18):points[i]=Gf.Vec3f(points[i][0],points[i][1],.090)
        mesh.GetPointsAttr().Set(points)
    return result
partkit._spawn_part=long_spawn

# Retention stiffness tested after removing the independent wrist/rack collision.
OBJECTS=[replace(o,sliders=tuple(replace(j,rate=2000.) for j in o.sliders)) if o.name=='driver' else o for o in OBJECTS]

# Distinct actual working-tip geometry, common material and unchanged mounting shanks.
OBJECTS=[replace(o,boxes=(o.boxes[0],((0,0,.094),(.007,.0014,.008),GREY))) if o.name=='bit_flat' else o for o in OBJECTS]
_tip_spawn=partkit._spawn_part
def distinct_tip_spawn(prim_path,cfg,*args,**kwargs):
    result=_tip_spawn(prim_path,cfg,*args,**kwargs)
    if cfg.part_name=='bit_hex':
        from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab.sim.utils import bind_physics_material
        stage=get_current_stage();stage.RemovePrim(prim_path+'/body/box_1')
        # A true 5 mm across-flats hex working end; all six faces are collision geometry.
        radius=.0025/math.cos(math.pi/6)
        pts=[Gf.Vec3f(radius*math.cos(math.pi/6+k*math.pi/3),radius*math.sin(math.pi/6+k*math.pi/3),z) for z in (.090,.098) for k in range(6)]
        faces=[list(range(5,-1,-1)),list(range(6,12))]+[[k,(k+1)%6,(k+1)%6+6,k+6] for k in range(6)]
        mesh=UsdGeom.Mesh.Define(stage,prim_path+'/body/hex_working_tip')
        mesh.CreatePointsAttr(pts);mesh.CreateFaceVertexCountsAttr([len(f) for f in faces]);mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDisplayColorAttr([Gf.Vec3f(*GREY)])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        pc=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim());pc.CreateContactOffsetAttr(.0001);pc.CreateRestOffsetAttr(0.)
        bind_physics_material(str(mesh.GetPath()),prim_path+'/Looks/physics',stage=stage)
    return result
partkit._spawn_part=distinct_tip_spawn
