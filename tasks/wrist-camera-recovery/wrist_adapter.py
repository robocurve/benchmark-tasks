"""Physical bracket plus keyed snap-latch approximation. Never writes camera body poses.
Latch models a reusable clip, not arbitrary attachment: tight seating and low speed required.
"""
import numpy as np
BRACKET_AUDIT=[]
def install():
 import yam_arena.embodiment as emb
 import isaaclab.sim as sim
 from isaaclab.sim.utils import clone
 from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
 from yam_arena import station
 from yam_arena.demo import grip_rotation
 from yam_arena.robot import asset_link_paths
 import scene as spec
 original=emb.yam_arm_cfg;R=np.array(spec.LAYOUT['mount_rotation_in_gripper']).T;center=np.array(spec.LAYOUT['mount_position_in_gripper'])
 @clone
 def spawn(path,cfg,*args,**kwargs):
  from yam_arena.robot import spawn_in_dream_look
  prim=spawn_in_dream_look.__wrapped__(path,cfg,*args,**kwargs)
  parent=path+asset_link_paths()['gripper']
  from latch_geometry import MOUNT,TOOTH_RETRACTED,TOOTH_SIZE
  from yam_arena import ik
  from scipy.spatial.transform import Rotation
  for name,delta,size in MOUNT:
   c=sim.CuboidCfg(size=size,collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0002,rest_offset=0),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.18,.32,.48),metallic=.6))
   c.func(parent+'/recovery_'+name,c,translation=tuple(center+R.T@np.array(delta)),orientation=tuple(ik.matrix_to_quat_xyzw(R.T)))
  # Spring leaf visible outside the channel; tooth's deflection is approximated.
  for name,delta,size in [('spring_leaf',(.018,-.025,-.005),(.001,.006,.030)),('spring_tooth',TOOTH_RETRACTED,TOOTH_SIZE)]:
   c=sim.CuboidCfg(size=size,visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.65,.67,.70),metallic=.8))
   c.func(parent+'/recovery_'+name,c,translation=tuple(center+R.T@np.array(delta)),orientation=tuple(ik.matrix_to_quat_xyzw(R.T)))
  for side in (-1,1):
   start=np.array([side*.017,-.020,.025]);end=center+R.T@np.array([side*.015,-.030,-.025])
   delta=end-start;length=np.linalg.norm(delta);z=delta/length;x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
   c=sim.CuboidCfg(size=(.007,.007,float(length+.007)),collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0002,rest_offset=0),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.32,.36,.40),metallic=.6))
   c.func(parent+'/recovery_strut_'+str(side).replace('-','m'),c,translation=tuple((start+end)/2),orientation=tuple(Rotation.from_matrix(np.column_stack([x,y,z])).as_quat()))
  return prim
 def configured(*args,**kwargs):
  cfg=original(*args,**kwargs)
  if 'LeftArm' in cfg.prim_path:cfg.spawn.func=spawn;cfg.spawn.make_uninstanceable=True
  return cfg
 emb.yam_arm_cfg=configured
 # Spawn compound camera collision geometry before physics initialization.
 import yam_arena.scene as arena_scene
 original_object=arena_scene.task_object
 def camera_object(spec):
  obj=original_object(spec)
  if spec.name=='loose_camera':
   original_spawn=obj.spawner_cfg.func
   @clone
   def compound(path,cfg,*args,**kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    prim=spawn_cuboid.__wrapped__(path,cfg,*args,**kwargs)
    add_camera_shoe(path)
    return prim
   obj.spawner_cfg.func=compound
   obj.object_cfg.spawn.func=compound
   obj.object_cfg.spawn.collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0002,rest_offset=0)
  return obj
 arena_scene.task_object=camera_object

def latch(stage,parent_path,camera_path,gripper_pos,gripper_rot,camera_pos,camera_rot):
 from pxr import UsdPhysics,Gf,Sdf
 from scipy.spatial.transform import Rotation
 # Joint frames coincide at current camera pose, so latching cannot snap/teleport it.
 joint=UsdPhysics.FixedJoint.Define(stage,'/World/RecoveryLatch')
 joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
 joint.CreateBody1Rel().SetTargets([Sdf.Path(camera_path)])
 local=gripper_rot.T@(camera_pos-gripper_pos);q=Rotation.from_matrix(gripper_rot.T@camera_rot).as_quat()
 joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float,local)))
 joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
 joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0,0,0));joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
 joint.CreateExcludeFromArticulationAttr().Set(True)
 return joint

def add_camera_shoe(camera_path):
 import isaaclab.sim as sim
 from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
 from latch_geometry import SHOE
 for name,pos,size in SHOE:
  c=sim.CuboidCfg(size=size,collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0002,rest_offset=0),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.12,.12,.14)))
  c.func(camera_path+'/'+name,c,translation=pos)

def engage_tooth(stage,parent):
 from pxr import UsdGeom,Gf
 import scene
 from latch_geometry import TOOTH_ENGAGED
 R=np.array(scene.LAYOUT['mount_rotation_in_gripper']);p=np.array(scene.LAYOUT['mount_position_in_gripper'])+R@np.array(TOOTH_ENGAGED)
 prim=stage.GetPrimAtPath(parent+'/recovery_spring_tooth')
 for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
  if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(*p))
