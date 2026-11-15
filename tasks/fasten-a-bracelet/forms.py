"""Static horizontal practice forms with native robot contact."""
from dataclasses import dataclass
from yam_arena.task_file import Box
@dataclass(frozen=True)
class Form(Box):
 radius:float=.03
 length:float=.20
 axis:str='x'

def install():
 from yam_arena import scene
 if getattr(scene,'_practice_forms',False):return
 old=scene.task_object
 def task_object(spec):
  if not isinstance(spec,Form):return old(spec)
  import math
  import isaaclab.sim as sim
  from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
  from isaaclab_arena.assets.object import Object
  from isaaclab_arena.assets.object_type import ObjectType
  from isaaclab_arena.utils.pose import Pose
  from yam_arena.station import linear_rgb
  c=math.sqrt(.5);q=(0,c,0,c) if spec.axis=='x' else (-c,0,0,c)
  return Object(name=spec.name,object_type=ObjectType.BASE,spawner_cfg=sim.CylinderCfg(radius=spec.radius,height=spec.length,collision_props=PhysxCollisionPropertiesCfg(),physics_material=sim.RigidBodyMaterialCfg(static_friction=.5,dynamic_friction=.5,restitution=0.),visual_material=sim.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color),roughness=.7)),initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=q))
 scene.task_object=task_object;scene._practice_forms=True
install()
