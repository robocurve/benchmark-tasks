"""Kinematic collision shells follow the independently integrated pram body.

Only the cart's current physical x/y/yaw state positions these shells. They
block the robot from passing through the carrycot or occupant; they do not
follow demo targets or animate completion. The handle grip remains reduced.
"""
from dataclasses import dataclass
from yam_arena.task_file import Box
@dataclass(frozen=True)
class MovingShell(Box):pass
SHELLS=[
 MovingShell('cot_base_contact',pos=(.72,0,.57),size=(.76,.40,.07)),
 MovingShell('cot_left_contact',pos=(.72,.205,.69),size=(.76,.035,.22)),
 MovingShell('cot_right_contact',pos=(.72,-.205,.69),size=(.76,.035,.22)),
 MovingShell('cot_near_contact',pos=(.325,0,.69),size=(.035,.40,.22)),
 MovingShell('cot_far_contact',pos=(1.115,0,.69),size=(.035,.40,.22)),
 MovingShell('doll_contact',pos=(.705,0,.70),size=(.56,.18,.14))]

def install():
 from yam_arena import scene
 if getattr(scene,'_pram_contact_installed',False):return
 old=scene.task_object
 def task_object(spec):
  if not isinstance(spec,MovingShell):return old(spec)
  import isaaclab.sim as sim
  from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg,PhysxCollisionPropertiesCfg
  from isaaclab_arena.assets.object import Object
  from isaaclab_arena.assets.object_type import ObjectType
  from isaaclab_arena.utils.pose import Pose
  cfg=sim.CuboidCfg(size=spec.size,rigid_props=PhysxRigidBodyPropertiesCfg(kinematic_enabled=True,disable_gravity=True),mass_props=sim.MassPropertiesCfg(mass=.1),collision_props=PhysxCollisionPropertiesCfg(),visible=False)
  return Object(name=spec.name,object_type=ObjectType.RIGID,spawner_cfg=cfg,initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=(0,0,0,1)))
 scene.task_object=task_object;scene._pram_contact_installed=True
install()
