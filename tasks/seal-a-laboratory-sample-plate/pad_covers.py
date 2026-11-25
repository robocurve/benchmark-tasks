"""Stock YAM fingers; ordinary 4 N effort limit for handling thin film.

No added meshes, finger extensions, changed joint travel or altered TCP.
"""
def configure_kinematics():
 pass

def install():
 from yam_arena import scene, robot as robot_module
 from yam_arena.constants import GRIPPER
 GRIPPER['effort']=4.
 original=robot_module.yam_arm_cfg
 def configured(*args,**kwargs):
  cfg=original(*args,**kwargs)
  cfg.spawn.articulation_props.solver_position_iteration_count=64
  cfg.spawn.articulation_props.solver_velocity_iteration_count=16
  return cfg
 robot_module.yam_arm_cfg=configured
 if hasattr(scene,'yam_arm_cfg'):scene.yam_arm_cfg=configured
