"""Actual robot assembly: bit starts on its rest, not in the driver."""
from clearance_grip_task import *
OBJECTS=[replace(o,pos=(.045,-.13,TOP+.070)) if o.name=='bit' else o for o in OBJECTS]
_assembly_object=partkit._arena_object
def _assembly_reset(spec):
    obj=_assembly_object(spec)
    if spec.name=='bit':
        from isaaclab_arena.utils.pose import Pose
        obj.initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=(0.,2**-.5,0.,2**-.5))
        obj._pose_event_cfg=obj._build_reset_event()
    return obj
partkit._arena_object=_assembly_reset
