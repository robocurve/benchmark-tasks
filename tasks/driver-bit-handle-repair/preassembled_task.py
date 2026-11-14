"""Alignment-only reset fixture: bit starts seated; never a full task demo."""
from long_bits_task import *
OBJECTS=[replace(o,pos=(.18,.0878,TOP+.067)) if o.name=='bit' else o for o in OBJECTS]
_previous=partkit._arena_object
def _preassembled_object(spec):
    obj=_previous(spec)
    if spec.name=='bit':
        from isaaclab_arena.utils.pose import Pose
        obj.initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=(-2**-.5,0.,0.,2**-.5));obj._pose_event_cfg=obj._build_reset_event()
    return obj
partkit._arena_object=_preassembled_object
