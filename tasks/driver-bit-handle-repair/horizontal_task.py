"""Horizontal empty-tool stand prototype; no repair score yet."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from mount_task import *
from dataclasses import replace
import numpy as np
original_arena_object=partkit._arena_object
def horizontal_object(spec):
    obj=original_arena_object(spec)
    if spec.name=='driver':
        from isaaclab_arena.utils.pose import Pose
        obj.initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=(-2**-.5,0.,0.,2**-.5))
        obj._pose_event_cfg=obj._build_reset_event()
    return obj
partkit._arena_object=horizontal_object
OBJECTS=[replace(o,pos=(.18,-.015,TOP+.065)) if o.name=='driver' else o for o in OBJECTS if o.name!='cradle']
for i,y in enumerate((-.006,.076)):
    OBJECTS.append(part('tool_support_'+str(i),(.18,y,TOP),
        [((0,0,.0225),(.055,.012,.045),DARK),
         ((-.025,0,.048),(.006,.012,.015),DARK),
         ((.025,0,.048),(.006,.012,.015),DARK)],.2,fixed_to_world=True))
INSTRUCTION='Prototype: grasp the empty horizontal screwdriver and point it down over the screw.'
