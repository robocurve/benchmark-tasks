"""Stage stock passive cart components on an ordinary lowered work deck."""

import numpy as np

import geometry as geo
from native_joints import JOINTS, Joint
from partkit import Part
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.task_file import Box
from yam_arena.urdf import axis_angle_matrix

PARTS = {}
ORIENTATIONS = {}


def component(name, xyz, mass, boxes, *, angle=0., axis="Z", cylinders=(), friction=.47):
    direction = np.eye(3)[{"X":0,"Y":1,"Z":2}[axis]]
    rotation = axis_angle_matrix(direction, angle)
    ORIENTATIONS[name] = rotation
    obj = Part(name, pos=tuple(map(float,xyz)), size=(.02,.02,.02), mass=mass,
               boxes=boxes, cylinders=cylinders, friction=friction,
               orientation=tuple(map(float,matrix_to_quat_xyzw(rotation))))
    PARTS[name] = obj
    return obj


def pivot(name, parent, child, at, axis, limits, angular_friction=None, collide=False):
    a,b = ORIENTATIONS[parent],ORIENTATIONS[child]
    JOINTS.append(Joint(
        name,"revolute",parent,child,
        tuple(a.T @ (at-PARTS[parent].pos)), tuple(b.T @ (at-PARTS[child].pos)),
        axis=axis,limits=limits,collisions=collide,spring=angular_friction,
    ))


rear = np.array([geo.REAR_X,0.,geo.BASE_Z])
front = np.array([geo.fore_hinge(1)[0],0.,geo.BASE_Z])
back_wheels = tuple(((-.019,s*.203,-.032),.026,.020,"Y",geo.WHEEL) for s in (-1,1))
component("cart_back",rear,1.40,geo.endwall(False),cylinders=back_wheels,friction=.42)
component("cart_front",front,.94,geo.endwall(True),friction=.45)

floor_axis = front+[-.018,0.,.025]
floor_parts = tuple(((-p[0],p[1],p[2]),size,c) for p,size,c in geo.floor())
component("cart_floor",floor_axis,.61,floor_parts,axis="Y",angle=geo.FLOOR_TILT)
# Each real end of the lowered floor rests on molded supports. The end-to-end hinge has free fall.
pivot("floor_hinge","cart_front","cart_floor",floor_axis,"Y",(-3.,87.),(0.,.15,0.),True)

for label,side in (("left",1),("right",-1)):
    back_leaf, front_leaf = f"{label}_back_leaf", f"{label}_front_leaf"
    slider = f"{label}_rigidity_bar"
    back, middle, fore = geo.rear_hinge(side),geo.middle_hinge(side),geo.fore_hinge(side)
    angle = -side*geo.FOLD
    component(back_leaf,back,.265,geo.folding_side(),angle=angle)
    component(front_leaf,middle,.265,geo.folding_side(),angle=-angle)
    pivot(f"{label}_rear_pivot","cart_back",back_leaf,back,"Z",
          (-85.,1.5) if side==1 else (-1.5,85.))
    pivot(f"{label}_middle_pivot",back_leaf,front_leaf,middle,"Z",
          (-2.,167.) if side==1 else (-167.,2.))
    pivot(f"{label}_front_pivot",front_leaf,"cart_front",fore,"Z",
          (-85.,1.5) if side==1 else (-1.5,85.))
    local_start = np.array([geo.BAR_START,0.,geo.RAIL_Z-geo.BASE_Z])
    rotation = ORIENTATIONS[back_leaf]
    component(slider,back+rotation @ local_start,.047,geo.slider(),angle=angle,friction=.50)
    JOINTS.append(Joint(f"{label}_bar_track","prismatic",back_leaf,slider,
                        tuple(local_start),(0.,0.,0.),axis="X",limits=(0.,geo.BAR_TRAVEL),
                        collisions=False,spring=(0.,8.0,0.)))

OBJECTS = [
    Box("lower_work_deck",pos=(.285,0.,geo.SURFACE-.026),size=(.70,.78,.052),color=(.56,.48,.37)),
    *(Box(f"deck_leg_{i}", pos=(x,y,geo.SURFACE/2-.030),
           size=(.050,.050,geo.SURFACE-.044), color=(.36,.34,.30))
      for i,(x,y) in enumerate(((-.041,-.344),(-.041,.344),(.600,-.344),(.600,.344)))),
    Box("left_arm_riser",pos=(-.035,.338,.625),size=(.165,.20,.245),color=(.34,.36,.38)),
    Box("right_arm_riser",pos=(-.035,-.338,.625),size=(.165,.20,.245),color=(.34,.36,.38)),
    *PARTS.values(),
]
