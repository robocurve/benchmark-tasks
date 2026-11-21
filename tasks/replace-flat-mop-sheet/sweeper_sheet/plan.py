"""Wrap the two wide fixation strips and close the retaining levers by contact."""

import numpy as np
from yam_arena.constants import DOWN_XYZW
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as c
from .motion import Program, rotation

DOWN = quat_xyzw_to_matrix(DOWN_XYZW)
CLIP_DOWN = rotation("Z",np.radians(90.)) @ DOWN
HANDLE_YAW = 120.
HANDLE_TILT = 30.


def cloth_push(side,y,z,dx=-.009):
    approach = rotation("X", side*-.38) @ CLIP_DOWN
    return c.end_point(side,y,z,dx),approach,.03


def fold_pair(y,z,dx=-.009):
    return {name:cloth_push(s,y,z,dx) for name,s in (("left",1),("right",-1))}


def clip_point(side,angle,dx=-.012,lift=0.):
    theta = np.radians(angle)
    y = c.CLAMP_Y + c.CLAMP_LENGTH*.85*np.cos(theta)
    z = c.HEAD_Z + c.CLAMP_Z + c.CLAMP_LENGTH*.85*np.sin(theta) + lift
    downward = rotation("X", side*-.22) @ CLIP_DOWN
    return c.end_point(side,y,z,dx),downward,.08


def clips(angle,lift=0.,dx=-.012):
    return {name:clip_point(s,angle,dx,lift) for name,s in (("left",1),("right",-1))}


def waypoints(mode):
    p=Program()
    if mode=="hold":
        return p.moves
    p.add(3.8,"approach the two loose upright strips",**fold_pair(.239,c.FABRIC_Z+.107))
    p.add(1.35,"lower each wrist beside a loose strip",**fold_pair(.231,c.FABRIC_Z+.066))
    p.add(1.5,"sweep the two fixation strips inwards",**fold_pair(.195,c.FABRIC_Z+.051))
    p.add(.85,"guide the strips down under the open levers",**fold_pair(.176,c.FABRIC_Z+.033))
    p.add(.65,"press the ends flat over the clamp anvils",**fold_pair(.175,c.FABRIC_Z+.031))
    p.add(.55,"leave both folded strips resting on the anvils",**fold_pair(.174,c.FABRIC_Z+.067))
    if mode == "one-clamp":
        p.add(1.55,"approach only the left lever",left=clip_point(1,64.,lift=.060))
        p.add(1.0,"lower the left wrist to the lever",left=clip_point(1,64.,lift=.004))
        for angle in (50.,29.,0.):
            p.add(.65,"press just the left lever",left=clip_point(1,angle,lift=-.003))
    else:
        p.add(1.55,"approach the two open levers",**clips(64.,lift=.060))
        p.add(1.0,"contact the free ends of both levers",**clips(64.,lift=.004))
        for angle in (50.,29.,0.):
            p.add(.65,"swing the levers shut over both strips",**clips(angle,lift=-.003))
    p.add(1.0,"withdraw both fingers vertically clear of the closed clamps",
          **clips(0.,lift=.109))
    p.add(.8,"retreat sideways with the left hand high",
          left=(np.array([c.X,.276,c.HEAD_Z+.155]),clip_point(1,0.)[1],.7))
    if mode == "one-clamp":
        p.add(.9,"leave both hands clear with the right clamp still open",
              right=(np.array([c.X,-.278,c.HEAD_Z+.155]),clip_point(-1,0.)[1],.7))
        return p.moves
    hold = np.array([c.X,0.,c.HANDLE_Z])
    handle_basis = rotation("X", np.radians(HANDLE_TILT)) @ rotation("Z", np.radians(HANDLE_YAW)) @ DOWN
    p.add(1.55,"approach above the swivel connector",
          right=(hold+[0.,0.,.110],handle_basis,.58))
    p.add(1.3,"place the open pads around the connector",right=(hold,handle_basis,.58))
    p.add(.75,"grasp the connector",right=(hold,handle_basis,.09))
    p.add(1.4,"lift to check the pad stays under the head",right=(hold+[0.,0.,.045],handle_basis,.09))
    p.add(.55,"hold the fitted pad clear of the tile",right=(hold+[0.,0.,.045],handle_basis,.09))
    wipe_start = np.array([.201, .003, c.HANDLE_Z-.005])
    wipe_end = np.array([.111, .003, c.HANDLE_Z-.005])
    p.add(1.65,"carry the lifted mop over the floor sample",
          right=(wipe_start+[0.,0.,.048],handle_basis,.09))
    p.add(1.35,"lower the fitted pad onto the tile",right=(wipe_start,handle_basis,.09))
    p.add(1.85,"draw the fitted pad across the tile",right=(wipe_end,handle_basis,.09))
    p.add(.60,"let the mop settle on the tile",right=(wipe_end,handle_basis,.09))
    p.add(.72,"release the handle",right=(wipe_end,handle_basis,.65))
    p.add(1.25,"leave both hands clear",right=(wipe_end+[.04,-.16,.105],handle_basis,.75))
    return p.moves
