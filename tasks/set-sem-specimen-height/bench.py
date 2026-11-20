"""Small repeatable operations for the new bench fixtures."""
from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))
from plan import OPEN, TABLE, TIP, Track, check_tracks, waypoints
from runtime import bodies, contacts, register, reset_contacts, tick

HOVER = TABLE + .140


def move_pick(track, xy, grip_height, aperture, label, *, yaw=0.):
    at = (*xy, grip_height+TIP-.002)
    track.go((*xy,HOVER),OPEN,f"above {label}",1.35,yaw)
    track.go(at,OPEN,f"approach {label}",1.25,yaw)
    track.go(at,aperture,f"grasp {label}",.9,yaw)
    track.go((*xy,HOVER),aperture,f"lift {label}",1.35,yaw)


def move_place(track, xy, grip_height, aperture, label, *, yaw=0.):
    at = (*xy, grip_height+TIP-.002)
    track.go((*xy,HOVER),aperture,f"above {label}",1.55,yaw)
    track.go(at,aperture,f"seat into {label}",1.55,yaw)
    track.go(at,OPEN,f"release at {label}",.8,yaw)
    track.go((*xy,HOVER),OPEN,f"clear {label}",1.3,yaw)


def fixed(name, position, size):
    from scenekit import UsdBody
    return UsdBody(name,usd=f"assets/{name}.usda",pos=position,size=size,mass=None)


def moving(name, position, size, mass):
    from scenekit import UsdBody
    return UsdBody(name,usd=f"assets/{name}.usda",pos=position,size=size,mass=mass,
                   friction=(1.0,.77),torsional_patch=.009,solver_iters=24)


def rig(position,size,joints,damping,stiffness,effort):
    from scenekit import UsdBody
    return UsdBody("rig",usd="assets/rig.usda",pos=position,size=size,articulation=True,
                   joint_pos=tuple(joints.items()),joint_damping=damping,joint_stiffness=stiffness,
                   joint_effort_limit=effort,solver_iters=32,friction=(.75,.54))


def register_trial(items, cls, state, sample, namespace):
    def reset(env):
        namespace["_trial"]=cls()
        state.clear()
        reset_contacts()
    def step(env):
        readings=sample(env)
        namespace["_trial"].step(*readings,tick(env))
    return register(items,reset,step)


def proof(layout,limit,argv):
    check_tracks(layout,limit,ik="--ik" in argv)
