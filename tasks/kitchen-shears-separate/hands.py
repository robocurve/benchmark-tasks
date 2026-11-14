"""Installed YAM pad endpoints and conservative hand-volume measurements."""
import math
import numpy as np

def _np(value):
    value=getattr(value,'torch',value)
    return value.detach().cpu().numpy().astype(float)

# Pad inner endpoints expressed in each installed finger link's coordinates.
# Calibrated from the URDF closed pose to gripper-frame (0, 0, .134 + .013).
PAD_POINTS={'tip_left':np.array([.0239931,-.0445119,-.0903171]),
            'tip_right':np.array([-.0239931,.0445119,-.0903171])}

def pad_endpoints(links):
    """Transform each finger's own endpoint; no jaw mirroring or sign mapping."""
    return np.array([links[n][:3,:3]@p+links[n][:3,3] for n,p in PAD_POINTS.items()])

def measure(env):
    from yam_arena import runtime
    base=env.unwrapped; origin=_np(base.scene.env_origins[0])
    tips,centers,radii=[],[],[]
    for name in ('left_robot','right_robot'):
        robot=base.scene[name]
        mid=np.asarray(runtime.pad_tips_world(robot),dtype=float)-origin
        links=runtime.link_poses(robot)
        hand=np.asarray(links['gripper'],dtype=float)
        root=hand[:3,3]-origin
        endpoints=pad_endpoints(links)-origin
        half=float(np.max(np.abs((endpoints-mid)@hand[:3,0])))
        tips.extend(endpoints)
        length=float(np.linalg.norm(mid-root)); n=max(2,int(math.ceil(length/.010))+1)
        for f in np.linspace(0,1,n):
            centers.append(root*(1-f)+mid*f)
            radii.append((1-f)*.040+f*(half+.008)+length/(2*(n-1)))
    return np.asarray(tips),np.asarray(centers),np.asarray(radii)
