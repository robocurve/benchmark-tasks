"""Private, physically interpretable pose measurements; never policy inputs."""
import math
import numpy as np
from . import layout as L

GUIDE_POINTS = np.array([L.point(a) for a in np.linspace(math.radians(-45), math.radians(135), 73)])

def rotation_xyzw(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def threaded_through_tunnel(position, rotation):
    """Connected guide centreline crosses both openings and stays inside the tunnel.

    Both directions are legal. This is threading, not clearance certification;
    actual physics contacts enforce clearance of the finite-thickness guide.
    """
    local=(GUIDE_POINTS-position)@rotation
    local[:,0]+=L.HANDLE_OFFSET
    dy=np.diff(local[:,1]);ends=[]
    for face in (-L.DEPTH/2,L.DEPTH/2):
        indices=np.flatnonzero((abs(dy)>1e-12)&((local[:-1,1]-face)*(local[1:,1]-face)<=1e-14))
        hits=[]
        for i in indices:
            u=(face-local[i,1])/dy[i]
            if not -1e-10<=u<=1+1e-10:continue
            point=local[i]+u*(local[i+1]-local[i])
            if max(abs(point[0]),abs(point[2]))<=L.APERTURE/2+1e-10:hits.append((float(i+u),point))
        ends.append(hits)
    for a,pa in ends[0]:
        for b,pb in ends[1]:
            lo,hi=sorted((a,b));inside=local[math.floor(lo)+1:math.ceil(hi)]
            if np.max(np.abs(np.vstack([pa,inside,pb])[:,[0,2]]))<=L.APERTURE/2+1e-10:return True
    return False

def pose_measurements(pose):
    pose=np.asarray(pose,dtype=float)
    if pose.shape!=(7,) or not np.isfinite(pose).all():raise ValueError('invalid pose')
    norm=np.linalg.norm(pose[3:])
    if abs(norm-1)>.001:raise ValueError('non-unit orientation')
    q=pose[3:]/norm;rotation=rotation_xyzw(q)
    centre=pose[:3]+rotation@np.array([-L.HANDLE_OFFSET,0.,0.])
    angle=math.atan2(centre[1]-L.CENTER[1],centre[0]-L.CENTER[0])
    remaining=abs(math.atan2(math.sin(angle-L.FINISH),math.cos(angle-L.FINISH)))
    # Measure the tunnel axis, not a named side of the grasp handle. A square
    # cross-section can rest on any of its four equivalent faces.
    tunnel_heading=math.atan2(rotation[1,1],rotation[0,1])
    alignment=math.acos(float(np.clip(abs(math.cos(tunnel_heading-L.FINISH-math.pi/2)),0,1)))
    tilt=math.acos(float(np.clip(max(abs(rotation[2,0]),abs(rotation[2,2])),0,1)))
    return dict(pose=pose.tolist(),quaternion=q,centre=centre,angle_deg=math.degrees(angle),
                progress=float(np.clip(1-remaining/(L.FINISH-L.START),0,1)),
                distance_m=float(np.linalg.norm(centre-L.point(L.FINISH))),
                alignment_deg=math.degrees(alignment),tilt_rad=tilt,
                threaded=threaded_through_tunnel(pose[:3],rotation))
