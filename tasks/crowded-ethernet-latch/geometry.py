"""Shared candidate collider dimensions; SI units, not vendor-validated CAD.

Native export consumes these boxes. CPU obstacles use the same definitions;
round boots/cables are enclosed by oriented boxes for conservative absence
certificates. Positive separation is sound; unresolved overlap is not a claim
of physical collision. Full robot arm links are outside this object builder.
"""
import math
import numpy as np
from config import *

GUIDE_X_MIN=.3995
GUIDE_X_MAX=.4255
GUIDE_Y=.00475
GUIDE_WIDTH=.0025
GUIDE_BOTTOM=.0043
GUIDE_TOP=.0068


def socket_boxes(include_guides=True):
    back=.3985+PANEL_DEPTH;cz=(.95-.0055+.95+PITCH_Z+.0078)/2
    yield 'back',(back-.006,0,cz),(.012,PANEL_WIDTH,PANEL_HEIGHT),'case'
    height=cz-TOP-.012
    yield 'post',(back-.003,0,TOP+.012+height/2),(.020,.045,height),'stand'
    yield 'foot',(back-.005,0,TOP+.006),(.095,.135,.012),'stand'
    for i in range(8):
        _,y,z=port(i)
        yield f'floor_{i}',(.412,y,z-.005),(.027,.015,.001),'socket'
        yield f'roof_{i}',(.412,y,z+.0073),(.027,.015,.001),'socket'
        for sign,label in ((-1,'m'),(1,'p')):
            yield f'side_{i}_{label}',(.412,y+sign*.0075,z+.00115),(.027,.002,.0133),'socket'
            if include_guides:
                yield f'guide_{i}_{label}',((GUIDE_X_MIN+GUIDE_X_MAX)/2,y+sign*GUIDE_Y,z+(GUIDE_BOTTOM+GUIDE_TOP)/2),(GUIDE_X_MAX-GUIDE_X_MIN,GUIDE_WIDTH,GUIDE_TOP-GUIDE_BOTTOM),'socket'
        yield f'ledge_{i}',(.408,y,z+.006),(.002,.0055,.002),'socket'
        yield f'contact_{i}',(back-.004,y,z-.001),(.003,.010,.006),'contact'


def plug_boxes():
    yield 'shell',(0,0,-.0015),(.024,.012,.005)
    for sign,label in ((-1,'m'),(1,'p')):
        yield f'shoulder_{label}',(.003,sign*.00475,.0025),(.014,.0025,.003)


def hook_mesh():
    vertices=[(.006,y,.0005) for y in (-.0025,.0025)]+[(.006,y,.0025) for y in (-.0025,.0025)]+[(.010,y,.0005) for y in (-.0025,.0025)]
    faces=((0,1,3,2),(0,4,5,1),(2,3,5,4),(0,2,4),(1,5,3))
    return vertices,faces


def box_triangles(center,size):
    vertices=np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])*np.asarray(size)/2+center
    faces=np.array(((0,1,3),(0,3,2),(4,6,7),(4,7,5),(0,4,5),(0,5,1),(2,3,7),(2,7,6),(0,2,6),(0,6,4),(1,5,7),(1,7,3)))
    return vertices[faces]


def wire_local_frame(i,k):
    """Capsule geometry's native local rotation and cylinder height."""
    from yam_arena.urdf import quat_xyzw_to_matrix
    a,b=cable_points(i)[k:k+2];delta=np.asarray(b)-a;height=float(np.linalg.norm(delta));axis=delta/height
    # Same shortest Z->delta rotation as native Gf.Rotation((0,0,1),delta).
    q=np.r_[np.cross((0.,0.,1.),axis),1.+axis[2]];q/=np.linalg.norm(q)
    return quat_xyzw_to_matrix(q),height


def object_obstacles(plug_poses,clip_poses,wire_poses=None,*,include_wires=True,include_guides=True):
    """Every task collider, with live body poses; no object or score writes.

    Pose dictionaries use integer port IDs and (i,k) wire IDs. Omitting cables
    requires include_wires=False explicitly, and is only a scoped rigid-object
    certificate. Boot/capsule boxes enclose the actual round geometry.
    """
    from yam_arena.urdf import quat_xyzw_to_matrix
    if include_wires and wire_poses is None:raise ValueError('Actual wire poses required for a complete object certificate')
    result=[]
    for name,center,size,_ in socket_boxes(include_guides):result.append(('switch',name,box_triangles(center,size)))
    hook_vertices,hook_faces=hook_mesh();hook_indices=[(f[0],f[n],f[n+1]) for f in hook_faces for n in range(1,len(f)-1)]
    for i in USED:
        pose=np.asarray(plug_poses[i]);rotation=quat_xyzw_to_matrix(pose[3:])
        for name,center,size in plug_boxes():result.append((f'plug_{i}',name,box_triangles(center,size)@rotation.T+pose[:3]))
        result.append((f'plug_{i}','boot_bound',box_triangles((-.0205,0,-.0005),(.014,.007,.007))@rotation.T+pose[:3]))
        clip=np.asarray(clip_poses[i]);cr=quat_xyzw_to_matrix(clip[3:])
        result.append((f'clip_{i}','tab',box_triangles((-.003,0,0),(.020,.005,.001))@cr.T+clip[:3]))
        result.append((f'clip_{i}','hook',np.asarray(hook_vertices)[np.asarray(hook_indices)]@cr.T+clip[:3]))
        if include_wires:
            for k in range(LINKS):
                wire=np.asarray(wire_poses[i,k]);wr=quat_xyzw_to_matrix(wire[3:]);local,height=wire_local_frame(i,k)
                bound=box_triangles((0,0,0),(2*CABLE_RADIUS,2*CABLE_RADIUS,height+2*CABLE_RADIUS))
                result.append((f'wire_{i}_{k}','capsule_bound',bound@(wr@local).T+wire[:3]))
    return result
