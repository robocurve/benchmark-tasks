"""Open snack-pouch geometry in metres. Rigid chips do not model fracture."""
import math
import numpy as np

PACKET_XY=(.285,.015)
PLATE_XY=(.370,-.145)
CHIP_ORIGINS=((.008,-.006,.028),(0.,0.,.018),(-.008,.006,.008))
IDENTITY=(0.,0.,0.,1.)


def box(pos,size,color=(.75,.24,.13)):
    return dict(pos=tuple(pos),size=tuple(size),color=color,contact_offset=.00005)


def packet_parts():
    # Lower gusset and lower wall sections. No invisible enclosing box.
    return [box((0,0,0),(.090,.070,.001)),
            box((0,-.035,.014),(.090,.001,.028)),
            box((0,.035,.014),(.090,.001,.028)),
            box((-.045,0,.014),(.001,.070,.028)),
            box((.045,0,.014),(.001,.070,.028)),
            # The left arm grips this heat-sealed side fin, not a custom handle.
            box((-.060,.015,.028),(.030,.001,.050))]


def chip_parts():
    # Seven convex strips approximate the curved surface of a 32 x 40 mm chip.
    parts=[]
    for i in range(7):
        x=-.016+(i+.5)*.032/7
        z=.004*(x/.016)**2
        p=box((x,0,z),(.032/7+.00015,.040,.002),(.91,.72,.34))
        parts.append(p)
    return parts


def plate_parts():
    c=(.88,.9,.92)
    return [box((0,0,.002),(.100,.100,.004),c),
            box((-.049,0,.006),(.002,.100,.012),c),
            box((.049,0,.006),(.002,.100,.012),c),
            box((0,-.049,.006),(.096,.002,.012),c),
            box((0,.049,.006),(.096,.002,.012),c)]


def rotation(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.01:
        raise ValueError('Invalid XYZW quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def corners(pos,q):
    signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    pts=np.concatenate([np.array(p['pos'])+signs*np.array(p['size'])/2 for p in chip_parts()])
    return pts@rotation(q).T+np.asarray(pos)
