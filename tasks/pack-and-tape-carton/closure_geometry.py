"""Plain lateral flap-rim grasps, CPU geometry only; no object pose writes."""
import numpy as np

# Near-hinge lateral rims avoid the unreachable high arc of the outer free edge.
# Tuple: arm, lateral edge sign, tangent sign, small face-normal approach tilt.
GRASPS={'east':('left',-1,1,0.),'west':('left',1,1,3.),
        'north':('right',-1,1,0.),'south':('right',1,1,3.)}
PAD_CONTACT_FORWARD=.004
GRASP_FROM_HINGE=.055
EDGE_INSET=.014


def rx(degrees):
    a=np.radians(degrees);c,s=np.cos(a),np.sin(a)
    return np.array(((1,0,0),(0,c,-s),(0,s,c)))


def grasp_rotation(flap_rotation,angle,edge,tangent,tilt_amplitude):
    """Finger X crosses the board normal; approach Z enters its lateral rim.

    A <=3 degree tilt clears the wrist pitch limit on the two near-side grasps.
    The normal projection stays >=cos(3deg); no tangent-closing pseudo-grasp.
    """
    r=np.asarray(flap_rotation);tilt=np.radians(tilt_amplitude*np.cos(np.radians(angle)))
    z=-edge*r[:,0]*np.cos(tilt)-r[:,2]*np.sin(tilt)
    y=tangent*r[:,1];x=np.cross(y,z)
    return np.column_stack((x,y,z))


def flap_target(wall_pos,wall_rotation,hinge,flap_size,angle,edge,tangent,tilt,
                contact_offset=None,rotation_offset=None):
    wr=np.asarray(wall_rotation);fr=wr@rx(angle)
    hinge_world=np.asarray(wall_pos)+wr@np.asarray(hinge['anchor_a'])
    fp=hinge_world-fr@np.asarray(hinge['anchor_b'])
    if contact_offset is None:
        contact_offset=np.array((edge*(flap_size[0]/2-EDGE_INSET),
                                 hinge['anchor_b'][1]-GRASP_FROM_HINGE,0.))
    r=grasp_rotation(fr,angle,edge,tangent,tilt)
    if rotation_offset is not None:r=r@rotation_offset
    contact=fp+fr@contact_offset
    return contact-r[:,2]*PAD_CONTACT_FORWARD,r,contact
