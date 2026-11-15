"""Curved rigid rim sectors: circular contact edge with a compliant joint ring.

A box-only 32-gon produces about 1 mm of geometric radial runout even when its
node circle is perfect. Curved sector colliders reduce that discretization error
without moving the rigid bodies or prescribing the deformed rim shape.
"""
import math
import numpy as np


def sector_mesh(radius=.210, radial_depth=.020, axial_width=.022, sectors=32,
                index=0, subdivisions=6):
    if radius<=radial_depth/2 or min(radial_depth,axial_width)<=0 or sectors<8 or subdivisions<2:
        raise ValueError('Invalid rim sector dimensions')
    start=index*2*math.pi/sectors;end=(index+1)*2*math.pi/sectors
    vertices=[]
    for angle in np.linspace(start,end,subdivisions+1):
        for r,z in ((radius-radial_depth/2,-axial_width/2),
                    (radius+radial_depth/2,-axial_width/2),
                    (radius+radial_depth/2, axial_width/2),
                    (radius-radial_depth/2, axial_width/2)):
            vertices.append([r*math.cos(angle),r*math.sin(angle),z])
    vertices=np.asarray(vertices)
    center=np.array([radius*(math.cos(start)+math.cos(end))/2,
                     radius*(math.sin(start)+math.sin(end))/2,0.])
    yaw=(start+end)/2+math.pi/2
    R=np.array(((math.cos(yaw),-math.sin(yaw),0.),(math.sin(yaw),math.cos(yaw),0.),(0.,0.,1.)))
    local=(vertices-center)@R
    faces=[]
    for j in range(subdivisions):
        for side in range(4):
            a=4*j+side;b=4*j+(side+1)%4;c=b+4;d=a+4
            faces.extend(((a,b,c),(a,c,d)))
    faces.extend(((0,2,1),(0,3,2)))
    last=4*subdivisions;faces.extend(((last,last+1,last+2),(last,last+2,last+3)))
    # The traversal above winds the volume inward. Reverse each triangle so
    # the renderer sees the exterior faces (the convex collision hull is unchanged).
    return local,np.asarray(faces,int)[:,::-1]


def maximum_radial_chord_error_m(radius=.220,sectors=32,subdivisions=6):
    return radius*(1-math.cos(math.pi/(sectors*subdivisions)))
