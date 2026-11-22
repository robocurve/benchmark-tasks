"""Measurements from solver body poses. Geometry is expressed in the fixture frame."""
import numpy as np
from math import sin,cos,sqrt
from fixture import COUNT,PITCH,K_LOAD,SHAPES
from scoring import Frame
CORNERS=np.array([(x,y,z) for x in (.000125,PITCH-.000125) for y in (-.018,.018) for z in (-.0015,.0015)])
def shoe_gap(angle):
    # Minimum vertical gap between the tangent shoe faces and circular roller crown.
    c,s=cos(angle),sin(angle);radius=.032
    gaps=[]
    for x,z in ((-.071,-.0005),(-.060,.0011),(-.049,-.0005)):
        centre=.060+c*x+s*z
        lo=max(-radius,centre-.0055*c);hi=min(radius,centre+.0055*c)
        if lo>hi:continue
        p=min(hi,max(lo,radius*s))
        height=.088-s*x+c*z-(p-centre)*s/c
        gaps.append(height-(.045+sqrt(max(0.,radius**2-p**2))))
    return min(gaps) if gaps else .064

_GLOVE=[(np.asarray(p),np.asarray(size)/2) for _,owner,p,size,_,kind in SHAPES if owner=="hand" and kind=="ellipsoid"]

def glove_geometry(poses):
    p,basis=poses["hand"]
    margin=float("inf")
    for local,half in _GLOVE:
        centre=p+basis@local
        radius=np.sqrt(np.sum((basis*half)**2,axis=1))
        lo=centre-radius;hi=centre+radius
        margin=min(margin,lo[0]+.417,-.021-hi[0],lo[1]+.254,.168-hi[1])
    origin,arm=poses["arm"]
    wrist=origin+arm@np.array([0,.204,0])
    return {"glove_table_margin_m":float(margin),"wrist_y":float(wrist[1])}

def observe(q,v,poses,interval,arms,right_surface,limits):
    peak,nip,rest,touch,lever,arm,hand,forearm,table=interval
    xmax=max(float(p[0]+max(CORNERS@R[0])) for p,R in (poses["fabric"+str(i)] for i in range(COUNT)))
    p,R=poses["arm"]
    wrist=float((p+R@np.array([0,.204,0]))[0])
    load=K_LOAD*max(float(q["extension"]),0.)
    return Frame(xmax,wrist,shoe_gap(q["lift"]),max(peak,load),load,nip,rest,touch,
                 max(abs(v["elbow_yaw"]),abs(v["elbow_pitch"]),abs(v["wrist_flex"])),max(abs(v["seam"+str(i)]) for i in range(COUNT)),lever,arm,hand,arms[0][0],arms[1][0],arms[0][1],arms[1][1],right_surface,forearm_n=forearm,table_n=table,**limits)


def link_poses(q):
    from fixture import BODIES
    poses={}
    for n,parent,xyz,joint,kind,axis,*_ in BODIES:
        if parent is None:
            poses[n]=(np.zeros(3),np.eye(3))
            continue
        origin,basis=poses[parent]
        a=np.asarray(axis);x,y,z=a
        if kind=="slide":
            poses[n]=(origin+basis@(np.asarray(xyz)+a*q[joint]),basis)
        else:
            angle=q[joint]
            cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            local=np.eye(3)+sin(angle)*cross+(1-cos(angle))*(cross@cross)
            poses[n]=(origin+basis@xyz,basis@local)
    return poses
