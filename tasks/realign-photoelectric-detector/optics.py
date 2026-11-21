"""Geometric return/occlusion model; no task score or policy data involved."""
import math
import numpy as np
import spec

def ray_box(origin, direction, low, high, distance):
    lo,hi=0.,float(distance)
    for a,d,mn,mx in zip(origin,direction,low,high):
        if abs(d)<1.e-12:
            if a<mn or a>mx:return False
        else:
            u,v=sorted(((mn-a)/d,(mx-a)/d));lo=max(lo,u);hi=min(hi,v)
            if hi<lo:return False
    return hi>=max(lo,1.e-5)

def response(origin, rotation, reflector, blockers, occlusion=None):
    """Nine rays sampling the narrow beam. Reflector faces -X, 18 x 60 mm."""
    origin=np.asarray(origin);rotation=np.asarray(rotation);target=np.asarray(reflector)
    samples=[(0.,0.)]+[(math.cos(i*math.pi/4),math.sin(i*math.pi/4)) for i in range(8)]
    received=0;hits=[];blocked=set()
    spread=math.tan(spec.BEAM_HALF_ANGLE_RAD)
    for u,v in samples:
        direction=rotation@np.array([1.,spread*u,spread*v]);direction/=np.linalg.norm(direction)
        if direction[0]<=math.cos(math.radians(spec.MAX_REFLECTOR_INCIDENCE_DEG)):continue
        distance=(target[0]-origin[0])/direction[0]
        if distance<=0:continue
        hit=origin+distance*direction
        if abs(hit[1]-target[1])>spec.REFLECTOR_SIZE[1]/2 or abs(hit[2]-target[2])>spec.REFLECTOR_SIZE[2]/2:continue
        hits.append(hit.tolist());obstructed=False
        if occlusion is not None:
            name=occlusion(origin,direction,distance)
            if name is not None:blocked.add(name);obstructed=True
        for name,low,high in blockers:
            if ray_box(origin,direction,low,high,distance):blocked.add(name);obstructed=True
        if not obstructed:received+=1
    return {'on':received>=7,'received_rays':received,'geometric_hits':len(hits),'blocked_by':sorted(blocked),'hits':hits}
