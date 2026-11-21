"""Procedural metric thread with 60-degree flanks; no external CAD."""
import math
import numpy as np
import spec
def surface():
    n=72;m=round(spec.BOLT_LENGTH/(spec.PITCH/12));points=[];faces=[];coordinates=[]
    for j in range(m+1):
        z=spec.BOLT_LENGTH*j/m
        for i in range(n):
            a=2*math.pi*i/n;r=spec.thread_radius(a,z);points.append((r*math.cos(a),r*math.sin(a),z))
    for j in range(m):
        for i in range(n):
            faces.append([j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i])
            coordinates.append((2*math.pi*(i+.5)/n,spec.BOLT_LENGTH*(j+.5)/m))
    return points,faces,np.asarray(coordinates)
def ridge_segments():
    n=24
    for turn in range(-1,9):
        for i in range(n):
            angles=[2*math.pi*i/n,2*math.pi*(i+1)/n];points=[]
            for a in angles:
                center=spec.PITCH*(turn+a/(2*math.pi))
                for r,offset in [(spec.MINOR_RADIUS,-5*spec.PITCH/12),(spec.MAJOR_RADIUS,-spec.PITCH/16),(spec.MAJOR_RADIUS,spec.PITCH/16),(spec.MINOR_RADIUS,5*spec.PITCH/12)]:
                    points.append((r*math.cos(a),r*math.sin(a),center+offset))
            if min(p[2] for p in points)<0 or max(p[2] for p in points)>.016:continue
            yield points,[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
