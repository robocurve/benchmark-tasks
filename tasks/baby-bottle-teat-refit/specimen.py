"""Diagnostic axisymmetric teat section and rigid collar aperture, assumed millimetres."""
from itertools import permutations
import numpy as np

SECTORS = 48
BORE = 22.0
OUTER = 31.0
LIP_HALF = 1.2
BEAD = 7
FLANGE = 0
PROFILE = np.array([(28,-11.5),(25,-11.5),(22,-11.5),(20,-10.5),(20,-9),
                    (20,-7.9),(22.6,-7.0),(23.1,-5.7),(22.2,-4.3),(20.5,-3.0),
                    (18.3,0),(16,3),(13,6),(9.5,8.7),(6.4,12),(5.5,15),(5.1,18)],float)

def specimen():
    tangent = np.gradient(PROFILE, axis=0)
    normal = tangent[:,[1,0]] * [1,-1]
    normal /= np.linalg.norm(normal,axis=1,keepdims=True)
    sides = PROFILE[:,None,:] + normal[:,None,:]*np.array([-.60,.60])[None,:,None]
    angles = np.arange(SECTORS)*2*np.pi/SECTORS
    radial,z = sides.T.swapaxes(1,2) # profile, then wall side
    points = np.stack(np.broadcast_arrays(np.cos(angles)[:,None,None]*radial,
                      np.sin(angles)[:,None,None]*radial,z),axis=-1).reshape(-1,3)*.001
    grid = np.arange(len(points)).reshape(SECTORS,len(PROFILE),2)
    tets = []
    for i in range(SECTORS):
        for j in range(len(PROFILE)-1):
            for order in permutations(range(3)):
                key = [i,j,0]
                tet = [grid[tuple(key)]]
                for axis in order:
                    key[axis] += 1
                    key[0] %= SECTORS
                    tet.append(grid[tuple(key)])
                tets.append(tet)
    tets = np.array(tets,np.int32)
    det = np.linalg.det(points[tets[:,1:]]-points[tets[:,0,None]])
    assert np.abs(det).min()>1e-14
    tets[det<0,:2]=tets[det<0,:2][:,::-1]
    return points,tets,grid
