"""Joint-limit bounded FK check for stock left YAM."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from yam_arena import ik,station

def solve(point,attitude,seeds,side="left"):
    def error(q):
        p,r=ik.tcp_pose(q,station.BASES[side])
        return np.r_[p-point,.20*(attitude*R.from_matrix(r).inv()).as_rotvec()]
    best=None
    for initial in seeds:
        q=least_squares(error,np.clip(initial,ik.LOWER+1e-6,ik.UPPER-1e-6),
                        bounds=(ik.LOWER,ik.UPPER),max_nfev=65,ftol=1e-10,xtol=1e-10,gtol=1e-10).x
        residual=error(q)
        d,a=np.linalg.norm(residual[:3]),5*np.linalg.norm(residual[3:])
        if best is None or d+.2*a < best[1]+.2*best[2]:best=q,d,a
        if d<.00015 and a<.002:break
    return best

SEEDS=[[-.5,1.7,1.1,-1.3,.3,-.5],[0,1,1,-.6,0,0],
       [-.7,1.1,1.2,1.5,.7,1.5],[-1.2,1.4,1.2,-1.4,.2,2.5],
       [-.7,1.,1.,1.2,-1.,-1.8]]
