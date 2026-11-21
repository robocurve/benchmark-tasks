"""Choose the lateral placement between four measured load-bearing cords."""
import numpy as np

_axis=np.linspace(-.040,.040,33)
_grid=np.stack(np.meshgrid(_axis,_axis),axis=-1).reshape(-1,2)

def center(section,knot):
    if len(section)!=4 or any(len(crossings)!=1 for crossings in section):
        return None
    pts=np.asarray(section,dtype=float)[:,0,:]
    if not np.isfinite(pts).all():
        return None
    delta=pts[None,:,:]-_grid[:,None,:]
    radius=np.linalg.norm(delta,axis=-1)
    angles=np.sort(np.arctan2(delta[:,:,1],delta[:,:,0]),axis=1)
    largest=np.diff(np.column_stack((angles,angles[:,0]+2*np.pi)),axis=1).max(axis=1)
    offset=np.linalg.norm(_grid-knot,axis=1)
    # At 50 mm height the tapered wall has a 52.3 mm radius. Give each
    # strand room to seat against it, with the knot under the 44 mm base.
    valid=(largest<np.pi)&(radius.min(axis=1)>.047)&(radius.max(axis=1)<.078)&(offset<.040)
    if not valid.any():
        return None
    angular_cost=.040*largest
    cost=angular_cost+np.mean(abs(radius-.055),axis=1)+.22*offset+.24*np.linalg.norm(_grid,axis=1)
    return _grid[np.argmin(np.where(valid,cost,np.inf))].copy()
