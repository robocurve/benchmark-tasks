from pathlib import Path
from types import SimpleNamespace
import sys,numpy as np,math
root=Path(__file__).resolve().parent
sys.path[:0]=[str(root/'runtime'),str(root)]
import task
corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
for sign in (-1,1):
 spec=task.HALVES[0 if sign==1 else 1]
 yaw=math.radians(3);q=np.array([0.,0.,math.sin(yaw/2),math.cos(yaw/2)])
 r=task.quat_xyzw_to_matrix(q);origin=np.array([.28,0,task.TOP+.010])
 points=np.concatenate([corners*np.asarray(part['size'])/2+part['pos'] for part in spec.parts])@r.T+origin
 state=SimpleNamespace(pos=(points.min(axis=0)+points.max(axis=0))/2,quat_xyzw=q)
 old=state.pos-r@np.array([0,sign*.0575,.0075])
 assert np.linalg.norm(old-origin)>.001,'The asymmetric rotated half must expose the old offset error'
 assert np.max(np.abs(task.root_position(state,sign)-origin))<1e-12
print('PASS rotated compound root recovery')
